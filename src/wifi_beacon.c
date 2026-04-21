/*
 * wifi_beacon.c
 *
 * WiFi Beacon Open Drone ID broadcaster.
 *
 * Injects an ODID message pack as a vendor-specific Information Element (IE)
 * into 802.11 beacon frames using the nl80211 Linux kernel netlink interface.
 *
 * ODID WiFi IE format (EN 4709-002 §5.4.2):
 *   Element ID:  0xDD (Vendor Specific)
 *   Length:      variable
 *   OUI:         0xFA 0x0B 0xBC  (Open Drone ID OUI)
 *   OUI Type:    0x0D
 *   Payload:     ODID message pack bytes
 *
 * The interface must already be in AP mode (managed by hostapd) or monitor
 * mode.  If nl80211 vendor IE injection fails (driver limitation), the code
 * falls back to writing a hostapd vendor_elements config snippet and sending
 * SIGHUP to hostapd to reload it.
 *
 * Requires: libnl-3-dev, libnl-genl-3-dev
 *           CAP_NET_ADMIN
 */

#include "wifi_beacon.h"
#include "utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

/* libnl */
#include <netlink/netlink.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <linux/nl80211.h>


static const uint8_t ODID_WIFI_OUI[3]  = { 0xFA, 0x0B, 0xBC };
static const uint8_t ODID_WIFI_OUI_TYPE = 0x0D;


static struct {
    char            iface[16];
    int             ifindex;
    struct nl_sock *nl;
    int             nl80211_id;
    bool            initialized;
    bool            nl_ok;         /* nl80211 path works */
    bool            hostapd_cli_ok; /* true if using hostapd_cli fallback */
    uint8_t         msg_counter;   /* rolling beacon counter per EN 4709-002 */
} s;


/*
 * Builds the raw bytes of one 802.11 vendor IE element for the ODID pack.
 *
 * IE layout per EN 4709-002 §5.4.2 / ASTM F3411-22a:
 *   [0xDD][length][OUI:FA 0B BC][type:0x0D]
 *   [message_counter]                     ← ODID_service_info.message_counter
 *   [MsgPackHeader][SingleMsgSize][Count]  ← ODID_MessagePack_encoded header
 *   [msg0..msgN-1]                         ← 25 bytes each
 *
 * buf must be at least (2 + 3+1+1+3 + pack->count * ODID_MSG_SIZE) bytes.
 * Returns total length written.
 */
static int build_vendor_ie(const odid_pack_t *pack, uint8_t *buf) {
    int n = pack->count;
    /* payload = OUI(3) + type(1) + counter(1) + pack_hdr(3) + msgs */
    int payload_len = 3 + 1 + 1 + 3 + n * ODID_MSG_SIZE;
    int idx = 0;
    buf[idx++] = 0xDD;                    /* Element ID: Vendor Specific */
    buf[idx++] = (uint8_t)payload_len;    /* Length */
    memcpy(&buf[idx], ODID_WIFI_OUI, 3);  idx += 3;
    buf[idx++] = ODID_WIFI_OUI_TYPE;      /* 0x0D */
    /* ODID_service_info */
    buf[idx++] = s.msg_counter++;         /* message_counter (rolling) */
    /* ODID_MessagePack_encoded header */
    buf[idx++] = (0xF << 4) | 0x02;      /* MsgPackHeader: type=PACKED(0xF), version=2 */
    buf[idx++] = ODID_MSG_SIZE;           /* SingleMessageSize = 25 */
    buf[idx++] = (uint8_t)n;             /* MsgPackSize */
    for (int i = 0; i < n; i++) {
        memcpy(&buf[idx], pack->msgs[i].data, ODID_MSG_SIZE);
        idx += ODID_MSG_SIZE;
    }
    return idx;
}


static int nl_init(void) {
    s.nl = nl_socket_alloc();
    if (!s.nl) {
        LOG_ERROR("wifi: nl_socket_alloc failed");
        return -1;
    }
    if (genl_connect(s.nl) < 0) {
        LOG_ERROR("wifi: genl_connect failed: %s", nl_geterror(errno));
        nl_socket_free(s.nl);
        s.nl = NULL;
        return -1;
    }
    s.nl80211_id = genl_ctrl_resolve(s.nl, "nl80211");
    if (s.nl80211_id < 0) {
        LOG_ERROR("wifi: nl80211 not found");
        nl_socket_free(s.nl);
        s.nl = NULL;
        return -1;
    }
    return 0;
}

static int nl_set_vendor_ie(const uint8_t *ie, int ie_len) {
    struct nl_msg *msg = nlmsg_alloc();
    if (!msg) return -1;

    genlmsg_put(msg, NL_AUTO_PORT, NL_AUTO_SEQ,
                s.nl80211_id, 0, 0, NL80211_CMD_SET_BEACON, 0);
    nla_put_u32(msg, NL80211_ATTR_IFINDEX, (uint32_t)s.ifindex);
    /* NL80211_ATTR_IE carries vendor/extra IEs for beacon frames */
    nla_put(msg, NL80211_ATTR_IE, ie_len, ie);

    int ret = nl_send_auto(s.nl, msg);
    nlmsg_free(msg);
    if (ret < 0) {
        LOG_WARN("wifi: nl_send_auto: %s", nl_geterror(ret));
        return -1;
    }
    /* Wait for ACK */
    ret = nl_wait_for_ack(s.nl);
    if (ret < 0 && ret != -NLE_OBJ_NOTFOUND) {
        LOG_WARN("wifi: nl80211 SET_BEACON ack: %s", nl_geterror(ret));
        return -1;
    }
    return 0;
}


static bool hostapd_available(const char *iface) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd),
             "hostapd_cli -i %s ping > /dev/null 2>&1", iface);
    return system(cmd) == 0;
}

static int hostapd_set_vendor_ie(const uint8_t *ie, int ie_len) {
    /* Build hex string of the IE bytes */
    char hex[ie_len * 2 + 1];
    for (int i = 0; i < ie_len; i++)
        sprintf(&hex[i * 2], "%02x", ie[i]);
    hex[ie_len * 2] = '\0';

    char cmd[64 + ie_len * 2];
    snprintf(cmd, sizeof(cmd),
             "hostapd_cli -i %s set vendor_elements %s > /dev/null 2>&1",
             s.iface, hex);
    if (system(cmd) != 0) {
        LOG_WARN("wifi: hostapd_cli set vendor_elements failed");
        return -1;
    }
    /* hostapd 2.10+: apply vendor_elements to live beacon frames */
    char update_cmd[48];
    snprintf(update_cmd, sizeof(update_cmd),
             "hostapd_cli -i %s update_beacon > /dev/null 2>&1", s.iface);
    system(update_cmd);
    return 0;
}


int wifi_beacon_init(const char *iface) {
    memset(&s, 0, sizeof(s));
    strncpy(s.iface, iface, sizeof(s.iface) - 1);

    s.ifindex = (int)if_nametoindex(iface);
    if (s.ifindex == 0) {
        LOG_ERROR("wifi: interface '%s' not found", iface);
        return -1;
    }

    /* Try nl80211 */
    if (nl_init() == 0) {
        s.nl_ok = true;
        LOG_INFO("wifi: nl80211 ready on %s (ifindex=%d)", iface, s.ifindex);
    } else {
        /* Fall back to hostapd_cli */
        if (hostapd_available(iface)) {
            LOG_WARN("wifi: nl80211 unavailable, using hostapd_cli fallback on %s", iface);
        } else {
            LOG_WARN("wifi: nl80211 unavailable and hostapd not running on %s; "
                     "WiFi beacon disabled", iface);
            return -1;
        }
    }

    s.initialized = true;
    return 0;
}

void wifi_beacon_broadcast(const odid_encoded_t *single,
                           const odid_pack_t    *pack,
                           bool                  location_only,
                           void                 *user) {
    (void)single;
    (void)location_only;
    (void)user;
    if (!s.initialized) return;

    uint8_t ie_buf[2 + 3 + 1 + 1 + 3 + ODID_PACK_MAX_MSGS * ODID_MSG_SIZE];
    int ie_len = build_vendor_ie(pack, ie_buf);

    if (s.nl_ok) {
        if (nl_set_vendor_ie(ie_buf, ie_len) < 0) {
            /* nl80211 failed — switch to hostapd */
            LOG_WARN("wifi: nl80211 inject failed, switching to hostapd fallback");
            s.nl_ok = false;
        } else {
            LOG_DEBUG("wifi: beacon IE updated (%d bytes)", ie_len);
            return;
        }
    }

    /* hostapd fallback */
    if (hostapd_set_vendor_ie(ie_buf, ie_len) < 0)
        LOG_WARN("wifi: hostapd vendor IE write failed");
    else
        LOG_DEBUG("wifi: hostapd vendor IE updated (%d bytes)", ie_len);
}

void wifi_beacon_stop(void) {
    if (!s.initialized) return;
    if (s.nl) {
        nl_socket_free(s.nl);
        s.nl = NULL;
    }
    s.initialized = false;
    LOG_INFO("wifi: stopped");
}
