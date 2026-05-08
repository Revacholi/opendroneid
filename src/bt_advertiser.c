/*
 * bt_advertiser.c
 *
 * Bluetooth Low Energy Open Drone ID broadcaster.
 *
 * BT4 Legacy Advertising  — single ODID message (25 bytes + 4 byte header)
 *                           via LE_SET_ADVERTISING_DATA HCI command.
 *
 * BT5 Long Range          — ODID message pack (up to N×25 bytes)
 *                           via LE_SET_EXTENDED_ADVERTISING_DATA HCI command.
 *                           PHY: LE Coded (S=8, 500 kbps long range).
 *
 * Uses raw AF_BLUETOOTH / BTPROTO_HCI sockets — no D-Bus/BlueZ daemon
 * involvement in the advertising path.  Requires CAP_NET_RAW.
 *
 * ODID Bluetooth Service UUID: 0xFFFA
 * ODID AD payload format (inside Manufacturer Specific AD type 0xFF):
 *   [Company ID lo] [Company ID hi] [SvcUUID lo] [SvcUUID hi] [ODID msg...]
 *   Company ID = 0xFFFF (reserved for Direct Remote ID per EN 4709-002)
 *
 * References:
 *   EN 4709-002 §5.3  (Bluetooth broadcast)
 *   EU 2019/945 Annex IX (Remote ID module requirements)
 *   Bluetooth Core Spec 5.3 — Vol 4, Part E, §7.8.5/7.8.53/7.8.57
 *   opendroneid/transmitter-linux (referenced for HCI command structures)
 */

#include "bt_advertiser.h"
#include "utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

/* BlueZ / Linux Bluetooth headers */
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>


/* Assigned number for Direct Remote ID (EN 4709-002 / Bluetooth SIG UUID 0xFFFA) */
#define ODID_AD_SERVICE_UUID    0xFFFA
/* Company ID used in Manufacturer Specific data for ODID */
#define ODID_COMPANY_ID         0xFFFF

/* AD type codes */
#define AD_TYPE_FLAGS           0x01
#define AD_TYPE_COMPLETE_UUID16 0x03
#define AD_TYPE_MFR_SPECIFIC    0xFF

/* Advertising flags: LE General Discoverable, BR/EDR Not Supported */
#define AD_FLAGS_VALUE          0x06


static int hci_send_cmd_wait(int dd, uint16_t ogf, uint16_t ocf,
                             const void *param, uint8_t plen) {
    uint8_t buf[HCI_MAX_DEV];
    struct hci_request rq;
    uint8_t status;

    memset(&rq, 0, sizeof(rq));
    rq.ogf    = ogf;
    rq.ocf    = ocf;
    rq.cparam = (void *)param;
    rq.clen   = plen;
    rq.rparam = &status;
    rq.rlen   = sizeof(status);

    (void)buf;

    if (hci_send_req(dd, &rq, 2000) < 0) {
        LOG_WARN("bt: hci_send_req ogf=0x%02x ocf=0x%04x: %s",
                 ogf, ocf, strerror(errno));
        return -1;
    }
    if (status) {
        LOG_WARN("bt: HCI error 0x%02x for ogf=0x%02x ocf=0x%04x",
                 status, ogf, ocf);
        return -1;
    }
    return 0;
}


static struct {
    int  dd;           /* HCI device descriptor */
    int  dev_id;
    bool bt4_enabled;
    bool bt5_enabled;
    bool bt4_advertising; /* true after first successful BT4 enable */
    bool bt5_supported;   /* set false after first BT5 "unknown command" failure */
    bool initialized;
} s;


/*
 * Build a BT4 advertising payload per EN 4709-002 §5.3.
 *
 * Drone Scanner and all ODID receivers look for:
 *   AD type 0x16 (Service Data - 16-bit UUID)
 *   UUID: 0xFFFA (little-endian: 0xFA 0xFF)
 *   Data: 25-byte ODID encoded message
 *
 * Layout (31 bytes total, exactly fills the BT4 limit):
 *   [0x1E] length = 30
 *   [0x16] AD type: Service Data
 *   [0xFA] UUID LSB
 *   [0xFF] UUID MSB
 *   [0x0D] ODID Application Code
 *   [counter] rolling message counter (receiver-android parseData offset=6 expects this)
 *   [25 bytes] ODID encoded message
 */
static uint8_t s_bt4_counter = 0;

static void bt4_build_adv_data(const odid_encoded_t *single,
                               uint8_t *adv, uint8_t *adv_len) {
    int idx = 0;
    /* Service Data AD element: length = 1(type) + 2(uuid) + 1(appcode) + 1(counter) + 25(odid) = 30 */
    adv[idx++] = 30;                            /* length (excludes this byte) */
    adv[idx++] = 0x16;                          /* AD type: Service Data 16-bit UUID */
    adv[idx++] = ODID_AD_SERVICE_UUID & 0xFF;   /* UUID LSB: 0xFA */
    adv[idx++] = (ODID_AD_SERVICE_UUID >> 8) & 0xFF; /* UUID MSB: 0xFF */
    adv[idx++] = 0x0D;                          /* ODID App Code (ASTM F3411) */
    adv[idx++] = s_bt4_counter++;               /* rolling counter — parseData(data,6) starts after this */
    memcpy(&adv[idx], single->data, ODID_MSG_SIZE);
    idx += ODID_MSG_SIZE;
    *adv_len = (uint8_t)idx;  /* = 31 bytes */
}

static int bt4_start(void) {
    /* Full startup sequence (used on first call and after any failure).
     * LE_SET_ADVERTISING_PARAMETERS requires advertising to be disabled. */

    /* Disable first — clears any state from BlueZ or a previous daemon run.
     * Ignore the return: 0x0c (Command Disallowed) is normal when advertising
     * was already off. The important check is that set_params succeeds below. */
    le_set_advertise_enable_cp dis = { .enable = 0 };
    uint8_t _dis_status;
    struct hci_request _dis_rq = {
        .ogf = OGF_LE_CTL, .ocf = OCF_LE_SET_ADVERTISE_ENABLE,
        .cparam = &dis, .clen = LE_SET_ADVERTISE_ENABLE_CP_SIZE,
        .rparam = &_dis_status, .rlen = sizeof(_dis_status)
    };
    hci_send_req(s.dd, &_dis_rq, 2000);  /* ignore return and status */

    /* Set advertising parameters.
     * ADV_NONCONN_IND (0x03): non-connectable, non-scannable — EN 4709-002 §6.2
     * compliant type. No scan response needed or sent. iOS CoreBluetooth matches
     * the Service Data UUID (0xFFFA in AD type 0x16) for withServices: filter. */
    le_set_advertising_parameters_cp adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    /* 100 ms = 160 × 0.625 ms */
    adv_params.min_interval = htobs(160);
    adv_params.max_interval = htobs(160);
    adv_params.advtype      = 0x03;  /* ADV_NONCONN_IND (non-connectable, non-scannable) */
    adv_params.chan_map      = 0x07; /* all 3 channels */

    if (hci_send_cmd_wait(s.dd, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                          &adv_params, LE_SET_ADVERTISING_PARAMETERS_CP_SIZE) < 0)
        return -1;

    LOG_INFO("bt: params set — ADV_NONCONN_IND 100ms all-channels");

    /* Enable advertising (data will be set separately on the first broadcast) */
    le_set_advertise_enable_cp enable = { .enable = 1 };
    if (hci_send_cmd_wait(s.dd, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                          &enable, LE_SET_ADVERTISE_ENABLE_CP_SIZE) < 0)
        return -1;

    s.bt4_advertising = true;
    return 0;
}

static int bt4_advertise(const odid_encoded_t *single) {
    /* Start advertising if not already running */
    if (!s.bt4_advertising) {
        if (bt4_start() < 0)
            return -1;
    }

    /* Update advertising data in-place — LE_SET_ADVERTISING_DATA can be sent
     * at any time, even while advertising is enabled (BT Core Spec §7.8.7).
     * This keeps advertising continuous with zero RF gaps. */
    le_set_advertising_data_cp adv_data;
    memset(&adv_data, 0, sizeof(adv_data));
    bt4_build_adv_data(single, adv_data.data, &adv_data.length);

    if (hci_send_cmd_wait(s.dd, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA,
                          &adv_data, LE_SET_ADVERTISING_DATA_CP_SIZE) < 0) {
        /* Data update failed — mark as not advertising so we restart next call */
        s.bt4_advertising = false;
        return -1;
    }

    return 0;
}


/*
 * HCI LE Set Extended Advertising Parameters (OGF=0x08, OCF=0x0036)
 * Bluetooth Core 5.3, Vol 4 Part E §7.8.53
 */
#define OCF_LE_SET_EXT_ADV_PARAMS   0x0036
#define OCF_LE_SET_EXT_ADV_DATA     0x0037
#define OCF_LE_SET_EXT_ADV_ENABLE   0x0039

typedef struct __attribute__((packed)) {
    uint8_t  handle;
    uint16_t event_props;      /* 0x0000 = non-connectable, non-scannable */
    uint8_t  min_interval[3]; /* 3-byte little-endian: 100 ms = 0x000050 * 1.25 */
    uint8_t  max_interval[3];
    uint8_t  channel_map;      /* 0x07 = all */
    uint8_t  own_addr_type;    /* 0x00 = public */
    uint8_t  peer_addr_type;
    uint8_t  peer_addr[6];
    uint8_t  filter_policy;    /* 0x00 */
    int8_t   tx_power;         /* 0x7F = host has no preference */
    uint8_t  primary_phy;      /* 0x03 = LE Coded */
    uint8_t  secondary_max_skip;
    uint8_t  secondary_phy;    /* 0x03 = LE Coded */
    uint8_t  adv_sid;
    uint8_t  scan_req_notify;
} le_ext_adv_params_t;

typedef struct __attribute__((packed)) {
    uint8_t  handle;
    uint8_t  operation;        /* 0x03 = complete data */
    uint8_t  frag_preference;  /* 0x01 = no fragmentation (RPi fix) */
    uint8_t  data_len;
    uint8_t  data[255];
} le_ext_adv_data_t;

typedef struct __attribute__((packed)) {
    uint8_t enable;
    uint8_t num_sets;
    uint8_t handle;
    uint16_t duration;         /* 0 = indefinite */
    uint8_t  max_events;       /* 0 = no limit */
} le_ext_adv_enable_t;

static void bt5_build_pack_data(const odid_pack_t *pack,
                                uint8_t *buf, uint8_t *len) {
    /* Manufacturer Specific AD element carrying the message pack */
    int n = pack->count * ODID_MSG_SIZE;
    int idx = 0;
    buf[idx++] = (uint8_t)(3 + n);  /* len: type(1) + cid(2) + uuid(2) + data(n) - but len field excludes itself */
    /* Actually: length = 1(type) + 2(cid) + 2(uuid) + n(data) */
    buf[0] = (uint8_t)(1 + 2 + 2 + n);
    buf[idx++] = AD_TYPE_MFR_SPECIFIC;
    buf[idx++] = ODID_COMPANY_ID & 0xFF;
    buf[idx++] = (ODID_COMPANY_ID >> 8) & 0xFF;
    buf[idx++] = ODID_AD_SERVICE_UUID & 0xFF;
    buf[idx++] = (ODID_AD_SERVICE_UUID >> 8) & 0xFF;
    for (int i = 0; i < pack->count; i++) {
        memcpy(&buf[idx], pack->msgs[i].data, ODID_MSG_SIZE);
        idx += ODID_MSG_SIZE;
    }
    *len = (uint8_t)idx;
}

static int bt5_advertise(const odid_pack_t *pack) {
    if (!s.bt5_supported) return -1;

    /* 1. Set extended advertising parameters */
    le_ext_adv_params_t params;
    memset(&params, 0, sizeof(params));
    params.handle       = 0;
    params.event_props  = htobs(0x0000);  /* non-conn, non-scan */
    /* 100 ms interval = 80 * 1.25 ms = 0x000050 */
    params.min_interval[0] = 0x50; params.min_interval[1] = 0x00; params.min_interval[2] = 0x00;
    params.max_interval[0] = 0x50; params.max_interval[1] = 0x00; params.max_interval[2] = 0x00;
    params.channel_map  = 0x07;
    params.own_addr_type = 0x00;
    params.tx_power     = 0x7F;   /* no preference */
    params.primary_phy  = 0x03;   /* LE Coded */
    params.secondary_phy = 0x03;  /* LE Coded */
    params.adv_sid      = 0;

    if (hci_send_cmd_wait(s.dd, OGF_LE_CTL, OCF_LE_SET_EXT_ADV_PARAMS,
                          &params, sizeof(params)) < 0)
        return -1;

    /* 2. Set extended advertising data */
    le_ext_adv_data_t adv_data;
    memset(&adv_data, 0, sizeof(adv_data));
    adv_data.handle          = 0;
    adv_data.operation       = 0x03;  /* complete data */
    adv_data.frag_preference = 0x01;  /* no fragmentation — RPi BCM43455 fix */
    bt5_build_pack_data(pack, adv_data.data, &adv_data.data_len);

    /* Command parameter length = 4 fixed bytes + data_len */
    uint8_t cp_len = 4 + adv_data.data_len;
    if (hci_send_cmd_wait(s.dd, OGF_LE_CTL, OCF_LE_SET_EXT_ADV_DATA,
                          &adv_data, cp_len) < 0)
        return -1;

    /* 3. Enable extended advertising */
    le_ext_adv_enable_t enable;
    memset(&enable, 0, sizeof(enable));
    enable.enable    = 1;
    enable.num_sets  = 1;
    enable.handle    = 0;
    enable.duration  = 0;
    enable.max_events = 0;

    if (hci_send_cmd_wait(s.dd, OGF_LE_CTL, OCF_LE_SET_EXT_ADV_ENABLE,
                          &enable, sizeof(enable)) < 0)
        return -1;

    return 0;
}


int bt_advertiser_init(const char *adapter, bool bt4, bool bt5) {
    memset(&s, 0, sizeof(s));
    s.bt4_enabled = bt4;
    s.bt5_enabled = bt5;

    /* Parse adapter name "hciN" */
    int idx = 0;
    if (sscanf(adapter, "hci%d", &idx) != 1) {
        LOG_ERROR("bt: invalid adapter '%s', expected hciN", adapter);
        return -1;
    }

    /* Bring adapter up before hci_devid — adapter may be DOWN at boot when
     * bluetooth.service is disabled or when the daemon is run without systemd.
     * EALREADY is fine (already up). Requires CAP_NET_ADMIN. */
    int ctl = socket(AF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI);
    if (ctl < 0) {
        LOG_WARN("bt: control socket: %s", strerror(errno));
    } else {
        if (ioctl(ctl, HCIDEVUP, idx) < 0 && errno != EALREADY)
            LOG_WARN("bt: could not bring %s up: %s", adapter, strerror(errno));
        close(ctl);
    }

    s.dev_id = hci_devid(adapter);
    if (s.dev_id < 0) {
        LOG_ERROR("bt: adapter %s not found: %s", adapter, strerror(errno));
        return -1;
    }

    s.dd = hci_open_dev(s.dev_id);
    if (s.dd < 0) {
        LOG_ERROR("bt: hci_open_dev(%d): %s", s.dev_id, strerror(errno));
        return -1;
    }

    s.bt5_supported = true;  /* assume supported until first failure */
    s.initialized = true;
    LOG_INFO("bt: initialised adapter %s (bt4=%d bt5=%d)", adapter, bt4, bt5);
    return 0;
}

void bt_advertiser_broadcast(const odid_encoded_t *single,
                             const odid_pack_t    *pack,
                             bool                  location_only,
                             void                 *user) {
    (void)user;
    if (!s.initialized) return;

    if (s.bt4_enabled) {
        static const char *type_names[] = {"BasicID","Location","System","OperatorID","SelfID"};
        const odid_encoded_t *msg;

        if (location_only) {
            /* 1 Hz tick: always broadcast Location per EN 4709-002 §5.3 (EU 2019/945) */
            msg = single;
            if (bt4_advertise(msg) < 0)
                LOG_WARN("bt: BT4 advertise failed");
            else
                LOG_DEBUG("bt: BT4 advertised Location");
        } else {
            /* 0.33 Hz tick: cycle one non-Location message (BasicID→System→OperatorID→SelfID) */
            static const int slow_cycle[] = { 0, 2, 3, 4 };  /* indices: BasicID, System, OperatorID, SelfID */
            static int slow_rot = 0;
            int idx = slow_cycle[slow_rot % 4];
            slow_rot++;

            odid_encoded_t tmp;
            if (idx < pack->count)
                memcpy(tmp.data, pack->msgs[idx].data, ODID_MSG_SIZE);
            else
                memcpy(tmp.data, single->data, ODID_MSG_SIZE);
            msg = &tmp;

            if (bt4_advertise(msg) < 0)
                LOG_WARN("bt: BT4 advertise failed");
            else
                LOG_DEBUG("bt: BT4 advertised %s", idx < 5 ? type_names[idx] : "?");
        }
    }

    if (s.bt5_enabled && s.bt5_supported) {
        if (bt5_advertise(pack) < 0) {
            /* HCI error 0x01 = Unknown Command: chip doesn't support extended adv */
            LOG_WARN("bt: BT5 LR not supported by this controller — disabling");
            s.bt5_supported = false;
        } else {
            LOG_DEBUG("bt: BT5 LR advertised %d messages", pack->count);
        }
    }
}

void bt_advertiser_stop(void) {
    if (!s.initialized) return;

    /* Disable legacy advertising */
    if (s.bt4_enabled) {
        le_set_advertise_enable_cp disable = { .enable = 0 };
        hci_send_cmd_wait(s.dd, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                          &disable, LE_SET_ADVERTISE_ENABLE_CP_SIZE);
    }

    /* Disable extended advertising */
    if (s.bt5_enabled) {
        le_ext_adv_enable_t disable;
        memset(&disable, 0, sizeof(disable));
        disable.enable   = 0;
        disable.num_sets = 0;
        hci_send_cmd_wait(s.dd, OGF_LE_CTL, OCF_LE_SET_EXT_ADV_ENABLE,
                          &disable, sizeof(disable));
    }

    hci_close_dev(s.dd);
    s.initialized = false;
    LOG_INFO("bt: stopped");
}
