# System Architecture

## Overview

This document describes the hardware and software architecture of the SSRS Open Drone ID
daemon (`odid-daemon`), a software Remote ID broadcaster developed for the Swedish Sea Rescue
Society (SSRS) fixed-wing sea rescue drone.

The system replaces a dedicated hardware Remote ID module by implementing Direct Remote ID
broadcasting in software on the companion computer (Raspberry Pi CM4 / RPi 400), using the
built-in BCM43455 wireless chipset. Regulatory basis: Transportstyrelsen (TSFS) regulations
implementing EU 2019/945 and EU 2019/947; technical format per EN 4709-002.

---

## Hardware Architecture

```
┌─────────────────┐       UART        ┌──────────────────────────┐
│   u-blox GNSS   │──────────────────▶│  mRo PixracerPro FC      │
│   (M8N/M9N)     │                   │  (Ardupilot firmware)    │
└─────────────────┘                   │                          │
                                      │  - Fuses GNSS + IMU      │
                                      │  - Outputs MAVLink v2    │
                                      └────────────┬─────────────┘
                                                   │ USB CDC-ACM
                                                   │ /dev/ttyACM0
                                                   │ (921600 baud)
                                      ┌────────────▼─────────────┐
                                      │  Raspberry Pi 400        │
                                      │  (BCM2711 + BCM43455)    │
                                      │                          │
                                      │  odid-daemon             │
                                      │  ┌────────────────────┐  │
                                      │  │ MAVLink reader     │  │
                                      │  │ ODID state machine │  │
                                      │  │ BT4 advertiser     │  │
                                      │  │ WiFi beacon        │  │
                                      │  └────────────────────┘  │
                                      └──────────┬───────────────┘
                                                 │
                              ┌──────────────────┼──────────────────┐
                              │                  │                  │
                    ┌─────────▼──────┐  ┌────────▼───────┐  ┌──────▼──────┐
                    │  BT4 Legacy    │  │  BT5 Long Range│  │ WiFi Beacon │
                    │  Advertising   │  │  (Extended Adv)│  │ (802.11 IE) │
                    │  ✅ Active     │  │  ❌ HW limit   │  │ ⚠️  hostapd │
                    └────────────────┘  └────────────────┘  └─────────────┘
```

### Hardware Components

| Component | Model | Interface | Notes |
|-----------|-------|-----------|-------|
| GNSS receiver | u-blox M8N/M9N | UART → FC | Connected to FC, not directly to RPi |
| Flight controller | mRo PixracerPro | USB CDC-ACM (`/dev/ttyACM0`) | Ardupilot firmware, MAVLink v2 |
| Companion computer | Raspberry Pi 400 | — | BCM2711 quad-core, BCM43455 WiFi/BT |
| Bluetooth | BCM43455 (built-in) | HCI (`hci0`) | BT4 Legacy supported; BT5 Extended Advertising not supported |
| WiFi | BCM43455 (built-in) | nl80211 (`wlan0`) | Direct beacon inject unsupported; hostapd fallback used |

### USB Device Identification

The PixracerPro presents as a USB CDC-ACM device (not a USB-to-serial bridge):

```
ID 1209:5740 Generic mRoPixracerPro
Driver: cdc_acm → /dev/ttyACM0
```

---

## Software Architecture

### Module Overview

```
src/
├── main.c            Entry point — wires modules, runs MAVLink consumer thread
├── mavlink_reader.c  Serial MAVLink reader — thread, queue, auto-reconnect
├── odid_state.c      ODID state machine — merges config + MAVLink, scheduler
├── bt_advertiser.c   BT4 Legacy + BT5 LR via raw HCI socket
├── wifi_beacon.c     WiFi Beacon via nl80211 / hostapd fallback
└── config.c          INI config parser (/etc/odid/odid.conf)

lib/
├── opendroneid-core-c/   ODID encode/decode (EN 4709-002 reference implementation)
├── mavlink/              MAVLink C headers v2, ardupilotmega dialect
└── inih/                 INI file parser
```

### Thread Model

```
main thread
│
├── mavlink_reader thread  (mavlink_reader.c)
│     Reads bytes from /dev/ttyACM0 via poll()
│     Parses MAVLink frames
│     Pushes decoded messages into bounded queue (cap 64)
│     Auto-reconnects on disconnect (poll POLLHUP/POLLERR, 2s retry)
│
├── mavlink_consumer thread  (main.c)
│     Pops messages from queue
│     Calls odid_state_update() for each message
│
└── scheduler (odid_state_run(), blocks main thread)
      Fires at 1 Hz → dispatch Location to all broadcast callbacks
      Fires at 0.33 Hz → dispatch one non-Location message (BT4 cycle)
```

### Data Flow

```
FC (GPS_INT / OPEN_DRONE_ID_*)
        │
        │ MAVLink v2 over USB
        ▼
mavlink_reader thread
  mavlink_parse_char()
        │
        │ odid_queue_msg_t (thread-safe queue)
        ▼
mavlink_consumer thread
  odid_state_update()
        │
        ├── GPS_INT → location.{lat, lon, alt, speed, heading}
        ├── OPEN_DRONE_ID_LOCATION → location.* (overrides GPS_INT)
        ├── OPEN_DRONE_ID_BASIC_ID → basic_id.*
        ├── OPEN_DRONE_ID_SYSTEM   → system.*
        ├── OPEN_DRONE_ID_OPERATOR_ID → operator_id.*
        ├── OPEN_DRONE_ID_SELF_ID  → self_id.*
        └── HEARTBEAT (base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                → location.Status = AIRBORNE / GROUND
        │
        ▼
odid_state scheduler (1 Hz / 0.33 Hz)
  encodeLocationMessage()
  encodeBasicIDMessage()  etc.
        │
        ├── bt_advertiser_broadcast()
        │     BT4: LE_SET_ADVERTISING_DATA (HCI raw socket)
        │     BT5: LE_SET_EXTENDED_ADVERTISING_DATA (disabled, HW limit)
        │
        └── wifi_beacon_broadcast()
              nl80211 vendor IE inject → fails on BCM43455
              fallback: write hostapd vendor_elements → SIGHUP
```

---

## Broadcast Channels

### BT4 Legacy Advertising (Active)

- **AD type:** `0x16` Service Data — 16-bit UUID
- **UUID:** `0xFFFA` (Remote ID service UUID, EN 4709-002 §5.3)
- **Payload:** 25-byte ODID encoded message
- **Interval:** 100 ms advertising interval (ADV_NONCONN_IND, all 3 channels)
- **Update rate:** Location message updated every 1 s; other messages cycled every 3 s
- **Implementation:** Raw `AF_BLUETOOTH / BTPROTO_HCI` socket, no BlueZ D-Bus involvement

BT4 advertising data layout (29 bytes, within 31-byte limit):
```
[0x1C] length = 28
[0x16] AD type: Service Data — 16-bit UUID
[0xFA] UUID LSB (0xFFFA)
[0xFF] UUID MSB
[25 bytes] ODID encoded message
```

### BT5 Long Range (Disabled — Hardware Limitation)

BCM43455 returns HCI error `0x01` (Unknown Command) for `LE_SET_EXTENDED_ADVERTISING_PARAMETERS`.
Extended Advertising is not supported. The daemon detects this on first attempt and disables
BT5 automatically. A USB Bluetooth 5 dongle supporting Extended Advertising would be required
to enable this channel.

### WiFi Beacon (Partial — hostapd fallback)

Direct beacon frame injection via nl80211 (`NL80211_CMD_SET_BEACON`) is not supported by the
BCM43455 driver (`-EOPNOTSUPP`). The fallback writes an ODID vendor IE to hostapd's
`vendor_elements` configuration and signals hostapd to reload. This requires hostapd to be
running in AP mode on `wlan0`.

---

## ODID Message Scheduling

| Message | Broadcast interval | Source priority |
|---------|--------------------|-----------------|
| Location | 1 Hz (BT4 + WiFi) | `OPEN_DRONE_ID_LOCATION` > `GLOBAL_POSITION_INT` > config default |
| BasicID | 0.33 Hz (BT4 cycle) | Config (`uas_id`, `id_type`, `ua_type`) > MAVLink override |
| System | 0.33 Hz (BT4 cycle) | Config defaults > `OPEN_DRONE_ID_SYSTEM` |
| OperatorID | 0.33 Hz (BT4 cycle) | Config (`operator_id`) > MAVLink override |
| SelfID | 0.33 Hz (BT4 cycle) | Config (`description`) > MAVLink override |

WiFi beacon (when active) sends the full message pack (all 5 message types) on every 1 Hz tick.

---

## Operational Status Detection

The daemon subscribes to MAVLink `HEARTBEAT` messages from the autopilot (`compid == 1`).
When `base_mode & MAV_MODE_FLAG_SAFETY_ARMED` changes:

- Armed → `ODID_STATUS_AIRBORNE` (status = 2)
- Disarmed → `ODID_STATUS_GROUND` (status = 1)

This applies only when the FC is not sending `OPEN_DRONE_ID_LOCATION` natively (i.e., when
using `GLOBAL_POSITION_INT` as the GPS source). If the FC sends `OPEN_DRONE_ID_LOCATION`,
it controls the status field directly.

---

## Security and Permissions

The daemon requires two Linux capabilities:

| Capability | Required for |
|------------|-------------|
| `CAP_NET_RAW` | Raw HCI socket for BT4/BT5 advertising |
| `CAP_NET_ADMIN` | nl80211 netlink for WiFi beacon |

The systemd service unit restricts the process to exactly these two capabilities via
`CapabilityBoundingSet` and `AmbientCapabilities`. Runtime verification:

```
CapEff: 0x3000  =  CAP_NET_ADMIN (bit 12) | CAP_NET_RAW (bit 13)
```

Additional hardening in the service unit: `NoNewPrivileges=yes`, `PrivateTmp=yes`,
`ProtectSystem=strict`.
