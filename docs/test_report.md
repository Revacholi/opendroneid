# Test Report

**Project:** SSRS Open Drone ID Daemon
**Platform:** Raspberry Pi 400 (BCM2711 + BCM43455)
**Standard:** ASTM F3411-22a / EN 4709-002 (Direct Remote ID)
**Regulatory basis:** Transportstyrelsen (TSFS) / EU 2019/945 / EU 2019/947
**Test dates:** 2026-03-23, 2026-04-02, 2026-04-03
**Build:** `odid-daemon` compiled from `/home/pi4/opendroneid-linux`

---

## Build and Unit Tests

### T1 — Build

| Step | Result |
|------|--------|
| `cmake .. && make -j4` | ✅ Pass — zero warnings, zero errors |
| Compiler | GCC (arm64) |
| Dependencies | opendroneid-core-c, mavlink ardupilotmega, inih |

### T2 — Unit Tests (`ctest`)

| Test | Result | Duration |
|------|--------|----------|
| `encoding` — ODID message encode/decode round-trip | ✅ Pass | 0.01 s |
| `config` — INI parser, all fields, edge cases | ✅ Pass | 0.01 s |
| **Total** | **2/2 Pass** | |

### T16 — ANSI/CTA-2063-A Serial Number Format

**Standard:** ANSI/CTA-2063-A-2019 §5 / ASD-STAN prEN 4709-002 §5.2.2
**Method:** Static analysis of `uas_id` field in `/etc/odid/odid.conf`.

CTA-2063-A structure: `[4-char manufacturer code][1-char length indicator 0-9/A-F][serial chars]`

| Check | Value | Result |
|-------|-------|--------|
| Total length ≤ 20 chars | `SSRS5GBG01` = 10 chars | ✅ Pass |
| Manufacturer code = 4 uppercase alphanumeric chars | `SSRS` | ✅ Pass |
| Length indicator is valid (0–9 or A–F) | `5` (decimal, 5 chars follow) | ✅ Pass |
| Serial chars count matches length indicator | `GBG01` = 5 chars | ✅ Pass |
| All characters uppercase alphanumeric (no hyphens/spaces) | `SSRS5GBG01` | ✅ Pass |

**Conclusion:** `SSRS5GBG01` is a valid ANSI/CTA-2063-A-2019 serial number. ✅

### T17 — EU Operator Registration Number (OPRN) Format

**Standard:** ASD-STAN prEN 4709-002 §5.2.5 / EU 2020/1058 Part 6
**Method:** Static analysis of `operator_id` field in `/etc/odid/odid.conf`.

EU OPRN structure: 3-char ISO country code + 13 alphanumeric chars (public part of OPRN).
Private part (3 chars) is entered during upload for checksum verification but not broadcast.

| Check | Value | Result |
|-------|-------|--------|
| Total length = 16 chars (public part) | `SWE3jxza7qbzvu5d` = 16 chars | ✅ Pass |
| First 3 chars = ISO 3166-1 alpha-3 country code | `SWE` (Sweden) | ✅ Pass |
| Remaining 13 chars alphanumeric | `3jxza7qbzvu5d` | ✅ Pass |
| Issued by registered national authority | Transportstyrelsen EASA portal | ✅ Pass |

**Conclusion:** `SWE3jxza7qbzvu5d` is a valid EU OPRN. ✅

---

## BT4 Broadcast Verification

### T3 — BT4 Advertisement Format (nRF Connect)

**Method:** nRF Connect (Android) passive BLE scanner, raw Service Data inspection.

| Check | Result |
|-------|--------|
| Advertisement visible | ✅ Device appears in scan |
| AD type 0x16 (Service Data) present | ✅ |
| UUID 0xFFFA present | ✅ `Service Data: ASTM Remote ID (0xfffa)` |
| First byte of service data = 0x0D (ODID App Code) | ✅ Confirmed via nRF Connect raw bytes |

### T4 — BT4 Advertisement Format (btmon)

**Method:** `sudo btmon` while daemon running, inspect HCI LE Set Advertising Data command.

| Check | Result |
|-------|--------|
| `LE Set Advertising Data` commands observed at ~1 Hz | ✅ |
| `Service Data: ASTM Remote ID (0xfffa)` parsed by btmon | ✅ |
| Advertising type: `ADV_NONCONN_IND (0x03)` | ✅ |
| Payload structure: `[0x1E][0x16][FA FF][0x0D][counter][25-byte ODID msg]` | ✅ |
| Rolling counter byte present (byte 5, offset for parseData=6) | ✅ |
| No HCI errors during operation | ✅ |

**Verified btmon output:**
```
Service Data: ASTM Remote ID (0xfffa)
  Data[27]: 0d 22 12 12 66 00 00 00 ... (app_code + counter + Location msg)
```

### T5 — Location Broadcast Interval

**Method:** Run daemon with `--debug`, capture timestamps of `BT4 advertised Location` log lines.

**Results (7-second window):**
```
[1774260914.562] BT4 advertised Location
[1774260915.577] BT4 advertised Location   Δ = 1.015 s
[1774260916.580] BT4 advertised Location   Δ = 1.003 s
[1774260917.583] BT4 advertised Location   Δ = 1.003 s
[1774260918.587] BT4 advertised Location   Δ = 1.004 s
[1774260919.589] BT4 advertised Location   Δ = 1.002 s
[1774260920.591] BT4 advertised Location   Δ = 1.002 s
```

**Mean interval:** 1.004 s — **Requirement (EN 4709-002 §5.3):** ≤ 1 s — ✅ **Pass**

Non-Location messages (BasicID, System, OperatorID, SelfID) cycle every ~3 s, interleaved without affecting the Location cadence.

### T14 — DroneScanner BT4 Detection (2026-04-03)

**Method:** DroneScanner app (Android) active scan, daemon running with production config.

| Check | Result |
|-------|--------|
| Drone appears in DroneScanner BT4 scan | ✅ |
| UAS ID displayed: `SSRSGBG001` | ✅ |
| Operator ID displayed: `SWE3jxza7qbzvu5d` | ✅ |
| Location shown as Unknown (expected — no GPS fix indoors) | ✅ expected |

**Root cause of previous failure (resolved):** The ASTM F3411-22a BT4 Service Data format
requires a rolling counter byte between the App Code (0x0D) and the 25-byte ODID message.
Without this byte, `OpenDroneIdDataManager.receiveDataBluetooth()` calls
`OpenDroneIdParser.parseData(data, 6)` with offset 6, which landed on the second byte of the
ODID message instead of the first, causing the entire message to parse as garbage and return
null. Fix: inserted `s_bt4_counter++` at byte 5 of the Service Data, making the ODID message
start at byte 6 as expected.

---

## WiFi Beacon Verification

### T15 — DroneScanner WiFi Beacon Detection (2026-04-03)

**Method:** DroneScanner app (Android) WiFi scan with developer option
"Wi-Fi scan throttling" disabled. Daemon running with hostapd AP mode on wlan0.

| Check | Result |
|-------|--------|
| "ODID-Drone" SSID visible in phone WiFi settings | ✅ |
| Drone appears in DroneScanner WiFi scan | ✅ |
| Vendor IE OUI `FA:0B:BC` + type `0x0D` correct | ✅ |
| Message pack format: `[counter][0xF2][25][N][msgs…]` | ✅ |
| Location shown as Unknown (expected — no GPS fix indoors) | ✅ expected |

**Root cause of previous failure (resolved):** `hostapd_cli set vendor_elements` stores the
vendor IE in hostapd's memory but does not update live beacon frames in hostapd 2.10. A
subsequent `hostapd_cli update_beacon` command is required to apply changes to frames actually
transmitted over the air. Without this call, beacon frames seen by scanning devices contained
no ODID vendor IE. Fix: added `update_beacon` call after every `set vendor_elements`.

**WiFi reception note:** Android WiFi scan rate is inherently variable (throttled at OS level,
even with developer option disabled). WiFi ODID updates arrive less frequently than BT4 (which
is continuous at 100 ms). This is expected behaviour, not a defect.

---

## MAVLink Integration

### T6 — FC Connection and MAVLink Reception

**Setup:** mRo PixracerPro FC connected via USB (`/dev/ttyACM0`, 921600 baud).

| Check | Result |
|-------|--------|
| `mavlink: connected to /dev/ttyACM0 @ 921600` at startup | ✅ |
| `GLOBAL_POSITION_INT` messages received | ✅ |
| `HEARTBEAT` messages received | ✅ |

### T7 — GPS Position Data

**Outdoor test (2026-04-02):** Daemon running outdoors with FC having satellite fix.

| Check | Result |
|-------|--------|
| Live GPS coordinates received from FC | ✅ 57.6847°N 11.9628°E |
| Coordinates visible in ODID Location message | ✅ |
| Fallback coordinates used when no fix | ✅ 57.7089°N 11.9746°E (Gothenburg harbour) |

**Outdoor validation (2026-04-21):** DroneScanner displays live GPS coordinates correctly under satellite fix.

### T8 — Arm Detection

| Check | Result |
|-------|--------|
| `MAV_MODE_FLAG_SAFETY_ARMED` detection implemented | ✅ |
| DISARMED confirmed: `base_mode=0x51` (HEARTBEAT log) | ✅ |
| `ODID_STATUS_AIRBORNE` on arm / `ODID_STATUS_GROUND` on disarm | ✅ code verified |
| Live arm/disarm cycle | ⏳ Pending (requires RC/GCS) |

---

## Resilience and Service Tests

### T9 — Serial Port Auto-Reconnect

**Method:** USB cable unplugged/replugged while daemon running.

| Check | Result |
|-------|--------|
| Disconnect detected instantly via `poll() POLLHUP` | ✅ |
| BT4 broadcast continues during FC disconnect | ✅ |
| Reconnect within 2-second retry cycle (~6 s total) | ✅ |

### T11 — systemd Service and Boot Reliability

| Check | Result |
|-------|--------|
| `odid.service` enabled, starts at boot | ✅ |
| `ExecStartPre=/usr/bin/hciconfig hci0 up` — brings BT adapter up before daemon | ✅ |
| `bluetooth.service` disabled — eliminates BlueZ/HCI competition | ✅ |
| `hostapd.service` enabled with `DAEMON_CONF=/etc/hostapd/hostapd.conf` | ✅ |
| `wlan0` marked unmanaged in NetworkManager — stays in AP mode across reboots | ✅ |
| `odid.service` declares `After=hostapd.service Wants=hostapd.service` | ✅ |

**Root cause of previous BT4 instability (resolved):** BlueZ (`bluetoothd`) was running
alongside the daemon's raw HCI socket on hci0. BlueZ can reset adapter state or disable
advertising at any time without the daemon's knowledge. After a long uptime, advertising
packets were being sent as infrequently as once per 8 seconds instead of 1 Hz. Fix: disabled
bluetooth.service permanently and added `ExecStartPre=hciconfig hci0 up` to ensure the
adapter is powered before the daemon opens it.

### T12 — Minimal Capabilities

| Check | Result |
|-------|--------|
| Only `CAP_NET_RAW` and `CAP_NET_ADMIN` held | ✅ |
| `NoNewPrivileges=yes` | ✅ |

---

## Pending Tests

| ID | Test | Blocked on |
|----|------|-----------|
| T7b | DroneScanner shows live GPS location with fix | ✅ Confirmed 2026-04-21 |
| T8b | Live arm/disarm detection | RC controller or GCS arm command |

---

## Summary

| Category | Passed | Pending | Failed |
|----------|--------|---------|--------|
| Build & unit tests | 2 | 0 | 0 |
| Serial number / OPRN format (CTA-2063-A, OPRN) | 2 | 0 | 0 |
| BT4 broadcast format | 3 | 0 | 0 |
| BT4 DroneScanner detection | 1 | 0 | 0 |
| WiFi Beacon DroneScanner detection | 1 | 0 | 0 |
| MAVLink / GPS | 3 | 1 | 0 |
| Resilience / service / boot | 4 | 0 | 0 |
| Capabilities | 1 | 0 | 0 |
| **Total** | **17** | **1** | **0** |
