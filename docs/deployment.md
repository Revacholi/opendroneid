# Deployment Guide

This guide covers building, installing, and configuring `odid-daemon` on a Raspberry Pi
running Raspberry Pi OS (64-bit, Bookworm or later).

---

## Prerequisites

### Hardware

- Raspberry Pi 400, RPi CM4, or any RPi with BCM43455 (built-in BT4 + WiFi)
- mRo PixracerPro (or any Ardupilot FC) connected via USB
- u-blox GNSS connected to the FC

### Software Dependencies

```bash
sudo apt update
sudo apt install -y \
    git cmake build-essential \
    libbluetooth-dev \
    libnl-3-dev libnl-genl-3-dev
```

---

## Build

```bash
git clone --recurse-submodules https://github.com/<your-org>/opendroneid-linux.git
cd opendroneid-linux
mkdir build && cd build
cmake ..
make -j4
```

Run unit tests:

```bash
ctest
# Expected: 2/2 tests passed
```

---

## Configuration

### 1. Create config directory

```bash
sudo mkdir -p /etc/odid
sudo cp odid.conf.example /etc/odid/odid.conf
```

### 2. Edit `/etc/odid/odid.conf`

```ini
[basic_id]
; UAS serial number in ANSI/CTA-2063-A-2019 format (ASD-STAN prEN 4709-002 §5.2.2).
; Structure: [4-char manufacturer code][1-char length code 0-9/A-F][serial chars]
; Only uppercase letters A-Z and digits 0-9 — no hyphens or spaces.
; Example: SSRS5GBG01 → mfr=SSRS, length=5, serial=GBG01
uas_id           = "XXXX5YYYYY"
id_type          = 1       ; 1 = SERIAL_NUMBER (ANSI/CTA-2063-A)
ua_type          = 1       ; 1 = Aeroplane (fixed wing)

[operator]
; EU Operator Registration Number (OPRN) from Transportstyrelsen / EASA portal.
; Format: 3-char ISO country code + 13 alphanumeric chars, e.g. "SWE3jxza7qbzvu5d"
operator_id      = "SWExxxxxxxxxxxx"
operator_id_type = 0       ; 0 = CAA Assigned (EU operator registration)

[self_id]
description      = "Sea rescue surveillance"
description_type = 0       ; 0 = plain text

[system]
operator_location_type = 0 ; 0 = Takeoff location
classification_type    = 1 ; 1 = EU classification
category               = 3 ; 3 = Specific
class                  = 4 ; 4 = C4

[serial]
device = "/dev/ttyACM0"
baud   = 921600

[broadcast]
bt4_enabled         = true
bt5_enabled         = true   ; will be disabled automatically if unsupported
wifi_beacon_enabled = true
wifi_iface          = "wlan0"
bt_adapter          = "hci0"

[location_defaults]
; Fallback coordinates shown until live GPS fix is available.
; Set to the drone's typical operating base.
lat    = 57.7089
lon    = 11.9746
alt    = 10.0
status = 1       ; 1 = Ground
```

### 3. Obtaining Registration IDs

| ID | Format | Where to get it |
|----|--------|----------------|
| `uas_id` | ANSI/CTA-2063-A: 4-char mfr code + length indicator + serial chars | Self-assigned by manufacturer for custom-built UAS; must be unique per drone |
| `operator_id` | EU OPRN: 3-char country code + 13 alphanumeric chars | Transportstyrelsen / EASA operator registration portal |

Both are required for legal Remote ID broadcast in Sweden.

---

## Installation

### Install binary

```bash
sudo cp build/odid-daemon /usr/local/bin/odid-daemon
sudo chmod 755 /usr/local/bin/odid-daemon
```

### Install systemd service

```bash
sudo cp systemd/odid.service /etc/systemd/system/odid.service
sudo systemctl daemon-reload
sudo systemctl enable odid.service
sudo systemctl start odid.service
```

### Verify

```bash
sudo systemctl status odid.service
```

Expected output (abbreviated):
```
● odid.service - Open Drone ID Broadcaster
     Active: active (running) since ...
     Main PID: XXXX (odid-daemon)
```

---

## Updating the Binary

Because the service holds the binary open while running, use `rm` then `cp`:

```bash
sudo systemctl stop odid.service
sudo rm /usr/local/bin/odid-daemon
sudo cp build/odid-daemon /usr/local/bin/odid-daemon
sudo systemctl start odid.service
```

---

## Monitoring and Logs

```bash
# Live log feed
sudo journalctl -u odid.service -f

# Last 50 lines
sudo journalctl -u odid.service -n 50

# Since last boot
sudo journalctl -u odid.service -b
```

Key log messages:

| Message | Meaning |
|---------|---------|
| `bt: params set — ADV_NONCONN_IND 100ms all-channels` | BT4 advertising active |
| `mavlink: connected to /dev/ttyACM0 @ 921600` | FC connected, MAVLink flowing |
| `mavlink: reader thread started, waiting for /dev/ttyACM0` | FC not yet connected — will retry |
| `mavlink: device disconnected — reconnecting` | FC USB unplugged |
| `mavlink: reconnected to /dev/ttyACM0` | FC reconnected |
| `odid_state: drone ARMED` | Arm detected, status → Airborne |
| `odid_state: drone DISARMED` | Disarm detected, status → Ground |
| `bt: BT5 LR not supported by this controller — disabling` | Hardware limitation, BT4 unaffected |

---

## WiFi Beacon Setup (Optional)

WiFi Beacon requires `wlan0` to be in AP mode. This conflicts with using `wlan0` as a
WiFi client. **Connect the RPi via Ethernet before proceeding.**

```bash
# Install hostapd
sudo apt install hostapd

# Create hostapd config
sudo tee /etc/hostapd/hostapd_odid.conf > /dev/null << 'EOF'
interface=wlan0
driver=nl80211
ssid=ODID-SSRS
hw_mode=g
channel=6
wmm_enabled=0
EOF

# Point hostapd to this config
sudo tee -a /etc/hostapd/hostapd.conf > /dev/null << 'EOF'
interface=wlan0
driver=nl80211
ssid=ODID-SSRS
hw_mode=g
channel=6
EOF

# Enable and start hostapd
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl start hostapd
```

The `odid-daemon` will automatically detect the fallback path and update the
`vendor_elements` in hostapd's configuration every broadcast cycle.

---

## Bluetooth rfkill (CM4 and some RPi variants)

On Compute Module 4 and certain RPi configurations, Bluetooth is soft-blocked by rfkill
at first boot.

**Docker deployment:** The container's entrypoint script runs `rfkill unblock bluetooth`
automatically on startup (requires the `NET_ADMIN` capability already present in
`compose.yaml`). No manual step needed.

**Native / systemd deployment:** Unblock it once — the setting persists across reboots:

```bash
sudo rfkill unblock bluetooth
```

Verify:
```bash
sudo rfkill list
# hci0: Bluetooth
#   Soft blocked: no
#   Hard blocked: no
```

---

## Verifying BT4 Broadcast

**Using nRF Connect (iOS or Android):**
1. Open nRF Connect → Scanner tab
2. Look for a device advertising UUID `0xFFFA` (ASTM Remote ID)
3. Tap to inspect — Service Data AD element should contain 25 bytes of ODID payload

**Using btmon (on the RPi):**
```bash
sudo btmon 2>&1 | grep -A5 "ASTM Remote ID"
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `open /dev/ttyACM0: No such file or directory` at startup | FC not yet connected | Normal — daemon retries every 2 s |
| `bt: HCI error 0x01 for ogf=0x08 ocf=0x0036` | BT5 not supported (expected) | Not an error — BT4 continues unaffected |
| `wifi: nl80211 inject failed, switching to hostapd fallback` | BCM43455 driver limitation (expected) | Normal — hostapd fallback used |
| Service not starting | Config file missing | Check `/etc/odid/odid.conf` exists |
| No BT4 advertisement visible | `hci0` adapter down | `sudo hciconfig hci0 up` |
| `bt: could not bring hci0 up: Operation not possible due to RF-kill` | Bluetooth soft-blocked by rfkill (common on CM4) | `sudo rfkill unblock bluetooth` — persists across reboots |
| `cp: cannot create regular file: Text file busy` when updating binary | Service still running | `sudo systemctl stop odid.service` first |

---

## Uninstall

```bash
sudo systemctl stop odid.service
sudo systemctl disable odid.service
sudo rm /etc/systemd/system/odid.service
sudo rm /usr/local/bin/odid-daemon
sudo rm -rf /etc/odid
sudo systemctl daemon-reload
```
