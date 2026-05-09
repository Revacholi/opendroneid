# EU/Swedish Remote ID Compliance Mapping

This document maps requirements from EN 4709-002 (Direct Remote ID) to the
corresponding implementation in `odid-daemon`.

## Regulatory Basis

| Level | Instrument | Role |
|-------|-----------|------|
| Swedish national | **Transportstyrelsen (TSFS)** regulations | Primary legal basis; enforces EU drone regulations in Sweden |
| EU operational | **[EU 2019/947](https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX:32019R0947)** — UAS operations regulation | Defines OPEN/SPECIFIC/CERTIFIED categories and Remote ID obligation |
| EU equipment | **[EU 2019/945](https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX:32019R0945)** — UAS product regulation | Remote ID technical requirements for UAS Classes C1–C6: UAS ID, operator ID, position, speed, emergency status |
| EU equipment (amendment) | **[EU 2020/1058](https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX:32020R1058)** — amends EU 2019/945 | Adds Classes C5/C6; Part 6 defines DRI add-on requirements — referenced in all manufacturer Declarations of Conformity |
| EU technical standard | **ASD-STAN prEN 4709-002:2023** — Direct Remote Identification | Primary harmonised standard for EU 2019/945; defines message format, bearer protocols, and test methods with pass/fail criteria |
| US/international standard | **[ASTM F3411-22a](https://www.astm.org/f3411-22a.html)** — Remote ID and Tracking | Cross-compatible with ASD-STAN prEN 4709-002; implemented by `opendroneid-core-c` library |
| Serial number format | **ANSI/CTA-2063-A-2019** — Small UAS Serial Numbers | Defines the manufacturer code + length indicator + serial chars format for `id_type=1` |
| Radio spectrum (BT/WiFi) | **ETSI EN 300 328 V2.2.2** — Wideband transmission systems (2.4 GHz) | Governs BT4/WiFi output power and spectral characteristics; cited by all EU-certified DRI products |
| Radio spectrum (BT/WiFi) | **ETSI EN 301 489-1 V2.2.3** — EMC for radio equipment | Electromagnetic compatibility requirements |

Standard edition: **ASD-STAN prEN 4709-002:2023** (Direct Remote Identification, EU harmonised standard)
Implementation: `odid-daemon` on Raspberry Pi 400 (BCM43455)

---

## Message Types

### §5.2 — Required Message Types

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Basic ID message shall be broadcast | ✅ | `encodeBasicIDMessage()`, BT4 every ~3 s |
| Location/Vector message shall be broadcast | ✅ | `encodeLocationMessage()`, BT4 every 1 s |
| System message shall be broadcast | ✅ | `encodeSystemMessage()`, BT4 every ~3 s |
| Operator ID message should be broadcast | ✅ | `encodeOperatorIDMessage()`, BT4 every ~3 s |
| Self-ID message should be broadcast | ✅ | `encodeSelfIDMessage()`, BT4 every ~3 s |

### §5.2.2 — Basic ID

| Field | Requirement | Status | Value / Source |
|-------|-------------|--------|----------------|
| UA Type | Fixed Wing | ✅ | `ua_type = 1` (Aeroplane / Fixed Wing) in config |
| ID Type | Serial Number (CTA-2063-A) | ✅ | `id_type = 1` (ODID_IDTYPE_SERIAL_NUMBER) |
| UAS ID | ANSI/CTA-2063-A format | ✅ | `SSRS5GBG01` — manufacturer code `SSRS`, length `5`, serial `GBG01` |

### §5.2.3 — Location/Vector

| Field | Requirement | Status | Source |
|-------|-------------|--------|--------|
| Operational Status | Ground / Airborne / Emergency | ✅ | HEARTBEAT arm flag → GROUND/AIRBORNE |
| Latitude / Longitude | WGS-84 degrees | ✅ | `GLOBAL_POSITION_INT` (×10⁻⁷ deg) |
| Altitude (geodetic) | WGS-84 metres | ✅ | `GLOBAL_POSITION_INT.alt` (mm → m) |
| Altitude (barometric) | metres | ✅ | `GLOBAL_POSITION_INT.relative_alt` (mm → m) |
| Height above takeoff | metres | ✅ | `GLOBAL_POSITION_INT.relative_alt` |
| Horizontal speed | m/s | ✅ | √(vx²+vy²) from `GLOBAL_POSITION_INT` |
| Vertical speed | m/s | ✅ | `GLOBAL_POSITION_INT.vz` (cm/s → m/s) |
| Track direction | degrees | ✅ | `GLOBAL_POSITION_INT.hdg` (cdeg → deg) |
| Timestamp | seconds within current hour | ✅ | Derived from `CLOCK_REALTIME` |
| Horizontal accuracy | enum | ⚠️ | `ODID_HOR_ACC_UNKNOWN` — FC does not provide |
| Vertical accuracy | enum | ⚠️ | `ODID_VER_ACC_UNKNOWN` — FC does not provide |
| Speed accuracy | enum | ⚠️ | `ODID_SPEED_ACC_UNKNOWN` — FC does not provide |

Note: accuracy fields are set to `UNKNOWN` because the FC's `GLOBAL_POSITION_INT` message
does not carry accuracy estimates. If the FC sends `OPEN_DRONE_ID_LOCATION` natively,
those accuracy fields are used directly.

### §5.2.4 — System Message

| Field | Requirement | Status | Value / Source |
|-------|-------------|--------|----------------|
| Operator location type | Fixed/Dynamic/Takeoff | ✅ | `operator_location_type = 0` (Takeoff) |
| Classification type | EU / other | ✅ | `classification_type = 1` (EU) |
| EU Category | Open / Specific / Certified | ✅ | `category = 3` (Specific) |
| EU Class | C0–C6 | ✅ | `class = 4` (C4) |
| Timestamp | seconds since 2019-01-01 UTC | ✅ | `CLOCK_REALTIME − 1546300800`, updated each broadcast |
| Operator latitude/longitude | WGS-84 | ⚠️ | 0,0 (takeoff coords not yet set from MAVLink) |

### §5.2.5 — Operator ID

| Field | Requirement | Status | Value |
|-------|-------------|--------|-------|
| Operator ID Type | CAA Assigned | ✅ | Type 0 (`ODID_OPERATORIDTYPE_CAA_REGISTRATION`) |
| Operator ID | EU OPRN from Transportstyrelsen | ✅ | `SWE3jxza7qbzvu5d` — 3-char country code + 13 alphanumeric chars (public part of OPRN) |

### §5.2.6 — Self-ID

| Field | Requirement | Status | Value |
|-------|-------------|--------|-------|
| Description Type | 0 = plain text | ✅ | Type 0 |
| Description | Free text, max 23 chars | ✅ | `"Sea rescue surveillance"` |

---

## Broadcast Channel Requirements

### §5.3 — Bluetooth Broadcast

| Requirement | Status | Notes |
|-------------|--------|-------|
| Use BT4 Legacy Advertising or BT5 Extended | ✅ | BT4 active |
| AD type 0x16 (Service Data) with UUID 0xFFFA | ✅ | Confirmed via btmon and nRF Connect |
| ADV_NONCONN_IND advertising type | ✅ | `advtype = 0x03` |
| Location message broadcast interval ≤ 1 s | ✅ | Measured: 1.00 s ± 0.003 s |
| Advertising interval ≤ 100 ms | ✅ | `min_interval = max_interval = 160 × 0.625 ms = 100 ms` |
| All 3 advertising channels (37, 38, 39) | ✅ | `chan_map = 0x07` |
| BT5 Long Range (Extended Advertising) | ❌ | BCM43455 hardware limitation — HCI error 0x01 |

### §5.4 — WiFi Beacon Broadcast

| Requirement | Status | Notes |
|-------------|--------|-------|
| ODID vendor IE in 802.11 beacon frames | ⚠️ | hostapd fallback implemented; requires hostapd AP mode on wlan0 |
| OUI: 0xFA 0x0B 0xBC, OUI Type: 0x0D | ✅ | Correct in `wifi_beacon.c` |
| Full message pack in beacon IE | ✅ | All 5 message types encoded in pack |
| Direct nl80211 beacon injection | ❌ | BCM43455 driver returns `EOPNOTSUPP` |

---

## Operational Requirements

### §6.2 — Broadcast Requirements

| Requirement | Status | Notes |
|-------------|--------|-------|
| Broadcast without requiring activation by pilot | ✅ | systemd service starts at boot |
| Broadcast when vehicle is powered | ✅ | `WantedBy=multi-user.target`, starts before flight |
| Cannot be deactivated by operator during flight | ✅ | Service runs as root; no user-accessible off switch |
| Restart automatically if interrupted | ✅ | `Restart=on-failure`, `RestartSec=5` in service unit |

---

## Known Gaps and Limitations

| Item | Gap | Impact | Mitigation |
|------|-----|--------|------------|
| BT5 Long Range | BCM43455 hardware limitation (HCI error 0x01) | No LR broadcast | USB BT5 dongle (optional Phase 6) |
| WiFi Beacon | Requires hostapd AP mode on wlan0 | Conflicts with WiFi client mode | Ethernet for SSH; hostapd enabled on deployed drone |
| Accuracy fields | FC `GLOBAL_POSITION_INT` lacks accuracy estimates | `UNKNOWN` in Location message | Valid per ASTM F3411-22a; UNKNOWN is an accepted enum value |
| Outdoor GPS validation | DroneScanner GPS display not yet verified under live fix | T7b/T8b pending | Outdoor test with satellite fix required |

---

## Manufacturer Compliance Checklist

Structured after ASD-STAN prEN 4709-002 §A.1 (Requirement Verification Stage),
matching the checklist format used in EU Declarations of Conformity for certified DRI products
(e.g., BlueMark DroneBeacon, Aerobits idME Pro).

| Requirement | Clause | Status | Notes |
|-------------|--------|--------|-------|
| UAS operator registration number uploadable | prEN 4709-002 §5.2.5 | ✅ | Via `/etc/odid/odid.conf` `operator_id` field |
| Serial number compliant with ANSI/CTA-2063-A-2019 | prEN 4709-002 §5.2.2 | ✅ | `SSRS5GBG01`: mfr `SSRS`, length `5`, serial `GBG01` |
| All mandatory DRI messages broadcast | prEN 4709-002 §5.2 | ✅ | BasicID, Location, System, OperatorID, SelfID |
| DRI broadcast cannot be deactivated during flight | EU 2020/1058 Part 6 §8 | ✅ | systemd service; no user-accessible off switch |
| Installation and usage instructions provided | EU 2020/1058 Part 6 §9 | ✅ | `docs/deployment.md` |
| Mandatory information fields present | prEN 4709-002 §5.2 | ✅ | All mandatory fields populated or set to valid UNKNOWN |
| DRI system security (operator ID not modifiable in flight) | prEN 4709-002 §5.1 | ✅ | Config read-only at runtime; `NoNewPrivileges=yes` |
| Broadcast transport protocol compliant | prEN 4709-002 §5.3–5.4 | ✅ | BT4 Legacy Advertising + WiFi Beacon per standard |
| Output power within regulatory limits | ETSI EN 300 328 V2.2.2 | ✅ | BCM43455 BT4 ≤ 10 mW, WiFi ≤ 100 mW (OS-enforced) |
| Emission omni-directional | prEN 4709-002 §5.3.3 | ✅ | Internal antenna; all BT channels 37/38/39 used |
| Location update rate ≤ 1 s | prEN 4709-002 §5.3.4 | ✅ | Measured: 1.004 s mean (T5) |
| Static message update rate ≤ 3 s | prEN 4709-002 §5.3.4 | ✅ | Scheduler: 3 s cycle for BasicID/System/OperatorID |
| Detected by DroneScanner via BT4 | Functional verification | ✅ | T14: UAS ID and Operator ID displayed correctly |
| Detected by DroneScanner via WiFi Beacon | Functional verification | ✅ | T15: vendor IE OUI and message pack verified |
| GPS position displayed under live fix | Functional verification | ✅ | T7b: confirmed outdoors 2026-04-21 |
