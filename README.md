# opendroneid-linux

Software Direct Remote ID broadcaster for Raspberry Pi (BCM43455). Broadcasts over Bluetooth 4 and WiFi Beacon without dedicated Remote ID hardware, receiving GPS from an Ardupilot flight controller via MAVLink. Compliant with ASTM F3411-22a and ASD-STAN prEN 4709-002 (EU 2019/945).

## Build and run

```bash
sudo apt install cmake build-essential libbluetooth-dev libnl-3-dev libnl-genl-3-dev
git clone --recurse-submodules https://github.com/Revacholi/opendroneid.git
cd opendroneid && mkdir build && cd build && cmake .. && make -j4

sudo cp build/odid-daemon /usr/local/bin/
sudo cp odid.conf.example /etc/odid/odid.conf   # edit uas_id and operator_id
sudo cp systemd/odid.service /etc/systemd/system/
sudo systemctl enable --now odid.service
```

## Verify

Install **[Drone Scanner](https://github.com/dronetag/drone-scanner)** (Android/iOS) — the drone should appear under both Bluetooth and Wi-Fi scans within a few seconds of the service starting.

## License

MIT
