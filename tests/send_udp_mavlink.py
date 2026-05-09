#!/usr/bin/env python3
"""
Send a few MAVLink packets over UDP to test the daemon's UDP source mode.
Usage: python3 send_udp_mavlink.py [host] [port]
"""
import sys
import time
from pymavlink import mavutil

host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 14550

conn = mavutil.mavlink_connection(f"udpout:{host}:{port}", source_system=1, source_component=1)

print(f"Sending MAVLink packets to UDP {host}:{port} ...")

for i in range(5):
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_FIXED_WING,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavutil.mavlink.MAV_MODE_AUTO_ARMED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )
    print(f"  [{i+1}] HEARTBEAT sent")

    conn.mav.global_position_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
        576847000,    # lat  57.6847 deg * 1e7
        119628000,    # lon  11.9628 deg * 1e7
        100000,       # alt  100 m MSL (mm)
        50000,        # relative_alt 50 m (mm)
        0, 0, 0,      # vx vy vz cm/s
        9000,         # hdg 90 deg (cdeg)
    )
    print(f"  [{i+1}] GLOBAL_POSITION_INT sent (lat=57.6847 lon=11.9628 alt=100m)")

    time.sleep(1)

print("Done.")
