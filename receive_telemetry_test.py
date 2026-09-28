#!/usr/bin/env python3
"""
receive_telemetry_test.py
-------------------------
A lightweight receiver script to run on your Ground Station Laptop (or any computer
with the Ground YRRC 433 MHz radio plugged into USB).
It prints received MAVLink packets (Heartbeat, Attitude, GPS, Battery) in real time.

Requirements:
    pip install pymavlink

Usage:
    python3 receive_telemetry_test.py [--port /dev/ttyUSB0] [--baud 57600]
"""

import sys
import argparse

try:
    from pymavlink import mavutil
except ImportError:
    print("[ERROR] 'pymavlink' is not installed.")
    print("Please install it using: pip3 install pymavlink (or pip install pymavlink)")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Receiver test for YRRC 433 MHz Ground Radio")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port of Ground Radio (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate (default: 57600)")
    args = parser.parse_args()

    print("=" * 65)
    print("      YRRC 433 MHz GROUND RADIO TELEMETRY RECEIVER TEST")
    print("=" * 65)
    print(f" Port      : {args.port}")
    print(f" Baud Rate : {args.baud}")
    print("-" * 65)

    try:
        mav = mavutil.mavlink_connection(args.port, baud=args.baud)
        print(f"[OK] Listening on {args.port} @ {args.baud}... Waiting for radio packets.\n")
    except Exception as e:
        print(f"[FAILED] Cannot open {args.port}: {e}")
        print("\nTroubleshooting tips:")
        print(" 1. Check your USB port: ls -l /dev/ttyUSB* or dmesg | tail")
        print(" 2. Ensure your user has permissions: sudo chmod 666 /dev/ttyUSB*")
        print(" 3. Close QGroundControl if it's already using this port.")
        sys.exit(1)

    try:
        while True:
            msg = mav.recv_match(blocking=True, timeout=2.0)
            if msg is None:
                print("[WAITING] No packet received in last 2 seconds... (Check RF green LED on radios)")
                continue

            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                print(f"[HEARTBEAT] Vehicle #{msg.get_srcSystem()} | Type: {msg.type} | Autopilot: {msg.autopilot} | Base Mode: {msg.base_mode}")

            elif msg_type == "SYS_STATUS":
                voltage = msg.voltage_battery / 1000.0
                current = msg.current_battery / 100.0
                print(f"[BATTERY]   Voltage: {voltage:.2f} V | Current: {current:.2f} A | Remaining: {msg.battery_remaining}%")

            elif msg_type == "GPS_RAW_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0
                print(f"[GPS]       Fix: {msg.fix_type}D | Lat: {lat:.6f}, Lon: {lon:.6f} | Alt: {alt:.1f} m | Sats: {msg.satellites_visible}")

            elif msg_type == "ATTITUDE":
                import math
                roll_deg = math.degrees(msg.roll)
                pitch_deg = math.degrees(msg.pitch)
                yaw_deg = math.degrees(msg.yaw)
                print(f"[ATTITUDE]  Roll: {roll_deg:+5.1f}° | Pitch: {pitch_deg:+5.1f}° | Yaw: {yaw_deg:+5.1f}°")

            elif msg_type == "BAD_DATA":
                print(f"[NOTE] Raw RF byte activity detected (framing: {len(msg.data)} bytes)")

            else:
                print(f"[{msg_type}] Packet received from vehicle #{msg.get_srcSystem()}")

    except KeyboardInterrupt:
        print("\n[STOP] Exiting receiver test.")
    finally:
        mav.close()


if __name__ == "__main__":
    main()
