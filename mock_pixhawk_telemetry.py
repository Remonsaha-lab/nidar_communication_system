#!/usr/bin/env python3
"""
mock_pixhawk_telemetry.py
-------------------------
Simulates a Pixhawk Flight Controller by generating hardcoded/simulated MAVLink
telemetry and streaming it directly over a YRRC 433 MHz Telemetry Radio connected
to a Raspberry Pi 4 USB port (/dev/ttyUSB0).

Requirements:
    pip install pymavlink

Usage:
    python3 mock_pixhawk_telemetry.py [--port /dev/ttyUSB0] [--baud 57600]
"""
### Ping test 
##laptop terminal
# python3 -c "import serial; s=serial.Serial('/dev/ttyUSB0', 57600, timeout=10); print('LISTENING...'); print('GOT:', s.readline().decode(errors='ignore')); s.close()"
## Rpi terminal 
# python3 -c "import serial; s=serial.Serial('/dev/ttyUSB0', 57600); s.write(b'HELLO FROM RPI\n'); s.close(); print('SENT!')"

"""
1. On your Laptop (remon@makima:~/nidar$):
Start the telemetry receiver:

bash
python3 receive_telemetry_test.py --port /dev/ttyUSB0 --baud 57600
2. On your Raspberry Pi (vihang26@vihang26:~/nidar$):
Start the mock Pixhawk generator (make sure to use 57600 baud):

bash
python3 mock_pixhawk_telemetry.py --port /dev/ttyUSB0 --baud 57600

"""

import sys
import time
import math
import argparse

try:
    from pymavlink import mavutil
except ImportError:
    print("[ERROR] 'pymavlink' is not installed.")
    print("Please install it using: pip3 install pymavlink (or pip install pymavlink)")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Mock Pixhawk Telemetry Generator for YRRC 433 MHz Radio")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port of the YRRC 433 radio (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate of the radio (default: 57600)")
    parser.add_argument("--lat", type=float, default=28.613939, help="Hardcoded Latitude in degrees (e.g. 28.613939)")
    parser.add_argument("--lon", type=float, default=77.209021, help="Hardcoded Longitude in degrees (e.g. 77.209021)")
    parser.add_argument("--alt", type=float, default=15.0, help="Hardcoded Altitude in meters (default: 15.0)")
    args = parser.parse_args()

    print("=" * 65)
    print("  MOCK PIXHAWK TELEMETRY GENERATOR (RPi 4 -> YRRC 433 MHz Radio)")
    print("=" * 65)
    print(f" Port        : {args.port}")
    print(f" Baud Rate   : {args.baud}")
    print(f" Base GPS    : Lat {args.lat:.6f}, Lon {args.lon:.6f}, Alt {args.alt} m")
    print("-" * 65)

    try:
        # Establish MAVLink serial connection over the YRRC radio port
        # source_system=1 (Vehicle 1), source_component=1 (Autopilot)
        mav = mavutil.mavlink_connection(
            device=args.port,
            baud=args.baud,
            source_system=1,
            source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        )
        print(f"[OK] Opened serial link on {args.port} @ {args.baud} baud.")
        print("[INFO] Broadcasting simulated MAVLink telemetry... Press Ctrl+C to stop.\n")
    except Exception as e:
        print(f"[FAILED] Could not open serial port {args.port}: {e}")
        print("\nTroubleshooting tips:")
        print(" 1. Check if the radio is plugged in: ls -l /dev/ttyUSB*")
        print(" 2. Grant permissions: sudo usermod -a -G dialout $USER (then re-login)")
        print("    or run: sudo chmod 666 /dev/ttyUSB0")
        sys.exit(1)

    start_time = time.time()
    last_1hz = 0.0
    last_5hz = 0.0

    step = 0

    try:
        while True:
            now = time.time()
            elapsed = now - start_time
            time_boot_ms = int(elapsed * 1000)

            # ------------------------------------------------------------------
            # 1 Hz Loop: HEARTBEAT & BATTERY / SYSTEM STATUS
            # ------------------------------------------------------------------
            if now - last_1hz >= 1.0:
                last_1hz = now

                # 1. Heartbeat: Informs GCS that Vehicle 1 (Quadcopter) is online
                mav.mav.heartbeat_send(
                    type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
                    autopilot=mavutil.mavlink.MAV_AUTOPILOT_PX4,
                    base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                    custom_mode=4,  # Auto / Guided
                    system_status=mavutil.mavlink.MAV_STATE_ACTIVE
                )

                # 2. System Status & Battery: 12.4V (4S storage/nominal), 92% battery
                mav.mav.sys_status_send(
                    onboard_control_sensors_present=0b11111111,
                    onboard_control_sensors_enabled=0b11111111,
                    onboard_control_sensors_health=0b11111111,
                    load=250,                   # 25.0% CPU load
                    voltage_battery=12400,      # 12,400 mV = 12.4 V
                    current_battery=1250,       # 1,250 cA = 12.5 A
                    battery_remaining=92,       # 92%
                    drop_rate_comm=0,
                    errors_comm=0,
                    errors_count1=0,
                    errors_count2=0,
                    errors_count3=0,
                    errors_count4=0
                )

                # 3. GPS Raw: 3D Fix, 14 satellites
                lat_int = int(args.lat * 1e7)
                lon_int = int(args.lon * 1e7)
                mav.mav.gps_raw_int_send(
                    time_usec=int(now * 1e6),
                    fix_type=3,                 # 3 = 3D Fix
                    lat=lat_int,
                    lon=lon_int,
                    alt=int(args.alt * 1000),   # millimeters
                    eph=120,                    # HDOP
                    epv=150,                    # VDOP
                    vel=250,                    # Ground speed in cm/s (2.5 m/s)
                    cog=9000,                   # Course over ground in cdeg (90.0 deg)
                    satellites_visible=14
                )

                print(f"[{elapsed:6.1f}s] Sent HEARTBEAT | SYS_STATUS (12.4V, 92%) | GPS 3D Fix (14 Sats)")

            # ------------------------------------------------------------------
            # 5 Hz Loop: ATTITUDE & GLOBAL POSITION (Fast telemetry)
            # ------------------------------------------------------------------
            if now - last_5hz >= 0.2:
                last_5hz = now
                step += 1

                # Simulate gentle rolling and pitching using sine wave
                roll = math.radians(5.0 * math.sin(step * 0.2))    # +/- 5 deg roll
                pitch = math.radians(3.0 * math.cos(step * 0.15))  # +/- 3 deg pitch
                yaw = math.radians((step * 2.0) % 360)             # slowly turning

                mav.mav.attitude_send(
                    time_boot_ms=time_boot_ms,
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    rollspeed=0.01,
                    pitchspeed=0.01,
                    yawspeed=0.05
                )

                # Send Global Position (lat/lon in 1e7 deg, alt in mm)
                mav.mav.global_position_int_send(
                    time_boot_ms=time_boot_ms,
                    lat=int(args.lat * 1e7),
                    lon=int(args.lon * 1e7),
                    alt=int(args.alt * 1000),
                    relative_alt=int(args.alt * 1000),
                    vx=100,  # 1.0 m/s
                    vy=0,
                    vz=0,
                    hdg=int(math.degrees(yaw) * 100)
                )

                # VFR HUD (speed, heading, altitude)
                mav.mav.vfr_hud_send(
                    airspeed=3.0,
                    groundspeed=3.2,
                    heading=int(math.degrees(yaw)) % 360,
                    throttle=45,
                    alt=args.alt,
                    climb=0.1
                )

            # Sleep briefly to avoid pegging CPU core
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[STOP] Stopping mock telemetry generator.")
    finally:
        mav.close()
        print("[OK] Serial connection closed cleanly.")


if __name__ == "__main__":
    main()
