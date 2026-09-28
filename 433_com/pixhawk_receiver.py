#!/usr/bin/env python3
"""
Receive and print telemetry from a Pixhawk over a SiK telemetry radio
(MAVLink, 57600 baud) using pymavlink.

Install:
    pip install pymavlink pyserial

usbipd list
usbipd bind --busid 4-2
usbipd attach --wsl --busid 4-2

Examples:
    # Linux (ground radio usually shows up as /dev/ttyUSB0)
    python pixhawk_receiver.py --conn /dev/ttyUSB0
    # Windows
    python pixhawk_receiver.py --conn COM5
    # Find your port
    python pixhawk_receiver.py --list-ports
    # Log everything to CSV, include extra (servo/local position) messages
    python pixhawk_receiver.py --conn /dev/ttyUSB0 --csv log.csv --extra
"""

import argparse
import csv
import math
import sys
import time

from pymavlink import mavutil

# The SiK link is slow (57600 baud serial, ~64 kbps over the air by default),
# so request only a light set of messages, at low rates.
BASE_MESSAGES = {
    "ATTITUDE": 4,
    "GLOBAL_POSITION_INT": 2,
    "GPS_RAW_INT": 1,
    "VFR_HUD": 2,
    "SYS_STATUS": 1,
    "RC_CHANNELS": 1,
}
EXTRA_MESSAGES = {
    "SERVO_OUTPUT_RAW": 1,
    "LOCAL_POSITION_NED": 2,
}


def list_ports():
    from serial.tools import list_ports as lp
    ports = list(lp.comports())
    if not ports:
        print("No serial ports found.")
    for p in ports:
        print(f"{p.device}\t{p.description}")


def connect(conn_str, baud, source_system):
    print(f"Connecting to {conn_str} @ {baud} baud ...")
    master = mavutil.mavlink_connection(
        conn_str,
        baud=baud,
        source_system=source_system,
        autoreconnect=True,
    )
    print("Waiting for heartbeat (radio link can take several seconds) ...")
    while True:
        hb = master.wait_heartbeat(timeout=5)
        if hb is not None:
            break
        print("  still waiting ... check radio LEDs, baud, and that the air-side radio is powered")
    print(f"Heartbeat from system {master.target_system}, component {master.target_component}")
    return master


def request_messages(master, wanted):
    """Ask the autopilot to stream the messages we care about."""
    for name, hz in wanted.items():
        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1e6 / hz),
            0, 0, 0, 0, 0,
        )
        time.sleep(0.05)  # don't flood the slow link
    # Legacy fallback for older firmware
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        2,  # Hz
        1,  # start
    )


def handle_message(msg):
    t = msg.get_type()

    if t == "HEARTBEAT":
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            return
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"[HEARTBEAT] mode={mavutil.mode_string_v10(msg)} armed={armed} status={msg.system_status}")

    elif t == "RADIO_STATUS":
        # Sent by the SiK radio itself. rssi values are raw 0-255.
        print(
            f"[RADIO] rssi={msg.rssi} remrssi={msg.remrssi} "
            f"noise={msg.noise} remnoise={msg.remnoise} "
            f"txbuf={msg.txbuf}% rxerrors={msg.rxerrors} fixed={msg.fixed}"
        )

    elif t == "ATTITUDE":
        print(
            f"[ATTITUDE] roll={math.degrees(msg.roll):7.2f}  "
            f"pitch={math.degrees(msg.pitch):7.2f}  "
            f"yaw={math.degrees(msg.yaw):7.2f} deg"
        )

    elif t == "GLOBAL_POSITION_INT":
        print(
            f"[POSITION] lat={msg.lat / 1e7:.7f} lon={msg.lon / 1e7:.7f} "
            f"alt_msl={msg.alt / 1000:.1f} m rel_alt={msg.relative_alt / 1000:.1f} m "
            f"hdg={msg.hdg / 100:.1f}"
        )

    elif t == "GPS_RAW_INT":
        print(f"[GPS] fix={msg.fix_type} sats={msg.satellites_visible} hdop={msg.eph / 100:.2f}")

    elif t == "VFR_HUD":
        print(
            f"[VFR_HUD] airspeed={msg.airspeed:.1f} groundspeed={msg.groundspeed:.1f} "
            f"alt={msg.alt:.1f} climb={msg.climb:.2f} throttle={msg.throttle}%"
        )

    elif t == "SYS_STATUS":
        volts = msg.voltage_battery / 1000 if msg.voltage_battery != 65535 else float("nan")
        amps = msg.current_battery / 100 if msg.current_battery != -1 else float("nan")
        print(f"[BATTERY] {volts:.2f} V  {amps:.2f} A  remaining={msg.battery_remaining}%")

    elif t == "RC_CHANNELS":
        print(f"[RC] {[getattr(msg, f'chan{i}_raw') for i in range(1, 9)]}")

    elif t == "SERVO_OUTPUT_RAW":
        print(f"[SERVO] {[getattr(msg, f'servo{i}_raw') for i in range(1, 9)]}")

    elif t == "LOCAL_POSITION_NED":
        print(
            f"[LOCAL_POS] x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f} "
            f"vx={msg.vx:.2f} vy={msg.vy:.2f} vz={msg.vz:.2f}"
        )

    elif t == "STATUSTEXT":
        print(f"[STATUSTEXT] sev={msg.severity} {msg.text}")


def main():
    ap = argparse.ArgumentParser(description="Pixhawk MAVLink receiver (SiK radio)")
    ap.add_argument("--conn", default="/dev/ttyUSB0",
                    help="Serial port (/dev/ttyUSB0, COM5) or MAVLink URL (udpin:0.0.0.0:14550)")
    ap.add_argument("--baud", type=int, default=57600, help="Must match the SiK SERIAL_SPEED (57 = 57600)")
    ap.add_argument("--csv", help="Optional CSV file to log all received messages")
    ap.add_argument("--all", action="store_true", help="Print every message (raw) instead of the curated view")
    ap.add_argument("--extra", action="store_true", help="Also request servo outputs and local position")
    ap.add_argument("--source-system", type=int, default=255, help="MAVLink system ID for this script")
    ap.add_argument("--list-ports", action="store_true", help="List serial ports and exit")
    args = ap.parse_args()

    if args.list_ports:
        list_ports()
        return 0

    wanted = dict(BASE_MESSAGES)
    if args.extra:
        wanted.update(EXTRA_MESSAGES)

    master = connect(args.conn, args.baud, args.source_system)
    request_messages(master, wanted)

    csv_file = csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["timestamp", "msg_type", "data"])

    last_hb_sent = 0.0
    last_rx = time.time()
    last_request = time.time()
    last_stats = time.time()
    got_data_msg = False

    try:
        while True:
            now = time.time()

            # Our own 1 Hz GCS heartbeat (SiK/ArduPilot expect a GCS to be present)
            if now - last_hb_sent >= 1.0:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
                last_hb_sent = now

            # Radio links drop packets, so re-send the stream request if nothing
            # useful has arrived yet, and refresh it every 60 s afterwards.
            interval = 60 if got_data_msg else 5
            if now - last_request >= interval:
                request_messages(master, wanted)
                last_request = now

            # Link statistics every 10 s
            if now - last_stats >= 10:
                print(f"[LINK] packet loss ~{master.packet_loss():.1f}% "
                      f"({master.mav_loss} lost / {master.mav_count} received)")
                last_stats = now

            msg = master.recv_match(blocking=True, timeout=1)
            if msg is None:
                if time.time() - last_rx > 5:
                    print("No data for 5 s ... link lost? (check radio LEDs / range)")
                    last_rx = time.time()
                continue

            if msg.get_type() == "BAD_DATA":
                continue

            last_rx = time.time()
            if msg.get_type() in wanted:
                got_data_msg = True

            if csv_writer:
                csv_writer.writerow([time.time(), msg.get_type(), msg.to_dict()])

            if args.all:
                print(msg)
            else:
                handle_message(msg)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if csv_file:
            csv_file.close()
        master.close()


if __name__ == "__main__":
    sys.exit(main())