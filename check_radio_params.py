#!/usr/bin/env python3
"""
check_radio_params.py
---------------------
Queries the SiK / YRRC 433 MHz radio via AT commands to read its firmware
version, Net ID, Air Speed, Serial Baud Rate, and Frequency settings.

Usage:
    python3 check_radio_params.py [--port /dev/ttyUSB0] [--baud 57600]
"""

import sys
import time
import argparse

try:
    import serial
except ImportError:
    print("[ERROR] 'pyserial' is not installed. Run: pip3 install pyserial")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Check SiK / YRRC Radio Parameters")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port of the radio")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate (default: 57600)")
    args = parser.parse_args()

    print("=" * 60)
    print("       CHECKING 433 MHz RADIO PARAMETERS (AT COMMANDS)")
    print("=" * 60)
    print(f" Port      : {args.port}")
    print(f" Baud Rate : {args.baud}")
    print("-" * 60)

    try:
        s = serial.Serial(args.port, args.baud, timeout=1.5)
    except Exception as e:
        print(f"[ERROR] Cannot open {args.port}: {e}")
        print("Tip: Run 'sudo chmod 666 " + args.port + "' or make sure no other program is using it.")
        sys.exit(1)

    print("[1/3] Entering AT command mode (waiting guard time)...")
    time.sleep(1.2)
    s.write(b"+++")
    time.sleep(1.2)

    resp = s.read(s.in_waiting or 100)
    if b"OK" not in resp:
        print(f"[FAILED] Radio did not respond with 'OK' (Got: {resp}).")
        print("\nPossible reasons:")
        print(" 1. The radio might be configured for a different baud rate (e.g. 115200 or 9600).")
        print(" 2. The radio might be in bootloader mode or not a SiK-compatible radio.")
        s.close()
        sys.exit(1)

    print("[2/3] Reading radio firmware and parameters...\n")

    # Read Version
    s.write(b"ATI\r\n")
    time.sleep(0.3)
    version = s.read(s.in_waiting).decode("latin1", errors="ignore").strip()
    print(f" Firmware Version :\n {version}\n")

    # Read EEPROM Parameters
    s.write(b"ATI5\r\n")
    time.sleep(0.5)
    params = s.read(s.in_waiting).decode("latin1", errors="ignore").strip()
    print(f" Radio Parameters (ATI5):\n{params}\n")

    # Exit AT mode
    s.write(b"ATO\r\n")
    time.sleep(0.2)
    s.close()
    print("=" * 60)
    print("[OK] Radio query completed. Radio returned to normal data mode.")


if __name__ == "__main__":
    main()
