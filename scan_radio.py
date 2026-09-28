import serial, time, sys

port = "/dev/ttyUSB0"
baud_rates = [115200, 57600, 9600, 38400, 19200]

print(f"Scanning {port} across standard baud rates for SiK radio...")

for baud in baud_rates:
    print(f"-> Testing {baud} baud...", end="", flush=True)
    try:
        s = serial.Serial(port, baud, timeout=1.5)
        time.sleep(1.2)
        s.reset_input_buffer()
        s.write(b"+++")
        time.sleep(1.2)
        resp = s.read(s.in_waiting or 100)
        
        if b"OK" in resp:
            print(" FOUND! (Radio responded OK)")
            print("\n" + "=" * 50)
            print(f"  RADIO IS AT: {baud} BAUD")
            print("=" * 50)
            
            # Read firmware and parameters
            s.write(b"ATI\r\n")
            time.sleep(0.3)
            print("Firmware:", s.read(s.in_waiting).decode('latin1', errors='ignore').strip())
            
            s.write(b"ATI5\r\n")
            time.sleep(0.5)
            params = s.read(s.in_waiting).decode('latin1', errors='ignore').strip()
            print("\nParameters:\n" + params)
            
            s.write(b"ATO\r\n")
            s.close()
            sys.exit(0)
        else:
            print(" No response")
        s.close()
    except Exception as e:
        print(f" Error opening {baud}: {e}")

print("\n[!] Could not connect at any standard baud rate.")
print("Check USB devices with: ls -l /dev/ttyUSB* or dmesg | tail -n 20")
