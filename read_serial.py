import serial
import time
import sys

port = 'COM5'
baud = 115200

print(f"Opening {port}...")
try:
    s = serial.Serial(port, baud, timeout=2)
except Exception as e:
    print(f"Cannot open port: {e}")
    sys.exit(1)

print("Connected. Reading output (15 seconds)...")
print("=" * 60)

start = time.time()
while time.time() - start < 15:
    try:
        line = s.readline()
        if line:
            text = line.decode('utf-8', errors='ignore').rstrip('\r\n')
            print(text)
    except serial.SerialException as e:
        print(f"\n[Serial error: {e}]")
        print("[Port may have disconnected - ESP32-P4 may be crashing]")
        s.close()
        time.sleep(2)
        try:
            s = serial.Serial(port, baud, timeout=2)
            print("[Reconnected]")
        except:
            print("[Cannot reconnect]")
            break
    except Exception as e:
        print(f"\n[Error: {e}]")
        break

print("=" * 60)
print("Done.")
try:
    s.close()
except:
    pass
