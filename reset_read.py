import serial
import time
import sys

port = 'COM5'
baud = 115200

print(f"Opening {port}...")
try:
    s = serial.Serial(port, baud, timeout=2, dsrdtr=False, rtscts=False)
except Exception as e:
    print(f"Cannot open port: {e}")
    sys.exit(1)

# Trigger hardware reset via RTS
print("Triggering reset...")
s.setDTR(False)
s.setRTS(True)
time.sleep(0.2)
s.setRTS(False)
time.sleep(0.1)

# Close and reopen to clear any USB errors
s.close()
time.sleep(1.5)

print("Reopening port...")
try:
    s = serial.Serial(port, baud, timeout=2, dsrdtr=False, rtscts=False)
except Exception as e:
    print(f"Cannot reopen: {e}")
    # Wait and retry
    time.sleep(2)
    try:
        s = serial.Serial(port, baud, timeout=2, dsrdtr=False, rtscts=False)
    except Exception as e2:
        print(f"Still cannot open: {e2}")
        sys.exit(1)

read_time = 120
print(f"Reading output ({read_time} seconds)...")
print("=" * 60)

start = time.time()
while time.time() - start < read_time:
    try:
        line = s.readline()
        if line:
            text = line.decode('utf-8', errors='replace').rstrip('\r\n')
            print(text)
    except serial.SerialException as e:
        err = str(e)
        if "ClearCommError" in err:
            # USB device reset - wait and reconnect
            print(f"\n[USB reset detected, reconnecting...]")
            try:
                s.close()
            except:
                pass
            time.sleep(2)
            try:
                s = serial.Serial(port, baud, timeout=2, dsrdtr=False, rtscts=False)
                print("[Reconnected]")
            except:
                print("[Cannot reconnect, waiting...]")
                time.sleep(3)
                try:
                    s = serial.Serial(port, baud, timeout=2, dsrdtr=False, rtscts=False)
                    print("[Reconnected on retry]")
                except:
                    print("[Failed to reconnect]")
                    break
        else:
            print(f"[Serial error: {e}]")
            break

print("=" * 60)
print("Done.")
try:
    s.close()
except:
    pass
