import serial
import time
import sys
import ctypes

port = 'COM5'
baud = 115200

print(f"Opening {port}...")
s = serial.Serial(port, baud, timeout=0.5, dsrdtr=False, rtscts=False)

# Clear buffer first
try:
    s.reset_input_buffer()
except:
    pass

print("Triggering reset (keeping port open)...")
try:
    s.setRTS(True)
    time.sleep(0.1)
    s.setRTS(False)
except:
    pass

print("Reading for 15 seconds... (output starts after ~300ms)")
print("=" * 60)

all_data = b''
start = time.time()
errors = 0

while time.time() - start < 15:
    try:
        chunk = s.read(256)
        if chunk:
            all_data += chunk
            errors = 0
    except serial.SerialException:
        errors += 1
        if errors > 3:
            print("[Too many serial errors - reconnecting...]")
            try: s.close()
            except: pass
            time.sleep(2)
            try:
                s = serial.Serial(port, baud, timeout=0.5, dsrdtr=False, rtscts=False)
                print("[Reconnected]")
                errors = 0
            except:
                print("[Cannot reconnect]")
                break
        else:
            time.sleep(0.1)
    except Exception as e:
        print(f"[Error: {e}]")
        break

text = all_data.decode('utf-8', errors='replace')
print(text)
print("=" * 60)
print(f"Total bytes received: {len(all_data)}")

try: s.close()
except: pass
