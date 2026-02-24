"""Flash and immediately monitor ESP32-P4 output"""
import subprocess
import serial
import time
import sys
import os

port = 'COM5'
baud = 115200
bin_dir = r"G:\_Organized_Loose_Files\esp32p4_hardware_test"

# Flash using esptool
print("=== Flashing camera_test.bin ===")
flash_cmd = [
    sys.executable, "-m", "esptool",
    "--chip", "esp32p4", "-p", port, "-b", "460800",
    "--after", "no_reset",  # Don't reset after flash!
    "write_flash", "--flash_mode", "dio", "--flash_freq", "80m",
    "--flash_size", "2MB",
    "0x10000", os.path.join(bin_dir, "camera_test.bin")
]

result = subprocess.run(flash_cmd, capture_output=True, text=True)
print(result.stdout[-500:] if len(result.stdout) > 500 else result.stdout)
if result.returncode != 0:
    print(f"Flash failed: {result.stderr[-300:]}")
    sys.exit(1)

# Now open serial and THEN trigger reset
print("\n=== Opening serial port ===")
try:
    s = serial.Serial(port, baud, timeout=2, dsrdtr=False, rtscts=False)
except Exception as e:
    print(f"Cannot open port: {e}")
    time.sleep(1)
    s = serial.Serial(port, baud, timeout=2, dsrdtr=False, rtscts=False)

print("Port open. Triggering reset...")
s.setRTS(True)
time.sleep(0.1)
s.setRTS(False)

print("Reading output for 20 seconds...")
print("=" * 60)

start = time.time()
while time.time() - start < 20:
    try:
        line = s.readline()
        if line:
            text = line.decode('utf-8', errors='replace').rstrip('\r\n')
            print(text)
    except serial.SerialException as e:
        if "ClearCommError" in str(e):
            # Brief USB disconnect during chip reset - retry
            time.sleep(0.05)
            continue
        print(f"[Error: {e}]")
        break
    except Exception as e:
        print(f"[Error: {e}]")
        break

print("=" * 60)
try:
    s.close()
except:
    pass
