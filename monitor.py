import serial
import time
import sys

def monitor_serial(port='COM5', baud=115200):
    print(f"Connecting to {port} at {baud} baud...")
    print("Press Ctrl+C to exit")
    print("-" * 50)
    
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                print(f"Connected! Press RESET on ESP32-P4...")
                time.sleep(0.5)
                ser.reset_input_buffer()
                
                while True:
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting)
                        try:
                            print(data.decode('utf-8', errors='ignore'), end='', flush=True)
                        except:
                            print(data, end='', flush=True)
                    time.sleep(0.01)
                    
        except serial.SerialException as e:
            print(f"\nSerial error: {e}")
            print("Retrying in 2 seconds...")
            time.sleep(2)
        except KeyboardInterrupt:
            print("\nExiting...")
            sys.exit(0)

if __name__ == '__main__':
    monitor_serial()
