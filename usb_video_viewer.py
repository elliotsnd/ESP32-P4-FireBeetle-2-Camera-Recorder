#!/usr/bin/env python3
"""
USB Serial Video Viewer for ESP32-P4 Camera
Receives JPEG frames over serial and displays them using OpenCV

Usage: python usb_video_viewer.py COM5
       python usb_video_viewer.py /dev/ttyUSB0
"""

import sys
import serial
import cv2
import numpy as np
import time
from collections import deque

# Frame markers
FRAME_START = b'\xFF\xD8'  # JPEG SOI marker
FRAME_END = b'\xFF\xD9'    # JPEG EOI marker

def find_port():
    """Find ESP32-P4 serial port"""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'USB' in port.description or 'Serial' in port.description:
            print(f"Found: {port.device} - {port.description}")
    return None

def main():
    if len(sys.argv) < 2:
        print("Usage: python usb_video_viewer.py <COM_PORT>")
        print("\nAvailable ports:")
        find_port()
        sys.exit(1)
    
    port = sys.argv[1]
    baud = 921600  # High baud rate for video
    
    print(f"Connecting to {port} at {baud} baud...")
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f"Error opening port: {e}")
        sys.exit(1)
    
    print("Connected! Waiting for video frames...")
    print("Press 'q' to quit, 's' to save screenshot")
    
    # Create window
    cv2.namedWindow('ESP32-P4 Camera', cv2.WINDOW_NORMAL)
    
    buffer = bytearray()
    frame_count = 0
    fps_times = deque(maxlen=30)
    last_fps_print = time.time()
    
    try:
        while True:
            # Read data
            data = ser.read(4096)
            if data:
                buffer.extend(data)
            
            # Find complete JPEG frames
            while True:
                # Find start marker
                start_idx = buffer.find(FRAME_START)
                if start_idx == -1:
                    buffer.clear()
                    break
                
                # Remove data before start marker
                if start_idx > 0:
                    buffer = buffer[start_idx:]
                
                # Find end marker
                end_idx = buffer.find(FRAME_END)
                if end_idx == -1:
                    break  # Incomplete frame, wait for more data
                
                # Extract complete frame
                frame_data = bytes(buffer[:end_idx + 2])
                buffer = buffer[end_idx + 2:]
                
                # Decode JPEG
                try:
                    img_array = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        frame_count += 1
                        fps_times.append(time.time())
                        
                        # Calculate FPS
                        if len(fps_times) > 1:
                            fps = len(fps_times) / (fps_times[-1] - fps_times[0])
                        else:
                            fps = 0
                        
                        # Draw FPS on frame
                        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.putText(frame, f"Frame: {frame_count}", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        
                        # Display frame
                        cv2.imshow('ESP32-P4 Camera', frame)
                        
                        # Print FPS periodically
                        if time.time() - last_fps_print > 2:
                            print(f"FPS: {fps:.1f}, Frames: {frame_count}, Size: {len(frame_data)} bytes")
                            last_fps_print = time.time()
                    
                except Exception as e:
                    print(f"Decode error: {e}")
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"screenshot_{frame_count}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Saved: {filename}")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        ser.close()
        cv2.destroyAllWindows()
        print(f"Total frames received: {frame_count}")

if __name__ == "__main__":
    main()
