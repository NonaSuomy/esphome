#!/usr/bin/env python3
"""Capture ESP32 boot logs including reset sequence."""

import serial
import time
import sys

def capture_logs(port='/dev/ttyUSB0', baudrate=115200, duration=45):
    """Capture serial logs for specified duration."""
    print(f"Opening {port} at {baudrate} baud...")
    print(f"Will capture for {duration} seconds")
    print("Reset your device NOW!")
    print("-" * 60)
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.5)  # Let port stabilize
        
        start_time = time.time()
        lines = []
        
        while (time.time() - start_time) < duration:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='replace')
                    print(line, end='')
                    lines.append(line)
                except Exception as e:
                    print(f"Error reading line: {e}", file=sys.stderr)
            else:
                time.sleep(0.01)
        
        ser.close()
        
        # Save to file
        with open('boot_capture3.txt', 'w') as f:
            f.writelines(lines)
        
        print("-" * 60)
        print(f"\nCaptured {len(lines)} lines to boot_capture3.txt")
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    
    return 0

if __name__ == '__main__':
    sys.exit(capture_logs())
