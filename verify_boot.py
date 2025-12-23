
import serial
import time
import sys

def reset_and_capture():
    try:
        # Open serial port
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        print("Resetting device via DTR/RTS...")
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        time.sleep(0.1)
        
        print("Capturing logs (Ctrl+C to stop, timeout 15s)...")
        start_time = time.time()
        
        while time.time() - start_time < 15:
            if ser.in_waiting:
                 line = ser.readline().decode('utf-8', errors='replace').strip()
                 print(line)
                 if "SD Card initialized" in line or "[C][sd_storage:" in line:
                     print("FOUND SD STORAGE LOG!")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    reset_and_capture()
