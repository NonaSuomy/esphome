#!/usr/bin/env python3
"""
Capture serial logs from ESP32 device after boot
"""

import subprocess
import os
import sys
import re
import time
import signal
from datetime import datetime

def capture_logs(config_file, device="/dev/ttyUSB0", duration=20):
    """Capture serial logs for specified duration"""
    
    print(f"📡 Capturing serial logs from {device} for {duration} seconds...")
    print(f"   Config: {config_file}")
    print(f"   Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    cmd = f"source ./venv/bin/activate && esphome logs {config_file} --device {device}"
    
    try:
        # Start the logs process
        process = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid  # Create new process group
        )
        
        # Collect output
        start_time = time.time()
        output_lines = []
        
        print("Collecting logs...")
        while time.time() - start_time < duration:
            line = process.stdout.readline()
            if line:
                output_lines.append(line.rstrip())
                # Print to console in real-time
                print(line.rstrip())
            else:
                if process.poll() is not None:
                    break
                time.sleep(0.1)
        
        # Terminate gracefully
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=2)
        except:
            process.kill()
        
        # Write to file
        log_file = "serial_boot_log.txt"
        with open(log_file, 'w') as f:
            f.write("="*70 + "\n")
            f.write(f"Serial Boot Log - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Device: {device}\n")
            f.write(f"Duration: {duration} seconds\n")
            f.write("="*70 + "\n\n")
            
            for line in output_lines:
                f.write(line + "\n")
        
        # Analyze for issues
        print(f"\n{'='*70}")
        print(f"✅ Captured {len(output_lines)} lines")
        print(f"📄 Saved to: {log_file}")
        
        # Look for specific issues
        issues = analyze_logs(output_lines)
        if issues:
            print(f"\n⚠️  Found {len(issues)} potential issues:")
            for issue_type, lines in issues.items():
                print(f"\n{issue_type} ({len(lines)} occurrences):")
                for line in lines[:3]:
                    print(f"  {line[:100]}")
                if len(lines) > 3:
                    print(f"  ... and {len(lines)-3} more")
        else:
            print("\n✅ No obvious errors detected")
        
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def analyze_logs(log_lines):
    """Analyze logs for common issues"""
    issues = {}
    
    patterns = {
        'SPI Errors': r'(invalid dev handle|Transmit failed|SPI device not ready)',
        'Storage Issues': r'(No storage device.*found|Preferred mount path not found|will fall back)',
        'Mount Issues': r'(mount.*failed|Mounting.*failed)',
        'Initialization Failures': r'initialization failed',
        'Error Logs': r'\[E\]',
        'Warning Logs': r'\[W\]',
    }
    
    for line in log_lines:
        for issue_type, pattern in patterns.items():
            if re.search(pattern, line, re.IGNORECASE):
                if issue_type not in issues:
                    issues[issue_type] = []
                issues[issue_type].append(line)
    
    return issues

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 capture_serial_logs.py <config_file> [duration_seconds]")
        sys.exit(1)
    
    config = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    
    capture_logs(config, duration=duration)
