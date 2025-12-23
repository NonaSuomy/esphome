#!/usr/bin/env python3
"""Analyze ESP32 logs for SD card and storage activity"""

import re
import sys
from collections import defaultdict

def analyze_logs(filename):
    print(f"Analyzing {filename}...")
    print("=" * 80)
    
    with open(filename, 'r', errors='ignore') as f:
        lines = f.readlines()
    
    print(f"\nTotal lines: {len(lines)}")
    
    # Categories
    storage_msgs = []
    sd_msgs = []
    file_ops = []
    vector_eyes_msgs = []
    errors = []
    warnings = []
    spi_errors = []
    boot_msgs = []
    
    for line in lines:
        line_lower = line.lower()
        
        # Storage related
        if any(x in line_lower for x in ['storage', 'sd_card', 'sd_spi', 'mount']):
            storage_msgs.append(line.strip())
        
        # SD card
        if 'sd' in line_lower and 'card' in line_lower:
            sd_msgs.append(line.strip())
        
        # File operations
        if any(x in line_lower for x in ['file_exists', 'read_file', 'open_file', 'close_file', '.json', '.wav']):
            file_ops.append(line.strip())
        
        # Vector eyes
        if 'vector' in line_lower or 'eyes' in line_lower:
            vector_eyes_msgs.append(line.strip())
        
        # Errors
        if '[E]' in line or 'error' in line_lower:
            errors.append(line.strip())
        
        # Warnings
        if '[W]' in line or 'warn' in line_lower:
            warnings.append(line.strip())
        
        # SPI errors
        if 'spi' in line_lower and ('error' in line_lower or 'fail' in line_lower or 'invalid' in line_lower):
            spi_errors.append(line.strip())
        
        # Boot messages
        if any(x in line_lower for x in ['boot', 'setup', 'starting', 'initializ']):
            boot_msgs.append(line.strip())
    
    # Print results
    print("\n" + "=" * 80)
    print("BOOT & INITIALIZATION MESSAGES")
    print("=" * 80)
    if boot_msgs:
        for msg in boot_msgs[:20]:
            print(msg)
    else:
        print("No boot messages found")
    
    print("\n" + "=" * 80)
    print("STORAGE & SD CARD MESSAGES")
    print("=" * 80)
    if storage_msgs or sd_msgs:
        for msg in (storage_msgs + sd_msgs)[:30]:
            print(msg)
    else:
        print("❌ NO STORAGE MESSAGES FOUND - Storage component may not be initializing!")
    
    print("\n" + "=" * 80)
    print("FILE OPERATIONS")
    print("=" * 80)
    if file_ops:
        for msg in file_ops[:30]:
            print(msg)
    else:
        print("❌ NO FILE OPERATIONS FOUND - SD card not being accessed!")
    
    print("\n" + "=" * 80)
    print("VECTOR EYES COMPONENT")
    print("=" * 80)
    if vector_eyes_msgs:
        for msg in vector_eyes_msgs[:30]:
            print(msg)
    else:
        print("❌ NO VECTOR EYES MESSAGES FOUND")
    
    print("\n" + "=" * 80)
    print("SPI ERRORS (sample)")
    print("=" * 80)
    if spi_errors:
        print(f"Total SPI errors: {len(spi_errors)}")
        print("\nFirst 10 SPI errors:")
        for msg in spi_errors[:10]:
            print(msg)
        print("\n⚠️  CRITICAL: Massive SPI errors detected!")
        print("This suggests SPI bus configuration issues with display or SD card")
    else:
        print("No SPI errors")
    
    print("\n" + "=" * 80)
    print("OTHER ERRORS (non-SPI)")
    print("=" * 80)
    non_spi_errors = [e for e in errors if e not in spi_errors]
    if non_spi_errors:
        for msg in non_spi_errors[:20]:
            print(msg)
    else:
        print("No other errors found")
    
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print(f"Total lines: {len(lines)}")
    print(f"Storage messages: {len(storage_msgs + sd_msgs)}")
    print(f"File operations: {len(file_ops)}")
    print(f"Vector eyes messages: {len(vector_eyes_msgs)}")
    print(f"SPI errors: {len(spi_errors)}")
    print(f"Other errors: {len([e for e in errors if e not in spi_errors])}")
    print(f"Warnings: {len(warnings)}")
    
    print("\n" + "=" * 80)
    print("DIAGNOSIS")
    print("=" * 80)
    
    if len(spi_errors) > 100:
        print("❌ CRITICAL: Severe SPI bus errors detected")
        print("   - Display or SD card SPI configuration is incorrect")
        print("   - Device may be in a boot loop or crash state")
        print("   - Recommendation: Check SPI pin assignments and bus configuration")
    
    if not storage_msgs and not sd_msgs:
        print("❌ PROBLEM: No storage component initialization detected")
        print("   - Storage component may not be starting")
        print("   - SD card may not be detected")
        print("   - Check YAML configuration for storage/sd_storage components")
    
    if not file_ops:
        print("❌ PROBLEM: No file operations detected")
        print("   - SD card is not being accessed")
        print("   - Files are not being read from SD card")
    
    if not vector_eyes_msgs:
        print("❌ PROBLEM: No vector_eyes component messages")
        print("   - Component may not be initializing")
        print("   - May be crashing during setup")
    
    print("\n" + "=" * 80)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        analyze_logs(sys.argv[1])
    else:
        analyze_logs("esp32_logs.txt")
