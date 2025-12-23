#!/usr/bin/env python3
"""Analyze the final boot capture for SD card and animation messages."""

import re

def analyze_boot_log(filename='boot_capture_final.txt'):
    """Analyze boot log for key messages."""
    
    with open(filename, 'r', errors='replace') as f:
        lines = f.readlines()
    
    print(f"Total lines captured: {len(lines)}")
    print("=" * 60)
    print()
    
    # Search for key patterns
    patterns = {
        'Boot/Reset': r'(rst:|Booting|boot:)',
        'SD Card': r'(SD card|sd_storage)',
        'Storage': r'(storage_adapter|Storage Host)',
        'Animations': r'(animations|Found device|Cannot list)',
        'SPI Errors': r'(spi_master.*invalid dev handle)',
    }
    
    matches = {key: [] for key in patterns}
    
    for i, line in enumerate(lines, 1):
        for key, pattern in patterns.items():
            if re.search(pattern, line, re.IGNORECASE):
                matches[key].append((i, line.strip()))
    
    # Print results
    for key in ['Boot/Reset', 'SD Card', 'Storage', 'Animations']:
        print(f"\n{key} Messages ({len(matches[key])} found):")
        print("-" * 60)
        if matches[key]:
            for line_num, line in matches[key][:20]:  # Show first 20
                # Clean up ANSI codes for readability
                clean_line = re.sub(r'\x1b\[[0-9;]*m', '', line)
                print(f"  Line {line_num}: {clean_line[:100]}")
        else:
            print("  None found")
    
    # SPI error count
    spi_errors = len(matches['SPI Errors'])
    print(f"\n\nSPI Errors: {spi_errors} occurrences")
    print("=" * 60)
    
    # Check if we captured a full boot
    has_boot = len(matches['Boot/Reset']) > 0
    has_storage = len(matches['Storage']) > 0
    has_sd = len(matches['SD Card']) > 0
    
    print("\nCapture Status:")
    print(f"  Boot sequence: {'✓ YES' if has_boot else '✗ NO'}")
    print(f"  Storage setup: {'✓ YES' if has_storage else '✗ NO'}")
    print(f"  SD card mount: {'✓ YES' if has_sd else '✗ NO'}")
    
    return matches

if __name__ == '__main__':
    analyze_boot_log()
