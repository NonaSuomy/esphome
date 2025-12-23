#!/usr/bin/env python3
"""Analyze serial boot log for issues"""

import re

def analyze():
    with open('serial_boot_log.txt', 'r') as f:
        lines = f.readlines()
    
    print("="*70)
    print("SERIAL LOG ANALYSIS")
    print("="*70)
    
    # Find key events
    storage_lines = [l for l in lines if 'storage' in l.lower() or 'sd' in l.lower()]
    spi_errors = [l for l in lines if 'invalid dev handle' in l or 'Transmit failed' in l]
    mount_lines = [l for l in lines if 'mount' in l.lower()]
    setup_lines = [l for l in lines if 'setup' in l.lower() or 'Setup' in l]
    
    print(f"\nTotal lines: {len(lines)}")
    print(f"Storage/SD related: {len(storage_lines)}")
    print(f"SPI errors: {len(spi_errors)}")
    print(f"Mount related: {len(mount_lines)}")
    print(f"Setup related: {len(setup_lines)}")
    
    if storage_lines:
        print("\n" + "="*70)
        print("STORAGE/SD EVENTS:")
        print("="*70)
        for line in storage_lines[:20]:
            print(line.rstrip())
    
    if mount_lines:
        print("\n" + "="*70)
        print("MOUNT EVENTS:")
        print("="*70)
        for line in mount_lines[:10]:
            print(line.rstrip())
    
    if setup_lines:
        print("\n" + "="*70)
        print("SETUP EVENTS:")
        print("="*70)
        for line in setup_lines[:15]:
            print(line.rstrip())
    
    # Look for the actual boot sequence
    boot_start = None
    for i, line in enumerate(lines):
        if 'Starting' in line or 'Booting' in line or 'rst:' in line:
            boot_start = i
            break
    
    if boot_start:
        print("\n" + "="*70)
        print("BOOT SEQUENCE (first 30 lines after boot):")
        print("="*70)
        for line in lines[boot_start:boot_start+30]:
            print(line.rstrip())
    
    # SPI error summary
    if spi_errors:
        print("\n" + "="*70)
        print(f"SPI ERRORS: {len(spi_errors)} total")
        print("="*70)
        print("Sample errors:")
        for line in spi_errors[:5]:
            print(line.rstrip())

if __name__ == "__main__":
    analyze()
