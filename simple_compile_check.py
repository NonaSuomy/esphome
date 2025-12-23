#!/usr/bin/env python3
"""Simple compilation checker that writes results to a file"""

import subprocess
import sys
import os
from pathlib import Path

config_file = sys.argv[1] if len(sys.argv) > 1 else "config/vector-eyes-ttgo.yaml"
output_file = "/tmp/compile_result.txt"

with open(output_file, 'w') as f:
    f.write(f"Checking compilation for: {config_file}\n")
    f.write("="*60 + "\n\n")
    
    # Check if firmware exists
    build_name = Path(config_file).stem
    firmware_paths = [
        f"config/.esphome/build/{build_name}/.pioenvs/{build_name}/firmware.elf",
        f".esphome/build/{build_name}/.pioenvs/{build_name}/firmware.elf",
    ]
    
    firmware_found = False
    for path in firmware_paths:
        if os.path.exists(path):
            size = os.path.getsize(path)
            f.write(f"✅ Firmware EXISTS: {path}\n")
            f.write(f"   Size: {size:,} bytes\n")
            f.write(f"   Modified: {os.path.getmtime(path)}\n")
            firmware_found = True
            break
    
    if not firmware_found:
        f.write("❌ Firmware NOT FOUND\n")
        f.write("Checked paths:\n")
        for path in firmware_paths:
            f.write(f"  - {path}\n")
    
    f.write("\n" + "="*60 + "\n")
    f.write(f"Result written to: {output_file}\n")

print(f"Results written to: {output_file}")
