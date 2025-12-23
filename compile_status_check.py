#!/usr/bin/env python3
"""Check compilation status and write results to workspace"""

import os
import sys
from pathlib import Path
from datetime import datetime

config_file = sys.argv[1] if len(sys.argv) > 1 else "config/vector-eyes-ttgo.yaml"
output_file = "compile_status.txt"

with open(output_file, 'w') as f:
    f.write(f"ESPHome Compilation Status Check\n")
    f.write(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write(f"Config: {config_file}\n")
    f.write("="*70 + "\n\n")
    
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
            mtime = datetime.fromtimestamp(os.path.getmtime(path))
            f.write(f"✅ FIRMWARE EXISTS\n")
            f.write(f"   Path: {path}\n")
            f.write(f"   Size: {size:,} bytes ({size/1024/1024:.2f} MB)\n")
            f.write(f"   Modified: {mtime.strftime('%Y-%m-%d %H:%M:%S')}\n")
            firmware_found = True
            
            # Check if it's recent (within last 10 minutes)
            age_seconds = (datetime.now() - mtime).total_seconds()
            if age_seconds < 600:
                f.write(f"   Age: {age_seconds:.0f} seconds (RECENT)\n")
            else:
                f.write(f"   Age: {age_seconds/60:.1f} minutes (may be old)\n")
            break
    
    if not firmware_found:
        f.write("❌ FIRMWARE NOT FOUND\n\n")
        f.write("Checked paths:\n")
        for path in firmware_paths:
            f.write(f"  - {path}\n")
        f.write("\nThis means compilation has not completed successfully.\n")
    
    f.write("\n" + "="*70 + "\n")
    
    # Check for build directory
    build_dirs = [
        f"config/.esphome/build/{build_name}",
        f".esphome/build/{build_name}",
    ]
    
    f.write("\nBuild Directory Status:\n")
    for build_dir in build_dirs:
        if os.path.exists(build_dir):
            f.write(f"  ✅ {build_dir} exists\n")
            # List some contents
            try:
                contents = os.listdir(build_dir)
                f.write(f"     Contents: {', '.join(contents[:10])}\n")
            except:
                pass
        else:
            f.write(f"  ❌ {build_dir} does not exist\n")

print(f"Status written to: {output_file}")
