#!/usr/bin/env python3
"""
Script to convert all WEM files from Vector audio assets to WAV format.
Uses wwtools to extract WEM→OGG, then ffmpeg to convert OGG→WAV.
"""

import os
import subprocess
from pathlib import Path

# Paths
WWTOOLS_BIN = "/home/nonasuomy/code/esphome003/esphome/wwise-audio-tools/build/bin/wwtools"
INPUT_DIR = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/victor_robot/victor_linux"
OUTPUT_DIR = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/converted_wav"

def main():
    # Create output directory if it doesn't exist
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Find all WEM files in INPUT_DIR
    wem_files = list(Path(INPUT_DIR).glob("*.wem"))
    
    print(f"Found {len(wem_files)} WEM files to convert")
    print(f"Input directory: {INPUT_DIR}")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Using wwtools: {WWTOOLS_BIN}")
    print()
    
    converted_to_ogg = 0
    converted_to_wav = 0
    skipped = 0
    failed = 0
    
    for wem_file in sorted(wem_files):
        try:
            # Check if OGG already exists in INPUT_DIR
            ogg_file_in_input = Path(INPUT_DIR) / f"{wem_file.stem}.ogg"
            ogg_file_in_output = Path(OUTPUT_DIR) / f"{wem_file.stem}.ogg"
            wav_file = Path(OUTPUT_DIR) / f"{wem_file.stem}.wav"
            
            # Skip if WAV already exists
            if wav_file.exists():
                skipped += 1
                print(f"⊘ {wem_file.name} (WAV already exists)")
                continue
            
            # Step 1: Convert WEM to OGG using wwtools
            # wwtools outputs to INPUT_DIR by default when run from INPUT_DIR
            if not ogg_file_in_input.exists():
                result = subprocess.run(
                    [WWTOOLS_BIN, "wem", str(wem_file)],
                    cwd=INPUT_DIR,
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                
                if result.returncode != 0 or not ogg_file_in_input.exists():
                    failed += 1
                    print(f"✗ {wem_file.name} (WEM→OGG failed)")
                    if result.stderr:
                        print(f"  Error: {result.stderr.strip()}")
                    continue
                
                converted_to_ogg += 1
            else:
                print(f"  {wem_file.name} → OGG exists, using existing")
            
            # Step 2: Convert OGG to WAV using ffmpeg
            result = subprocess.run(
                ["ffmpeg", "-i", str(ogg_file_in_input), "-ar", "16000", "-ac", "1", str(wav_file)],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0 and wav_file.exists():
                converted_to_wav += 1
                print(f"✓ {wem_file.name} → {wav_file.name}")
            else:
                failed += 1
                print(f"✗ {wem_file.name} (OGG→WAV failed)")
                if result.stderr:
                    # ffmpeg stderr is verbose, only show last line
                    error_lines = result.stderr.strip().split('\n')
                    print(f"  Error: {error_lines[-1] if error_lines else 'Unknown'}")
                    
        except subprocess.TimeoutExpired:
            failed += 1
            print(f"✗ {wem_file.name} (timeout)")
        except Exception as e:
            failed += 1
            print(f"✗ {wem_file.name} (error: {e})")
    
    print()
    print("=" * 60)
    print(f"Conversion complete!")
    print(f"  WEM→OGG conversions: {converted_to_ogg}")
    print(f"  OGG→WAV conversions: {converted_to_wav}")
    print(f"  Skipped (already converted): {skipped}")
    print(f"  Failed: {failed}")
    print(f"  Total WEM files: {len(wem_files)}")
    print("=" * 60)

if __name__ == "__main__":
    main()
