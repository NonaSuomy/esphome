#!/usr/bin/env python3
"""
Script to convert WEM files directly to WAV using ffmpeg.
This bypasses wwtools and converts RIFF/WAVE Microsoft ADPCM WEM files directly.
"""

import os
import subprocess
from pathlib import Path

# Paths
INPUT_DIR = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/victor_robot/victor_linux"
OUTPUT_DIR = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/converted_wav"

def main():
    # Create output directory if it doesn't exist
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Find all WEM files
    wem_files = list(Path(INPUT_DIR).glob("*.wem"))
    
    print(f"Found {len(wem_files)} WEM files to convert")
    print(f"Output directory: {OUTPUT_DIR}")
    print()
    
    converted = 0
    skipped = 0
    failed = 0
    
    for wem_file in sorted(wem_files):
        wav_file = Path(OUTPUT_DIR) / f"{wem_file.stem}.wav"
        
        # Skip if already converted
        if wav_file.exists():
            skipped += 1
            continue
        
        try:
            # Try direct ffmpeg conversion with error suppression
            # Use -y to overwrite, -hide_banner and -loglevel error to suppress warnings
            result = subprocess.run(
                [
                    "ffmpeg",
                    "-hide_banner",
                    "-loglevel", "error",
                    "-i", str(wem_file),
                    "-ar", "16000",  # Resample to 16kHz
                    "-ac", "1",       # Convert to mono
                    "-y",             # Overwrite output files
                    str(wav_file)
                ],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            # Check if conversion was successful by checking if output file exists and has content
            if wav_file.exists() and wav_file.stat().st_size > 100:
                converted += 1
                if converted % 50 == 0:  # Progress indicator every 50 files
                    print(f"Converted {converted} files...")
            else:
                failed += 1
                if result.stderr:
                    print(f"✗ {wem_file.name}: {result.stderr.strip()[:100]}")
                
        except subprocess.TimeoutExpired:
            failed += 1
            print(f"✗ {wem_file.name} (timeout)")
        except Exception as e:
            failed += 1
            print(f"✗ {wem_file.name} (error: {e})")
    
    print()
    print("=" * 60)
    print(f"Conversion complete!")
    print(f"  Successfully converted: {converted}")
    print(f"  Skipped (already exist): {skipped}")
    print(f"  Failed: {failed}")
    print(f"  Total WEM files: {len(wem_files)}")
    print("=" * 60)

if __name__ == "__main__":
    main()
