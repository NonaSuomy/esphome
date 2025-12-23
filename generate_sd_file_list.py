#!/usr/bin/env python3
"""Generate complete SD card file listing"""

import os
from datetime import datetime
from pathlib import Path

SD_PATH = "/run/media/nonasuomy/TTGOCAM"
OUTPUT = "SD_CARD_COMPLETE_FILE_LIST.md"

def get_files(directory, extension="*"):
    """Get all files with extension in directory"""
    try:
        path = Path(directory)
        if extension == "*":
            return sorted([f.name for f in path.iterdir() if f.is_file()])
        else:
            return sorted([f.name for f in path.glob(f"*.{extension}")])
    except:
        return []

def main():
    print("Generating complete SD card file listing...")
    
    # Get file counts
    root_files = get_files(SD_PATH)
    json_files = get_files(f"{SD_PATH}/animations", "json")
    csv_files = get_files(f"{SD_PATH}/animations_csv", "csv")
    wav_files = get_files(f"{SD_PATH}/audio", "wav")
    
    with open(OUTPUT, 'w') as f:
        f.write("# SD Card Complete File Listing\n\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        
        # Summary table
        f.write("## Summary\n\n")
        f.write("| Directory | File Count | File Types |\n")
        f.write("|-----------|------------|------------|\n")
        f.write(f"| `/animations/` | {len(json_files)} | JSON animation files |\n")
        f.write(f"| `/animations_csv/` | {len(csv_files)} | CSV animation files (legacy) |\n")
        f.write(f"| `/audio/` | {len(wav_files)} | WAV audio files |\n")
        f.write(f"| Root | {len(root_files)} | Configuration files |\n")
        f.write(f"| **Total** | **{len(json_files) + len(csv_files) + len(wav_files) + len(root_files)}** | All files |\n\n")
        
        # Root directory
        f.write("---\n\n")
        f.write("## Root Directory\n\n")
        f.write("```\n")
        for file in root_files:
            f.write(f"{file}\n")
        f.write("```\n\n")
        
        # JSON animations
        f.write("---\n\n")
        f.write(f"## /animations/ Directory ({len(json_files)} JSON files)\n\n")
        f.write("```\n")
        for file in json_files:
            f.write(f"{file}\n")
        f.write("```\n\n")
        
        # CSV animations
        f.write("---\n\n")
        f.write(f"## /animations_csv/ Directory ({len(csv_files)} CSV files)\n\n")
        f.write("```\n")
        for file in csv_files:
            f.write(f"{file}\n")
        f.write("```\n\n")
        
        # Audio files
        f.write("---\n\n")
        f.write(f"## /audio/ Directory ({len(wav_files)} WAV files)\n\n")
        f.write("```\n")
        for file in wav_files:
            f.write(f"{file}\n")
        f.write("```\n\n")
    
    # Print stats
    total_lines = sum(1 for _ in open(OUTPUT))
    file_size = os.path.getsize(OUTPUT)
    
    print(f"\n✅ Complete file listing generated: {OUTPUT}")
    print(f"   Lines: {total_lines:,}")
    print(f"   Size: {file_size:,} bytes ({file_size/1024:.1f} KB)")
    print(f"\n   JSON animations: {len(json_files)}")
    print(f"   CSV animations: {len(csv_files)}")
    print(f"   Audio files: {len(wav_files)}")
    print(f"   Root files: {len(root_files)}")
    print(f"   Total files: {len(json_files) + len(csv_files) + len(wav_files) + len(root_files)}")

if __name__ == "__main__":
    main()
