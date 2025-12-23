#!/usr/bin/env python3
"""
Script to map converted WEM file IDs to their human-readable sound names from Victor metadata.
"""

import csv
from pathlib import Path

# Paths
VICTOR_SFX_FILE = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/metadata/Victor_Linux/Victor_SFX.txt"
CONVERTED_FILES = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/converted_wav/converted_files.txt"
OUTPUT_FILE = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/converted_wav/sound_mappings.txt"

def load_victor_sfx_mapping():
    """Load the Victor SFX mapping from the metadata file."""
    id_to_name = {}
    
    with open(VICTOR_SFX_FILE, 'r', encoding='utf-8-sig') as f:
        # Skip to Event section (starts at line 2)
        for line in f:
            # Remove BOM and carriage returns
            line = line.strip().replace('\r', '')
            
            # Skip empty lines and section headers
            if not line or line.startswith('Event\t') or line.startswith('Switch') or line.startswith('State') or line.startswith('Game') or line.startswith('Source') or line.startswith('In Memory'):
                continue
            
            # Parse tab-separated values
            parts = line.split('\t')
            if len(parts) >= 3:
                # Column 1 is ID, Column 2 is Name
                try:
                    sound_id = parts[0].strip()
                    sound_name = parts[1].strip()
                    
                    if sound_id and sound_name and sound_id.isdigit():
                        id_to_name[sound_id] = sound_name
                except (ValueError, IndexError):
                    continue
    
    return id_to_name

def main():
    # Load the Victor SFX mapping
    id_to_name = load_victor_sfx_mapping()
    print(f"Loaded {len(id_to_name)} sound definitions from Victor metadata")
    
    # Read converted file IDs
    with open(CONVERTED_FILES, 'r') as f:
        converted_ids = [line.strip() for line in f if line.strip()]
    
    print(f"Found {len(converted_ids)} converted WAV files")
    
    # Create mapping output
    mapped = []
    unmapped = []
    
    for file_id in converted_ids:
        if file_id in id_to_name:
            sound_name = id_to_name[file_id]
            mapped.append((file_id, sound_name))
        else:
            unmapped.append(file_id)
    
    #  Write output
    with open(OUTPUT_FILE, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write(f"Converted Sound Mappings ({len(mapped)} files)\n")
        f.write("=" * 80 + "\n\n")
        
        for file_id, sound_name in sorted(mapped, key=lambda x: x[1]):
            f.write(f"{file_id:12} | {sound_name}\n")
        
        if unmapped:
            f.write("\n" + "=" * 80 + "\n")
            f.write(f"Unmapped Files ({len(unmapped)} files)\n")
            f.write("=" * 80 + "\n\n")
            for file_id in sorted(unmapped):
                f.write(f"{file_id}\n")
    
    print(f"\nMapping complete!")
    print(f"  Mapped sounds: {len(mapped)}")
    print(f"  Unmapped files: {len(unmapped)}")
    print(f"  Output saved to: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
