#!/usr/bin/env python3
"""
Extract all audio names from JSON animation files and find/convert corresponding WAV files.
"""

import os
import json
import subprocess
import re
from pathlib import Path

# Directories
JSON_DIR = "/home/nonasuomy/code/esphome003/esphome/esphome/components/vector_eyes/animations_json"
SOURCE_AUDIO_ROOT = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets"
OUTPUT_DIR = "/home/nonasuomy/code/esphome003/esphome/sd_card_audio"
SD_CARD = "/run/media/nonasuomy/TTGOCAM"

# Create output directory
os.makedirs(OUTPUT_DIR, exist_ok=True)

def extract_audio_names_from_json():
    """Extract all unique audio names from JSON animation files."""
    audio_names = set()
    
    for json_file in Path(JSON_DIR).glob("*.json"):
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            # Get the animation array (first key)
            for key, anim_array in data.items():
                if isinstance(anim_array, list):
                    for keyframe in anim_array:
                        if isinstance(keyframe, dict):
                            # Check for RobotAudioKeyFrame
                            if keyframe.get("Name") == "RobotAudioKeyFrame":
                                event_groups = keyframe.get("eventGroups", [])
                                for group in event_groups:
                                    audio_list = group.get("audioName", [])
                                    for audio_name in audio_list:
                                        if audio_name:
                                            audio_names.add(audio_name)
        except Exception as e:
            print(f"Error parsing {json_file}: {e}")
    
    return audio_names

def clean_audio_name(name):
    """Convert Wwise-style name to potential file patterns."""
    # Remove Play__ prefix
    name = re.sub(r'^Play__', '', name)
    # Remove Robot_Vic_ prefix
    name = re.sub(r'^Robot_Vic_', '', name)
    # Replace __ with _
    name = name.replace('__', '_')
    return name

def find_audio_file(audio_name, search_dirs):
    """Search for a matching audio file."""
    clean_name = clean_audio_name(audio_name)
    
    # Patterns to try
    patterns = [
        clean_name.lower() + ".wav",
        clean_name.lower() + ".ogg",
        clean_name + ".wav",
        clean_name + ".ogg",
        # Try extracting keywords
    ]
    
    # Also try to extract the key part
    # e.g., "Sfx__Scrn_Sad_Long" -> "sad_long" or "scrn_sad_long"
    key_parts = re.sub(r'^Sfx_+', '', clean_name, flags=re.IGNORECASE)
    patterns.extend([
        key_parts.lower() + ".wav",
        key_parts.lower() + ".ogg",
    ])
    
    for search_dir in search_dirs:
        if not os.path.exists(search_dir):
            continue
        for root, dirs, files in os.walk(search_dir):
            for f in files:
                f_lower = f.lower()
                for pattern in patterns:
                    if pattern.lower() in f_lower or f_lower in pattern.lower():
                        return os.path.join(root, f)
                # Also check if any part of the clean name is in the filename
                clean_lower = clean_name.lower().replace('_', '')
                f_normalized = f_lower.replace('_', '').replace('-', '')
                if clean_lower in f_normalized or f_normalized.startswith(clean_lower[:8]):
                    return os.path.join(root, f)
    
    return None

def convert_to_wav(input_path, output_path):
    """Convert audio file to 44.1kHz 16-bit mono WAV."""
    try:
        cmd = [
            'ffmpeg', '-y', '-i', input_path,
            '-ar', '44100', '-ac', '1', '-acodec', 'pcm_s16le',
            output_path
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode == 0
    except Exception as e:
        print(f"Conversion error: {e}")
        return False

def main():
    print("Extracting audio names from JSON files...")
    audio_names = extract_audio_names_from_json()
    print(f"Found {len(audio_names)} unique audio references")
    
    # Search directories
    search_dirs = [
        os.path.join(SOURCE_AUDIO_ROOT, "victor_robot"),
        os.path.join(SOURCE_AUDIO_ROOT, "External Sources"),
        SOURCE_AUDIO_ROOT,
        "/home/nonasuomy/code/vector-audio-raw/VictorAudio/Originals/SFX",
    ]
    
    found = []
    not_found = []
    
    print("\nSearching for audio files...")
    for audio_name in sorted(audio_names):
        source_file = find_audio_file(audio_name, search_dirs)
        if source_file:
            found.append((audio_name, source_file))
            print(f"  ✓ {audio_name}")
            print(f"    -> {source_file}")
        else:
            not_found.append(audio_name)
            print(f"  ✗ {audio_name} (not found)")
    
    print(f"\n=== Summary ===")
    print(f"Found: {len(found)}")
    print(f"Not found: {len(not_found)}")
    
    # Convert found files
    print(f"\nConverting {len(found)} audio files...")
    converted = 0
    for audio_name, source_file in found:
        # Create output filename based on audio name
        clean = clean_audio_name(audio_name)
        output_name = clean.lower() + ".wav"
        output_path = os.path.join(OUTPUT_DIR, output_name)
        
        if convert_to_wav(source_file, output_path):
            converted += 1
            print(f"  Converted: {output_name}")
        else:
            print(f"  Failed: {output_name}")
    
    print(f"\nConverted {converted} files to {OUTPUT_DIR}")
    
    # List not found for reference
    if not_found:
        print("\nNot found audio names (save for manual search):")
        with open(os.path.join(OUTPUT_DIR, "not_found.txt"), 'w') as f:
            for name in sorted(not_found):
                f.write(name + "\n")
                print(f"  {name}")
    
    # Copy to SD card if mounted
    if os.path.exists(SD_CARD):
        print(f"\nCopying to SD card: {SD_CARD}")
        for f in os.listdir(OUTPUT_DIR):
            if f.endswith('.wav'):
                src = os.path.join(OUTPUT_DIR, f)
                dst = os.path.join(SD_CARD, f)
                subprocess.run(['cp', src, dst])
        print("Done!")
    else:
        print(f"\nSD card not mounted at {SD_CARD}")
        print(f"Files are in {OUTPUT_DIR}")

if __name__ == "__main__":
    main()
