#!/usr/bin/env python3
"""
Prepare SD card with Vector's original animation JSON files and audio mappings.

This script:
1. Copies Vector's original JSON animation files to SD card
2. Copies Vector's WAV audio files to SD card
3. Generates audio_mappings.json for the ESP32 to use

Usage:
    python prepare_sd_card.py --animations /path/to/animations_json --audio /path/to/vectorsounds --output /path/to/sdcard
"""

import argparse
import json
import os
import shutil
from pathlib import Path

try:
    from audio_mapper import AudioMapper
except ImportError:
    print("Warning: audio_mapper.py not found, will use basic mappings")
    AudioMapper = None


def copy_animations(source_dir, dest_dir):
    """Copy all JSON animation files to SD card."""
    source_path = Path(source_dir)
    dest_path = Path(dest_dir)
    
    if not source_path.exists():
        print(f"Error: Animation source directory not found: {source_dir}")
        return 0
    
    dest_path.mkdir(parents=True, exist_ok=True)
    
    json_files = list(source_path.glob("*.json"))
    print(f"\nCopying {len(json_files)} animation files...")
    
    copied = 0
    for json_file in json_files:
        dest_file = dest_path / json_file.name
        shutil.copy2(json_file, dest_file)
        copied += 1
        if copied % 100 == 0:
            print(f"  Copied {copied}/{len(json_files)} files...")
    
    print(f"✓ Copied {copied} animation JSON files")
    return copied


def copy_audio(source_dir, dest_dir, audio_mappings):
    """Copy only the WAV files that are referenced in audio mappings."""
    source_path = Path(source_dir)
    dest_path = Path(dest_dir)
    
    if not source_path.exists():
        print(f"Error: Audio source directory not found: {source_dir}")
        return 0
    
    dest_path.mkdir(parents=True, exist_ok=True)
    
    # Get list of WAV files referenced in mappings
    referenced_wavs = set()
    if "mappings" in audio_mappings:
        for wav_file in audio_mappings["mappings"].values():
            if wav_file and wav_file != "nullptr":
                # Add .wav extension if not present
                if not wav_file.endswith(".wav"):
                    wav_file += ".wav"
                referenced_wavs.add(wav_file)
    
    print(f"\nCopying {len(referenced_wavs)} referenced audio files...")
    
    copied = 0
    missing = []
    
    for wav_file in sorted(referenced_wavs):
        source_file = source_path / wav_file
        if source_file.exists():
            dest_file = dest_path / wav_file
            shutil.copy2(source_file, dest_file)
            copied += 1
        else:
            missing.append(wav_file)
    
    print(f"✓ Copied {copied} audio WAV files")
    
    if missing:
        print(f"⚠ Warning: {len(missing)} referenced WAV files not found:")
        for wav in missing[:10]:  # Show first 10
            print(f"    - {wav}")
        if len(missing) > 10:
            print(f"    ... and {len(missing) - 10} more")
    
    return copied


def generate_audio_mappings(output_dir, mapping_file=None):
    """Generate audio_mappings.json for ESP32."""
    dest_path = Path(output_dir)
    dest_path.mkdir(parents=True, exist_ok=True)
    
    # Load mappings from existing file or use AudioMapper
    if mapping_file and Path(mapping_file).exists():
        print(f"\nLoading audio mappings from {mapping_file}...")
        with open(mapping_file, 'r') as f:
            mappings = json.load(f)
    elif AudioMapper:
        print("\nGenerating audio mappings using AudioMapper...")
        mapper = AudioMapper("audio_mappings.json")
        # Export mappings
        mappings = {
            "mappings": mapper.mappings if hasattr(mapper, 'mappings') else {}
        }
    else:
        print("\nCreating basic audio mappings...")
        # Basic fallback mappings
        mappings = {
            "mappings": {
                "Play__Robot_Vic_Sfx__Blink": "blink.wav",
                "Play__Robot_Vic_Sfx__Happy": "happy.wav",
                "Play__Robot_Vic_Sfx__Sad": "sad.wav",
                "Play__Robot_Vic_Sfx__Curious": "curious.wav",
                "Play__Robot_Vic_Sfx__Neutral": "neutral.wav"
            }
        }
    
    # Write to SD card
    output_file = dest_path / "audio_mappings.json"
    with open(output_file, 'w') as f:
        json.dump(mappings, f, indent=2)
    
    mapping_count = len(mappings.get("mappings", {}))
    print(f"✓ Generated audio_mappings.json with {mapping_count} mappings")
    
    return mappings


def create_readme(output_dir, stats):
    """Create a README file on the SD card."""
    readme_path = Path(output_dir) / "README.txt"
    
    with open(readme_path, 'w') as f:
        f.write("Vector Eyes SD Card\n")
        f.write("=" * 50 + "\n\n")
        f.write("This SD card contains Vector's original animation and audio files.\n\n")
        f.write("Contents:\n")
        f.write(f"  - {stats['animations']} animation JSON files\n")
        f.write(f"  - {stats['audio']} audio WAV files\n")
        f.write(f"  - 1 audio_mappings.json file\n\n")
        f.write("File Structure:\n")
        f.write("  /anim_*.json          - Vector animation files\n")
        f.write("  /*.wav                - Audio files\n")
        f.write("  /audio_mappings.json  - Audio event to WAV file mappings\n\n")
        f.write("Generated by prepare_sd_card.py\n")
    
    print(f"✓ Created README.txt")


def main():
    parser = argparse.ArgumentParser(
        description="Prepare SD card with Vector animation and audio files"
    )
    parser.add_argument(
        "--animations",
        required=True,
        help="Path to Vector animations_json directory"
    )
    parser.add_argument(
        "--audio",
        required=True,
        help="Path to Vector audio files (vectorsounds directory)"
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Path to SD card mount point or output directory"
    )
    parser.add_argument(
        "--mappings",
        help="Path to existing audio_mappings.json (optional)"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("Vector Eyes SD Card Preparation")
    print("=" * 60)
    
    # Generate audio mappings first (needed to know which WAV files to copy)
    audio_mappings = generate_audio_mappings(args.output, args.mappings)
    
    # Copy files
    stats = {
        'animations': copy_animations(args.animations, args.output),
        'audio': copy_audio(args.audio, args.output, audio_mappings)
    }
    
    # Create README
    create_readme(args.output, stats)
    
    print("\n" + "=" * 60)
    print("SD Card Preparation Complete!")
    print("=" * 60)
    print(f"\nTotal files on SD card:")
    print(f"  - {stats['animations']} animations")
    print(f"  - {stats['audio']} audio files")
    print(f"  - 1 audio mappings file")
    print(f"  - 1 README file")
    print(f"\nTotal: {stats['animations'] + stats['audio'] + 2} files")
    print(f"\nSD card ready at: {args.output}")
    print("\nNext steps:")
    print("  1. Insert SD card into ESP32")
    print("  2. Flash the updated firmware")
    print("  3. Animations will load from SD card automatically")


if __name__ == "__main__":
    main()
