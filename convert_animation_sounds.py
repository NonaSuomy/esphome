#!/usr/bin/env python3
"""
Script to find all WAV files referenced in generated_animations.h and convert them for SD card usage.
"""

import os
import re
import subprocess
import glob

# Configuration
ANIMATIONS_HEAER = "esphome/components/vector_eyes/generated_animations.h"
SOURCE_AUDIO_ROOT = "/home/nonasuomy/code/vector-audio-raw/VictorAudio/Originals/SFX"
OUTPUT_DIR = "sd_card_files"

# Mapping adjustments (Animation Sound Name -> Source Search Pattern)
# Some animation sounds might have prefixes/suffixes in source
NAME_MAPPINGS = {
    "happy.wav": ["Robot_Vic_Sfx__Scrn_Happy*"],
    "sad.wav": ["Robot_Vic_Sfx__Scrn_Sad*"],
    "surprised.wav": ["Robot_Vic_Sfx__Scrn_Surprised*"],
    "curious.wav": ["Robot_Vic_Sfx__Scrn_Curious*"],
    "curious_short.wav": ["Robot_Vic_Sfx__Scrn_Curious*Short*"],
    "blink.wav": ["Robot_Vic_Sfx__Scrn_Blink*"],
    "intro.wav": ["Robot_Vic_Sfx__Scrn_Intro*"],
    "zelda.wav": ["Robot_Vic_Sfx__Scrn_Happy*"],
    "angry.wav": ["Robot_Vic_Sfx__Scrn_Angry*"],
    "neutral.wav": ["Robot_Vic_Sfx__Scrn_Neutral*"],
    "neutral_short.wav": ["Robot_Vic_Sfx__Scrn_Neutral*"], # Fallback
    "happy_short.wav": ["Robot_Vic_Sfx__Scrn_Happy*Short*"],
    "head_curious.wav": ["Robot_Vic_Sfx__Head_Curious*", "Robot_Vic_Sfx__Scrn_Curious*"],
    "tread_curious.wav": ["Robot_Vic_Sfx__Drive_Curious*", "Robot_Vic_Sfx__Scrn_Curious*"],
    "emote_happy.wav": ["Robot_Vic_Sfx__Scrn_Happy*"],
    "emote_curious.wav": ["Robot_Vic_Sfx__Scrn_Curious*"],
    "snore.wav": ["Robot_Vic_Sfx__Scrn_Sleep_Snore*"],
}

def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def extract_sound_names(header_path):
    try:
        with open(header_path, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: Header file not found at {header_path}")
        return set()

    # Find strings ending in .wav inside quotes
    # Matches: "something.wav"
    regex = re.compile(r'"([^"]+\.wav)"')
    sounds = set()
    for match in regex.finditer(content):
        sounds.add(match.group(1))
    
    return sounds

def find_source_file(sound_name):
    # 1. Direct match check
    direct_path = os.path.join(SOURCE_AUDIO_ROOT, sound_name)
    if os.path.exists(direct_path):
        return direct_path

    # 2. Vector naming convention check
    # Vector sounds often look like: emote_happy.wav -> Robot_Vic_Sfx__Scrn_Happy_01_WM.wav
    # We strip 'emote_' or generic prefixes and search
    
    search_term = sound_name.replace(".wav", "").replace("emote_", "").replace("head_", "").replace("tread_", "")
    
    # Try finding files containing this term (case insensitive)
    # This is a bit loose but effective for this dataset
    
    pattern = os.path.join(SOURCE_AUDIO_ROOT, f"*{search_term}*")
    matches = glob.glob(pattern)
    
    # Filter for .wav
    matches = [m for m in matches if m.lower().endswith(".wav")]
    
    if matches:
        # Prefer "Scrn" (Screen) sounds if multiple
        scrn_matches = [m for m in matches if "Scrn" in m]
        if scrn_matches:
            return scrn_matches[0]
        return matches[0]
        
    return None

def convert_file(src_path, dst_name):
    dst_path = os.path.join(OUTPUT_DIR, dst_name)
    
    # Skip if exists? No, overwrite to be sure
    print(f"Converting {dst_name}...")
    try:
        subprocess.run([
            "ffmpeg", "-y", "-i", src_path,
            "-ar", "44100", "-ac", "1", "-c:a", "pcm_s16le",
            dst_path
        ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"  OK: {src_path} -> {dst_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"  FAILED: {e}")
        return False

def main():
    print(f"Scanning {ANIMATIONS_HEAER} for sounds...")
    sounds = extract_sound_names(ANIMATIONS_HEAER)
    print(f"Found {len(sounds)} unique sound references: {sounds}")
    
    ensure_dir(OUTPUT_DIR)
    
    success_count = 0
    missing = []
    
    for sound in sounds:
        # Check custom mapping first
        src_file = None
        if sound in NAME_MAPPINGS:
            # Handle special mappings (search pattern)
            for pattern in NAME_MAPPINGS[sound]:
                full_pattern = os.path.join(SOURCE_AUDIO_ROOT, pattern)
                matches = glob.glob(full_pattern)
                matches = [m for m in matches if m.lower().endswith(".wav")]
                if matches:
                    src_file = matches[0]
                    break
        
        if not src_file:
            src_file = find_source_file(sound)
            
        if src_file:
            if convert_file(src_file, sound):
                success_count += 1
        else:
            print(f"  MISSING: Could not find source for '{sound}'")
            missing.append(sound)
            
    print("-" * 30)
    print(f"Conversion complete.")
    print(f"Success: {success_count}")
    print(f"Missing: {len(missing)}")
    if missing:
        print("Missing files:", missing)

if __name__ == "__main__":
    main()
