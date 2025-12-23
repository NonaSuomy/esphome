#!/usr/bin/env python3
"""
Script to prepare WAV files for the Vector Eyes SD card.
Converts source WAVs to 44.1kHz, 16-bit, Mono, and saves them to 'sd_card_files/'.
"""

import os
import subprocess
import shutil

# Configuration
ORIGINAL_SFX_DIR = "/home/nonasuomy/code/vector-audio-raw/VictorAudio/Originals/SFX"
OUTPUT_DIR = "/home/nonasuomy/code/esphome003/esphome/sd_card_files"

# Sound mapping: (SD Filename, Original Filename)
# Note: SD card filenames should be short (8.3 format is safest, but ESP32 supports long names typically)
# We use the mapping from vector_eyes.cpp logic
SOUNDS = [
    ("intro.wav", "Robot_Vic_Sfx__Scrn_Intro_01_WM.wav"),
    ("blink.wav", "Robot_Vic_Sfx__Scrn_Blink_01_WM.wav"), # Using 01 as default blink
    ("blink_01.wav", "Robot_Vic_Sfx__Scrn_Blink_01_WM.wav"),
    ("blink_02.wav", "Robot_Vic_Sfx__Scrn_Blink_02_WM.wav"),
    ("blink_03.wav", "Robot_Vic_Sfx__Scrn_Blink_03_WM.wav"),
    ("happy.wav", "Robot_Vic_Sfx__Scrn_Happy_01_WM.wav"),
    ("happy_sh.wav", "Robot_Vic_Sfx__Scrn_Happy_Short_01_WM.wav"), # Shortened name
    ("sad.wav", "Robot_Vic_Sfx__Scrn_Sad_01_WM.wav"),
    ("angry.wav", "Robot_Vic_Sfx__Scrn_Angry_01_WM.wav"),
    ("curious.wav", "Robot_Vic_Sfx__Scrn_Curious_01_WM.wav"), # Using 01
    ("confused.wav", "Robot_Vic_Sfx__Scrn_Confused_01_WM.wav"),
    ("surprised.wav", "Robot_Vic_Sfx__Scrn_Surprised_01_WM.wav"),
    ("tread.wav", "Robot_Vic_Sfx__Drive_Loop_01.wav"), # Placeholder
    ("zelda.wav", "Robot_Vic_Sfx__Scrn_Happy_01_WM.wav"), # Placeholder for Zelda if original not found
    # Add real zelda if you have it, reusing Happy for now as per previous logic
]

def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def convert_file(src_name, dst_name):
    src_path = os.path.join(ORIGINAL_SFX_DIR, src_name)
    dst_path = os.path.join(OUTPUT_DIR, dst_name)
    
    # Check if we need to search locally if original dir is missing
    if not os.path.exists(src_path):
        print(f"Warning: Source not found: {src_path}")
        return

    print(f"Converting {src_name} -> {dst_name}...")
    
    try:
        # Convert to 44100Hz, 16-bit, Mono WAV
        subprocess.run([
            "ffmpeg", "-y", "-i", src_path,
            "-ar", "44100", "-ac", "1", "-c:a", "pcm_s16le",
            dst_path
        ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("  OK")
    except subprocess.CalledProcessError as e:
        print(f"  FAILED: {e}")

def main():
    print(f"Preparing SD card files in: {OUTPUT_DIR}")
    ensure_dir(OUTPUT_DIR)
    
    for dst_name, src_name in SOUNDS:
        convert_file(src_name, dst_name)
        
    print("\nDone! Copy the contents of 'sd_card_files/' to the root of your SD card.")

if __name__ == "__main__":
    main()
