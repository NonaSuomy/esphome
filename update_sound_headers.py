#!/usr/bin/env python3
"""
Script to update the C++ header files for Vector Eyes sounds.
Reads selected WAV files and writes them as C arrays to .h files.
"""

import os

# Configuration
ORIGINAL_SFX_DIR = "/home/nonasuomy/code/vector-audio-raw/VictorAudio/Originals/SFX"
OUTPUT_DIR = "/home/nonasuomy/code/esphome003/esphome/esphome/components/vector_eyes"

# Sound mapping: (Header Filename, Variable Name Base, WAV Filename)
SOUNDS = [
    # Blink variants (3 variants)
    ("blink_01.h", "config_sounds_blink_01_wav", "Robot_Vic_Sfx__Scrn_Blink_01_WM.wav"),
    ("blink_02.h", "config_sounds_blink_02_wav", "Robot_Vic_Sfx__Scrn_Blink_02_WM.wav"),
    ("blink_03.h", "config_sounds_blink_03_wav", "Robot_Vic_Sfx__Scrn_Blink_03_WM.wav"),
    
    # Happy
    ("happy.h", "config_sounds_happy_wav", "Robot_Vic_Sfx__Scrn_Happy_01_WM.wav"),
    
    # Happy Short variants (3 variants)
    ("happy_short_01.h", "config_sounds_happy_short_01_wav", "Robot_Vic_Sfx__Scrn_Happy_Short_01_WM.wav"),
    ("happy_short_02.h", "config_sounds_happy_short_02_wav", "Robot_Vic_Sfx__Scrn_Happy_Short_02_WM.wav"),
    ("happy_short_03.h", "config_sounds_happy_short_03_wav", "Robot_Vic_Sfx__Scrn_Happy_Short_03_WM.wav"),
    
    # Curious variants (4 variants)
    ("curious_01.h", "config_sounds_curious_01_wav", "Robot_Vic_Sfx__Scrn_Curious_01_WM.wav"),
    ("curious_02.h", "config_sounds_curious_02_wav", "Robot_Vic_Sfx__Scrn_Curious_02_WM.wav"),
    ("curious_04.h", "config_sounds_curious_04_wav", "Robot_Vic_Sfx__Scrn_Curious_04_WM.wav"),
    
    # Curious Short variants (4 variants)
    ("curious_short_01.h", "config_sounds_curious_short_01_wav", "Robot_Vic_Sfx__Scrn_Curious_Short_01_WM.wav"),
    ("curious_short_02.h", "config_sounds_curious_short_02_wav", "Robot_Vic_Sfx__Scrn_Curious_Short_02_WM.wav"),
    ("curious_short_03.h", "config_sounds_curious_short_03_wav", "Robot_Vic_Sfx__Scrn_Curious_Short_03_WM.wav"),
    ("curious_short_04.h", "config_sounds_curious_short_04_wav", "Robot_Vic_Sfx__Scrn_Curious_Short_04_WM.wav"),
    
    # Neutral variants (3 variants)
    ("neutral_01.h", "config_sounds_neutral_01_wav", "Robot_Vic_Sfx__Scrn_Neutral_Short_01_WM.wav"),
    ("neutral_02.h", "config_sounds_neutral_02_wav", "Robot_Vic_Sfx__Scrn_Neutral_Short_02_WM.wav"),
    ("neutral_03.h", "config_sounds_neutral_03_wav", "Robot_Vic_Sfx__Scrn_Neutral_Short_03_WM.wav"),
    
    # Sad
    ("sad.h", "config_sounds_sad_wav", "Robot_Vic_Sfx__Scrn_Sad_01_WM.wav"),
    
    # Angry
    ("angry.h", "config_sounds_angry_wav", "Robot_Vic_Sfx__Scrn_Angry_01_WM.wav"),
    
    # Zelda (uses Happy sound as placeholder)
    ("zelda.h", "config_sounds_zelda_wav", "Robot_Vic_Sfx__Scrn_Happy_01_WM.wav"),
]

import subprocess
import wave

def convert_and_get_raw_data(src_path):
    tmp_path = src_path + ".tmp.wav"
    try:
        # Convert to 44100Hz, 16-bit, Mono, RAW PCM (s16le) using ffmpeg
        # We output raw data directly to avoid WAV header parsing issues
        subprocess.run([
            "ffmpeg", "-y", "-i", src_path,
            "-ar", "44100", "-ac", "1", "-f", "s16le", "-acodec", "pcm_s16le",
            tmp_path
        ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        with open(tmp_path, "rb") as f:
            data = f.read()
            
        # Verify data is not all zeros
        non_zero_count = sum(1 for b in data if b != 0)
        if non_zero_count == 0:
            print(f"WARNING: {src_path} produced all-zero data!")
        else:
            print(f"  -> Raw PCM size: {len(data)} bytes (Non-zero: {non_zero_count})")
            
        return data
    except Exception as e:
        print(f"Error converting {src_path}: {e}")
        return None
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)

def write_header(header_path, var_name, wav_path):
    try:
        print(f"Processing {wav_path}...")
        data = convert_and_get_raw_data(wav_path)
        
        if data is None:
            return

        print(f"  -> Raw PCM size: {len(data)} bytes")
        
        with open(header_path, "w") as f:
            f.write(f"const unsigned char {var_name}[] = {{\n")
            
            # Write data in hex format, 16 bytes per line
            for i in range(0, len(data), 16):
                chunk = data[i:i+16]
                hex_str = ", ".join(f"0x{b:02x}" for b in chunk)
                f.write(f"    {hex_str}")
                if i + 16 < len(data):
                    f.write(",")
                f.write("\n")
            
            f.write("};\n")
            f.write(f"unsigned int {var_name}_len = {len(data)};\n")
            
        print(f"Wrote {header_path}")
        
    except Exception as e:
        print(f"Error processing {wav_path}: {e}")

def main():
    print("Updating Vector Eyes sound headers...")
    
    for header_file, var_base, wav_filename in SOUNDS:
        wav_path = os.path.join(ORIGINAL_SFX_DIR, wav_filename)
        header_path = os.path.join(OUTPUT_DIR, header_file)
        
        if not os.path.exists(wav_path):
            print(f"Error: WAV file not found: {wav_path}")
            continue
            
        write_header(header_path, var_base, wav_path)
        
    print("Done!")

if __name__ == "__main__":
    main()
