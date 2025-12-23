#!/usr/bin/env python3
"""
Generate C++ headers for priority animation audio files
"""
import os
import struct

WAV_DIR = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/converted_wav"
OUTPUT_DIR = "."

# Priority sounds to embed (ID, variable_name, usage_count)
PRIORITY_SOUNDS = [
    # Screen sounds (highest priority - already have some)
    ("256459301", "blink", 38),  # Already embedded
    ("319435599", "curious", 70),  # Already embedded (Scrn_Curious_Long)
    ("1052205531", "happy", 11),  # Already embedded (Scrn_Happy_Long)
    
    # New ones to add
    ("247835548", "neutral", 16),  # Scrn_Neutral_01
    ("60087899", "happy_short", 6),  # Scrn_Happy_Short_01
    ("157638400", "scrn_happy_01", 11),  # Scrn_Happy_01 (alternate)
    
    # More emotions
    ("766188798", "neutral_short", 9),  # Scrn_Neutral_Short_01
    # TODO: Find Scrn_Surprised, Scrn_Angry, Scrn_Sad
]

def generate_header(wav_id, var_name):
    """Generate a C++ header file from a WAV file"""
    wav_path = os.path.join(WAV_DIR, f"{wav_id}.wav")
    
    if not os.path.exists(wav_path):
        print(f"WARNING: {wav_path} not found!")
        return False
    
    with open(wav_path, 'rb') as f:
        wav_data = f.read()
    
    # Verify it's a valid WAV
    if not wav_data.startswith(b'RIFF'):
        print(f"WARNING: {wav_id}.wav is not a valid WAV file!")
        return False
    
    output_path = os.path.join(OUTPUT_DIR, f"{var_name}.h")
    
    # Check if already exists and skip
    if var_name in ["blink", "curious", "happy"]:
        print(f"Skipping {var_name}.h (already exists)")
        return True
    
    with open(output_path, 'w') as f:
        f.write(f"// Generated from {wav_id}.wav\n")
        f.write(f"#pragma once\n\n")
        f.write(f"const unsigned char config_sounds_{var_name}_wav[] = {{\n")
        
        # Write data in rows of 12 bytes
        for i in range(0, len(wav_data), 12):
            chunk = wav_data[i:i+12]
            hex_str = ', '.join(f'0x{b:02x}' for b in chunk)
            f.write(f"  {hex_str},\n")
        
        f.write("};\n")
        f.write(f"unsigned int config_sounds_{var_name}_wav_len = {len(wav_data)};\n")
    
    size_kb = len(wav_data) / 1024
    print(f"Generated {output_path} ({size_kb:.1f} KB)")
    return True

def main():
    print("Generating C++ headers for priority sounds...")
    print("=" * 80)
    
    total_size = 0
    generated = []
    
    for wav_id, var_name, usage_count in PRIORITY_SOUNDS:
        if generate_header(wav_id, var_name):
            wav_path = os.path.join(WAV_DIR, f"{wav_id}.wav")
            if os.path.exists(wav_path):
                size = os.path.getsize(wav_path)
                total_size += size
                generated.append((var_name, size, usage_count))
    
    print("\n" + "=" * 80)
    print(f"Total embedded audio: {total_size / 1024:.1f} KB ({total_size / 1024 / 1024:.2f} MB)")
    print(f"Generated {len(generated)} sound headers")
    
    print("\nTo use in vector_eyes.cpp, add to play_wav_file():")
    for var_name, size, usage in sorted(generated, key=lambda x: -x[2]):
        print(f'  else if (filename == "{var_name}.wav") {{')
        print(f'    data = config_sounds_{var_name}_wav;')
        print(f'    len = config_sounds_{var_name}_wav_len;')
        print(f'  }}')

if __name__ == "__main__":
    main()
