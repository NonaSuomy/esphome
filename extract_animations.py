#!/usr/bin/env python3
import re
import os
import csv

# Configuration
INPUT_FILE = "esphome/components/vector_eyes/generated_animations.h"
OUTPUT_DIR = "sd_card_files"

def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def parse_animations(content):
    # Regex to find animation arrays
    # Matches: static const AnimationKeyframe NAME[] = { ... };
    anim_regex = re.compile(r'static const AnimationKeyframe (\w+)\[\] = \{(.*?)\};', re.DOTALL)
    
    animations = {}
    
    for match in anim_regex.finditer(content):
        name = match.group(1)
        body = match.group(2)
        
        # Clean up name (remove ANIM_ prefix and convert to lowercase)
        if name.startswith("ANIM_"):
            clean_name = name[5:].lower()
        else:
            clean_name = name.lower()
            
        keyframes = []
        
        # Regex to match individual keyframes
        # { 0, 33, { 1.0f, ... }, nullptr }
        # { 66, 66, { ... }, "sound.wav" }
        kf_regex = re.compile(r'\{\s*(\d+),\s*(\d+),\s*\{\s*([^}]+)\s*\},\s*([^}]+?)\s*\}', re.DOTALL)
        
        for kf_match in kf_regex.finditer(body):
            trigger_time = int(kf_match.group(1))
            duration = int(kf_match.group(2))
            face_params_str = kf_match.group(3)
            sound_part = kf_match.group(4).strip()
            
            # Parse face params
            face_params = [float(x.strip().rstrip('f')) for x in face_params_str.split(',')]
            
            # Parse sound
            sound_name = ""
            if sound_part != "nullptr":
                # Extract string content: "sound.wav" -> sound.wav
                sound_name = sound_part.strip('"')
            
            # Construct row
            # time, dur, sx, sy, ang, cx, cy, llt, llb, rlt, rlb, sound
            row = [trigger_time, duration] + face_params + [sound_name]
            keyframes.append(row)
            
        animations[clean_name] = keyframes
        
    return animations

def write_csv(name, keyframes, output_dir):
    filename = os.path.join(output_dir, f"{name}.csv")
    print(f"Writing {filename} ({len(keyframes)} frames)...")
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        # Header (optional, but good for reference)
        writer.writerow(["time", "duration", "scale_x", "scale_y", "angle", "center_x", "center_y", "l_lid_top", "l_lid_bottom", "r_lid_top", "r_lid_bottom", "sound"])
        writer.writerows(keyframes)

def main():
    print(f"Reading {INPUT_FILE}...")
    try:
        with open(INPUT_FILE, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: Could not find {INPUT_FILE}")
        return

    ensure_dir(OUTPUT_DIR)
    
    animations = parse_animations(content)
    print(f"Found {len(animations)} animations.")
    
    for name, keyframes in animations.items():
        write_csv(name, keyframes, OUTPUT_DIR)
        
    print("Done!")

if __name__ == "__main__":
    main()
