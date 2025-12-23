import json
import os
import re
from collections import defaultdict

# Paths
ANIMATIONS_DIR = "animations_json"
SOUND_MAPPINGS = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/converted_wav/sound_mappings.txt"
WAV_DIR = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/victor-audio-assets/converted_wav"

def parse_sound_mappings():
    """Parse sound_mappings.txt to create ID -> name mapping"""
    mappings = {}
    with open(SOUND_MAPPINGS, 'r') as f:
        for line in f:
            line = line.strip()
            if '|' in line and not line.startswith('='):
                parts = line.split('|')
                if len(parts) == 2:
                    sound_id = parts[0].strip()
                    sound_name = parts[1].strip()
                    mappings[sound_name] = sound_id
    return mappings

def extract_audio_events():
    """Extract all unique audio events from animation JSONs"""
    audio_events = defaultdict(list)  # event_name -> [animation_file, ...]
    
    for filename in os.listdir(ANIMATIONS_DIR):
        if not filename.endswith('.json'):
            continue
            
        filepath = os.path.join(ANIMATIONS_DIR, filename)
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                
            # Find the animation array
            for key in data:
                if isinstance(data[key], list):
                    for frame in data[key]:
                        if frame.get("Name") == "RobotAudioKeyFrame":
                            audio_names = frame.get("audioName", [])
                            for audio_name in audio_names:
                                audio_events[audio_name].append(filename)
        except Exception as e:
            print(f"Error processing {filename}: {e}")
    
    return audio_events

def main():
    print("Extracting audio events from animations...")
    audio_events = extract_audio_events()
    
    print(f"\nFound {len(audio_events)} unique audio events")
    print(f"Referenced across animations\n")
    
    print("Parsing sound mappings...")
    sound_mappings = parse_sound_mappings()
    
    # Analyze mappings
    matched = []
    unmatched = []
    
    for event_name in sorted(audio_events.keys()):
        # Try to match event name to sound file
        # Event names are like "Play__Robot_Vic_Sfx__Head_Up_Short_Curious"
        # Sound names are like "Robot_Vic_Sfx_Head_Up_Short_Curious_01_WM"
        
        # Extract the core name
        clean_event = event_name.replace("Play__", "").replace("Stop__", "")
        clean_event = clean_event.replace("__", "_")
        
        # Find matching sounds
        matches = []
        for sound_name, sound_id in sound_mappings.items():
            if clean_event.lower() in sound_name.lower():
                wav_path = os.path.join(WAV_DIR, f"{sound_id}.wav")
                if os.path.exists(wav_path):
                    size = os.path.getsize(wav_path)
                    matches.append((sound_name, sound_id, size))
        
        if matches:
            matched.append((event_name, matches, len(audio_events[event_name])))
        else:
            unmatched.append((event_name, len(audio_events[event_name])))
    
    print(f"\n{'='*80}")
    print(f"ANALYSIS RESULTS")
    print(f"{'='*80}")
    print(f"Total unique audio events: {len(audio_events)}")
    print(f"Matched to WAV files: {len(matched)}")
    print(f"Unmatched: {len(unmatched)}")
    
    # Calculate total size
    total_size = 0
    unique_wavs = set()
    
    for event_name, matches, usage_count in matched:
        # Use the first match (usually the best)
        if matches:
            sound_name, sound_id, size = matches[0]
            unique_wavs.add(sound_id)
            total_size += size
    
    print(f"\nUnique WAV files needed: {len(unique_wavs)}")
    print(f"Total size: {total_size / 1024 / 1024:.2f} MB")
    print(f"Average WAV size: {total_size / len(unique_wavs) / 1024:.2f} KB")
    
    # Write detailed report
    with open("audio_analysis_report.txt", "w") as f:
        f.write("="*80 + "\n")
        f.write("AUDIO INTEGRATION ANALYSIS\n")
        f.write("="*80 + "\n\n")
        
        f.write(f"Total unique audio events: {len(audio_events)}\n")
        f.write(f"Matched to WAV files: {len(matched)}\n")
        f.write(f"Unmatched: {len(unmatched)}\n")
        f.write(f"Unique WAV files needed: {len(unique_wavs)}\n")
        f.write(f"Total size: {total_size / 1024 / 1024:.2f} MB\n\n")
        
        f.write("="*80 + "\n")
        f.write("MATCHED EVENTS (Event -> WAV mapping)\n")
        f.write("="*80 + "\n\n")
        
        for event_name, matches, usage_count in sorted(matched, key=lambda x: -x[2]):
            f.write(f"{event_name} (used in {usage_count} animations)\n")
            for sound_name, sound_id, size in matches[:3]:  # Show top 3 matches
                f.write(f"  -> {sound_id}.wav ({size/1024:.1f} KB) {sound_name}\n")
            f.write("\n")
        
        if unmatched:
            f.write("\n" + "="*80 + "\n")
            f.write("UNMATCHED EVENTS\n")
            f.write("="*80 + "\n\n")
            
            for event_name, usage_count in sorted(unmatched, key=lambda x: -x[1]):
                f.write(f"{event_name} (used in {usage_count} animations)\n")
    
    print(f"\nDetailed report written to: audio_analysis_report.txt")
    
    # Write mapping CSV for easy processing
    with open("audio_event_mapping.csv", "w") as f:
        f.write("EventName,SoundID,SoundName,Size,UsageCount\n")
        for event_name, matches, usage_count in matched:
            if matches:
                sound_name, sound_id, size = matches[0]
                f.write(f'"{event_name}",{sound_id},"{sound_name}",{size},{usage_count}\n')
    
    print("Mapping CSV written to: audio_event_mapping.csv")

if __name__ == "__main__":
    main()
