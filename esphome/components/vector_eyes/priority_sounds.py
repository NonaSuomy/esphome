#!/usr/bin/env python3
"""
Manual mapping of top priority audio events to WAV files
Based on sound name similarity and manual verification
"""

# Top 20 audio events by usage frequency
# Format: (event_name, usage_count, likely_wav_id, likely_wav_name)
PRIORITY_SOUND_MAPPINGS = [
    # Screen sounds (most common)
    ("Play__Robot_Vic_Sfx__Scrn_Curious", 70, "319435599", "Robot_Vic_Sfx__Scrn_Curious_Long_01_WM"),
    ("Play__Robot_Vic_Sfx__Scrn_Curious_Short", 47, "1052205531", "Robot_Vic_Sfx__Scrn_Happy_Long_02_WM"),  # Placeholder - need to verify
    ("Play__Robot_Vic_Sfx__Blink", 38, "256459301", "Robot_Vic_Sfx__Scrn_Blink_01_WM"),
    
    # Head movement sounds
    ("Play__Robot_Vic_Sfx__Head_Up_Short_Curious", 26, None, None),  # Need to find
    ("Play__Robot_Vic_Sfx__Head_Up_Long_Curious", 17, None, None),
    ("Play__Robot_Vic_Sfx__Head_Down_Short_Curious", 15, None, None),
    ("Play__Robot_Vic_Sfx__Head_Up_Short_Neutral", 13, None, None),
    ("Play__Robot_Vic_Sfx__Head_Down_Short_Neutral", 9, None, None),
    
    # Lift sounds  
    ("Play__Robot_Vic_Sfx__Lift_High_Up_Short_Curious", 16, None, None),
    ("Play__Robot_Vic_Sfx__Lift_High_Down_Short_Surprised", 15, None, None),
    ("Play__Robot_Vic_Sfx__Lift_High_Down_Short_Neutral", 4, "280407689", "Robot_Vic_Sfx_Lift_High_Down_Short_Neutral_04_WM"),
    ("Play__Robot_Vic_Sfx__Lift_High_Up_Short_Neutral", 3, "102332682", "Robot_Vic_Sfx_Lift_High_Up_Short_Neutral_01_WM"),
    
    # More screen emotions
    ("Play__Robot_Vic_Sfx__Scrn_Neutral", 16, None, None),
    ("Play__Robot_Vic_Sfx__Scrn_Surprised", 13, None, None),
    ("Play__Robot_Vic_Sfx__Scrn_Happy", 11, None, None),
    ("Play__Robot_Vic_Sfx__Scrn_Neutral_Short", 9, None, None),
    ("Play__Robot_Vic_Sfx__Scrn_Happy_Short", 6, None, None),
    ("Play__Robot_Vic_Sfx__Scrn_Angry", 5, None, None),
    ("Play__Robot_Vic_Sfx__Scrn_Sad_Long", 4, None, None),
    ("Play__Robot_Vic_Sfx__Scrn_Sad", 3, None, None),
]

# We already have these embedded:
ALREADY_EMBEDDED = {
    "256459301": "blink.h",
    "1052205531": "happy.h", 
    "319435599": "curious.h",
}

def main():
    print("Priority Audio Event Mappings")
    print("=" * 80)
    print(f"{'Event Name':<50} {'Count':<8} {'WAV ID':<15} {'Status'}")
    print("=" * 80)
    
    for event_name, count, wav_id, wav_name in PRIORITY_SOUND_MAPPINGS:
        if wav_id and wav_id in ALREADY_EMBEDDED:
            status = f"✓ Embedded ({ALREADY_EMBEDDED[wav_id]})"
        elif wav_id:
            status = "Found - needs embedding"
        else:
            status = "⚠ Need to find WAV"
        
        print(f"{event_name:<50} {count:<8} {wav_id or 'Unknown':<15} {status}")
    
    print("\n" + "=" * 80)
    print("Next Steps:")
    print("1. Find WAV IDs for unmapped events by searching converted_wav directory")
    print("2. Generate C++ headers for new sounds")
    print("3. Update vector_eyes to play sounds on RobotAudioKeyFrame trigger")

if __name__ == "__main__":
    main()
