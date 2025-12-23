# SD Card Setup Guide for Vector Eyes

This guide explains how to set up an SD card to load Vector's original animation and audio files.

## Overview

The system now loads animations and audio directly from the SD card, allowing you to:
- Use all 800+ Vector animations without flash memory constraints
- Load Vector's original JSON animation files (no conversion needed)
- Play authentic Vector audio with synchronized timing
- Easily update animations by copying new files to SD card

## SD Card Requirements

- **Capacity**: 4GB or larger (8GB recommended)
- **Format**: FAT32
- **Speed**: Class 10 or higher recommended for smooth playback

## File Structure

```
SD Card Root/
├── anim_keepalive_blink_01.json
├── anim_eyes_neutral.json
├── anim_eyes_look_happy.json
├── ... (800+ animation JSON files)
├── blink.wav
├── happy.wav
├── curious.wav
├── ... (audio WAV files)
├── audio_mappings.json
└── README.txt
```

## Preparation Steps

### Option 1: Automated Setup (Recommended)

Use the provided Python script to automatically prepare your SD card:

```bash
cd esphome003/esphome/esphome/components/vector_eyes

python prepare_sd_card.py \
    --animations /path/to/vector/animations_json \
    --audio /path/to/vectorsounds \
    --output /path/to/sdcard
```

**Example:**
```bash
python prepare_sd_card.py \
    --animations ./animations_json \
    --audio ../../vectorsounds \
    --output /media/sdcard
```

This will:
1. Copy all Vector animation JSON files
2. Copy referenced audio WAV files
3. Generate audio_mappings.json
4. Create a README file

### Option 2: Manual Setup

1. **Format SD card as FAT32**

2. **Copy Animation Files**
   - Copy all `.json` files from Vector's `animations_json` directory to SD card root
   - Files should be named like: `anim_keepalive_blink_01.json`

3. **Copy Audio Files**
   - Copy `.wav` files from Vector's audio directory to SD card root
   - Files should be named like: `blink.wav`, `happy.wav`, etc.

4. **Create audio_mappings.json**
   - Copy the `audio_mappings.json` file to SD card root
   - This file maps Vector's audio event names to WAV filenames

## Audio Mappings Format

The `audio_mappings.json` file maps Vector's Wwise audio event names to WAV files:

```json
{
  "mappings": {
    "Play__Robot_Vic_Sfx__Blink": "blink.wav",
    "Play__Robot_Vic_Sfx__Happy": "happy.wav",
    "Play__Robot_Vic_Sfx__Curious_Short": "curious_short.wav",
    "Play__Robot_Vic_Sfx__Neutral": "neutral.wav"
  }
}
```

The system supports:
- **Exact matching**: Event name matches mapping key exactly
- **Pattern matching**: Event name contains mapping key as substring
- **Specificity precedence**: More specific matches take priority

## How It Works

### Animation Loading

1. **Trigger**: Animation is requested by name (e.g., "anim_keepalive_blink_01")
2. **File Lookup**: System checks SD card for `anim_keepalive_blink_01.json`
3. **JSON Parsing**: Animation JSON is parsed into keyframes
4. **Audio Extraction**: RobotAudioKeyFrame entries are extracted
5. **Audio Mapping**: Audio event names are mapped to WAV files using audio_mappings.json
6. **Synchronization**: Audio events are matched to closest visual keyframes (100ms tolerance)
7. **Playback**: Animation plays with synchronized audio

### Memory Management

- **On-Demand Loading**: Animations are loaded from SD card when needed
- **Dynamic Buffer**: Current animation stored in RAM (~10-50KB per animation)
- **No Flash Storage**: All animations stay on SD card, freeing up flash memory
- **Audio Streaming**: Audio files are streamed from SD card during playback

## Troubleshooting

### SD Card Not Detected

Check logs for:
```
[I][vector_eyes:xxx] SD Card initialized successfully
```

If not detected:
- Verify SD card is formatted as FAT32
- Check CS pin connection (GPIO0 on TTGO)
- Try lower SPI speed (already set to 400kHz for init)
- Ensure SD card is properly inserted

### Animation Not Found

Check logs for:
```
[W][vector_eyes:xxx] Animation not found: anim_name (tried .json and .csv)
```

Solutions:
- Verify JSON file exists on SD card root
- Check filename matches exactly (case-sensitive)
- Ensure file is valid JSON format

### Audio Not Playing

Check logs for:
```
[W][vector_eyes:xxx] No audio mapping found for event: Play__Robot_Vic_Sfx__XXX
```

Solutions:
- Verify audio_mappings.json exists on SD card
- Check mapping includes the audio event name
- Ensure WAV file exists on SD card
- Verify WAV file format (16-bit, 44.1kHz recommended)

### Audio Out of Sync

The system matches audio to keyframes within 100ms tolerance. Check logs for:
```
[I][vector_eyes:xxx] Synced audio 'blink.wav' to keyframe at 150ms (offset: 25ms)
```

If audio is consistently off:
- Check that audio_mappings.json is correct
- Verify JSON animation files are original Vector files
- Ensure system clock is running at correct speed

## Performance Tips

1. **Use Class 10 SD Card**: Faster read speeds improve loading times
2. **Keep Files at Root**: Avoid subdirectories for faster access
3. **Limit File Count**: While 800+ files work, fewer files = faster directory scans
4. **Pre-cache Common Animations**: System loads on-demand, so frequently used animations load faster after first use

## File Size Estimates

- **Animation JSON**: 5-50KB per file (average ~15KB)
- **Audio WAV**: 10-500KB per file (average ~100KB)
- **Total for 800 animations + audio**: ~200-300MB

A 4GB SD card can easily hold all Vector animations and audio files.

## Updating Animations

To add or update animations:

1. Copy new JSON files to SD card root
2. Copy any new WAV files to SD card root
3. Update audio_mappings.json if needed
4. Reboot ESP32 (or it will load new files automatically)

No firmware reflash required!

## Advanced: Custom Animations

You can create custom animations by:

1. Creating a JSON file following Vector's format
2. Adding it to SD card
3. Referencing it by name (without .json extension)

See Vector's original JSON files for format examples.

## Integration with Existing System

The SD card system integrates seamlessly with your existing setup:

- **Fallback Support**: If SD card fails, system continues with internal animations
- **Hybrid Mode**: Can mix SD card and flash-based animations
- **API Compatible**: All existing Home Assistant controls work unchanged
- **Autonomous Mode**: Random behavior system works with SD card animations

## Next Steps

After setting up your SD card:

1. Insert SD card into ESP32
2. Flash updated firmware
3. Check logs for successful SD card initialization
4. Test animations using Home Assistant buttons
5. Enjoy all 800+ Vector animations!
