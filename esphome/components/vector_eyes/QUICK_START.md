# Quick Start: SD Card Setup for Vector Eyes

## What You Need

1. **SD Card** (4GB+, FAT32 formatted)
2. **Vector's Animation Files** (animations_json directory)
3. **Vector's Audio Files** (vectorsounds directory)
4. **Python 3** (for automated setup)

## 5-Minute Setup

### Step 1: Prepare SD Card

Run the preparation script:

```bash
cd esphome003/esphome/esphome/components/vector_eyes

python prepare_sd_card.py \
    --animations ./animations_json \
    --audio ../../vectorsounds \
    --output /path/to/your/sdcard
```

**Example for mounted SD card:**
```bash
python prepare_sd_card.py \
    --animations ./animations_json \
    --audio ../../vectorsounds \
    --output /media/sdcard
```

This will copy all necessary files to your SD card.

### Step 2: Insert SD Card

1. Safely eject SD card from computer
2. Insert into ESP32 SD card slot
3. Ensure proper connection (CS pin = GPIO0 on TTGO)

### Step 3: Flash Firmware

```bash
cd esphome003/esphome
esphome run vector-eyes-ttgo.yaml
```

### Step 4: Verify

Check the logs for:

```
[I][vector_eyes] SD Card initialized successfully
[I][vector_eyes] Loaded 288 audio event mappings
[I][vector_eyes] Total: 800 JSON, 0 CSV, 150 WAV files
```

### Step 5: Test

Use Home Assistant to trigger an animation:

```yaml
service: esphome.vector_eyes_ttgo_play_animation
data:
  name: "anim_keepalive_blink_01"
```

Or use the built-in buttons in Home Assistant.

## What's on the SD Card?

After preparation, your SD card contains:

```
/
├── anim_keepalive_blink_01.json    (800+ animation files)
├── anim_eyes_neutral.json
├── anim_eyes_look_happy.json
├── ...
├── blink.wav                        (150+ audio files)
├── happy.wav
├── curious.wav
├── ...
├── audio_mappings.json              (audio event mappings)
└── README.txt                       (info file)
```

## Troubleshooting

### SD Card Not Detected

**Check:**
- SD card is FAT32 formatted
- SD card is properly inserted
- CS pin connection (GPIO0)

**Logs should show:**
```
[I][vector_eyes] Initializing SD card on CS pin 0...
[I][vector_eyes] SD Card initialized successfully
```

### Animation Not Playing

**Check:**
- Animation file exists on SD card
- Filename matches exactly (case-sensitive)
- JSON file is valid format

**Try:**
```bash
# List files on SD card
ls /path/to/sdcard/anim_*.json | head -10
```

### No Audio

**Check:**
- audio_mappings.json exists on SD card
- WAV files exist on SD card
- Speaker is connected and configured

**Logs should show:**
```
[I][vector_eyes] Loaded 288 audio event mappings
[I][vector_eyes] Synced audio 'blink.wav' to keyframe at 150ms
```

## Next Steps

- **Explore Animations**: Try different animation names
- **Customize Mappings**: Edit audio_mappings.json
- **Add Custom Animations**: Create your own JSON files
- **Monitor Performance**: Check logs for timing info

## Common Animation Names

Try these popular Vector animations:

- `anim_keepalive_blink_01` - Blink animation
- `anim_eyes_neutral` - Neutral expression
- `anim_eyes_look_happy` - Happy expression
- `anim_eyes_angry` - Angry expression
- `anim_eyes_awe` - Surprised/awe expression
- `anim_generic_look_up_01` - Look up
- `anim_gotosleep_getin_01` - Sleep animation
- `anim_dancebeat_getin_01` - Dance animation

## Getting Help

If you encounter issues:

1. Check the logs for error messages
2. Review `SD_CARD_SETUP.md` for detailed troubleshooting
3. Verify SD card file structure matches expected layout
4. Test with a simple animation first (e.g., blink)

## Success!

Once you see animations playing with synchronized audio, you're all set! You now have access to all 800+ Vector animations without any flash memory constraints.

Enjoy your Vector Eyes! 🤖👀
