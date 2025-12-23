# SD Card Setup Instructions

## Fresh SD Card Setup

You have a fresh SD card mounted at `/run/media/nonasuomy/ESCAPE/`

### Quick Setup (Recommended)

Run the quick setup script:
```bash
cd esphome003/esphome
chmod +x setup_sdcard_quick.sh
bash setup_sdcard_quick.sh
```

This will copy:
- **1,191 animation JSON files** from `esphome/components/vector_eyes/animations_json/` to `/animations/`
- **1,347 audio WAV files** from `vectorsounds/` to `/audio/`
- **audio_mappings.json** to the root directory

### Manual Setup

If you prefer to copy manually:

```bash
SD="/run/media/nonasuomy/ESCAPE"

# Create directories
mkdir -p "$SD/animations" "$SD/audio"

# Copy animations
cp esphome/components/vector_eyes/animations_json/*.json "$SD/animations/"

# Copy audio (this takes a few minutes)
cp ../../../vectorsounds/*.wav "$SD/audio/"

# Copy mappings
cp esphome/components/vector_eyes/audio_mappings.json "$SD/"
```

### Verification

After copying, verify the SD card contents:
```bash
SD="/run/media/nonasuomy/ESCAPE"
echo "Animations: $(ls "$SD/animations"/*.json 2>/dev/null | wc -l)"
echo "Audio: $(ls "$SD/audio"/*.wav 2>/dev/null | wc -l)"
ls -lh "$SD/audio_mappings.json"
```

Expected output:
- Animations: 1191 files
- Audio: 1347 files
- audio_mappings.json: present

### Critical File Check

The ESP32 code checks for this specific file to detect animations:
```bash
ls -lh /run/media/nonasuomy/ESCAPE/animations/anim_blackjack_idle_01.json
```

This file MUST exist for the fix to work!

## SD Card Structure

After setup, your SD card should look like this:
```
/run/media/nonasuomy/ESCAPE/
├── animations/
│   ├── anim_blackjack_idle_01.json  ← Critical test file
│   ├── anim_blackjack_deal_01.json
│   ├── ... (1,191 total JSON files)
├── audio/
│   ├── 117560463.wem.wav
│   ├── 117954389.wem.wav
│   ├── ... (1,347 total WAV files)
└── audio_mappings.json
```

## Next Steps

After SD card setup:

1. **Safely eject the SD card**
   ```bash
   umount /run/media/nonasuomy/ESCAPE
   ```

2. **Insert SD card into ESP32**

3. **Upload the fixed firmware**
   ```bash
   cd esphome003/esphome
   bash upload_fix.sh
   ```

4. **Capture boot logs to verify**
   ```bash
   bash capture_boot.sh
   ```

5. **Look for success messages**:
   ```
   [I][sd_storage.spi:183]: SD card mounted successfully at /sd
   [I][vector_eyes.storage_adapter:386]: Testing for specific file: /sd/animations/anim_blackjack_idle_01.json
   [I][vector_eyes.storage_adapter:389]: Found device with animations at /sd (verified via file check)
   [I][vector_eyes.storage_adapter:048]: Storage adapter initialized successfully
   ```

## Troubleshooting

### SD Card Not Detected
- Check if mounted: `ls /run/media/nonasuomy/`
- Check permissions: `ls -la /run/media/nonasuomy/ESCAPE`

### Copy Fails
- Check disk space: `df -h /run/media/nonasuomy/ESCAPE`
- Check write permissions: `touch /run/media/nonasuomy/ESCAPE/test.txt`

### Files Missing After Copy
- Verify source files exist:
  - Animations: `ls esphome/components/vector_eyes/animations_json/*.json | wc -l`
  - Audio: `ls ../../../vectorsounds/*.wav | wc -l`
