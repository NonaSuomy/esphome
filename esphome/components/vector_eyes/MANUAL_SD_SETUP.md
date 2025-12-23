# Manual SD Card Setup

Since the automated script is taking a while, here's how to manually prepare your SD card.

## Your SD Card Location

Your SD card is mounted at: `/run/media/nonasuomy/TTGOCAM/`

## Files to Copy

### 1. Animation JSON Files (Required)

Copy all JSON files from the animations directory:

```bash
cp esphome003/esphome/esphome/components/vector_eyes/animations_json/*.json /run/media/nonasuomy/TTGOCAM/
```

This will copy ~1191 animation files. It may take 5-10 minutes.

### 2. Audio Mappings File (Required)

```bash
cp esphome003/esphome/esphome/components/vector_eyes/audio_mappings.json /run/media/nonasuomy/TTGOCAM/
```

### 3. Audio WAV Files (Required for sound)

Copy the WAV files from vectorsounds:

```bash
cp vectorsounds/*.wav /run/media/nonasuomy/TTGOCAM/
```

## Quick Commands

Run these commands in sequence:

```bash
# 1. Copy audio mappings (fast)
cp esphome003/esphome/esphome/components/vector_eyes/audio_mappings.json /run/media/nonasuomy/TTGOCAM/

# 2. Copy animations (slow - will take several minutes)
cp esphome003/esphome/esphome/components/vector_eyes/animations_json/*.json /run/media/nonasuomy/TTGOCAM/

# 3. Copy audio files (medium speed)
cp vectorsounds/*.wav /run/media/nonasuomy/TTGOCAM/

# 4. Verify
ls /run/media/nonasuomy/TTGOCAM/ | wc -l
```

## Expected Result

After copying, your SD card should contain:
- ~1191 `.json` files (animations)
- ~288 `.wav` files (audio)
- 1 `audio_mappings.json` file

Total: ~1480 files

## Verify SD Card Contents

```bash
# Count files
echo "JSON files: $(ls /run/media/nonasuomy/TTGOCAM/*.json 2>/dev/null | wc -l)"
echo "WAV files: $(ls /run/media/nonasuomy/TTGOCAM/*.wav 2>/dev/null | wc -l)"
echo "Total files: $(ls /run/media/nonasuomy/TTGOCAM/ 2>/dev/null | wc -l)"

# Check specific files
ls /run/media/nonasuomy/TTGOCAM/anim_keepalive_blink_01.json
ls /run/media/nonasuomy/TTGOCAM/audio_mappings.json
```

## After Copying

1. **Safely eject SD card:**
   ```bash
   sync
   umount /run/media/nonasuomy/TTGOCAM
   ```

2. **Insert into ESP32**

3. **Flash firmware:**
   ```bash
   cd esphome003/esphome
   esphome run vector-eyes-ttgo.yaml
   ```

4. **Check logs** for:
   ```
   [I][vector_eyes] SD Card initialized successfully
   [I][vector_eyes] Loaded 288 audio event mappings
   [I][vector_eyes] Total: 1191 JSON files
   ```

## Troubleshooting

### Copy is Very Slow

This is normal - copying 1191 small files takes time. You can:

1. **Use rsync** (faster for many small files):
   ```bash
   rsync -av --progress esphome003/esphome/esphome/components/vector_eyes/animations_json/*.json /run/media/nonasuomy/TTGOCAM/
   ```

2. **Copy in batches**:
   ```bash
   # Copy first 100 files
   ls esphome003/esphome/esphome/components/vector_eyes/animations_json/*.json | head -100 | xargs -I {} cp {} /run/media/nonasuomy/TTGOCAM/
   ```

3. **Use tar** (fastest):
   ```bash
   cd esphome003/esphome/esphome/components/vector_eyes/animations_json
   tar cf - *.json | (cd /run/media/nonasuomy/TTGOCAM && tar xf -)
   ```

### SD Card Full

Check available space:
```bash
df -h /run/media/nonasuomy/TTGOCAM
```

You need at least 300MB free. If full:
- Use a larger SD card (4GB+ recommended)
- Or copy only essential animations (see priority list below)

### Essential Animations Only

If space is limited, copy only these essential animations:

```bash
# Core animations (blink, neutral, emotions)
cp esphome003/esphome/esphome/components/vector_eyes/animations_json/anim_keepalive_blink_*.json /run/media/nonasuomy/TTGOCAM/
cp esphome003/esphome/esphome/components/vector_eyes/animations_json/anim_eyes_neutral.json /run/media/nonasuomy/TTGOCAM/
cp esphome003/esphome/esphome/components/vector_eyes/animations_json/anim_eyes_look_happy.json /run/media/nonasuomy/TTGOCAM/
cp esphome003/esphome/esphome/components/vector_eyes/animations_json/anim_eyes_angry.json /run/media/nonasuomy/TTGOCAM/
cp esphome003/esphome/esphome/components/vector_eyes/animations_json/anim_eyes_awe.json /run/media/nonasuomy/TTGOCAM/
```

## Next Steps

Once files are copied:
1. Eject SD card safely
2. Insert into ESP32
3. Flash firmware
4. Test with Home Assistant buttons
5. Enjoy all Vector animations!
