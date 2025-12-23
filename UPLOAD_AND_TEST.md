# Upload Fixed Firmware and Test

## Step 1: Upload Firmware

Run this command to upload the fixed firmware:

```bash
cd esphome003/esphome
source venv/bin/activate
esphome upload config/vector-eyes-ttgo.yaml --device /dev/ttyUSB0
```

The upload will take a few minutes. The ESP32 will automatically reboot after upload.

## Step 2: Monitor Serial Output

After upload completes, monitor the serial output to see the boot sequence:

```bash
esphome logs config/vector-eyes-ttgo.yaml --device /dev/ttyUSB0
```

Or use screen:
```bash
screen /dev/ttyUSB0 115200
```

## Step 3: Look for Success Messages

Watch for these log messages that indicate the fix worked:

### ✓ SD Card Mounted
```
[I][sd_storage.spi:183]: SD card mounted successfully at /sd
```

### ✓ Testing for Animation File
```
[I][vector_eyes.storage_adapter:386]: Testing for specific file: /sd/animations/anim_blackjack_idle_01.json
```

### ✓ Animations Found (THE KEY MESSAGE!)
```
[I][vector_eyes.storage_adapter:389]: Found device with animations at /sd (verified via file check)
```

### ✓ Storage Initialized
```
[I][vector_eyes.storage_adapter:048]: Storage adapter initialized successfully
```

## What to Expect

### If the Fix Works ✓
You should see:
- SD card mounts at `/sd`
- File check for `anim_blackjack_idle_01.json` succeeds
- Storage adapter initializes successfully
- **NO** "falling back to internal animations" warning
- Animations should play from SD card

### If There's Still an Issue ✗
You might see:
- `[W][vector_eyes.storage_adapter]: Test file not found`
- `[W][vector_eyes.storage_adapter]: No storage device with animations found`
- `[W][vector_eyes]: No storage available, using internal animations only`

## Troubleshooting

### SD Card Not Detected
- Check SD card is fully inserted
- Verify SD card has files (re-mount on PC and check)
- Check for hardware connection issues

### File Not Found
- Verify `anim_blackjack_idle_01.json` exists on SD card
- Check SD card filesystem (should be FAT32)
- Try reformatting SD card and copying files again

### Still Using Internal Animations
- Capture full boot logs and review
- Check for SPI bus errors
- Verify SD card pins in YAML config

## Capture Boot Logs

To save the boot sequence for analysis:

```bash
cd esphome003/esphome
bash capture_boot.sh
```

This will save logs to `boot_logs_TIMESTAMP.txt`

## Next Steps

Once you see the success messages:
1. Verify animations are playing
2. Check that audio is working
3. Test different animation triggers
4. Enjoy your Vector Eyes with SD card storage!
