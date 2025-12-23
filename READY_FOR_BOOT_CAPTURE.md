# Ready for Boot Log Capture

## Current Status

The firmware has been updated with enhanced SD card detection using `list_dir()` instead of `dir_exists()`.

### What Changed

In `storage_adapter.cpp`, the `find_device_with_animations()` function now:
1. **Primary method**: Uses `list_dir()` to check for `/sd/animations` directory
2. **Fallback method**: Falls back to `dir_exists()` if `list_dir()` fails
3. **Enhanced logging**: Reports the number of entries found (should be 1,191 for your SD card)

### Expected Log Output

If the fix works, you should see:
```
[I][vector_eyes.storage_adapter:xxx]: Checking for animations at: /sd/animations
[I][vector_eyes.storage_adapter:xxx]: Found device with animations: sd_storage_spi at /sd (1191 entries)
```

If it still fails, you'll see:
```
[W][vector_eyes.storage_adapter:xxx]: Cannot list animations directory at: /sd/animations (trying dir_exists)
[W][vector_eyes.storage_adapter:054]: No storage device with animations found, will fall back to internal animations
```

## How to Capture Boot Logs

### Option 1: Automated Script (Recommended)

```bash
cd esphome003/esphome
./capture_full_boot.sh
```

This script will:
- Prompt you when to reset the device
- Capture 60 seconds of serial output
- Save to `boot_capture_final.txt`
- Show a summary of what was captured

### Option 2: Manual Capture

```bash
cd esphome003/esphome
cat /dev/ttyUSB0 > boot_capture_manual.txt &
# Reset your device now
sleep 60
pkill cat
```

## What to Look For

### 1. Boot Sequence
- `rst:0x1 (POWERON_RESET)` or similar reset reason
- ESP-IDF version info
- Component initialization

### 2. SD Card Mount
- `[I][sd_storage.spi:183]: SD card mounted successfully at /sd`
- Card type and size info

### 3. Animation Discovery (THE KEY PART)
- `[I][vector_eyes.storage_adapter:xxx]: Checking for animations at: /sd/animations`
- Either:
  - SUCCESS: `Found device with animations... (1191 entries)`
  - FAILURE: `Cannot list animations directory` or `No storage device with animations found`

### 4. SPI Errors (Known Issue)
- You'll see many `[E][spi-esp-idf:077]: Transmit failed - err 102` errors
- These are from the display and are a separate issue
- They don't prevent SD card access

## Next Steps After Capture

1. **If list_dir() succeeds** (shows 1191 entries):
   - SD card detection is fixed!
   - Move on to testing animation playback

2. **If list_dir() fails** (same error as before):
   - The ESPHome storage component has a deeper issue
   - Will need to investigate alternative approaches:
     - Remove directory check entirely
     - Use `file_exists()` for a specific file
     - Add delay after SD mount
     - Check ESPHome storage component source code

## Your SD Card Contents (Verified)

```
/sd/
├── animations/          (1,191 JSON files)
├── audio/              (1,347 WAV files)
└── audio_mappings.json (1 file)
```

The directory structure is correct. The issue is purely in the detection code.
