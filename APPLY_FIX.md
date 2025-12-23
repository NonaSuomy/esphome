# SD Card Animation Detection Fix Applied

## Problem
The ESP32 storage component's `list_dir()` and `dir_exists()` methods were failing to detect the `/sd/animations` directory even though:
- SD card mounted successfully at `/sd`
- Directory exists with 1,191 JSON animation files
- Path was correct

## Root Cause
ESPHome storage component bug where directory operations fail on ESP32 SD cards immediately after mount.

## Solution Implemented
Modified `storage_adapter.cpp` in the `find_device_with_animations()` function to:
- **BEFORE**: Used `list_dir()` and `dir_exists()` to check for animations directory
- **AFTER**: Check for a specific known file (`anim_blackjack_idle_01.json`) to verify animations exist

This bypasses the broken directory operations and directly tests if animation files are accessible.

## Code Changes
File: `esphome003/esphome/esphome/components/vector_eyes/storage_adapter.cpp`

Lines ~370-435: Replaced directory checks with file existence checks:
```cpp
// WORKAROUND: list_dir() and dir_exists() fail on ESP32 SD cards
// Instead, check for a known animation file to verify directory exists
std::string test_file = mount_path + ANIMATIONS_DIR + "/anim_blackjack_idle_01.json";
ESP_LOGI(TAG, "Testing for specific file: %s", test_file.c_str());

if (device->file_exists(test_file.c_str())) {
    ESP_LOGI(TAG, "Found device with animations at %s (verified via file check)", 
             mount_path.c_str());
    return device;
}
```

## Next Steps

### 1. Upload the Fixed Firmware
```bash
cd esphome003/esphome
bash upload_fix.sh
```

Or manually:
```bash
cd esphome003/esphome
source venv/bin/activate
esphome upload config/vector-eyes-ttgo.yaml --device /dev/ttyUSB0
```

### 2. Capture Boot Logs
After upload, capture the boot sequence to verify the fix:
```bash
bash capture_boot.sh
```

### 3. Expected Log Output
You should now see:
```
[I][sd_storage.spi:183]: SD card mounted successfully at /sd
[I][vector_eyes.storage_adapter:383]: Checking for animations at: /sd/animations
[I][vector_eyes.storage_adapter:386]: Testing for specific file: /sd/animations/anim_blackjack_idle_01.json
[I][vector_eyes.storage_adapter:389]: Found device with animations at /sd (verified via file check)
[I][vector_eyes.storage_adapter:048]: Storage adapter initialized successfully
```

### 4. Verify Animation Playback
Once the device boots with the fix:
- Animations should load from SD card
- Check logs for animation playback messages
- Verify no "falling back to internal animations" warnings

## Files Modified
- `esphome003/esphome/esphome/components/vector_eyes/storage_adapter.cpp`

## Files Created
- `esphome003/esphome/upload_fix.sh` - Upload script
- `esphome003/esphome/APPLY_FIX.md` - This documentation

## Firmware Status
- Firmware compiled successfully
- Location: `config/.esphome/build/vector-eyes-ttgo/.pioenvs/vector-eyes-ttgo/firmware.bin`
- Ready to upload

## Testing Checklist
- [ ] Upload firmware to device
- [ ] Capture boot logs
- [ ] Verify SD card detected with animations
- [ ] Verify animations play from SD card
- [ ] Check for any errors in logs
