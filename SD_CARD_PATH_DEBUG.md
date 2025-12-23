# SD Card Path Debugging

## Issue
SD card has files but storage adapter reports "No storage device with animations directory found"

## SD Card Contents (Confirmed)
- **Animations (JSON)**: 1,191 files (~160 KB) in `/animations/` directory
- **Animations (CSV)**: 69 files (~8 KB)
- **Audio**: 1,347 files (~132 KB)

## Code Configuration
- Mount path: `/sd`
- Looking for: `/sd/animations`
- Constant: `ANIMATIONS_DIR = "/animations"`

## Debug Steps

### 1. Added Enhanced Logging
Added detailed logging to show:
- Exact path being checked
- Whether dir_exists() returns true/false
- Which devices are being scanned

### 2. Capture New Logs
Run: `./capture_boot_logs.sh` and reset device

### 3. Check Logs For
Look for these new log messages:
```
[I][vector_eyes.storage_adapter:XXX]: Checking for animations at: /sd/animations
[W][vector_eyes.storage_adapter:XXX]: Animations directory does not exist at: /sd/animations
```

Or success:
```
[I][vector_eyes.storage_adapter:XXX]: Found device with animations: SD Card (SPI) at /sd
```

## Possible Causes

1. **Case Sensitivity**: Linux filesystems are case-sensitive
   - SD card might have `/Animations` or `/ANIMATIONS`
   - Check: `ls -la /path/to/sdcard/`

2. **Hidden Directory**: Directory might be hidden
   - Check: `ls -la /path/to/sdcard/` (shows hidden files)

3. **Permissions**: Directory might not be readable
   - Check: `ls -ld /path/to/sdcard/animations/`

4. **FAT32 Issues**: SD card filesystem issues
   - Verify: SD card is formatted as FAT32
   - Check: No filesystem errors

5. **Empty Directory**: Directory exists but is empty
   - The code checks `dir_exists()` which should return true even if empty
   - But verify files are actually there

## Next Steps

1. Compile and upload firmware with enhanced logging
2. Reset device and capture boot logs
3. Review logs to see exact path being checked
4. Verify SD card directory structure matches expectations
