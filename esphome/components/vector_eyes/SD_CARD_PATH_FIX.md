# SD Card Path Format Fix

## Problem Discovered

The Arduino SD library on ESP32 has a quirk in how it handles file paths:

- **Directory listing**: Uses "/" to open the root directory
- **File access**: Does NOT use leading "/" for files in the root directory

### Error Message
```
[E][vfs_api.cpp:31] open(): does not start with /
```

This error message is misleading - it actually means the path SHOULD NOT start with "/".

## Root Cause

The Arduino SD library mounts the FAT filesystem using ESP-IDF's VFS (Virtual File System), but it doesn't mount it at the root "/". Instead, files in the root directory are accessed without the leading slash.

## Solution

All file access operations must use paths WITHOUT the leading slash:

### Before (WRONG):
```cpp
File file = SD.open("/audio_mappings.json", FILE_READ);
File file = SD.open("/anim_blink_01.json", FILE_READ);
```

### After (CORRECT):
```cpp
File file = SD.open("audio_mappings.json", FILE_READ);
File file = SD.open("anim_blink_01.json", FILE_READ);
```

### Exception - Directory Listing:
```cpp
// This is correct for listing the root directory
File root = SD.open("/");
```

## Files Changed

Updated all `SD.open()` and `SD.exists()` calls in `vector_eyes.cpp` to remove leading slashes from file paths.

## Testing

After this fix, the SD card should be able to:
1. List files in the root directory
2. Open and read `audio_mappings.json`
3. Open and read animation JSON/CSV files
4. Open and play WAV audio files

## Related Issues

- SD.exists() is also unreliable with the Arduino SD library
- Always try to open files directly rather than checking existence first
