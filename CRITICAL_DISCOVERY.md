# CRITICAL DISCOVERY - Both list_dir() and dir_exists() Fail

## Boot Log Analysis

Successfully captured boot sequence showing:

### SD Card Mount - SUCCESS ✓
```
[I][sd_storage.spi:183]: SD card mounted successfully at /sd
[D][storage:368]: Registered storage device: SD Card (SPI) (id: sd_card, type: 1)
[D][storage:104]: Registered mount: /sd (platform: sd_mmc)
```

### Animation Discovery - FAILED ✗
```
[I][vector_eyes.storage_adapter:383]: Checking for animations at: /sd/animations
[W][vector_eyes.storage_adapter:392]: Cannot list animations directory at: /sd/animations (trying dir_exists)
[W][vector_eyes.storage_adapter:421]: Cannot list animations directory at: /sd/animations (trying dir_exists)
[D][vector_eyes.storage_adapter:432]: No storage device with animations directory found
```

## The Problem

**BOTH `list_dir()` AND `dir_exists()` FAIL** to detect `/sd/animations` even though:

1. ✓ SD card mounts successfully
2. ✓ Directory `/animations` exists on card (verified via Linux mount)
3. ✓ Directory contains 1,191 JSON files
4. ✓ Path `/sd/animations` is correct

## Root Cause Analysis

The ESPHome storage component has an issue where:
- `device->list_dir("/sd/animations", &entries)` returns false
- `device->dir_exists("/sd/animations")` returns false
- But the directory actually exists!

This suggests:
1. **Timing issue**: Directory operations happen too soon after mount
2. **Path handling bug**: Storage component may not handle subdirectories correctly
3. **Filesystem state**: SD card filesystem may not be fully ready

## Proposed Solutions

### Solution 1: Test for Specific File (RECOMMENDED)
Instead of checking if directory exists, check for a known animation file:
```cpp
if (device->file_exists("/sd/animations/anim_blackjack_idle_01.json")) {
    // Animations directory exists and has files
    return device;
}
```

### Solution 2: Add Delay After Mount
Wait for filesystem to stabilize:
```cpp
delay(500);  // Wait 500ms after mount
if (device->list_dir(animations_path.c_str(), &entries)) {
    // Try again after delay
}
```

### Solution 3: Skip Directory Check Entirely
Just assume animations exist if SD card is mounted:
```cpp
if (device->get_info().mount_path == "/sd") {
    ESP_LOGI(TAG, "Using SD card at /sd (skipping directory check)");
    return device;
}
```

### Solution 4: Investigate ESPHome Storage Component
Check the source code of:
- `esphome/components/storage/storage_device.h`
- `esphome/components/sd_storage/sd_storage.cpp`

Look for bugs in `list_dir()` and `dir_exists()` implementations.

## Immediate Action

I recommend **Solution 1** (test for specific file) as it:
- Works around the directory check bug
- Still validates animations exist
- Minimal code change
- Most reliable approach

## Files to Modify

`esphome/components/vector_eyes/storage_adapter.cpp` line ~385:

```cpp
// OLD CODE:
if (device->list_dir(animations_path.c_str(), &entries)) {
    ESP_LOGI(TAG, "Found device with animations...");
    return device;
}

// NEW CODE:
// Try to find a known animation file as proof directory exists
std::string test_file = mount_path + "/animations/anim_blackjack_idle_01.json";
if (device->file_exists(test_file.c_str())) {
    ESP_LOGI(TAG, "Found device with animations at %s (verified via file check)", mount_path.c_str());
    return device;
}
```

This bypasses the broken directory operations and directly tests if animation files are accessible.
