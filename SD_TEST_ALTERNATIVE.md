# Alternative: Test SD Card Using ESPHome Code Directly

Since Arduino CLI installation is taking too long, let's test the SD card using ESPHome's own storage component with enhanced logging.

## The Issue

We discovered that both `list_dir()` and `dir_exists()` fail to detect `/sd/animations` even though:
- SD card mounts successfully
- Directory exists with 1,191 files
- Path is correct

## Quick Fix to Test

Instead of waiting for Arduino, let's modify the ESPHome code to test if we can read a specific file:

### Option 1: Test for Specific File (FASTEST)

Modify `storage_adapter.cpp` to check for a known file instead of directory:

```cpp
// Around line 385 in find_device_with_animations()
// REPLACE:
if (device->list_dir(animations_path.c_str(), &entries)) {
    ESP_LOGI(TAG, "Found device with animations...");
    return device;
}

// WITH:
std::string test_file = mount_path + "/animations/anim_blackjack_idle_01.json";
ESP_LOGI(TAG, "Testing for specific file: %s", test_file.c_str());
if (device->file_exists(test_file.c_str())) {
    ESP_LOGI(TAG, "Found device with animations at %s (verified via file check)", mount_path.c_str());
    return device;
} else {
    ESP_LOGW(TAG, "Test file not found: %s", test_file.c_str());
}
```

This bypasses the broken directory operations entirely.

### Option 2: Add Delay After Mount

The SD card filesystem might need time to stabilize:

```cpp
// In find_device_with_animations(), after getting devices:
delay(500);  // Wait 500ms for filesystem to stabilize
```

### Option 3: Skip Directory Check Entirely

Just use any SD card that's mounted:

```cpp
// If device is mounted at /sd, assume it has animations
if (device->get_info().mount_path == "/sd") {
    ESP_LOGI(TAG, "Using SD card at /sd (skipping directory check due to known bug)");
    return device;
}
```

## Recommendation

**Use Option 1** - it's the most reliable and still validates that animation files exist.

Would you like me to implement this fix now?
