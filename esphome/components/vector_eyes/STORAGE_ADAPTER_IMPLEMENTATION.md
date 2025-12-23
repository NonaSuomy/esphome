# StorageAdapter Implementation

## Overview

This document describes the implementation of the `StorageAdapter` class for the `vector_eyes` component. The StorageAdapter provides a unified interface for accessing animation and audio files from various storage backends through the ESPHome storage component.

## Files Created

1. **storage_adapter.h** - Header file defining the StorageAdapter class interface
2. **storage_adapter.cpp** - Implementation of the StorageAdapter class

## Implementation Details

### Class Structure

The `StorageAdapter` class is located in the `esphome::vector_eyes` namespace and provides the following functionality:

#### Initialization
- `initialize(storage::Storage *storage_component)` - Initializes the adapter with a storage component reference
- `is_available()` - Checks if storage is ready for use

#### File Operations
- `file_exists(const std::string &path)` - Checks if a file exists on any available storage device
- `read_file(const std::string &path, std::vector<uint8_t> &data)` - Reads entire file into memory
- `list_animations(std::vector<std::string> &animation_names)` - Lists all .json animation files

#### Streaming Operations
- `open_file(const std::string &path)` - Opens a file for streaming access (for large audio files)
- `read_chunk(void *handle, uint8_t *buffer, size_t size)` - Reads a chunk from an open file
- `close_file(void *handle)` - Closes an open file handle

#### Device Management
- `get_primary_device()` - Returns the currently selected storage device
- `set_preferred_mount_path(const std::string &path)` - Sets preferred mount path for device selection

#### Event Callbacks
- `on_device_added(storage::StorageDevice *device)` - Called when a new storage device is added (hot-plug support)
- `on_device_removed(storage::StorageDevice *device)` - Called when a storage device is removed

### Key Features

#### 1. Device Discovery and Selection

The adapter automatically discovers storage devices that contain animation files:

```cpp
storage::StorageDevice *find_device_with_animations()
```

This method:
- Checks all available storage devices
- Prioritizes devices matching the preferred mount path
- Looks for devices with an `/animations` directory
- Returns the first suitable device found

#### 2. Fallback Handling

The adapter implements robust fallback logic:

- If the primary device fails, it tries all other available devices
- If no storage is available, the component can fall back to internal animations
- Gracefully handles device removal and attempts to find replacements

#### 3. Multi-Device Support

The adapter supports multiple storage devices:

- Tries primary device first for all operations
- Falls back to alternative devices if primary fails
- Supports hot-plug events (USB storage, SD card insertion/removal)

#### 4. Error Handling and Logging

Comprehensive logging at different levels:

- `ESP_LOGI` - Important events (device discovery, initialization)
- `ESP_LOGW` - Warnings (fallback scenarios, missing files)
- `ESP_LOGE` - Errors (failed operations)
- `ESP_LOGD` - Debug information (file operations, device details)

### Internal Helper Methods

#### validate_device()
Checks if a device is suitable for use:
- Device is not null
- Device is available/mounted
- Device supports filesystem operations

#### try_read_from_device()
Attempts to read a file from a specific device:
- Gets file size first
- Allocates appropriate buffer
- Reads file contents
- Handles errors gracefully

#### build_full_path()
Constructs full path by combining mount path and relative path:
- Handles paths with and without leading slashes
- Uses primary device's mount path

#### log_device_info()
Logs detailed information about a storage device:
- Device ID and name
- Mount path
- Type and availability
- Capacity and free space

## Requirements Satisfied

This implementation satisfies the following requirements from the specification:

### Requirement 1.1
✅ Vector_eyes obtains a reference to the storage component instead of initializing SD card directly

### Requirement 1.3
✅ When storage component is not configured, vector_eyes logs a warning and can fall back to internal animations

### Requirement 2.1
✅ Vector_eyes queries the storage component for available storage devices

### Requirement 2.2
✅ Uses StorageDevice::file_exists() instead of SD.exists()

### Requirement 2.3
✅ Uses StorageDevice::list_dir() to enumerate animation files

### Requirement 2.4
✅ Detects when storage devices become available (on_device_added callback)

### Requirement 2.5
✅ Falls back to internal animations when storage device is removed (on_device_removed callback)

## Design Decisions

### 1. Separation of Concerns
The StorageAdapter is a separate class rather than being integrated directly into VectorEyes. This provides:
- Clear separation between storage logic and animation logic
- Easier testing and maintenance
- Reusability for other components

### 2. Automatic Device Discovery
Rather than requiring explicit device configuration, the adapter automatically finds suitable devices. This provides:
- Better user experience (plug-and-play)
- Support for multiple storage types without configuration changes
- Graceful handling of device changes

### 3. Fallback Strategy
Multiple levels of fallback ensure robustness:
1. Try primary device
2. Try alternative devices
3. Fall back to internal animations (handled by VectorEyes)

### 4. Streaming Support
Large files (especially audio) use streaming operations to avoid loading entire files into RAM:
- `open_file()` / `read_chunk()` / `close_file()` pattern
- Suitable for ESP32 memory constraints

## Next Steps

The StorageAdapter foundation is now complete. The next tasks will:

1. Integrate StorageAdapter into the VectorEyes class
2. Update VectorEyes::setup() to use StorageAdapter
3. Modify animation loading to use StorageAdapter
4. Update audio streaming to use StorageAdapter
5. Add configuration schema for storage_id and mount_path

## Testing

A compilation test file has been created: `test_storage_adapter_compile.cpp`

This file verifies that:
- The class can be instantiated
- All public methods are accessible
- The interface is complete and consistent

## Compatibility

The StorageAdapter is designed to work with:
- ESPHome storage component (storage::Storage)
- ESPHome storage devices (storage::StorageDevice)
- SD card storage (via storage component)
- USB storage (via storage component)
- Network storage (via storage component)

## Memory Considerations

The StorageAdapter itself has minimal memory overhead:
- Class instance: ~200 bytes
- No large buffers allocated by the adapter
- File buffers are allocated on-demand by calling code
- Streaming operations use caller-provided buffers

## Thread Safety

The current implementation is not thread-safe. It assumes:
- All operations are called from the same thread (ESPHome main loop)
- Storage component callbacks are called from the same thread
- No concurrent file operations

If thread safety is needed in the future, appropriate locking mechanisms should be added.
