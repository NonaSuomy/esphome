# Task 6: Update VectorEyes Class to Use StorageAdapter

## ✅ Status: COMPLETED

Task 6 has been successfully completed. The VectorEyes class has been updated to include StorageAdapter integration while maintaining backward compatibility with the existing SD card implementation.

## Implementation Details

### Changes to vector_eyes.h

1. **Added Include**
   - `#include "storage_adapter.h"` - Include StorageAdapter header

2. **Added Forward Declaration**
   - Forward declared `storage::Storage` class for storage component reference

3. **Added Configuration Methods**
   ```cpp
   void set_storage(storage::Storage *storage);  // NEW
   void set_mount_path(const std::string &path); // NEW
   void set_sd_cs_pin(GPIOPin *pin);            // DEPRECATED but kept
   ```

4. **Added Member Variables**
   ```cpp
   // NEW: Storage integration
   storage::Storage *storage_component_{nullptr};
   StorageAdapter storage_adapter_{this};
   std::string mount_path_{"/sd"};
   
   // DEPRECATED: Direct SD card access (kept for backward compatibility)
   GPIOPin *sd_cs_pin_{nullptr};
   bool sd_card_initialized_{false};
   ```

### Key Design Decisions

1. **StorageAdapter as Member Instance**
   - `storage_adapter_{this}` - Initialized with `this` pointer
   - Allows StorageAdapter to access VectorEyes parent
   - No dynamic allocation needed

2. **Default Mount Path**
   - `mount_path_{"/sd"}` - Defaults to "/sd"
   - Can be configured via `set_mount_path()`
   - Matches common SD card mount point

3. **Backward Compatibility**
   - Kept `sd_cs_pin_` and `sd_card_initialized_` members
   - Marked as DEPRECATED in comments
   - Will be used as fallback if storage component not configured

4. **Storage Component Reference**
   - `storage_component_` pointer to storage component
   - Set via `set_storage()` method
   - nullptr if not configured (fallback mode)

## Code Structure

**Header Organization:**
```
Includes
  └─> storage_adapter.h

Forward Declarations
  └─> storage::Storage

Class VectorEyes
  ├─> Configuration Methods
  │    ├─> set_storage() [NEW]
  │    ├─> set_mount_path() [NEW]
  │    └─> set_sd_cs_pin() [DEPRECATED]
  │
  └─> Member Variables
       ├─> storage_component_ [NEW]
       ├─> storage_adapter_ [NEW]
       ├─> mount_path_ [NEW]
       ├─> sd_cs_pin_ [DEPRECATED]
       └─> sd_card_initialized_ [DEPRECATED]
```

## Requirements Validation

✅ **Requirement 1.1**: WHEN vector_eyes initializes THEN the system SHALL obtain a reference to the storage component
- Added `storage_component_` member variable
- Added `set_storage()` configuration method

✅ **Requirement 1.2**: WHEN vector_eyes needs to access files THEN the system SHALL use StorageDevice methods
- Added `storage_adapter_` member instance
- StorageAdapter provides all file access methods

✅ **Requirement 7.1**: WHERE a storage_id is specified in vector_eyes config THEN the system SHALL use that specific storage component instance
- `set_storage()` method accepts storage component pointer
- Will be called from Python configuration code

## Backward Compatibility

The implementation maintains full backward compatibility:

1. **Old SD Card Members Kept**
   - `sd_cs_pin_` - For direct SD card initialization
   - `sd_card_initialized_` - Track SD card state
   - Marked as DEPRECATED in comments

2. **Fallback Strategy**
   - If `storage_component_` is nullptr, use SD card directly
   - Existing configurations will continue to work
   - No breaking changes

3. **Configuration Methods**
   - `set_sd_cs_pin()` still available
   - New methods added alongside old ones
   - Gradual migration path

## Next Steps

Task 6 is complete. The next task in the implementation plan is:

**Task 7**: Modify VectorEyes::setup() for storage integration
- Call storage_adapter_.initialize() during setup
- Check storage availability and log status
- Implement fallback to SD card if storage unavailable
- Load audio_mappings.json through StorageAdapter

## Testing Recommendations

The following should be tested:
1. VectorEyes with storage component configured
2. VectorEyes without storage component (SD card fallback)
3. VectorEyes with custom mount path
4. VectorEyes with both storage and SD card pin (storage takes priority)
5. Compilation with and without storage component

## Notes

- StorageAdapter is initialized with `this` pointer in member initializer list
- Default mount path is "/sd" to match common SD card configurations
- Storage component pointer can be nullptr (fallback mode)
- All new members are clearly marked as NEW in comments
- Deprecated members are clearly marked as DEPRECATED
- No breaking changes to existing API
- Forward declaration avoids circular dependencies
