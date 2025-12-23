# Task 12: Configuration Schema Updates Summary

## Overview
Updated the Python configuration schema for the vector_eyes component to support the new storage component integration while maintaining full backward compatibility with existing SD card configurations.

## Requirements Addressed
- **Requirement 7.1**: Support storage_id configuration
- **Requirement 7.2**: Support mount_path configuration
- **Requirement 7.3**: Keep cs_pin for backward compatibility
- **Requirement 7.4**: Configuration validation

## Implementation Details

### 1. New Configuration Options
**Location**: `__init__.py`

Added two new configuration options:

```python
CONF_STORAGE_ID = 'storage_id'
CONF_MOUNT_PATH = 'mount_path'
```

**storage_id** (Optional):
- Type: Component reference
- Purpose: Reference to a storage component instance
- Usage: Links vector_eyes to a configured storage component

**mount_path** (Optional):
- Type: String
- Default: "/sd"
- Purpose: Specifies the mount path where animations are located
- Usage: Allows customization of animation directory location

### 2. Updated Schema
**Location**: `__init__.py` - `CONFIG_SCHEMA`

```python
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(VectorEyes),
    cv.Required(CONF_DISPLAY_ID): cv.use_id(display.DisplayBuffer),
    cv.Optional(CONF_SPEAKER_ID): cv.use_id(speaker.Speaker),
    # NEW: Storage component integration
    cv.Optional(CONF_STORAGE_ID): cv.use_id(cg.Component),
    cv.Optional(CONF_MOUNT_PATH, default="/sd"): cv.string,
    # DEPRECATED: Direct SD card configuration
    cv.Optional(CONF_CS_PIN): pins.gpio_output_pin_schema,
}).extend(cv.COMPONENT_SCHEMA)
```

**Key Features**:
- All new options are optional
- Default mount_path is "/sd" for consistency
- cs_pin remains optional for backward compatibility
- No breaking changes to existing configurations

### 3. Updated Code Generation
**Location**: `__init__.py` - `to_code()`

Enhanced the code generation to handle new configuration:

```python
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # Display (required)
    disp = await cg.get_variable(config[CONF_DISPLAY_ID])
    cg.add(var.set_display(disp))
    
    # Speaker (optional)
    if CONF_SPEAKER_ID in config:
        spk = await cg.get_variable(config[CONF_SPEAKER_ID])
        cg.add(var.set_speaker(spk))
    
    # NEW: Storage component (optional, preferred)
    if CONF_STORAGE_ID in config:
        storage = await cg.get_variable(config[CONF_STORAGE_ID])
        cg.add(var.set_storage(storage))
    
    # NEW: Mount path (optional)
    if CONF_MOUNT_PATH in config:
        cg.add(var.set_mount_path(config[CONF_MOUNT_PATH]))
    
    # DEPRECATED: SD card pin (optional, for backward compatibility)
    if CONF_CS_PIN in config:
        if CONF_STORAGE_ID not in config:
            cs = await cg.gpio_pin_expression(config[CONF_CS_PIN])
            cg.add(var.set_sd_cs_pin(cs))
        else:
            # Warn if both are provided
            cg.add(cg.RawExpression(
                'ESP_LOGW("vector_eyes", "cs_pin is deprecated and ignored when storage_id is provided")'
            ))
```

**Logic**:
1. If `storage_id` is provided, use storage component
2. If `mount_path` is provided, configure mount path
3. If `cs_pin` is provided WITHOUT `storage_id`, use legacy SD card
4. If BOTH `storage_id` and `cs_pin` are provided, prefer `storage_id` and log warning

### 4. Backward Compatibility
**Requirement 7.3**: Keep cs_pin for backward compatibility

**Strategy**:
- `cs_pin` remains in schema as optional
- Existing configurations work unchanged
- No migration required for existing users
- New users should use `storage_id` instead

**Example - Old Configuration (Still Works)**:
```yaml
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  cs_pin: GPIO0  # Old style, still supported
```

**Example - New Configuration (Recommended)**:
```yaml
storage:
  id: main_storage
  mounts:
    - path: /sd
      platform: sd_direct

vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  storage_id: main_storage  # NEW: Preferred method
  mount_path: /sd           # NEW: Optional, defaults to /sd
```

**Example - Mixed Configuration (storage_id takes precedence)**:
```yaml
storage:
  id: main_storage
  mounts:
    - path: /sd
      platform: sd_direct

vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  storage_id: main_storage  # Used
  cs_pin: GPIO0             # Ignored with warning
```

### 5. Configuration Validation
**Requirement 7.4**: Configuration validation

**Validation Rules**:
1. ✅ `display_id` is required (enforced by schema)
2. ✅ `speaker_id` is optional (enforced by schema)
3. ✅ `storage_id` must reference valid component (enforced by `cv.use_id()`)
4. ✅ `mount_path` must be string (enforced by `cv.string`)
5. ✅ `cs_pin` must be valid GPIO (enforced by `pins.gpio_output_pin_schema`)
6. ✅ All options are optional except `display_id`

**Error Handling**:
- Invalid `storage_id`: ESPHome will fail at compile time with clear error
- Invalid `mount_path`: Type validation ensures it's a string
- Invalid `cs_pin`: GPIO validation ensures valid pin number
- Missing `display_id`: Schema validation requires it

## Configuration Examples

### Example 1: New Configuration with Storage Component
```yaml
storage:
  id: main_storage
  mounts:
    - path: /sd
      platform: sd_direct
      cs_pin: GPIO0

vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  storage_id: main_storage
  mount_path: /sd
```

### Example 2: Multiple Storage Devices
```yaml
storage:
  id: main_storage
  mounts:
    - path: /sd
      platform: sd_direct
      cs_pin: GPIO0
    - path: /usb
      platform: usb_storage

vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  storage_id: main_storage
  mount_path: /sd  # Prefer SD card
```

### Example 3: Custom Mount Path
```yaml
storage:
  id: main_storage
  mounts:
    - path: /data
      platform: sd_direct

vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  storage_id: main_storage
  mount_path: /data  # Custom path
```

### Example 4: Legacy Configuration (Still Works)
```yaml
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  cs_pin: GPIO0  # Old style
```

### Example 5: Minimal Configuration (No Storage)
```yaml
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  # No storage configured - uses internal animations
```

## Migration Guide

### For Existing Users (No Changes Required)
Existing configurations with `cs_pin` continue to work without modification:

```yaml
# This still works exactly as before
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  cs_pin: GPIO0
```

### For New Users (Recommended Approach)
New users should use the storage component:

```yaml
# Step 1: Configure storage component
storage:
  id: main_storage
  mounts:
    - path: /sd
      platform: sd_direct
      cs_pin: GPIO0

# Step 2: Reference storage in vector_eyes
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  storage_id: main_storage
```

### Migration Steps (Optional)
To migrate from old to new configuration:

1. **Add storage component**:
   ```yaml
   storage:
     id: main_storage
     mounts:
       - path: /sd
         platform: sd_direct
         cs_pin: GPIO0  # Move cs_pin here
   ```

2. **Update vector_eyes**:
   ```yaml
   vector_eyes:
     id: my_vector_eyes
     display_id: display1
     speaker_id: vector_speaker
     storage_id: main_storage  # Add this
     # cs_pin: GPIO0  # Remove this
   ```

3. **Test and deploy**

## Testing Performed

### Syntax Validation
✅ Python syntax is correct
✅ No diagnostic errors
✅ Schema validation is proper

### Code Review
✅ Configuration options are correctly defined
✅ Backward compatibility is maintained
✅ Code generation logic is sound
✅ Error handling is appropriate

### Expected Behavior
- ✅ New configurations with `storage_id` work
- ✅ Old configurations with `cs_pin` still work
- ✅ Mixed configurations prefer `storage_id`
- ✅ Default `mount_path` is "/sd"
- ✅ Invalid configurations fail at compile time

## Compliance Matrix

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| 7.1 - storage_id support | ✅ Complete | `CONF_STORAGE_ID` in schema |
| 7.2 - mount_path support | ✅ Complete | `CONF_MOUNT_PATH` with default |
| 7.3 - cs_pin compatibility | ✅ Complete | `CONF_CS_PIN` still optional |
| 7.4 - Validation | ✅ Complete | Schema validation + type checking |

## Benefits

### For Users
1. **Flexibility**: Choose between storage component or direct SD card
2. **Future-Proof**: Ready for USB, network, and other storage backends
3. **No Breaking Changes**: Existing configurations work unchanged
4. **Clear Migration Path**: Easy to upgrade when ready

### For Developers
1. **Unified Interface**: Single storage abstraction
2. **Better Testing**: Can mock storage component
3. **Extensibility**: Easy to add new storage backends
4. **Maintainability**: Less duplicate code

## Documentation Needs

### User Documentation
1. Update YAML configuration examples
2. Document new `storage_id` and `mount_path` options
3. Provide migration guide from `cs_pin` to `storage_id`
4. Show examples with multiple storage devices

### Developer Documentation
1. Document configuration schema changes
2. Explain backward compatibility strategy
3. Provide testing guidelines

## Future Enhancements

### 1. Deprecation Warning
Add compile-time warning for `cs_pin` usage:

```python
if CONF_CS_PIN in config and CONF_STORAGE_ID not in config:
    cg.add(cg.RawExpression(
        'ESP_LOGW("vector_eyes", "cs_pin is deprecated, please use storage_id instead")'
    ))
```

### 2. Configuration Validation
Add custom validation to ensure sensible configurations:

```python
def validate_config(config):
    if CONF_STORAGE_ID in config and CONF_CS_PIN in config:
        raise cv.Invalid("Cannot use both storage_id and cs_pin")
    return config

CONFIG_SCHEMA = cv.All(
    cv.Schema({...}),
    validate_config
)
```

### 3. Auto-Migration
Provide a migration tool to convert old configs to new format.

## Conclusion

Task 12 is **complete** with full configuration schema support:

✅ Added `storage_id` configuration option
✅ Added `mount_path` configuration option with default
✅ Maintained `cs_pin` for backward compatibility
✅ Implemented proper configuration validation
✅ Added precedence logic (storage_id > cs_pin)
✅ Generated appropriate C++ code
✅ No breaking changes to existing configurations

The implementation provides a smooth migration path from the old SD card configuration to the new storage component while maintaining full backward compatibility. Users can adopt the new configuration at their own pace without any forced changes.
