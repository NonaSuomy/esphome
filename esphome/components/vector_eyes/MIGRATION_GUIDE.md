# Vector Eyes Storage Component Migration Guide

## Overview

This guide helps you migrate from the legacy SD card configuration to the new storage component integration. **Migration is optional** - existing configurations continue to work unchanged.

## Why Migrate?

### Benefits of Storage Component
- **Multi-device support**: Use SD card, USB, or network storage
- **Hot-plug detection**: Add/remove storage without restart
- **Better error handling**: Automatic retry and fallback
- **Future-proof**: Ready for new storage backends
- **Unified interface**: Consistent API across all storage types

### When to Migrate
- ✅ You want to use multiple storage devices
- ✅ You want hot-plug support
- ✅ You're setting up a new system
- ✅ You want better error handling
- ❌ Your current setup works fine (no need to change)

## Migration Steps

### Step 1: Understand Your Current Configuration

**Legacy Configuration** (still works):
```yaml
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  cs_pin: GPIO0  # Direct SD card pin
```

### Step 2: Add Storage Component

Add the storage component to your YAML:

```yaml
# Add this section
storage:
  id: main_storage
  mounts:
    - path: /sd
      platform: sd_direct
      cs_pin: GPIO0  # Move cs_pin here
```

### Step 3: Update Vector Eyes Configuration

Update vector_eyes to use the storage component:

```yaml
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  storage_id: main_storage  # NEW: Reference storage component
  mount_path: /sd           # NEW: Optional, defaults to /sd
  # cs_pin: GPIO0  # REMOVE: No longer needed here
```

### Step 4: Test and Deploy

1. **Validate configuration**: `esphome config your-device.yaml`
2. **Upload to device**: `esphome upload your-device.yaml`
3. **Check logs**: Verify storage initialization
4. **Test animations**: Ensure animations play correctly

## Configuration Examples

### Example 1: Simple SD Card Migration

**Before**:
```yaml
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  cs_pin: GPIO0
```

**After**:
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
```

### Example 2: SD Card + USB Storage

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
  mount_path: /sd  # Prefer SD, fall back to USB
```

### Example 3: Network Storage with Local Fallback

```yaml
storage:
  id: main_storage
  mounts:
    - path: /network
      platform: network_storage
      host: 192.168.1.100
      share: /animations
    - path: /sd
      platform: sd_direct
      cs_pin: GPIO0

vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  storage_id: main_storage
  mount_path: /network  # Try network first, fall back to SD
```

### Example 4: Multiple Vector Eyes Instances

```yaml
storage:
  id: storage1
  mounts:
    - path: /sd1
      platform: sd_direct
      cs_pin: GPIO0

storage:
  id: storage2
  mounts:
    - path: /sd2
      platform: sd_direct
      cs_pin: GPIO5

vector_eyes:
  id: vector_eyes_1
  display_id: display1
  speaker_id: speaker1
  storage_id: storage1  # Use first storage

vector_eyes:
  id: vector_eyes_2
  display_id: display2
  speaker_id: speaker2
  storage_id: storage2  # Use second storage
```

## Backward Compatibility

### Legacy Configurations Still Work

You don't need to migrate. Your existing configuration will continue to work:

```yaml
# This still works exactly as before
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  cs_pin: GPIO0
```

### Mixed Configurations

If you provide both `storage_id` and `cs_pin`, the storage component takes precedence:

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
  cs_pin: GPIO0             # Ignored (warning logged)
```

**Log output**:
```
[W][vector_eyes:XX]: cs_pin is deprecated and ignored when storage_id is provided
```

## Troubleshooting

### Issue: "Storage component is null"

**Symptom**:
```
[W][vector_eyes.storage_adapter:XX]: Storage component is null, storage adapter will not be available
```

**Solution**: Check that `storage_id` references a valid storage component:
```yaml
storage:
  id: main_storage  # Must match storage_id below
  mounts:
    - path: /sd
      platform: sd_direct

vector_eyes:
  storage_id: main_storage  # Must match id above
```

### Issue: "No storage device with animations found"

**Symptom**:
```
[W][vector_eyes.storage_adapter:XX]: No storage device with animations found, will fall back to internal animations
```

**Solution**: Ensure your SD card has an `/animations` directory:
```
/sd/
  ├── animations/
  │   ├── anim_happy_01.json
  │   ├── anim_blink_01.json
  │   └── ...
  └── audio/
      ├── happy.wav
      └── ...
```

### Issue: "Failed to read file from any storage device"

**Symptom**:
```
[E][vector_eyes.storage_adapter:XX]: Failed to read file from any storage device: /sd/animations/anim_happy.json
```

**Solutions**:
1. Check file exists on SD card
2. Verify SD card is properly formatted (FAT32)
3. Check file permissions
4. Try re-inserting SD card
5. Check logs for SD card initialization errors

### Issue: Animations not playing after migration

**Symptom**: No errors, but animations don't play

**Solutions**:
1. Check `mount_path` matches your storage mount point
2. Verify animations directory structure
3. Check logs for file access errors
4. Ensure SD card is properly initialized

### Issue: "Slow operation" warnings

**Symptom**:
```
[W][vector_eyes.storage_adapter:XX]: Slow operation: read_file took 150 ms
```

**Solutions**:
1. Normal for network storage (expected)
2. For SD card: Check card speed (use Class 10 or better)
3. For SD card: Check wiring and connections
4. Consider using local storage for frequently accessed files

## Rollback Procedure

If you need to roll back to the legacy configuration:

### Step 1: Remove Storage Component

```yaml
# Remove or comment out
# storage:
#   id: main_storage
#   mounts:
#     - path: /sd
#       platform: sd_direct
#       cs_pin: GPIO0
```

### Step 2: Restore cs_pin

```yaml
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  cs_pin: GPIO0  # Restore this
  # storage_id: main_storage  # Remove this
  # mount_path: /sd  # Remove this
```

### Step 3: Upload and Test

```bash
esphome upload your-device.yaml
```

## Best Practices

### 1. Test Before Deploying

Always test the new configuration on a development device before deploying to production.

### 2. Keep Backups

Keep a backup of your working configuration:
```bash
cp your-device.yaml your-device.yaml.backup
```

### 3. Monitor Logs

After migration, monitor logs for the first few hours:
```bash
esphome logs your-device.yaml
```

### 4. Use Preferred Mount Path

Specify `mount_path` to control device priority:
```yaml
vector_eyes:
  storage_id: main_storage
  mount_path: /sd  # Prefer SD card
```

### 5. Plan for Fallback

Configure multiple storage devices for redundancy:
```yaml
storage:
  id: main_storage
  mounts:
    - path: /sd
      platform: sd_direct
    - path: /usb
      platform: usb_storage
```

## FAQ

### Q: Do I have to migrate?
**A**: No, migration is optional. Legacy configurations continue to work.

### Q: Will my animations still work?
**A**: Yes, the same animation files work with both configurations.

### Q: Can I use both old and new configurations?
**A**: Yes, but storage_id takes precedence if both are provided.

### Q: What if I don't have a storage component?
**A**: Vector Eyes will fall back to internal animations.

### Q: Can I migrate gradually?
**A**: Yes, migrate one device at a time to minimize risk.

### Q: Will this break my Home Assistant integration?
**A**: No, Home Assistant controls work identically.

### Q: Do I need to reformat my SD card?
**A**: No, existing SD card contents work unchanged.

### Q: Can I switch back to the old configuration?
**A**: Yes, simply restore the `cs_pin` and remove `storage_id`.

## Support

### Getting Help

1. **Check logs**: Most issues are visible in logs
2. **Review documentation**: Check SD_CARD_SETUP.md
3. **Community forum**: Ask on ESPHome forums
4. **GitHub issues**: Report bugs on GitHub

### Useful Log Commands

```bash
# View live logs
esphome logs your-device.yaml

# Filter for storage-related logs
esphome logs your-device.yaml | grep storage

# Filter for vector_eyes logs
esphome logs your-device.yaml | grep vector_eyes
```

## Summary

- ✅ Migration is **optional** - legacy configs still work
- ✅ Migration provides **multi-device support** and **hot-plug**
- ✅ Process is **straightforward** - add storage component, update vector_eyes
- ✅ **Rollback is easy** - restore cs_pin, remove storage_id
- ✅ **No data migration** needed - same files work with both configs

**Recommendation**: Migrate when you need new features, otherwise keep existing configuration.
