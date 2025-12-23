# Critical Fix Applied

## Issue Found

The boot logs showed:
```
[C][storage:070]:   Mounts configured: 0
[C][storage:075]:   Device nodes configured: 0
```

This meant the `sd_storage` component was NOT registering with the `storage` component.

## Root Cause

Component loading order issue:
1. The `storage` component's `to_code()` runs first
2. It looks for `CORE.data["sd_storage_devices"]` to register callbacks
3. But `sd_storage` hasn't run its `to_code()` yet, so the list doesn't exist
4. Result: SD card never gets registered with storage component

## Fix Applied

Changed `esphome/components/sd_storage/__init__.py`:

```python
# BEFORE:
AUTO_LOAD = []

# AFTER:
AUTO_LOAD = ["storage"]
```

This ensures the `storage` component is loaded as a dependency of `sd_storage`, which should fix the registration order.

## Next Steps

1. Recompile the firmware
2. Upload to device
3. Capture boot logs again
4. Should now see:
   - "Mounts configured: 1" (or more)
   - "Device nodes configured: 1" (or more)
   - SD card mounted successfully
   - Animations found

## Compiling Now...
