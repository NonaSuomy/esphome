# Migration Guide: usb_hidx_minimal.h → usb_hidx Component

## Overview

This guide helps you migrate from the header-based USB HID implementation to the new modular component architecture.

## Key Benefits of New Architecture

1. **Modular Design**: Device drivers are separate, maintainable modules
2. **Auto-detection**: Devices automatically matched to appropriate drivers
3. **No Manual Setup**: No need for custom lambdas or includes
4. **Type Safety**: Proper ESPHome component integration
5. **Extensible**: Easy to add new device types

## Migration Steps

### Step 1: Remove Old Configuration

**Old (esp32-usb-hub.yaml):**
```yaml
esphome:
  includes:
    - usb_hidx_minimal.h
  on_boot:
    - priority: 600
      then:
        - lambda: |-
            setup_usb_keyboard();

globals:
  - id: keyboard_esc_pressed
    type: bool
    initial_value: 'false'

interval:
  - interval: 10ms
    then:
      - lambda: |-
          process_usb_events();
```

### Step 2: Add New Component

**New:**
```yaml
usb_hidx:
  hub: true
```

### Step 3: Convert Sensors

**Old:**
```yaml
text_sensor:
  - platform: template
    name: "Keyboard Input"
    id: keyboard_input
    update_interval: never

binary_sensor:
  - platform: template
    id: keyboard_esc_sensor
    name: "Keyboard ESC"
    lambda: return id(keyboard_esc_pressed);
```

**New:**
```yaml
text_sensor:
  - platform: usb_hidx
    keyboard:
      device_id: keyboard_001
      name: "Keyboard Input"
```

### Step 4: Remove Manual Globals

All device state is now managed internally by the component. Remove all manual globals like:
- `caps_lock_state`
- `keyboard_esc_pressed`
- `mouse_left_button`
- etc.

### Step 5: Remove Manual Intervals

The component handles USB event processing automatically. Remove:
```yaml
interval:
  - interval: 10ms
    then:
      - lambda: |-
          process_usb_events();
```

## Complete Example

### Before (Old Approach)

```yaml
esphome:
  name: esp32-usb-hub
  includes:
    - usb_hidx_minimal.h
  on_boot:
    - priority: 600
      then:
        - lambda: setup_usb_keyboard();

globals:
  - id: keyboard_esc_pressed
    type: bool
    initial_value: 'false'

text_sensor:
  - platform: template
    name: "Keyboard Input"
    id: keyboard_input

interval:
  - interval: 10ms
    then:
      - lambda: process_usb_events();
```

### After (New Component)

```yaml
esphome:
  name: esp32-usb-hub

usb_hidx:
  hub: true

text_sensor:
  - platform: usb_hidx
    keyboard:
      device_id: keyboard_001
      name: "Keyboard Input"
```

## Device-Specific Migration

### Keyboards

**Old:** Manual key tracking in globals
**New:** Automatic key event publishing to text sensors

### Mice

**Old:** Manual button/movement tracking
**New:** Binary sensors for buttons, numeric sensors for movement

### Gamepads

**Old:** Manual button state globals
**New:** Binary sensors for each button, numeric sensors for axes

## Backward Compatibility

The old `usb_hidx_minimal.h` approach will continue to work but is deprecated. New features will only be added to the component architecture.

## Troubleshooting

### Component Not Found

Ensure the component is in the correct location:
```
esphome/components/usb_hidx/
```

### Device Not Detected

Check logs for VID:PID and ensure appropriate driver exists in `devices/` folder.

### Multiple Devices

Each device needs a unique `device_id`:
```yaml
text_sensor:
  - platform: usb_hidx
    keyboard:
      device_id: keyboard_001
      name: "Keyboard 1"
  - platform: usb_hidx
    keyboard:
      device_id: keyboard_002
      name: "Keyboard 2"
```
