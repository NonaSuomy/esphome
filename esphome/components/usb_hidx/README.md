# USB HIDX Component

A robust ESPHome component for USB HID device support on ESP32-S3 with hot-plug capability.

## Features

- Auto-detection of USB HID devices
- Hot-plug support (plug/unplug devices without restart)
- Multi-device support (up to 4 devices simultaneously)
- Plugin-style device drivers
- Supports keyboards, mice, gamepads, and custom HID devices

## Hardware Requirements

- ESP32-S3 with USB host support
- USB hub (optional, for multiple devices)

## Limitations

- ESP32-S3 supports maximum 8 USB channels
- Practical limit: 1 hub + 2-3 HID devices

## Configuration

### Basic Setup

```yaml
esphome:
  name: usb-hid-monitor

esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_USB_HOST_ENABLE: y
      CONFIG_USB_HOST_HCD_DWC_NUM_CHANNELS: "8"
      CONFIG_ESP32S3_BROWNOUT_DET: "n"

usb_host:
  enable_hubs: true

usb_hidx:
  hub: true
```

### Keyboard Device

```yaml
usb_hidx:
  hub: true

text_sensor:
  - platform: usb_hidx
    keyboard:
      device_id: keyboard_001
      layout: US
      name: "Keyboard Input"
```

### Mouse Device

```yaml
usb_hidx:
  hub: true

binary_sensor:
  - platform: usb_hidx
    mouse:
      device_id: mouse_001
      left_button:
        name: "Mouse Left"
      right_button:
        name: "Mouse Right"

sensor:
  - platform: usb_hidx
    mouse:
      device_id: mouse_001
      x_position:
        name: "Mouse X"
      y_position:
        name: "Mouse Y"
```

## Device Drivers

Device drivers are automatically loaded based on detected USB VID/PID and HID protocol.

### Supported Devices

- **Keyboards**: HID protocol 0x01
- **Mice**: HID protocol 0x02
- **Gamepads**: Generic HID devices

### Creating Custom Device Drivers

1. Create a new directory under `devices/`
2. Implement `HIDDeviceDriver` interface
3. Register driver in component initialization

## Architecture

```
usb_hidx/
├── __init__.py              # Component configuration
├── usb_hidx.h               # Core component header
├── usb_hidx.cpp             # Core component implementation
└── devices/                 # Device drivers
    ├── hid_device_driver.h  # Base driver interface
    ├── keyboard/            # Keyboard driver
    ├── mouse/               # Mouse driver
    └── gamepad/             # Gamepad driver
```

## Technical Details

- Uses ESP-IDF USB Host API
- Implements proper device lifecycle management
- Per-device transfer handling
- Automatic interface release on disconnect
