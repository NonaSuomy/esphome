# USB HIDX Component Structure

## Directory Layout

```
esphome/components/usb_hidx/
├── __init__.py                      # Component registration & config
├── usb_hidx.h                       # Core component header
├── usb_hidx.cpp                     # Core component implementation
├── manifest.json                    # Component metadata
├── README.md                        # Documentation
├── MIGRATION.md                     # Migration guide from old approach
├── example.yaml                     # Example configuration
└── devices/                         # Device driver plugins
    ├── hid_device_driver.h          # Base driver interface
    └── keyboard/                    # Keyboard driver
        ├── __init__.py              # Keyboard config
        ├── keyboard_device.h        # Keyboard header
        └── keyboard_device.cpp      # Keyboard implementation
```

## Component Architecture

### Core Component (`usb_hidx.cpp/h`)

**Responsibilities:**
- USB host client registration
- Device enumeration and detection
- Device lifecycle management (connect/disconnect)
- Transfer management
- Device driver routing

**Key Classes:**
- `USBHIDXComponent`: Main component class
- `HIDDevice`: Device tracking structure

### Device Drivers (`devices/`)

**Base Interface (`hid_device_driver.h`):**
```cpp
class HIDDeviceDriver {
  virtual bool match_device(vid, pid, protocol);
  virtual void setup_device(device, client);
  virtual void process_report(data, len);
  virtual void device_removed();
};
```

**Driver Plugins:**
- Each device type has its own subdirectory
- Implements `HIDDeviceDriver` interface
- Auto-registered based on VID/PID/protocol matching
- Independent sensor/entity management

### Configuration Flow

1. User defines `usb_hidx:` in YAML
2. Component initializes USB host client
3. User defines device-specific sensors (keyboard, mouse, etc.)
4. Device drivers register with component
5. On device connect:
   - Component detects device
   - Matches to appropriate driver
   - Driver sets up transfers and sensors
6. On device disconnect:
   - Driver cleanup called
   - Resources released
   - Sensors marked unavailable

## Adding New Device Types

### 1. Create Device Directory

```bash
mkdir -p devices/my_device/
```

### 2. Create Python Config (`devices/my_device/__init__.py`)

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from ... import usb_hidx_ns

MyDevice = usb_hidx_ns.class_("MyDevice", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MyDevice),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
```

### 3. Create Driver Header (`devices/my_device/my_device.h`)

```cpp
#pragma once
#include "../hid_device_driver.h"

namespace esphome {
namespace usb_hidx {

class MyDevice : public Component, public HIDDeviceDriver {
 public:
  void setup() override;
  bool match_device(uint16_t vid, uint16_t pid, uint8_t protocol) override;
  void setup_device(HIDDevice *device, usb_host_client_handle_t client) override;
  void process_report(const uint8_t *data, size_t len) override;
  void device_removed() override;
};

}  // namespace usb_hidx
}  // namespace esphome
```

### 4. Create Driver Implementation (`devices/my_device/my_device.cpp`)

```cpp
#include "my_device.h"
#include "esphome/core/log.h"

namespace esphome {
namespace usb_hidx {

static const char *TAG = "usb_hidx.my_device";

bool MyDevice::match_device(uint16_t vid, uint16_t pid, uint8_t protocol) {
  // Return true if this driver handles the device
  return vid == 0x1234 && pid == 0x5678;
}

void MyDevice::setup_device(HIDDevice *device, usb_host_client_handle_t client) {
  this->device_ = device;
  // Setup transfers, claim interfaces, etc.
}

void MyDevice::process_report(const uint8_t *data, size_t len) {
  // Parse HID report and update sensors
}

void MyDevice::device_removed() {
  // Cleanup
}

}  // namespace usb_hidx
}  // namespace esphome
```

## Design Principles

1. **Separation of Concerns**: Core component handles USB, drivers handle device-specific logic
2. **Plugin Architecture**: New devices don't modify core code
3. **Auto-detection**: Devices automatically matched to drivers
4. **Hot-plug Support**: Full connect/disconnect lifecycle management
5. **ESPHome Integration**: Proper component/sensor patterns
6. **Minimal Configuration**: Users specify what they want, not how it works

## Future Enhancements

- [ ] Mouse driver implementation
- [ ] Gamepad driver implementation
- [ ] Custom HID descriptor parsing
- [ ] Device-specific VID/PID matching
- [ ] Multiple instances of same device type
- [ ] Device capability discovery
- [ ] Advanced keyboard layouts
- [ ] Consumer control keys
- [ ] LED output support (Caps Lock, etc.)
