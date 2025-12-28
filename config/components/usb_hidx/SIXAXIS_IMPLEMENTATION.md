# PlayStation Sixaxis Controller Implementation

## Overview
This document describes the implementation of PS3 Sixaxis controller support in the ESPHome USB HIDX component, based on the Linux kernel's `hid-sony.c` driver.

## Device Identification
- **Vendor ID**: 0x054C (Sony)
- **Product ID**: 0x0268 (Sixaxis/DualShock 3)

## Initialization Sequence

### USB Mode
The Sixaxis requires a special initialization sequence to become operational:

1. **GET_REPORT 0xF2** (17 bytes) - Required for operational mode
2. **GET_REPORT 0xF5** (8 bytes) - For compatible controllers (Speedlink, Gasia)
3. **Set LED** - Indicate player number (LED 1 = Player 1)
4. **Deferred Activation** - Controller won't send reports until PS button is pressed

### Key Differences from Xbox 360
- Uses **interrupt OUT endpoint** for output reports (not control transfers)
- Requires **deferred initialization** (waits for PS button press)
- Reports are **49 bytes** (vs 20 bytes for Xbox 360)
- No report ID in output endpoint data

## Report Format

### Input Report (49 bytes)
```
Byte 0:    Report ID (0x01)
Byte 1:    Status (0xFF = invalid Bluetooth report, ignore)
Bytes 2-3: Button states (16-bit little-endian)
Byte 4:    PS button (bit 0)
Bytes 6-9: Analog sticks (LX, LY, RX, RY) - 0-255, 128=center
Bytes 41-46: Accelerometer data (big-endian, needs byte swapping)
Byte 30:   Battery status
```

### Button Mapping (Bytes 2-3)
```
Bit 0  (0x0001): Select
Bit 1  (0x0002): L3 (Left stick button)
Bit 2  (0x0004): R3 (Right stick button)
Bit 3  (0x0008): Start
Bit 4  (0x0010): D-Pad Up
Bit 5  (0x0020): D-Pad Right
Bit 6  (0x0040): D-Pad Down
Bit 7  (0x0080): D-Pad Left
Bit 8  (0x0100): L2
Bit 9  (0x0200): R2
Bit 10 (0x0400): L1
Bit 11 (0x0800): R1
Bit 12 (0x1000): Triangle
Bit 13 (0x2000): Circle
Bit 14 (0x4000): Cross
Bit 15 (0x8000): Square
```

PS Button is in Byte 4, Bit 0 (separate from main button word)

### Output Report (36 bytes)
```
Byte 0:     Report ID (0x01)
Byte 1:     Padding
Byte 2:     Right motor (0x00=off, 0x01=on)
Byte 3:     Padding
Byte 4:     Left motor force (0-255)
Bytes 5-9:  Padding
Byte 10:    LED bitmap (0x02=LED1, 0x04=LED2, 0x08=LED3, 0x10=LED4)
Bytes 11-35: LED configuration (5 bytes per LED × 4 LEDs + 5 reserved)
```

LED Configuration (5 bytes per LED):
- Byte 0: Time enabled (0xFF = forever)
- Byte 1: Duty cycle length (deciseconds, 0 = fast)
- Byte 2: Enabled flag
- Byte 3: Duty off percentage (0xFF = 100%)
- Byte 4: Duty on percentage (0xFF = 100%)

Default LED values: `0xFF, 0x27, 0x10, 0x00, 0x32`

## Implementation Details

### Driver Structure
The `PlayStationDriver` class in `playstation_driver.h`:
- Matches device by VID/PID
- Implements deferred initialization
- Processes 49-byte input reports
- Sends output reports via interrupt OUT endpoint
- Supports rumble and LED control

### Key Functions

#### `on_device_ready()`
Called when device is first detected. Sets `defer_init_` flag to wait for PS button press.

#### `process_report()`
- Validates report (49 bytes, ID 0x01)
- Ignores invalid Bluetooth reports (byte 1 = 0xFF)
- Triggers initialization on first valid report
- Parses buttons and analog sticks

#### `init_sixaxis_usb()`
Sends initialization sequence:
1. GET_REPORT 0xF2
2. GET_REPORT 0xF5
3. Set LED to Player 1

#### `send_rumble()`
Sends 36-byte output report with rumble values and LED state.

### Special Handling

1. **Invalid Reports**: Bluetooth mode occasionally sends reports with byte 1 = 0xFF (all zeros). These must be ignored.

2. **Byte Swapping**: Accelerometer data (bytes 41-48) is big-endian and needs swapping:
   ```cpp
   swap(data[41], data[42]);  // X axis
   swap(data[43], data[44]);  // Y axis
   swap(data[45], data[46]);  // Z axis
   ```

3. **Output Endpoint**: Unlike Xbox 360, Sixaxis uses interrupt OUT endpoint, not control transfers.

4. **LED Persistence**: LEDs must be set in every output report to maintain state.

## YAML Configuration

```yaml
usb_hidx:
  id: usb_hidx_component
  gamepad:
    type: ps3

binary_sensor:
  - platform: usb_hidx
    name: "PS3 Cross"
    type: gamepad
    button_cross: true
  - platform: usb_hidx
    name: "PS3 Circle"
    type: gamepad
    button_circle: true
```

## Testing

1. Connect Sixaxis via USB
2. Press PS button to activate controller
3. Check logs for "Sixaxis controller connected"
4. Test buttons - should see log messages
5. Test rumble via button press actions

## Known Issues

1. **Shanwan Clones**: Some clones require skipping step 3 of initialization (USB interrupt)
2. **Bluetooth**: Not yet implemented (requires different initialization)
3. **Accelerometer**: Data available but not yet exposed to ESPHome sensors

## Future Enhancements

1. Add accelerometer sensor support
2. Add gyroscope sensor support (if available)
3. Implement Bluetooth mode support
4. Add battery level sensor
5. Support for Navigation Controller (partial DS3)
6. Support for PS Move Motion Controller

## References

- Linux kernel `drivers/hid/hid-sony.c`
- USB HID specification
- PlayStation 3 controller documentation
