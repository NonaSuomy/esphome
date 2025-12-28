# Sixaxis Quick Test Guide

## Setup

1. Flash the updated firmware to your ESP32-S3
2. Connect Sixaxis controller via USB
3. Open ESPHome logs

## Expected Behavior

### On Connection
```
[usb_hidx] New USB device detected at address X
[usb_hidx] Device VID:PID = 054C:0268
[usb_hidx.playstation] Sixaxis controller connected - waiting for PS button press
```

### After Pressing PS Button
```
[usb_hidx.playstation] Initializing Sixaxis USB controller
[usb_hidx.playstation] Sixaxis LEDs set to 0x02
```

### Button Presses
Each button press should log:
```
[usb_hidx.playstation] Cross
[usb_hidx.playstation] Circle
[usb_hidx.playstation] Square
[usb_hidx.playstation] Triangle
[usb_hidx.playstation] L1
[usb_hidx.playstation] R1
[usb_hidx.playstation] L2
[usb_hidx.playstation] R2
[usb_hidx.playstation] Select
[usb_hidx.playstation] Start
[usb_hidx.playstation] L3
[usb_hidx.playstation] R3
[usb_hidx.playstation] PS Button
[usb_hidx.playstation] Up
[usb_hidx.playstation] Down
[usb_hidx.playstation] Left
[usb_hidx.playstation] Right
```

### Analog Sticks
Moving sticks should log:
```
[usb_hidx.playstation] Left Stick: X=XXX Y=YYY
[usb_hidx.playstation] Right Stick: X=XXX Y=YYY
```

## Troubleshooting

### Controller Not Detected
- Check USB cable (must support data, not just charging)
- Try different USB port
- Check logs for "Device VID:PID"

### No Response After Connection
- **Press the PS button!** Controller requires activation
- Check for "waiting for PS button press" message

### Buttons Not Working
- Verify report format in logs
- Check for "RAW:" debug messages
- Ensure report is 49 bytes

### LEDs Not Lighting
- Check for "Sixaxis LEDs set" message
- Verify interrupt OUT endpoint is available
- Some clone controllers may not support LEDs

## Testing Rumble

Add this to your YAML:
```yaml
binary_sensor:
  - platform: usb_hidx
    name: "PS3 Cross"
    type: gamepad
    button_cross: true
    on_press:
      - lambda: |-
          auto *ps = id(usb_hidx_component).get_playstation_driver();
          if (ps) ps->send_rumble(nullptr, 255, 0);
    on_release:
      - lambda: |-
          auto *ps = id(usb_hidx_component).get_playstation_driver();
          if (ps) ps->send_rumble(nullptr, 0, 0);
```

Press Cross button - should feel rumble.

## Common Issues

1. **Clone Controllers**: May have different initialization requirements
2. **Wireless Receivers**: Not supported (different protocol)
3. **Bluetooth**: Not yet implemented
4. **Battery Status**: Not yet exposed to sensors

## Debug Mode

Enable verbose logging:
```yaml
logger:
  level: VERBOSE
  logs:
    usb_hidx: VERBOSE
    usb_hidx.playstation: VERBOSE
```

This will show all raw report data.
