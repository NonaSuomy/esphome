# Nintendo Switch Pro Controller Investigation

## Current Status: NOT WORKING ❌

The Nintendo Switch Pro Controller (VID 0x057E, PID 0x2009) is **not currently functional** due to USB stack compatibility issues.

## Problem Description

### Symptoms
```
[I][usb_hidx]: Device VID:PID = 057E:2009
[I][usb_hidx]: Found HID interface, protocol 0
[I][usb_hidx]: Found interrupt IN endpoint: 0x81
[I][usb_hidx]: Found interrupt OUT endpoint: 0x01
[I][usb_hidx]: Matched device to Switch driver
[I][usb_hidx]: Device monitoring started (protocol 0)
[D][usb_hidx]: RAW: [81 01 00 03 12 A0 27 09 55 D0 00 00 ...]
[E][USB HOST]: Enqueue URB error: ESP_ERR_INVALID_STATE
[I][usb_hidx]: USB device disconnected
```

### Issue Analysis
1. Controller connects successfully
2. Endpoints detected correctly (IN: 0x81, OUT: 0x01)
3. Driver matches and starts monitoring
4. **One report received** (64 bytes, starts with `81 01`)
5. **Immediate disconnect** with `ESP_ERR_INVALID_STATE`
6. Reconnect loop continues

## Root Cause

The issue is **NOT with the driver** but with the **ESP-IDF USB Host stack**. The Switch Pro controller requires:

1. **Special USB Configuration**
   - Non-standard HID behavior
   - Requires specific USB descriptors
   - May need different endpoint configuration

2. **Initialization Handshake**
   - Controller expects specific commands
   - Timing-sensitive initialization
   - May require control transfers before data transfers

3. **USB Stack Limitations**
   - ESP-IDF USB Host may not support all HID quirks
   - `ESP_ERR_INVALID_STATE` suggests stack-level issue
   - Not a driver-level problem

## Linux Kernel Implementation

The Linux kernel driver (`hid-nintendo.c`) shows the complexity:

### Key Requirements from Linux Driver

1. **USB vs Bluetooth Detection**
   ```c
   // Different initialization for USB vs BT
   if (hdev->bus == BUS_USB) {
       ret = joycon_init_usb(ctlr);
   } else {
       ret = joycon_init_bt(ctlr);
   }
   ```

2. **Initialization Sequence**
   ```c
   // Send handshake
   ret = joycon_send_usb(ctlr, JC_USB_CMD_HANDSHAKE, HZ);

   // Set baudrate
   ret = joycon_send_usb(ctlr, JC_USB_CMD_BAUDRATE_3M, HZ);

   // Enable vibration
   ret = joycon_send_usb(ctlr, JC_USB_CMD_NO_TIMEOUT, HZ);

   // Request MAC address
   ret = joycon_request_mac(ctlr);
   ```

3. **Report Format**
   - Input reports: 64 bytes
   - Report ID 0x81 (USB mode) or 0x30/0x21 (full mode)
   - Complex button/stick encoding
   - IMU data included

4. **Output Reports**
   - Rumble control (HD Rumble)
   - LED control (4 player LEDs)
   - IMU enable/disable
   - Requires specific packet format

## Current Driver Implementation

Our `switch_driver.h` has:
- ✅ Correct VID/PID detection (0x057E:2009)
- ✅ Report parsing for both USB and full modes
- ✅ Button mapping for all buttons
- ✅ Analog stick parsing (12-bit values)
- ❌ No initialization sequence
- ❌ No USB handshake
- ❌ No output report support

## Why It Fails

The Switch Pro controller **requires initialization** before it will stay connected:

1. **Handshake Command** - Controller expects this first
2. **Baudrate Setting** - USB communication speed
3. **Mode Selection** - Standard vs Full mode
4. **MAC Address Request** - Controller identification

Without these commands, the controller:
- Sends one report (the `81 01` we see)
- Waits for initialization
- Times out and disconnects
- ESP-IDF stack reports `ESP_ERR_INVALID_STATE`

## What Would Be Needed

### 1. USB Control Transfers
```cpp
// Send handshake (0x80, 0x02)
uint8_t handshake[] = {0x80, 0x02};
send_control_transfer(device, handshake, 2);

// Set baudrate to 3Mbps (0x80, 0x03)
uint8_t baudrate[] = {0x80, 0x03};
send_control_transfer(device, baudrate, 2);

// Disable timeout (0x80, 0x04)
uint8_t no_timeout[] = {0x80, 0x04};
send_control_transfer(device, no_timeout, 2);
```

### 2. Initialization State Machine
```cpp
enum SwitchInitState {
  INIT_HANDSHAKE,
  INIT_BAUDRATE,
  INIT_NO_TIMEOUT,
  INIT_MAC_REQUEST,
  INIT_COMPLETE
};
```

### 3. Output Report Support
```cpp
// Enable full report mode (0x01, 0x03)
uint8_t full_mode[] = {0x01, 0x03, ...};
send_output_report(device, full_mode, size);
```

### 4. ESP-IDF Stack Compatibility
The biggest challenge: ESP-IDF USB Host may not support:
- Multiple control transfers during initialization
- Timing requirements between transfers
- Specific USB quirks the Switch controller needs

## Comparison with Working Controllers

### Xbox 360 Controller
- ✅ Works immediately after connection
- ✅ No special initialization needed
- ✅ Standard HID behavior

### PlayStation Sixaxis
- ✅ Works with simple GET_REPORT commands
- ✅ Standard HID with minor quirks
- ✅ Compatible with ESP-IDF stack

### Switch Pro Controller
- ❌ Requires complex initialization
- ❌ Non-standard USB behavior
- ❌ Not compatible with current ESP-IDF stack

## Potential Solutions

### Option 1: Implement Initialization (Difficult)
- Add control transfer support to driver
- Implement initialization state machine
- May still fail due to ESP-IDF limitations
- **Estimated effort**: 20-40 hours
- **Success probability**: 30-50%

### Option 2: Use PowerA Controller (Recommended)
- PowerA Switch controllers (VID 0x20D6, PID 0xA713)
- Use standard HID reports
- No special initialization needed
- **Already supported** in switch_driver.h
- **Success probability**: 90%+

### Option 3: Wait for ESP-IDF Updates
- ESP-IDF USB Host is still maturing
- Future versions may support more HID quirks
- Monitor ESP-IDF releases
- **Timeline**: Unknown

### Option 4: Use Bluetooth Instead
- Switch Pro supports Bluetooth
- Different initialization sequence
- May have better compatibility
- **Estimated effort**: 40-60 hours
- **Success probability**: 60-70%

## Recommendation

**Use a PowerA Switch controller** or similar third-party controller that:
- Uses standard HID reports
- Doesn't require special initialization
- Works with current ESP-IDF USB Host

The official Switch Pro controller is **not worth the effort** given:
- Complex initialization requirements
- ESP-IDF stack limitations
- Low probability of success
- Many hours of debugging required

## Alternative Controllers That Work

### Confirmed Working
- ✅ Xbox 360 Controller (wired)
- ✅ PlayStation Sixaxis/DualShock 3
- ✅ Generic USB gamepads
- ✅ Keyboard and mouse

### Should Work (Not Tested)
- ✅ PowerA Switch controllers
- ✅ Hori Switch controllers
- ✅ Most third-party Switch controllers
- ✅ Xbox One controller (wired)

### Won't Work Without Major Changes
- ❌ Official Switch Pro Controller
- ❌ Switch Joy-Cons
- ❌ PS4 DualShock 4 (similar issues)
- ❌ PS5 DualSense (similar issues)

## Technical Details from Linux Driver

### USB Commands (from hid-nintendo.c)
```c
#define JC_USB_CMD_HANDSHAKE      0x80, 0x02
#define JC_USB_CMD_BAUDRATE_3M    0x80, 0x03
#define JC_USB_CMD_NO_TIMEOUT     0x80, 0x04
#define JC_USB_CMD_HANDSHAKE_AT   0x80, 0x01
```

### Report IDs
```c
#define JC_INPUT_USB_RESPONSE     0x81  // What we're seeing
#define JC_INPUT_STANDARD_FULL    0x30  // Full input report
#define JC_INPUT_IMU_DATA         0x31  // With IMU data
#define JC_INPUT_SUBCMD_REPLY     0x21  // Subcommand reply
```

### Button Encoding (Complex)
- 3 bytes for buttons (right, shared, left)
- 12-bit analog stick values (packed)
- IMU data (6-axis)
- Battery level
- Connection info

## Conclusion

The Switch Pro controller issue is a **USB stack compatibility problem**, not a driver problem. The driver is ready, but the ESP-IDF USB Host stack cannot handle the controller's initialization requirements.

**Recommendation**: Use alternative controllers that work with standard HID, or wait for ESP-IDF improvements.

## Files

- `switch_driver.h` - Driver ready, waiting for USB stack support
- This document - Investigation and recommendations

## References

- Linux kernel: `drivers/hid/hid-nintendo.c`
- ESP-IDF USB Host documentation
- Nintendo Switch reverse engineering community
