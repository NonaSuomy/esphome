# SD Card Runtime Diagnosis

## Test Date
December 8, 2025 - 22:29

## Test Results
❌ **FAILED** - ESP32 unable to read files from SD card

## Log Analysis Summary

### Captured Data
- **Total log lines**: 3,886
- **Duration**: ~30 seconds
- **Device**: /dev/ttyUSB0
- **Configuration**: vector-eyes-ttgo.yaml

### Critical Issues Found

#### 1. Severe SPI Bus Errors (CRITICAL)
- **Count**: 2,364 SPI errors in 30 seconds
- **Error types**:
  - `invalid dev handle` (spi_master errors)
  - `SPI device not ready, cannot begin transaction`
  - `Transmit failed - err 102`

**Impact**: Device appears to be in a crash/boot loop state. The SPI bus is not functioning correctly.

#### 2. No Storage Component Initialization
- **Storage messages**: 0
- **SD card messages**: 0
- **Mount messages**: 0

**Impact**: Storage component is not starting. SD card is not being detected or initialized.

#### 3. No File Operations
- **File read operations**: 0
- **Animation file access**: 0
- **Audio file access**: 0
- **JSON parsing**: 0

**Impact**: No files are being read from the SD card. The application cannot load animations or audio.

#### 4. Vector Eyes Component Not Running
- **Component messages**: 1 (only YAML config read)
- **Setup messages**: 0
- **Animation messages**: 0

**Impact**: The vector_eyes component is not initializing properly, likely due to the SPI errors causing crashes.

## Root Cause Analysis

### Primary Issue: SPI Bus Configuration Conflict

The TTGO Camera Plus board has:
- **Display (ST7789)** on SPI bus
- **SD Card** on same SPI bus
- Both devices sharing CLK, MOSI, MISO pins
- Separate CS pins for device selection

**Current Configuration**:
```yaml
spi:
  clk_pin: GPIO21
  mosi_pin: GPIO19
  miso_pin: GPIO22
  interface: hardware

display:
  cs_pin: GPIO12  # Display CS
  
sd_storage:
  cs_pin: GPIO0   # SD Card CS
```

### Possible Causes

1. **SPI Bus Initialization Order**
   - Display may be initializing before storage component
   - SD card initialization may be interfering with display
   - Both devices trying to use SPI bus simultaneously

2. **Invalid Device Handles**
   - The "invalid dev handle" errors suggest devices are not properly registered with SPI master
   - Storage component may not be creating valid SPI device handle

3. **Pin Configuration Issues**
   - GPIO0 is a strapping pin (boot mode selection)
   - Using GPIO0 for SD CS may cause boot issues
   - Display pins (GPIO2, GPIO12, GPIO15) are also strapping pins

4. **Storage Component Not Linked**
   - The storage component may not be properly compiled/linked
   - sd_storage platform may not be available in ESP-IDF framework

## Recommendations

### Immediate Actions

1. **Check Compilation**
   ```bash
   # Verify storage component is compiled
   grep -r "sd_storage" .esphome/build/vector-eyes-ttgo/
   ```

2. **Add Debug Logging**
   Add to YAML:
   ```yaml
   logger:
     level: VERBOSE
     logs:
       storage: VERBOSE
       sd_storage: VERBOSE
       spi: DEBUG
   ```

3. **Test SD Card Pin**
   - Try different CS pin (not a strapping pin)
   - Suggested: GPIO13 or GPIO27

4. **Simplify Configuration**
   - Temporarily disable display
   - Test SD card initialization alone
   - Add back display once SD works

### Configuration Changes to Try

#### Option 1: Change SD CS Pin
```yaml
substitutions:
  sd_cs: GPIO13  # Non-strapping pin
```

#### Option 2: Add Explicit SPI Device IDs
```yaml
spi:
  - id: spi_bus
    clk_pin: GPIO21
    mosi_pin: GPIO19
    miso_pin: GPIO22

display:
  spi_id: spi_bus
  cs_pin: GPIO12

sd_storage:
  spi_id: spi_bus
  cs_pin: GPIO13
```

#### Option 3: Use SD MMC Mode (if hardware supports)
```yaml
# Instead of sd_storage with SPI
sd_mmc:
  clk_pin: GPIO14
  cmd_pin: GPIO15
  d0_pin: GPIO2
```

### Testing Steps

1. **Verify Hardware**
   - Check SD card is inserted
   - Check SD card is formatted (FAT32)
   - Check SD card has files

2. **Test Minimal Config**
   - Create minimal YAML with only SD card
   - No display, no vector_eyes
   - Just storage component

3. **Check ESPHome Version**
   - Verify storage component is available
   - Check if sd_storage platform exists
   - May need to use sd_spi instead

4. **Review Component Code**
   - Check if storage_adapter.cpp is being compiled
   - Verify vector_eyes links to storage component
   - Check for missing dependencies

## Next Steps

1. ✅ Capture and analyze logs (COMPLETE)
2. ⏳ Fix SPI configuration issues
3. ⏳ Get storage component initializing
4. ⏳ Verify SD card detection
5. ⏳ Test file reading
6. ⏳ Verify animation loading

## Files Generated
- `esp32_logs.txt` - Raw log capture (289KB, 3,886 lines)
- `log_analysis.txt` - Automated analysis
- `SD_CARD_RUNTIME_DIAGNOSIS.md` - This file

## Conclusion

The ESP32 is **NOT** able to read files from the SD card. The device is experiencing severe SPI bus errors that prevent normal operation. The storage component is not initializing, and no file operations are occurring.

**Priority**: Fix SPI configuration and storage component initialization before attempting to load animations.
