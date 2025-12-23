# Arduino SD Card Test

This is a simple Arduino sketch to test if the SD card hardware is working correctly, independent of ESPHome.

## Purpose

Test if:
1. SD card can be mounted via SPI
2. `/animations` directory exists and can be read
3. Animation files can be opened and read
4. The issue is with ESPHome or with the hardware/SD card

## Hardware Configuration

Matches the ESPHome configuration:
- **CS Pin**: GPIO 0
- **MOSI Pin**: GPIO 19
- **MISO Pin**: GPIO 22
- **SCK Pin**: GPIO 21

## How to Compile and Upload

### Option 1: Using Arduino IDE

1. Open Arduino IDE
2. Install ESP32 board support if not already installed:
   - File → Preferences
   - Add to "Additional Board Manager URLs": 
     `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Tools → Board → Boards Manager
   - Search for "esp32" and install

3. Open `test_sd_card.ino`
4. Select board: Tools → Board → ESP32 Arduino → ESP32 Dev Module
5. Select port: Tools → Port → /dev/ttyUSB0
6. Upload: Sketch → Upload
7. Open Serial Monitor: Tools → Serial Monitor (115200 baud)

### Option 2: Using arduino-cli (Command Line)

```bash
# Install arduino-cli if not installed
# curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Install ESP32 core
arduino-cli core update-index
arduino-cli core install esp32:esp32

# Compile
arduino-cli compile --fqbn esp32:esp32:esp32 test_sd_card.ino

# Upload
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 test_sd_card.ino

# Monitor serial output
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

### Option 3: Using PlatformIO

Create a `platformio.ini` file:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_port = /dev/ttyUSB0
monitor_port = /dev/ttyUSB0
```

Then:
```bash
pio run --target upload
pio device monitor
```

## Expected Output

If SD card is working correctly, you should see:

```
========================================
ESP32 SD Card Test
========================================

Initializing SD card...
  CS Pin: 0
  MOSI Pin: 19
  MISO Pin: 22
  SCK Pin: 21

SUCCESS: SD card initialized!

Card Type: SDHC
Card Size: 15193 MB
Total Space: 15193 MB
Used Space: 300 MB

========================================
Test 1: Check /animations directory
========================================
SUCCESS: /animations directory opened!
SUCCESS: /animations is a directory!

Listing first 20 files in /animations:
----------------------------------------
  anim_blackjack_idle_01.json (1234 bytes)
  anim_blackjack_idle_02.json (1456 bytes)
  ...
----------------------------------------
Total files found: 1191
JSON files: 1191

========================================
Test 2: Read specific animation file
========================================
Trying to open: /animations/anim_blackjack_idle_01.json
SUCCESS: File opened!
  File size: 1234 bytes

First 200 bytes of file:
----------------------------------------
{"Name":"anim_blackjack_idle_01",...}
----------------------------------------

========================================
Test 3: Check /audio directory
========================================
SUCCESS: /audio directory opened!
Audio files found: 1347

========================================
Test 4: List root directory
========================================
Contents of root directory:
----------------------------------------
  animations [DIR]
  audio [DIR]
  audio_mappings.json (12345 bytes)
----------------------------------------

========================================
SD Card Test Complete!
========================================
```

## Troubleshooting

### If SD card fails to initialize:
- Check wiring
- Verify SD card is inserted
- Try a different SD card
- Check if card is formatted as FAT32

### If /animations directory not found:
- SD card may not have the files
- Directory name might be different
- Card may need to be reformatted

### If files can't be opened:
- File permissions issue
- Corrupted filesystem
- Bad SD card

## What This Tells Us

- **If Arduino test WORKS**: The issue is with ESPHome's storage component
- **If Arduino test FAILS**: The issue is with hardware, wiring, or SD card

## Next Steps Based on Results

### If Arduino Works, ESPHome Doesn't:
This confirms the ESPHome storage component has a bug. Solutions:
1. Bypass directory checks in ESPHome (test for specific file)
2. Add delay after SD mount
3. Report bug to ESPHome project
4. Use Arduino SD library directly in ESPHome

### If Arduino Also Fails:
Hardware/SD card issue. Check:
1. Wiring connections
2. SD card format (must be FAT32)
3. SD card quality
4. Voltage levels (3.3V)
