#!/bin/bash
# Compile and upload Arduino SD card test

echo "=========================================="
echo "Arduino SD Card Test - Compile & Upload"
echo "=========================================="
echo ""

# Check if arduino-cli is installed
if ! command -v arduino-cli &> /dev/null; then
    echo "ERROR: arduino-cli not found!"
    echo ""
    echo "Install with:"
    echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    echo ""
    echo "Or use Arduino IDE instead (see ARDUINO_SD_TEST.md)"
    exit 1
fi

echo "Step 1: Update core index..."
arduino-cli core update-index

echo ""
echo "Step 2: Install ESP32 core (if not already installed)..."
arduino-cli core install esp32:esp32 || echo "ESP32 core already installed"

echo ""
echo "Step 3: Compile sketch..."
arduino-cli compile --fqbn esp32:esp32:esp32 test_sd_card.ino

if [ $? -ne 0 ]; then
    echo ""
    echo "ERROR: Compilation failed!"
    exit 1
fi

echo ""
echo "Step 4: Upload to device..."
echo "Port: /dev/ttyUSB0"
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 test_sd_card.ino

if [ $? -ne 0 ]; then
    echo ""
    echo "ERROR: Upload failed!"
    echo "Make sure device is connected to /dev/ttyUSB0"
    exit 1
fi

echo ""
echo "=========================================="
echo "Upload complete!"
echo "=========================================="
echo ""
echo "Opening serial monitor in 3 seconds..."
echo "Press Ctrl+C to exit monitor"
echo ""
sleep 3

arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
