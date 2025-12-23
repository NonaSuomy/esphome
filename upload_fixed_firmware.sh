#!/bin/bash
# Upload the fixed firmware to ESP32

set -e

echo "=== Uploading Fixed Firmware to ESP32 ==="
echo ""

# Activate venv
source venv/bin/activate

# Upload firmware
echo "Uploading to /dev/ttyUSB0..."
esphome upload config/vector-eyes-ttgo.yaml --device /dev/ttyUSB0

echo ""
echo "=== Upload Complete ==="
echo ""
echo "The ESP32 should now reboot with the fixed firmware."
echo "Watch for these success messages in the logs:"
echo "  [I][sd_storage.spi:183]: SD card mounted successfully at /sd"
echo "  [I][vector_eyes.storage_adapter:386]: Testing for specific file: /sd/animations/anim_blackjack_idle_01.json"
echo "  [I][vector_eyes.storage_adapter:389]: Found device with animations at /sd (verified via file check)"
echo "  [I][vector_eyes.storage_adapter:048]: Storage adapter initialized successfully"
