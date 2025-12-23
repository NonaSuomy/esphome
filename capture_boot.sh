#!/bin/bash
# Quick script to capture boot logs after SD card insertion

cd "$(dirname "$0")"

echo "=========================================="
echo "ESP32 Boot Log Capture"
echo "=========================================="
echo ""
echo "This will capture 40 seconds of logs."
echo ""
echo "INSTRUCTIONS:"
echo "1. This script will start in 5 seconds"
echo "2. When you see 'PRESS RESET NOW', press the RESET button on your ESP32"
echo "3. Wait for capture to complete"
echo ""
echo "Starting in 5 seconds..."
sleep 5

echo ""
echo "=========================================="
echo "⚡ PRESS RESET NOW! ⚡"
echo "=========================================="
echo ""

# Run the capture
python3 capture_serial_logs.py config/vector-eyes-ttgo.yaml 40

echo ""
echo "=========================================="
echo "Capture Complete!"
echo "=========================================="
echo ""
echo "Analyzing logs for storage initialization..."
echo ""

# Show storage-related messages
if [ -f serial_boot_log.txt ]; then
    echo "=== Storage Initialization Messages ==="
    grep -E "storage|SD|mount|animation|Found|vector_eyes" serial_boot_log.txt | head -50
    echo ""
    echo "=== Full log saved to: serial_boot_log.txt ==="
    echo ""
    
    # Check for success or failure
    if grep -q "Found.*animation.*files" serial_boot_log.txt; then
        echo "✅ SUCCESS: Animations found on SD card!"
    elif grep -q "No storage device with animations found" serial_boot_log.txt; then
        echo "❌ FAILURE: Animations not found - check SD card structure"
    else
        echo "⚠️  UNKNOWN: Could not determine status - check full log"
    fi
else
    echo "❌ ERROR: Log file not created"
fi
