#!/bin/bash
# Automated ESP32 boot log capture

PORT="/dev/ttyUSB0"
BAUD="115200"
OUTPUT="boot_capture_final.txt"
DURATION=60

echo "=========================================="
echo "ESP32 Boot Log Capture (Automated)"
echo "=========================================="
echo "Port: $PORT"
echo "Baud: $BAUD"
echo "Duration: ${DURATION}s"
echo ""
echo "Starting capture in 3 seconds..."
echo "RESET YOUR DEVICE NOW!"
echo ""
sleep 1
echo "2..."
sleep 1
echo "1..."
sleep 1
echo ""
echo "=========================================="
echo "CAPTURING..."
echo "=========================================="
echo ""

# Configure serial port
stty -F $PORT $BAUD raw -echo

# Use cat with timeout to capture raw serial data
timeout ${DURATION}s cat $PORT > $OUTPUT 2>&1

echo ""
echo "=========================================="
echo "Capture complete!"
echo "=========================================="
echo "Output saved to: $OUTPUT"
echo ""

# Show summary
if [ -f "$OUTPUT" ]; then
    LINES=$(wc -l < "$OUTPUT")
    SIZE=$(du -h "$OUTPUT" | cut -f1)
    echo "Captured: $LINES lines ($SIZE)"
    echo ""
    echo "Checking for key messages..."
    echo ""
    
    if grep -q "rst:" "$OUTPUT"; then
        echo "✓ Boot sequence detected"
    else
        echo "✗ No boot sequence found"
    fi
    
    if grep -q "SD card" "$OUTPUT"; then
        echo "✓ SD card messages found"
    else
        echo "✗ No SD card messages"
    fi
    
    if grep -q "animations" "$OUTPUT"; then
        echo "✓ Animation messages found"
    else
        echo "✗ No animation messages"
    fi
    
    echo ""
    echo "Extracting key log lines..."
    echo ""
    grep -E "(rst:|SD card|animations|storage_adapter|Found device|Cannot list)" "$OUTPUT" | head -30
else
    echo "ERROR: Output file not created"
fi
