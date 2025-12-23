#!/bin/bash
# Helper script to compile ESPHome and check results
# Usage: ./compile_and_check.sh <config_file> [upload]

set -e

CONFIG_FILE="$1"
DO_UPLOAD="$2"
VENV_PATH="./venv/bin/activate"
LOG_FILE="/tmp/esphome_compile_$(date +%s).log"

if [ -z "$CONFIG_FILE" ]; then
    echo "ERROR: Config file required"
    echo "Usage: $0 <config_file> [upload]"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: Config file not found: $CONFIG_FILE"
    exit 1
fi

echo "=== ESPHome Compile Helper ==="
echo "Config: $CONFIG_FILE"
echo "Log: $LOG_FILE"
echo ""

# Activate venv and compile
source "$VENV_PATH"

echo "Starting compilation..."
if esphome compile "$CONFIG_FILE" 2>&1 | tee "$LOG_FILE"; then
    echo ""
    echo "✅ COMPILATION SUCCESSFUL"
    
    # Check if firmware exists
    BUILD_NAME=$(basename "$CONFIG_FILE" .yaml)
    FIRMWARE_PATH="config/.esphome/build/$BUILD_NAME/.pioenvs/$BUILD_NAME/firmware.elf"
    
    if [ -f "$FIRMWARE_PATH" ]; then
        echo "✅ Firmware found: $FIRMWARE_PATH"
        FIRMWARE_SIZE=$(stat -f%z "$FIRMWARE_PATH" 2>/dev/null || stat -c%s "$FIRMWARE_PATH" 2>/dev/null)
        echo "   Size: $FIRMWARE_SIZE bytes"
    else
        echo "⚠️  Warning: Firmware not found at expected path"
    fi
    
    # Upload if requested
    if [ "$DO_UPLOAD" = "upload" ]; then
        echo ""
        echo "Starting upload..."
        if esphome upload "$CONFIG_FILE" --device /dev/ttyUSB0 2>&1 | tee -a "$LOG_FILE"; then
            echo "✅ UPLOAD SUCCESSFUL"
        else
            echo "❌ UPLOAD FAILED"
            exit 1
        fi
    fi
    
    exit 0
else
    echo ""
    echo "❌ COMPILATION FAILED"
    echo ""
    echo "=== Error Summary ==="
    
    # Extract key errors from log
    if grep -q "fatal error:" "$LOG_FILE"; then
        echo "Fatal errors found:"
        grep "fatal error:" "$LOG_FILE" | head -5
    fi
    
    if grep -q "undefined reference" "$LOG_FILE"; then
        echo ""
        echo "Linker errors found:"
        grep "undefined reference" "$LOG_FILE" | head -10
    fi
    
    if grep -q "error:" "$LOG_FILE"; then
        echo ""
        echo "Compilation errors:"
        grep "error:" "$LOG_FILE" | grep -v "fatal error:" | head -10
    fi
    
    echo ""
    echo "Full log: $LOG_FILE"
    exit 1
fi
