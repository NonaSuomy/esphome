#!/bin/bash
# Setup fresh SD card with Vector animations and audio files

set -e

SD_MOUNT="/run/media/nonasuomy/ESCAPE"
ANIMATIONS_SRC="esphome/components/vector_eyes/animations_json"
AUDIO_SRC="../../../vectorsounds"
AUDIO_MAPPINGS_SRC="esphome/components/vector_eyes/audio_mappings.json"

echo "=== Vector Eyes SD Card Setup ==="
echo "Target: $SD_MOUNT"
echo ""

# Check if SD card is mounted
if [ ! -d "$SD_MOUNT" ]; then
    echo "ERROR: SD card not found at $SD_MOUNT"
    echo "Please ensure the SD card is mounted"
    exit 1
fi

# Check if we have write permission
if [ ! -w "$SD_MOUNT" ]; then
    echo "ERROR: No write permission to $SD_MOUNT"
    echo "You may need to run with sudo or adjust permissions"
    exit 1
fi

echo "✓ SD card found and writable"
echo ""

# Create directories
echo "Creating directory structure..."
mkdir -p "$SD_MOUNT/animations"
mkdir -p "$SD_MOUNT/audio"
echo "✓ Directories created"
echo ""

# Copy animations
echo "Copying animation files..."
if [ -d "$ANIMATIONS_SRC" ]; then
    ANIM_COUNT=$(find "$ANIMATIONS_SRC" -name "*.json" | wc -l)
    echo "  Found $ANIM_COUNT animation JSON files"
    cp -v "$ANIMATIONS_SRC"/*.json "$SD_MOUNT/animations/" 2>&1 | tail -10
    echo "✓ Animations copied"
else
    echo "WARNING: Animation source directory not found: $ANIMATIONS_SRC"
fi
echo ""

# Copy audio files
echo "Copying audio files..."
if [ -d "$AUDIO_SRC" ]; then
    AUDIO_COUNT=$(find "$AUDIO_SRC" -name "*.wav" | wc -l)
    echo "  Found $AUDIO_COUNT audio WAV files"
    echo "  This may take a while..."
    cp -v "$AUDIO_SRC"/*.wav "$SD_MOUNT/audio/" 2>&1 | tail -10
    echo "✓ Audio files copied"
else
    echo "WARNING: Audio source directory not found: $AUDIO_SRC"
fi
echo ""

# Copy audio mappings
echo "Copying audio mappings..."
if [ -f "$AUDIO_MAPPINGS_SRC" ]; then
    cp -v "$AUDIO_MAPPINGS_SRC" "$SD_MOUNT/"
    echo "✓ Audio mappings copied"
else
    echo "WARNING: Audio mappings file not found: $AUDIO_MAPPINGS_SRC"
fi
echo ""

# Verify contents
echo "=== Verification ==="
echo "Animations: $(find "$SD_MOUNT/animations" -name "*.json" 2>/dev/null | wc -l) files"
echo "Audio: $(find "$SD_MOUNT/audio" -name "*.wav" 2>/dev/null | wc -l) files"
echo "Audio mappings: $(test -f "$SD_MOUNT/audio_mappings.json" && echo "present" || echo "missing")"
echo ""

# Check for the test file we use in the code
TEST_FILE="$SD_MOUNT/animations/anim_blackjack_idle_01.json"
if [ -f "$TEST_FILE" ]; then
    echo "✓ Test file found: anim_blackjack_idle_01.json"
else
    echo "⚠ WARNING: Test file not found: anim_blackjack_idle_01.json"
    echo "  The ESP32 code checks for this file to detect animations"
fi
echo ""

echo "=== SD Card Setup Complete ==="
echo "You can now safely eject the SD card and insert it into the ESP32"
