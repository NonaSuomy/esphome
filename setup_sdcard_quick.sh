#!/bin/bash
# Quick SD card setup with progress

SD_MOUNT="/run/media/nonasuomy/ESCAPE"
ANIMATIONS_SRC="esphome/components/vector_eyes/animations_json"
AUDIO_SRC="../../../vectorsounds"
AUDIO_MAPPINGS_SRC="esphome/components/vector_eyes/audio_mappings.json"

echo "=== Vector Eyes SD Card Setup ==="

# Create directories
mkdir -p "$SD_MOUNT/animations" "$SD_MOUNT/audio"

# Copy animations (fast)
echo "Copying animations..."
cp "$ANIMATIONS_SRC"/*.json "$SD_MOUNT/animations/" 2>/dev/null
echo "✓ Animations: $(ls "$SD_MOUNT/animations"/*.json 2>/dev/null | wc -l) files"

# Copy audio (slow - show progress)
echo "Copying audio files (this will take a few minutes)..."
rsync -ah --info=progress2 "$AUDIO_SRC"/*.wav "$SD_MOUNT/audio/" 2>/dev/null || \
    cp "$AUDIO_SRC"/*.wav "$SD_MOUNT/audio/" 2>/dev/null
echo "✓ Audio: $(ls "$SD_MOUNT/audio"/*.wav 2>/dev/null | wc -l) files"

# Copy mappings
cp "$AUDIO_MAPPINGS_SRC" "$SD_MOUNT/" 2>/dev/null
echo "✓ Audio mappings copied"

echo ""
echo "=== Complete ==="
echo "SD card is ready for ESP32"
