#!/bin/bash
# Copy files to SD card with correct paths

SD="/run/media/nonasuomy/ESCAPE"

echo "=== Copying to SD Card ==="
echo "Target: $SD"
echo ""

# Create directories
mkdir -p "$SD/animations" "$SD/audio"

# Copy animations
echo "Copying animations..."
cp esphome/components/vector_eyes/animations_json/*.json "$SD/animations/"
ANIM_COUNT=$(ls "$SD/animations"/*.json 2>/dev/null | wc -l)
echo "✓ Animations: $ANIM_COUNT files"

# Copy audio - use absolute path from workspace root
echo "Copying audio files..."
cp /home/nonasuomy/code/vectorsounds/*.wav "$SD/audio/"
AUDIO_COUNT=$(ls "$SD/audio"/*.wav 2>/dev/null | wc -l)
echo "✓ Audio: $AUDIO_COUNT files"

# Copy mappings
echo "Copying audio mappings..."
cp esphome/components/vector_eyes/audio_mappings.json "$SD/"
echo "✓ Mappings copied"

echo ""
echo "=== Verification ==="
echo "Animations: $(ls "$SD/animations"/*.json 2>/dev/null | wc -l)"
echo "Audio: $(ls "$SD/audio"/*.wav 2>/dev/null | wc -l)"
test -f "$SD/audio_mappings.json" && echo "Mappings: present" || echo "Mappings: MISSING"
test -f "$SD/animations/anim_blackjack_idle_01.json" && echo "Test file: present" || echo "Test file: MISSING"
echo ""
echo "Done!"
