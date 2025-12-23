#!/bin/bash
# Setup SD card directory structure for vector_eyes

SD_PATH="/run/media/nonasuomy/TTGOCAM"

echo "Setting up SD card at: $SD_PATH"
echo ""

# Create directories
echo "Creating directories..."
mkdir -p "$SD_PATH/animations"
mkdir -p "$SD_PATH/audio"

# Count files in root
JSON_COUNT=$(find "$SD_PATH" -maxdepth 1 -name "*.json" -type f 2>/dev/null | wc -l)
WAV_COUNT=$(find "$SD_PATH" -maxdepth 1 -name "*.wav" -type f 2>/dev/null | wc -l)

echo "Found $JSON_COUNT JSON files in root"
echo "Found $WAV_COUNT WAV files in root"
echo ""

# Move animation JSON files to /animations
if [ $JSON_COUNT -gt 0 ]; then
    echo "Moving JSON files to /animations..."
    find "$SD_PATH" -maxdepth 1 -name "anim_*.json" -type f -exec mv {} "$SD_PATH/animations/" \; 2>/dev/null
    
    # Keep audio_mappings.json in root
    if [ -f "$SD_PATH/animations/audio_mappings.json" ]; then
        mv "$SD_PATH/animations/audio_mappings.json" "$SD_PATH/"
        echo "  Kept audio_mappings.json in root"
    fi
fi

# Move WAV files to /audio
if [ $WAV_COUNT -gt 0 ]; then
    echo "Moving WAV files to /audio..."
    find "$SD_PATH" -maxdepth 1 -name "*.wav" -type f -exec mv {} "$SD_PATH/audio/" \; 2>/dev/null
fi

echo ""
echo "Final structure:"
echo "================"
ls -lh "$SD_PATH/" | grep -E "^d|audio_mappings"
echo ""
echo "Animations directory:"
ANIM_COUNT=$(ls "$SD_PATH/animations/"*.json 2>/dev/null | wc -l)
echo "  $ANIM_COUNT animation files"
ls "$SD_PATH/animations/"*.json 2>/dev/null | head -5
if [ $ANIM_COUNT -gt 5 ]; then
    echo "  ... and $((ANIM_COUNT - 5)) more"
fi

echo ""
echo "Audio directory:"
AUDIO_COUNT=$(ls "$SD_PATH/audio/"*.wav 2>/dev/null | wc -l)
echo "  $AUDIO_COUNT audio files"
ls "$SD_PATH/audio/"*.wav 2>/dev/null | head -5
if [ $AUDIO_COUNT -gt 5 ]; then
    echo "  ... and $((AUDIO_COUNT - 5)) more"
fi

echo ""
echo "✅ SD card setup complete!"
