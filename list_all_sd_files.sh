#!/bin/bash
# Generate a complete listing of all SD card files

SD_PATH="/run/media/nonasuomy/TTGOCAM"
OUTPUT="SD_CARD_COMPLETE_FILE_LIST.md"

echo "Generating complete SD card file listing..."

cat > "$OUTPUT" << 'EOF'
# SD Card Complete File Listing

Generated: $(date)

## Summary

| Directory | File Count | File Types |
|-----------|------------|------------|
| `/animations/` | 1191 | JSON animation files |
| `/animations_csv/` | 69 | CSV animation files (legacy) |
| `/audio/` | 1347 | WAV audio files |
| Root | 1 | audio_mappings.json |
| **Total** | **2608** | All files |

---

## Root Directory

EOF

# List root files
echo "" >> "$OUTPUT"
echo '```' >> "$OUTPUT"
ls -lh "$SD_PATH"/*.* 2>/dev/null | awk '{print $9}' | xargs -I {} basename {} >> "$OUTPUT" 2>/dev/null
echo '```' >> "$OUTPUT"

# Animations (JSON)
echo "" >> "$OUTPUT"
echo "---" >> "$OUTPUT"
echo "" >> "$OUTPUT"
echo "## /animations/ Directory (1191 JSON files)" >> "$OUTPUT"
echo "" >> "$OUTPUT"
echo '```' >> "$OUTPUT"
ls "$SD_PATH/animations/"*.json 2>/dev/null | xargs -I {} basename {} >> "$OUTPUT"
echo '```' >> "$OUTPUT"

# Animations CSV
echo "" >> "$OUTPUT"
echo "---" >> "$OUTPUT"
echo "" >> "$OUTPUT"
echo "## /animations_csv/ Directory (69 CSV files)" >> "$OUTPUT"
echo "" >> "$OUTPUT"
echo '```' >> "$OUTPUT"
ls "$SD_PATH/animations_csv/"*.csv 2>/dev/null | xargs -I {} basename {} >> "$OUTPUT"
echo '```' >> "$OUTPUT"

# Audio
echo "" >> "$OUTPUT"
echo "---" >> "$OUTPUT"
echo "" >> "$OUTPUT"
echo "## /audio/ Directory (1347 WAV files)" >> "$OUTPUT"
echo "" >> "$OUTPUT"
echo '```' >> "$OUTPUT"
ls "$SD_PATH/audio/"*.wav 2>/dev/null | xargs -I {} basename {} >> "$OUTPUT"
echo '```' >> "$OUTPUT"

echo ""
echo "✅ Complete file listing generated: $OUTPUT"
echo ""
echo "File size: $(wc -l < "$OUTPUT") lines"
