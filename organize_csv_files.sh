#!/bin/bash
# Move CSV files to their own folder

SD_PATH="/run/media/nonasuomy/TTGOCAM"

echo "Organizing CSV files on SD card..."
echo ""

# Create CSV animations folder
mkdir -p "$SD_PATH/animations_csv"

# Count CSV files in root
CSV_COUNT=$(find "$SD_PATH" -maxdepth 1 -name "*.csv" -type f 2>/dev/null | wc -l)

if [ $CSV_COUNT -gt 0 ]; then
    echo "Moving $CSV_COUNT CSV files to /animations_csv..."
    find "$SD_PATH" -maxdepth 1 -name "*.csv" -type f -exec mv {} "$SD_PATH/animations_csv/" \; 2>/dev/null
    echo "✅ Done!"
else
    echo "No CSV files found in root"
fi

echo ""
echo "Final SD card structure:"
echo "========================"
ls -lh "$SD_PATH/" | grep -E "^d|audio_mappings"

echo ""
echo "CSV animations folder:"
CSV_FINAL=$(ls "$SD_PATH/animations_csv/"*.csv 2>/dev/null | wc -l)
echo "  $CSV_FINAL CSV files"
ls "$SD_PATH/animations_csv/"*.csv 2>/dev/null | head -5
if [ $CSV_FINAL -gt 5 ]; then
    echo "  ... and $((CSV_FINAL - 5)) more"
fi

echo ""
echo "Note: CSV animations are not currently supported with the storage component."
echo "They are kept in /animations_csv for future use or legacy SD mode."
