#!/bin/bash
# Check for CSV files on SD card

SD_PATH="/run/media/nonasuomy/TTGOCAM"

echo "Checking for CSV files on SD card..."
echo ""

# Count CSV files in root
CSV_ROOT=$(find "$SD_PATH" -maxdepth 1 -name "*.csv" -type f 2>/dev/null | wc -l)
echo "CSV files in root: $CSV_ROOT"

if [ $CSV_ROOT -gt 0 ]; then
    echo ""
    echo "CSV files found:"
    find "$SD_PATH" -maxdepth 1 -name "*.csv" -type f 2>/dev/null | head -10
    if [ $CSV_ROOT -gt 10 ]; then
        echo "... and $((CSV_ROOT - 10)) more"
    fi
fi

echo ""
echo "Note: CSV animation format is NOT supported with storage component yet."
echo "Only JSON animations work with the new storage system."
