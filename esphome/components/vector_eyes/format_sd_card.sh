#!/bin/bash

# Script to format SD card for ESP32 compatibility
# WARNING: This will erase all data on the SD card!

echo "=== ESP32 SD Card Formatter ==="
echo ""
echo "This script will format your SD card to FAT32 with ESP32-compatible settings"
echo "WARNING: ALL DATA ON THE SD CARD WILL BE ERASED!"
echo ""

# Find the SD card device
echo "Available devices:"
lsblk -o NAME,SIZE,TYPE,MOUNTPOINT | grep -E "disk|part"
echo ""
read -p "Enter the SD card device (e.g., /dev/sdb or /dev/mmcblk0): " DEVICE

if [ -z "$DEVICE" ]; then
    echo "Error: No device specified"
    exit 1
fi

# Confirm
read -p "Are you SURE you want to format $DEVICE? This will erase ALL data! (yes/no): " CONFIRM
if [ "$CONFIRM" != "yes" ]; then
    echo "Aborted"
    exit 0
fi

echo ""
echo "Unmounting any mounted partitions..."
sudo umount ${DEVICE}* 2>/dev/null

echo "Creating new MBR partition table..."
sudo parted -s $DEVICE mklabel msdos

echo "Creating FAT32 partition..."
sudo parted -s $DEVICE mkpart primary fat32 1MiB 100%

# Determine partition name
if [[ $DEVICE == *"mmcblk"* ]]; then
    PARTITION="${DEVICE}p1"
else
    PARTITION="${DEVICE}1"
fi

echo "Formatting partition as FAT32 with 4KB clusters..."
sudo mkfs.vfat -F 32 -s 8 -n "TTGOCAM" $PARTITION

echo ""
echo "SD card formatted successfully!"
echo "Please remove and reinsert the SD card, then run the prepare script."
