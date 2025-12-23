#!/bin/bash

# Capture runtime logs from ESP32 via USB
# This script will capture logs for 60 seconds and save to a file

YAML_FILE="config/vector-eyes-ttgo.yaml"
LOG_FILE="runtime_logs_$(date +%Y%m%d_%H%M%S).txt"

echo "========================================" | tee "$LOG_FILE"
echo "ESP32 Runtime Log Capture (USB)" | tee -a "$LOG_FILE"
echo "Date: $(date)" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

echo "Capturing logs from $YAML_FILE via /dev/ttyUSB0..." | tee -a "$LOG_FILE"
echo "Logs will be saved to: $LOG_FILE" | tee -a "$LOG_FILE"
echo "Capturing for 60 seconds... Press Ctrl+C to stop early" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Run esphome logs with explicit device
timeout 60s esphome logs "$YAML_FILE" --device /dev/ttyUSB0 2>&1 | tee -a "$LOG_FILE"

echo "" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "Log capture complete!" | tee -a "$LOG_FILE"
echo "Log file: $LOG_FILE" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"

# Analyze the logs for SD card related messages
echo "" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "SD Card & Storage Activity Analysis:" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"

echo "" | tee -a "$LOG_FILE"
echo "--- Storage/SD Card Initialization ---" | tee -a "$LOG_FILE"
grep -i "storage\|sd_card\|sd_spi\|mount" "$LOG_FILE" | grep -v "^Analyzing\|^---" || echo "No storage messages found"

echo "" | tee -a "$LOG_FILE"
echo "--- File Operations ---" | tee -a "$LOG_FILE"
grep -i "file_exists\|read_file\|open_file\|close_file" "$LOG_FILE" | grep -v "^Analyzing\|^---\|^Log file" | head -30 || echo "No file operation messages found"

echo "" | tee -a "$LOG_FILE"
echo "--- Animation Loading ---" | tee -a "$LOG_FILE"
grep -i "animation\|anim_\|\.json\|loading" "$LOG_FILE" | grep -v "^Analyzing\|^---" | head -30 || echo "No animation messages found"

echo "" | tee -a "$LOG_FILE"
echo "--- Audio/WAV Files ---" | tee -a "$LOG_FILE"
grep -i "audio\|\.wav\|sound\|speaker" "$LOG_FILE" | grep -v "^Analyzing\|^---" | head -30 || echo "No audio messages found"

echo "" | tee -a "$LOG_FILE"
echo "--- Vector Eyes Component ---" | tee -a "$LOG_FILE"
grep -i "vector_eyes\|VectorEyes" "$LOG_FILE" | grep -v "^Analyzing\|^---" | head -30 || echo "No vector_eyes messages found"

echo "" | tee -a "$LOG_FILE"
echo "--- Errors/Warnings ---" | tee -a "$LOG_FILE"
grep -E "\[E\]|\[W\]|error|Error|ERROR|warn|Warn|WARN|fail|Fail|FAIL" "$LOG_FILE" | grep -v "^Analyzing\|^---\|strapping PIN" | head -30 || echo "No errors/warnings found"

echo "" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "Analysis complete! Check $LOG_FILE for full details" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
