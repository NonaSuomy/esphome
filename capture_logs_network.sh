#!/bin/bash

# Capture runtime logs from ESP32 via network (OTA)
# This script will capture logs for 60 seconds and save to a file

YAML_FILE="config/vector-eyes-ttgo.yaml"
LOG_FILE="runtime_logs_$(date +%Y%m%d_%H%M%S).txt"

echo "========================================" | tee "$LOG_FILE"
echo "ESP32 Runtime Log Capture (Network)" | tee -a "$LOG_FILE"
echo "Date: $(date)" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

echo "Capturing logs from $YAML_FILE via network..." | tee -a "$LOG_FILE"
echo "Logs will be saved to: $LOG_FILE" | tee -a "$LOG_FILE"
echo "Press Ctrl+C to stop capture" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Run esphome logs without device specification (will use network)
timeout 60s esphome logs "$YAML_FILE" 2>&1 | tee -a "$LOG_FILE"

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
echo "--- Storage/SD Card Messages ---" | tee -a "$LOG_FILE"
grep -i "storage\|sd_card\|mount" "$LOG_FILE" | grep -v "^Analyzing\|^---" || echo "No storage messages found"

echo "" | tee -a "$LOG_FILE"
echo "--- File Operations ---" | tee -a "$LOG_FILE"
grep -i "file\|read\|open\|close" "$LOG_FILE" | grep -v "^Analyzing\|^---\|^Log file" | head -20 || echo "No file operation messages found"

echo "" | tee -a "$LOG_FILE"
echo "--- Animation Loading ---" | tee -a "$LOG_FILE"
grep -i "animation\|anim_\|\.json" "$LOG_FILE" | grep -v "^Analyzing\|^---" | head -20 || echo "No animation messages found"

echo "" | tee -a "$LOG_FILE"
echo "--- Audio/WAV Files ---" | tee -a "$LOG_FILE"
grep -i "audio\|wav\|sound" "$LOG_FILE" | grep -v "^Analyzing\|^---" | head -20 || echo "No audio messages found"

echo "" | tee -a "$LOG_FILE"
echo "--- Errors/Warnings ---" | tee -a "$LOG_FILE"
grep -i "error\|warn\|fail" "$LOG_FILE" | grep -v "^Analyzing\|^---" | head -20 || echo "No errors/warnings found"

echo "" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "Analysis complete!" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
