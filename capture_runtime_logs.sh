#!/bin/bash

# Capture runtime logs from ESP32 to check SD card file reading
# This script will capture logs for 60 seconds and save to a file

YAML_FILE="config/vector-eyes-ttgo.yaml"
LOG_FILE="runtime_logs_$(date +%Y%m%d_%H%M%S).txt"

echo "========================================" | tee "$LOG_FILE"
echo "ESP32 Runtime Log Capture" | tee -a "$LOG_FILE"
echo "Date: $(date)" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

echo "Capturing logs from $YAML_FILE..." | tee -a "$LOG_FILE"
echo "Logs will be saved to: $LOG_FILE" | tee -a "$LOG_FILE"
echo "Press Ctrl+C to stop capture early" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Run esphome logs and capture output
timeout 60s esphome logs "$YAML_FILE" --device /dev/ttyUSB0 2>&1 | tee -a "$LOG_FILE"

echo "" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "Log capture complete!" | tee -a "$LOG_FILE"
echo "Log file: $LOG_FILE" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"

# Analyze the logs for SD card related messages
echo "" | tee -a "$LOG_FILE"
echo "Analyzing SD card activity..." | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

grep -i "sd\|storage\|file\|mount" "$LOG_FILE" | head -50
