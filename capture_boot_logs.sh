#!/bin/bash

# Capture boot logs from ESP32 including full boot sequence
# User should reset device after starting this script

YAML_FILE="config/vector-eyes-ttgo.yaml"
LOG_FILE="boot_logs_$(date +%Y%m%d_%H%M%S).txt"

echo "========================================" | tee "$LOG_FILE"
echo "ESP32 Boot Log Capture" | tee -a "$LOG_FILE"
echo "Date: $(date)" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

echo "🔌 Waiting for device connection..." | tee -a "$LOG_FILE"
echo "📋 Logs will be saved to: $LOG_FILE" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"
echo "⚡ RESET THE DEVICE NOW to capture boot sequence!" | tee -a "$LOG_FILE"
echo "⏱️  Capturing for 60 seconds..." | tee -a "$LOG_FILE"
echo "🛑 Press Ctrl+C to stop early" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Run esphome logs with explicit device - capture for 60 seconds
timeout 60s esphome logs "$YAML_FILE" --device /dev/ttyUSB0 2>&1 | tee -a "$LOG_FILE"

echo "" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"
echo "✅ Log capture complete!" | tee -a "$LOG_FILE"
echo "📄 Log file: $LOG_FILE" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"

# Run analysis
echo "" | tee -a "$LOG_FILE"
echo "🔍 Running automated analysis..." | tee -a "$LOG_FILE"
python analyze_esp32_logs.py "$LOG_FILE" > "${LOG_FILE%.txt}_analysis.txt" 2>&1

echo "" | tee -a "$LOG_FILE"
echo "📊 Analysis saved to: ${LOG_FILE%.txt}_analysis.txt" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"
