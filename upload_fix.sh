#!/bin/bash
# Upload the fixed firmware

source venv/bin/activate
esphome upload config/vector-eyes-ttgo.yaml --device /dev/ttyUSB0
