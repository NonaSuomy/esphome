#!/bin/bash
# Check SD card contents

SD="/run/media/nonasuomy/ESCAPE"

echo "=== SD Card Contents ==="
echo "Animations: $(find "$SD/animations" -name "*.json" 2>/dev/null | wc -l) files"
echo "Audio: $(find "$SD/audio" -name "*.wav" 2>/dev/null | wc -l) files"
echo ""
echo "Critical files:"
test -f "$SD/animations/anim_blackjack_idle_01.json" && echo "✓ anim_blackjack_idle_01.json" || echo "✗ anim_blackjack_idle_01.json MISSING"
test -f "$SD/audio_mappings.json" && echo "✓ audio_mappings.json" || echo "✗ audio_mappings.json MISSING"
echo ""
echo "Sample audio files:"
ls "$SD/audio" 2>/dev/null | head -5
