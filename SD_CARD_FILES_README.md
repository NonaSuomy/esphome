# SD Card Files Documentation

## Overview

This directory contains a complete listing of all files on the SD card for the vector_eyes ESP32 project.

## Files Generated

### SD_CARD_COMPLETE_FILE_LIST.md
**Complete listing of all 2609 files on the SD card**

- **Size**: 71.4 KB (2,651 lines)
- **Contents**:
  - 2 root files (README.txt, audio_mappings.json)
  - 1191 JSON animation files
  - 69 CSV animation files (legacy, not currently supported)
  - 1347 WAV audio files

### File Breakdown

| Directory | Files | Status | Purpose |
|-----------|-------|--------|---------|
| Root | 2 | ✅ Active | Configuration files |
| `/animations/` | 1191 | ✅ Active | JSON animations for vector_eyes |
| `/animations_csv/` | 69 | ⚠️ Inactive | Legacy CSV format (not supported) |
| `/audio/` | 1347 | ✅ Active | WAV audio files for animations |
| **Total** | **2609** | - | All files |

## How to Use

### View Complete File List
```bash
cat SD_CARD_COMPLETE_FILE_LIST.md
```

### Regenerate File List
If you modify the SD card contents, regenerate the list:
```bash
python3 generate_sd_file_list.py
```

### Search for Specific Files
```bash
# Find animation by name
grep "anim_happy" SD_CARD_COMPLETE_FILE_LIST.md

# Count files of a type
grep -c "\.json" SD_CARD_COMPLETE_FILE_LIST.md
```

## File Naming Conventions

### JSON Animations
Format: `anim_<category>_<name>_<variant>.json`

Examples:
- `anim_keepalive_blink_01.json`
- `anim_reacttoface_happy_01.json`
- `anim_avs_listen_timeout_03.json`

### CSV Animations (Legacy)
Format: `<category>_<name>_<variant>.csv`

Examples:
- `dancebeat_getin_01.csv`
- `chargerdocking_comeoff_straight_03.csv`

### Audio Files
Format: `<numeric_id>.wem.wav` or `<descriptive_name>.wav`

Examples:
- `1000932328.wem.wav`
- `Robot_Vic_Sfx__Scrn_Blink_01_WM.wav`

## Storage Statistics

- **Total Files**: 2,609
- **Total Size**: ~300 KB (directories)
- **Animations (JSON)**: 1,191 files (~160 KB)
- **Animations (CSV)**: 69 files (~8 KB)
- **Audio**: 1,347 files (~132 KB)
- **Config**: 2 files (~5 KB)

## Notes

- CSV animations are kept for archival purposes but are not currently supported by the storage component
- All JSON animations are automatically discovered by the storage adapter
- Audio files are mapped to animation events via `audio_mappings.json`
- The file list is sorted alphabetically within each directory

## Related Documentation

- `SD_CARD_READY.md` - SD card setup completion status
- `SD_CARD_FINAL_STATUS.md` - Final organization summary
- `NEXT_STEPS.md` - Testing instructions
- `RUNTIME_ISSUES.md` - Known issues and solutions
