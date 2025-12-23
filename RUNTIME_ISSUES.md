# Runtime Issues Found

## Issue 1: Missing Animations Directory on SD Card

**Status**: ✅ FIXED

**Description**:
The storage adapter successfully mounts the SD card at `/sd`, but could not find the required `/animations` directory because animation files were in the root directory instead.

**Evidence from logs**:
```
[18:43:10.361][I][sd_storage.spi:183]: SD card mounted successfully at /sd
[18:43:59.884][D][vector_eyes.storage_adapter:391]: Preferred mount path not found or has no animations, trying all devices
[18:44:00.477][D][vector_eyes.storage_adapter:409]: No storage device with animations directory found
[18:44:00.478][W][vector_eyes.storage_adapter:054]: No storage device with animations found, will fall back to internal animations
```

**Root Cause**:
Animation JSON files were placed in the SD card root directory instead of in an `/animations` subdirectory.

**Solution Applied**:
✅ Created `/animations` directory on SD card
✅ Moved 1191 animation JSON files to `/animations/`
✅ Created `/audio` directory on SD card  
✅ Moved 1347 audio WAV files to `/audio/`
✅ Kept `audio_mappings.json` in root directory

**Current SD Card Structure**:
```
/
├── animations/          (1191 files)
│   ├── anim_attention_lookatdevice_01.json
│   ├── anim_avs_back2listen_03.json
│   └── ...
├── audio/               (1347 files)
│   ├── 1000932328.wem.wav
│   └── ...
└── audio_mappings.json
```

**Next Step**: Unmount SD card, insert into ESP32, and test

---

## Issue 2: SPI Device Handle Errors (Display)

**Status**: ⚠️  NON-BLOCKING (but spamming logs)

**Description**:
Continuous SPI errors indicating the display SPI device handle is invalid. This generates 2000+ error messages per minute.

**Evidence from logs**:
```
[18:49:32.456]E (81014) spi_master: check_trans_valid(1083): invalid dev handle
[18:49:32.477][E][spi-esp-idf:077]: Transmit failed - err 102
[18:49:32.478][W][spi-esp-idf:033]: SPI device not ready, cannot begin transaction
```

**Analysis**:
- 2085 SPI errors captured in 25 seconds
- Error code 102 = ESP_ERR_INVALID_ARG
- "invalid dev handle" suggests the SPI device wasn't properly initialized
- This is likely the ILI9341 display trying to communicate

**Possible Causes**:
1. Display SPI bus conflict with SD card SPI bus
2. Display not properly initialized before use
3. SPI bus configuration mismatch between display and SD card
4. Display component trying to use SPI before it's ready

**Investigation Needed**:
1. Check if display and SD card are on same SPI bus
2. Verify display initialization order in setup()
3. Check SPI bus configuration in YAML
4. Consider if display needs separate SPI bus from SD card

**Workaround**:
The device appears to function despite these errors (SD card works, system boots), but the log spam makes debugging difficult.

---

## Next Steps

1. **PRIORITY 1**: Fix SD card directory structure
   - Create `/animations` directory on SD card
   - Add animation files
   - Test that storage adapter finds them

2. **PRIORITY 2**: Investigate SPI display errors
   - Check YAML configuration for SPI bus conflicts
   - Review display initialization code
   - Consider separating display and SD card onto different SPI buses if possible

3. **PRIORITY 3**: Re-test with proper SD card structure
   - Capture new boot logs
   - Verify animations load from SD card
   - Confirm no storage-related warnings
