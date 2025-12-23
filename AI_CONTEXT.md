# AI Context Handoff - Vector Eyes Project
**Last Updated:** 2025-12-15
**Status:** STABLE / VERIFIED

This document serves as a restore point for future AI sessions. It summarizes the current state of the codebase, recent fixes, known behavior, and essential development commands.

## 1. Project Overview
**Device:** TTGO Camera Plus (ESP32)
**Goal:** Play animations (stored on NFS) on a display based on triggers.
**Key Components:** `vector_eyes` (C++ component), `esphome` (framework), `nfs_client`.

## 2. Recent Accomplishments (Memory Optimization)
We resolved critical Out-of-Memory (OOM) crashes and Watchdog Timeouts (WDT) caused by loading large JSON files from network storage.

### Fix 1: Streaming JSON Parser (`BehaviorEngine`)
*   **Problem:** Loading `AnimationTriggerMap.json` (50KB) allocating a full buffer caused OOM.
*   **Solution:** Refactored `load_trigger_map` to use `ArduinoJson`'s streaming parser directly from the file stream.
*   **Result:** Map loads with constant low memory usage.

### Fix 2: Buffered Read Stream (`StorageAdapterStream`)
*   **Problem:** `ArduinoJson` reads byte-by-byte. Doing this over NFS caused excessive network roundtrips, triggering the Task Watchdog (5s timeout).
*   **Solution:** Added a **512-byte internal buffer** to `StorageAdapterStream`.
*   **Result:** Reads are batched. WDT avoided. Animations play smoothly.

### Fix 3: Scripted Port Management
*   **Problem:** `[Errno 11] Resource temporarily unavailable` when trying to upload logging commands due to zombie processes holding the serial port.
*   **Solution:** Updated `script/run-in-env.py` to auto-detect and kill processes locking `/dev/tty*` devices before execution.

## 3. Current Behavior & Known Issues
*   **Animation Playback:** Works correctly. Animations like `anim_lookinplaceforfaces...` play on the screen.
*   **NFS Lookup Delay (Non-Critical):**
    *   **Symptom:** A ~1.5s delay occurs between Trigger selection and Animation start.
    *   **Log:** `[W] [nfs_client] LOOKUP failed: status=2` (repeated 3 times).
    *   **Cause:** The system probes for the file (likely checking file extensions or paths) and fails a few times before succeeding.
    *   **Status:** **Known/Acceptable.** Do not "fix" unless user requests latency optimization.

## 4. Development Commands
Use the `script/run-in-env.py` wrapper. It handles the virtual environment AND serial port cleanup automatically.

### Compile and Run (Upload + Monitor)
```bash
python3 script/run-in-env.py esphome run config/vector-eyes-ttgo.yaml --device /dev/ttyUSB0
```

### View Logs Only
```bash
python3 script/run-in-env.py esphome logs config/vector-eyes-ttgo.yaml --device /dev/ttyUSB0
```

### Compile Only (No Upload)
```bash
python3 script/run-in-env.py esphome compile config/vector-eyes-ttgo.yaml
```

## 5. File Locations
*   **Vector Source Repo**: `/home/nonasuomy/code/esphome003/esphome/victor_repo/` (Original source for porting)
*   **Raw Audio Assets**: `/home/nonasuomy/code/vector-audio-raw/VictorAudio/Originals/SFX/` (Original WAV files)
*   **ESPHome Config**: `esphome/config/vector-eyes-ttgo.yaml`
*   **Component Source:** `esphome/esphome/components/vector_eyes/`
    *   `behavior_engine.cpp`: Map loading logic.
    *   `storage_adapter.h`: `StorageAdapterStream` (Buffering logic).
    *   `vector_eyes.cpp`: Animation playback logic.
