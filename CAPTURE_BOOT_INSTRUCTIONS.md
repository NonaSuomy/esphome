# How to Capture Full Boot Sequence

The device is currently running, so we're only seeing the SPI errors (display issue). To see the storage initialization, we need to capture from the actual boot.

## Option 1: Reset Device and Capture (Recommended)

1. **Start the log capture first**:
   ```bash
   cd esphome003/esphome
   python3 capture_serial_logs.py config/vector-eyes-ttgo.yaml 40
   ```

2. **Within 5 seconds, press the RESET button on your ESP32**

3. **Wait for capture to complete** (40 seconds)

4. **Check the results**:
   ```bash
   cat serial_boot_log.txt | grep -E "storage|SD|mount|animation|Found"
   ```

## Option 2: Manual Logs

If you prefer to watch the logs manually:

```bash
cd esphome003/esphome
source ./venv/bin/activate
esphome logs config/vector-eyes-ttgo.yaml --device /dev/ttyUSB0
```

Then press RESET on the device and watch for these messages.

## What to Look For

### ✅ SUCCESS - You should see:
```
[I][sd_storage.spi:183]: SD card mounted successfully at /sd
[I][vector_eyes.storage_adapter:XXX]: Set preferred mount path: /sd
[I][vector_eyes.storage_adapter:XXX]: Found device with animations: sd_card at /sd
[D][vector_eyes.storage_adapter:XXX]: Found 1191 animation files
[I][vector_eyes:XXX]: Storage adapter initialized successfully
```

### ❌ FAILURE - You would see:
```
[W][vector_eyes.storage_adapter:054]: No storage device with animations found
[W][vector_eyes:035]: Storage adapter initialization failed
[W][vector_eyes:088]: No storage available, using internal animations only
```

## Current Issue

The device is running but we're only seeing SPI display errors. We need to see the boot sequence to verify if the SD card and animations were found.

**Next step**: Reset the device while the log capture is running to see the full boot sequence.
