#!/usr/bin/env python3
"""Analyze the boot logs from the user's paste"""

# The logs from the user's message
logs = """[19:16:46.211][I][logger:121]: Log initialized[19:16:46.212][C][safe_mode:084]: Unsuccessful boot attempts: 0[19:16:46.214][D][esp32.preferences:149]: Writing 1 items: 0 cached, 1 written, 0 failed[19:16:46.222][D][vector_eyes.storage_adapter:613]: Initialized 2 buffer pools of 4096 bytes each[19:16:46.223][I][app:073]: Running through setup()[19:16:46.223][I][i2c.idf:191]: Performing bus recovery[19:16:46.262][C][component:199]: Setup i2c took 40ms[19:16:46.263][C][component:199]: Setup spi took 1ms[19:16:46.263][C][storage:065]: Setting up Storage Host Component...(92) spi_hal: The clock_speed_hz should less than 26666666[19:16:46.286]E (95) spi_master: spi_bus_add_device(500): assigned clock speed not supported[19:16:46.286]0;35m[C][storage:070]:   Mounts configured: 0[19:16:46.287][C][storage:075]:   Device nodes configured: 0[19:16:46.287][C][component:199]: Setup storage took 0ms"""

print("=" * 80)
print("BOOT LOG ANALYSIS")
print("=" * 80)
print()

# Check for key messages
print("KEY FINDINGS:")
print()

if "Initialized 2 buffer pools" in logs:
    print("✅ Storage adapter buffer pools initialized")
else:
    print("❌ Storage adapter buffer pools NOT initialized")

if "Setting up Storage Host Component" in logs:
    print("✅ Storage component setup started")
else:
    print("❌ Storage component NOT setup")

if "Mounts configured: 0" in logs:
    print("❌ CRITICAL: No mounts configured! SD card not mounted!")
else:
    print("✅ Mounts configured")

if "Device nodes configured: 0" in logs:
    print("❌ CRITICAL: No device nodes configured! SD card not detected!")
else:
    print("✅ Device nodes configured")

if "spi_bus_add_device" in logs and "assigned clock speed not supported" in logs:
    print("⚠️  WARNING: SPI clock speed issue detected")

print()
print("=" * 80)
print("DIAGNOSIS:")
print("=" * 80)
print()
print("The storage component initialized but found:")
print("  - 0 mounts configured")
print("  - 0 device nodes configured")
print()
print("This means the sd_storage component did NOT initialize properly.")
print("The SD card was not detected or mounted during boot.")
print()
print("POSSIBLE CAUSES:")
print("1. SD card not properly inserted")
print("2. SD card SPI configuration issue")
print("3. sd_storage component not configured in YAML")
print("4. SPI clock speed incompatibility")
print()
print("NEXT STEP: Check the YAML configuration for sd_storage component")
