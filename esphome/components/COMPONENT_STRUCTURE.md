# LSM6DS Component Structure

This document describes the organization of the LSM6DS family IMU sensor components for ESPHome.

## Folder Structure

```
esphome/components/
├── lsm6ds_base/              # Base component (shared code and logic)
│   ├── __init__.py          # Base schema, enums, configuration, register_lsm6ds()
│   ├── const.py             # Constants: register addresses, WHO_AM_I values, CONF_* keys
│   ├── lsm6ds.h             # Base C++ class: LSM6DSComponent (abstract)
│   └── lsm6ds.cpp           # Base C++ implementation: setup(), update(), dump_config()
│
├── lsm6ds_i2c/              # I2C interface implementation
│   ├── __init__.py          # I2C component registration (AUTO_LOAD: lsm6ds_base)
│   ├── sensor.py            # Optional sensor platform (hub + sensor pattern)
│   ├── lsm6ds_i2c.h         # LSM6DSI2CDevice class (implements read/write via I2C)
│   ├── lsm6ds_i2c.cpp       # I2C implementation (minimal/empty)
│   └── README_EXAMPLE.md    # I2C usage examples and configuration guide
│
└── lsm6ds_spi/              # SPI interface implementation
    ├── __init__.py          # SPI component registration (AUTO_LOAD: lsm6ds_base)
    ├── sensor.py            # Optional sensor platform (hub + sensor pattern)
    ├── lsm6ds_spi.h         # LSM6DSSPIDevice class (implements read/write via SPI)
    ├── lsm6ds_spi.cpp       # SPI implementation (minimal/empty)
    └── README_EXAMPLE.md    # SPI usage examples and configuration guide
```

## Component Relationships

### Auto-Loading Pattern
Both `lsm6ds_i2c` and `lsm6ds_spi` use `AUTO_LOAD = ["lsm6ds_base"]` to automatically include the base component when either interface variant is used. Users only need to configure `lsm6ds_i2c` or `lsm6ds_spi` directly.

### Dependencies
- **lsm6ds_base**: No dependencies (standalone base)
- **lsm6ds_i2c**: Depends on `i2c`, auto-loads `lsm6ds_base`
- **lsm6ds_spi**: Depends on `spi`, auto-loads `lsm6ds_base`

### Class Hierarchy

```
C++ Class Structure:

LSM6DSComponent (abstract base class)
  ↑
  ├── LSM6DSI2CDevice : public LSM6DSComponent, public i2c::I2CDevice
  │   └── Implements: read_register() and write_register() via I2C
  │
  └── LSM6DSSPIDevice : public LSM6DSComponent, public spi::SPIDevice
      └── Implements: read_register() and write_register() via SPI
```

## Usage Patterns

### Pattern 1: All-in-One (Recommended for most users)
Configure the sensor with all outputs directly in the component block:

```yaml
lsm6ds_i2c:  # or lsm6ds_spi
  id: imu_sensor
  # ... interface config (I2C address or SPI CS pin)
  accel_range: 4G
  gyro_range: 500DPS
  accel_odr: 104HZ
  gyro_odr: 104HZ
  accel_x:
    name: "Accel X"
  gyro_x:
    name: "Gyro X"
  # ... other sensors
```

### Pattern 2: Hub + Sensor Platform
Configure the hub separately, then add sensors under the sensor platform:

```yaml
lsm6ds_i2c:  # or lsm6ds_spi
  id: imu_hub
  # ... interface config
  accel_range: 4G
  gyro_range: 500DPS

sensor:
  - platform: lsm6ds_i2c  # or lsm6ds_spi
    lsm6ds_id: imu_hub
    accel_x:
      name: "Accel X"
    # ... other sensors
```

## Supported Sensors

- **LSM6DSOX**: WHO_AM_I = 0x6C
- **LSM6DS3TR-C**: WHO_AM_I = 0x6A
- **LSM6DSO32**: WHO_AM_I = 0x69

All variants support:
- 3-axis accelerometer (±2G, ±4G, ±8G, ±16G)
- 3-axis gyroscope (±125, ±250, ±500, ±1000, ±2000 DPS)
- Temperature sensor
- Configurable output data rate (12.5Hz to 6.66kHz)
- Both I2C and SPI interfaces

## Code Generation Flow

1. User configures `lsm6ds_i2c` or `lsm6ds_spi` in YAML
2. ESPHome auto-loads `lsm6ds_base` component
3. Python `__init__.py` validates config and generates C++ code
4. `register_lsm6ds()` function:
   - Registers component with ESPHome
   - Sets range and ODR configuration
   - Creates sensor entities for requested outputs
5. C++ `setup()` method:
   - Detects sensor variant via WHO_AM_I
   - Configures sensor registers
   - Calculates sensitivity values
6. C++ `update()` method:
   - Reads sensor data
   - Publishes to sensor entities

## CODEOWNERS

All three components list `@NonaSuomy` as the code owner for maintenance and review.
