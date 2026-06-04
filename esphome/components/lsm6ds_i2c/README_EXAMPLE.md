# LSM6DS I2C Example Configuration

This component supports LSM6DS family IMU sensors (accelerometer + gyroscope) via I2C interface.

## Supported Sensors
- LSM6DSOX (WHO_AM_I: 0x6C, I2C address: 0x6A or 0x6B)
- LSM6DS3TR-C (WHO_AM_I: 0x6A, I2C address: 0x6A or 0x6B)
- LSM6DSO32 (WHO_AM_I: 0x69, I2C address: 0x6A or 0x6B)

## Example 1: All-in-one Pattern

Configure the sensor with all outputs in the component block:

```yaml
i2c:
  - id: i2c_bus
    sda: GPIO21
    scl: GPIO22
    scan: true

lsm6ds_i2c:
  id: imu_sensor
  i2c_id: i2c_bus
  address: 0x6B
  accel_range: 4G
  gyro_range: 500DPS
  accel_odr: 104HZ
  gyro_odr: 104HZ
  update_interval: 50ms
  accel_x:
    name: "Accel X"
  accel_y:
    name: "Accel Y"
  accel_z:
    name: "Accel Z"
  gyro_x:
    name: "Gyro X"
  gyro_y:
    name: "Gyro Y"
  gyro_z:
    name: "Gyro Z"
  temperature:
    name: "IMU Temperature"
```

## Example 2: Hub + Sensor Platform Pattern

Configure the hub separately, then add sensor outputs under the sensor platform:

```yaml
i2c:
  - id: i2c_bus
    sda: GPIO21
    scl: GPIO22
    scan: true

lsm6ds_i2c:
  id: imu_hub
  i2c_id: i2c_bus
  address: 0x6B
  accel_range: 4G
  gyro_range: 500DPS
  accel_odr: 104HZ
  gyro_odr: 104HZ
  update_interval: 50ms

sensor:
  - platform: lsm6ds_i2c
    lsm6ds_id: imu_hub
    accel_x:
      name: "Accel X"
    accel_y:
      name: "Accel Y"
    accel_z:
      name: "Accel Z"
    gyro_x:
      name: "Gyro X"
    gyro_y:
      name: "Gyro Y"
    gyro_z:
      name: "Gyro Z"
    temperature:
      name: "IMU Temperature"
```

## Configuration Options

### I2C Address
- `0x6A` (default, SDO/SA0 pin LOW)
- `0x6B` (SDO/SA0 pin HIGH)

### Accelerometer Range
- `2G` (default)
- `4G`
- `8G`
- `16G`

### Gyroscope Range
- `125DPS`
- `250DPS` (default)
- `500DPS`
- `1000DPS`
- `2000DPS`

### Output Data Rate (ODR)
- `12.5HZ`
- `26HZ`
- `52HZ`
- `104HZ` (default)
- `208HZ`
- `416HZ`
- `833HZ`
- `1660HZ`
- `3330HZ`
- `6660HZ`

## I2C Configuration

The LSM6DS I2C implementation uses standard I2C communication:
- **Speed**: Supports up to 400 kHz (Fast mode)
- **7-bit addressing**: 0x6A or 0x6B depending on SDO/SA0 pin state
- **Auto-increment**: Multi-byte reads automatically increment register address

The component handles the I2C protocol automatically.
