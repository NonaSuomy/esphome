# LSM6DS SPI Example Configuration

This component supports LSM6DS family IMU sensors (accelerometer + gyroscope) via SPI interface.

## Supported Sensors
- LSM6DSOX (WHO_AM_I: 0x6C)
- LSM6DS3TR-C (WHO_AM_I: 0x6A)
- LSM6DSO32 (WHO_AM_I: 0x69)

## Example 1: All-in-one Pattern

Configure the sensor with all outputs in the component block:

```yaml
spi:
  - id: spi_bus
    clk_pin: GPIO12
    mosi_pin: GPIO11
    miso_pin: GPIO13

lsm6ds_spi:
  id: imu_sensor
  spi_id: spi_bus
  cs_pin: GPIO10
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
spi:
  - id: spi_bus
    clk_pin: GPIO12
    mosi_pin: GPIO11
    miso_pin: GPIO13

lsm6ds_spi:
  id: imu_hub
  spi_id: spi_bus
  cs_pin: GPIO10
  accel_range: 4G
  gyro_range: 500DPS
  accel_odr: 104HZ
  gyro_odr: 104HZ
  update_interval: 50ms

sensor:
  - platform: lsm6ds_spi
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

## SPI Configuration

The LSM6DS SPI implementation uses:
- **Mode 0** or **Mode 3** (CPOL=0, CPHA=0 or CPOL=1, CPHA=1)
- **MSB first**
- **Read bit**: MSB set to 1 (reg | 0x80)
- **Write bit**: MSB set to 0 (reg & 0x7F)
- **Max speed**: 10 MHz

The component handles the SPI protocol automatically.
