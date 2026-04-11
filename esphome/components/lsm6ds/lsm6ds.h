#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

#ifdef USE_SPI
#include "esphome/components/spi/spi.h"
#endif

namespace esphome {
namespace lsm6ds {

enum LSM6DSAccelRange {
  LSM6DS_ACCEL_RANGE_2G = 0x00,
  LSM6DS_ACCEL_RANGE_16G = 0x01,
  LSM6DS_ACCEL_RANGE_4G = 0x02,
  LSM6DS_ACCEL_RANGE_8G = 0x03,
};

enum LSM6DSGyroRange {
  LSM6DS_GYRO_RANGE_250DPS = 0x00,
  LSM6DS_GYRO_RANGE_500DPS = 0x01,
  LSM6DS_GYRO_RANGE_1000DPS = 0x02,
  LSM6DS_GYRO_RANGE_2000DPS = 0x03,
  LSM6DS_GYRO_RANGE_125DPS = 0x04,
};

enum LSM6DSODR {
  LSM6DS_ODR_OFF = 0x00,
  LSM6DS_ODR_12_5HZ = 0x01,
  LSM6DS_ODR_26HZ = 0x02,
  LSM6DS_ODR_52HZ = 0x03,
  LSM6DS_ODR_104HZ = 0x04,
  LSM6DS_ODR_208HZ = 0x05,
  LSM6DS_ODR_416HZ = 0x06,
  LSM6DS_ODR_833HZ = 0x07,
  LSM6DS_ODR_1_66KHZ = 0x08,
  LSM6DS_ODR_3_33KHZ = 0x09,
  LSM6DS_ODR_6_66KHZ = 0x0A,
};

class LSM6DSComponent : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_accel_x_sensor(sensor::Sensor *accel_x) { accel_x_sensor_ = accel_x; }
  void set_accel_y_sensor(sensor::Sensor *accel_y) { accel_y_sensor_ = accel_y; }
  void set_accel_z_sensor(sensor::Sensor *accel_z) { accel_z_sensor_ = accel_z; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x) { gyro_x_sensor_ = gyro_x; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y) { gyro_y_sensor_ = gyro_y; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z) { gyro_z_sensor_ = gyro_z; }
  void set_temperature_sensor(sensor::Sensor *temp) { temperature_sensor_ = temp; }

  void set_accel_range(LSM6DSAccelRange range) { accel_range_ = range; }
  void set_gyro_range(LSM6DSGyroRange range) { gyro_range_ = range; }
  void set_accel_odr(LSM6DSODR odr) { accel_odr_ = odr; }
  void set_gyro_odr(LSM6DSODR odr) { gyro_odr_ = odr; }

 protected:
  virtual bool read_register(uint8_t reg, uint8_t *data, size_t len) = 0;
  virtual bool write_register(uint8_t reg, uint8_t data) = 0;

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  LSM6DSAccelRange accel_range_;
  LSM6DSGyroRange gyro_range_;
  LSM6DSODR accel_odr_;
  LSM6DSODR gyro_odr_;

  float accel_sensitivity_;
  float gyro_sensitivity_;
};

class LSM6DSI2CDevice : public LSM6DSComponent, public i2c::I2CDevice {
 public:
  bool read_register(uint8_t reg, uint8_t *data, size_t len) override {
    return this->read_bytes(reg, data, len);
  }
  bool write_register(uint8_t reg, uint8_t data) override {
    return this->write_byte(reg, data);
  }
};

#ifdef USE_SPI
class LSM6DSSPIDevice : public LSM6DSComponent, public spi::SPIDevice {
 public:
  bool read_register(uint8_t reg, uint8_t *data, size_t len) override {
    this->enable();
    this->transfer_byte(reg | 0x80);
    this->transfer_bytes(nullptr, data, len);
    this->disable();
    return true;
  }
  bool write_register(uint8_t reg, uint8_t data) override {
    this->enable();
    this->transfer_byte(reg & 0x7F);
    this->transfer_byte(data);
    this->disable();
    return true;
  }
};
#endif

}  // namespace lsm6ds
}  // namespace esphome
