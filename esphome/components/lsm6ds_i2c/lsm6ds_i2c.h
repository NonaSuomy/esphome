#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/lsm6ds_base/lsm6ds.h"

namespace esphome {
namespace lsm6ds_i2c {

class LSM6DSI2CDevice : public lsm6ds_base::LSM6DSComponent, public i2c::I2CDevice {
 public:
  bool read_register(uint8_t reg, uint8_t *data, size_t len) override { return this->read_bytes(reg, data, len); }
  bool write_register(uint8_t reg, uint8_t data) override { return this->write_byte(reg, data); }
};

}  // namespace lsm6ds_i2c
}  // namespace esphome
