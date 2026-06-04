#pragma once

#include "esphome/components/lsm6ds_base/lsm6ds_base.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace lsm6ds_spi {

class LSM6DSSPIDevice : public lsm6ds_base::LSM6DSComponent,
                        public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                              spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
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

}  // namespace lsm6ds_spi
}  // namespace esphome
