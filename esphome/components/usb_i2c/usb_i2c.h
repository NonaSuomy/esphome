#pragma once

#include "esphome/core/component.h"
#include "i2c_bus.h"

namespace esphome {

// Forward declare MCP2221Driver
namespace usb_hidx {
class MCP2221Driver;
}

namespace usb_i2c {

class USBI2CBus : public i2c::I2CBus, public Component {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  // I2C interface implementation
  i2c::ErrorCode write_readv(uint8_t address, const uint8_t *write_buffer, size_t write_count, uint8_t *read_buffer,
                             size_t read_count) override;

  void set_scan(bool scan) { this->scan_ = scan; }
  void set_frequency(uint32_t frequency) { this->frequency_ = frequency; }

  // Bridge driver registration
  void set_mcp2221_driver(usb_hidx::MCP2221Driver *driver);

 protected:
  uint32_t frequency_{100000};
  usb_hidx::MCP2221Driver *mcp2221_driver_{nullptr};
  bool scanned_{false};
};

// Global pointer for driver registration
extern USBI2CBus *global_usb_i2c_bus;

}  // namespace usb_i2c
}  // namespace esphome
