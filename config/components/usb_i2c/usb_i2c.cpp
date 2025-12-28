#include "usb_i2c.h"
#include "esphome/core/log.h"
#include "../usb_hidx/devices/mcp2221/mcp2221_driver.h"

namespace esphome {
namespace usb_i2c {

static const char *const TAG = "usb_i2c";

// Global pointer for driver registration
USBI2CBus *global_usb_i2c_bus = nullptr;

void USBI2CBus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up USB I2C Bus...");
  global_usb_i2c_bus = this;
}

void USBI2CBus::dump_config() {
  ESP_LOGCONFIG(TAG, "USB I2C Bus:");
  ESP_LOGCONFIG(TAG, "  Scan: %s", YESNO(this->scan_));
  ESP_LOGCONFIG(TAG, "  Frequency: %u Hz", this->frequency_);

  if (this->mcp2221_driver_) {
    ESP_LOGCONFIG(TAG, "  Bridge: MCP2221A detected");
  } else {
    ESP_LOGW(TAG, "  Bridge: Not detected (plug in MCP2221A)");
  }
}

void USBI2CBus::loop() {
  if (this->scan_ && this->mcp2221_driver_ && !this->scanned_) {
    this->scanned_ = true;
    ESP_LOGI(TAG, "Scanning I2C bus...");

    for (uint8_t address = 0x08; address < 0x78; address++) {
      uint8_t dummy;
      i2c::ErrorCode err = this->write_readv(address, nullptr, 0, &dummy, 1);
      if (err == i2c::ERROR_OK) {
        ESP_LOGI(TAG, "Found I2C device at address 0x%02X", address);
      }
    }
  }
}

void USBI2CBus::set_mcp2221_driver(usb_hidx::MCP2221Driver *driver) {
  this->mcp2221_driver_ = driver;
  ESP_LOGI(TAG, "MCP2221A driver registered");
  this->scanned_ = false;  // Trigger rescan
}

i2c::ErrorCode USBI2CBus::write_readv(uint8_t address, const uint8_t *write_buffer, size_t write_count,
                                      uint8_t *read_buffer, size_t read_count) {
  if (!this->mcp2221_driver_) {
    return i2c::ERROR_NOT_INITIALIZED;
  }

  // Write-then-read transaction
  if (write_count > 0 && read_count > 0) {
    if (!this->mcp2221_driver_->i2c_write_sync(address, write_buffer, write_count)) {
      return i2c::ERROR_TIMEOUT;
    }
    if (!this->mcp2221_driver_->i2c_read_sync(address, read_buffer, read_count)) {
      return i2c::ERROR_TIMEOUT;
    }
    return i2c::ERROR_OK;
  }

  // Write-only transaction
  if (write_count > 0) {
    if (!this->mcp2221_driver_->i2c_write_sync(address, write_buffer, write_count)) {
      return i2c::ERROR_TIMEOUT;
    }
    return i2c::ERROR_OK;
  }

  // Read-only transaction
  if (read_count > 0) {
    if (!this->mcp2221_driver_->i2c_read_sync(address, read_buffer, read_count)) {
      return i2c::ERROR_TIMEOUT;
    }
    return i2c::ERROR_OK;
  }

  return i2c::ERROR_OK;
}

}  // namespace usb_i2c
}  // namespace esphome
