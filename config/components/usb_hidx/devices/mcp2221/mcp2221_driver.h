#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <functional>

namespace esphome {
namespace usb_hidx {

class MCP2221Driver : public HIDDeviceDriver {
 public:
  MCP2221Driver(USBHIDXComponent *parent) : parent_(parent) { response_sem_ = xSemaphoreCreateBinary(); }

  ~MCP2221Driver() {
    if (response_sem_) {
      vSemaphoreDelete(response_sem_);
    }
  }

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    if (vid == 0x04D8 && pid == 0x00DD) {
      return true;
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.mcp2221", "MCP2221A USB-to-I2C Bridge detected");
    ESP_LOGI("usb_hidx.mcp2221", "Features: I2C Master, 4x GPIO, 3x ADC");
  }

  // Synchronous I2C write
  bool i2c_write_sync(uint8_t address, const uint8_t *data, size_t len) {
    if (!device_ || len > 60)
      return false;

    uint8_t cmd[64] = {0};
    cmd[0] = 0x90;  // I2C Write Data
    cmd[1] = (uint8_t) len;
    cmd[2] = 0;
    cmd[3] = address << 1;
    memcpy(&cmd[4], data, len);

    response_received_ = false;
    esp_err_t err = parent_->send_hid_output_report(device_, cmd, 64);
    if (err != ESP_OK)
      return false;

    // Wait for response (100ms timeout)
    if (xSemaphoreTake(response_sem_, pdMS_TO_TICKS(100)) == pdTRUE) {
      return response_status_ == 0x00;
    }
    return false;
  }

  // Synchronous I2C read
  bool i2c_read_sync(uint8_t address, uint8_t *data, size_t len) {
    if (!device_ || len > 60)
      return false;

    uint8_t cmd[64] = {0};
    cmd[0] = 0x91;  // I2C Read Data
    cmd[1] = (uint8_t) len;
    cmd[2] = 0;
    cmd[3] = (address << 1) | 0x01;

    response_received_ = false;
    response_data_len_ = 0;
    esp_err_t err = parent_->send_hid_output_report(device_, cmd, 64);
    if (err != ESP_OK)
      return false;

    // Wait for response (100ms timeout)
    if (xSemaphoreTake(response_sem_, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (response_status_ == 0x00 && response_data_len_ > 0) {
        memcpy(data, response_data_, response_data_len_);
        return true;
      }
    }
    return false;
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 2)
      return;

    uint8_t cmd = data[0];

    switch (cmd) {
      case 0x10:  // Status response
        response_status_ = data[1];
        response_received_ = true;
        xSemaphoreGive(response_sem_);
        break;
      case 0x91:  // I2C Read Data response
        process_i2c_read_response(data, len);
        break;
      default:
        ESP_LOGV("usb_hidx.mcp2221", "Response: 0x%02X", cmd);
        break;
    }
  }

  const char *get_name() override { return "MCP2221"; }

 protected:
  void process_i2c_read_response(const uint8_t *data, size_t len) {
    if (len < 4)
      return;

    response_status_ = data[1];
    if (response_status_ == 0x00) {
      response_data_len_ = data[3];
      if (response_data_len_ > 0 && len >= 4 + response_data_len_) {
        memcpy(response_data_, &data[4], response_data_len_);
        ESP_LOGV("usb_hidx.mcp2221", "I2C Read: %d bytes", response_data_len_);
      }
    }
    response_received_ = true;
    xSemaphoreGive(response_sem_);
  }

  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  SemaphoreHandle_t response_sem_{nullptr};
  bool response_received_{false};
  uint8_t response_status_{0xFF};
  uint8_t response_data_[64];
  size_t response_data_len_{0};
};

}  // namespace usb_hidx
}  // namespace esphome
