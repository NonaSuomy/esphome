#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class FT260Driver : public HIDDeviceDriver {
 public:
  FT260Driver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // FTDI FT260
    if (vid == 0x0403 && pid == 0x6030) {
      return true;
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.ft260", "FT260 USB-to-I2C Bridge detected");
    ESP_LOGI("usb_hidx.ft260", "Features: I2C Master, UART, 6x GPIO");
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 2)
      return;

    // FT260 HID report format
    uint8_t report_id = data[0];

    switch (report_id) {
      case 0xA0:  // Chip Version
        process_chip_version(data, len);
        break;
      case 0xA1:  // System Status
        process_system_status(data, len);
        break;
      case 0xC0:  // I2C Read Request Response
      case 0xC2:  // I2C Read Response
        process_i2c_read_response(data, len);
        break;
      case 0xD0:  // I2C Write Response
        ESP_LOGV("usb_hidx.ft260", "I2C write response");
        break;
      case 0xB0:  // GPIO Read Response
        process_gpio_response(data, len);
        break;
      case 0xE0:  // UART Status
        ESP_LOGV("usb_hidx.ft260", "UART status");
        break;
      default:
        ESP_LOGV("usb_hidx.ft260", "Unknown report: 0x%02X", report_id);
        break;
    }
  }

  const char *get_name() override { return "FT260"; }

 protected:
  void process_chip_version(const uint8_t *data, size_t len) {
    if (len < 5)
      return;

    uint8_t chip_code[4];
    memcpy(chip_code, &data[1], 4);

    ESP_LOGI("usb_hidx.ft260", "Chip Version: %c%c%c%c", chip_code[0], chip_code[1], chip_code[2], chip_code[3]);
  }

  void process_system_status(const uint8_t *data, size_t len) {
    if (len < 3)
      return;

    uint8_t chip_mode = data[1];
    uint8_t clock = data[2];

    const char *modes[] = {"I2C Master", "UART", "I2C Master + UART"};
    if (chip_mode < 3) {
      ESP_LOGV("usb_hidx.ft260", "Mode: %s, Clock: %dMHz", modes[chip_mode], clock == 0 ? 12 : clock == 1 ? 24 : 48);
    }
  }

  void process_i2c_read_response(const uint8_t *data, size_t len) {
    if (len < 3)
      return;

    uint8_t length = data[1];

    if (length > 0 && len >= 2 + length) {
      ESP_LOGI("usb_hidx.ft260", "I2C Read: %d bytes", length);
      ESP_LOGV("usb_hidx.ft260", "Data: [%02X %02X %02X...]", data[2], data[3], data[4]);
    }
  }

  void process_gpio_response(const uint8_t *data, size_t len) {
    if (len < 3)
      return;

    uint8_t gpio_state = data[2];

    if (gpio_state != last_gpio_state_) {
      ESP_LOGI("usb_hidx.ft260", "GPIO State: 0x%02X", gpio_state);
      for (int i = 0; i < 6; i++) {
        bool state = gpio_state & (1 << i);
        if (state != ((last_gpio_state_ & (1 << i)) != 0)) {
          ESP_LOGV("usb_hidx.ft260", "GPIO%d: %s", i, state ? "HIGH" : "LOW");
        }
      }
      last_gpio_state_ = gpio_state;
    }
  }

  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  uint8_t last_gpio_state_{0xFF};
};

}  // namespace usb_hidx
}  // namespace esphome
