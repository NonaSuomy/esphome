#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class CP2112Driver : public HIDDeviceDriver {
 public:
  CP2112Driver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Silicon Labs CP2112
    if (vid == 0x10C4 && pid == 0xEA90) {
      return true;
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.cp2112", "CP2112 USB-to-SMBus/I2C Bridge detected");
    ESP_LOGI("usb_hidx.cp2112", "Features: SMBus/I2C Master, 8x GPIO");
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 2)
      return;

    // CP2112 HID report format
    uint8_t report_id = data[0];

    switch (report_id) {
      case 0x10:  // SMBus Configuration response
        ESP_LOGV("usb_hidx.cp2112", "SMBus config response");
        break;
      case 0x13:  // Data Read Response
        process_data_read_response(data, len);
        break;
      case 0x14:  // Transfer Status Response
        process_transfer_status(data, len);
        break;
      case 0x15:  // Data Read Force Send
        ESP_LOGV("usb_hidx.cp2112", "Data read force send");
        break;
      case 0x20:  // GPIO Get response
        process_gpio_response(data, len);
        break;
      case 0x21:  // GPIO Set response
        ESP_LOGV("usb_hidx.cp2112", "GPIO set response");
        break;
      case 0x22:  // Version response
        process_version_response(data, len);
        break;
      default:
        ESP_LOGV("usb_hidx.cp2112", "Unknown report: 0x%02X", report_id);
        break;
    }
  }

  const char *get_name() override { return "CP2112"; }

 protected:
  void process_data_read_response(const uint8_t *data, size_t len) {
    if (len < 4)
      return;

    uint8_t status = data[1];
    uint8_t length = data[2];

    if (status == 0x00) {
      ESP_LOGI("usb_hidx.cp2112", "I2C Read: %d bytes", length);
      if (len >= 3 + length) {
        ESP_LOGV("usb_hidx.cp2112", "Data: [%02X %02X %02X...]", data[3], data[4], data[5]);
      }
    } else if (status == 0x02) {
      ESP_LOGW("usb_hidx.cp2112", "I2C Read: Busy");
    } else if (status == 0x03) {
      ESP_LOGW("usb_hidx.cp2112", "I2C Read: Error");
    }
  }

  void process_transfer_status(const uint8_t *data, size_t len) {
    if (len < 7)
      return;

    uint8_t status0 = data[1];
    uint8_t status1 = data[2];
    uint16_t retries = (data[3] << 8) | data[4];
    uint16_t bytes_read = (data[5] << 8) | data[6];

    if (status0 == 0x00) {
      ESP_LOGV("usb_hidx.cp2112", "Transfer: Idle");
    } else if (status0 == 0x01) {
      ESP_LOGV("usb_hidx.cp2112", "Transfer: Busy");
    } else if (status0 == 0x02) {
      ESP_LOGI("usb_hidx.cp2112", "Transfer: Complete (%d bytes)", bytes_read);
    } else if (status0 == 0x03) {
      ESP_LOGW("usb_hidx.cp2112", "Transfer: Error (status1=0x%02X)", status1);
    }
  }

  void process_gpio_response(const uint8_t *data, size_t len) {
    if (len < 3)
      return;

    uint8_t gpio_state = data[2];

    if (gpio_state != last_gpio_state_) {
      ESP_LOGI("usb_hidx.cp2112", "GPIO State: 0x%02X", gpio_state);
      for (int i = 0; i < 8; i++) {
        bool state = gpio_state & (1 << i);
        if (state != ((last_gpio_state_ & (1 << i)) != 0)) {
          ESP_LOGV("usb_hidx.cp2112", "GPIO%d: %s", i, state ? "HIGH" : "LOW");
        }
      }
      last_gpio_state_ = gpio_state;
    }
  }

  void process_version_response(const uint8_t *data, size_t len) {
    if (len < 3)
      return;

    uint8_t part_num = data[1];
    uint8_t version = data[2];

    ESP_LOGI("usb_hidx.cp2112", "Part: 0x%02X, Version: %d.%d", part_num, version >> 4, version & 0x0F);
  }

  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  uint8_t last_gpio_state_{0xFF};
};

}  // namespace usb_hidx
}  // namespace esphome
