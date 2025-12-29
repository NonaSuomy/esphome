#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class LogitechDriver : public HIDDeviceDriver {
 public:
  LogitechDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Logitech Unifying Receiver
    if (vid == 0x046D && (pid == 0xC52B || pid == 0xC532 || pid == 0xC52F)) {
      return true;
    }
    // Logitech wireless devices (direct connection)
    if (vid == 0x046D) {
      return true;  // Accept all Logitech devices
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.logitech", "Logitech device detected");
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 3)
      return;

    // Logitech HID++ protocol detection
    if (data[0] == 0x10 || data[0] == 0x11) {
      // HID++ short/long message
      process_hidpp_report(data, len);
    } else {
      // Standard HID report (mouse/keyboard)
      process_standard_report(data, len);
    }
  }

  const char *get_name() override { return "Logitech"; }

 protected:
  void process_hidpp_report(const uint8_t *data, size_t len) {
    // HID++ protocol for Logitech devices
    uint8_t device_index = data[1];
    uint8_t feature = data[2];

    ESP_LOGV("usb_hidx.logitech", "HID++ report: device=%d, feature=0x%02X", device_index, feature);
  }

  void process_standard_report(const uint8_t *data, size_t len) {
    // Standard mouse/keyboard reports
    if (len >= 3) {
      // Mouse report format
      uint8_t buttons = data[0];
      int8_t x = data[1];
      int8_t y = data[2];

      if (buttons != last_buttons_) {
        if (buttons & 0x01)
          ESP_LOGI("usb_hidx.logitech", "Left Click");
        if (buttons & 0x02)
          ESP_LOGI("usb_hidx.logitech", "Right Click");
        if (buttons & 0x04)
          ESP_LOGI("usb_hidx.logitech", "Middle Click");
        last_buttons_ = buttons;
      }
    }
  }

  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  uint8_t last_buttons_{0};
};

}  // namespace usb_hidx
}  // namespace esphome
