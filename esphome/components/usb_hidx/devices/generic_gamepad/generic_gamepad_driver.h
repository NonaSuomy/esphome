#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class GenericGamepadDriver : public HIDDeviceDriver {
 public:
  GenericGamepadDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    return protocol == 0x00;  // Generic HID (gamepads typically use protocol 0)
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (len < 8)
      return;

    // Generic gamepad button parsing (varies by device)
    // This is a basic implementation for common gamepads
    uint8_t btn0 = data[0];
    uint8_t btn1 = data[1];

    // Track button state changes
    if (btn0 != last_buttons_[0]) {
      if ((btn0 & 0x02) && !(last_buttons_[0] & 0x02)) {
        ESP_LOGI("usb_hidx.gamepad", "Button B pressed");
      }
      if ((btn0 & 0x04) && !(last_buttons_[0] & 0x04)) {
        ESP_LOGI("usb_hidx.gamepad", "Button A pressed");
      }
      last_buttons_[0] = btn0;
    }

    if (btn1 != last_buttons_[1]) {
      if ((btn1 & 0x10) && !(last_buttons_[1] & 0x10)) {
        ESP_LOGI("usb_hidx.gamepad", "Home button pressed");
      }
      last_buttons_[1] = btn1;
    }
  }

  const char *get_name() override { return "GenericGamepad"; }

 protected:
  USBHIDXComponent *parent_;
  uint8_t last_buttons_[2]{0};
};

}  // namespace usb_hidx
}  // namespace esphome
