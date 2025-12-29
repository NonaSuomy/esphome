#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class GenericGamepadDriver : public HIDDeviceDriver {
 public:
  GenericGamepadDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Don't match Xbox 360 (handled by Xbox360Driver)
    if (vid == 0x045E && pid == 0x028E)
      return false;
    // Don't match Switch controllers (handled by SwitchDriver)
    if ((vid == 0x057E && pid == 0x2009) || (vid == 0x20D6 && pid == 0xA713))
      return false;
    // Match generic HID gamepads (protocol 0) as fallback
    return protocol == 0x00;
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (len < 8)
      return;

    // PowerA Switch controller format: [btn0, btn1, dpad, lx, ly, rx, ry, 0x00]
    uint8_t btn0 = data[0];
    uint8_t btn1 = data[1];
    uint8_t dpad = data[2];
    uint8_t lx = data[3];
    uint8_t ly = data[4];
    uint8_t rx = data[5];
    uint8_t ry = data[6];

    // Track button state changes
    if (btn0 != last_buttons_[0]) {
      // Byte 0: Y(0x01), B(0x02), A(0x04), X(0x08), L(0x10), R(0x20), ZL(0x40), ZR(0x80)
      if ((btn0 & 0x01) && !(last_buttons_[0] & 0x01))
        ESP_LOGI("usb_hidx.gamepad", "Button Y pressed");
      if ((btn0 & 0x02) && !(last_buttons_[0] & 0x02))
        ESP_LOGI("usb_hidx.gamepad", "Button B pressed");
      if ((btn0 & 0x04) && !(last_buttons_[0] & 0x04))
        ESP_LOGI("usb_hidx.gamepad", "Button A pressed");
      if ((btn0 & 0x08) && !(last_buttons_[0] & 0x08))
        ESP_LOGI("usb_hidx.gamepad", "Button X pressed");
      if ((btn0 & 0x10) && !(last_buttons_[0] & 0x10))
        ESP_LOGI("usb_hidx.gamepad", "Button L pressed");
      if ((btn0 & 0x20) && !(last_buttons_[0] & 0x20))
        ESP_LOGI("usb_hidx.gamepad", "Button R pressed");
      if ((btn0 & 0x40) && !(last_buttons_[0] & 0x40))
        ESP_LOGI("usb_hidx.gamepad", "Button ZL pressed");
      if ((btn0 & 0x80) && !(last_buttons_[0] & 0x80))
        ESP_LOGI("usb_hidx.gamepad", "Button ZR pressed");
      last_buttons_[0] = btn0;
    }

    if (btn1 != last_buttons_[1]) {
      // Byte 1: Minus(0x01), Plus(0x02), L-Stick(0x04), R-Stick(0x08), Home(0x10), Capture(0x20)
      if ((btn1 & 0x01) && !(last_buttons_[1] & 0x01))
        ESP_LOGI("usb_hidx.gamepad", "Button Minus pressed");
      if ((btn1 & 0x02) && !(last_buttons_[1] & 0x02))
        ESP_LOGI("usb_hidx.gamepad", "Button Plus pressed");
      if ((btn1 & 0x04) && !(last_buttons_[1] & 0x04))
        ESP_LOGI("usb_hidx.gamepad", "Button L-Stick pressed");
      if ((btn1 & 0x08) && !(last_buttons_[1] & 0x08))
        ESP_LOGI("usb_hidx.gamepad", "Button R-Stick pressed");
      if ((btn1 & 0x10) && !(last_buttons_[1] & 0x10))
        ESP_LOGI("usb_hidx.gamepad", "Button Home pressed");
      if ((btn1 & 0x20) && !(last_buttons_[1] & 0x20))
        ESP_LOGI("usb_hidx.gamepad", "Button Capture pressed");
      last_buttons_[1] = btn1;
    }

    // D-Pad (byte 2)
    if (dpad != last_dpad_ && dpad != 0x0F) {
      const char *dir[] = {"Up", "Up-Right", "Right", "Down-Right", "Down", "Down-Left", "Left", "Up-Left"};
      if (dpad < 8)
        ESP_LOGI("usb_hidx.gamepad", "D-Pad: %s", dir[dpad]);
    }
    last_dpad_ = dpad;

    // Analog sticks (0x80 = center, deadzone = 30)
    if (abs((int) lx - (int) last_lx_) > 30 || abs((int) ly - (int) last_ly_) > 30) {
      ESP_LOGI("usb_hidx.gamepad", "Left Stick: X=%d Y=%d", lx, ly);
      last_lx_ = lx;
      last_ly_ = ly;
    }
    if (abs((int) rx - (int) last_rx_) > 30 || abs((int) ry - (int) last_ry_) > 30) {
      ESP_LOGI("usb_hidx.gamepad", "Right Stick: X=%d Y=%d", rx, ry);
      last_rx_ = rx;
      last_ry_ = ry;
    }
  }

  const char *get_name() override { return "GenericGamepad"; }

 protected:
  USBHIDXComponent *parent_;
  uint8_t last_buttons_[2]{0};
  uint8_t last_dpad_{0x0F};
  uint8_t last_lx_{0x80};
  uint8_t last_ly_{0x80};
  uint8_t last_rx_{0x80};
  uint8_t last_ry_{0x80};
};

}  // namespace usb_hidx
}  // namespace esphome
