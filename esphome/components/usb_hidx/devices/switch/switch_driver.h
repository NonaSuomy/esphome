#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class SwitchDriver : public HIDDeviceDriver {
 public:
  SwitchDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    is_official_ = (vid == 0x057E && pid == 0x2009);
    is_powera_ = (vid == 0x20D6 && pid == 0xA713);
    return is_official_ || is_powera_;
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (len < 8)
      return;

    if (is_powera_) {
      // PowerA format: 8 bytes
      uint8_t btn0 = data[0];
      uint8_t btn1 = data[1];
      uint8_t dpad = data[2];
      uint8_t lx = data[3];
      uint8_t ly = data[4];
      uint8_t rx = data[5];
      uint8_t ry = data[6];

      // D-Pad
      if (dpad != last_dpad_ && dpad != 0x0F) {
        const char *dir[] = {"Up", "Up-Right", "Right", "Down-Right", "Down", "Down-Left", "Left", "Up-Left"};
        if (dpad < 8)
          ESP_LOGI("usb_hidx.switch", "D-Pad: %s", dir[dpad]);
        last_dpad_ = dpad;
      } else if (dpad == 0x0F && last_dpad_ != 0x0F) {
        last_dpad_ = 0x0F;
      }

      // Buttons byte 0: Y(0x01), B(0x02), A(0x04), X(0x08), L(0x10), R(0x20), ZL(0x40), ZR(0x80)
      if (btn0 != last_btn0_) {
        if ((btn0 & 0x01) && !(last_btn0_ & 0x01))
          ESP_LOGI("usb_hidx.switch", "Y Button");
        if ((btn0 & 0x02) && !(last_btn0_ & 0x02))
          ESP_LOGI("usb_hidx.switch", "B Button");
        if ((btn0 & 0x04) && !(last_btn0_ & 0x04))
          ESP_LOGI("usb_hidx.switch", "A Button");
        if ((btn0 & 0x08) && !(last_btn0_ & 0x08))
          ESP_LOGI("usb_hidx.switch", "X Button");
        if ((btn0 & 0x10) && !(last_btn0_ & 0x10))
          ESP_LOGI("usb_hidx.switch", "L Button");
        if ((btn0 & 0x20) && !(last_btn0_ & 0x20))
          ESP_LOGI("usb_hidx.switch", "R Button");
        if ((btn0 & 0x40) && !(last_btn0_ & 0x40))
          ESP_LOGI("usb_hidx.switch", "ZL Button");
        if ((btn0 & 0x80) && !(last_btn0_ & 0x80))
          ESP_LOGI("usb_hidx.switch", "ZR Button");
        last_btn0_ = btn0;
      }

      // Buttons byte 1: Minus(0x01), Plus(0x02), L-Stick(0x04), R-Stick(0x08), Home(0x10), Capture(0x20)
      if (btn1 != last_btn1_) {
        if ((btn1 & 0x01) && !(last_btn1_ & 0x01))
          ESP_LOGI("usb_hidx.switch", "Minus");
        if ((btn1 & 0x02) && !(last_btn1_ & 0x02))
          ESP_LOGI("usb_hidx.switch", "Plus");
        if ((btn1 & 0x04) && !(last_btn1_ & 0x04))
          ESP_LOGI("usb_hidx.switch", "L-Stick");
        if ((btn1 & 0x08) && !(last_btn1_ & 0x08))
          ESP_LOGI("usb_hidx.switch", "R-Stick");
        if ((btn1 & 0x10) && !(last_btn1_ & 0x10))
          ESP_LOGI("usb_hidx.switch", "Home");
        if ((btn1 & 0x20) && !(last_btn1_ & 0x20))
          ESP_LOGI("usb_hidx.switch", "Capture");
        last_btn1_ = btn1;
      }

      // Analog sticks with deadzone
      if (abs((int) lx - (int) last_lx_) > 30 || abs((int) ly - (int) last_ly_) > 30) {
        ESP_LOGI("usb_hidx.switch", "Left Stick: X=%d Y=%d", lx, ly);
        last_lx_ = lx;
        last_ly_ = ly;
      }
      if (abs((int) rx - (int) last_rx_) > 30 || abs((int) ry - (int) last_ry_) > 30) {
        ESP_LOGI("usb_hidx.switch", "Right Stick: X=%d Y=%d", rx, ry);
        last_rx_ = rx;
        last_ry_ = ry;
      }
    } else if (is_official_ && len >= 64) {
      // Official controller: 64 bytes with report ID 0x30 or 0x21
      if (data[0] != 0x30 && data[0] != 0x21)
        return;

      int offset = 3;
      uint8_t btn_right = data[offset];
      uint8_t btn_shared = data[offset + 1];
      uint8_t btn_left = data[offset + 2];

      // Right buttons (Y,X,B,A,R,ZR)
      if (btn_right != last_btn0_) {
        if ((btn_right & 0x01) && !(last_btn0_ & 0x01))
          ESP_LOGI("usb_hidx.switch", "Y Button");
        if ((btn_right & 0x02) && !(last_btn0_ & 0x02))
          ESP_LOGI("usb_hidx.switch", "X Button");
        if ((btn_right & 0x04) && !(last_btn0_ & 0x04))
          ESP_LOGI("usb_hidx.switch", "B Button");
        if ((btn_right & 0x08) && !(last_btn0_ & 0x08))
          ESP_LOGI("usb_hidx.switch", "A Button");
        if ((btn_right & 0x40) && !(last_btn0_ & 0x40))
          ESP_LOGI("usb_hidx.switch", "R Button");
        if ((btn_right & 0x80) && !(last_btn0_ & 0x80))
          ESP_LOGI("usb_hidx.switch", "ZR Button");
        last_btn0_ = btn_right;
      }

      // Shared buttons (Minus, Plus, Home, Capture)
      if (btn_shared != last_btn1_) {
        if ((btn_shared & 0x01) && !(last_btn1_ & 0x01))
          ESP_LOGI("usb_hidx.switch", "Minus");
        if ((btn_shared & 0x02) && !(last_btn1_ & 0x02))
          ESP_LOGI("usb_hidx.switch", "Plus");
        if ((btn_shared & 0x10) && !(last_btn1_ & 0x10)) {
          ESP_LOGI("usb_hidx.switch", "Home - Rumble ON");
          // TODO: Trigger rumble via control transfer
        }
        if (!(btn_shared & 0x10) && (last_btn1_ & 0x10)) {
          ESP_LOGI("usb_hidx.switch", "Home Released - Rumble OFF");
        }
        if ((btn_shared & 0x20) && !(last_btn1_ & 0x20))
          ESP_LOGI("usb_hidx.switch", "Capture");
        last_btn1_ = btn_shared;
      }

      // Left buttons (L, ZL)
      if ((btn_left & 0x40) && !(last_btn_left_ & 0x40))
        ESP_LOGI("usb_hidx.switch", "L Button");
      if ((btn_left & 0x80) && !(last_btn_left_ & 0x80))
        ESP_LOGI("usb_hidx.switch", "ZL Button");
      last_btn_left_ = btn_left;

      // Analog sticks (12-bit values)
      uint16_t lx = (data[6] | ((data[7] & 0x0F) << 8));
      uint16_t ly = ((data[7] >> 4) | (data[8] << 4));
      uint16_t rx = (data[9] | ((data[10] & 0x0F) << 8));
      uint16_t ry = ((data[10] >> 4) | (data[11] << 4));

      if (abs((int) lx - (int) last_lx_) > 300 || abs((int) ly - (int) last_ly_) > 300) {
        ESP_LOGI("usb_hidx.switch", "Left Stick: X=%d Y=%d", lx, ly);
        last_lx_ = lx;
        last_ly_ = ly;
      }
      if (abs((int) rx - (int) last_rx_) > 300 || abs((int) ry - (int) last_ry_) > 300) {
        ESP_LOGI("usb_hidx.switch", "Right Stick: X=%d Y=%d", rx, ry);
        last_rx_ = rx;
        last_ry_ = ry;
      }
    }
  }

  const char *get_name() override { return "Switch"; }

 protected:
  USBHIDXComponent *parent_;
  bool is_official_{false};
  bool is_powera_{false};
  uint8_t last_dpad_{0x0F};
  uint8_t last_btn0_{0};
  uint8_t last_btn1_{0};
  uint8_t last_btn_left_{0};
  uint16_t last_lx_{128};
  uint16_t last_ly_{128};
  uint16_t last_rx_{128};
  uint16_t last_ry_{128};
};

}  // namespace usb_hidx
}  // namespace esphome
