#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class StadiaDriver : public HIDDeviceDriver {
 public:
  StadiaDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Google Stadia Controller
    if (vid == 0x18D1 && pid == 0x9400) {
      return true;
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.stadia", "Google Stadia Controller detected");
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 10)
      return;

    // Stadia controller HID report format
    // Buttons at offset 5-7
    uint8_t btn1 = data[5];
    uint8_t btn2 = data[6];
    uint8_t btn3 = data[7];

    // Face buttons (btn1)
    bool a = btn1 & 0x01;
    bool b = btn1 & 0x02;
    bool x = btn1 & 0x08;
    bool y = btn1 & 0x10;

    // Shoulder buttons (btn1)
    bool l1 = btn1 & 0x40;
    bool r1 = btn1 & 0x80;

    // Triggers (btn2)
    bool l2 = btn2 & 0x01;
    bool r2 = btn2 & 0x02;

    // Special buttons (btn2)
    bool options = btn2 & 0x08;
    bool menu = btn2 & 0x10;
    bool stadia = btn2 & 0x20;

    // Stick buttons (btn2)
    bool l3 = btn2 & 0x40;
    bool r3 = btn2 & 0x80;

    // D-Pad (btn3, lower 4 bits)
    uint8_t dpad = btn3 & 0x0F;

    // Assistant button (btn3)
    bool assistant = btn3 & 0x10;

    // Capture button (btn3)
    bool capture = btn3 & 0x20;

    // Analog sticks (8-bit each at offset 1-4)
    uint8_t lx = data[1];
    uint8_t ly = data[2];
    uint8_t rx = data[3];
    uint8_t ry = data[4];

    // Analog triggers (8-bit each at offset 8-9)
    uint8_t lt = data[8];
    uint8_t rt = data[9];

    // Log button presses
    if (a && !(last_btn1_ & 0x01))
      ESP_LOGI("usb_hidx.stadia", "A Button");
    if (b && !(last_btn1_ & 0x02))
      ESP_LOGI("usb_hidx.stadia", "B Button");
    if (x && !(last_btn1_ & 0x08))
      ESP_LOGI("usb_hidx.stadia", "X Button");
    if (y && !(last_btn1_ & 0x10))
      ESP_LOGI("usb_hidx.stadia", "Y Button");
    if (l1 && !(last_btn1_ & 0x40))
      ESP_LOGI("usb_hidx.stadia", "L1");
    if (r1 && !(last_btn1_ & 0x80))
      ESP_LOGI("usb_hidx.stadia", "R1");
    if (l2 && !(last_btn2_ & 0x01))
      ESP_LOGI("usb_hidx.stadia", "L2");
    if (r2 && !(last_btn2_ & 0x02))
      ESP_LOGI("usb_hidx.stadia", "R2");
    if (stadia && !(last_btn2_ & 0x20))
      ESP_LOGI("usb_hidx.stadia", "Stadia Button");
    if (assistant && !(last_btn3_ & 0x10))
      ESP_LOGI("usb_hidx.stadia", "Assistant Button");
    if (capture && !(last_btn3_ & 0x20))
      ESP_LOGI("usb_hidx.stadia", "Capture Button");
    if (options && !(last_btn2_ & 0x08))
      ESP_LOGI("usb_hidx.stadia", "Options");
    if (menu && !(last_btn2_ & 0x10))
      ESP_LOGI("usb_hidx.stadia", "Menu");

    // D-Pad
    if (dpad != last_dpad_ && dpad != 0x0F) {
      const char *dir[] = {"Up", "Up-Right", "Right", "Down-Right", "Down", "Down-Left", "Left", "Up-Left"};
      if (dpad < 8)
        ESP_LOGI("usb_hidx.stadia", "D-Pad: %s", dir[dpad]);
      last_dpad_ = dpad;
    } else if (dpad == 0x0F && last_dpad_ != 0x0F) {
      last_dpad_ = 0x0F;
    }

    // Log stick movement
    if (abs((int) lx - (int) last_lx_) > 20 || abs((int) ly - (int) last_ly_) > 20) {
      ESP_LOGV("usb_hidx.stadia", "Left Stick: X=%d Y=%d", lx, ly);
      last_lx_ = lx;
      last_ly_ = ly;
    }
    if (abs((int) rx - (int) last_rx_) > 20 || abs((int) ry - (int) last_ry_) > 20) {
      ESP_LOGV("usb_hidx.stadia", "Right Stick: X=%d Y=%d", rx, ry);
      last_rx_ = rx;
      last_ry_ = ry;
    }

    last_btn1_ = btn1;
    last_btn2_ = btn2;
    last_btn3_ = btn3;
  }

  const char *get_name() override { return "Stadia"; }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  uint8_t last_btn1_{0};
  uint8_t last_btn2_{0};
  uint8_t last_btn3_{0};
  uint8_t last_dpad_{0x0F};
  uint8_t last_lx_{128};
  uint8_t last_ly_{128};
  uint8_t last_rx_{128};
  uint8_t last_ry_{128};
};

}  // namespace usb_hidx
}  // namespace esphome
