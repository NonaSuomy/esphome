#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class SteamDriver : public HIDDeviceDriver {
 public:
  SteamDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Valve Steam Controller
    if (vid == 0x28DE && pid == 0x1142) {
      return true;
    }
    // Steam Controller (wired)
    if (vid == 0x28DE && pid == 0x1102) {
      return true;
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.steam", "Steam Controller detected");
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 64)
      return;

    // Steam Controller report format
    // Report type at offset 2
    uint8_t report_type = data[2];

    if (report_type == 0x01) {  // Input report
      // Buttons at offset 8-10
      uint8_t btn1 = data[8];
      uint8_t btn2 = data[9];
      uint8_t btn3 = data[10];

      // Face buttons (btn2)
      bool a = btn2 & 0x80;
      bool b = btn2 & 0x20;
      bool x = btn2 & 0x40;
      bool y = btn2 & 0x10;

      // Shoulder buttons (btn1)
      bool lb = btn1 & 0x10;
      bool rb = btn1 & 0x20;
      bool lt = btn1 & 0x40;
      bool rt = btn1 & 0x80;

      // Special buttons (btn3)
      bool steam = btn3 & 0x01;
      bool back = btn2 & 0x04;
      bool forward = btn2 & 0x08;

      // Stick button (btn1)
      bool stick = btn1 & 0x04;

      // Left pad click (btn1)
      bool lpad = btn1 & 0x08;

      // Right pad click (btn1)
      bool rpad = btn1 & 0x02;

      // Grip buttons (btn2)
      bool lgrip = btn2 & 0x01;
      bool rgrip = btn2 & 0x02;

      // Analog stick (16-bit signed at offset 16-19)
      int16_t stick_x = (data[17] << 8) | data[16];
      int16_t stick_y = (data[19] << 8) | data[18];

      // Left touchpad (16-bit signed at offset 20-23)
      int16_t lpad_x = (data[21] << 8) | data[20];
      int16_t lpad_y = (data[23] << 8) | data[22];

      // Right touchpad (16-bit signed at offset 24-27)
      int16_t rpad_x = (data[25] << 8) | data[24];
      int16_t rpad_y = (data[27] << 8) | data[26];

      // Log button presses
      if (a && !(last_btn2_ & 0x80))
        ESP_LOGI("usb_hidx.steam", "A Button");
      if (b && !(last_btn2_ & 0x20))
        ESP_LOGI("usb_hidx.steam", "B Button");
      if (x && !(last_btn2_ & 0x40))
        ESP_LOGI("usb_hidx.steam", "X Button");
      if (y && !(last_btn2_ & 0x10))
        ESP_LOGI("usb_hidx.steam", "Y Button");
      if (steam && !(last_btn3_ & 0x01))
        ESP_LOGI("usb_hidx.steam", "Steam Button");
      if (lpad && !(last_btn1_ & 0x08))
        ESP_LOGI("usb_hidx.steam", "Left Pad Click");
      if (rpad && !(last_btn1_ & 0x02))
        ESP_LOGI("usb_hidx.steam", "Right Pad Click");
      if (lgrip && !(last_btn2_ & 0x01))
        ESP_LOGI("usb_hidx.steam", "Left Grip");
      if (rgrip && !(last_btn2_ & 0x02))
        ESP_LOGI("usb_hidx.steam", "Right Grip");

      last_btn1_ = btn1;
      last_btn2_ = btn2;
      last_btn3_ = btn3;
    }
  }

  const char *get_name() override { return "Steam"; }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  uint8_t last_btn1_{0};
  uint8_t last_btn2_{0};
  uint8_t last_btn3_{0};
};

}  // namespace usb_hidx
}  // namespace esphome
