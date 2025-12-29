#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class XboxOneDriver : public HIDDeviceDriver {
 public:
  XboxOneDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Microsoft Xbox One controllers
    if (vid == 0x045E && (pid == 0x02D1 || pid == 0x02DD || pid == 0x02E3 || pid == 0x02EA || pid == 0x0B00 ||
                          pid == 0x0B05 || pid == 0x0B12 || pid == 0x0B13)) {
      return true;
    }
    // Xbox Series X|S controllers
    if (vid == 0x045E && (pid == 0x0B13 || pid == 0x0B20)) {
      is_series_xs_ = true;
      return true;
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.xbone", is_series_xs_ ? "Xbox Series X|S controller detected" : "Xbox One controller detected");
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 18)
      return;

    // Xbox One report format
    // Buttons at offset 4-5
    uint16_t buttons = (data[5] << 8) | data[4];

    // Face buttons
    bool a = buttons & 0x1000;
    bool b = buttons & 0x2000;
    bool x = buttons & 0x4000;
    bool y = buttons & 0x8000;

    // D-Pad
    bool dpad_up = buttons & 0x0001;
    bool dpad_down = buttons & 0x0002;
    bool dpad_left = buttons & 0x0004;
    bool dpad_right = buttons & 0x0008;

    // Shoulder buttons
    bool lb = buttons & 0x0100;
    bool rb = buttons & 0x0200;

    // Special buttons
    bool view = buttons & 0x0020;
    bool menu = buttons & 0x0040;
    bool xbox = buttons & 0x0400;

    // Stick buttons
    bool ls = buttons & 0x0010;
    bool rs = buttons & 0x0080;

    // Triggers (0-255)
    uint8_t lt = data[6];
    uint8_t rt = data[7];

    // Analog sticks (16-bit signed)
    int16_t lx = (data[9] << 8) | data[8];
    int16_t ly = (data[11] << 8) | data[10];
    int16_t rx = (data[13] << 8) | data[12];
    int16_t ry = (data[15] << 8) | data[14];

    // Log button presses
    if (a && !(last_buttons_ & 0x1000))
      ESP_LOGI("usb_hidx.xbone", "A Button");
    if (b && !(last_buttons_ & 0x2000))
      ESP_LOGI("usb_hidx.xbone", "B Button");
    if (x && !(last_buttons_ & 0x4000))
      ESP_LOGI("usb_hidx.xbone", "X Button");
    if (y && !(last_buttons_ & 0x8000))
      ESP_LOGI("usb_hidx.xbone", "Y Button");
    if (xbox && !(last_buttons_ & 0x0400))
      ESP_LOGI("usb_hidx.xbone", "Xbox Button");

    last_buttons_ = buttons;
  }

  const char *get_name() override { return "XboxOne"; }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  bool is_series_xs_{false};
  uint16_t last_buttons_{0};
};

}  // namespace usb_hidx
}  // namespace esphome
