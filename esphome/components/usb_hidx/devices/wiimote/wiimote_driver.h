#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class WiimoteDriver : public HIDDeviceDriver {
 public:
  WiimoteDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Nintendo Wii Remote
    if (vid == 0x057E && pid == 0x0306) {
      return true;
    }
    // Wii Remote Plus
    if (vid == 0x057E && pid == 0x0330) {
      is_plus_ = true;
      return true;
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.wiimote", is_plus_ ? "Wii Remote Plus detected" : "Wii Remote detected");
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 6)
      return;

    // Wii Remote report format
    uint8_t report_id = data[0];

    if (report_id == 0x30 || report_id == 0x31) {
      // Core buttons report
      uint16_t buttons = (data[1] << 8) | data[2];

      // D-Pad
      bool dpad_left = buttons & 0x0001;
      bool dpad_right = buttons & 0x0002;
      bool dpad_down = buttons & 0x0004;
      bool dpad_up = buttons & 0x0008;

      // Face buttons
      bool plus = buttons & 0x0010;
      bool two = buttons & 0x0100;
      bool one = buttons & 0x0200;
      bool b = buttons & 0x0400;
      bool a = buttons & 0x0800;
      bool minus = buttons & 0x1000;
      bool home = buttons & 0x8000;

      // Log button presses
      if (a && !(last_buttons_ & 0x0800))
        ESP_LOGI("usb_hidx.wiimote", "A Button");
      if (b && !(last_buttons_ & 0x0400))
        ESP_LOGI("usb_hidx.wiimote", "B Button");
      if (one && !(last_buttons_ & 0x0200))
        ESP_LOGI("usb_hidx.wiimote", "1 Button");
      if (two && !(last_buttons_ & 0x0100))
        ESP_LOGI("usb_hidx.wiimote", "2 Button");
      if (plus && !(last_buttons_ & 0x0010))
        ESP_LOGI("usb_hidx.wiimote", "Plus");
      if (minus && !(last_buttons_ & 0x1000))
        ESP_LOGI("usb_hidx.wiimote", "Minus");
      if (home && !(last_buttons_ & 0x8000))
        ESP_LOGI("usb_hidx.wiimote", "Home");

      last_buttons_ = buttons;
    }

    if (report_id == 0x31 && len >= 9) {
      // Accelerometer data (3 bytes at offset 3-5)
      uint8_t accel_x = data[3];
      uint8_t accel_y = data[4];
      uint8_t accel_z = data[5];

      // Log significant motion
      if (abs((int) accel_x - (int) last_accel_x_) > 20 || abs((int) accel_y - (int) last_accel_y_) > 20 ||
          abs((int) accel_z - (int) last_accel_z_) > 20) {
        ESP_LOGV("usb_hidx.wiimote", "Motion: X=%d Y=%d Z=%d", accel_x, accel_y, accel_z);
        last_accel_x_ = accel_x;
        last_accel_y_ = accel_y;
        last_accel_z_ = accel_z;
      }
    }
  }

  const char *get_name() override { return "Wiimote"; }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  bool is_plus_{false};
  uint16_t last_buttons_{0};
  uint8_t last_accel_x_{128};
  uint8_t last_accel_y_{128};
  uint8_t last_accel_z_{128};
};

}  // namespace usb_hidx
}  // namespace esphome
