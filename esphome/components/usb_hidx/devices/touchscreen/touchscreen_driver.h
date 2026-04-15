#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class TouchscreenDriver : public HIDDeviceDriver {
 public:
  TouchscreenDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Generic multitouch devices (usage page 0x0D)
    // Accept any device that reports as digitizer/touchscreen
    // Common manufacturers:

    // 3M touchscreens
    if (vid == 0x0596)
      return true;

    // eGalax/EETI touchscreens
    if (vid == 0x0EEF)
      return true;

    // Elo TouchSystems
    if (vid == 0x04E7)
      return true;

    // ILITEK touchscreens
    if (vid == 0x222A)
      return true;

    // Atmel maXTouch
    if (vid == 0x03EB && pid == 0x211C)
      return true;

    // Synaptics touchscreens
    if (vid == 0x06CB)
      return true;

    // Goodix touchscreens
    if (vid == 0x27C6)
      return true;

    // Generic HID touchscreen (will match based on usage page in descriptor)
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.touch", "USB Touchscreen detected");
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 5)
      return;

    uint8_t report_id = data[0];

    // Single touch report (most common)
    if (len >= 7) {
      bool touching = data[1] & 0x01;
      uint16_t x = (data[3] << 8) | data[2];
      uint16_t y = (data[5] << 8) | data[4];

      if (touching) {
        // Publish delta movement relative to last position
        if (last_touching_) {
          int16_t dx = (int16_t)x - (int16_t)last_x_;
          int16_t dy = (int16_t)y - (int16_t)last_y_;
          if (dx != 0 && parent_->get_mouse_x_sensor())
            parent_->get_mouse_x_sensor()->publish_state(dx);
          if (dy != 0 && parent_->get_mouse_y_sensor())
            parent_->get_mouse_y_sensor()->publish_state(dy);
        }
        // Publish left button held while touching
        if (parent_->get_mouse_left_sensor())
          parent_->get_mouse_left_sensor()->publish_state(true);
        ESP_LOGV("usb_hidx.touch", "Touch: X=%d Y=%d", x, y);
      } else if (last_touching_) {
        if (parent_->get_mouse_left_sensor())
          parent_->get_mouse_left_sensor()->publish_state(false);
        ESP_LOGI("usb_hidx.touch", "Touch Up");
      }

      last_touching_ = touching;
      if (touching) {
        last_x_ = x;
        last_y_ = y;
      }
    }
  }

  const char *get_name() override { return "Touchscreen"; }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  bool last_touching_{false};
  uint16_t last_x_{0};
  uint16_t last_y_{0};
};

}  // namespace usb_hidx
}  // namespace esphome
