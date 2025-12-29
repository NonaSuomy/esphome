#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class MouseDriver : public HIDDeviceDriver {
 public:
  MouseDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    return protocol == 0x02;  // HID mouse protocol
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (len < 3)
      return;

    uint8_t buttons = data[0];
    int8_t x = (int8_t) data[1];
    int8_t y = (int8_t) data[2];
    int8_t wheel = (len >= 4) ? (int8_t) data[3] : 0;

    // Button 1 (Left)
    if (parent_->get_mouse_left_sensor()) {
      parent_->get_mouse_left_sensor()->publish_state(buttons & 0x01);
    }
    // Button 2 (Right)
    if (parent_->get_mouse_right_sensor()) {
      parent_->get_mouse_right_sensor()->publish_state(buttons & 0x02);
    }
    // Button 3 (Middle)
    if (parent_->get_mouse_middle_sensor()) {
      parent_->get_mouse_middle_sensor()->publish_state(buttons & 0x04);
    }

    // Movement
    if (x != 0 && parent_->get_mouse_x_sensor()) {
      parent_->get_mouse_x_sensor()->publish_state(x);
    }
    if (y != 0 && parent_->get_mouse_y_sensor()) {
      parent_->get_mouse_y_sensor()->publish_state(y);
    }
    if (wheel != 0 && parent_->get_mouse_wheel_sensor()) {
      parent_->get_mouse_wheel_sensor()->publish_state(wheel);
    }
  }

  const char *get_name() override { return "Mouse"; }

 protected:
  USBHIDXComponent *parent_;
};

}  // namespace usb_hidx
}  // namespace esphome
