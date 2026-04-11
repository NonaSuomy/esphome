#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class InteractDriver : public HIDDeviceDriver {
 public:
  InteractDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    return (vid == 0x05FD && pid == 0x0251);
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (len < 6)
      return;

    // Report format: [ReportID, Buttons, X, Y, D-pad/Buttons, Extra]
    uint8_t buttons = data[1];
    uint8_t x = data[2];  // Analog X: 0x00=left, 0x7F=center, 0xFF=right
    uint8_t y = data[3];  // Analog Y: 0x00=up, 0x7F=center, 0xFF=down
    uint8_t hat_byte = data[4];  // 8-way hat switch + buttons
    uint8_t extra_byte = data[5];

    // Analog stick to D-pad with deadzone
    bool up = (y < 0x40);
    bool down = (y > 0xC0);
    bool left = (x < 0x40);
    bool right = (x > 0xC0);

    // Check if analog position changed significantly
    bool analog_changed = (abs((int)x - (int)last_x_) > 20 || abs((int)y - (int)last_y_) > 20);
    
    if (analog_changed || hat_byte != last_hat_byte_ || extra_byte != last_extra_byte_) {
      if (analog_changed)
        ESP_LOGI("usb_hidx.interact", "Stick X:%d Y:%d", x, y);
    }

    // Publish analog stick as D-pad
    if (parent_->gamepad_dpad_up_sensor_)
      parent_->gamepad_dpad_up_sensor_->publish_state(up);
    if (parent_->gamepad_dpad_down_sensor_)
      parent_->gamepad_dpad_down_sensor_->publish_state(down);
    if (parent_->gamepad_dpad_left_sensor_)
      parent_->gamepad_dpad_left_sensor_->publish_state(left);
    if (parent_->gamepad_dpad_right_sensor_)
      parent_->gamepad_dpad_right_sensor_->publish_state(right);

    // 8-way hat switch (byte 4, lower bits)
    uint8_t hat = hat_byte & 0x0F;
    if (hat != last_hat_ && hat < 8) {
      const char *dir[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
      ESP_LOGI("usb_hidx.interact", "Hat: %s", dir[hat]);
      last_hat_ = hat;
    }

    // Buttons in byte 4 (upper bits)
    if (hat_byte != last_hat_byte_) {
      if ((hat_byte & 0x10) && !(last_hat_byte_ & 0x10)) {
        ESP_LOGI("usb_hidx.interact", "Trigger");
        if (parent_->gamepad_button_a_sensor_)
          parent_->gamepad_button_a_sensor_->publish_state(true);
      }
      if (!(hat_byte & 0x10) && (last_hat_byte_ & 0x10)) {
        if (parent_->gamepad_button_a_sensor_)
          parent_->gamepad_button_a_sensor_->publish_state(false);
      }
      
      if ((hat_byte & 0x20) && !(last_hat_byte_ & 0x20))
        ESP_LOGI("usb_hidx.interact", "Button 1");
      if ((hat_byte & 0x40) && !(last_hat_byte_ & 0x40))
        ESP_LOGI("usb_hidx.interact", "Button 2");
      if ((hat_byte & 0x80) && !(last_hat_byte_ & 0x80))
        ESP_LOGI("usb_hidx.interact", "Button 3");
      
      last_hat_byte_ = hat_byte;
    }

    // Extra buttons in byte 5
    if (extra_byte != last_extra_byte_) {
      if ((extra_byte & 0x01) && !(last_extra_byte_ & 0x01))
        ESP_LOGI("usb_hidx.interact", "Button 4");
      if ((extra_byte & 0x02) && !(last_extra_byte_ & 0x02))
        ESP_LOGI("usb_hidx.interact", "Button 5");
      last_extra_byte_ = extra_byte;
    }
    
    last_x_ = x;
    last_y_ = y;
    last_buttons_ = buttons;
  }

  const char *get_name() override { return "Interact"; }

 protected:
  USBHIDXComponent *parent_;
  uint8_t last_hat_{0xFF};
  uint8_t last_buttons_{0};
  uint8_t last_hat_byte_{0};
  uint8_t last_extra_byte_{0};
  uint8_t last_x_{0x7F};
  uint8_t last_y_{0x7F};
};

}  // namespace usb_hidx
}  // namespace esphome
