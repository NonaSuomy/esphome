#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class Xbox360Driver : public HIDDeviceDriver {
 public:
  Xbox360Driver(USBHIDXComponent *parent) : parent_(parent) {}

  void set_button_a_sensor(binary_sensor::BinarySensor *sensor) { button_a_sensor_ = sensor; }
  void set_button_b_sensor(binary_sensor::BinarySensor *sensor) { button_b_sensor_ = sensor; }

  void set_device(HIDDevice *device) { device_ = device; }

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Xbox 360 controllers (gamepad, drum kit, guitar, wireless receiver)
    is_drum_kit_ = (vid == 0x1BAD && pid == 0x0003);
    is_guitar_ = (vid == 0x1430 && pid == 0x4748);
    is_gamepad_ = (vid == 0x045E && (pid == 0x028E || pid == 0x0719));
    return is_drum_kit_ || is_guitar_ || is_gamepad_;
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (len < 20)
      return;

    // Wireless receiver format: 29 bytes, starts with 0x00 0x01 or 0x00 0x00
    if (len == 29 && data[0] == 0x00 && (data[1] == 0x01 || data[1] == 0x00)) {
      is_wireless_ = true;
      if (!initialized_) {
        device_ = device;
        init_xbox360_controller(device);
        initialized_ = true;
      } else if (!device_) {
        device_ = device;
      }
      if (is_gamepad_)
        process_wireless_gamepad_report(data, len, device);
      return;
    }

    // Wired format: 20 bytes, starts with 0x00 0x14
    if (data[0] != 0x00 || data[1] != 0x14)
      return;

    // Store device pointer and initialize on first report
    if (!initialized_) {
      device_ = device;
      init_xbox360_controller(device);
      initialized_ = true;
    } else if (!device_) {
      device_ = device;
    }

    if (is_gamepad_) {
      process_gamepad_report(data, len, device);
    } else {
      process_instrument_report(data, len);
    }
  }

  const char *get_name() override { return "Xbox360"; }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  binary_sensor::BinarySensor *button_a_sensor_{nullptr};
  binary_sensor::BinarySensor *button_b_sensor_{nullptr};
  bool is_drum_kit_{false};
  bool is_guitar_{false};
  bool is_gamepad_{false};
  bool is_wireless_{false};
  bool initialized_{false};
  bool last_was_idle_{false};
  uint8_t last_dpad_{0};
  uint8_t last_buttons1_{0};
  uint8_t last_buttons2_{0};
  int16_t last_whammy_{-32768};
  int16_t last_lx_{0};
  int16_t last_ly_{0};
  int16_t last_rx_{0};
  int16_t last_ry_{0};
  uint8_t last_lt_{0};
  uint8_t last_rt_{0};

  void init_xbox360_controller(HIDDevice *device) {
    if (is_gamepad_) {
      ESP_LOGI("usb_hidx.xbox360", "Initializing Xbox 360 gamepad - setting LED to Player 1");
      send_led_command(device, 0x06);
    }
  }

  void send_led_command(HIDDevice *device, uint8_t pattern) {
    if (is_wireless_) {
      // Wireless: 0x42 = Player 1 solid, 0x43 = Player 2, 0x44 = Player 3, 0x45 = Player 4
      uint8_t led_cmd[] = {0x00, 0x00, 0x08, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      parent_->send_xbox360_interrupt_out(device, led_cmd, sizeof(led_cmd));
      ESP_LOGI("usb_hidx.xbox360", "Xbox 360 Wireless LED set to Player 1");
    } else {
      uint8_t led_cmd[] = {0x01, 0x03, pattern};
      parent_->send_xbox360_output(device, led_cmd, sizeof(led_cmd));
      ESP_LOGI("usb_hidx.xbox360", "Xbox 360 LED set to pattern 0x%02X", pattern);
    }
  }

 public:
  void send_rumble(HIDDevice *device, uint8_t left_motor, uint8_t right_motor) {
    if (!device && device_)
      device = device_;
    if (!device) {
      ESP_LOGW("usb_hidx.xbox360", "Cannot send rumble - device not available");
      return;
    }
    if (is_wireless_) {
      uint8_t rumble_cmd[] = {0x00, 0x01, 0x0F, 0xC0, 0x00, left_motor, right_motor, 0x00, 0x00, 0x00, 0x00, 0x00};
      parent_->send_xbox360_interrupt_out(device, rumble_cmd, sizeof(rumble_cmd));
    } else {
      uint8_t rumble_cmd[] = {0x00, 0x08, 0x00, left_motor, right_motor, 0x00, 0x00, 0x00};
      parent_->send_xbox360_output(device, rumble_cmd, sizeof(rumble_cmd));
    }
    ESP_LOGI("usb_hidx.xbox360", "Xbox 360 Rumble: Left=%d Right=%d", left_motor, right_motor);
  }

 protected:
  void process_gamepad_report(const uint8_t *data, size_t len, HIDDevice *device) {
    uint8_t dpad = data[2];
    uint8_t buttons1 = data[3];
    uint8_t buttons2 = data[4];
    uint8_t lt = data[5];
    uint8_t rt = data[6];

    // Analog sticks
    int16_t lx = (int16_t) (data[7] | (data[8] << 8));
    int16_t ly = (int16_t) (data[9] | (data[10] << 8));
    int16_t rx = (int16_t) (data[11] | (data[12] << 8));
    int16_t ry = (int16_t) (data[13] | (data[14] << 8));

    process_common_gamepad(dpad, buttons1, buttons2, lt, rt, lx, ly, rx, ry);
  }

  void process_wireless_gamepad_report(const uint8_t *data, size_t len, HIDDevice *device) {
    if (data[1] == 0x00)
      return;  // Skip idle reports

    uint8_t dpad = data[6] & 0x0F;      // Lower 4 bits for D-pad
    uint8_t buttons1 = data[7];         // A, B, X, Y, LB, RB
    uint8_t buttons2 = data[6] & 0xF0;  // Upper 4 bits for Start, Back, L-Stick, R-Stick
    uint8_t lt = data[8];
    uint8_t rt = data[9];

    // Analog sticks
    int16_t lx = (int16_t) (data[10] | (data[11] << 8));
    int16_t ly = (int16_t) (data[12] | (data[13] << 8));
    int16_t rx = (int16_t) (data[14] | (data[15] << 8));
    int16_t ry = (int16_t) (data[16] | (data[17] << 8));

    // Check Guide button separately (byte 7, bit 2)
    if ((data[7] & 0x04) && !(last_buttons2_ & 0x01)) {
      ESP_LOGI("usb_hidx.xbox360", "Button Guide pressed");
      last_buttons2_ |= 0x01;
    } else if (!(data[7] & 0x04) && (last_buttons2_ & 0x01)) {
      last_buttons2_ &= ~0x01;
    }

    process_common_gamepad(dpad, buttons1, buttons2, lt, rt, lx, ly, rx, ry);
  }

  void process_common_gamepad(uint8_t dpad, uint8_t buttons1, uint8_t buttons2, uint8_t lt, uint8_t rt, int16_t lx,
                              int16_t ly, int16_t rx, int16_t ry) {
    // Check if this is an idle report (no buttons, sticks near center, triggers released)
    bool is_idle = (dpad == 0 && buttons1 == 0 && buttons2 == 0 && lt < 30 && rt < 30 && abs(lx) < 8000 &&
                    abs(ly) < 8000 && abs(rx) < 8000 && abs(ry) < 8000);

    if (is_idle && last_was_idle_) {
      return;  // Skip logging idle reports
    }
    last_was_idle_ = is_idle;

    // D-pad (wireless uses different byte than wired)
    if (dpad != last_dpad_) {
      if ((dpad & 0x01) && !(last_dpad_ & 0x01))
        ESP_LOGI("usb_hidx.xbox360", "D-Pad Up pressed");
      if ((dpad & 0x02) && !(last_dpad_ & 0x02))
        ESP_LOGI("usb_hidx.xbox360", "D-Pad Down pressed");
      if ((dpad & 0x04) && !(last_dpad_ & 0x04))
        ESP_LOGI("usb_hidx.xbox360", "D-Pad Left pressed");
      if ((dpad & 0x08) && !(last_dpad_ & 0x08))
        ESP_LOGI("usb_hidx.xbox360", "D-Pad Right pressed");
      last_dpad_ = dpad;
    }

    // Face buttons and shoulders
    if (buttons1 != last_buttons1_) {
      if ((buttons1 & 0x10) && !(last_buttons1_ & 0x10)) {
        ESP_LOGI("usb_hidx.xbox360", "Button A pressed");
        if (button_a_sensor_)
          button_a_sensor_->publish_state(true);
      }
      if (!(buttons1 & 0x10) && (last_buttons1_ & 0x10)) {
        if (button_a_sensor_)
          button_a_sensor_->publish_state(false);
      }
      if ((buttons1 & 0x20) && !(last_buttons1_ & 0x20)) {
        ESP_LOGI("usb_hidx.xbox360", "Button B pressed");
        if (button_b_sensor_)
          button_b_sensor_->publish_state(true);
      }
      if (!(buttons1 & 0x20) && (last_buttons1_ & 0x20)) {
        if (button_b_sensor_)
          button_b_sensor_->publish_state(false);
      }
      if ((buttons1 & 0x40) && !(last_buttons1_ & 0x40))
        ESP_LOGI("usb_hidx.xbox360", "Button X pressed");
      if ((buttons1 & 0x80) && !(last_buttons1_ & 0x80))
        ESP_LOGI("usb_hidx.xbox360", "Button Y pressed");
      if ((buttons1 & 0x01) && !(last_buttons1_ & 0x01))
        ESP_LOGI("usb_hidx.xbox360", "Button LB pressed");
      if ((buttons1 & 0x02) && !(last_buttons1_ & 0x02))
        ESP_LOGI("usb_hidx.xbox360", "Button RB pressed");
      last_buttons1_ = buttons1;
    }

    if (buttons2 != last_buttons2_) {
      if ((buttons2 & 0x10) && !(last_buttons2_ & 0x10))
        ESP_LOGI("usb_hidx.xbox360", "Button Start pressed");
      if ((buttons2 & 0x20) && !(last_buttons2_ & 0x20))
        ESP_LOGI("usb_hidx.xbox360", "Button Back pressed");
      if ((buttons2 & 0x40) && !(last_buttons2_ & 0x40))
        ESP_LOGI("usb_hidx.xbox360", "Button L-Stick pressed");
      if ((buttons2 & 0x80) && !(last_buttons2_ & 0x80))
        ESP_LOGI("usb_hidx.xbox360", "Button R-Stick pressed");
      last_buttons2_ = buttons2;
    }

    // Analog sticks - apply deadzone and only log significant movements from center
    const int16_t DEADZONE = 8000;
    if ((abs(lx) > DEADZONE || abs(ly) > DEADZONE) && (abs(lx - last_lx_) > 5000 || abs(ly - last_ly_) > 5000)) {
      ESP_LOGI("usb_hidx.xbox360", "Left Stick: X=%d Y=%d", lx, ly);
      last_lx_ = lx;
      last_ly_ = ly;
    }
    if ((abs(rx) > DEADZONE || abs(ry) > DEADZONE) && (abs(rx - last_rx_) > 5000 || abs(ry - last_ry_) > 5000)) {
      ESP_LOGI("usb_hidx.xbox360", "Right Stick: X=%d Y=%d", rx, ry);
      last_rx_ = rx;
      last_ry_ = ry;
    }

    // Triggers - only log significant changes (>50)
    if (abs((int) lt - (int) last_lt_) > 50) {
      ESP_LOGI("usb_hidx.xbox360", "Left Trigger: %d", lt);
      last_lt_ = lt;
    }
    if (abs((int) rt - (int) last_rt_) > 50) {
      ESP_LOGI("usb_hidx.xbox360", "Right Trigger: %d", rt);
      last_rt_ = rt;
    }
  }

  void process_instrument_report(const uint8_t *data, size_t len) {
    uint8_t dpad = data[2];
    uint8_t buttons1 = data[3];
    uint8_t buttons2 = data[4];

    const char *prefix = is_guitar_ ? "Guitar" : "Drum";

    // D-pad (Strum for guitar/drums)
    if (dpad != last_dpad_) {
      if ((dpad & 0x01) && !(last_dpad_ & 0x01))
        ESP_LOGI("usb_hidx.xbox360", "%s: Strum Up", prefix);
      if ((dpad & 0x02) && !(last_dpad_ & 0x02))
        ESP_LOGI("usb_hidx.xbox360", "%s: Strum Down", prefix);
      if ((dpad & 0x04) && !(last_dpad_ & 0x04))
        ESP_LOGI("usb_hidx.xbox360", "%s: D-Pad Left", prefix);
      if ((dpad & 0x08) && !(last_dpad_ & 0x08))
        ESP_LOGI("usb_hidx.xbox360", "%s: D-Pad Right", prefix);
      if ((dpad & 0x10) && !(last_dpad_ & 0x10))
        ESP_LOGI("usb_hidx.xbox360", "%s: Start", prefix);
      if ((dpad & 0x20) && !(last_dpad_ & 0x20))
        ESP_LOGI("usb_hidx.xbox360", "%s: Back", prefix);
      last_dpad_ = dpad;
    }

    // Face buttons (Drum pads/Guitar frets)
    if (buttons1 != last_buttons1_) {
      if ((buttons1 & 0x10) && !(last_buttons1_ & 0x10))
        ESP_LOGI("usb_hidx.xbox360", "%s: A (Green)", prefix);
      if ((buttons1 & 0x20) && !(last_buttons1_ & 0x20))
        ESP_LOGI("usb_hidx.xbox360", "%s: B (Red)", prefix);
      if ((buttons1 & 0x40) && !(last_buttons1_ & 0x40))
        ESP_LOGI("usb_hidx.xbox360", "%s: X (Blue)", prefix);
      if ((buttons1 & 0x80) && !(last_buttons1_ & 0x80))
        ESP_LOGI("usb_hidx.xbox360", "%s: Y (Yellow)", prefix);
      if ((buttons1 & 0x01) && !(last_buttons1_ & 0x01)) {
        ESP_LOGI("usb_hidx.xbox360", "%s: %s", prefix, is_drum_kit_ ? "Kick Pedal" : "Orange");
      }
      if ((buttons1 & 0x02) && !(last_buttons1_ & 0x02))
        ESP_LOGI("usb_hidx.xbox360", "%s: RB", prefix);
      last_buttons1_ = buttons1;
    }

    // Guide button
    if ((buttons2 & 0x04) && !(last_buttons2_ & 0x04)) {
      ESP_LOGI("usb_hidx.xbox360", "%s: Guide Button", prefix);
    }
    last_buttons2_ = buttons2;

    // Whammy bar for guitar (RX axis)
    if (is_guitar_) {
      int16_t rx = (int16_t) (data[10] | (data[11] << 8));
      if (abs(rx - last_whammy_) > 1000) {
        ESP_LOGI("usb_hidx.xbox360", "Guitar: Whammy Bar = %d", rx);
        last_whammy_ = rx;
      }
    }
  }
};

}  // namespace usb_hidx
}  // namespace esphome
