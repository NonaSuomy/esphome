#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class PlayStationDriver : public HIDDeviceDriver {
 public:
  PlayStationDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    if (vid == 0x054C && pid == 0x0268) { controller_type_ = PS3; return true; }  // DualShock 3 / Sixaxis
    if (vid == 0x054C && pid == 0x042F) { controller_type_ = PS3; return true; }  // Navigation controller
    if (vid == 0x054C && (pid == 0x05C4 || pid == 0x09CC)) { controller_type_ = PS4; return true; }
    if (vid == 0x054C && (pid == 0x0CE6 || pid == 0x0DF2)) { controller_type_ = PS5; return true; }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    if (controller_type_ == PS3) {
      ESP_LOGI("usb_hidx.ps", "PS3 DualShock 3 / Sixaxis detected");
      // PS3 requires a magic enable packet over USB to start sending reports
      uint8_t buf[4] = {0x42, 0x0C, 0x00, 0x00};
      parent_->send_hid_output_report(device, buf, sizeof(buf));
    } else if (controller_type_ == PS4) {
      ESP_LOGI("usb_hidx.ps", "PS4 DualShock 4 detected");
    } else {
      ESP_LOGI("usb_hidx.ps", "PS5 DualSense detected");
    }
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_) device_ = device;
    if (controller_type_ == PS3)      process_ps3_report(data, len);
    else if (controller_type_ == PS4) process_ps4_report(data, len);
    else                              process_ps5_report(data, len);
  }

  const char *get_name() override {
    if (controller_type_ == PS3) return "PS3";
    if (controller_type_ == PS4) return "PS4";
    return "PS5";
  }

 protected:
  static void pub(binary_sensor::BinarySensor *s, bool v) { if (s) s->publish_state(v); }

  void process_ps3_report(const uint8_t *data, size_t len) {
    if (len < 10) return;
    // Log raw for debugging
    ESP_LOGD("usb_hidx.ps3", "RAW[%d]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             len, data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9]);
    // PS3 USB HID report layout:
    // byte 0: report ID (0x01)
    // byte 2: Select L3 R3 Start Up Right Down Left
    // byte 3: L2 R2 L1 R1 Triangle Circle Cross Square
    // byte 4: PS button (bit 0)
    // byte 6: LX, byte 7: LY, byte 8: RX, byte 9: RY (0-255, 128=centre)
    uint8_t b2 = data[2];
    uint8_t b3 = data[3];
    uint8_t b4 = data[4];

    if (b2 != last_btn1_) {
      pub(parent_->gamepad_button_minus_sensor_,  b2 & 0x01);  // Select
      pub(parent_->gamepad_button_l3_sensor_,     b2 & 0x02);
      pub(parent_->gamepad_button_r3_sensor_,     b2 & 0x04);
      pub(parent_->gamepad_button_plus_sensor_,   b2 & 0x08);  // Start
      pub(parent_->gamepad_dpad_up_sensor_,       b2 & 0x10);
      pub(parent_->gamepad_dpad_right_sensor_,    b2 & 0x20);
      pub(parent_->gamepad_dpad_down_sensor_,     b2 & 0x40);
      pub(parent_->gamepad_dpad_left_sensor_,     b2 & 0x80);
      last_btn1_ = b2;
    }
    if (b3 != last_btn2_) {
      pub(parent_->gamepad_button_zl_sensor_,  b3 & 0x01);  // L2
      pub(parent_->gamepad_button_zr_sensor_,  b3 & 0x02);  // R2
      pub(parent_->gamepad_button_l_sensor_,   b3 & 0x04);  // L1
      pub(parent_->gamepad_button_r_sensor_,   b3 & 0x08);  // R1
      pub(parent_->gamepad_button_y_sensor_,   b3 & 0x10);  // Triangle -> Y/top
      pub(parent_->gamepad_button_b_sensor_,   b3 & 0x20);  // Circle   -> B/right
      pub(parent_->gamepad_button_a_sensor_,   b3 & 0x40);  // Cross    -> A/bottom
      pub(parent_->gamepad_button_x_sensor_,   b3 & 0x80);  // Square   -> X/left
      last_btn2_ = b3;
    }
    if (b4 != last_btn3_) {
      pub(parent_->gamepad_button_home_sensor_, b4 & 0x01);
      last_btn3_ = b4;
    }
    // Analog sticks scaled to -100..100
    auto scale = [](uint8_t v) -> float { return ((int)v - 128) / 1.28f; };
    if (abs((int)data[6] - (int)last_lx_) > 3 || abs((int)data[7] - (int)last_ly_) > 3) {
      if (parent_->get_gamepad_left_stick_x_sensor())  parent_->get_gamepad_left_stick_x_sensor()->publish_state(scale(data[6]));
      if (parent_->get_gamepad_left_stick_y_sensor())  parent_->get_gamepad_left_stick_y_sensor()->publish_state(-scale(data[7]));
      last_lx_ = data[6]; last_ly_ = data[7];
    }
    if (abs((int)data[8] - (int)last_rx_) > 3 || abs((int)data[9] - (int)last_ry_) > 3) {
      if (parent_->get_gamepad_right_stick_x_sensor()) parent_->get_gamepad_right_stick_x_sensor()->publish_state(scale(data[8]));
      if (parent_->get_gamepad_right_stick_y_sensor()) parent_->get_gamepad_right_stick_y_sensor()->publish_state(-scale(data[9]));
      last_rx_ = data[8]; last_ry_ = data[9];
    }
  }

  void process_ps4_report(const uint8_t *data, size_t len) {
    if (len < 10) return;
    uint8_t btn1 = data[5];
    uint8_t btn2 = data[6];
    uint8_t btn3 = data[7];
    uint8_t dpad = btn1 & 0x0F;

    if (btn1 != last_btn1_) {
      if (dpad != (last_btn1_ & 0x0F)) {
        pub(parent_->gamepad_dpad_up_sensor_,    dpad == 0 || dpad == 1 || dpad == 7);
        pub(parent_->gamepad_dpad_right_sensor_, dpad == 1 || dpad == 2 || dpad == 3);
        pub(parent_->gamepad_dpad_down_sensor_,  dpad == 3 || dpad == 4 || dpad == 5);
        pub(parent_->gamepad_dpad_left_sensor_,  dpad == 5 || dpad == 6 || dpad == 7);
      }
      pub(parent_->gamepad_button_x_sensor_,  btn1 & 0x10);  // Square
      pub(parent_->gamepad_button_a_sensor_,  btn1 & 0x20);  // Cross
      pub(parent_->gamepad_button_b_sensor_,  btn1 & 0x40);  // Circle
      pub(parent_->gamepad_button_y_sensor_,  btn1 & 0x80);  // Triangle
      last_btn1_ = btn1;
    }
    if (btn2 != last_btn2_) {
      pub(parent_->gamepad_button_l_sensor_,     btn2 & 0x01);
      pub(parent_->gamepad_button_r_sensor_,     btn2 & 0x02);
      pub(parent_->gamepad_button_zl_sensor_,    btn2 & 0x04);
      pub(parent_->gamepad_button_zr_sensor_,    btn2 & 0x08);
      pub(parent_->gamepad_button_minus_sensor_, btn2 & 0x10);
      pub(parent_->gamepad_button_plus_sensor_,  btn2 & 0x20);
      pub(parent_->gamepad_button_l3_sensor_,    btn2 & 0x40);
      pub(parent_->gamepad_button_r3_sensor_,    btn2 & 0x80);
      last_btn2_ = btn2;
    }
    if (btn3 != last_btn3_) {
      pub(parent_->gamepad_button_home_sensor_,    btn3 & 0x01);
      pub(parent_->gamepad_button_capture_sensor_, btn3 & 0x02);
      last_btn3_ = btn3;
    }
    auto scale = [](uint8_t v) -> float { return ((int)v - 128) / 1.28f; };
    if (abs((int)data[1] - (int)last_lx_) > 3 || abs((int)data[2] - (int)last_ly_) > 3) {
      if (parent_->get_gamepad_left_stick_x_sensor())  parent_->get_gamepad_left_stick_x_sensor()->publish_state(scale(data[1]));
      if (parent_->get_gamepad_left_stick_y_sensor())  parent_->get_gamepad_left_stick_y_sensor()->publish_state(-scale(data[2]));
      last_lx_ = data[1]; last_ly_ = data[2];
    }
    if (abs((int)data[3] - (int)last_rx_) > 3 || abs((int)data[4] - (int)last_ry_) > 3) {
      if (parent_->get_gamepad_right_stick_x_sensor()) parent_->get_gamepad_right_stick_x_sensor()->publish_state(scale(data[3]));
      if (parent_->get_gamepad_right_stick_y_sensor()) parent_->get_gamepad_right_stick_y_sensor()->publish_state(-scale(data[4]));
      last_rx_ = data[3]; last_ry_ = data[4];
    }
  }

  void process_ps5_report(const uint8_t *data, size_t len) {
    if (len < 11) return;
    uint8_t btn1 = data[8];
    uint8_t btn2 = data[9];
    uint8_t btn3 = data[10];
    uint8_t dpad = btn1 & 0x0F;

    if (btn1 != last_btn1_) {
      if (dpad != (last_btn1_ & 0x0F)) {
        pub(parent_->gamepad_dpad_up_sensor_,    dpad == 0 || dpad == 1 || dpad == 7);
        pub(parent_->gamepad_dpad_right_sensor_, dpad == 1 || dpad == 2 || dpad == 3);
        pub(parent_->gamepad_dpad_down_sensor_,  dpad == 3 || dpad == 4 || dpad == 5);
        pub(parent_->gamepad_dpad_left_sensor_,  dpad == 5 || dpad == 6 || dpad == 7);
      }
      pub(parent_->gamepad_button_x_sensor_, btn1 & 0x10);
      pub(parent_->gamepad_button_a_sensor_, btn1 & 0x20);
      pub(parent_->gamepad_button_b_sensor_, btn1 & 0x40);
      pub(parent_->gamepad_button_y_sensor_, btn1 & 0x80);
      last_btn1_ = btn1;
    }
    if (btn2 != last_btn2_) {
      pub(parent_->gamepad_button_l_sensor_,     btn2 & 0x01);
      pub(parent_->gamepad_button_r_sensor_,     btn2 & 0x02);
      pub(parent_->gamepad_button_zl_sensor_,    btn2 & 0x04);
      pub(parent_->gamepad_button_zr_sensor_,    btn2 & 0x08);
      pub(parent_->gamepad_button_minus_sensor_, btn2 & 0x10);
      pub(parent_->gamepad_button_plus_sensor_,  btn2 & 0x20);
      pub(parent_->gamepad_button_l3_sensor_,    btn2 & 0x40);
      pub(parent_->gamepad_button_r3_sensor_,    btn2 & 0x80);
      last_btn2_ = btn2;
    }
    if (btn3 != last_btn3_) {
      pub(parent_->gamepad_button_home_sensor_,    btn3 & 0x01);
      pub(parent_->gamepad_button_capture_sensor_, btn3 & 0x02);
      last_btn3_ = btn3;
    }
    auto scale = [](uint8_t v) -> float { return ((int)v - 128) / 1.28f; };
    if (abs((int)data[1] - (int)last_lx_) > 3 || abs((int)data[2] - (int)last_ly_) > 3) {
      if (parent_->get_gamepad_left_stick_x_sensor())  parent_->get_gamepad_left_stick_x_sensor()->publish_state(scale(data[1]));
      if (parent_->get_gamepad_left_stick_y_sensor())  parent_->get_gamepad_left_stick_y_sensor()->publish_state(-scale(data[2]));
      last_lx_ = data[1]; last_ly_ = data[2];
    }
    if (abs((int)data[3] - (int)last_rx_) > 3 || abs((int)data[4] - (int)last_ry_) > 3) {
      if (parent_->get_gamepad_right_stick_x_sensor()) parent_->get_gamepad_right_stick_x_sensor()->publish_state(scale(data[3]));
      if (parent_->get_gamepad_right_stick_y_sensor()) parent_->get_gamepad_right_stick_y_sensor()->publish_state(-scale(data[4]));
      last_rx_ = data[3]; last_ry_ = data[4];
    }
  }

  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  enum ControllerType { PS3, PS4, PS5 } controller_type_{PS4};
  uint8_t last_btn1_{0}, last_btn2_{0}, last_btn3_{0};
  uint8_t last_lx_{128}, last_ly_{128}, last_rx_{128}, last_ry_{128};
};

}  // namespace usb_hidx
}  // namespace esphome
