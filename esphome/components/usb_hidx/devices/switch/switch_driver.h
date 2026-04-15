#pragma once

#include <cmath>
#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class SwitchDriver : public HIDDeviceDriver {
 public:
  SwitchDriver(USBHIDXComponent *parent) : parent_(parent) {}

  void set_device(HIDDevice *device) { device_ = device; }

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    is_official_ = (vid == 0x057E && pid == 0x2009);
    is_powera_ = (vid == 0x20D6 && pid == 0xA713);
    return is_official_ || is_powera_;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    if (is_official_) {
#ifdef CONFIG_IDF_TARGET_ESP32S3
      ESP_LOGW("usb_hidx.switch", "Official Switch Pro controller detected");
      ESP_LOGW("usb_hidx.switch", "NOTE: Not supported on ESP32-S3 due to USB Host stack limitations");
      ESP_LOGW("usb_hidx.switch", "Use ESP32-P4 or third-party Switch controllers instead");
      init_state_ = INIT_COMPLETE;
      initialized_ = false;
#else
      ESP_LOGI("usb_hidx.switch", "Official Switch Pro controller detected, waiting for first report");
      init_state_ = INIT_HANDSHAKE;
#endif
    }
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;

#ifdef CONFIG_IDF_TARGET_ESP32S3
    // Official controllers not supported on ESP32-S3
    if (is_official_)
      return;
#else
    // Start initialization after receiving first report (USB stack is ready)
    if (is_official_ && !init_started_) {
      init_started_ = true;
      ESP_LOGI("usb_hidx.switch", "First report received, starting initialization");
      send_usb_command(device, 0x80, 0x02);  // USB handshake
      return;
    }

    // Handle initialization responses
    if (is_official_ && init_started_ && init_state_ != INIT_COMPLETE) {
      // 0x81 = USB command ack
      if (len >= 2 && data[0] == 0x81) {
        ESP_LOGI("usb_hidx.switch", "USB ack: state=%d data[1]=0x%02X", init_state_, data[1]);
        if (init_state_ == INIT_HANDSHAKE) {
          // Step 1: send HID-only mode
          send_usb_command(device, 0x80, 0x04);
          init_state_ = INIT_BAUDRATE;
        } else if (init_state_ == INIT_BAUDRATE) {
          // Step 2: HID-only acked - set report mode 0x30
          uint8_t mode = 0x30;
          send_subcommand(device, 0x03, &mode, 1);
          init_state_ = INIT_HANDSHAKE2;
          ESP_LOGI("usb_hidx.switch", "Sent report mode 0x30, waiting for 0x21 ack");
        }
        return;
      }
      // 0x21 = subcommand ack. byte[13]=ACK byte[14]=subcommand ID
      if (len >= 15 && data[0] == 0x21) {
        uint8_t acked = data[14];
        ESP_LOGI("usb_hidx.switch", "Subcmd ack: 0x%02X state=%d", acked, init_state_);
        if (init_state_ == INIT_HANDSHAKE2 && acked == 0x03) {
          // Report mode confirmed - send LED subcommand, wait for its ack to send IMU
          set_leds(device, 0x01);
          init_state_ = INIT_COMPLETE;  // use COMPLETE as "waiting for LED ack"
          ESP_LOGI("usb_hidx.switch", "LED sent, waiting for ack to enable IMU");
        } else if (init_state_ == INIT_COMPLETE && !initialized_ && acked == 0x30) {
          // LED acked - now enable IMU
          uint8_t enable = 0x01;
          send_subcommand(device, 0x40, &enable, 1);
          initialized_ = true;
          ESP_LOGI("usb_hidx.switch", "IMU enabled - fully initialized");
        }
        return;
      }
      // Controller already sending 0x30 reports - send LED then IMU if not done yet
      if (len >= 2 && data[0] == 0x30 && init_state_ == INIT_HANDSHAKE2) {
        ESP_LOGI("usb_hidx.switch", "0x30 confirmed, sending LED");
        set_leds(device, 0x01);
        init_state_ = INIT_COMPLETE;
      }
    }
#endif

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
      if (dpad != last_dpad_) {
        if (dpad != 0x0F) {
          const char *dir[] = {"Up", "Up-Right", "Right", "Down-Right", "Down", "Down-Left", "Left", "Up-Left"};
          if (dpad < 8)
            ESP_LOGI("usb_hidx.switch", "[FIXED_V2] D-Pad: %s", dir[dpad]);
          // Publish D-pad states: 0=Up, 1=UpRight, 2=Right, 3=DownRight, 4=Down, 5=DownLeft, 6=Left, 7=UpLeft
          bool up = (dpad == 7 || dpad == 0 || dpad == 1);
          bool left = (dpad == 5 || dpad == 6 || dpad == 7);
          bool right = (dpad == 1 || dpad == 2 || dpad == 3);
          bool down = (dpad == 3 || dpad == 4 || dpad == 5);
          if (parent_->gamepad_dpad_up_sensor_)
            parent_->gamepad_dpad_up_sensor_->publish_state(up);
          if (parent_->gamepad_dpad_left_sensor_)
            parent_->gamepad_dpad_left_sensor_->publish_state(left);
          if (parent_->gamepad_dpad_right_sensor_)
            parent_->gamepad_dpad_right_sensor_->publish_state(right);
          if (parent_->gamepad_dpad_down_sensor_)
            parent_->gamepad_dpad_down_sensor_->publish_state(down);
        } else {
          // D-pad released
          if (parent_->gamepad_dpad_up_sensor_)
            parent_->gamepad_dpad_up_sensor_->publish_state(false);
          if (parent_->gamepad_dpad_left_sensor_)
            parent_->gamepad_dpad_left_sensor_->publish_state(false);
          if (parent_->gamepad_dpad_right_sensor_)
            parent_->gamepad_dpad_right_sensor_->publish_state(false);
          if (parent_->gamepad_dpad_down_sensor_)
            parent_->gamepad_dpad_down_sensor_->publish_state(false);
        }
        last_dpad_ = dpad;
      }

      // Buttons byte 0: Y(0x01), B(0x02), A(0x04), X(0x08), L(0x10), R(0x20), ZL(0x40), ZR(0x80)
      // Map to Xbox-position sensors: A=bottom(a), B=right(b), X=left(x), Y=top(y)
      // Switch Y=left->x_sensor, B=bottom->a_sensor, A=right->b_sensor, X=top->y_sensor
      if (btn0 != last_btn0_) {
        if ((btn0 & 0x01) && !(last_btn0_ & 0x01)) { ESP_LOGI("usb_hidx.switch", "Y Button"); if (parent_->gamepad_button_x_sensor_) parent_->gamepad_button_x_sensor_->publish_state(true); }
        if ((btn0 & 0x02) && !(last_btn0_ & 0x02)) { ESP_LOGI("usb_hidx.switch", "B Button"); if (parent_->gamepad_button_a_sensor_) parent_->gamepad_button_a_sensor_->publish_state(true); }
        if ((btn0 & 0x04) && !(last_btn0_ & 0x04)) { ESP_LOGI("usb_hidx.switch", "A Button"); if (parent_->gamepad_button_b_sensor_) parent_->gamepad_button_b_sensor_->publish_state(true); }
        if ((btn0 & 0x08) && !(last_btn0_ & 0x08)) { ESP_LOGI("usb_hidx.switch", "X Button"); if (parent_->gamepad_button_y_sensor_) parent_->gamepad_button_y_sensor_->publish_state(true); }
        if ((btn0 & 0x10) && !(last_btn0_ & 0x10)) { ESP_LOGI("usb_hidx.switch", "L Button"); if (parent_->gamepad_button_l_sensor_) parent_->gamepad_button_l_sensor_->publish_state(true); }
        if ((btn0 & 0x20) && !(last_btn0_ & 0x20)) { ESP_LOGI("usb_hidx.switch", "R Button"); if (parent_->gamepad_button_r_sensor_) parent_->gamepad_button_r_sensor_->publish_state(true); }
        if ((btn0 & 0x40) && !(last_btn0_ & 0x40)) { ESP_LOGI("usb_hidx.switch", "ZL Button"); if (parent_->gamepad_button_zl_sensor_) parent_->gamepad_button_zl_sensor_->publish_state(true); }
        if ((btn0 & 0x80) && !(last_btn0_ & 0x80)) { ESP_LOGI("usb_hidx.switch", "ZR Button"); if (parent_->gamepad_button_zr_sensor_) parent_->gamepad_button_zr_sensor_->publish_state(true); }
        if (!(btn0 & 0x01) && (last_btn0_ & 0x01)) { if (parent_->gamepad_button_x_sensor_)  parent_->gamepad_button_x_sensor_->publish_state(false); }
        if (!(btn0 & 0x02) && (last_btn0_ & 0x02)) { if (parent_->gamepad_button_a_sensor_)  parent_->gamepad_button_a_sensor_->publish_state(false); }
        if (!(btn0 & 0x04) && (last_btn0_ & 0x04)) { if (parent_->gamepad_button_b_sensor_)  parent_->gamepad_button_b_sensor_->publish_state(false); }
        if (!(btn0 & 0x08) && (last_btn0_ & 0x08)) { if (parent_->gamepad_button_y_sensor_)  parent_->gamepad_button_y_sensor_->publish_state(false); }
        if (!(btn0 & 0x10) && (last_btn0_ & 0x10)) { if (parent_->gamepad_button_l_sensor_)  parent_->gamepad_button_l_sensor_->publish_state(false); }
        if (!(btn0 & 0x20) && (last_btn0_ & 0x20)) { if (parent_->gamepad_button_r_sensor_)  parent_->gamepad_button_r_sensor_->publish_state(false); }
        if (!(btn0 & 0x40) && (last_btn0_ & 0x40)) { if (parent_->gamepad_button_zl_sensor_) parent_->gamepad_button_zl_sensor_->publish_state(false); }
        if (!(btn0 & 0x80) && (last_btn0_ & 0x80)) { if (parent_->gamepad_button_zr_sensor_) parent_->gamepad_button_zr_sensor_->publish_state(false); }
        last_btn0_ = btn0;
      }

      // Buttons byte 1: Minus(0x01), Plus(0x02), L-Stick(0x04), R-Stick(0x08), Home(0x10), Capture(0x20)
      if (btn1 != last_btn1_) {
        if ((btn1 & 0x01) && !(last_btn1_ & 0x01)) {
          ESP_LOGI("usb_hidx.switch", "Minus");
          if (parent_->gamepad_button_minus_sensor_)
            parent_->gamepad_button_minus_sensor_->publish_state(true);
        }
        if ((btn1 & 0x02) && !(last_btn1_ & 0x02)) {
          ESP_LOGI("usb_hidx.switch", "Plus");
          if (parent_->gamepad_button_plus_sensor_)
            parent_->gamepad_button_plus_sensor_->publish_state(true);
        }
        if ((btn1 & 0x04) && !(last_btn1_ & 0x04)) { ESP_LOGI("usb_hidx.switch", "L-Stick"); if (parent_->gamepad_button_l3_sensor_) parent_->gamepad_button_l3_sensor_->publish_state(true); }
        if ((btn1 & 0x08) && !(last_btn1_ & 0x08)) { ESP_LOGI("usb_hidx.switch", "R-Stick"); if (parent_->gamepad_button_r3_sensor_) parent_->gamepad_button_r3_sensor_->publish_state(true); }
        if ((btn1 & 0x10) && !(last_btn1_ & 0x10)) {
          ESP_LOGI("usb_hidx.switch", "Home");
          if (parent_->gamepad_button_home_sensor_)
            parent_->gamepad_button_home_sensor_->publish_state(true);
        }
        if (!(btn1 & 0x01) && (last_btn1_ & 0x01)) {
          if (parent_->gamepad_button_minus_sensor_)
            parent_->gamepad_button_minus_sensor_->publish_state(false);
        }
        if (!(btn1 & 0x02) && (last_btn1_ & 0x02)) {
          if (parent_->gamepad_button_plus_sensor_)
            parent_->gamepad_button_plus_sensor_->publish_state(false);
        }
        if (!(btn1 & 0x10) && (last_btn1_ & 0x10)) {
          if (parent_->gamepad_button_home_sensor_)
            parent_->gamepad_button_home_sensor_->publish_state(false);
        }
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
      // Map to Xbox-position sensors: Switch Y=left->x, X=top->y, B=bottom->a, A=right->b
      if (btn_right != last_btn0_) {
        if ((btn_right & 0x01) && !(last_btn0_ & 0x01)) { ESP_LOGI("usb_hidx.switch", "Y Button"); if (parent_->gamepad_button_x_sensor_) parent_->gamepad_button_x_sensor_->publish_state(true); }
        if ((btn_right & 0x02) && !(last_btn0_ & 0x02)) { ESP_LOGI("usb_hidx.switch", "X Button"); if (parent_->gamepad_button_y_sensor_) parent_->gamepad_button_y_sensor_->publish_state(true); }
        if ((btn_right & 0x04) && !(last_btn0_ & 0x04)) { ESP_LOGI("usb_hidx.switch", "B Button"); if (parent_->gamepad_button_a_sensor_) parent_->gamepad_button_a_sensor_->publish_state(true); }
        if ((btn_right & 0x08) && !(last_btn0_ & 0x08)) { ESP_LOGI("usb_hidx.switch", "A Button"); if (parent_->gamepad_button_b_sensor_) parent_->gamepad_button_b_sensor_->publish_state(true); }
        if ((btn_right & 0x40) && !(last_btn0_ & 0x40)) { ESP_LOGI("usb_hidx.switch", "R Button");  if (parent_->gamepad_button_r_sensor_)  parent_->gamepad_button_r_sensor_->publish_state(true); }
        if ((btn_right & 0x80) && !(last_btn0_ & 0x80)) { ESP_LOGI("usb_hidx.switch", "ZR Button"); if (parent_->gamepad_button_zr_sensor_) parent_->gamepad_button_zr_sensor_->publish_state(true); }
        if (!(btn_right & 0x01) && (last_btn0_ & 0x01)) { if (parent_->gamepad_button_x_sensor_)  parent_->gamepad_button_x_sensor_->publish_state(false); }
        if (!(btn_right & 0x02) && (last_btn0_ & 0x02)) { if (parent_->gamepad_button_y_sensor_)  parent_->gamepad_button_y_sensor_->publish_state(false); }
        if (!(btn_right & 0x04) && (last_btn0_ & 0x04)) { if (parent_->gamepad_button_a_sensor_)  parent_->gamepad_button_a_sensor_->publish_state(false); }
        if (!(btn_right & 0x08) && (last_btn0_ & 0x08)) { if (parent_->gamepad_button_b_sensor_)  parent_->gamepad_button_b_sensor_->publish_state(false); }
        if (!(btn_right & 0x40) && (last_btn0_ & 0x40)) { if (parent_->gamepad_button_r_sensor_)  parent_->gamepad_button_r_sensor_->publish_state(false); }
        if (!(btn_right & 0x80) && (last_btn0_ & 0x80)) { if (parent_->gamepad_button_zr_sensor_) parent_->gamepad_button_zr_sensor_->publish_state(false); }
        last_btn0_ = btn_right;
      }

      // Shared buttons (Minus, Plus, Home, Capture, L3, R3)
      if (btn_shared != last_btn1_) {
        if ((btn_shared & 0x01) && !(last_btn1_ & 0x01)) { ESP_LOGI("usb_hidx.switch", "Minus"); if (parent_->gamepad_button_minus_sensor_) parent_->gamepad_button_minus_sensor_->publish_state(true); }
        if ((btn_shared & 0x02) && !(last_btn1_ & 0x02)) { ESP_LOGI("usb_hidx.switch", "Plus");  if (parent_->gamepad_button_plus_sensor_)  parent_->gamepad_button_plus_sensor_->publish_state(true); }
        if ((btn_shared & 0x08) && !(last_btn1_ & 0x08)) { ESP_LOGI("usb_hidx.switch", "L3");    if (parent_->gamepad_button_l3_sensor_)    parent_->gamepad_button_l3_sensor_->publish_state(true); }
        if ((btn_shared & 0x04) && !(last_btn1_ & 0x04)) { ESP_LOGI("usb_hidx.switch", "R3");    if (parent_->gamepad_button_r3_sensor_)    parent_->gamepad_button_r3_sensor_->publish_state(true); }
        if ((btn_shared & 0x10) && !(last_btn1_ & 0x10)) { ESP_LOGI("usb_hidx.switch", "Home");  if (parent_->gamepad_button_home_sensor_)  parent_->gamepad_button_home_sensor_->publish_state(true); }
        if (!(btn_shared & 0x01) && (last_btn1_ & 0x01)) { if (parent_->gamepad_button_minus_sensor_) parent_->gamepad_button_minus_sensor_->publish_state(false); }
        if (!(btn_shared & 0x02) && (last_btn1_ & 0x02)) { if (parent_->gamepad_button_plus_sensor_)  parent_->gamepad_button_plus_sensor_->publish_state(false); }
        if (!(btn_shared & 0x08) && (last_btn1_ & 0x08)) { if (parent_->gamepad_button_l3_sensor_)    parent_->gamepad_button_l3_sensor_->publish_state(false); }
        if (!(btn_shared & 0x04) && (last_btn1_ & 0x04)) { if (parent_->gamepad_button_r3_sensor_)    parent_->gamepad_button_r3_sensor_->publish_state(false); }
        if (!(btn_shared & 0x10) && (last_btn1_ & 0x10)) { if (parent_->gamepad_button_home_sensor_)  parent_->gamepad_button_home_sensor_->publish_state(false); }
        if ((btn_shared & 0x20) && !(last_btn1_ & 0x20)) { ESP_LOGI("usb_hidx.switch", "Capture"); if (parent_->gamepad_button_capture_sensor_) parent_->gamepad_button_capture_sensor_->publish_state(true); }
        if (!(btn_shared & 0x20) && (last_btn1_ & 0x20)) { if (parent_->gamepad_button_capture_sensor_) parent_->gamepad_button_capture_sensor_->publish_state(false); }
        last_btn1_ = btn_shared;
      }

      // Left buttons (L, ZL, dpad)
      if ((btn_left & 0x40) && !(last_btn_left_ & 0x40)) { ESP_LOGI("usb_hidx.switch", "L Button");  if (parent_->gamepad_button_l_sensor_)  parent_->gamepad_button_l_sensor_->publish_state(true); }
      if ((btn_left & 0x80) && !(last_btn_left_ & 0x80)) { ESP_LOGI("usb_hidx.switch", "ZL Button"); if (parent_->gamepad_button_zl_sensor_) parent_->gamepad_button_zl_sensor_->publish_state(true); }
      if (!(btn_left & 0x40) && (last_btn_left_ & 0x40)) { if (parent_->gamepad_button_l_sensor_)  parent_->gamepad_button_l_sensor_->publish_state(false); }
      if (!(btn_left & 0x80) && (last_btn_left_ & 0x80)) { if (parent_->gamepad_button_zl_sensor_) parent_->gamepad_button_zl_sensor_->publish_state(false); }
      // Dpad in left byte bits 0-3
      uint8_t dpad_pro = btn_left & 0x0F;
      if (dpad_pro != (last_btn_left_ & 0x0F)) {
        if (parent_->gamepad_dpad_up_sensor_)    parent_->gamepad_dpad_up_sensor_->publish_state(btn_left & 0x02);
        if (parent_->gamepad_dpad_down_sensor_)  parent_->gamepad_dpad_down_sensor_->publish_state(btn_left & 0x01);
        if (parent_->gamepad_dpad_left_sensor_)  parent_->gamepad_dpad_left_sensor_->publish_state(btn_left & 0x08);
        if (parent_->gamepad_dpad_right_sensor_) parent_->gamepad_dpad_right_sensor_->publish_state(btn_left & 0x04);
      }
      last_btn_left_ = btn_left;

      // Analog sticks (12-bit values, centre ~2048, range 0-4095)
      uint16_t lx = (data[6] | ((data[7] & 0x0F) << 8));
      uint16_t ly = ((data[7] >> 4) | (data[8] << 4));
      uint16_t rx = (data[9] | ((data[10] & 0x0F) << 8));
      uint16_t ry = ((data[10] >> 4) | (data[11] << 4));

      if (abs((int)lx - (int)last_lx_) > 50 || abs((int)ly - (int)last_ly_) > 50) {
        float lx_f = ((int)lx - 2048) / 20.48f;
        float ly_f = ((int)ly - 2048) / 20.48f;
        if (parent_->get_gamepad_left_stick_x_sensor())  parent_->get_gamepad_left_stick_x_sensor()->publish_state(lx_f);
        if (parent_->get_gamepad_left_stick_y_sensor())  parent_->get_gamepad_left_stick_y_sensor()->publish_state(ly_f);
        last_lx_ = lx; last_ly_ = ly;
      }
      if (abs((int)rx - (int)last_rx_) > 50 || abs((int)ry - (int)last_ry_) > 50) {
        float rx_f = ((int)rx - 2048) / 20.48f;
        float ry_f = ((int)ry - 2048) / 20.48f;
        if (parent_->get_gamepad_right_stick_x_sensor()) parent_->get_gamepad_right_stick_x_sensor()->publish_state(rx_f);
        if (parent_->get_gamepad_right_stick_y_sensor()) parent_->get_gamepad_right_stick_y_sensor()->publish_state(ry_f);
        last_rx_ = rx; last_ry_ = ry;
      }

      // IMU: 3 samples of accel+gyro at bytes 13-24 each (total 36 bytes)
      // Average the 3 samples. Each value is int16_t little-endian.
      // Order per sample: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
      if (len >= 49) {
        int32_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
        for (int s = 0; s < 3; s++) {
          int base = 13 + s * 12;
          ax += (int16_t)(data[base]     | (data[base+1]  << 8));
          ay += (int16_t)(data[base+2]   | (data[base+3]  << 8));
          az += (int16_t)(data[base+4]   | (data[base+5]  << 8));
          gx += (int16_t)(data[base+6]   | (data[base+7]  << 8));
          gy += (int16_t)(data[base+8]   | (data[base+9]  << 8));
          gz += (int16_t)(data[base+10]  | (data[base+11] << 8));
        }
        // Scale: accel ~0.000244 G/unit, gyro ~0.06103 deg/s/unit
        float ax_g = (ax / 3) * 0.000244f;
        float ay_g = (ay / 3) * 0.000244f;
        float az_g = (az / 3) * 0.000244f;
        float gx_d = (gx / 3) * 0.06103f;
        float gy_d = (gy / 3) * 0.06103f;
        float gz_d = (gz / 3) * 0.06103f;
        if (abs(ax_g - last_ax_) > 0.01f || abs(ay_g - last_ay_) > 0.01f || abs(az_g - last_az_) > 0.01f) {
          ESP_LOGD("usb_hidx.switch", "IMU Accel: X=%.2f Y=%.2f Z=%.2f G", ax_g, ay_g, az_g);
          last_ax_ = ax_g; last_ay_ = ay_g; last_az_ = az_g;
        }
        if (abs(gx_d - last_gx_) > 1.0f || abs(gy_d - last_gy_) > 1.0f || abs(gz_d - last_gz_) > 1.0f) {
          ESP_LOGD("usb_hidx.switch", "IMU Gyro:  X=%.1f Y=%.1f Z=%.1f dps", gx_d, gy_d, gz_d);
          last_gx_ = gx_d; last_gy_ = gy_d; last_gz_ = gz_d;
        }
        // Publish IMU via keyboard sensor as formatted string for display
#ifdef USE_TEXT_SENSOR
        static uint32_t last_imu_pub = 0;
        uint32_t now = millis();
        if (now - last_imu_pub > 100) {
          last_imu_pub = now;
          char buf[64];
          snprintf(buf, sizeof(buf), "A:%.1f,%.1f,%.1f G:%.0f,%.0f,%.0f",
                   ax_g, ay_g, az_g, gx_d, gy_d, gz_d);
          if (parent_->get_keyboard_sensor())
            parent_->get_keyboard_sensor()->publish_state(buf);
        }
#endif
      }
    }
  }

  const char *get_name() override { return "Switch"; }

 public:
  // Rumble: strength 0-255.
  // Uses proper Switch HD rumble encoding from RE docs.
  // freq=160Hz (HF=0x0080, LF=0x40), amplitude scaled from strength.
  void send_rumble(uint8_t left_strength, uint8_t right_strength) {
    if (!device_) return;
    uint8_t buf[64] = {0};
    buf[0] = 0x10;  // Rumble-only output report
    buf[1] = global_packet_counter_++ & 0x0F;

    auto encode = [](uint8_t strength, uint8_t *out) {
      if (strength == 0) {
        // Neutral: no vibration
        out[0] = 0x00; out[1] = 0x01; out[2] = 0x40; out[3] = 0x40;
        return;
      }
      // Use 160Hz frequency: HF=0x0080, LF=0x40
      // Amplitude: scale strength 0-255 to encoded amp
      // encoded_hex_amp = round(log2(strength/255.0 * 8.7) * 32)
      // For simplicity use precomputed values:
      // strength=255 -> amp~0x72, strength=128 -> amp~0x60, strength=64 -> amp~0x50
      float amp = (strength / 255.0f) * 1.0f;  // 0.0-1.0
      uint8_t enc_amp;
      if (amp > 0.23f)
        enc_amp = (uint8_t)(logf(amp * 8.7f) / logf(2.0f) * 32.0f);
      else if (amp > 0.12f)
        enc_amp = (uint8_t)(logf(amp * 17.0f) / logf(2.0f) * 16.0f);
      else
        enc_amp = 0x20;  // minimum audible
      uint16_t hf_amp = enc_amp * 2;
      uint8_t  lf_amp_hi = enc_amp / 2 + 64;  // +0x40
      // HF=0x0080 (160Hz), LF=0x40 (160Hz)
      uint16_t hf = 0x0080;
      uint8_t  lf = 0x40;
      out[0] = hf & 0xFF;
      out[1] = (hf_amp & 0xFF) + ((hf >> 8) & 0xFF);
      out[2] = lf + ((lf_amp_hi >> 8) & 0xFF);
      out[3] = lf_amp_hi & 0xFF;
    };

    encode(left_strength,  &buf[2]);
    encode(right_strength, &buf[6]);
    parent_->send_xbox360_interrupt_out(device_, buf, 10);
  }

  void send_usb_command(HIDDevice *device, uint8_t cmd1, uint8_t cmd2) {
    uint8_t buf[2] = {cmd1, cmd2};
    parent_->send_xbox360_interrupt_out(device, buf, 2);
    ESP_LOGI("usb_hidx.switch", "Sent USB command: 0x%02X 0x%02X", cmd1, cmd2);
  }

  void send_subcommand(HIDDevice *device, uint8_t subcommand, const uint8_t *data, size_t len) {
    uint8_t buf[64] = {0};
    buf[0] = 0x01;  // Report ID for subcommands
    buf[1] = global_packet_counter_++ & 0x0F;  // Incrementing counter 0x0-0xF
    // Neutral rumble data bytes 2-9
    buf[2] = 0x00; buf[3] = 0x01; buf[4] = 0x40; buf[5] = 0x40;
    buf[6] = 0x00; buf[7] = 0x01; buf[8] = 0x40; buf[9] = 0x40;
    buf[10] = subcommand;
    if (data && len > 0)
      memcpy(&buf[11], data, len);
    parent_->send_xbox360_interrupt_out(device, buf, 11 + len);
    ESP_LOGI("usb_hidx.switch", "Sent subcommand: 0x%02X", subcommand);
  }

  void enable_full_mode(HIDDevice *device) {
    // Enable full input report mode (0x30)
    uint8_t buf[10] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
    parent_->send_xbox360_interrupt_out(device, buf, 10);
    ESP_LOGI("usb_hidx.switch", "Enabled full report mode");
  }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  bool is_official_{false};
  bool is_powera_{false};
  bool initialized_{false};
  bool init_started_{false};
  uint8_t last_dpad_{0x0F};
  uint8_t last_btn0_{0};
  uint8_t last_btn1_{0};
  uint8_t last_btn_left_{0};
  uint16_t last_lx_{128};
  uint16_t last_ly_{128};
  uint16_t last_rx_{128};
  uint16_t last_ry_{128};
  float last_ax_{0}, last_ay_{0}, last_az_{0};
  float last_gx_{0}, last_gy_{0}, last_gz_{0};
  uint8_t global_packet_counter_{0};

  enum InitState { INIT_HANDSHAKE, INIT_BAUDRATE, INIT_HANDSHAKE2, INIT_COMPLETE };
  InitState init_state_{INIT_HANDSHAKE};

  void set_leds(HIDDevice *device, uint8_t led_mask) {
    send_subcommand(device, 0x30, &led_mask, 1);
    ESP_LOGI("usb_hidx.switch", "Sent LED mask: 0x%02X", led_mask);
  }
};

}  // namespace usb_hidx
}  // namespace esphome
