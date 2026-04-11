#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class GenericGamepadDriver : public HIDDeviceDriver {
 public:
  GenericGamepadDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Don't match Xbox 360 (handled by Xbox360Driver)
    if (vid == 0x045E && pid == 0x028E)
      return false;
    // Don't match 8BitDo in Xbox mode (handled by Xbox360Driver)
    if (vid == 0x2DC8 && pid == 0x310B)
      return false;
    // Don't match Switch controllers (handled by SwitchDriver)
    if ((vid == 0x057E && pid == 0x2009) || (vid == 0x20D6 && pid == 0xA713))
      return false;
    // Match 8BitDo controllers - both dongle and direct connect
    if (vid == 0x2DC8) {
      ESP_LOGI("usb_hidx.gamepad", "Matched 8BitDo controller VID:0x%04X PID:0x%04X", vid, pid);
      is_8bitdo_ = true;
      return true;
    }
    // Match generic HID gamepads (protocol 0) as fallback
    if (protocol == 0x00) {
      ESP_LOGI("usb_hidx.gamepad", "Matched generic gamepad (protocol 0) VID:0x%04X PID:0x%04X", vid, pid);
      return true;
    }
    return false;
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (len < 8)
      return;

    // Debug: Log raw report data for 8BitDo controllers (first time only)
    if (is_8bitdo_ && !report_logged_) {
      ESP_LOGI("usb_hidx.gamepad", "8BitDo Report Length: %d", len);
      std::string hex_dump = "Raw: ";
      for (size_t i = 0; i < len && i < 32; i++) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%02X ", data[i]);
        hex_dump += buf;
      }
      ESP_LOGI("usb_hidx.gamepad", "%s", hex_dump.c_str());
      report_logged_ = true;
    }

    // Log every report for debugging (remove after testing)
    if (is_8bitdo_) {
      ESP_LOGD("usb_hidx.gamepad", "Report [%d bytes]: %02X %02X %02X %02X %02X %02X %02X %02X",
               len, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    }

    // 8BitDo controllers use standard HID gamepad format
    // Typical format: [buttons_low, buttons_high, lx, ly, rx, ry, triggers...]
    if (is_8bitdo_) {
      process_8bitdo_report(data, len);
      return;
    }

    // PowerA Switch controller format: [btn0, btn1, dpad, lx, ly, rx, ry, 0x00]
    uint8_t btn0 = data[0];
    uint8_t btn1 = data[1];
    uint8_t dpad = data[2];
    uint8_t lx = data[3];
    uint8_t ly = data[4];
    uint8_t rx = data[5];
    uint8_t ry = data[6];

    // Track button state changes
    if (btn0 != last_buttons_[0]) {
      // Byte 0: Y(0x01), B(0x02), A(0x04), X(0x08), L(0x10), R(0x20), ZL(0x40), ZR(0x80)
      if ((btn0 & 0x01) && !(last_buttons_[0] & 0x01))
        ESP_LOGI("usb_hidx.gamepad", "Button Y pressed");
      if ((btn0 & 0x02) && !(last_buttons_[0] & 0x02)) {
        ESP_LOGI("usb_hidx.gamepad", "Button B pressed");
        if (parent_->gamepad_button_b_sensor_)
          parent_->gamepad_button_b_sensor_->publish_state(true);
      }
      if ((btn0 & 0x04) && !(last_buttons_[0] & 0x04)) {
        ESP_LOGI("usb_hidx.gamepad", "Button A pressed");
        if (parent_->gamepad_button_a_sensor_)
          parent_->gamepad_button_a_sensor_->publish_state(true);
      }
      if ((btn0 & 0x08) && !(last_buttons_[0] & 0x08))
        ESP_LOGI("usb_hidx.gamepad", "Button X pressed");
      if ((btn0 & 0x10) && !(last_buttons_[0] & 0x10))
        ESP_LOGI("usb_hidx.gamepad", "Button L pressed");
      if ((btn0 & 0x20) && !(last_buttons_[0] & 0x20))
        ESP_LOGI("usb_hidx.gamepad", "Button R pressed");
      if ((btn0 & 0x40) && !(last_buttons_[0] & 0x40))
        ESP_LOGI("usb_hidx.gamepad", "Button ZL pressed");
      if ((btn0 & 0x80) && !(last_buttons_[0] & 0x80))
        ESP_LOGI("usb_hidx.gamepad", "Button ZR pressed");
      
      // Handle button releases
      if (!(btn0 & 0x02) && (last_buttons_[0] & 0x02)) {
        if (parent_->gamepad_button_b_sensor_)
          parent_->gamepad_button_b_sensor_->publish_state(false);
      }
      if (!(btn0 & 0x04) && (last_buttons_[0] & 0x04)) {
        if (parent_->gamepad_button_a_sensor_)
          parent_->gamepad_button_a_sensor_->publish_state(false);
      }
      
      last_buttons_[0] = btn0;
    }

    if (btn1 != last_buttons_[1]) {
      // Byte 1: Minus(0x01), Plus(0x02), L-Stick(0x04), R-Stick(0x08), Home(0x10), Capture(0x20)
      if ((btn1 & 0x01) && !(last_buttons_[1] & 0x01))
        ESP_LOGI("usb_hidx.gamepad", "Button Minus pressed");
      if ((btn1 & 0x02) && !(last_buttons_[1] & 0x02))
        ESP_LOGI("usb_hidx.gamepad", "Button Plus pressed");
      if ((btn1 & 0x04) && !(last_buttons_[1] & 0x04))
        ESP_LOGI("usb_hidx.gamepad", "Button L-Stick pressed");
      if ((btn1 & 0x08) && !(last_buttons_[1] & 0x08))
        ESP_LOGI("usb_hidx.gamepad", "Button R-Stick pressed");
      if ((btn1 & 0x10) && !(last_buttons_[1] & 0x10))
        ESP_LOGI("usb_hidx.gamepad", "Button Home pressed");
      if ((btn1 & 0x20) && !(last_buttons_[1] & 0x20))
        ESP_LOGI("usb_hidx.gamepad", "Button Capture pressed");
      last_buttons_[1] = btn1;
    }

    // D-Pad (byte 2) - Custom encoding for this gamepad
    // 0x00=Neutral, 0x01=Up, 0x02=Right, 0x04=Down, 0x06=Left
    if (dpad != last_dpad_) {
      ESP_LOGD("usb_hidx.gamepad", "[FIXED_V2] D-Pad changed: %02X -> %02X", last_dpad_, dpad);
      
      // Release previous directions
      if (last_dpad_ == 0x01 && parent_->gamepad_dpad_up_sensor_)
        parent_->gamepad_dpad_up_sensor_->publish_state(false);
      if (last_dpad_ == 0x02 && parent_->gamepad_dpad_right_sensor_)
        parent_->gamepad_dpad_right_sensor_->publish_state(false);
      if (last_dpad_ == 0x04 && parent_->gamepad_dpad_down_sensor_)
        parent_->gamepad_dpad_down_sensor_->publish_state(false);
      if (last_dpad_ == 0x06 && parent_->gamepad_dpad_left_sensor_)
        parent_->gamepad_dpad_left_sensor_->publish_state(false);
      
      // Press new direction
      if (dpad == 0x01) {
        ESP_LOGI("usb_hidx.switch", "[FIXED_V2] D-Pad: Up");
        if (parent_->gamepad_dpad_up_sensor_)
          parent_->gamepad_dpad_up_sensor_->publish_state(true);
      } else if (dpad == 0x02) {
        ESP_LOGI("usb_hidx.switch", "[FIXED_V2] D-Pad: Right");
        if (parent_->gamepad_dpad_right_sensor_)
          parent_->gamepad_dpad_right_sensor_->publish_state(true);
      } else if (dpad == 0x04) {
        ESP_LOGI("usb_hidx.switch", "[FIXED_V2] D-Pad: Down");
        if (parent_->gamepad_dpad_down_sensor_)
          parent_->gamepad_dpad_down_sensor_->publish_state(true);
      } else if (dpad == 0x06) {
        ESP_LOGI("usb_hidx.switch", "[FIXED_V2] D-Pad: Left");
        if (parent_->gamepad_dpad_left_sensor_)
          parent_->gamepad_dpad_left_sensor_->publish_state(true);
      } else if (dpad == 0x00) {
        ESP_LOGD("usb_hidx.gamepad", "[FIXED_V2] D-Pad: Neutral");
      }
      
      last_dpad_ = dpad;
    }

    // Analog sticks (0x80 = center, deadzone = 30)
    if (abs((int) lx - (int) last_lx_) > 30 || abs((int) ly - (int) last_ly_) > 30) {
      ESP_LOGI("usb_hidx.gamepad", "Left Stick: X=%d Y=%d", lx, ly);
      last_lx_ = lx;
      last_ly_ = ly;
    }
    if (abs((int) rx - (int) last_rx_) > 30 || abs((int) ry - (int) last_ry_) > 30) {
      ESP_LOGI("usb_hidx.gamepad", "Right Stick: X=%d Y=%d", rx, ry);
      last_rx_ = rx;
      last_ry_ = ry;
    }
  }

  const char *get_name() override { return is_8bitdo_ ? "8BitDo" : "GenericGamepad"; }

 protected:
  void process_8bitdo_report(const uint8_t *data, size_t len) {
    // 8BitDo Ultimate 2 format (standard HID gamepad):
    // Byte 0-1: Button bits (16 buttons)
    // Byte 2: Left stick X (low byte)
    // Byte 3: Left stick X (high byte)
    // Byte 4: Left stick Y (low byte)
    // Byte 5: Left stick Y (high byte)
    // Byte 6: Right stick X (low byte)
    // Byte 7: Right stick X (high byte)
    // Byte 8: Right stick Y (low byte)
    // Byte 9: Right stick Y (high byte)
    // Byte 10: D-pad hat switch
    // Byte 11-12: Triggers
    
    uint16_t buttons = data[0] | (data[1] << 8);
    
    // Standard button mapping (matches Linux BTN_SOUTH/EAST/NORTH/WEST)
    // Bit 0: BTN_SOUTH (A/Cross) - Bottom button
    // Bit 1: BTN_EAST (B/Circle) - Right button  
    // Bit 2: BTN_WEST (X/Square) - Left button
    // Bit 3: BTN_NORTH (Y/Triangle) - Top button
    // Bit 4: BTN_TL (L1/LB)
    // Bit 5: BTN_TR (R1/RB)
    // Bit 6: BTN_SELECT (Back/Share)
    // Bit 7: BTN_START (Start/Options)
    // Bit 8: BTN_MODE (Home/Guide)
    // Bit 9: BTN_THUMBL (L3)
    // Bit 10: BTN_THUMBR (R3)
    
    if (buttons != last_buttons_16_) {
      ESP_LOGI("usb_hidx.gamepad", "Button state changed: 0x%04X -> 0x%04X", last_buttons_16_, buttons);
      
      // BTN_SOUTH (A button)
      if ((buttons & 0x0001) && !(last_buttons_16_ & 0x0001)) {
        ESP_LOGI("usb_hidx.gamepad", "Button A (South) pressed");
        if (parent_->gamepad_button_a_sensor_)
          parent_->gamepad_button_a_sensor_->publish_state(true);
      } else if (!(buttons & 0x0001) && (last_buttons_16_ & 0x0001)) {
        if (parent_->gamepad_button_a_sensor_)
          parent_->gamepad_button_a_sensor_->publish_state(false);
      }
      
      // BTN_EAST (B button)
      if ((buttons & 0x0002) && !(last_buttons_16_ & 0x0002)) {
        ESP_LOGI("usb_hidx.gamepad", "Button B (East) pressed");
        if (parent_->gamepad_button_b_sensor_)
          parent_->gamepad_button_b_sensor_->publish_state(true);
      } else if (!(buttons & 0x0002) && (last_buttons_16_ & 0x0002)) {
        if (parent_->gamepad_button_b_sensor_)
          parent_->gamepad_button_b_sensor_->publish_state(false);
      }
      
      // BTN_WEST (X button)
      if ((buttons & 0x0004) && !(last_buttons_16_ & 0x0004)) {
        ESP_LOGI("usb_hidx.gamepad", "Button X (West) pressed");
        if (parent_->gamepad_button_x_sensor_)
          parent_->gamepad_button_x_sensor_->publish_state(true);
      } else if (!(buttons & 0x0004) && (last_buttons_16_ & 0x0004)) {
        if (parent_->gamepad_button_x_sensor_)
          parent_->gamepad_button_x_sensor_->publish_state(false);
      }
      
      // BTN_NORTH (Y button)
      if ((buttons & 0x0008) && !(last_buttons_16_ & 0x0008)) {
        ESP_LOGI("usb_hidx.gamepad", "Button Y (North) pressed");
        if (parent_->gamepad_button_y_sensor_)
          parent_->gamepad_button_y_sensor_->publish_state(true);
      } else if (!(buttons & 0x0008) && (last_buttons_16_ & 0x0008)) {
        if (parent_->gamepad_button_y_sensor_)
          parent_->gamepad_button_y_sensor_->publish_state(false);
      }
      
      // BTN_TL (L1/LB)
      if ((buttons & 0x0010) && !(last_buttons_16_ & 0x0010)) {
        ESP_LOGI("usb_hidx.gamepad", "Button L pressed");
        if (parent_->gamepad_button_l_sensor_)
          parent_->gamepad_button_l_sensor_->publish_state(true);
      } else if (!(buttons & 0x0010) && (last_buttons_16_ & 0x0010)) {
        if (parent_->gamepad_button_l_sensor_)
          parent_->gamepad_button_l_sensor_->publish_state(false);
      }
      
      // BTN_TR (R1/RB)
      if ((buttons & 0x0020) && !(last_buttons_16_ & 0x0020)) {
        ESP_LOGI("usb_hidx.gamepad", "Button R pressed");
        if (parent_->gamepad_button_r_sensor_)
          parent_->gamepad_button_r_sensor_->publish_state(true);
      } else if (!(buttons & 0x0020) && (last_buttons_16_ & 0x0020)) {
        if (parent_->gamepad_button_r_sensor_)
          parent_->gamepad_button_r_sensor_->publish_state(false);
      }
      
      // BTN_SELECT (Back/Share)
      if ((buttons & 0x0040) && !(last_buttons_16_ & 0x0040)) {
        ESP_LOGI("usb_hidx.gamepad", "Button Select pressed");
        if (parent_->gamepad_button_minus_sensor_)
          parent_->gamepad_button_minus_sensor_->publish_state(true);
      } else if (!(buttons & 0x0040) && (last_buttons_16_ & 0x0040)) {
        if (parent_->gamepad_button_minus_sensor_)
          parent_->gamepad_button_minus_sensor_->publish_state(false);
      }
      
      // BTN_START (Start/Options)
      if ((buttons & 0x0080) && !(last_buttons_16_ & 0x0080)) {
        ESP_LOGI("usb_hidx.gamepad", "Button Start pressed");
        if (parent_->gamepad_button_plus_sensor_)
          parent_->gamepad_button_plus_sensor_->publish_state(true);
      } else if (!(buttons & 0x0080) && (last_buttons_16_ & 0x0080)) {
        if (parent_->gamepad_button_plus_sensor_)
          parent_->gamepad_button_plus_sensor_->publish_state(false);
      }
      
      // BTN_MODE (Home/Guide)
      if ((buttons & 0x0100) && !(last_buttons_16_ & 0x0100)) {
        ESP_LOGI("usb_hidx.gamepad", "Button Home pressed");
        if (parent_->gamepad_button_home_sensor_)
          parent_->gamepad_button_home_sensor_->publish_state(true);
      } else if (!(buttons & 0x0100) && (last_buttons_16_ & 0x0100)) {
        if (parent_->gamepad_button_home_sensor_)
          parent_->gamepad_button_home_sensor_->publish_state(false);
      }
      
      // BTN_THUMBL (L3)
      if ((buttons & 0x0200) && !(last_buttons_16_ & 0x0200)) {
        ESP_LOGI("usb_hidx.gamepad", "Button L3 pressed");
        if (parent_->gamepad_button_l3_sensor_)
          parent_->gamepad_button_l3_sensor_->publish_state(true);
      } else if (!(buttons & 0x0200) && (last_buttons_16_ & 0x0200)) {
        if (parent_->gamepad_button_l3_sensor_)
          parent_->gamepad_button_l3_sensor_->publish_state(false);
      }
      
      // BTN_THUMBR (R3)
      if ((buttons & 0x0400) && !(last_buttons_16_ & 0x0400)) {
        ESP_LOGI("usb_hidx.gamepad", "Button R3 pressed");
        if (parent_->gamepad_button_r3_sensor_)
          parent_->gamepad_button_r3_sensor_->publish_state(true);
      } else if (!(buttons & 0x0400) && (last_buttons_16_ & 0x0400)) {
        if (parent_->gamepad_button_r3_sensor_)
          parent_->gamepad_button_r3_sensor_->publish_state(false);
      }
      
      last_buttons_16_ = buttons;
    }
    
    // D-pad (HAT switch) - typically at byte 10 for 8BitDo
    if (len > 10) {
      uint8_t dpad = data[10];
      if (dpad != last_dpad_) {
        // Release all directions first
        if (parent_->gamepad_dpad_up_sensor_)
          parent_->gamepad_dpad_up_sensor_->publish_state(false);
        if (parent_->gamepad_dpad_down_sensor_)
          parent_->gamepad_dpad_down_sensor_->publish_state(false);
        if (parent_->gamepad_dpad_left_sensor_)
          parent_->gamepad_dpad_left_sensor_->publish_state(false);
        if (parent_->gamepad_dpad_right_sensor_)
          parent_->gamepad_dpad_right_sensor_->publish_state(false);
        
        // HAT switch values: 0=Up, 1=UpRight, 2=Right, 3=DownRight, 4=Down, 5=DownLeft, 6=Left, 7=UpLeft, 8=Neutral
        if (dpad == 0 || dpad == 1 || dpad == 7) { // Up
          if (parent_->gamepad_dpad_up_sensor_)
            parent_->gamepad_dpad_up_sensor_->publish_state(true);
        }
        if (dpad == 4 || dpad == 3 || dpad == 5) { // Down
          if (parent_->gamepad_dpad_down_sensor_)
            parent_->gamepad_dpad_down_sensor_->publish_state(true);
        }
        if (dpad == 6 || dpad == 5 || dpad == 7) { // Left
          if (parent_->gamepad_dpad_left_sensor_)
            parent_->gamepad_dpad_left_sensor_->publish_state(true);
        }
        if (dpad == 2 || dpad == 1 || dpad == 3) { // Right
          if (parent_->gamepad_dpad_right_sensor_)
            parent_->gamepad_dpad_right_sensor_->publish_state(true);
        }
        
        last_dpad_ = dpad;
      }
    }
    
    // Analog sticks (16-bit values)
    if (len >= 10) {
      int16_t lx = (int16_t)(data[2] | (data[3] << 8));
      int16_t ly = (int16_t)(data[4] | (data[5] << 8));
      int16_t rx = (int16_t)(data[6] | (data[7] << 8));
      int16_t ry = (int16_t)(data[8] | (data[9] << 8));
      
      if (abs(lx - last_lx_16_) > 2000 || abs(ly - last_ly_16_) > 2000) {
        ESP_LOGD("usb_hidx.gamepad", "Left Stick: X=%d Y=%d", lx, ly);
        last_lx_16_ = lx;
        last_ly_16_ = ly;
      }
      if (abs(rx - last_rx_16_) > 2000 || abs(ry - last_ry_16_) > 2000) {
        ESP_LOGD("usb_hidx.gamepad", "Right Stick: X=%d Y=%d", rx, ry);
        last_rx_16_ = rx;
        last_ry_16_ = ry;
      }
    }
  }

  USBHIDXComponent *parent_;
  bool is_8bitdo_{false};
  bool report_logged_{false};
  uint8_t last_buttons_[2]{0};
  uint16_t last_buttons_16_{0};
  uint8_t last_dpad_{0xFF};
  uint8_t last_lx_{0x80};
  uint8_t last_ly_{0x80};
  uint8_t last_rx_{0x80};
  uint8_t last_ry_{0x80};
  int16_t last_lx_16_{0};
  int16_t last_ly_16_{0};
  int16_t last_rx_16_{0};
  int16_t last_ry_16_{0};
};

}  // namespace usb_hidx
}  // namespace esphome
