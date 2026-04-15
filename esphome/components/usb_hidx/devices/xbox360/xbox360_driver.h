#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

namespace esphome {
namespace usb_hidx {

class Xbox360Driver : public HIDDeviceDriver {
 public:
  Xbox360Driver(USBHIDXComponent *parent) : parent_(parent) {}

  void set_button_a_sensor(binary_sensor::BinarySensor *s)       { button_a_sensor_ = s; }
  void set_button_b_sensor(binary_sensor::BinarySensor *s)       { button_b_sensor_ = s; }
  void set_button_x_sensor(binary_sensor::BinarySensor *s)       { button_x_sensor_ = s; }
  void set_button_y_sensor(binary_sensor::BinarySensor *s)       { button_y_sensor_ = s; }
  void set_button_l_sensor(binary_sensor::BinarySensor *s)       { button_l_sensor_ = s; }
  void set_button_r_sensor(binary_sensor::BinarySensor *s)       { button_r_sensor_ = s; }
  void set_button_zl_sensor(binary_sensor::BinarySensor *s)      { button_zl_sensor_ = s; }
  void set_button_zr_sensor(binary_sensor::BinarySensor *s)      { button_zr_sensor_ = s; }
  void set_button_minus_sensor(binary_sensor::BinarySensor *s)   { button_minus_sensor_ = s; }
  void set_button_plus_sensor(binary_sensor::BinarySensor *s)    { button_plus_sensor_ = s; }
  void set_button_home_sensor(binary_sensor::BinarySensor *s)    { button_home_sensor_ = s; }
  void set_button_l3_sensor(binary_sensor::BinarySensor *s)      { button_l3_sensor_ = s; }
  void set_button_r3_sensor(binary_sensor::BinarySensor *s)      { button_r3_sensor_ = s; }
  void set_dpad_up_sensor(binary_sensor::BinarySensor *s)        { dpad_up_sensor_ = s; }
  void set_dpad_down_sensor(binary_sensor::BinarySensor *s)      { dpad_down_sensor_ = s; }
  void set_dpad_left_sensor(binary_sensor::BinarySensor *s)      { dpad_left_sensor_ = s; }
  void set_dpad_right_sensor(binary_sensor::BinarySensor *s)     { dpad_right_sensor_ = s; }
#ifdef USE_SENSOR
  void set_axis_lx_sensor(sensor::Sensor *s) { axis_lx_sensor_ = s; }
  void set_axis_ly_sensor(sensor::Sensor *s) { axis_ly_sensor_ = s; }
  void set_axis_rx_sensor(sensor::Sensor *s) { axis_rx_sensor_ = s; }
  void set_axis_ry_sensor(sensor::Sensor *s) { axis_ry_sensor_ = s; }
#endif

  void set_device(HIDDevice *device) { device_ = device; }

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    is_drum_kit_ = (vid == 0x1BAD && pid == 0x0003);
    is_guitar_   = (vid == 0x1430 && pid == 0x4748);
    is_gamepad_  = (vid == 0x045E && (pid == 0x028E || pid == 0x0719)) ||
                   (vid == 0x2DC8 && pid == 0x310B);
    return is_drum_kit_ || is_guitar_ || is_gamepad_;
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (len < 20) return;

    if (len == 29 && data[0] == 0x00 && (data[1] == 0x01 || data[1] == 0x00)) {
      is_wireless_ = true;
      if (!device_) device_ = device;
      if (is_gamepad_) process_wireless_gamepad_report(data, len, device);
      return;
    }

    if (data[0] != 0x00 || data[1] != 0x14) return;
    if (!device_) device_ = device;

    if (is_gamepad_)
      process_gamepad_report(data, len, device);
    else
      process_instrument_report(data, len);
  }

  const char *get_name() override { return "Xbox360"; }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};

  binary_sensor::BinarySensor *button_a_sensor_{nullptr};
  binary_sensor::BinarySensor *button_b_sensor_{nullptr};
  binary_sensor::BinarySensor *button_x_sensor_{nullptr};
  binary_sensor::BinarySensor *button_y_sensor_{nullptr};
  binary_sensor::BinarySensor *button_l_sensor_{nullptr};
  binary_sensor::BinarySensor *button_r_sensor_{nullptr};
  binary_sensor::BinarySensor *button_zl_sensor_{nullptr};
  binary_sensor::BinarySensor *button_zr_sensor_{nullptr};
  binary_sensor::BinarySensor *button_minus_sensor_{nullptr};
  binary_sensor::BinarySensor *button_plus_sensor_{nullptr};
  binary_sensor::BinarySensor *button_home_sensor_{nullptr};
  binary_sensor::BinarySensor *button_l3_sensor_{nullptr};
  binary_sensor::BinarySensor *button_r3_sensor_{nullptr};
  binary_sensor::BinarySensor *dpad_up_sensor_{nullptr};
  binary_sensor::BinarySensor *dpad_down_sensor_{nullptr};
  binary_sensor::BinarySensor *dpad_left_sensor_{nullptr};
  binary_sensor::BinarySensor *dpad_right_sensor_{nullptr};

  bool is_drum_kit_{false};
  bool is_guitar_{false};
  bool is_gamepad_{false};
  bool is_wireless_{false};
  bool last_was_idle_{false};
  uint8_t last_dpad_{0};
  uint8_t last_buttons1_{0};
  uint8_t last_buttons2_{0};
  int16_t last_whammy_{-32768};
  int16_t last_lx_{0}, last_ly_{0}, last_rx_{0}, last_ry_{0};
  uint8_t last_lt_{0}, last_rt_{0};
#ifdef USE_SENSOR
  sensor::Sensor *axis_lx_sensor_{nullptr};
  sensor::Sensor *axis_ly_sensor_{nullptr};
  sensor::Sensor *axis_rx_sensor_{nullptr};
  sensor::Sensor *axis_ry_sensor_{nullptr};
#endif

  // Helper: publish a sensor if non-null
  static void pub(binary_sensor::BinarySensor *s, bool state) {
    if (s) s->publish_state(state);
  }

  void init_xbox360_controller(HIDDevice *device) {
    if (is_gamepad_) {
      if (device->out_endpoint) {
        uint8_t led_cmd[] = {0x01, 0x03, 0x06};
        parent_->send_xbox360_interrupt_out(device, led_cmd, sizeof(led_cmd));
      } else {
        send_led_command(device, 0x06);
      }
    }
  }

  void send_led_command(HIDDevice *device, uint8_t pattern) {
    if (is_wireless_) {
      uint8_t led_cmd[] = {0x00, 0x00, 0x08, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      parent_->send_xbox360_interrupt_out(device, led_cmd, sizeof(led_cmd));
    } else {
      uint8_t led_cmd[] = {0x01, 0x03, pattern};
      parent_->send_xbox360_output(device, led_cmd, sizeof(led_cmd));
    }
  }

 public:
  void init_controller(HIDDevice *device) { init_xbox360_controller(device); }

  void send_rumble(HIDDevice *device, uint8_t left_motor, uint8_t right_motor) {
    if (!device && device_) device = device_;
    if (!device) return;
    if (is_wireless_) {
      uint8_t cmd[] = {0x00, 0x01, 0x0F, 0xC0, 0x00, left_motor, right_motor, 0x00, 0x00, 0x00, 0x00, 0x00};
      parent_->send_xbox360_interrupt_out(device, cmd, sizeof(cmd));
    } else if (device->out_endpoint) {
      uint8_t cmd[] = {0x00, 0x08, 0x00, left_motor, right_motor, 0x00, 0x00, 0x00};
      parent_->send_xbox360_interrupt_out(device, cmd, sizeof(cmd));
    } else {
      uint8_t cmd[] = {0x00, 0x08, 0x00, left_motor, right_motor, 0x00, 0x00, 0x00};
      parent_->send_xbox360_output(device, cmd, sizeof(cmd));
    }
  }

 protected:
  // Wired report layout (partsnotincluded.com):
  // [0]=0x00 [1]=0x14 [2]=buttons_lo [3]=buttons_hi [4]=LT [5]=RT
  // [6-7]=LX [8-9]=LY [10-11]=RX [12-13]=RY
  // buttons_lo: bit0=Up bit1=Down bit2=Left bit3=Right bit4=Start bit5=Back bit6=L3 bit7=R3
  // buttons_hi: bit0=LB bit1=RB bit2=Guide bit3=0 bit4=A bit5=B bit6=X bit7=Y
  void process_gamepad_report(const uint8_t *data, size_t len, HIDDevice *device) {
    uint8_t buttons_lo = data[2];
    uint8_t buttons_hi = data[3];
    uint8_t lt = data[4], rt = data[5];
    int16_t lx = (int16_t)(data[6]  | (data[7]  << 8));
    int16_t ly = (int16_t)(data[8]  | (data[9]  << 8));
    int16_t rx = (int16_t)(data[10] | (data[11] << 8));
    int16_t ry = (int16_t)(data[12] | (data[13] << 8));
    process_common_gamepad(buttons_lo, buttons_hi, lt, rt, lx, ly, rx, ry);
  }

  void process_wireless_gamepad_report(const uint8_t *data, size_t len, HIDDevice *device) {
    if (data[1] == 0x00) return;
    // Wireless has a 4-byte header before the standard layout
    uint8_t buttons_lo = data[6];
    uint8_t buttons_hi = data[7];
    uint8_t lt = data[8], rt = data[9];
    int16_t lx = (int16_t)(data[10] | (data[11] << 8));
    int16_t ly = (int16_t)(data[12] | (data[13] << 8));
    int16_t rx = (int16_t)(data[14] | (data[15] << 8));
    int16_t ry = (int16_t)(data[16] | (data[17] << 8));
    process_common_gamepad(buttons_lo, buttons_hi, lt, rt, lx, ly, rx, ry);
  }

  void process_common_gamepad(uint8_t buttons_lo, uint8_t buttons_hi,
                               uint8_t lt, uint8_t rt,
                               int16_t lx, int16_t ly, int16_t rx, int16_t ry) {
    bool is_idle = (buttons_lo == 0 && buttons_hi == 0 &&
                    lt < 30 && rt < 30 &&
                    abs(lx) < 8000 && abs(ly) < 8000 &&
                    abs(rx) < 8000 && abs(ry) < 8000);
    if (is_idle && last_was_idle_) return;
    last_was_idle_ = is_idle;

    // D-pad: buttons_lo bits 0-3
    uint8_t dpad = buttons_lo & 0x0F;
    if (dpad != last_dpad_) {
      pub(dpad_up_sensor_,    (dpad & 0x01) != 0);
      pub(dpad_down_sensor_,  (dpad & 0x02) != 0);
      pub(dpad_left_sensor_,  (dpad & 0x04) != 0);
      pub(dpad_right_sensor_, (dpad & 0x08) != 0);
      if ((dpad & 0x01) && !(last_dpad_ & 0x01)) ESP_LOGI("usb_hidx.xbox360", "D-Pad Up pressed");
      if ((dpad & 0x02) && !(last_dpad_ & 0x02)) ESP_LOGI("usb_hidx.xbox360", "D-Pad Down pressed");
      if ((dpad & 0x04) && !(last_dpad_ & 0x04)) ESP_LOGI("usb_hidx.xbox360", "D-Pad Left pressed");
      if ((dpad & 0x08) && !(last_dpad_ & 0x08)) ESP_LOGI("usb_hidx.xbox360", "D-Pad Right pressed");
      last_dpad_ = dpad;
    }

    // Start/Back/L3/R3: buttons_lo bits 4-7
    uint8_t nav = buttons_lo & 0xF0;
    if (nav != last_buttons2_) {
      pub(button_plus_sensor_,  (nav & 0x10) != 0);
      pub(button_minus_sensor_, (nav & 0x20) != 0);
      pub(button_l3_sensor_,    (nav & 0x40) != 0);
      pub(button_r3_sensor_,    (nav & 0x80) != 0);
      if ((nav & 0x10) && !(last_buttons2_ & 0x10)) ESP_LOGI("usb_hidx.xbox360", "Button Start pressed");
      if ((nav & 0x20) && !(last_buttons2_ & 0x20)) ESP_LOGI("usb_hidx.xbox360", "Button Back pressed");
      if ((nav & 0x40) && !(last_buttons2_ & 0x40)) ESP_LOGI("usb_hidx.xbox360", "Button L3 pressed");
      if ((nav & 0x80) && !(last_buttons2_ & 0x80)) ESP_LOGI("usb_hidx.xbox360", "Button R3 pressed");
      last_buttons2_ = nav;
    }

    // LB/RB/Guide/A/B/X/Y: buttons_hi
    if (buttons_hi != last_buttons1_) {
      pub(button_l_sensor_,    (buttons_hi & 0x01) != 0);
      pub(button_r_sensor_,    (buttons_hi & 0x02) != 0);
      pub(button_home_sensor_, (buttons_hi & 0x04) != 0);
      pub(button_a_sensor_,    (buttons_hi & 0x10) != 0);
      pub(button_b_sensor_,    (buttons_hi & 0x20) != 0);
      pub(button_x_sensor_,    (buttons_hi & 0x40) != 0);
      pub(button_y_sensor_,    (buttons_hi & 0x80) != 0);
      if ((buttons_hi & 0x01) && !(last_buttons1_ & 0x01)) ESP_LOGI("usb_hidx.xbox360", "Button LB pressed");
      if ((buttons_hi & 0x02) && !(last_buttons1_ & 0x02)) ESP_LOGI("usb_hidx.xbox360", "Button RB pressed");
      if ((buttons_hi & 0x04) && !(last_buttons1_ & 0x04)) ESP_LOGI("usb_hidx.xbox360", "Button Guide pressed");
      if ((buttons_hi & 0x10) && !(last_buttons1_ & 0x10)) ESP_LOGI("usb_hidx.xbox360", "Button A pressed");
      if ((buttons_hi & 0x20) && !(last_buttons1_ & 0x20)) ESP_LOGI("usb_hidx.xbox360", "Button B pressed");
      if ((buttons_hi & 0x40) && !(last_buttons1_ & 0x40)) ESP_LOGI("usb_hidx.xbox360", "Button X pressed");
      if ((buttons_hi & 0x80) && !(last_buttons1_ & 0x80)) ESP_LOGI("usb_hidx.xbox360", "Button Y pressed");
      last_buttons1_ = buttons_hi;
    }

    // Triggers as binary (threshold 30)
    pub(button_zl_sensor_, lt > 30);
    pub(button_zr_sensor_, rt > 30);

    // Publish axis sensors (scaled to -100..100 for arc widgets)
#ifdef USE_SENSOR
    const int16_t DEADZONE = 4000;
    float lx_f = (abs(lx) < DEADZONE) ? 0.0f : (lx / 327.67f);
    float ly_f = (abs(ly) < DEADZONE) ? 0.0f : (ly / 327.67f);
    float rx_f = (abs(rx) < DEADZONE) ? 0.0f : (rx / 327.67f);
    float ry_f = (abs(ry) < DEADZONE) ? 0.0f : (ry / 327.67f);
    if (axis_lx_sensor_) axis_lx_sensor_->publish_state(lx_f);
    if (axis_ly_sensor_) axis_ly_sensor_->publish_state(ly_f);
    if (axis_rx_sensor_) axis_rx_sensor_->publish_state(rx_f);
    if (axis_ry_sensor_) axis_ry_sensor_->publish_state(ry_f);
#endif
    if (abs((int)lt - (int)last_lt_) > 10) { ESP_LOGI("usb_hidx.xbox360", "Left Trigger: %d", lt);  last_lt_ = lt; }
    if (abs((int)rt - (int)last_rt_) > 10) { ESP_LOGI("usb_hidx.xbox360", "Right Trigger: %d", rt); last_rt_ = rt; }
  }

  void process_instrument_report(const uint8_t *data, size_t len) {
    uint8_t dpad = data[2], buttons1 = data[3], buttons2 = data[4];
    const char *prefix = is_guitar_ ? "Guitar" : "Drum";
    if (dpad != last_dpad_) {
      if ((dpad & 0x01) && !(last_dpad_ & 0x01)) ESP_LOGI("usb_hidx.xbox360", "%s: Strum Up", prefix);
      if ((dpad & 0x02) && !(last_dpad_ & 0x02)) ESP_LOGI("usb_hidx.xbox360", "%s: Strum Down", prefix);
      last_dpad_ = dpad;
    }
    if (buttons1 != last_buttons1_) {
      if ((buttons1 & 0x10) && !(last_buttons1_ & 0x10)) ESP_LOGI("usb_hidx.xbox360", "%s: A", prefix);
      if ((buttons1 & 0x20) && !(last_buttons1_ & 0x20)) ESP_LOGI("usb_hidx.xbox360", "%s: B", prefix);
      if ((buttons1 & 0x40) && !(last_buttons1_ & 0x40)) ESP_LOGI("usb_hidx.xbox360", "%s: X", prefix);
      if ((buttons1 & 0x80) && !(last_buttons1_ & 0x80)) ESP_LOGI("usb_hidx.xbox360", "%s: Y", prefix);
      last_buttons1_ = buttons1;
    }
    last_buttons2_ = buttons2;
    if (is_guitar_) {
      int16_t rx = (int16_t)(data[10] | (data[11] << 8));
      if (abs(rx - last_whammy_) > 1000) { ESP_LOGI("usb_hidx.xbox360", "Guitar: Whammy=%d", rx); last_whammy_ = rx; }
    }
  }
};

}  // namespace usb_hidx
}  // namespace esphome
