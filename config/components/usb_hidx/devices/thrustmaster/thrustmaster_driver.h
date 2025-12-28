#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class ThrustmasterDriver : public HIDDeviceDriver {
 public:
  ThrustmasterDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Thrustmaster devices
    if (vid == 0x044F) {
      // Detect device type by PID
      if (pid == 0xB65D || pid == 0xB66D || pid == 0xB66E) {
        device_type_ = RACING_WHEEL;
        ESP_LOGI("usb_hidx.thrustmaster", "Racing wheel detected");
      } else if (pid == 0x0402 || pid == 0xB108) {
        device_type_ = FLIGHT_STICK;
        ESP_LOGI("usb_hidx.thrustmaster", "Flight stick detected");
      } else {
        device_type_ = GAMEPAD;
        ESP_LOGI("usb_hidx.thrustmaster", "Gamepad detected");
      }
      return true;
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    const char *type_name[] = {"Racing Wheel", "Flight Stick", "Gamepad"};
    ESP_LOGI("usb_hidx.thrustmaster", "Thrustmaster %s ready", type_name[device_type_]);
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;

    if (device_type_ == RACING_WHEEL) {
      process_wheel_report(data, len);
    } else if (device_type_ == FLIGHT_STICK) {
      process_stick_report(data, len);
    } else {
      process_gamepad_report(data, len);
    }
  }

  const char *get_name() override { return "Thrustmaster"; }

 protected:
  void process_wheel_report(const uint8_t *data, size_t len) {
    if (len < 8)
      return;

    // Racing wheel format
    // Steering wheel (16-bit, centered at 0x8000)
    int16_t steering = (data[1] << 8) | data[0];

    // Pedals (8-bit each)
    uint8_t throttle = data[2];
    uint8_t brake = data[3];
    uint8_t clutch = len > 4 ? data[4] : 0;

    // Buttons (2 bytes)
    uint16_t buttons = len > 6 ? ((data[6] << 8) | data[5]) : 0;

    // Log significant steering changes
    if (abs(steering - last_steering_) > 1000) {
      ESP_LOGI("usb_hidx.thrustmaster", "Steering: %d", steering);
      last_steering_ = steering;
    }

    // Log pedal changes
    if (abs((int) throttle - (int) last_throttle_) > 10) {
      ESP_LOGI("usb_hidx.thrustmaster", "Throttle: %d%%", (throttle * 100) / 255);
      last_throttle_ = throttle;
    }
    if (abs((int) brake - (int) last_brake_) > 10) {
      ESP_LOGI("usb_hidx.thrustmaster", "Brake: %d%%", (brake * 100) / 255);
      last_brake_ = brake;
    }

    // Log button presses
    if (buttons != last_buttons_) {
      for (int i = 0; i < 16; i++) {
        if ((buttons & (1 << i)) && !(last_buttons_ & (1 << i))) {
          ESP_LOGI("usb_hidx.thrustmaster", "Button %d", i + 1);
        }
      }
      last_buttons_ = buttons;
    }
  }

  void process_stick_report(const uint8_t *data, size_t len) {
    if (len < 6)
      return;

    // Flight stick format
    // X/Y axes (16-bit each)
    int16_t x = (data[1] << 8) | data[0];
    int16_t y = (data[3] << 8) | data[2];

    // Throttle/rudder
    uint8_t throttle = len > 4 ? data[4] : 0;
    uint8_t rudder = len > 5 ? data[5] : 0;

    // Buttons
    uint16_t buttons = len > 7 ? ((data[7] << 8) | data[6]) : 0;

    // Log stick movement
    if (abs(x - last_x_) > 1000 || abs(y - last_y_) > 1000) {
      ESP_LOGV("usb_hidx.thrustmaster", "Stick: X=%d Y=%d", x, y);
      last_x_ = x;
      last_y_ = y;
    }

    // Log button presses
    if (buttons != last_buttons_) {
      for (int i = 0; i < 16; i++) {
        if ((buttons & (1 << i)) && !(last_buttons_ & (1 << i))) {
          ESP_LOGI("usb_hidx.thrustmaster", "Button %d", i + 1);
        }
      }
      last_buttons_ = buttons;
    }
  }

  void process_gamepad_report(const uint8_t *data, size_t len) {
    if (len < 6)
      return;

    // Standard gamepad format
    uint8_t buttons = data[5];

    if (buttons != last_buttons_) {
      if (buttons & 0x01)
        ESP_LOGI("usb_hidx.thrustmaster", "Button 1");
      if (buttons & 0x02)
        ESP_LOGI("usb_hidx.thrustmaster", "Button 2");
      if (buttons & 0x04)
        ESP_LOGI("usb_hidx.thrustmaster", "Button 3");
      if (buttons & 0x08)
        ESP_LOGI("usb_hidx.thrustmaster", "Button 4");
      last_buttons_ = buttons;
    }
  }

  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};

  enum DeviceType { RACING_WHEEL, FLIGHT_STICK, GAMEPAD } device_type_{GAMEPAD};

  int16_t last_steering_{0};
  int16_t last_x_{0};
  int16_t last_y_{0};
  uint8_t last_throttle_{0};
  uint8_t last_brake_{0};
  uint16_t last_buttons_{0};
};

}  // namespace usb_hidx
}  // namespace esphome
