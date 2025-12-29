#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class PlayStationDriver : public HIDDeviceDriver {
 public:
  PlayStationDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // PS4 DualShock 4
    if (vid == 0x054C && (pid == 0x05C4 || pid == 0x09CC)) {
      controller_type_ = PS4;
      return true;
    }
    // PS5 DualSense
    if (vid == 0x054C && (pid == 0x0CE6 || pid == 0x0DF2)) {
      controller_type_ = PS5;
      return true;
    }
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    if (controller_type_ == PS4) {
      ESP_LOGI("usb_hidx.ps", "PS4 DualShock 4 detected");
    } else if (controller_type_ == PS5) {
      ESP_LOGI("usb_hidx.ps", "PS5 DualSense detected");
    }
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;

    if (controller_type_ == PS4) {
      process_ps4_report(data, len);
    } else if (controller_type_ == PS5) {
      process_ps5_report(data, len);
    }
  }

  const char *get_name() override { return controller_type_ == PS4 ? "PS4" : "PS5"; }

 protected:
  void process_ps4_report(const uint8_t *data, size_t len) {
    if (len < 10)
      return;

    // PS4 report format: buttons at offset 5-7
    uint8_t btn1 = data[5];
    uint8_t btn2 = data[6];
    uint8_t btn3 = data[7];

    // D-Pad (lower 4 bits of btn1)
    uint8_t dpad = btn1 & 0x0F;

    // Face buttons (upper 4 bits of btn1)
    bool square = btn1 & 0x10;
    bool cross = btn1 & 0x20;
    bool circle = btn1 & 0x40;
    bool triangle = btn1 & 0x80;

    // Shoulder buttons (btn2)
    bool l1 = btn2 & 0x01;
    bool r1 = btn2 & 0x02;
    bool l2 = btn2 & 0x04;
    bool r2 = btn2 & 0x08;
    bool share = btn2 & 0x10;
    bool options = btn2 & 0x20;
    bool l3 = btn2 & 0x40;
    bool r3 = btn2 & 0x80;

    // Special buttons (btn3)
    bool ps = btn3 & 0x01;
    bool touchpad = btn3 & 0x02;

    // Analog sticks (1 byte each, 0-255)
    uint8_t lx = data[1];
    uint8_t ly = data[2];
    uint8_t rx = data[3];
    uint8_t ry = data[4];

    // Log button presses
    if (cross && !(last_btn1_ & 0x20))
      ESP_LOGI("usb_hidx.ps", "Cross");
    if (circle && !(last_btn1_ & 0x40))
      ESP_LOGI("usb_hidx.ps", "Circle");
    if (square && !(last_btn1_ & 0x10))
      ESP_LOGI("usb_hidx.ps", "Square");
    if (triangle && !(last_btn1_ & 0x80))
      ESP_LOGI("usb_hidx.ps", "Triangle");
    if (ps && !(last_btn3_ & 0x01))
      ESP_LOGI("usb_hidx.ps", "PS Button");

    last_btn1_ = btn1;
    last_btn2_ = btn2;
    last_btn3_ = btn3;
  }

  void process_ps5_report(const uint8_t *data, size_t len) {
    if (len < 11)
      return;

    // PS5 report format similar to PS4 but with offset
    uint8_t btn1 = data[8];
    uint8_t btn2 = data[9];
    uint8_t btn3 = data[10];

    // Face buttons
    bool square = btn1 & 0x10;
    bool cross = btn1 & 0x20;
    bool circle = btn1 & 0x40;
    bool triangle = btn1 & 0x80;

    // Shoulder buttons
    bool l1 = btn2 & 0x01;
    bool r1 = btn2 & 0x02;
    bool create = btn2 & 0x10;
    bool options = btn2 & 0x20;

    // Special buttons
    bool ps = btn3 & 0x01;

    // Analog sticks
    uint8_t lx = data[1];
    uint8_t ly = data[2];
    uint8_t rx = data[3];
    uint8_t ry = data[4];

    // Log button presses
    if (cross && !(last_btn1_ & 0x20))
      ESP_LOGI("usb_hidx.ps", "Cross");
    if (circle && !(last_btn1_ & 0x40))
      ESP_LOGI("usb_hidx.ps", "Circle");
    if (square && !(last_btn1_ & 0x10))
      ESP_LOGI("usb_hidx.ps", "Square");
    if (triangle && !(last_btn1_ & 0x80))
      ESP_LOGI("usb_hidx.ps", "Triangle");
    if (ps && !(last_btn3_ & 0x01))
      ESP_LOGI("usb_hidx.ps", "PS Button");

    last_btn1_ = btn1;
    last_btn2_ = btn2;
    last_btn3_ = btn3;
  }

  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};

  enum ControllerType { PS4, PS5 } controller_type_{PS4};

  uint8_t last_btn1_{0};
  uint8_t last_btn2_{0};
  uint8_t last_btn3_{0};
};

}  // namespace usb_hidx
}  // namespace esphome
