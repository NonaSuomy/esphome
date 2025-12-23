#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class Xbox360Driver : public HIDDeviceDriver {
 public:
  Xbox360Driver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Xbox 360 controllers (including drum kit and guitar)
    is_drum_kit_ = (vid == 0x1BAD && pid == 0x0003);
    is_guitar_ = (vid == 0x1430 && pid == 0x4748);
    return is_drum_kit_ || is_guitar_ || (vid == 0x045E && pid == 0x028E);
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (len < 20)
      return;

    // Xbox 360 format: 20 bytes, starts with 0x00 0x14
    if (data[0] != 0x00 || data[1] != 0x14)
      return;

    uint8_t dpad = data[2];
    uint8_t buttons1 = data[3];
    uint8_t buttons2 = data[4];

    const char *prefix = is_guitar_ ? "Guitar" : (is_drum_kit_ ? "Drum" : "Xbox360");

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

  const char *get_name() override { return "Xbox360"; }

 protected:
  USBHIDXComponent *parent_;
  bool is_drum_kit_{false};
  bool is_guitar_{false};
  uint8_t last_dpad_{0};
  uint8_t last_buttons1_{0};
  uint8_t last_buttons2_{0};
  int16_t last_whammy_{-32768};
};

}  // namespace usb_hidx
}  // namespace esphome
