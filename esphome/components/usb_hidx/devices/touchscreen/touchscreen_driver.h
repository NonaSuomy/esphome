#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class TouchscreenDriver : public HIDDeviceDriver {
 public:
  TouchscreenDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Generic multitouch devices (usage page 0x0D)
    // Accept any device that reports as digitizer/touchscreen
    // Common manufacturers:

    // 3M touchscreens
    if (vid == 0x0596)
      return true;

    // eGalax/EETI touchscreens
    if (vid == 0x0EEF)
      return true;

    // Elo TouchSystems
    if (vid == 0x04E7)
      return true;

    // ILITEK touchscreens
    if (vid == 0x222A)
      return true;

    // Atmel maXTouch
    if (vid == 0x03EB && pid == 0x211C)
      return true;

    // Synaptics touchscreens
    if (vid == 0x06CB)
      return true;

    // Goodix touchscreens
    if (vid == 0x27C6)
      return true;

    // Generic HID touchscreen (will match based on usage page in descriptor)
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.touch", "USB Touchscreen detected");
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_)
      device_ = device;
    if (len < 5)
      return;

    // Generic multitouch report format
    // Most touchscreens follow similar patterns

    uint8_t report_id = data[0];

    // Single touch report (most common)
    if (len >= 7) {
      // Touch state (byte 1, bit 0 = tip switch/contact)
      bool touching = data[1] & 0x01;

      // X coordinate (16-bit, bytes 2-3)
      uint16_t x = (data[3] << 8) | data[2];

      // Y coordinate (16-bit, bytes 4-5)
      uint16_t y = (data[5] << 8) | data[4];

      // Contact ID (if multitouch)
      uint8_t contact_id = len > 6 ? data[6] : 0;

      // Log touch events
      if (touching && !last_touching_) {
        ESP_LOGI("usb_hidx.touch", "Touch Down: X=%d Y=%d", x, y);
        last_x_ = x;
        last_y_ = y;
      } else if (!touching && last_touching_) {
        ESP_LOGI("usb_hidx.touch", "Touch Up");
      } else if (touching) {
        // Log movement if significant
        if (abs((int) x - (int) last_x_) > 10 || abs((int) y - (int) last_y_) > 10) {
          ESP_LOGV("usb_hidx.touch", "Touch Move: X=%d Y=%d", x, y);
          last_x_ = x;
          last_y_ = y;
        }
      }

      last_touching_ = touching;
    }

    // Multitouch report (multiple contacts)
    if (len >= 10 && data[1] > 1) {
      uint8_t contact_count = data[1];
      ESP_LOGV("usb_hidx.touch", "Multitouch: %d contacts", contact_count);

      // Parse each contact (typically 5-7 bytes per contact)
      for (int i = 0; i < contact_count && (2 + i * 7) < len; i++) {
        int offset = 2 + i * 7;
        bool touching = data[offset] & 0x01;
        uint16_t x = (data[offset + 2] << 8) | data[offset + 1];
        uint16_t y = (data[offset + 4] << 8) | data[offset + 3];
        uint8_t contact_id = data[offset + 5];

        if (touching) {
          ESP_LOGV("usb_hidx.touch", "Contact %d: X=%d Y=%d", contact_id, x, y);
        }
      }
    }
  }

  const char *get_name() override { return "Touchscreen"; }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};
  bool last_touching_{false};
  uint16_t last_x_{0};
  uint16_t last_y_{0};
};

}  // namespace usb_hidx
}  // namespace esphome
