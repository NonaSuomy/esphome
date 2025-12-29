#pragma once

#include "../hid_device_driver.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace usb_hidx {

struct KeyboardReport {
  uint8_t modifier;
  uint8_t reserved;
  uint8_t keycode[6];
} __attribute__((packed));

class KeyboardDevice : public Component, public HIDDeviceDriver {
 public:
  void setup() override;
  void dump_config() override;

  bool match_device(uint16_t vid, uint16_t pid, uint8_t protocol) override;
  void setup_device(HIDDevice *device, usb_host_client_handle_t client) override;
  void process_report(const uint8_t *data, size_t len) override;
  void device_removed() override;

  void set_text_sensor(text_sensor::TextSensor *sensor) { text_sensor_ = sensor; }

 protected:
  text_sensor::TextSensor *text_sensor_{nullptr};
  uint8_t prev_keys_[6]{0};

  const char *get_key_name(uint8_t keycode);
  static void transfer_callback(usb_transfer_t *transfer);
};

}  // namespace usb_hidx
}  // namespace esphome
