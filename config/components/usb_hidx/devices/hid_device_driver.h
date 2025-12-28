#pragma once

#include "esphome/core/component.h"
#include "../usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class HIDDeviceDriver {
 public:
  virtual ~HIDDeviceDriver() = default;

  virtual bool match_device(uint16_t vid, uint16_t pid, uint8_t protocol) = 0;
  virtual void setup_device(HIDDevice *device, usb_host_client_handle_t client) = 0;
  virtual void process_report(const uint8_t *data, size_t len) = 0;
  virtual void device_removed() = 0;

  void set_device(HIDDevice *device) { device_ = device; }
  HIDDevice *get_device() { return device_; }

 protected:
  HIDDevice *device_{nullptr};
};

}  // namespace usb_hidx
}  // namespace esphome
