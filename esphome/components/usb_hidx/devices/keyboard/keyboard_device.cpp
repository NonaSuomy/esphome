#include "keyboard_device.h"
#include "esphome/core/log.h"

namespace esphome {
namespace usb_hidx {

static const char *TAG = "usb_hidx.keyboard";

void KeyboardDevice::setup() { ESP_LOGI(TAG, "Keyboard device driver initialized"); }

void KeyboardDevice::dump_config() { ESP_LOGCONFIG(TAG, "USB HIDX Keyboard:"); }

bool KeyboardDevice::match_device(uint16_t vid, uint16_t pid, uint8_t protocol) {
  return protocol == 0x01;  // HID keyboard protocol
}

void KeyboardDevice::setup_device(HIDDevice *device, usb_host_client_handle_t client) {
  this->device_ = device;
  ESP_LOGI(TAG, "Keyboard device setup complete");
}

void KeyboardDevice::process_report(const uint8_t *data, size_t len) {
  if (len < sizeof(KeyboardReport)) {
    return;
  }

  const KeyboardReport *report = reinterpret_cast<const KeyboardReport *>(data);

  for (int i = 0; i < 6; i++) {
    if (report->keycode[i] != 0) {
      bool was_pressed = false;
      for (int j = 0; j < 6; j++) {
        if (prev_keys_[j] == report->keycode[i]) {
          was_pressed = true;
          break;
        }
      }

      if (!was_pressed) {
        const char *key_name = get_key_name(report->keycode[i]);
        ESP_LOGI(TAG, "Key pressed: %s (0x%02X)", key_name, report->keycode[i]);

        if (this->text_sensor_) {
          this->text_sensor_->publish_state(key_name);
        }
      }
    }
  }

  memcpy(prev_keys_, report->keycode, 6);
}

void KeyboardDevice::device_removed() {
  ESP_LOGI(TAG, "Keyboard device removed");
  this->device_ = nullptr;
}

const char *KeyboardDevice::get_key_name(uint8_t keycode) {
  if (keycode >= 0x04 && keycode <= 0x1D) {
    static const char *letters[] = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
                                    "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"};
    return letters[keycode - 0x04];
  } else if (keycode >= 0x1E && keycode <= 0x27) {
    static const char *numbers[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0"};
    return numbers[keycode - 0x1E];
  }

  switch (keycode) {
    case 0x28:
      return "Enter";
    case 0x29:
      return "Esc";
    case 0x2A:
      return "Backspace";
    case 0x2C:
      return "Space";
    default:
      return "Unknown";
  }
}

}  // namespace usb_hidx
}  // namespace esphome
