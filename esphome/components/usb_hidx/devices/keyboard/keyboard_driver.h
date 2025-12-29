#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"
#include "esphome/components/usb_hidx/hid_keycodes.h"
#include "esphome/components/usb_hidx/consumer_codes.h"

namespace esphome {
namespace usb_hidx {

class KeyboardDriver : public HIDDeviceDriver {
 public:
  KeyboardDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Don't match Xbox 360 controller
    if (vid == 0x045E && pid == 0x028E)
      return false;
    return protocol == 0x01;  // HID keyboard protocol
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    // Check if this is marked as a media report (0xFF prefix from transfer callback)
    if (len >= 2 && data[0] == 0xFF) {
      // Media report - skip the marker byte
      process_media_report(&data[1], len - 1);
      return;
    }

    // Standard keyboard report (8 bytes)
    if (len < 8)
      return;

    // Log modifier keys when they change
    static uint8_t prev_modifier = 0;
    if (data[0] != prev_modifier) {
      uint8_t changed = data[0] ^ prev_modifier;
      if (changed & 0x01)
        ESP_LOGI("KeyboardDriver", "Left Ctrl %s", (data[0] & 0x01) ? "pressed" : "released");
      if (changed & 0x02)
        ESP_LOGI("KeyboardDriver", "Left Shift %s", (data[0] & 0x02) ? "pressed" : "released");
      if (changed & 0x04)
        ESP_LOGI("KeyboardDriver", "Left Alt %s", (data[0] & 0x04) ? "pressed" : "released");
      if (changed & 0x08)
        ESP_LOGI("KeyboardDriver", "Left GUI (Windows) %s", (data[0] & 0x08) ? "pressed" : "released");
      if (changed & 0x10)
        ESP_LOGI("KeyboardDriver", "Right Ctrl %s", (data[0] & 0x10) ? "pressed" : "released");
      if (changed & 0x20)
        ESP_LOGI("KeyboardDriver", "Right Shift %s", (data[0] & 0x20) ? "pressed" : "released");
      if (changed & 0x40)
        ESP_LOGI("KeyboardDriver", "Right Alt %s", (data[0] & 0x40) ? "pressed" : "released");
      if (changed & 0x80)
        ESP_LOGI("KeyboardDriver", "Right GUI (Windows) %s", (data[0] & 0x80) ? "pressed" : "released");
      prev_modifier = data[0];
    }

    // Update key binary sensors
    for (auto &kv : parent_->get_keyboard_key_sensors()) {
      uint8_t target_key = kv.first;
      bool is_pressed = false;
      for (int i = 2; i < 8; i++) {
        if (data[i] == target_key) {
          is_pressed = true;
          break;
        }
      }
      kv.second->publish_state(is_pressed);
    }

    // Publish text sensor for new key presses
    bool shift = (data[0] & 0x22) != 0;
    bool win_key = (data[0] & 0x08) != 0;   // Left GUI/Windows key
    bool ctrl_key = (data[0] & 0x01) != 0;  // Left Ctrl key

    for (int i = 2; i < 8; i++) {
      if (data[i] != 0) {
        bool was_pressed = false;
        for (int j = 0; j < 6; j++) {
          if (prev_keys_[j] == data[i]) {
            was_pressed = true;
            break;
          }
        }
        if (!was_pressed) {
          // Check for Windows key combinations (Logitech K400r media keys F1-F6)
          if (win_key && !ctrl_key) {
            const char *combo_name = nullptr;
            switch (data[i]) {
              case 0x07:
                combo_name = "Show Desktop (Win+D)";
                break;  // F1/PC button
              case 0x0B:
                combo_name = "Dictation (Win+H)";
                break;  // F4
              case 0x0E:
                combo_name = "Connect (Win+K)";
                break;  // F5
              case 0x0C:
                combo_name = "Settings (Win+I)";
                break;  // F6
            }
            if (combo_name) {
              ESP_LOGI("KeyboardDriver", "Media key: %s", combo_name);
              if (parent_->get_keyboard_sensor()) {
                parent_->get_keyboard_sensor()->publish_state(combo_name);
              }
              memcpy(prev_keys_, &data[2], 6);
              return;
            }
          } else if (win_key && ctrl_key && data[i] == 0x2A) {
            // Ctrl+Win+Backspace = F2
            ESP_LOGI("KeyboardDriver", "Media key: Task View (Ctrl+Win+Backspace)");
            if (parent_->get_keyboard_sensor()) {
              parent_->get_keyboard_sensor()->publish_state("Task View (Ctrl+Win+Backspace)");
            }
            memcpy(prev_keys_, &data[2], 6);
            return;
          }

          // Handle lock keys
          if (data[i] == 0x39) {  // Caps Lock
            caps_lock_state_ = !caps_lock_state_;
            ESP_LOGI("KeyboardDriver", "Caps Lock: %s", caps_lock_state_ ? "ON" : "OFF");
            update_keyboard_leds(device);
          } else if (data[i] == 0x53) {  // Num Lock
            num_lock_state_ = !num_lock_state_;
            ESP_LOGI("KeyboardDriver", "Num Lock: %s", num_lock_state_ ? "ON" : "OFF");
            update_keyboard_leds(device);
          } else if (data[i] == 0x47) {  // Scroll Lock
            scroll_lock_state_ = !scroll_lock_state_;
            ESP_LOGI("KeyboardDriver", "Scroll Lock: %s", scroll_lock_state_ ? "ON" : "OFF");
            update_keyboard_leds(device);
          } else {
            // Convert to ASCII and build string
            char ascii = hid_to_ascii(data[i], shift);
            if (ascii != 0) {
              if (ascii == '\b') {
                if (!keyboard_buffer_.empty()) {
                  keyboard_buffer_.pop_back();
                }
              } else if (ascii == '\n') {
                ESP_LOGI("KeyboardDriver", "Enter pressed, clearing buffer");
                keyboard_buffer_.clear();
              } else {
                keyboard_buffer_ += ascii;
              }
              if (parent_->get_keyboard_sensor()) {
                parent_->get_keyboard_sensor()->publish_state(keyboard_buffer_);
              }
            } else {
              // Non-ASCII key, publish key name
              const char *key_name = hid_keycode_to_name(data[i]);
              ESP_LOGI("KeyboardDriver", "Key: %s (0x%02X)", key_name, data[i]);
            }
          }
        }
      }
    }
    memcpy(prev_keys_, &data[2], 6);
  }

  const char *get_name() override { return "Keyboard"; }

 protected:
  USBHIDXComponent *parent_;
  uint8_t prev_keys_[6]{0};
  std::string keyboard_buffer_;
  bool caps_lock_state_{false};
  bool num_lock_state_{false};
  bool scroll_lock_state_{false};

  char hid_to_ascii(uint8_t keycode, bool shift) {
    if (keycode >= 0x04 && keycode <= 0x1D) {
      char c = 'a' + (keycode - 0x04);
      bool make_uppercase = shift ^ caps_lock_state_;
      return make_uppercase ? (c - 32) : c;
    } else if (keycode >= 0x1E && keycode <= 0x27) {
      const char numbers[] = "1234567890";
#if defined(KEYBOARD_LAYOUT_UK)
      const char shifted[] = "!\"£$%^&*()";
#elif defined(KEYBOARD_LAYOUT_DE)
      const char shifted[] = "!\"§$%&/()=";
#elif defined(KEYBOARD_LAYOUT_FR)
      const char shifted[] = "&é\"'(-è_çà";
#elif defined(KEYBOARD_LAYOUT_ES)
      const char shifted[] = "!\"·$%&/()=";
#else  // US layout
      const char shifted[] = "!@#$%^&*()";
#endif
      return shift ? shifted[keycode - 0x1E] : numbers[keycode - 0x1E];
    }
    switch (keycode) {
      case 0x2C:
        return ' ';
      case 0x28:
        return '\n';
      case 0x2A:
        return '\b';
#if defined(KEYBOARD_LAYOUT_UK)
      case 0x2D:
        return shift ? '_' : '-';
      case 0x2E:
        return shift ? '+' : '=';
      case 0x2F:
        return shift ? '{' : '[';
      case 0x30:
        return shift ? '}' : ']';
      case 0x31:
        return shift ? '~' : '#';
      case 0x33:
        return shift ? ':' : ';';
      case 0x34:
        return shift ? '@' : '\'';
      case 0x35:
        return shift ? '¬' : '`';
      case 0x36:
        return shift ? '<' : ',';
      case 0x37:
        return shift ? '>' : '.';
      case 0x38:
        return shift ? '?' : '/';
      case 0x64:
        return shift ? '|' : '\\';
#elif defined(KEYBOARD_LAYOUT_DE)
      case 0x2D:
        return shift ? '?' : 'ß';
      case 0x2E:
        return shift ? '`' : '´';
      case 0x2F:
        return shift ? '?' : 'ü';
      case 0x30:
        return shift ? '*' : '+';
      case 0x31:
        return shift ? '\'' : '#';
      case 0x33:
        return shift ? ';' : 'ö';
      case 0x34:
        return shift ? ':' : 'ä';
      case 0x35:
        return shift ? '°' : '^';
      case 0x36:
        return shift ? ';' : ',';
      case 0x37:
        return shift ? ':' : '.';
      case 0x38:
        return shift ? '_' : '-';
      case 0x64:
        return shift ? '>' : '<';
#else  // US, FR, ES layouts (similar to US)
      case 0x2D:
        return shift ? '_' : '-';
      case 0x2E:
        return shift ? '+' : '=';
      case 0x2F:
        return shift ? '{' : '[';
      case 0x30:
        return shift ? '}' : ']';
      case 0x31:
        return shift ? '|' : '\\';
      case 0x33:
        return shift ? ':' : ';';
      case 0x34:
        return shift ? '"' : '\'';
      case 0x35:
        return shift ? '~' : '`';
      case 0x36:
        return shift ? '<' : ',';
      case 0x37:
        return shift ? '>' : '.';
      case 0x38:
        return shift ? '?' : '/';
#endif
      case 0x2B:
        return '\t';
      case 0x59:
        return num_lock_state_ ? '1' : 0;
      case 0x5A:
        return num_lock_state_ ? '2' : 0;
      case 0x5B:
        return num_lock_state_ ? '3' : 0;
      case 0x5C:
        return num_lock_state_ ? '4' : 0;
      case 0x5D:
        return num_lock_state_ ? '5' : 0;
      case 0x5E:
        return num_lock_state_ ? '6' : 0;
      case 0x5F:
        return num_lock_state_ ? '7' : 0;
      case 0x60:
        return num_lock_state_ ? '8' : 0;
      case 0x61:
        return num_lock_state_ ? '9' : 0;
      case 0x62:
        return num_lock_state_ ? '0' : 0;
      case 0x63:
        return num_lock_state_ ? '.' : 0;
      case 0x54:
        return '/';
      case 0x55:
        return '*';
      case 0x56:
        return '-';
      case 0x57:
        return '+';
      case 0x58:
        return '\n';
      default:
        return 0;
    }
  }

  void update_keyboard_leds(HIDDevice *device) {
    uint8_t led_report = 0;
    if (num_lock_state_)
      led_report |= 0x01;
    if (caps_lock_state_)
      led_report |= 0x02;
    if (scroll_lock_state_)
      led_report |= 0x04;

    ESP_LOGI("KeyboardDriver", "Updating LEDs: Caps=%s Num=%s Scroll=%s", caps_lock_state_ ? "ON" : "OFF",
             num_lock_state_ ? "ON" : "OFF", scroll_lock_state_ ? "ON" : "OFF");

    parent_->update_keyboard_leds(device, led_report);
  }

  void process_media_report(const uint8_t *data, size_t len) {
    // Report ID 0x02: Touchpad (Logitech K400r)
    if (data[0] == 0x02 && len >= 8) {
      uint8_t buttons = data[1];
      uint16_t x_raw = (uint16_t) data[2] | ((uint16_t) data[3] << 8);
      uint16_t y_raw = (uint16_t) data[4] | ((uint16_t) data[5] << 8);

      uint16_t x_coord = x_raw & 0x0FFF;
      uint16_t y_coord = y_raw & 0x0FFF;

      static uint8_t last_buttons = 0;
      static uint16_t last_x = 0;
      static uint16_t last_y = 0;

      if (buttons != last_buttons) {
        if ((buttons & 0x01) && !(last_buttons & 0x01)) {
          ESP_LOGI("KeyboardDriver", "Touchpad: Left Click at X=%d Y=%d", x_coord, y_coord);
        }
        if (!(buttons & 0x01) && (last_buttons & 0x01)) {
          ESP_LOGI("KeyboardDriver", "Touchpad: Left Release");
        }
        if ((buttons & 0x02) && !(last_buttons & 0x02)) {
          ESP_LOGI("KeyboardDriver", "Touchpad: Right Click at X=%d Y=%d", x_coord, y_coord);
        }
        if (!(buttons & 0x02) && (last_buttons & 0x02)) {
          ESP_LOGI("KeyboardDriver", "Touchpad: Right Release");
        }
        last_buttons = buttons;
      }

      if ((x_coord != 0 || y_coord != 0) &&
          (abs((int) x_coord - (int) last_x) > 200 || abs((int) y_coord - (int) last_y) > 200)) {
        ESP_LOGI("KeyboardDriver", "Touchpad: Position X=%d Y=%d", x_coord, y_coord);
        last_x = x_coord;
        last_y = y_coord;
      }
      return;
    }

    // Report ID 0x03: Consumer control (5 bytes) - single byte format
    if (data[0] == 0x03 && len >= 5) {
      uint8_t byte1 = data[1];
      uint8_t byte2 = data[2];

      static uint8_t prev_byte1 = 0;
      static uint8_t prev_byte2 = 0;

      // Check for key press in byte1 (changed from 0 to non-zero)
      if (byte1 != 0 && prev_byte1 == 0) {
        const char *key_name = consumer_code_to_name(byte1);
        if (key_name) {
          publish_media_key(key_name);
        } else {
          ESP_LOGW("KeyboardDriver", "Unknown consumer code in Report 0x03 byte1: 0x%02X", byte1);
        }
      }

      // Check for key press in byte2 (changed from 0 to non-zero)
      // Skip 0x02 as it appears to be a modifier/flag byte
      if (byte2 != 0 && byte2 != 0x02 && prev_byte2 == 0) {
        const char *key_name = consumer_code_to_name(byte2);
        if (key_name) {
          publish_media_key(key_name);
        } else {
          ESP_LOGW("KeyboardDriver", "Unknown consumer code in Report 0x03 byte2: 0x%02X", byte2);
        }
      }

      prev_byte1 = byte1;
      prev_byte2 = byte2;
      return;
    }

    if (data[0] == 0x01 && len >= 8) {
      uint8_t byte1 = data[1];
      uint8_t byte2 = data[2];
      uint8_t byte3 = data[3];
      uint8_t byte5 = data[5];

      // Check for touchpad movement (byte5 == 0x01 indicates movement mode)
      if (byte5 == 0x01 && byte2 == 0x00) {
        int8_t x_delta = (int8_t) byte1;
        int8_t y_delta = (int8_t) byte2;
        if (x_delta != 0 || y_delta != 0) {
          ESP_LOGI("KeyboardDriver", "Touchpad: Movement delta X=%d Y=%d", x_delta, y_delta);
        }
        return;
      }

      // Log all non-zero reports for debugging
      if (byte1 != 0 || byte2 != 0 || byte3 != 0 || byte5 != 0) {
        ESP_LOGD("KeyboardDriver", "Media: [%02X %02X %02X %02X %02X %02X %02X %02X]", data[0], data[1], data[2],
                 data[3], data[4], data[5], data[6], data[7]);
      }

      const char *key_name = nullptr;

      // Check for byte3 keys (numpad special keys)
      if (!key_name && byte3 != 0) {
        switch (byte3) {
          case 0x67:
            key_name = "Numpad Equals";
            break;
          case 0xB6:
            key_name = "Numpad [";
            break;
          case 0xB7:
            key_name = "Numpad ]";
            break;
        }
      }

      // Check for Zoom/Favourites/F-Lock keys FIRST (byte1+byte2 combinations)
      if (byte2 != 0x00) {
        if (byte1 == 0x82 && byte2 == 0x01) {
          key_name = "Favourites";
        } else if (byte1 == 0x2D && byte2 == 0x02) {
          key_name = "Zoom In";
        } else if (byte1 == 0x2E && byte2 == 0x02) {
          key_name = "Zoom Out";
        } else if (byte1 == 0x1A && byte2 == 0x02) {
          key_name = "Undo";
        } else if (byte1 == 0x79 && byte2 == 0x02) {
          key_name = "Redo";
        } else if (byte1 == 0x01 && byte2 == 0x02) {
          key_name = "New";
        } else if (byte1 == 0x02 && byte2 == 0x02) {
          key_name = "Open";
        } else if (byte1 == 0x03 && byte2 == 0x02) {
          key_name = "Close";
        } else if (byte1 == 0x89 && byte2 == 0x02) {
          key_name = "Reply";
        } else if (byte1 == 0x8B && byte2 == 0x02) {
          key_name = "Forward Mail";
        } else if (byte1 == 0x8C && byte2 == 0x02) {
          key_name = "Send Mail";
        } else if (byte1 == 0xAB && byte2 == 0x01) {
          key_name = "Spell";
        } else if (byte1 == 0x07 && byte2 == 0x02) {
          key_name = "Save";
        } else if (byte1 == 0x08 && byte2 == 0x02) {
          key_name = "Print";
        }
      }

      // Check for media keys (byte1 only)
      if (!key_name && byte1 != 0) {
        switch (byte1) {
          case 0x95:
            key_name = "Help";
            break;
          case 0xE9:
            key_name = "Volume Up";
            break;
          case 0xEA:
            key_name = "Volume Down";
            break;
          case 0xE2:
            key_name = "Mute";
            break;
          case 0xCD:
            key_name = "Play/Pause";
            break;
          case 0xB5:
            key_name = "Next Track";
            break;
          case 0xB6:
            key_name = "Previous Track";
            break;
          case 0xB7:
            key_name = "Stop";
            break;
          case 0x8A:
            key_name = "Mail";
            break;
          case 0x92:
            key_name = "Calculator";
            break;
          case 0x94:
            key_name = "My Computer";
            break;
          case 0x23:
            key_name = "WWW Home";
            break;
          case 0x21:
            key_name = "WWW Search";
            break;
          case 0x24:
            key_name = "WWW Back";
            break;
          case 0x25:
            key_name = "WWW Forward";
            break;
        }
      }

      // Check for special function keys (byte5 based)
      if (!key_name && byte5 != 0x01 && byte5 != 0x00) {
        switch (byte5) {
          case 0x04:
            key_name = "Custom Key 1";
            break;
          case 0x08:
            key_name = "Custom Key 2";
            break;
          case 0x10:
            key_name = "Custom Key 3";
            break;
          case 0x20:
            key_name = "Custom Key 4";
            break;
          case 0x40:
            key_name = "Custom Key 5";
            break;
        }
      }

      if (key_name) {
        publish_media_key(key_name);
      }
      return;
    }

    if (data[0] == 0x01 && len >= 2) {
      // Report ID 0x01: Single-byte consumer codes (Microsoft keyboards)
      static uint8_t prev_byte1 = 0;
      if (data[1] != prev_byte1 && data[1] != 0) {
        const char *key_name = consumer_code_to_name(data[1]);
        if (key_name) {
          publish_media_key(key_name);
        } else {
          ESP_LOGW("KeyboardDriver", "Unknown consumer code: 0x%02X", data[1]);
        }
      }
      prev_byte1 = data[1];
    } else if (data[0] == 0x02 && len >= 2) {
      // Report ID 0x02: System control (sleep, power)
      static uint8_t prev_byte1_r2 = 0;
      if (data[1] == 0x02 && prev_byte1_r2 != 0x02) {
        ESP_LOGI("KeyboardDriver", "Sleep key detected!");
        publish_media_key("Sleep");
      }
      prev_byte1_r2 = data[1];
    } else if (data[0] == 0x01 && len >= 3) {
      // Report ID 0x01: Zoom keys
      uint8_t byte2 = data[2];
      static uint8_t prev_byte2_r1 = 0;

      if (byte2 == 0x01 && prev_byte2_r1 != 0x01)
        publish_media_key("Zoom In");
      else if (byte2 == 0xFF && prev_byte2_r1 != 0xFF)
        publish_media_key("Zoom Out");

      prev_byte2_r1 = byte2;
    }
  }

  void publish_media_key(const char *key_name) {
    ESP_LOGI("KeyboardDriver", "Media key: %s", key_name);
    if (parent_->get_keyboard_sensor()) {
      parent_->get_keyboard_sensor()->publish_state(key_name);
    }
  }
};

}  // namespace usb_hidx
}  // namespace esphome
