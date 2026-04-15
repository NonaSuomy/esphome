#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"
#include "esphome/components/usb_hidx/hid_keycodes.h"
#include "esphome/components/usb_hidx/consumer_codes.h"

#ifdef USE_POCKETSSH
#include "esphome/components/pocketssh/pocketssh.h"
#endif

namespace esphome {
namespace usb_hidx {

class KeyboardDriver : public HIDDeviceDriver {
 public:
  KeyboardDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    // Don't match Xbox 360 controllers
    if (vid == 0x045E && (pid == 0x028E || pid == 0x0719))
      return false;
    // Don't match 8BitDo in Xbox mode
    if (vid == 0x2DC8 && pid == 0x310B)
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
    
    // Skip error codes (0x01 = ErrorRollOver, 0x02 = POSTFail, 0x03 = ErrorUndefined)
    bool has_error = false;
    for (int i = 2; i < 8; i++) {
      if (data[i] >= 0x01 && data[i] <= 0x03) {
        has_error = true;
        break;
      }
    }
    if (has_error)
      return;

    // Log modifier keys when they change
    static uint8_t prev_modifier = 0;
    if (data[0] != prev_modifier) {
      uint8_t changed = data[0] ^ prev_modifier;
      auto pub_mod = [&](const char *name, bool pressed) {
        ESP_LOGI("KeyboardDriver", "%s %s", name, pressed ? "pressed" : "released");
#ifdef USE_TEXT_SENSOR
        if (pressed && parent_->get_keyboard_sensor())
          parent_->get_keyboard_sensor()->publish_state(name);
#endif
      };
      if (changed & 0x01) pub_mod("Left Ctrl",    (data[0] & 0x01));
      if (changed & 0x02) pub_mod("Left Shift",   (data[0] & 0x02));
      if (changed & 0x04) pub_mod("Left Alt",     (data[0] & 0x04));
      if (changed & 0x08) pub_mod("Win",           (data[0] & 0x08));
      if (changed & 0x10) pub_mod("Right Ctrl",   (data[0] & 0x10));
      if (changed & 0x20) pub_mod("Right Shift",  (data[0] & 0x20));
      if (changed & 0x40) pub_mod("Right Alt",    (data[0] & 0x40));
      if (changed & 0x80) pub_mod("Right Win",    (data[0] & 0x80));
      prev_modifier = data[0];
    }

    // Update key binary sensors
#ifdef USE_BINARY_SENSOR
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
#endif

    // Publish text sensor for new key presses AND handle key repeat
    bool shift = (data[0] & 0x22) != 0;
    bool win_key = (data[0] & 0x08) != 0;   // Left GUI/Windows key
    bool ctrl_key = (data[0] & 0x11) != 0;  // Left or Right Ctrl key
    
    uint32_t now = esphome::millis();

    for (int i = 2; i < 8; i++) {
      if (data[i] != 0) {
        bool was_pressed = false;
        bool should_repeat = false;
        
        for (int j = 0; j < 6; j++) {
          if (prev_keys_[j] == data[i]) {
            was_pressed = true;
            // Check if key should repeat
            if (now - last_key_time_ > 500 && repeat_count_ == 0) {
              should_repeat = true;
              repeat_count_ = 1;
            } else if (repeat_count_ > 0 && now - last_key_time_ > 50) {
              should_repeat = true;
            }
            break;
          }
        }
        
        if (!was_pressed) {
          repeat_count_ = 0;
          last_key_time_ = now;
        } else {
          continue;  // Skip - key already held, no repeat
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
#ifdef USE_TEXT_SENSOR
              if (parent_->get_keyboard_sensor()) {
                parent_->get_keyboard_sensor()->publish_state(combo_name);
              }
#endif
              memcpy(prev_keys_, &data[2], 6);
              return;
            }
          } else if (win_key && ctrl_key && data[i] == 0x2A) {
            // Ctrl+Win+Backspace = F2
            ESP_LOGI("KeyboardDriver", "Media key: Task View (Ctrl+Win+Backspace)");
#ifdef USE_TEXT_SENSOR
            if (parent_->get_keyboard_sensor()) {
              parent_->get_keyboard_sensor()->publish_state("Task View (Ctrl+Win+Backspace)");
            }
#endif
            memcpy(prev_keys_, &data[2], 6);
            return;
          }

          // Handle lock keys
          if (data[i] == 0x39) {  // Caps Lock
            caps_lock_state_ = !caps_lock_state_;
            ESP_LOGI("KeyboardDriver", "Caps Lock: %s", caps_lock_state_ ? "ON" : "OFF");
            update_keyboard_leds(device);
#ifdef USE_TEXT_SENSOR
            if (parent_->get_keyboard_sensor())
              parent_->get_keyboard_sensor()->publish_state(caps_lock_state_ ? "Caps Lock ON" : "Caps Lock OFF");
#endif
          } else if (data[i] == 0x53) {  // Num Lock
            num_lock_state_ = !num_lock_state_;
            ESP_LOGI("KeyboardDriver", "Num Lock: %s", num_lock_state_ ? "ON" : "OFF");
            update_keyboard_leds(device);
#ifdef USE_TEXT_SENSOR
            if (parent_->get_keyboard_sensor())
              parent_->get_keyboard_sensor()->publish_state(num_lock_state_ ? "Num Lock ON" : "Num Lock OFF");
#endif
          } else if (data[i] == 0x47) {  // Scroll Lock
            scroll_lock_state_ = !scroll_lock_state_;
            ESP_LOGI("KeyboardDriver", "Scroll Lock: %s", scroll_lock_state_ ? "ON" : "OFF");
            update_keyboard_leds(device);
#ifdef USE_TEXT_SENSOR
            if (parent_->get_keyboard_sensor())
              parent_->get_keyboard_sensor()->publish_state(scroll_lock_state_ ? "Scroll Lock ON" : "Scroll Lock OFF");
#endif
          } else {
            // Convert to ASCII and build string
            char ascii = hid_to_ascii(data[i], shift);
            
            // Handle Ctrl key combinations
            if (ctrl_key && ascii >= 'a' && ascii <= 'z') {
              // Ctrl+letter = control code (Ctrl+A=0x01, Ctrl+K=0x0B, etc.)
              ascii = ascii - 'a' + 1;
            } else if (ctrl_key && ascii >= 'A' && ascii <= 'Z') {
              // Ctrl+Shift+letter = control code
              ascii = ascii - 'A' + 1;
            }
            
            if (ascii != 0) {
              ESP_LOGI("KeyboardDriver", "ASCII key: 0x%02X ('%c')", ascii, (ascii >= 32 && ascii <= 126) ? ascii : '?');
#ifdef USE_TEXT_SENSOR
              if (parent_->get_keyboard_sensor()) {
                const char *name = nullptr;
                if      (ascii == 0x08) name = "Backspace";
                else if (ascii == 0x09) name = "Tab";
                else if (ascii == 0x0A) name = "Enter";
                else if (ascii == 0x1B) name = "Escape";
                if (name) {
                  parent_->get_keyboard_sensor()->publish_state(name);
                } else {
                  char buf[2] = {ascii, 0};
                  parent_->get_keyboard_sensor()->publish_state(buf);
                }
              }
#endif
              // Also send to PocketSSH if available
#ifdef USE_POCKETSSH
              if (esphome::pocketssh::global_pocketssh) {
                esphome::pocketssh::global_pocketssh->handle_key(ascii);
              }
#endif
            } else {
              // Non-ASCII key - publish named string
              ESP_LOGD("KeyboardDriver", "Non-ASCII key: 0x%02X", data[i]);
              const char *key_name = nullptr;
              switch (data[i]) {
                case 0x29: key_name = "Escape";     break;
                case 0x3A: key_name = "F1";         break;
                case 0x3B: key_name = "F2";         break;
                case 0x3C: key_name = "F3";         break;
                case 0x3D: key_name = "F4";         break;
                case 0x3E: key_name = "F5";         break;
                case 0x3F: key_name = "F6";         break;
                case 0x40: key_name = "F7";         break;
                case 0x41: key_name = "F8";         break;
                case 0x42: key_name = "F9";         break;
                case 0x43: key_name = "F10";        break;
                case 0x44: key_name = "F11";        break;
                case 0x45: key_name = "F12";        break;
                case 0x46: key_name = "Print Screen"; break;
                case 0x47: key_name = "Scroll Lock"; break;
                case 0x48: key_name = "Pause";      break;
                case 0x49: key_name = "Insert";     break;
                case 0x4A: key_name = "Home";       break;
                case 0x4B: key_name = "Page Up";    break;
                case 0x4C: key_name = "Delete";     break;
                case 0x4D: key_name = "End";        break;
                case 0x4E: key_name = "Page Down";  break;
                case 0x4F: key_name = "Right";      break;
                case 0x50: key_name = "Left";       break;
                case 0x51: key_name = "Down";       break;
                case 0x52: key_name = "Up";         break;
                case 0x65: key_name = "Menu";       break;
              }
#ifdef USE_TEXT_SENSOR
              if (key_name && parent_->get_keyboard_sensor())
                parent_->get_keyboard_sensor()->publish_state(key_name);
#endif
#ifdef USE_POCKETSSH
              if (esphome::pocketssh::global_pocketssh)
                esphome::pocketssh::global_pocketssh->handle_key(data[i]);
#endif
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
  bool caps_lock_state_{false};
  bool num_lock_state_{false};
  bool scroll_lock_state_{false};
  uint32_t last_key_time_{0};
  int repeat_count_{0};

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

      // Both X and Y are signed 16-bit relative deltas
      // X = int16_t(data[3] | data[4]<<8)
      // Y = int16_t(data[4] | data[5]<<8) -- but X only uses low byte, Y uses bytes 4+5
      // From raw capture: data[3]=X low byte, data[4]+data[5]=Y as int16_t LE
      int16_t x_delta = (int8_t)data[3];
      int16_t y_delta = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));

      static uint8_t last_buttons = 0;

      if (buttons != last_buttons) {
        if ((buttons & 0x01) && !(last_buttons & 0x01)) {
          ESP_LOGI("KeyboardDriver", "Touchpad: Left Click at X=%d Y=%d", x_delta, y_delta);
          if (parent_->get_mouse_left_sensor()) parent_->get_mouse_left_sensor()->publish_state(true);
        }
        if (!(buttons & 0x01) && (last_buttons & 0x01)) {
          ESP_LOGI("KeyboardDriver", "Touchpad: Left Release");
          if (parent_->get_mouse_left_sensor()) parent_->get_mouse_left_sensor()->publish_state(false);
        }
        if ((buttons & 0x02) && !(last_buttons & 0x02)) {
          ESP_LOGI("KeyboardDriver", "Touchpad: Right Click at X=%d Y=%d", x_delta, y_delta);
          if (parent_->get_mouse_right_sensor()) parent_->get_mouse_right_sensor()->publish_state(true);
        }
        if (!(buttons & 0x02) && (last_buttons & 0x02)) {
          ESP_LOGI("KeyboardDriver", "Touchpad: Right Release");
          if (parent_->get_mouse_right_sensor()) parent_->get_mouse_right_sensor()->publish_state(false);
        }
        last_buttons = buttons;
      }

      if (x_delta != 0 || y_delta != 0) {
        if (x_delta != 0 && parent_->get_mouse_x_sensor())
          parent_->get_mouse_x_sensor()->publish_state(x_delta / 3.0f);
        if (y_delta != 0 && parent_->get_mouse_y_sensor())
          parent_->get_mouse_y_sensor()->publish_state(y_delta / 10.0f);
        ESP_LOGI("KeyboardDriver", "Touchpad: dx=%d dy=%d", x_delta, y_delta);
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
          if (parent_->get_mouse_x_sensor()) parent_->get_mouse_x_sensor()->publish_state(x_delta);
          if (parent_->get_mouse_y_sensor()) parent_->get_mouse_y_sensor()->publish_state(y_delta);
        }
        return;
      }

      // Log all non-zero reports for scroll/gesture debugging
      if (byte1 != 0 || byte2 != 0 || byte3 != 0 || byte5 != 0) {
        ESP_LOGI("KeyboardDriver", "Media 0x01: [%02X %02X %02X %02X %02X %02X %02X %02X]",
                 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
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
#ifdef USE_TEXT_SENSOR
    if (parent_->get_keyboard_sensor()) {
      parent_->get_keyboard_sensor()->publish_state(key_name);
    }
#endif
  }
};

}  // namespace usb_hidx
}  // namespace esphome
