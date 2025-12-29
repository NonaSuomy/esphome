#pragma once

namespace esphome {
namespace usb_hidx {

// Complete HID keyboard key code to name mapping
inline const char *hid_keycode_to_name(uint8_t keycode) {
  // Letters A-Z (0x04-0x1D)
  if (keycode >= 0x04 && keycode <= 0x1D) {
    static const char *letters[] = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
                                    "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"};
    return letters[keycode - 0x04];
  }

  // Numbers 1-0 (0x1E-0x27)
  if (keycode >= 0x1E && keycode <= 0x27) {
    static const char *numbers[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0"};
    return numbers[keycode - 0x1E];
  }

  // Function keys F1-F12 (0x3A-0x45)
  if (keycode >= 0x3A && keycode <= 0x45) {
    static const char *fkeys[] = {"F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12"};
    return fkeys[keycode - 0x3A];
  }

  // Keypad numbers 1-0 (0x59-0x62)
  if (keycode >= 0x59 && keycode <= 0x62) {
    static const char *kp_nums[] = {"KP1", "KP2", "KP3", "KP4", "KP5", "KP6", "KP7", "KP8", "KP9", "KP0"};
    return kp_nums[keycode - 0x59];
  }

  switch (keycode) {
    // Control keys
    case 0x28:
      return "Enter";
    case 0x29:
      return "Esc";
    case 0x2A:
      return "Backspace";
    case 0x2B:
      return "Tab";
    case 0x2C:
      return "Space";
    case 0x2D:
      return "Minus";
    case 0x2E:
      return "Equal";
    case 0x2F:
      return "LeftBracket";
    case 0x30:
      return "RightBracket";
    case 0x31:
      return "Backslash";
    case 0x33:
      return "Semicolon";
    case 0x34:
      return "Apostrophe";
    case 0x35:
      return "Grave";
    case 0x36:
      return "Comma";
    case 0x37:
      return "Period";
    case 0x38:
      return "Slash";
    case 0x39:
      return "CapsLock";

    // Navigation
    case 0x46:
      return "PrintScreen";
    case 0x47:
      return "ScrollLock";
    case 0x48:
      return "Pause";
    case 0x49:
      return "Insert";
    case 0x4A:
      return "Home";
    case 0x4B:
      return "PageUp";
    case 0x4C:
      return "Delete";
    case 0x4D:
      return "End";
    case 0x4E:
      return "PageDown";
    case 0x4F:
      return "Right";
    case 0x50:
      return "Left";
    case 0x51:
      return "Down";
    case 0x52:
      return "Up";

    // Keypad
    case 0x53:
      return "NumLock";
    case 0x54:
      return "KPDivide";
    case 0x55:
      return "KPMultiply";
    case 0x56:
      return "KPMinus";
    case 0x57:
      return "KPPlus";
    case 0x58:
      return "KPEnter";
    case 0x63:
      return "KPPeriod";

    // Extended function keys F13-F24
    case 0x68:
      return "F13";
    case 0x69:
      return "F14";
    case 0x6A:
      return "F15";
    case 0x6B:
      return "F16";
    case 0x6C:
      return "F17";
    case 0x6D:
      return "F18";
    case 0x6E:
      return "F19";
    case 0x6F:
      return "F20";
    case 0x70:
      return "F21";
    case 0x71:
      return "F22";
    case 0x72:
      return "F23";
    case 0x73:
      return "F24";

    // Media keys (some keyboards send through main interface)
    case 0x81:
      return "Volume Up";
    case 0x82:
      return "Volume Down";
    case 0x83:
      return "Mute";
    case 0xB5:
      return "Next Track";
    case 0xB6:
      return "Previous Track";
    case 0xB7:
      return "Stop";
    case 0xCD:
      return "Play/Pause";
    case 0x8A:
      return "Mail";
    case 0x92:
      return "Calculator";
    case 0x94:
      return "My Computer";

    // Application/Menu keys
    case 0x65:
      return "Application";
    case 0x76:
      return "Menu";

    default:
      return "Unknown";
  }
}

}  // namespace usb_hidx
}  // namespace esphome
