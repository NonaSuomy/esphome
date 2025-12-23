#pragma once

namespace esphome {
namespace usb_hidx {

// Map consumer control usage codes to names
inline const char *consumer_code_to_name(uint8_t code) {
  switch (code) {
    // Media control
    case 0xE9:
      return "Volume Up";
    case 0xEA:
      return "Volume Down";
    case 0xE2:
      return "Mute";
    case 0xCD:
      return "Play/Pause";
    case 0xB5:
      return "Next Track";
    case 0xB6:
      return "Previous Track";
    case 0xB7:
      return "Stop";
    // Application launch
    case 0x8A:
      return "Mail";
    case 0x92:
      return "Calculator";
    case 0x94:
      return "My Computer";
    // Browser control
    case 0x23:
      return "WWW Home";
    case 0x21:
      return "WWW Search";
    case 0x24:
      return "WWW Back";
    case 0x25:
      return "WWW Forward";
    case 0x82:
      return "Favourites";
    // Zoom
    case 0x2D:
      return "Zoom In";
    case 0x2E:
      return "Zoom Out";
    case 0x95:
      return "Help";
    // Microsoft keyboard F-Lock keys
    case 0x1A:
      return "Undo";
    case 0x79:
      return "Redo";
    case 0x89:
      return "Reply";
    case 0x8B:
      return "Forward Mail";
    case 0x8C:
      return "Send Mail";
    default:
      return nullptr;
  }
}

}  // namespace usb_hidx
}  // namespace esphome
