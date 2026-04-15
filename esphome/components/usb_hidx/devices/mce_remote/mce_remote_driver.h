#pragma once

#include "esphome/components/usb_hidx/usb_hidx.h"

namespace esphome {
namespace usb_hidx {

class MCERemoteDriver : public HIDDeviceDriver {
 public:
  MCERemoteDriver(USBHIDXComponent *parent) : parent_(parent) {}

  bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) override {
    if (vid == 0x1784) return true;
    if (vid == 0x045E && (pid == 0x006D || pid == 0x00A0 || pid == 0x00F2 || pid == 0x0745)) return true;
    if (vid == 0x0471 && (pid == 0x0608 || pid == 0x060C || pid == 0x060D || pid == 0x060F ||
                          pid == 0x0613 || pid == 0x0815 || pid == 0x206C || pid == 0x2088 || pid == 0x2093)) return true;
    if (vid == 0x0609 && (pid == 0x031D || pid == 0x0322 || pid == 0x0334 || pid == 0x0338 ||
                          pid == 0x0353 || pid == 0x0357)) return true;
    if (vid == 0x1460 && pid == 0x9150) return true;
    if (vid == 0x1308 && pid == 0xC001) return true;
    if (vid == 0x051C && pid == 0xC001) return true;
    if (vid == 0x107B && pid == 0x3009) return true;
    if (vid == 0x03EE && pid == 0x2501) return true;
    if (vid == 0x179D && pid == 0x0010) return true;
    if (vid == 0x195D && pid == 0x7002) return true;
    if (vid == 0x1509 && pid == 0x9242) return true;
    if (vid == 0x043E && pid == 0x9803) return true;
    if (vid == 0x147A && (pid == 0xE015 || pid == 0xE016 || pid == 0xE017 || pid == 0xE018 ||
                          pid == 0xE03A || pid == 0xE03C || pid == 0xE03E || pid == 0xE042)) return true;
    if (vid == 0x1934 && (pid == 0x5168 || pid == 0x0602 || pid == 0x0702)) return true;
    if (vid == 0x2304 && pid == 0x0225) return true;
    if (vid == 0x1019 && pid == 0x0F38) return true;
    if (vid == 0x0FB8 && pid == 0x0002) return true;
    if (vid == 0x185B && (pid == 0x3020 || pid == 0x3082)) return true;
    if (vid == 0x04EB && pid == 0xE004) return true;
    if (vid == 0x105A && pid == 0x2000) return true;
    if (vid == 0x0572 && (pid == 0x58A1 || pid == 0x58A5)) return true;
    if (vid == 0x0BDA && pid == 0x0161) return true;
    if (vid == 0x2596 && (pid == 0x8008 || pid == 0x8016 || pid == 0x8042)) return true;
    if (vid == 0x03F3 && pid == 0x0094) return true;
    return false;
  }

  void on_device_ready(HIDDevice *device) {
    device_ = device;
    ESP_LOGI("usb_hidx.mce", "MCE IR Receiver detected, sending gen2 init");
    uint8_t resume[] = {0x00, 0xFF, 0xAA};
    parent_->send_xbox360_interrupt_out(device, resume, sizeof(resume));
    uint8_t wake[] = {0xFF, 0x18};
    parent_->send_xbox360_interrupt_out(device, wake, sizeof(wake));
    uint8_t unk[] = {0x9F, 0x05};
    parent_->send_xbox360_interrupt_out(device, unk, sizeof(unk));
  }

  void process_report(const uint8_t *data, size_t len, HIDDevice *device) override {
    if (!device_) device_ = device;
    if (len == 0) return;

    for (size_t i = 0; i < len; i++) {
      uint8_t b = data[i];

      // PORT_SYS (0xFF) or PORT_IR (0x9F) command responses - skip byte + subcommand
      if (b == 0xFF || b == 0x9F) { i++; continue; }

      // IR trailer (0x80) = end of signal - try to emit
      if (b == 0x80) {
        emit_if_ready();
        reset_decoder();
        continue;
      }

      // IR data packet: header byte 0x80+N followed by N sample bytes
      if ((b & 0xE0) == 0x80 && b != 0x80) {
        uint8_t num = b & 0x1F;
        for (uint8_t j = 0; j < num && (i + 1 + j) < len; j++) {
          uint8_t s = data[i + 1 + j];
          if (s == 0x7F) {
            // Max space = end of signal
            emit_if_ready();
            reset_decoder();
          } else {
            feed_sample(bool(s & 0x80), (s & 0x7F) * 50);
          }
        }
        i += num;
      }
    }
  }

  const char *get_name() override { return "MCERemote"; }

 protected:
  USBHIDXComponent *parent_;
  HIDDevice *device_{nullptr};

  // RC6 decoder state machine - faithful port of Linux ir-rc6-decoder.c
  // Uses T=450us (9 MCE units) instead of 444us to match 50us quantization
  static const uint32_t T   = 450;   // half-bit period in us
  static const uint32_t TOL = 200;   // tolerance in us

  enum State {
    INACTIVE, PREFIX_SPACE, HDR_BIT_START, HDR_BIT_END,
    TOGGLE_START, TOGGLE_END, BODY_BIT_START, BODY_BIT_END, FINISHED
  } state_{INACTIVE};

  uint32_t header_{0};
  uint32_t body_{0};
  int      count_{0};
  bool     toggle_{false};
  bool     prev_pulse_{false};

  static bool eq(uint32_t a, uint32_t b)  { return (a > b ? a-b : b-a) <= TOL; }
  static bool geq(uint32_t a, uint32_t b) { return a + TOL >= b; }

  void reset_decoder() {
    state_ = INACTIVE;
    header_ = body_ = count_ = 0;
    toggle_ = prev_pulse_ = false;
  }

  void emit_if_ready() {
    // Emit if we have a complete or near-complete body
    if (count_ < 16) return;

    uint32_t sc = body_;
    // For RC6-6A-32 MCE: body contains full 32-bit scancode
    // MCE customer code = 0x800f0000, toggle in bit 15
    if (count_ >= 32 && (sc & 0xFFFF0000) == 0x800F0000) {
      bool tog = (sc & 0x8000) != 0;
      sc &= ~0x8000U;
      uint8_t cmd = sc & 0xFF;
      ESP_LOGI("usb_hidx.mce", "MCE key: cmd=0x%02X toggle=%d scancode=0x%08X", cmd, tog, sc);
      const char *name = rc6_mce_keyname(cmd);
      if (name) {
        ESP_LOGI("usb_hidx.mce", "Key: %s", name);
#ifdef USE_TEXT_SENSOR
        if (parent_->get_keyboard_sensor())
          parent_->get_keyboard_sensor()->publish_state(name);
#endif
      } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "RC6:0x%02X", cmd);
        ESP_LOGI("usb_hidx.mce", "Unknown key: %s", buf);
#ifdef USE_TEXT_SENSOR
        if (parent_->get_keyboard_sensor())
          parent_->get_keyboard_sensor()->publish_state(buf);
#endif
      }
    }
  }

  void feed_sample(bool pulse, uint32_t dur) {
    bool again = true;
    while (again) {
      again = false;
      switch (state_) {
        case INACTIVE:
          if (!pulse || !eq(dur, 6*T)) break;
          state_ = PREFIX_SPACE; count_ = 0; break;

        case PREFIX_SPACE:
          if (pulse || !eq(dur, 2*T)) break;
          state_ = HDR_BIT_START; header_ = 0; break;

        case HDR_BIT_START:
          if (!eq(dur, T)) break;
          header_ = (header_ << 1) | (pulse ? 1 : 0);
          count_++; prev_pulse_ = pulse;
          state_ = HDR_BIT_END; break;

        case HDR_BIT_END:
          if (pulse == prev_pulse_) { state_ = INACTIVE; break; }
          state_ = (count_ == 4) ? TOGGLE_START : HDR_BIT_START;
          if (dur > T + TOL) { dur -= T; again = true; }
          break;

        case TOGGLE_START:
          if (!eq(dur, 2*T)) { state_ = INACTIVE; break; }
          toggle_ = pulse; prev_pulse_ = pulse;
          state_ = TOGGLE_END; break;

        case TOGGLE_END:
          if (pulse == prev_pulse_) { state_ = INACTIVE; break; }
          if (!geq(dur, 2*T)) { state_ = INACTIVE; break; }
          if (!(header_ & 0x08)) { state_ = INACTIVE; break; }
          state_ = BODY_BIT_START;
          if (dur > 2*T + TOL) { dur -= 2*T; again = true; } else { dur = 0; }
          count_ = 0; body_ = 0;
          break;

        case BODY_BIT_START:
          if (eq(dur, T)) {
            if (count_ < 32) body_ = (body_ << 1) | (pulse ? 1 : 0);
            count_++; prev_pulse_ = pulse;
            state_ = BODY_BIT_END; break;
          } else if (!pulse && geq(dur, 6*T)) {
            state_ = FINISHED; again = true;
          } else {
            // Tolerance: accept slightly off durations
            if (dur < 2*T) {
              if (count_ < 32) body_ = (body_ << 1) | (pulse ? 1 : 0);
              count_++; prev_pulse_ = pulse;
              state_ = BODY_BIT_END; break;
            }
            state_ = INACTIVE;
          }
          break;

        case BODY_BIT_END:
          if (pulse == prev_pulse_) { state_ = INACTIVE; break; }
          state_ = (count_ >= 32) ? FINISHED : BODY_BIT_START;
          if (dur > T + TOL) { dur -= T; again = true; }
          break;

        case FINISHED:
          emit_if_ready();
          reset_decoder();
          // Re-feed this sample as potential new leader
          if (pulse && eq(dur, 6*T)) {
            state_ = PREFIX_SPACE; count_ = 0; again = true;
          }
          break;
      }
    }
  }

  static const char *rc6_mce_keyname(uint8_t cmd) {
    switch (cmd) {
      case 0x00: return "0";
      case 0x01: return "1";
      case 0x02: return "2";
      case 0x03: return "3";
      case 0x04: return "4";
      case 0x05: return "5";
      case 0x06: return "6";
      case 0x07: return "7";
      case 0x08: return "8";
      case 0x09: return "9";
      case 0x0C: return "Power";
      case 0x0D: return "MCE/Start";
      case 0x0E: return "Mute";
      case 0x0F: return "Info";
      case 0x10: return "Volume Up";
      case 0x11: return "Volume Down";
      case 0x12: return "Channel Up";
      case 0x13: return "Channel Down";
      case 0x14: return "Fast Forward";
      case 0x15: return "Rewind";
      case 0x16: return "Play";
      case 0x17: return "Record";
      case 0x18: return "Pause";
      case 0x19: return "Stop";
      case 0x1A: return "Skip Forward";
      case 0x1B: return "Skip Back";
      case 0x1C: return "Up";
      case 0x1D: return "Down";
      case 0x1E: return "Left";
      case 0x1F: return "Right";
      case 0x20: return "OK";
      case 0x21: return "Back";
      case 0x22: return "DVD Menu";
      case 0x23: return "Guide";
      case 0x24: return "Live TV";
      case 0x25: return "My TV";
      case 0x26: return "My Music";
      case 0x27: return "Recorded TV";
      case 0x28: return "My Pictures";
      case 0x29: return "My Videos";
      case 0x2A: return "DVD Angle";
      case 0x2B: return "DVD Audio";
      case 0x2C: return "DVD Subtitle";
      case 0x2D: return "Radio";
      case 0x2E: return "Teletext";
      case 0x2F: return "Enter";
      case 0x30: return "Red";
      case 0x31: return "Green";
      case 0x32: return "Yellow";
      case 0x33: return "Blue";
      case 0x34: return "Closed Caption";
      case 0x35: return "Ext";
      case 0x36: return "Zoom";
      case 0x37: return "Sleep";
      case 0x38: return "Details";
      case 0x39: return "Eject";
      case 0x3A: return "Input";
      case 0x3B: return "Messenger";
      case 0x3C: return "Edit";
      case 0x3D: return "Delete";
      case 0x3E: return "Print";
      case 0x3F: return "Favorites";
      case 0x40: return "Replay";
      case 0x41: return "TV";
      case 0x42: return "Music";
      case 0x43: return "Photos";
      case 0x44: return "Videos";
      case 0x45: return "Radio";
      case 0x46: return "Clear";
      case 0x47: return "Hash";
      case 0x48: return "Star";
      case 0x4A: return "3D";
      default:   return nullptr;
    }
  }
};

}  // namespace usb_hidx
}  // namespace esphome
