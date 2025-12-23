#pragma once

#include "esphome/components/display/display_buffer.h"

namespace esphome {
namespace vector_eyes {

// Compact procedural face structure for Vector-style eye animations
// Size: 36 bytes (9 floats) - optimized for PROGMEM storage
struct ProceduralFace {
  // Use fixed-point representation to save memory (int16_t = 2 bytes vs float = 4 bytes)
  static constexpr float SCALE = 1000.0f;

  // Transform parameters
  int16_t scale_x{1000};
  int16_t scale_y{1000};
  int16_t angle{0};
  
  // Position parameters
  int16_t center_x{0};
  int16_t center_y{0};
  
  // Eyelid parameters
  int16_t l_lid_top{0};
  int16_t l_lid_bottom{0};
  int16_t r_lid_top{0};
  int16_t r_lid_bottom{0};
  
  // Default constructor
  constexpr ProceduralFace() = default;
  
  // Constructor accepting floats - auto-scales to int16_t at compile time
  // This allows generated_animations.h to use float literals unchanged
  constexpr ProceduralFace(float sx, float sy, float a, float cx, float cy,
                           float lt, float lb, float rt, float rb)
      : scale_x(static_cast<int16_t>(sx * SCALE)),
        scale_y(static_cast<int16_t>(sy * SCALE)),
        angle(static_cast<int16_t>(a * SCALE)),
        center_x(static_cast<int16_t>(cx * SCALE)),
        center_y(static_cast<int16_t>(cy * SCALE)),
        l_lid_top(static_cast<int16_t>(lt * SCALE)),
        l_lid_bottom(static_cast<int16_t>(lb * SCALE)),
        r_lid_top(static_cast<int16_t>(rt * SCALE)),
        r_lid_bottom(static_cast<int16_t>(rb * SCALE)) {}
  
  // Eye dimensions optimized for 128x64 display
  static constexpr float EYE_WIDTH = 24.0f;
  static constexpr float EYE_HEIGHT = 32.0f;
  static constexpr float EYE_SPACING = 16.0f;
  
  // Runtime eye dimensions (can be modified per instance if needed)
  int16_t eye_width{24000};   // 24.0 * 1000
  int16_t eye_height{32000};  // 32.0 * 1000
  int16_t eye_spacing{16000}; // 16.0 * 1000
  
  void Reset() {
    scale_x = 1000;
    scale_y = 1000;
    angle = 0;
    center_x = 0;
    center_y = 0;
    l_lid_top = 1000;
    l_lid_bottom = 1000;
    r_lid_top = 1000;
    r_lid_bottom = 1000;
  }
  
  void Draw(display::DisplayBuffer *display);
  
 private:
  void draw_rounded_eye(display::DisplayBuffer *display, int cx, int cy, int w, int h, float lid_top, float lid_bottom);
};

}  // namespace vector_eyes
}  // namespace esphome
