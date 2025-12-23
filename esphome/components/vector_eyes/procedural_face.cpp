#include "procedural_face.h"
#include "esphome/core/log.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace vector_eyes {

static const char *TAG = "procedural_face";

void ProceduralFace::Draw(display::DisplayBuffer *display) {
  // Unpack fixed-point to float (shadowing member variables)
  float scale_x = (float)this->scale_x / SCALE;
  float scale_y = (float)this->scale_y / SCALE;
  float center_x = (float)this->center_x / SCALE;
  float center_y = (float)this->center_y / SCALE;
  float l_lid_top = (float)this->l_lid_top / SCALE;
  float l_lid_bottom = (float)this->l_lid_bottom / SCALE;
  float r_lid_top = (float)this->r_lid_top / SCALE;
  float r_lid_bottom = (float)this->r_lid_bottom / SCALE;

  // Clear display buffer before drawing to prevent artifacts from previous frames
  display->fill(display::COLOR_OFF);
  
  int width = display->get_width();
  int height = display->get_height();
  
  // Auto-scale based on screen width (Reference width 128)
  float screen_scale = (float)width / 128.0f;
  
  // Calculate base dimensions scaled to screen
  float base_eye_width = EYE_WIDTH * screen_scale;
  float base_eye_height = EYE_HEIGHT * screen_scale;
  float base_eye_spacing = EYE_SPACING * screen_scale;
  
  // Clamp center position to prevent eyes from going off-screen
  // Scale clamp range too
  float max_cx = 10.0f * screen_scale;
  float max_cy = 8.0f * screen_scale;
  float clamped_cx = std::max(-max_cx, std::min(max_cx, center_x * screen_scale)); // Assuming center_x is normalized? No, it's pixels. Scale it.
  float clamped_cy = std::max(-max_cy, std::min(max_cy, center_y * screen_scale));
  
  int cx = width / 2 + (int)clamped_cx;
  int cy = height / 2 + (int)clamped_cy;
  
  // Calculate eye positions - Vector style spacing
  int l_eye_cx = cx - (int)(base_eye_spacing / 2) - (int)(base_eye_width / 2);
  int r_eye_cx = cx + (int)(base_eye_spacing / 2) + (int)(base_eye_width / 2);
  int eye_cy = cy;
  
  // Apply scale with bounds
  float clamped_scale_x = std::max(0.3f, std::min(1.5f, scale_x));
  float clamped_scale_y = std::max(0.3f, std::min(1.5f, scale_y));
  
  int l_w = (int)(base_eye_width * clamped_scale_x);
  int l_h = (int)(base_eye_height * clamped_scale_y);
  int r_w = (int)(base_eye_width * clamped_scale_x);
  int r_h = (int)(base_eye_height * clamped_scale_y);
  
  // Draw eyes
  draw_rounded_eye(display, l_eye_cx, eye_cy, l_w, l_h, l_lid_top, l_lid_bottom);
  draw_rounded_eye(display, r_eye_cx, eye_cy, r_w, r_h, r_lid_top, r_lid_bottom);
}

void ProceduralFace::draw_rounded_eye(display::DisplayBuffer *display, int cx, int cy, int w, int h, float lid_top, float lid_bottom) {
  // Calculate eye bounds
  int x = cx - w/2;
  int y = cy - h/2;
  
  // Corner radius for more oval/rounded shape (30% of width for Vector-like eyes)
  int radius = w / 3;
  radius = std::max(2, std::min(radius, h/3));
  
  // Fill the eye using scanlines for proper rounded rectangle
  for (int dy = 0; dy < h; dy++) {
    int py = y + dy;
    
    // Skip if off-screen
    if (py < 0 || py >= display->get_height()) continue;
    
    int line_w = w;
    int line_x = x;
    
    // Apply rounding to corners for oval shape
    if (dy < radius) {
      // Top corners
      int offset = radius - dy;
      float arc = std::sqrt((float)(radius * radius - offset * offset));
      int reduce = radius - (int)arc;
      line_w -= 2 * reduce;
      line_x += reduce;
    } else if (dy >= h - radius) {
      // Bottom corners
      int offset = dy - (h - radius);
      float arc = std::sqrt((float)(radius * radius - offset * offset));
      int reduce = radius - (int)arc;
      line_w -= 2 * reduce;
      line_x += reduce;
    }
    
    // Bounds check and draw
    if (line_w > 0 && line_x >= 0 && line_x + line_w <= display->get_width()) {
      display->horizontal_line(line_x, py, line_w, display::COLOR_ON);
    }
  }
  
  // Draw eyelids on top (clamped to eye bounds)
  // Lid values are "openness" (0.0 = closed, 1.0 = fully open)
  // We calculate the height of the black rectangle to draw from the edge
  float clamped_lid_top = std::max(0.0f, std::min(1.0f, lid_top));
  float clamped_lid_bottom = std::max(0.0f, std::min(1.0f, lid_bottom));
  
  // Top lid (draws from top down)
  if (clamped_lid_top < 1.0f) {
    // Calculate how much is covered (1.0 - openness) * half_height
    // If openness is 0.0 (closed), we cover half height (meeting in middle)
    // Actually, if both are 0, they should meet. So we cover h/2.
    // If openness is 1.0, we cover 0.
    int lid_h = (int)((h / 2.0f) * (1.0f - clamped_lid_top));
    
    // Ensure we don't draw negative or zero height if fully open
    if (lid_h > 0) {
      for (int i = 0; i < lid_h && i < h; i++) {
        int py = y + i;
        if (py >= 0 && py < display->get_height() && x >= 0 && x + w <= display->get_width()) {
          display->horizontal_line(x, py, w, display::COLOR_OFF);
        }
      }
    }
  }
  
  // Bottom lid (draws from bottom up)
  if (clamped_lid_bottom < 1.0f) {
    int lid_h = (int)((h / 2.0f) * (1.0f - clamped_lid_bottom));
    
    if (lid_h > 0) {
      for (int i = 0; i < lid_h && i < h; i++) {
        int py = y + h - 1 - i;
        if (py >= 0 && py < display->get_height() && x >= 0 && x + w <= display->get_width()) {
          display->horizontal_line(x, py, w, display::COLOR_OFF);
        }
      }
    }
  }
}

}  // namespace vector_eyes
}  // namespace esphome
