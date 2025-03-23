#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "camera_to_display.h"
#include "esp_jpg_decode.h"

namespace esphome {
namespace camera_to_display {

static const char *TAG = "camera_to_display";

void CameraToDisplayComponent::setup() {
  if (this->camera_ == nullptr || this->display_ == nullptr) {
    ESP_LOGE(TAG, "Camera or display not set!");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Display dimensions: %dx%d", display_->get_width(), display_->get_height());
}

void CameraToDisplayComponent::loop() {
  const uint32_t now = millis();
  if (now - last_update_ < update_interval_) {
    return;
  }
  last_update_ = now;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGW(TAG, "Camera capture failed");
    return;
  }

  ESP_LOGD(TAG, "Got frame: %dx%d (len=%d, format=%d)", 
           fb->width, fb->height, fb->len, fb->format);

  // Clear display
  display_->fill(Color::BLACK);

  // Convert JPEG to RGB565 and draw directly to display
  if (fb->format == PIXFORMAT_JPEG) {
    // Allocate buffer for full RGB565 image
    size_t rgb_buf_size = fb->width * fb->height * 2; // 2 bytes per pixel for RGB565
    uint8_t *rgb_buf = (uint8_t *)heap_caps_malloc(rgb_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    
    if (rgb_buf) {
      // Convert JPEG to RGB565 (no scaling)
      if (jpg2rgb565(fb->buf, fb->len, rgb_buf, JPG_SCALE_NONE)) {
        // Calculate scaling factors
        float scale_x = (float)fb->width / display_->get_width();
        float scale_y = (float)fb->height / display_->get_height();
        
        // Draw to display
        for (int y = 0; y < (int)fb->height; y++) {
          int display_y = (int)(y / scale_y);
          if (display_y >= display_->get_height()) continue;
          
          for (int x = 0; x < (int)fb->width; x++) {
            int display_x = (int)(x / scale_x);
            if (display_x >= display_->get_width()) continue;
            
            // For RGB565, each pixel is 2 bytes
            int idx = (y * fb->width + x) * 2;
            uint16_t pixel = (rgb_buf[idx + 1] << 8) | rgb_buf[idx];
            
            // Convert RGB565 to RGB888
            uint8_t r = ((pixel >> 11) & 0x1F) << 3;
            uint8_t g = ((pixel >> 5) & 0x3F) << 2;
            uint8_t b = (pixel & 0x1F) << 3;
            
            display_->draw_pixel_at(display_x, display_y, Color(r, g, b));
          }
        }
        ESP_LOGD(TAG, "Frame converted and drawn");
      } else {
        ESP_LOGE(TAG, "JPEG conversion failed");
      }
      free(rgb_buf);
    } else {
      ESP_LOGE(TAG, "Failed to allocate RGB buffer (requested %d bytes)", rgb_buf_size);
    }
  } else {
    ESP_LOGE(TAG, "Unsupported image format: %d", fb->format);
  }

  display_->update();
  esp_camera_fb_return(fb);
}

}  // namespace camera_to_display
}  // namespace esphome
