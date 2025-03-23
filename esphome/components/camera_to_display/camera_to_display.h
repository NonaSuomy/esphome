#pragma once

#include "esphome/core/component.h"
#include "esphome/components/display/display_buffer.h"
#include "esphome/components/esp32_camera/esp32_camera.h"

namespace esphome {
namespace camera_to_display {

class CameraToDisplayComponent : public Component {
 public:
  void set_display(display::DisplayBuffer *display) { display_ = display; }
  void set_camera(esp32_camera::ESP32Camera *camera) { camera_ = camera; }
  void set_update_interval(uint32_t update_interval) { update_interval_ = update_interval; }
  
  void setup() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

 protected:
  display::DisplayBuffer *display_{nullptr};
  esp32_camera::ESP32Camera *camera_{nullptr};
  uint32_t update_interval_{200};
  uint32_t last_update_{0};
};

}  // namespace camera_to_display
}  // namespace esphome
