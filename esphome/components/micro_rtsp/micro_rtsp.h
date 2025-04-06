// micro_rtsp.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/esp32_camera/esp32_camera.h"
#include "esphome/components/wifi/wifi_component.h"
#include "rtsp_session.h"
#include <vector>
#include <memory>
#include <mutex>

namespace esphome {
namespace micro_rtsp {

class MicroRTSP : public Component {
public:
    MicroRTSP() : port_(8554) {
        static constexpr uint32_t TARGET_FPS = 15;
        frame_duration_ms_ = 1000 / TARGET_FPS;
    }

    void setup() override;
    void loop() override;
    void set_camera(esp32_camera::ESP32Camera *camera) { camera_ = camera; }
    void set_port(uint16_t port) { port_ = port; }
    void set_frame_duration(uint32_t duration) { frame_duration_ms_ = duration; }
    float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

private:
    void handle_new_connections();
    void cleanup_sessions();

    esp32_camera::ESP32Camera *camera_{nullptr};
    int server_socket_{-1};
    uint16_t port_;
    std::vector<std::shared_ptr<RTSPSession>> sessions_;
    uint32_t last_frame_time_{0};
    uint32_t frame_duration_ms_;
    std::mutex session_mutex_;
    static constexpr uint16_t FRAME_WIDTH = 320;
    static constexpr uint16_t FRAME_HEIGHT = 240;
};

}  // namespace micro_rtsp
}  // namespace esphome
