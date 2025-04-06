// micro_rtsp.cpp
#include "micro_rtsp.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>
#include <fcntl.h>

#ifdef USE_ESP32
#include <esp_wifi.h>
#endif

namespace esphome {
namespace micro_rtsp {

static const char *const TAG = "micro_rtsp";

void MicroRTSP::setup() {
    if (!wifi::global_wifi_component->is_connected()) {
        ESP_LOGW(TAG, "WiFi not connected. RTSP server setup delayed.");
        return;
    }

    ESP_LOGD(TAG, "Creating RTSP server socket...");
    server_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_socket_ < 0) {
        ESP_LOGE(TAG, "Failed to create server socket: %d", errno);
        return;
    }

    int enable = 1;
    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
        ESP_LOGE(TAG, "setsockopt(SO_REUSEADDR) failed: %d", errno);
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);

    if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind server socket: %d", errno);
        close(server_socket_);
        server_socket_ = -1;
        return;
    }

    if (listen(server_socket_, 5) < 0) {
        ESP_LOGE(TAG, "Failed to listen on server socket: %d", errno);
        close(server_socket_);
        server_socket_ = -1;
        return;
    }

    // Set non-blocking
    int flags = fcntl(server_socket_, F_GETFL, 0);
    fcntl(server_socket_, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "RTSP server listening on port %d", port_);
}

void MicroRTSP::loop() {
    if (server_socket_ < 0 && wifi::global_wifi_component->is_connected()) {
        ESP_LOGD(TAG, "Attempting RTSP server setup...");
        setup();
        return;
    }

    static uint32_t last_cleanup = 0;
    static uint32_t last_status = 0;
    uint32_t now = millis();

    // Handle new connections
    handle_new_connections();

    // Cleanup sessions periodically
    if (now - last_cleanup >= 1000) {
        cleanup_sessions();
        last_cleanup = now;
    }

    // Print status periodically
    if (now - last_status >= 5000) {
        ESP_LOGD(TAG, "RTSP Status - Active sessions: %d, Server socket: %d", 
                sessions_.size(), server_socket_);
        last_status = now;
    }

    // Stream camera frames with consistent timing
    if (camera_ != nullptr && !sessions_.empty() && 
        (now - last_frame_time_) >= frame_duration_ms_) {
        
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb != nullptr) {
            if (fb->len > 0 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) {
                ESP_LOGV(TAG, "Valid JPEG frame received, size: %d", fb->len);
                
                std::lock_guard<std::mutex> lock(session_mutex_);
                for (const auto& session : sessions_) {
                    if (session && session->is_playing()) {
                        session->send_frame(fb->buf, fb->len);
                        delay(1);  // Small delay between clients
                    }
                }
            } else {
                ESP_LOGW(TAG, "Invalid JPEG frame received");
            }
            esp_camera_fb_return(fb);
        }
        
        last_frame_time_ = now;
    }

    // Process RTSP requests for each session
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        for (auto& session : sessions_) {
            if (session) {
                session->handle_request();
            }
        }
    }
}

// Helper function to create a unique_ptr<RTSPSession>
static std::unique_ptr<RTSPSession> create_rtsp_session(int client_socket, uint16_t width, uint16_t height, uint16_t port) {
  return std::unique_ptr<RTSPSession>(new RTSPSession(client_socket, width, height, port));
}
void MicroRTSP::handle_new_connections() {
  if (server_socket_ < 0) {
      return;
  }

  struct sockaddr_in client_addr;
  socklen_t client_len = sizeof(client_addr);
  
  int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
  if (client_socket >= 0) {
      // Set non-blocking
      int flags = fcntl(client_socket, F_GETFL, 0);
      fcntl(client_socket, F_SETFL, flags | O_NONBLOCK);
      
      char client_ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
      ESP_LOGI(TAG, "New RTSP client connected from %s:%d", 
               client_ip, ntohs(client_addr.sin_port));
      
               std::lock_guard<std::mutex> lock(session_mutex_);
               // Use std::make_shared correctly
               sessions_.emplace_back(std::make_shared<RTSPSession>(client_socket, FRAME_WIDTH, FRAME_HEIGHT, port_));
       
  }
}

void MicroRTSP::cleanup_sessions() {
    std::lock_guard<std::mutex> lock(session_mutex_);
    size_t initial_size = sessions_.size();
    sessions_.erase(
        std::remove_if(
            sessions_.begin(), 
            sessions_.end(),
            [](const std::shared_ptr<RTSPSession>& session) {
                return !session || !session->is_alive();
            }
        ),
        sessions_.end()
    );
    if (initial_size != sessions_.size()) {
        ESP_LOGD(TAG, "Cleaned up sessions. Active sessions: %d", sessions_.size());
    }
}

}  // namespace micro_rtsp
}  // namespace esphome
