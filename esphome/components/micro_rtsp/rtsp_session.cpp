// components/micro_rtsp/rtsp_session.cpp
#include "esphome/core/log.h"
#include "rtsp_session.h"
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_netif.h>
#include <string>
#include <cstring>
#include <random>

namespace esphome {
namespace micro_rtsp {

static const char *const TAG = "rtsp_session";
static const uint32_t SESSION_TIMEOUT = 30000; // 30 seconds

std::string RTSPSession::get_local_ip() const {
  esp_netif_ip_info_t ip_info;
  esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (sta_netif == nullptr) {
    ESP_LOGE(TAG, "Could not get WIFI_STA_DEF netif handle");
    return "";
  }
  esp_err_t err = esp_netif_get_ip_info(sta_netif, &ip_info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_netif_get_ip_info failed: %s", esp_err_to_name(err));
    return "";
  }
  char buf[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &ip_info.ip, buf, INET_ADDRSTRLEN);
  return buf;
}

RTSPSession::RTSPSession(int client_socket, uint16_t width, uint16_t height, uint16_t port)
    : socket_(client_socket), width_(width), height_(height), port_(port) {
  // Generate a random session ID
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, UINT32_MAX);
  uint32_t random_id = dis(gen);
  char session_id_buf[17];
  snprintf(session_id_buf, sizeof(session_id_buf), "%08X%08X", (uint32_t)(random_id >> 16), (uint32_t)(random_id & 0xFFFF));
  session_id_ = session_id_buf;

  last_activity_ = millis();
  ESP_LOGD(TAG, "Created RTSP session %s", session_id_.c_str());
}

RTSPSession::~RTSPSession() {
  if (socket_ >= 0) {
    close(socket_);
  }
  ESP_LOGD(TAG, "Destroyed RTSP session %s", session_id_.c_str());
}

bool RTSPSession::is_alive() const {
  return socket_ >= 0 && (millis() - last_activity_) < SESSION_TIMEOUT;
}

void RTSPSession::send_response(const char* response) {
    send(socket_, response, strlen(response), 0);
    last_activity_ = millis();
}

void RTSPSession::send_error(int cseq, int code, const char* message) {
    char response[256];
    snprintf(response, sizeof(response),
             "RTSP/1.0 %d %s\r\n"
             "CSeq: %d\r\n"
             "\r\n",
             code, message, cseq);
    send_response(response);
}

int RTSPSession::get_cseq(const std::string& request) {
    size_t pos = request.find("CSeq:");
    if (pos == std::string::npos) return 0;
    
    std::string line = request.substr(pos);
    size_t end = line.find("\r\n");
    if (end == std::string::npos) return 0;
    line = line.substr(0, end);
    
    std::string key, value;
    if (!parse_header_line(line, key, value)) return 0;
    
    return atoi(value.c_str());
}

std::string RTSPSession::get_url(const std::string& request) {
    std::string method, url, version;
    size_t pos = request.find("\r\n");
    if (pos == std::string::npos) return "";
    std::string line = request.substr(0, pos);
    if (!parse_request_line(line, method, url, version)) return "";
    return url;
}

void RTSPSession::handle_options(const std::string& request) {
    int cseq = get_cseq(request);
    char response[256];
    snprintf(response, sizeof(response),
             "RTSP/1.0 200 OK\r\n"
             "CSeq: %d\r\n"
             "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n"
             "\r\n",
             cseq);
    send_response(response);
}

//ESP_LOGI(TAG, "Sending SDP with IP address: %s", ip_address.c_str());
//std::string sdp = "v=0\r\n"
//                  "o=- 211954 1 IN IP4 " + ip_address + "\r\n"

void RTSPSession::handle_describe(const std::string& request) {
  int cseq = get_cseq(request);
  stream_path_ = get_url(request);

  std::string ip_address = get_local_ip();
  if (ip_address.empty()) {
    send_error(cseq, 500, "Internal Server Error: Could not get IP address");
    return;
  }

  std::string sdp = "v=0\r\n"
                    "o=- 211954 1 IN IP4 " + ip_address + "\r\n"
                    "s=ESPHome RTSP Stream\r\n"
                    "i=Live JPEG Stream\r\n"
                    "t=0 0\r\n"
                    "a=tool:ESPHome\r\n"
                    "a=type:broadcast\r\n"
                    "a=control:*\r\n"
                    "a=range:npt=0-100\r\n"
                    "m=video 0 RTP/AVP 26\r\n"
                    "c=IN IP4 " + ip_address + "\r\n"
                    "b=AS:2000\r\n"
                    "a=rtpmap:26 JPEG/90000\r\n"
                    "a=control:trackID=0\r\n"
                    "a=framerate:15\r\n"
                    "a=quality:5\r\n";

  char response[512];
  snprintf(response, sizeof(response),
           "RTSP/1.0 200 OK\r\n"
           "CSeq: %d\r\n"
           "Content-Type: application/sdp\r\n"
           "Content-Base: rtsp://%s:%u/stream/\r\n"
           "Content-Length: %zu\r\n"
           "\r\n"
           "%s",
           cseq, ip_address.c_str(), port_, sdp.length(), sdp.c_str());
  send_response(response);
}

void RTSPSession::handle_setup(const std::string& request) {
    int cseq = get_cseq(request);
    
    // Parse client ports
    size_t pos = request.find("client_port=");
    if (pos == std::string::npos) {
        send_error(cseq, 400, "Bad Request");
        return;
    }
    
    sscanf(request.c_str() + pos + 12, "%hu-%hu", 
           &client_rtp_port_, &client_rtcp_port_);
    
    ESP_LOGD(TAG, "Client ports - RTP: %hu, RTCP: %hu", client_rtp_port_, client_rtcp_port_);
    
    // Create RTP packetizer
    rtp_packetizer_.reset(new RTPPacketizer(socket_, client_rtp_port_, client_rtcp_port_));
    if (!rtp_packetizer_->is_valid()) {
        send_error(cseq, 500, "Internal Server Error");
        return;
    }
    server_rtp_port_ = rtp_packetizer_->get_server_rtp_port();
    
    ESP_LOGD(TAG, "Handling SETUP request for stream: %s", stream_path_.c_str());
    ESP_LOGD(TAG, "Creating new RTP packetizer with ports RTP:%hu RTCP:%hu", client_rtp_port_, client_rtcp_port_);
    
    char response[256];
    snprintf(response, sizeof(response),
             "RTSP/1.0 200 OK\r\n"
             "CSeq: %d\r\n"
             "Session: %s\r\n"
             "Transport: RTP/AVP;unicast;client_port=%hu-%hu;server_port=%hu-%hu\r\n"
             "\r\n",
             cseq, session_id_.c_str(),
             client_rtp_port_, client_rtcp_port_,
             server_rtp_port_, server_rtp_port_ + 1);
    send_response(response);
    
    state_ = State::READY;
}

void RTSPSession::handle_play(const std::string& request) {
    int cseq = get_cseq(request);
    
    if (state_ != State::READY) {
        send_error(cseq, 455, "Method Not Valid In This State");
        return;
    }
    
    char response[256];
    snprintf(response, sizeof(response),
             "RTSP/1.0 200 OK\r\n"
             "CSeq: %d\r\n"
             "Session: %s\r\n"
             "Range: npt=0.000-\r\n"
             "\r\n",
             cseq, session_id_.c_str());
    send_response(response);
    
    state_ = State::PLAYING;
}

void RTSPSession::handle_teardown(const std::string& request) {
    int cseq = get_cseq(request);
    
    char response[256];
    snprintf(response, sizeof(response),
             "RTSP/1.0 200 OK\r\n"
             "CSeq: %d\r\n"
             "Session: %s\r\n"
             "\r\n",
             cseq, session_id_.c_str());
    send_response(response);
    
    state_ = State::INIT;
}

void RTSPSession::send_frame(const uint8_t* frame_data, size_t frame_size) {
    if (state_ != State::PLAYING || rtp_packetizer_ == nullptr) {
        return;
    }

    ESP_LOGD(TAG, "Sending frame of size %zu", frame_size);
    rtp_packetizer_->packetize_and_send(frame_data, frame_size);
    last_activity_ = millis();
}

void RTSPSession::handle_request() {
    if (recv_buffer_pos_ >= BUFFER_SIZE) {
        ESP_LOGW(TAG, "Request too large");
        close_connection();
        return;
    }

    int ret = recv(socket_, recv_buffer_.data() + recv_buffer_pos_,
                    BUFFER_SIZE - recv_buffer_pos_ - 1, 0);
  if (ret < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      ESP_LOGD(TAG, "recv() returned EAGAIN/EWOULDBLOCK"); // Log this specifically
      return;
    }
      ESP_LOGE(TAG, "recv() failed: %d (%s)", errno, strerror(errno)); // More detailed logging
      close_connection();
      return;
    }
    if (ret == 0) {
        ESP_LOGD(TAG, "Client disconnected");
        close_connection();
        return;
    }

    recv_buffer_pos_ += ret;
    recv_buffer_[recv_buffer_pos_] = '\0';

    // Check for complete request
    for (size_t i = 0; i < recv_buffer_pos_ - 3; i++) {
        if (recv_buffer_[i] == '\r' && recv_buffer_[i+1] == '\n' &&
            recv_buffer_[i+2] == '\r' && recv_buffer_[i+3] == '\n') {
            std::string request(recv_buffer_.data());
            recv_buffer_pos_ = 0;
            recv_buffer_[0] = '\0';

            if (request.find("OPTIONS") == 0) {
                handle_options(request);
            } else if (request.find("DESCRIBE") == 0) {
                handle_describe(request);
            } else if (request.find("SETUP") == 0) {
                handle_setup(request);
            } else if (request.find("PLAY") == 0) {
                handle_play(request);
            } else if (request.find("TEARDOWN") == 0) {
                handle_teardown(request);
            } else {
                send_error(get_cseq(request), 400, "Bad Request");
            }
            return;
        }
    }
    return;
}

void RTSPSession::close_connection() {
    if (socket_ >= 0) {
        ESP_LOGD(TAG, "Closing connection for session %s", session_id_.c_str());
        close(socket_);
        socket_ = -1;
        rtp_packetizer_.reset();
    }
    state_ = State::INIT;
}

bool RTSPSession::parse_request_line(const std::string& line, std::string& method, 
                                 std::string& url, std::string& version) {
    size_t pos1 = line.find(' ');
    if (pos1 == std::string::npos) return false;
    method = line.substr(0, pos1);

    size_t pos2 = line.find(' ', pos1 + 1);
    if (pos2 == std::string::npos) return false;
    url = line.substr(pos1 + 1, pos2 - pos1 - 1);

    version = line.substr(pos2 + 1);
    return true;
}

bool RTSPSession::parse_header_line(const std::string& line, std::string& key, 
                                 std::string& value) {
    size_t pos = line.find(':');
    if (pos == std::string::npos) return false;
    key = line.substr(0, pos);
    value = line.substr(pos + 1);
    // Trim whitespace
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t") + 1);
    return true;
}

}  // namespace micro_rtsp
}  // namespace esphome
