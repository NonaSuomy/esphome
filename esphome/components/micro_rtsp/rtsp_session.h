// rtsp_session.h
#pragma once

#include "rtp_packetizer.h"
#include "esphome/core/hal.h"
#include <string>
#include <memory>
#include <mutex>
#include <array>

namespace esphome {
namespace micro_rtsp {

class RTSPSession {
public:
RTSPSession(int client_socket, uint16_t width = 320, uint16_t height = 240, uint16_t port = 8554);
RTSPSession(const RTSPSession&) = default; // Enable copy constructor
RTSPSession(RTSPSession&&) = default; // Enable move constructor
~RTSPSession();

    void handle_request();
    void send_frame(const uint8_t* frame_data, size_t frame_size);
    bool is_playing() const { return state_ == State::PLAYING; }
    bool is_alive() const;
    const std::string& get_session_id() const { return session_id_; }

private:
    static constexpr size_t BUFFER_SIZE = 2048;
    static constexpr uint32_t SESSION_TIMEOUT = 60000;  // 60 seconds
    static constexpr uint16_t DEFAULT_WIDTH = 320;
    static constexpr uint16_t DEFAULT_HEIGHT = 240;
    static constexpr uint16_t DEFAULT_FRAMERATE = 15;

    enum class State {
        INIT,
        READY,
        PLAYING
    };

    void process_request(const std::string& request);
    void handle_options(const std::string& request);
    void handle_describe(const std::string& request);
    void handle_setup(const std::string& request);
    void handle_play(const std::string& request);
    void handle_teardown(const std::string& request);
    void send_response(const char* response);
    void send_error(int cseq, int code, const char* message);
    void close_connection();
    void parse_transport(const std::string& transport);
    void send_sdp_response();
    std::string generate_session_id();
    bool parse_request_line(const std::string& line, std::string& method, 
                          std::string& url, std::string& version);
    bool parse_header_line(const std::string& line, std::string& key, 
                          std::string& value);
    std::string get_local_ip() const;
    int get_cseq(const std::string& request);
    std::string get_url(const std::string& request);

    int socket_;
    State state_{State::INIT};
    uint32_t last_activity_{0};
    std::string session_id_;
    std::string stream_path_;
    int cseq_{0};
    uint16_t client_rtp_port_{0};
    uint16_t client_rtcp_port_{0};
    uint16_t server_rtp_port_{0};
    uint16_t width_;
    uint16_t height_;
    std::unique_ptr<RTPPacketizer> rtp_packetizer_;
    std::array<char, BUFFER_SIZE> recv_buffer_;
    size_t recv_buffer_pos_{0};
    mutable std::mutex send_mutex_;
    uint16_t port_;
};

}  // namespace micro_rtsp
}  // namespace esphome
