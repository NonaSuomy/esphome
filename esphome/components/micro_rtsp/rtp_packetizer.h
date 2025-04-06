// rtp_packetizer.h
#pragma once

#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstdint>
#include <array>
#include <memory>
#include <lwip/sockets.h>
#include <errno.h>

namespace esphome {
namespace micro_rtsp {

class RTPPacketizer {
public:
    RTPPacketizer(int client_socket, uint16_t client_rtp_port, uint16_t client_rtcp_port);
    ~RTPPacketizer();

    void packetize_and_send(const uint8_t* frame_data, size_t frame_size);
    uint16_t get_server_rtp_port() const { return server_rtp_port_; }
    uint16_t get_server_rtcp_port() const { return server_rtcp_port_; }
    uint32_t get_ssrc() const { return ssrc_; }
    uint16_t get_sequence_number() const { return sequence_number_; }
    uint32_t get_timestamp() const { return timestamp_; }
    bool is_valid() const { return rtp_socket_ >= 0; }

private:
    static constexpr size_t RTP_HEADER_SIZE = 12;
    static constexpr size_t MAX_PACKET_SIZE = 1400;  // Keep below MTU
    static constexpr uint8_t RTP_VERSION = 2;
    static constexpr uint8_t JPEG_PAYLOAD_TYPE = 26;
    static constexpr uint8_t JPEG_TYPE_BASELINE = 1;
    static constexpr size_t MAX_FRAGMENT_SIZE = MAX_PACKET_SIZE - RTP_HEADER_SIZE - 8;  // 8 for JPEG header

    struct RTPHeader {
        uint8_t version:2;
        uint8_t padding:1;
        uint8_t extension:1;
        uint8_t csrc_count:4;
        uint8_t marker:1;
        uint8_t payload_type:7;
        uint16_t sequence_number;
        uint32_t timestamp;
        uint32_t ssrc;
    } __attribute__((packed));

    struct JPEGHeader {
        uint8_t type_specific;
        uint8_t fragment_offset[3];
        uint8_t type;
        uint8_t q;
        uint8_t width;
        uint8_t height;
    } __attribute__((packed));

    void create_rtp_socket();
    void create_rtcp_socket();
    bool send_rtp_packet(const uint8_t* data, size_t size, bool marker);
    void send_rtcp_sr();
    uint32_t get_ntp_timestamp();
    void extract_jpeg_info(const uint8_t* jpeg_data, size_t jpeg_size);

    int client_socket_;
    uint16_t client_rtp_port_;
    uint16_t client_rtcp_port_;
    int rtp_socket_;
    int rtcp_socket_;
    uint16_t server_rtp_port_;
    uint16_t server_rtcp_port_;
    uint16_t sequence_number_{0};
    uint32_t timestamp_{0};
    uint32_t ssrc_;
    uint32_t packet_count_{0};
    uint32_t octet_count_{0};
    uint8_t jpeg_q_{255};
    uint8_t jpeg_width_{0};
    uint8_t jpeg_height_{0};
    uint8_t *packet_buffer_;
};

}  // namespace micro_rtsp
}  // namespace esphome
