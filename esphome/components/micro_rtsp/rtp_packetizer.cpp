// rtp_packetizer.cpp
#include "rtp_packetizer.h"
#include <random>
#include <algorithm>

namespace esphome {
namespace micro_rtsp {

static const char *const TAG = "rtp_packetizer";

RTPPacketizer::RTPPacketizer(int client_socket, uint16_t client_rtp_port, uint16_t client_rtcp_port)
    : client_socket_(client_socket)
    , client_rtp_port_(client_rtp_port)
    , client_rtcp_port_(client_rtcp_port)
    , rtp_socket_(-1)
    , rtcp_socket_(-1) {
    
    // Generate random SSRC
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint32_t> dis;
    ssrc_ = dis(gen);
    
    packet_buffer_ = new uint8_t[MAX_PACKET_SIZE];

    create_rtp_socket();
    create_rtcp_socket();
}

RTPPacketizer::~RTPPacketizer() {
    if (rtp_socket_ >= 0) {
        close(rtp_socket_);
    }
    if (rtcp_socket_ >= 0) {
        close(rtcp_socket_);
    }
    delete[] packet_buffer_;
}

void RTPPacketizer::create_rtp_socket() {
    rtp_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rtp_socket_ < 0) {
        ESP_LOGE(TAG, "Failed to create RTP socket");
        return;
    }

    // Add socket options for better reliability
    int enable = 1;
    if (setsockopt(rtp_socket_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
        ESP_LOGW(TAG, "Failed to set SO_REUSEADDR on RTP socket");
    }

    // Set send buffer size
    int sendbuf = 32768;//65535;
    if (setsockopt(rtp_socket_, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(sendbuf)) < 0) {
        ESP_LOGW(TAG, "Failed to set SO_SNDBUF on RTP socket: %s", strerror(errno));
    }

    // Bind to specific port
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(0);  // Let system choose port

    if (bind(rtp_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind RTP socket: %d", errno);
        close(rtp_socket_);
        rtp_socket_ = -1;
        return;
    }

    // Get assigned port
    socklen_t addr_len = sizeof(addr);
    if (getsockname(rtp_socket_, (struct sockaddr*)&addr, &addr_len) < 0) {
        ESP_LOGE(TAG, "Failed to get socket name: %d", errno);
        close(rtp_socket_);
        rtp_socket_ = -1;
        return;
    }
    server_rtp_port_ = ntohs(addr.sin_port);
    ESP_LOGD(TAG, "RTP socket bound to port %d", server_rtp_port_);
}

void RTPPacketizer::create_rtcp_socket() {
    rtcp_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rtcp_socket_ < 0) {
        ESP_LOGE(TAG, "Failed to create RTCP socket");
        return;
    }

    int enable = 1;
    if (setsockopt(rtcp_socket_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
        ESP_LOGW(TAG, "Failed to set SO_REUSEADDR on RTCP socket");
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(server_rtp_port_ + 1);

    if (bind(rtcp_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind RTCP socket: %d", errno);
        close(rtcp_socket_);
        rtcp_socket_ = -1;
        return;
    }

    socklen_t addr_len = sizeof(addr);
    if (getsockname(rtcp_socket_, (struct sockaddr*)&addr, &addr_len) < 0) {
        ESP_LOGE(TAG, "Failed to get RTCP socket name: %d", errno);
        close(rtcp_socket_);
        rtcp_socket_ = -1;
        return;
    }
    server_rtcp_port_ = ntohs(addr.sin_port);
    ESP_LOGD(TAG, "RTCP socket bound to port %d", server_rtcp_port_);
}

void RTPPacketizer::packetize_and_send(const uint8_t* frame_data, size_t frame_size) {
    if (!frame_data || frame_size == 0 || rtp_socket_ < 0) return;

    // Verify JPEG header
    if (frame_data[0] != 0xFF || frame_data[1] != 0xD8) {
        ESP_LOGW(TAG, "Invalid JPEG header");
        return;
    }

    // Extract JPEG dimensions
    extract_jpeg_info(frame_data, frame_size);

    // Increment timestamp (90kHz clock)
    timestamp_ += 90000 / 15;  // 15 fps

    size_t offset = 0;
    bool first_fragment = true;

    while (offset < frame_size) {
        size_t remaining = frame_size - offset;
        size_t payload_size = std::min(remaining, MAX_FRAGMENT_SIZE);
        bool is_last = (offset + payload_size) >= frame_size;

        // Prepare RTP packet
        RTPHeader* rtp = reinterpret_cast<RTPHeader*>(packet_buffer_);
        rtp->version = RTP_VERSION;
        rtp->padding = 0;
        rtp->extension = 0;
        rtp->csrc_count = 0;
        rtp->marker = is_last ? 1 : 0;
        rtp->payload_type = JPEG_PAYLOAD_TYPE;
        rtp->sequence_number = htons(sequence_number_++);
        rtp->timestamp = htonl(timestamp_);
        rtp->ssrc = htonl(ssrc_);

        // Add JPEG header
        JPEGHeader* jpeg = reinterpret_cast<JPEGHeader*>(packet_buffer_ + RTP_HEADER_SIZE);
        jpeg->type_specific = 0;
        jpeg->fragment_offset[0] = (offset >> 16) & 0xFF;
        jpeg->fragment_offset[1] = (offset >> 8) & 0xFF;
        jpeg->fragment_offset[2] = offset & 0xFF;
        jpeg->type = JPEG_TYPE_BASELINE;
        jpeg->q = jpeg_q_;
        jpeg->width = jpeg_width_;
        jpeg->height = jpeg_height_;

        // Copy JPEG data
        memcpy(packet_buffer_ + RTP_HEADER_SIZE + sizeof(JPEGHeader),
               frame_data + offset,
               payload_size);

        // Send packet
        size_t packet_size = RTP_HEADER_SIZE + sizeof(JPEGHeader) + payload_size;
        if (!send_rtp_packet(packet_buffer_, packet_size, is_last)) {
            ESP_LOGW(TAG, "Failed to send RTP packet");
            return;
        }

        offset += payload_size;
        first_fragment = false;
    }

    // Send RTCP SR periodically
    packet_count_++;
    if (packet_count_ % 450 == 0) {  // Every 30 seconds at 15 fps
        send_rtcp_sr();
    }
}

bool RTPPacketizer::send_rtp_packet(const uint8_t* data, size_t size, bool marker) {
    if (rtp_socket_ < 0) return false;

    struct sockaddr_in client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(client_rtp_port_);

    // Get client IP from client socket
    struct sockaddr_in peer_addr;
    socklen_t peer_len = sizeof(peer_addr);
    if (getpeername(client_socket_, (struct sockaddr*)&peer_addr, &peer_len) < 0) {
        ESP_LOGW(TAG, "Failed to get peer address: %d", errno);
        return false;
    }
    client_addr.sin_addr = peer_addr.sin_addr;

    // Try to send with retries
    int retries = 3;
    while (retries > 0) {
        ssize_t sent = sendto(rtp_socket_, data, size, 0,
                            (struct sockaddr*)&client_addr, sizeof(client_addr));
        if (sent >= 0) {
            octet_count_ += size;
            ESP_LOGV(TAG, "RTP: seq=%u, ts=%lu, marker=%d, size=%zu", // Changed %u to %lu
              ntohs(reinterpret_cast<const RTPHeader*>(data)->sequence_number),
              ntohl(reinterpret_cast<const RTPHeader*>(data)->timestamp),
              marker, size);
            return true;
        }
        
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "RTP send failed: %d", errno);
            return false;
        }
        
        retries--;
        delay(1);  // Small delay before retry
    }

    ESP_LOGW(TAG, "RTP send failed after retries");
    return false;
}

void RTPPacketizer::send_rtcp_sr() {
    if (rtcp_socket_ < 0) return;

    uint8_t rtcp_packet[28];
    memset(rtcp_packet, 0, sizeof(rtcp_packet));

    // RTCP header
    rtcp_packet[0] = (RTP_VERSION << 6);  // Version 2, no padding
    rtcp_packet[1] = 200;   // SR packet type
    rtcp_packet[2] = 0;     // Length high byte
    rtcp_packet[3] = 6;     // Length low byte (6 32-bit words minus 1)

    // SSRC
    uint32_t ssrc_n = htonl(ssrc_);
    memcpy(rtcp_packet + 4, &ssrc_n, 4);

    // NTP timestamp
    uint32_t ntp_ts = get_ntp_timestamp();
    uint32_t ntp_ts_n = htonl(ntp_ts);
    memcpy(rtcp_packet + 8, &ntp_ts_n, 4);
    memcpy(rtcp_packet + 12, &ntp_ts_n, 4);

    // RTP timestamp
    uint32_t rtp_ts_n = htonl(timestamp_);
    memcpy(rtcp_packet + 16, &rtp_ts_n, 4);

    // Packet and octet counts
    uint32_t packet_count_n = htonl(packet_count_);
    uint32_t octet_count_n = htonl(octet_count_);
    memcpy(rtcp_packet + 20, &packet_count_n, 4);
    memcpy(rtcp_packet + 24, &octet_count_n, 4);

    struct sockaddr_in client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(client_rtcp_port_);

    // Get client IP
    struct sockaddr_in peer_addr;
    socklen_t peer_len = sizeof(peer_addr);
    if (getpeername(client_socket_, (struct sockaddr*)&peer_addr, &peer_len) >= 0) {
        client_addr.sin_addr = peer_addr.sin_addr;
        sendto(rtcp_socket_, rtcp_packet, sizeof(rtcp_packet), 0,
               (struct sockaddr*)&client_addr, sizeof(client_addr));
    }
}

void RTPPacketizer::extract_jpeg_info(const uint8_t* jpeg_data, size_t jpeg_size) {
    size_t pos = 2;  // Skip SOI marker
    while (pos + 8 < jpeg_size) {
        if (jpeg_data[pos] != 0xFF) {
            pos++;
            continue;
        }

        uint8_t marker = jpeg_data[pos + 1];
        uint16_t segment_size = (jpeg_data[pos + 2] << 8) | jpeg_data[pos + 3];

        if (marker == 0xC0) {  // SOF0 marker
            jpeg_height_ = ((jpeg_data[pos + 5] << 8) | jpeg_data[pos + 6]) >> 3;
            jpeg_width_ = ((jpeg_data[pos + 7] << 8) | jpeg_data[pos + 8]) >> 3;
            ESP_LOGV(TAG, "JPEG dimensions: %dx%d blocks", jpeg_width_, jpeg_height_);
            break;
        }

        if (segment_size < 2) break;
        pos += 2 + segment_size;
    }
}

uint32_t RTPPacketizer::get_ntp_timestamp() {
    return (esphome::millis() / 1000) + 2208988800UL;  // Convert to NTP timestamp
}

}  // namespace micro_rtsp
}  // namespace esphome
