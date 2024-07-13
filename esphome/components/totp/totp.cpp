// totp.cpp
#include "totp.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>
#include <iomanip>
#include <sstream>
#include <vector>

namespace esphome {
namespace totp {

static const char *TAG = "totp";

std::array<uint8_t, 20> SHA1::compute(const uint8_t* data, size_t length) {
  uint32_t state[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
  uint8_t buffer[64];
  size_t i;
  uint64_t bit_len = length * 8;

  for (i = 0; i < length - 63; i += 64) {
    std::memcpy(buffer, &data[i], 64);
    transform(state, buffer);
  }

  size_t remain = length - i;
  std::memcpy(buffer, &data[i], remain);
  buffer[remain] = 0x80;

  if (remain > 55) {
    std::memset(&buffer[remain + 1], 0, 63 - remain);
    transform(state, buffer);
    std::memset(buffer, 0, 56);
  } else {
    std::memset(&buffer[remain + 1], 0, 55 - remain);
  }

  for (int j = 0; j < 8; ++j) {
    buffer[63 - j] = (bit_len >> (j * 8)) & 0xFF;
  }
  transform(state, buffer);

  std::array<uint8_t, 20> result;
  for (int j = 0; j < 20; ++j) {
    result[j] = (state[j / 4] >> ((3 - (j % 4)) * 8)) & 0xFF;
  }
  return result;
}

void SHA1::transform(uint32_t state[5], const uint8_t buffer[64]) {
  uint32_t a, b, c, d, e, m[80];
  
  for (int i = 0; i < 16; ++i) {
    m[i] = (buffer[i * 4] << 24) | (buffer[i * 4 + 1] << 16) | (buffer[i * 4 + 2] << 8) | (buffer[i * 4 + 3]);
  }

  for (int i = 16; i < 80; ++i) {
    m[i] = (m[i - 3] ^ m[i - 8] ^ m[i - 14] ^ m[i - 16]);
    m[i] = (m[i] << 1) | (m[i] >> 31);
  }

  a = state[0];
  b = state[1];
  c = state[2];
  d = state[3];
  e = state[4];

  for (int i = 0; i < 80; ++i) {
    uint32_t f, k;
    if (i < 20) {
      f = (b & c) | ((~b) & d);
      k = 0x5A827999;
    } else if (i < 40) {
      f = b ^ c ^ d;
      k = 0x6ED9EBA1;
    } else if (i < 60) {
      f = (b & c) | (b & d) | (c & d);
      k = 0x8F1BBCDC;
    } else {
      f = b ^ c ^ d;
      k = 0xCA62C1D6;
    }
    uint32_t temp = ((a << 5) | (a >> 27)) + f + e + k + m[i];
    e = d;
    d = c;
    c = (b << 30) | (b >> 2);
    b = a;
    a = temp;
  }

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
}

void TOTPComponent::setup() {
  if (this->time_ == nullptr) {
    ESP_LOGE(TAG, "Time component not set!");
  }
}

void TOTPComponent::loop() {
  if (this->time_ == nullptr) return;
  auto now = this->time_->now();
  if (!now.is_valid()) return;
  uint32_t current_time = now.timestamp;
  
  if (current_time - this->last_update_ >= 1) {
    this->last_update_ = current_time;
    
    std::string current_totp = this->get_current_totp();
    float countdown = this->get_countdown();
    
    // Publish TOTP if it has changed or if it's the first time
    if (current_totp != this->last_published_totp_ && this->totp_ != nullptr) {
      this->totp_->publish_state(current_totp);
      this->last_published_totp_ = current_totp;
    }
    
    if (this->countdown_ != nullptr) {
      this->countdown_->publish_state(countdown);
    }
  }
}

std::string TOTPComponent::get_current_totp() {
  if (this->time_ == nullptr) return "000000";
  
  auto now = this->time_->now();
  if (!now.is_valid()) return "000000";
  
  uint32_t time_step = now.timestamp / 30;  // Time step of 30 seconds
  
  if (time_step != this->last_time_step_) {
    this->last_time_step_ = time_step;
    this->last_totp_ = generate_totp_();
  }
  
  return this->last_totp_;
}

float TOTPComponent::get_countdown() const {
  if (this->time_ == nullptr) return 30.0f;
  auto time = this->time_->now();
  if (!time.is_valid()) return 30.0f;
  return 30.0f - (time.timestamp % 30);
}

std::string TOTPComponent::generate_totp_() {
  if (this->time_ == nullptr) {
    ESP_LOGE(TAG, "Time component not available");
    return "000000";
  }
  auto time = this->time_->now();
  if (!time.is_valid()) {
    ESP_LOGE(TAG, "Current time is not valid");
    return "000000";
  }
  uint32_t now = time.timestamp;
  uint32_t time_step = now / 30;  // Time step of 30 seconds
  this->last_time_step_ = time_step;

  uint8_t time_bytes[8];
  for (int i = 7; i >= 0; i--) {
    time_bytes[i] = time_step & 0xFF;
    time_step >>= 8;
  }

  std::vector<uint8_t> key(this->secret_.begin(), this->secret_.end());
  std::vector<uint8_t> msg(time_bytes, time_bytes + 8);

  // HMAC-SHA1
  std::vector<uint8_t> o_key_pad(64, 0x5c);
  std::vector<uint8_t> i_key_pad(64, 0x36);

  for (size_t i = 0; i < key.size(); ++i) {
    o_key_pad[i] ^= key[i];
    i_key_pad[i] ^= key[i];
  }

  i_key_pad.insert(i_key_pad.end(), msg.begin(), msg.end());
  auto inner_hash = SHA1::compute(i_key_pad.data(), i_key_pad.size());

  o_key_pad.insert(o_key_pad.end(), inner_hash.begin(), inner_hash.end());
  auto hmac = SHA1::compute(o_key_pad.data(), o_key_pad.size());

  int offset = hmac[19] & 0xF;
  uint32_t code = (hmac[offset] & 0x7F) << 24 |
                  (hmac[offset + 1] & 0xFF) << 16 |
                  (hmac[offset + 2] & 0xFF) << 8 |
                  (hmac[offset + 3] & 0xFF);
  code = code % 1000000;

  std::stringstream ss;
  ss << std::setw(6) << std::setfill('0') << code;
  this->last_totp_ = ss.str();
  
  return this->last_totp_;
}

}  // namespace totp
}  // namespace esphome