// totp.h
#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include <array>

namespace esphome {
namespace totp {

class SHA1 {
 public:
  static std::array<uint8_t, 20> compute(const uint8_t* data, size_t length);
 private:
  static void transform(uint32_t state[5], const uint8_t buffer[64]);
};

class TOTPComponent : public Component {
 public:
  void set_secret(const std::string &secret) { this->secret_ = secret; }
  void set_time(time::RealTimeClock *time) { this->time_ = time; }
  void set_totp(text_sensor::TextSensor *totp) { this->totp_ = totp; }
  void set_countdown(sensor::Sensor *countdown) { this->countdown_ = countdown; }
  void setup() override;
  void loop() override;
  std::string get_current_totp();
  float get_countdown() const;

 protected:
  std::string secret_;
  time::RealTimeClock *time_{nullptr};
  text_sensor::TextSensor *totp_{nullptr};
  sensor::Sensor *countdown_{nullptr};
  uint32_t last_time_step_{0};
  std::string last_totp_{"000000"};
  uint32_t last_update_{0};
  std::string last_published_totp_{"000000"};
  std::string generate_totp_();
};

}  // namespace totp
}  // namespace esphome