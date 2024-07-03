#pragma once

#include "esphome.h"
#include "esphome/components/pn532/pn532.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace yubikey_otp {

class YubikeyOtpSensor : public text_sensor::TextSensor, public PollingComponent {
 public:
  YubikeyOtpSensor() : PollingComponent(10000) {}  // 10 seconds polling interval

  void set_pn532(pn532::PN532 *pn532) { pn532_ = pn532; }

  void update() override {
    ESP_LOGD("yubikey_otp", "YubikeyOtpSensor::update() called");
    if (pn532_ == nullptr) {
      ESP_LOGE("yubikey_otp", "PN532 component not set");
      return;
    }
    std::string otp = pn532_->read_yubikey_otp();
    if (!otp.empty()) {
      ESP_LOGD("yubikey_otp", "OTP Code: %s", otp.c_str());
      publish_state(otp);
    } else {
      ESP_LOGD("yubikey_otp", "No OTP code retrieved");
    }
  }

 private:
  pn532::PN532 *pn532_{nullptr};
};

}  // namespace yubikey_otp
}  // namespace esphome
