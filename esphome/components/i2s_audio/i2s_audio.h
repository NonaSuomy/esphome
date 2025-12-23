#pragma once

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include <driver/i2s_std.h>

namespace esphome {
namespace i2s_audio {

class I2SAudioComponent;

class I2SAudioBase : public Parented<I2SAudioComponent> {
 public:
  void set_bits_per_sample(uint8_t bits_per_sample) { this->bits_per_sample_ = bits_per_sample; }
  void set_sample_rate(uint32_t sample_rate) { this->sample_rate_ = sample_rate; }
  void set_std_slot_mask(i2s_std_slot_mask_t std_slot_mask) { this->std_slot_mask_ = std_slot_mask; }

 protected:
  uint8_t bits_per_sample_;
  uint32_t sample_rate_;
  i2s_std_slot_mask_t std_slot_mask_;
};

class I2SAudioComponent : public Component {
 public:
  void setup() override;

  void set_external_clk_freq(uint32_t external_clk_freq) { this->external_clk_freq_ = external_clk_freq; }
  void set_i2s_comm_fmt(std::string mode) { this->i2s_comm_fmt_ = std::move(mode); }
  void set_i2s_role(i2s_role_t role) { this->i2s_role_ = role; }
  void set_mclk_multiple(i2s_mclk_multiple_t mclk_multiple) { this->mclk_multiple_ = mclk_multiple; }
  void set_use_apll(uint32_t use_apll) { this->use_apll_ = use_apll; }

  void set_din_pin(int pin) { this->din_pin_ = pin; }
  void set_dout_pin(int pin) { this->dout_pin_ = pin; }
  void set_mclk_pin(int pin) { this->mclk_pin_ = pin; }
  void set_bclk_pin(int pin) { this->bclk_pin_ = pin; }
  void set_lrclk_pin(int pin) { this->lrclk_pin_ = pin; }

  i2s_port_t get_port() const { return this->port_; }

  esp_err_t setup_rx_handle(audio::AudioStreamInfo data_stream_info, uint8_t hardware_bits_per_sample,
                            i2s_std_slot_mask_t slot_mask);

  i2s_chan_handle_t get_rx_handle() { return this->rx_handle_; }
  bool get_rx_channel_enabled() { return this->rx_channel_enabled_; }

  esp_err_t setup_tx_handle(audio::AudioStreamInfo data_stream_info, uint8_t hardware_bits_per_sample,
                            i2s_std_slot_mask_t slot_mask);

  i2s_chan_handle_t get_tx_handle() { return this->tx_handle_; }
  bool get_tx_channel_enabled() { return this->tx_channel_enabled_; }

  void set_callbacks(i2s_event_callbacks_t callbacks, i2s_chan_handle_t channel_handle, void *user_context);

  void set_exclusive(bool exclusive) { this->exclusive_ = exclusive; }

 protected:
  i2s_std_clk_config_t get_clk_config_(uint32_t sample_rate) const;
  i2s_std_gpio_config_t get_pin_config_() const;
  i2s_std_slot_config_t get_slot_config_(uint8_t bits_per_sample, uint8_t channels,
                                         i2s_std_slot_mask_t slot_mask) const;

  esp_err_t enable_rx_handle_();
  esp_err_t disable_rx_handle_();
  esp_err_t enable_tx_handle_();
  esp_err_t disable_tx_handle_();

  esp_err_t initialize_channels_(bool initialize_rx, bool initialize_tx);

  uint32_t external_clk_freq_;
  std::string i2s_comm_fmt_;
  i2s_role_t i2s_role_{};
  i2s_mclk_multiple_t mclk_multiple_;

  i2s_chan_handle_t rx_handle_{nullptr};
  i2s_chan_handle_t tx_handle_{nullptr};

  bool use_apll_;
  bool rx_channel_enabled_{false};
  bool rx_channel_setup_{false};
  bool tx_channel_enabled_{false};
  bool tx_channel_setup_{false};
  bool exclusive_;
  bool initialized_{false};

  uint8_t output_bits_per_sample_;
  uint32_t sample_rate_;

  int bclk_pin_{I2S_GPIO_UNUSED};
  int din_pin_{I2S_GPIO_UNUSED};
  int dout_pin_{I2S_GPIO_UNUSED};
  int lrclk_pin_;
  int mclk_pin_{I2S_GPIO_UNUSED};

  i2s_port_t port_{};
};

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
