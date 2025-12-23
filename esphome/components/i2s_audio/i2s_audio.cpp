#include "i2s_audio.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

namespace esphome {
namespace i2s_audio {

static const char *const TAG = "i2s_audio";

static const size_t DMA_BUFFERS_COUNT = 4;
static const size_t DMA_FRAME_COUNT = 480;

#if ESP_IDF_VERSION_MAJOR >= 5
static const uint8_t I2S_NUM_MAX = SOC_I2S_NUM;  // because IDF 5+ took this away :(
#endif

void I2SAudioComponent::setup() {
  ESP_LOGCONFIG(TAG, "Running setup");

  static i2s_port_t next_port_num = I2S_NUM_0;
  if (next_port_num >= I2S_NUM_MAX) {
    ESP_LOGE(TAG, "Too many components");
    this->mark_failed();
    return;
  }

  this->port_ = next_port_num;
  next_port_num = (i2s_port_t) (next_port_num + 1);

  this->initialize_channels_(true, true);
}

esp_err_t I2SAudioComponent::initialize_channels_(bool initialize_rx, bool initialize_tx) {
  if (this->initialized_) {
    return ESP_FAIL;
  }

  i2s_chan_config_t chan_cfg = {
      .id = this->port_,
      .role = this->i2s_role_,
      .dma_desc_num = DMA_BUFFERS_COUNT,
      .dma_frame_num = DMA_FRAME_COUNT,
      .auto_clear = true,
      .intr_priority = 3,
  };

  esp_err_t err = ESP_OK;

  const bool rx_available = (this->din_pin_ != I2S_GPIO_UNUSED);
  const bool tx_available = (this->dout_pin_ != I2S_GPIO_UNUSED);

  if (this->exclusive_) {
    if (initialize_tx && tx_available) {
      err = i2s_new_channel(&chan_cfg, &this->tx_handle_, NULL);
    } else if (initialize_rx && rx_available) {
      err = i2s_new_channel(&chan_cfg, NULL, &this->rx_handle_);
    }
  } else {
    if (rx_available && tx_available) {
      err = i2s_new_channel(&chan_cfg, &this->tx_handle_, &this->rx_handle_);
    } else if (tx_available) {
      err = i2s_new_channel(&chan_cfg, &this->tx_handle_, NULL);
    } else if (rx_available) {
      err = i2s_new_channel(&chan_cfg, NULL, &this->rx_handle_);
    }
  }

  if (err != ESP_OK) {
    this->status_set_error("Failed to setup channel(s)");
  } else {
    this->initialized_ = true;
  }

  return err;
}

esp_err_t I2SAudioComponent::setup_rx_handle(audio::AudioStreamInfo data_stream_info, uint8_t hardware_bits_per_sample,
                                             i2s_std_slot_mask_t slot_mask) {
  // TODO: Verify sample rate is okay when other channel is running

  if (this->din_pin_ == I2S_GPIO_UNUSED) {
    return ESP_FAIL;  // TODO: give a better error message
  }

  if (this->rx_handle_ == nullptr) {
    esp_err_t err = this->initialize_channels_(true, false);
    if (err != ESP_OK) {
      return err;
    }
  }

  const i2s_std_clk_config_t clock_config = this->get_clk_config_(data_stream_info.get_sample_rate());
  const i2s_std_slot_config_t slot_config =
      this->get_slot_config_(data_stream_info.get_bits_per_sample(), data_stream_info.get_channels(), slot_mask);

  if (this->rx_channel_setup_ && this->rx_channel_enabled_) {
    this->disable_rx_handle_();
    esp_err_t err = i2s_channel_reconfig_std_clock(this->rx_handle_, &clock_config);
    if (err == ESP_OK) {
      err = i2s_channel_reconfig_std_slot(this->rx_handle_, &slot_config);
    }
    this->enable_rx_handle_();
    return err;
  }

  this->output_bits_per_sample_ = hardware_bits_per_sample;
  this->sample_rate_ = data_stream_info.get_sample_rate();

  i2s_std_config_t std_cfg = {
      .clk_cfg = clock_config,
      .slot_cfg = slot_config,
      .gpio_cfg = this->get_pin_config_(),
  };

  esp_err_t err = i2s_channel_init_std_mode(this->rx_handle_, &std_cfg);
  if (err == ESP_OK) {
    this->rx_channel_setup_ = true;
    this->enable_rx_handle_();
    if ((this->dout_pin_ != I2S_GPIO_UNUSED) && !this->tx_channel_setup_) {
      err = this->setup_tx_handle(data_stream_info, hardware_bits_per_sample, slot_mask);
    }
  }

  return err;
}

void I2SAudioComponent::set_callbacks(i2s_event_callbacks_t callbacks, i2s_chan_handle_t channel_handle,
                                      void *user_context) {
  bool reenable = false;

  if (channel_handle == this->rx_handle_ && this->rx_channel_enabled_) {
    this->disable_rx_handle_();
    reenable = true;
  } else if (channel_handle == this->tx_handle_ && this->tx_channel_enabled_) {
    this->disable_tx_handle_();
    reenable = true;
  } else {
    return;
  }

  i2s_channel_register_event_callback(channel_handle, &callbacks, user_context);

  if (reenable && (channel_handle == this->rx_handle_)) {
    this->enable_rx_handle_();
  } else if (reenable && (channel_handle == this->tx_handle_)) {
    this->enable_tx_handle_();
  }
}

esp_err_t I2SAudioComponent::enable_rx_handle_() {
  if ((this->din_pin_ != I2S_GPIO_UNUSED) && !this->rx_channel_enabled_) {
    esp_err_t err = i2s_channel_enable(this->rx_handle_);
    if (err == ESP_OK) {
      this->rx_channel_enabled_ = true;
    }
    return err;
  }
  return ESP_OK;  // TODO: probably return a useful error
}

esp_err_t I2SAudioComponent::disable_rx_handle_() {
  if (this->rx_channel_enabled_) {
    esp_err_t err = i2s_channel_disable(this->rx_handle_);
    if (err == ESP_OK) {
      this->rx_channel_enabled_ = false;
    }
    return err;
  }
  return ESP_OK;  // TODO: probably return a useful error
}

esp_err_t I2SAudioComponent::setup_tx_handle(audio::AudioStreamInfo data_stream_info, uint8_t hardware_bits_per_sample,
                                             i2s_std_slot_mask_t slot_mask) {
  // TODO: Verify sample rate is okay when other channel is running

  if (this->dout_pin_ == I2S_GPIO_UNUSED) {
    return ESP_FAIL;  // TODO: give a better error message
  }

  if (this->tx_handle_ == nullptr) {
    esp_err_t err = this->initialize_channels_(false, true);
    if (err != ESP_OK) {
      return err;
    }
  }

  const i2s_std_clk_config_t clock_config = this->get_clk_config_(data_stream_info.get_sample_rate());
  const i2s_std_slot_config_t slot_config =
      this->get_slot_config_(data_stream_info.get_bits_per_sample(), data_stream_info.get_channels(), slot_mask);

  if (this->tx_channel_setup_) {
    this->disable_tx_handle_();
    esp_err_t err = i2s_channel_reconfig_std_clock(this->tx_handle_, &clock_config);
    if (err == ESP_OK) {
      err = i2s_channel_reconfig_std_slot(this->tx_handle_, &slot_config);
    }
    this->enable_tx_handle_();
    return err;
  }

  this->output_bits_per_sample_ = hardware_bits_per_sample;
  this->sample_rate_ = data_stream_info.get_sample_rate();

  i2s_std_config_t std_cfg = {
      .clk_cfg = clock_config,
      .slot_cfg = slot_config,
      .gpio_cfg = this->get_pin_config_(),
  };

  esp_err_t err = i2s_channel_init_std_mode(this->tx_handle_, &std_cfg);
  if (err == ESP_OK) {
    this->tx_channel_setup_ = true;
    this->enable_tx_handle_();
    if ((this->din_pin_ != I2S_GPIO_UNUSED) && !this->rx_channel_setup_) {
      err = this->setup_rx_handle(data_stream_info, hardware_bits_per_sample, slot_mask);
    }
  }

  return err;
}

esp_err_t I2SAudioComponent::enable_tx_handle_() {
  if ((this->dout_pin_ != I2S_GPIO_UNUSED) && !this->tx_channel_enabled_) {
    esp_err_t err = i2s_channel_enable(this->tx_handle_);
    if (err == ESP_OK) {
      this->tx_channel_enabled_ = true;
    }
    return err;
  }
  return ESP_OK;  // TODO: probably return a useful error
}

esp_err_t I2SAudioComponent::disable_tx_handle_() {
  if (this->tx_channel_enabled_) {
    esp_err_t err = i2s_channel_disable(this->tx_handle_);
    if (err == ESP_OK) {
      this->tx_channel_enabled_ = false;
    }
    return err;
  }
  return ESP_OK;  // TODO: probably return a useful error
}

i2s_std_clk_config_t I2SAudioComponent::get_clk_config_(uint32_t sample_rate) const {
  i2s_clock_src_t clk_src = I2S_CLK_SRC_DEFAULT;

#ifdef I2S_CLK_SRC_APLL
  if (this->use_apll_) {
    clk_src = I2S_CLK_SRC_APLL;
  }
#endif

#ifdef I2S_CLK_SRC_EXTERNAL
  if (this->external_clk_freq_ > 0) {
    clk_src = I2S_CLK_SRC_EXTERNAL;
  }
#endif

  i2s_std_clk_config_t clk_cfg = {
      .sample_rate_hz = sample_rate,
      .clk_src = clk_src,
      .mclk_multiple = this->mclk_multiple_,
#ifdef I2S_CLK_SRC_EXTERNAL
      .ext_clk_freq_hz = this->external_clk_freq_,
#endif
  };

  return clk_cfg;
}

i2s_std_gpio_config_t I2SAudioComponent::get_pin_config_() const {
  return {.mclk = (gpio_num_t) this->mclk_pin_,
          .bclk = (gpio_num_t) this->bclk_pin_,
          .ws = (gpio_num_t) this->lrclk_pin_,
          .dout = (gpio_num_t) this->dout_pin_,
          .din = (gpio_num_t) this->din_pin_,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false,
          }};
}

i2s_std_slot_config_t I2SAudioComponent::get_slot_config_(uint8_t bits_per_sample, uint8_t channels,
                                                          i2s_std_slot_mask_t slot_mask) const {
  i2s_slot_mode_t slot_mode = I2S_SLOT_MODE_MONO;
  if (channels == 2) {
    slot_mode = I2S_SLOT_MODE_STEREO;
  }

  i2s_std_slot_config_t std_slot_cfg;
  if (this->i2s_comm_fmt_ == "std") {
    std_slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) bits_per_sample, slot_mode);
  } else if (this->i2s_comm_fmt_ == "pcm") {
    std_slot_cfg = I2S_STD_PCM_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) bits_per_sample, slot_mode);
  } else {
    std_slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t) bits_per_sample, slot_mode);
  }

#ifdef USE_ESP32_VARIANT_ESP32
  // There seems to be a bug on the ESP32 (non-variant) platform where setting the slot bit width higher then the bits
  // per sample causes the audio to play too fast. Setting the ws_width to the configured slot bit width seems to
  // make it play at the correct speed while sending more bits per slot.
  uint32_t configured_bit_width = static_cast<uint32_t>(this->output_bits_per_sample_);
  std_slot_cfg.ws_width = configured_bit_width;
  if (configured_bit_width > 16) {
    std_slot_cfg.msb_right = false;
  }

#else
  std_slot_cfg.slot_bit_width = (i2s_slot_bit_width_t) this->output_bits_per_sample_;
#endif
  std_slot_cfg.slot_mask = slot_mask;

  return std_slot_cfg;
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
