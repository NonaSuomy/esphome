#include "es8311.h"
#include "es8311_const.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cinttypes>

namespace esphome {
namespace es8311 {

static const char *const TAG = "es8311";

// Mark the component as failed; use only in setup
#define ES8311_ERROR_FAILED(func) \
  if (!(func)) { \
    this->mark_failed(); \
    return; \
  }
// Return false; use outside of setup
#define ES8311_ERROR_CHECK(func) \
  if (!(func)) { \
    return false; \
  }

// Using write_byte from I2CDevice base class instead

void ES8311::setup() {
  ESP_LOGCONFIG(TAG, "Initializing ES8311 codec...");

  // Check if ES8311 is present
  if (this->write(nullptr, 0, true) != esphome::i2c::ERROR_OK) {
    ESP_LOGE(TAG, "ES8311 not found at address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "ES8311 found at address 0x%02X", this->address_);

  // --- Initialization sequence from the working YAML example ---
  ES8311_ERROR_FAILED(this->write_byte(0x44, 0x08));
  delay(10);
  ES8311_ERROR_FAILED(this->write_byte(0x44, 0x08));

  ES8311_ERROR_FAILED(this->write_byte(0x01, 0x30));
  ES8311_ERROR_FAILED(this->write_byte(0x02, 0x00));
  ES8311_ERROR_FAILED(this->write_byte(0x03, 0x10));
  ES8311_ERROR_FAILED(this->write_byte(0x16, 0x24));
  ES8311_ERROR_FAILED(this->write_byte(0x04, 0x10));
  ES8311_ERROR_FAILED(this->write_byte(0x05, 0x00));
  ES8311_ERROR_FAILED(this->write_byte(0x0B, 0x00));
  ES8311_ERROR_FAILED(this->write_byte(0x0C, 0x00));
  ES8311_ERROR_FAILED(this->write_byte(0x10, 0x1F));
  ES8311_ERROR_FAILED(this->write_byte(0x11, 0x7F));
  ES8311_ERROR_FAILED(this->write_byte(0x00, 0x80));
  delay(10);

  ES8311_ERROR_FAILED(this->write_byte(0x01, 0x3F));
  ES8311_ERROR_FAILED(this->write_byte(0x06, 0x00));

  ES8311_ERROR_FAILED(this->write_byte(0x13, 0x10));
  ES8311_ERROR_FAILED(this->write_byte(0x1B, 0x0A));
  ES8311_ERROR_FAILED(this->write_byte(0x1C, 0x6A));
  ES8311_ERROR_FAILED(this->write_byte(0x44, 0x58));

  // --- Configuration for 16-bit, I2S Normal, 16kHz sample rate ---
  ES8311_ERROR_FAILED(this->write_byte(0x09, 0x0C));
  ES8311_ERROR_FAILED(this->write_byte(0x0A, 0x0C));

  // For 16kHz sample rate (assuming 12.288MHz MCLK)
  ES8311_ERROR_FAILED(this->write_byte(0x02, 0x40));
  ES8311_ERROR_FAILED(this->write_byte(0x05, 0x00));
  ES8311_ERROR_FAILED(this->write_byte(0x03, 0x10));
  ES8311_ERROR_FAILED(this->write_byte(0x04, 0x20));
  ES8311_ERROR_FAILED(this->write_byte(0x07, 0x00));
  ES8311_ERROR_FAILED(this->write_byte(0x08, 0xFF));
  ES8311_ERROR_FAILED(this->write_byte(0x06, 0x03));

  // --- Enable codec (from es8311_start) ---
  ES8311_ERROR_FAILED(this->write_byte(0x09, 0x0C));
  ES8311_ERROR_FAILED(this->write_byte(0x0A, 0x0C));

  ES8311_ERROR_FAILED(this->write_byte(0x17, 0xBF));
  ES8311_ERROR_FAILED(this->write_byte(0x0E, 0x02));
  ES8311_ERROR_FAILED(this->write_byte(0x12, 0x00));
  ES8311_ERROR_FAILED(this->write_byte(0x14, 0x1A));
  ES8311_ERROR_FAILED(this->write_byte(0x0D, 0x01));
  ES8311_ERROR_FAILED(this->write_byte(0x15, 0x40));
  ES8311_ERROR_FAILED(this->write_byte(0x37, 0x08));
  ES8311_ERROR_FAILED(this->write_byte(0x45, 0x00));
  
  // Set microphone gain to 30dB (0x05) as in the working example
  ES8311_ERROR_FAILED(this->write_byte(0x16, 0x05));

  // Set initial volume
  this->set_volume(0.75);  // 0.75 = 0xBF = 0dB

  ESP_LOGI(TAG, "ES8311 initialization complete.");
  delay(100);
}

void ES8311::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ES8311 Audio Codec:\n"
                "  Use MCLK: %s\n"
                "  Use Microphone: %s\n"
                "  Microphone Type: %s\n"
                "  DAC Bits per Sample: %d\n"
                "  Sample Rate: %d Hz",
                YESNO(this->use_mclk_), 
                YESNO(this->use_mic_),
                (this->microphone_type_ == ES8311_MICROPHONE_ANALOG ? "Analog" : "Digital"), 
                (int)this->resolution_out_, 
                (int)this->sample_frequency_);

  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Failed to initialize!");
    return;
  }
}

bool ES8311::set_volume(float volume) {
  volume = clamp(volume, 0.0f, 1.0f);
  uint8_t reg32 = remap<uint8_t, float>(volume, 0.0f, 1.0f, 0, 255);
  return this->write_byte(ES8311_REG32_DAC, reg32);
}

float ES8311::volume() {
  uint8_t reg32;
  this->read_byte(ES8311_REG32_DAC, &reg32);
  return remap<float, uint8_t>(reg32, 0, 255, 0.0f, 1.0f);
}

uint8_t ES8311::calculate_resolution_value(ES8311Resolution resolution) {
  switch (resolution) {
    case ES8311_RESOLUTION_16:
      return (3 << 2);
    case ES8311_RESOLUTION_18:
      return (2 << 2);
    case ES8311_RESOLUTION_20:
      return (1 << 2);
    case ES8311_RESOLUTION_24:
      return (0 << 2);
    case ES8311_RESOLUTION_32:
      return (4 << 2);
    default:
      return 0;
  }
}

const ES8311Coefficient *ES8311::get_coefficient(uint32_t mclk, uint32_t rate) {
  for (const auto &coefficient : ES8311_COEFFICIENTS) {
    if (coefficient.mclk == mclk && coefficient.rate == rate)
      return &coefficient;
  }
  return nullptr;
}

bool ES8311::configure_clock_() {
  // Register 0x01: select clock source for internal MCLK and determine its frequency
  uint8_t reg01 = 0x3F;  // Enable all clocks

  // Default to 12.288MHz MCLK for 16kHz sample rate if not specified
  uint32_t mclk_frequency = this->sample_frequency_ * this->mclk_multiple_;
  if (this->sample_frequency_ == 16000 && this->mclk_multiple_ == 256) {
    mclk_frequency = 12288000;  // 12.288MHz is optimal for 16kHz
  }
  
  if (!this->use_mclk_) {
    reg01 |= BIT(7);  // Use SCLK
    mclk_frequency = this->sample_frequency_ * (int) this->resolution_out_ * 2;
  }
  if (this->mclk_inverted_) {
    reg01 |= BIT(6);  // Invert MCLK pin
  }
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG01_CLK_MANAGER, reg01));

  // Get clock coefficients from coefficient table
  auto *coefficient = get_coefficient(mclk_frequency, this->sample_frequency_);
  if (coefficient == nullptr) {
    ESP_LOGE(TAG, "Unable to configure sample rate %" PRIu32 "Hz with %" PRIu32 "Hz MCLK", this->sample_frequency_,
             mclk_frequency);
    return false;
  }

  // Register 0x02
  uint8_t reg02;
  ES8311_ERROR_CHECK(this->read_byte(ES8311_REG02_CLK_MANAGER, &reg02));
  reg02 &= 0x07;
  reg02 |= (coefficient->pre_div - 1) << 5;
  reg02 |= coefficient->pre_mult << 3;
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG02_CLK_MANAGER, reg02));

  // Register 0x03
  const uint8_t reg03 = (coefficient->fs_mode << 6) | coefficient->adc_osr;
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG03_CLK_MANAGER, reg03));

  // Register 0x04
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG04_CLK_MANAGER, coefficient->dac_osr));

  // Register 0x05
  const uint8_t reg05 = ((coefficient->adc_div - 1) << 4) | (coefficient->dac_div - 1);
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG05_CLK_MANAGER, reg05));

  // Register 0x06
  uint8_t reg06;
  ES8311_ERROR_CHECK(this->read_byte(ES8311_REG06_CLK_MANAGER, &reg06));
  if (this->sclk_inverted_) {
    reg06 |= BIT(5);
  } else {
    reg06 &= ~BIT(5);
  }
  reg06 &= 0xE0;
  if (coefficient->bclk_div < 19) {
    reg06 |= (coefficient->bclk_div - 1) << 0;
  } else {
    reg06 |= (coefficient->bclk_div) << 0;
  }
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG06_CLK_MANAGER, reg06));

  // Register 0x07
  uint8_t reg07;
  ES8311_ERROR_CHECK(this->read_byte(ES8311_REG07_CLK_MANAGER, &reg07));
  reg07 &= 0xC0;
  reg07 |= coefficient->lrck_h << 0;
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG07_CLK_MANAGER, reg07));

  // Register 0x08
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG08_CLK_MANAGER, coefficient->lrck_l));
  
  // Successfully configured the clock
  return true;
}

bool ES8311::configure_format_() {
  // For all resolutions, use the hardcoded values that are known to work
  ESP_LOGI(TAG, "Using optimized format configuration for 16-bit resolution");
  
  // Configure I2S mode and format
  uint8_t reg00;
  ES8311_ERROR_CHECK(this->read_byte(ES8311_REG00_RESET, &reg00));
  reg00 &= 0xBF;  // Ensure bit 6 is cleared (slave mode)
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG00_RESET, reg00));
  
  // For 16-bit: ES8311_SDPIN_REG09 and ES8311_SDPOUT_REG0A
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG09_SDPIN, 0x0C));
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG0A_SDPOUT, 0x0C));
  
  return true;
}

bool ES8311::configure_mic_() {
  ESP_LOGI(TAG, "Configuring for Analog Microphone.");
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG14_SYSTEM, 0x1A));  // Analog Mic
  
  // Set microphone gain to 30dB (0x05)
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG16_ADC, 0x05));  // Analog Mic Gain
  
  // DMIC_SENSE
  ES8311_ERROR_CHECK(this->write_byte(ES8311_REG15_ADC, 0x40));  // DMIC_SENSE
  
  return true;
}

bool ES8311::set_mute_state_(bool mute_state) {
  uint8_t reg31;
  
  this->is_muted_ = mute_state;
  
  if (!this->read_byte(ES8311_REG31_DAC, &reg31)) {
    ESP_LOGE(TAG, "Failed to read mute register");
    return false;
  }
  
  if (mute_state) {
    reg31 |= BIT(6);  // Set mute bit
  } else {
    reg31 &= ~BIT(6);  // Clear mute bit
  }
  
  return this->write_byte(ES8311_REG31_DAC, reg31);
}

}  // namespace es8311
}  // namespace esphome
