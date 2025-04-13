#include "vl6180x_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace vl6180x {

static const char *const TAG = "vl6180x";
static const uint16_t ScalerValues[] = {0, 253, 127, 84};

void VL6180XSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up VL6180X...");
  
  // Initial delay
  delay(50);
  
  if (!init_()) {
    this->mark_failed();
    return;
  }

  // Configure default settings
  configure_default_();
}

bool VL6180XSensor::init_() {
  // Store part-to-part range offset
  this->ptp_offset_ = this->read_byte_(SYSRANGE__PART_TO_PART_RANGE_OFFSET);

  uint8_t reset_status;
  if (!this->read_byte(SYSTEM__FRESH_OUT_OF_RESET, &reset_status)) {
    ESP_LOGE(TAG, "Failed to read reset status");
    return false;
  }

  if (reset_status == 1) {
    ESP_LOGD(TAG, "Initializing VL6180X registers");
    this->scaling_ = 1;

    // Mandatory register settings - See datasheet
    this->write_byte(0x0207, 0x01);
    this->write_byte(0x0208, 0x01);
    this->write_byte(0x0096, 0x00);
    this->write_byte(0x0097, 0xFD);
    this->write_byte(0x00E3, 0x01);
    this->write_byte(0x00E4, 0x03);
    this->write_byte(0x00E5, 0x02);
    this->write_byte(0x00E6, 0x01);
    this->write_byte(0x00E7, 0x03);
    this->write_byte(0x00F5, 0x02);
    this->write_byte(0x00D9, 0x05);
    this->write_byte(0x00DB, 0xCE);
    this->write_byte(0x00DC, 0x03);
    this->write_byte(0x00DD, 0xF8);
    this->write_byte(0x009F, 0x00);
    this->write_byte(0x00A3, 0x3C);
    this->write_byte(0x00B7, 0x00);
    this->write_byte(0x00BB, 0x3C);
    this->write_byte(0x00B2, 0x09);
    this->write_byte(0x00CA, 0x09);
    this->write_byte(0x0198, 0x01);
    this->write_byte(0x01B0, 0x17);
    this->write_byte(0x01AD, 0x00);
    this->write_byte(0x00FF, 0x05);
    this->write_byte(0x0100, 0x05);
    this->write_byte(0x0199, 0x05);
    this->write_byte(0x01A6, 0x1B);
    this->write_byte(0x01AC, 0x3E);
    this->write_byte(0x01A7, 0x1F);
    this->write_byte(0x0030, 0x00);

    this->write_byte(SYSTEM__FRESH_OUT_OF_RESET, 0);
  } else {
    ESP_LOGD(TAG, "VL6180X already initialized");

    // Get existing scaling setting
    uint8_t scaler_bytes[2];
    if (!this->read_register16(RANGE_SCALER, scaler_bytes, 2)) {
      ESP_LOGE(TAG, "Failed to read scaling setting");
      return false;
    }
    
    uint16_t s = (uint16_t(scaler_bytes[0]) << 8) | scaler_bytes[1];
    
    if (s == ScalerValues[3])
      this->scaling_ = 3;
    else if (s == ScalerValues[2])
      this->scaling_ = 2;
    else
      this->scaling_ = 1;

    this->ptp_offset_ *= this->scaling_;
  }

  return true;
}

void VL6180XSensor::configure_default_() {
  // "Recommended : Public registers"
  this->write_byte(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);
  this->write_byte(SYSALS__ANALOGUE_GAIN, 0x46);
  this->write_byte(SYSRANGE__VHV_REPEAT_RATE, 0xFF);
  this->write_register16(SYSALS__INTEGRATION_PERIOD, 0x0063);
  this->write_byte(SYSRANGE__VHV_RECALIBRATE, 0x01);

  // Optional: Public registers
  this->write_byte(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);
  this->write_byte(SYSALS__INTERMEASUREMENT_PERIOD, 0x31);
  this->write_byte(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);

  // Set default scaling
  this->set_scaling(1);
}

void VL6180XSensor::set_scaling(uint8_t new_scaling) {
  static const uint16_t ScalerValues[] = {0, 253, 127, 84};
  uint8_t const DefaultCrosstalkValidHeight = 20;

  if (new_scaling < 1 || new_scaling > 3) {
    return;
  }

  scaling_ = new_scaling;
  this->write_register16(RANGE_SCALER, ScalerValues[scaling_]);

  // apply scaling on part-to-part offset
  this->write_byte(SYSRANGE__PART_TO_PART_RANGE_OFFSET, ptp_offset_ / scaling_);

  // apply scaling on CrossTalkValidHeight
  this->write_byte(SYSRANGE__CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / scaling_);

  // enable early convergence estimate only at 1x scaling
  uint8_t rce = this->read_byte_(SYSRANGE__RANGE_CHECK_ENABLES);
  this->write_byte(SYSRANGE__RANGE_CHECK_ENABLES, (rce & 0xFE) | (scaling_ == 1));
}

void VL6180XSensor::update() {
  // Start single shot ranging
  this->write_byte(SYSRANGE__START, 0x01);
  
  // Wait for measurement
  uint8_t status;
  uint16_t timeout = 0;
  do {
    if (!this->read_byte(RESULT__INTERRUPT_STATUS_GPIO, &status)) {
      ESP_LOGW(TAG, "Failed to read status");
      return;
    }
    delay(1);
    if (timeout++ > 100) {
      ESP_LOGW(TAG, "Timeout waiting for measurement");
      return;
    }
  } while ((status & 0x07) != 0x04);

  // Read range value
  uint8_t range = this->read_byte_(RESULT__RANGE_VAL);
  
  // Clear interrupt
  this->write_byte(SYSTEM__INTERRUPT_CLEAR, 0x01);

  // Get error code
  uint8_t error_code = this->read_byte_(RESULT__RANGE_STATUS) >> 4;

  if (error_code == VL6180X_ERROR_NONE) {
    // Convert to millimeters and apply scaling
    float range_mm = range * this->scaling_;
    
    if (this->distance_sensor_ != nullptr) {
      this->distance_sensor_->publish_state(range_mm);
    }
  } else {
    ESP_LOGW(TAG, "Range error: %d", error_code);
  }
}


void VL6180XSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "VL6180X:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with VL6180X failed!");
    return;
  }
  
  ESP_LOGCONFIG(TAG, "  Scaling: %dx", this->scaling_);
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "ALS", this->als_sensor_);
}

}  // namespace vl6180x
}  // namespace esphome
