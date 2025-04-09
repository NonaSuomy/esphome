#include "vl6180x_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace vl6180x {

static const char *const TAG = "vl6180x";

bool VL6180XSensor::init_sensor_() {
  // Check sensor ID
  uint8_t id = this->read_reg(IDENTIFICATION__MODEL_ID);
  if (id != 0xB4) {
    ESP_LOGE(TAG, "Wrong sensor ID: 0x%02X", id);
    return false;
  }

  // Check for fresh out of reset
  if (this->read_reg(SYSTEM__FRESH_OUT_OF_RESET) == 1) {
    ESP_LOGD(TAG, "Initializing VL6180X registers");

    // Required register settings from datasheet
    this->write_reg(0x0207, 0x01);
    this->write_reg(0x0208, 0x01);
    this->write_reg(0x0096, 0x00);
    this->write_reg(0x0097, 0xFD);
    this->write_reg(0x00E3, 0x00);
    this->write_reg(0x00E4, 0x04);
    this->write_reg(0x00E5, 0x02);
    this->write_reg(0x00E6, 0x01);
    this->write_reg(0x00E7, 0x03);
    this->write_reg(0x00F5, 0x02);
    this->write_reg(0x00D9, 0x05);
    this->write_reg(0x00DB, 0xCE);
    this->write_reg(0x00DC, 0x03);
    this->write_reg(0x00DD, 0xF8);
    this->write_reg(0x009F, 0x00);
    this->write_reg(0x00A3, 0x3C);
    this->write_reg(0x00B7, 0x00);
    this->write_reg(0x00BB, 0x3C);
    this->write_reg(0x00B2, 0x09);
    this->write_reg(0x00CA, 0x09);
    this->write_reg(0x0198, 0x01);
    this->write_reg(0x01B0, 0x17);
    this->write_reg(0x01AD, 0x00);
    this->write_reg(0x00FF, 0x05);
    this->write_reg(0x0100, 0x05);
    this->write_reg(0x0199, 0x05);
    this->write_reg(0x01A6, 0x1B);
    this->write_reg(0x01AC, 0x3E);
    this->write_reg(0x01A7, 0x1F);
    this->write_reg(0x0030, 0x00);

    // Recommended : Public registers - See data sheet for more detail
    this->write_reg(0x0011, 0x10);    // Enables polling for 'New Sample ready'
    this->write_reg(0x010A, 0x30);    // Set the averaging sample period
    this->write_reg(0x003F, 0x46);    // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
    this->write_reg(0x0031, 0xFF);    // sets the # of range measurements after which auto calibration of system is performed
    this->write_reg(0x0040, 0x63);    // Set ALS integration time to 100ms
    this->write_reg(0x002E, 0x01);    // perform a single temperature calibration of the ranging sensor

    // Additional settings for better range performance
    this->write_reg(SYSRANGE__MAX_CONVERGENCE_TIME, 0x32);    // Set max convergence time to 50ms
    this->write_reg(SYSRANGE__RANGE_CHECK_ENABLES, 0x10);     // Enable early convergence estimate
    this->write_reg(SYSALS__ANALOGUE_GAIN, 0x46);            // Set ALS gain
    this->write_reg(SYSALS__INTEGRATION_PERIOD, 0x63);        // Set ALS integration time to 100ms

    // Optional: Public registers - See data sheet for more detail
    this->write_reg(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);  // Set default ranging inter-measurement period to 100ms
    this->write_reg(SYSALS__INTERMEASUREMENT_PERIOD, 0x31);    // Set default ALS inter-measurement period to 500ms
    this->write_reg(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);      // Configures interrupt on 'New Sample Ready threshold event'

    // Clear fresh out of reset bit
    this->write_reg(SYSTEM__FRESH_OUT_OF_RESET, 0x00);
  } else {
    ESP_LOGD(TAG, "VL6180X already initialized");
  }

  // Set default scaling
  this->set_scaling(1);

  return true;
}

void VL6180XSensor::set_scaling(uint8_t new_scaling) {
  uint8_t const DefaultCrosstalkValidHeight = 20;

  // ScalerValues index corresponds to scaling value
  static const uint16_t ScalerValues[] = {0, 253, 127, 84};
  uint16_t scalerValue;

  if (new_scaling < 1 || new_scaling > 3) { 
    new_scaling = 1;
  }
  scalerValue = ScalerValues[new_scaling];

  // Update registers
  this->write_reg(RANGE_SCALER, scalerValue);

  // Update other scaling-dependent settings
  this->write_reg(SYSRANGE__MAX_CONVERGENCE_TIME, 49);  // Adjust convergence time
  this->write_reg(SYSRANGE__RANGE_CHECK_ENABLES, 0x10); // Enable early convergence estimate
  this->write_reg(SYSRANGE__MAX_AMBIENT_LEVEL_MULT, 0x07); // Set max ambient level multiplier
  
  // Set max convergence time
  uint8_t max_convergence = (new_scaling == 1) ? 49 : 63;
  this->write_reg(SYSRANGE__MAX_CONVERGENCE_TIME, max_convergence);
  
  this->write_reg(SYSRANGE__CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / new_scaling);

  // Store scaling factor
  this->scaling_ = new_scaling;
}

uint16_t VL6180XSensor::read_range_single() {  // Change return type to uint16_t
  // Start single range measurement
  this->write_reg(SYSRANGE__START, 0x01);

  // Wait for measurement ready
  uint8_t status;
  uint32_t start_time = millis();
  do {
    status = this->read_reg(RESULT__INTERRUPT_STATUS_GPIO);
    if (millis() - start_time > 500) {  // 500ms timeout
      ESP_LOGW(TAG, "Timeout waiting for range measurement");
      return 255;
    }
    delay(1);
  } while ((status & 0x04) == 0);

  // Read range in millimeters
  uint8_t range = this->read_reg(RESULT__RANGE_VAL);
  uint8_t range_status = this->read_reg(RESULT__RANGE_STATUS) >> 4;

  // Clear interrupt
  this->write_reg(SYSTEM__INTERRUPT_CLEAR, 0x07);

  // Calculate scaled range
  uint16_t scaled_range = range * this->scaling_;  // Use uint16_t for scaled value

  return scaled_range;  // Return the scaled value directly
}

void VL6180XSensor::update() {
  if (this->distance_sensor_ != nullptr) {
    uint16_t range = this->read_range_single();  // Change to uint16_t
    this->distance_sensor_->publish_state(range);
  }

  if (this->als_sensor_ != nullptr) {
    uint16_t als = this->read_ambient_single();
    this->als_sensor_->publish_state(als);
  }
}


uint16_t VL6180XSensor::read_ambient_single() {
  // Start single ALS measurement
  this->write_reg(SYSALS__START, 0x01);

  // Wait for measurement ready
  uint8_t status;
  uint32_t start_time = millis();
  do {
    status = this->read_reg(RESULT__INTERRUPT_STATUS_GPIO);
    if (millis() - start_time > 500) {  // 500ms timeout
      ESP_LOGW(TAG, "Timeout waiting for ALS measurement");
      return 0;
    }
    delay(1);
  } while ((status & 0x20) == 0);

  // Read ALS value
  uint16_t als = this->read_reg_16bit(RESULT__ALS_VAL);

  // Clear interrupt
  this->write_reg(SYSTEM__INTERRUPT_CLEAR, 0x07);

  return als;
}

uint8_t VL6180XSensor::read_range_status() {
  return this->read_reg(RESULT__RANGE_STATUS);
}

void VL6180XSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up VL6180X...");
  
  if (!this->init_sensor_()) {
    this->mark_failed();
    return;
  }
}

void VL6180XSensor::update() {
  if (this->distance_sensor_ != nullptr) {
    uint8_t range = this->read_range_single();
    uint16_t scaled_range = range * this->scaling_;
    this->distance_sensor_->publish_state(scaled_range);
  }

  if (this->als_sensor_ != nullptr) {
    uint16_t als = this->read_ambient_single();
    this->als_sensor_->publish_state(als);
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
