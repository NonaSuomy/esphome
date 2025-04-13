#include "vl6180x_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h" // Needed for delay()

namespace esphome {
namespace vl6180x {

// --- Static Member DEFINITION ---
const char *const VL6180XSensor::TAG = "vl6180x";
// Define ScalingValues lookup table ONLY here
static const uint16_t ScalingValues[] = {0, 253, 127, 84}; // Index 1=1x, 2=2x, 3=3x
// Define ALS integration period constant (100ms corresponds to register value 99)
static const uint16_t ALS_INTEGRATION_PERIOD_REGISTER_VALUE = 0x0063; // 99 -> 100ms
static const float ALS_INTEGRATION_PERIOD_MS = 100.0f;
// Define ALS constant from datasheet for lux calculation
static const float ALS_LUX_RESOLUTION_FACTOR = 0.32f;


void VL6180XSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up VL6180X...");
  delay(50); // Initial delay

  if (!init_()) {
    this->mark_failed();
    return;
  }
  configure_default_(); // Apply non-scaling defaults (includes default ALS gain & integration)
  apply_scaling_dependent_registers(); // Apply scaling-related registers after init and defaults

  ESP_LOGCONFIG(TAG, "VL6180X Setup finished.");
}

bool VL6180XSensor::init_() {
  ESP_LOGD(TAG, "Running VL6180X init_()...");
  this->ptp_offset_ = (int8_t)this->read_byte_(SYSRANGE__PART_TO_PART_RANGE_OFFSET);
  ESP_LOGV(TAG, "Read raw PTP offset value: %d", this->ptp_offset_);

  uint8_t reset_status;
  if (!this->read_byte(SYSTEM__FRESH_OUT_OF_RESET, &reset_status)) {
    ESP_LOGE(TAG, "init_: Failed to read reset status register.");
    return false;
  }
  ESP_LOGV(TAG, "init_: Reset status read: %d", reset_status);

  if (reset_status == 1) {
    ESP_LOGD(TAG, "Sensor is fresh out of reset. Applying mandatory settings.");
    this->write_byte(0x0207, 0x01);
    this->write_byte(0x0208, 0x01);
    this->write_byte(0x0096, 0x00);
    this->write_byte(0x0097, 0xFD); // RANGE_SCALING = 1x
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
    ESP_LOGD(TAG, "Calculated 1x Part-to-part offset (fresh reset): %d", this->ptp_offset_);
    // Re-apply desired scaling since mandatory writes forced 1x.
    ESP_LOGD(TAG, "Re-applying desired scaling (%dx) after mandatory reset settings.", this->scaling_);
    this->set_scaling(this->scaling_);

  } else {
    ESP_LOGD(TAG, "Sensor was not fresh out of reset. Calculating PTP offset based on current hardware scaling.");
    uint8_t detected_hardware_scaling = 1;
    uint8_t scaling_bytes[2];
    if (!this->read_register16(RANGE_SCALING, scaling_bytes, 2)) {
      ESP_LOGE(TAG, "Failed to read scaling setting for PTP offset calc. Assuming 1x.");
    } else {
        uint16_t s = (uint16_t(scaling_bytes[0]) << 8) | scaling_bytes[1];
        if (s == ScalingValues[3]) detected_hardware_scaling = 3;
        else if (s == ScalingValues[2]) detected_hardware_scaling = 2;
        else detected_hardware_scaling = 1;
        ESP_LOGD(TAG, "Existing scaling detected for PTP calc: %dx", detected_hardware_scaling);
    }
    this->ptp_offset_ *= detected_hardware_scaling;
    ESP_LOGD(TAG, "Calculated 1x Part-to-part offset: %d", this->ptp_offset_);
  }
  return true;
}

void VL6180XSensor::configure_default_() {
  ESP_LOGD(TAG, "Applying default configuration settings...");
  this->write_byte(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);
  this->als_gain_ = GAIN_1; // Set internal state
  this->write_byte(SYSALS__ANALOGUE_GAIN, 0x40 | this->als_gain_); // Write default gain to HW
  this->write_byte(SYSRANGE__VHV_REPEAT_RATE, 0xFF);
  this->write_register16(SYSALS__INTEGRATION_PERIOD, ALS_INTEGRATION_PERIOD_REGISTER_VALUE);
  this->write_byte(SYSRANGE__VHV_RECALIBRATE, 0x01);
  this->write_byte(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);
  this->write_byte(SYSALS__INTERMEASUREMENT_PERIOD, 0x31);
  this->write_byte(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);
  this->write_byte(SYSRANGE__MAX_CONVERGENCE_TIME, 0x31);
  this->write_byte(INTERLEAVED_MODE__ENABLE, 0);
}

void VL6180XSensor::apply_scaling_dependent_registers() {
  ESP_LOGD(TAG, "Applying scaling-dependent registers for %dx scaling...", this->scaling_);
  if (this->scaling_ < 1 || this->scaling_ > 3) {
      ESP_LOGE(TAG, "Internal error: Invalid scaling factor %d. Skipping.", this->scaling_);
      return;
  }
  uint8_t const DefaultCrosstalkValidHeight = 20;
  int scaled_offset = this->ptp_offset_ / this->scaling_;
  ESP_LOGV(TAG, "Applying scaled PTP offset: %d / %d = %d", this->ptp_offset_, this->scaling_, scaled_offset);
  if (!this->write_byte(SYSRANGE__PART_TO_PART_RANGE_OFFSET, (uint8_t)scaled_offset)) {
     ESP_LOGE(TAG, "Failed to write scaled PTP offset!");
  }
  uint8_t scaled_crosstalk_height = DefaultCrosstalkValidHeight / this->scaling_;
  ESP_LOGV(TAG, "Applying scaled Crosstalk Valid Height: %d / %d = %d", DefaultCrosstalkValidHeight, this->scaling_, scaled_crosstalk_height);
  if (!this->write_byte(SYSRANGE__CROSSTALK_VALID_HEIGHT, scaled_crosstalk_height)) {
      ESP_LOGE(TAG, "Failed to write scaled Crosstalk Valid Height!");
  }
  uint8_t rce;
  if (!this->read_byte(SYSRANGE__RANGE_CHECK_ENABLES, &rce)) {
      ESP_LOGW(TAG, "Failed to read RANGE_CHECK_ENABLES for ECE adjustment.");
  } else {
      uint8_t new_rce = (rce & 0xFE) | (this->scaling_ == 1);
      ESP_LOGV(TAG, "Applying ECE setting (enabled=%d): %d", (this->scaling_ == 1), new_rce);
      if (new_rce != rce && !this->write_byte(SYSRANGE__RANGE_CHECK_ENABLES, new_rce)) {
          ESP_LOGE(TAG, "Failed to write updated RANGE_CHECK_ENABLES!");
      }
  }
  ESP_LOGD(TAG, "Scaling-dependent register application finished.");
}

void VL6180XSensor::set_scaling(uint8_t new_scaling) {
  if (new_scaling < 1 || new_scaling > 3) {
     ESP_LOGW(TAG, "Invalid scaling factor %d requested. Ignoring.", new_scaling);
    return;
  }
  ESP_LOGD(TAG, "Storing desired scaling factor %dx", new_scaling);
  this->scaling_ = new_scaling;
  ESP_LOGV(TAG, "Writing RANGE_SCALING register (0x%04X) with value %d", RANGE_SCALING, ScalingValues[this->scaling_]);
  if (!this->write_register16(RANGE_SCALING, ScalingValues[this->scaling_])) {
      ESP_LOGE(TAG, "Failed to write RANGE_SCALING register in set_scaling!");
  }
  // Note: Other scaling registers are applied later by apply_scaling_dependent_registers()
}

// Helper to get ALS gain multiplier
float VL6180XSensor::get_als_gain_multiplier() const {
    switch (this->als_gain_) {
        case GAIN_1:    return 1.0f;
        case GAIN_1_25: return 1.25f;
        case GAIN_1_67: return 1.67f;
        case GAIN_2_5:  return 2.5f;
        case GAIN_5:    return 5.0f;
        case GAIN_10:   return 10.0f;
        case GAIN_20:   return 20.0f;
        case GAIN_40:   return 40.0f;
        default:        return 1.0f; // Should not happen
    }
}

// --- Error Handling Helper IMPLEMENTATION ---
void VL6180XSensor::handle_error(uint8_t error_code) {
  // Use ESP_LOGW for warnings/errors, ESP_LOGV for informational errors
  switch (error_code) {
    case VL6180X_ERROR_NONE:
      // No error - shouldn't be called in this case, but handle defensively
      break;
    case VL6180X_ERROR_SYSERR_1:
      ESP_LOGE(TAG, "Range Error: System error; VCSEL Continuity Test"); break;
    case VL6180X_ERROR_SYSERR_2:
      ESP_LOGE(TAG, "Range Error: System error; VCSEL Watchdog Test"); break;
    case VL6180X_ERROR_SYSERR_3:
      ESP_LOGE(TAG, "Range Error: System error; VCSEL Watchdog"); break;
    case VL6180X_ERROR_SYSERR_4:
      ESP_LOGE(TAG, "Range Error: System error; PLL1 Lock"); break;
    case VL6180X_ERROR_SYSERR_5:
      ESP_LOGE(TAG, "Range Error: System error; PLL2 Lock"); break;
    case VL6180X_ERROR_ECEFAIL:
      ESP_LOGW(TAG, "Range Error: Early Convergence Estimate failed"); break;
    case VL6180X_ERROR_NOCONVERGE:
      ESP_LOGW(TAG, "Range Error: No target convergence"); break;
    case VL6180X_ERROR_RANGEIGNORE:
      ESP_LOGW(TAG, "Range Error: Ignore threshold check failed"); break;
    case VL6180X_ERROR_SNR:
      ESP_LOGW(TAG, "Range Error: Ambient conditions too high (SNR check)"); break;
    case VL6180X_ERROR_RAWUFLOW:
      ESP_LOGW(TAG, "Range Error: Raw range algo underflow (target too close?)"); break;
    case VL6180X_ERROR_RAWOFLOW:
      ESP_LOGW(TAG, "Range Error: Raw range algo overflow (target too far?)"); break;
    case VL6180X_ERROR_RANGEUFLOW:
      ESP_LOGW(TAG, "Range Error: Range algo underflow (target too close?)"); break;
    case VL6180X_ERROR_RANGEOFLOW:
      ESP_LOGW(TAG, "Range Error: Range algo overflow (target too far?)"); break;
    case VL6180X_ERROR_RANGINGFILTERED:
      ESP_LOGW(TAG, "Range Error: Distance filtered by Wrap Around Filter (WAF)"); break;
    case VL6180X_ERROR_DATANOTREADY:
      // This shouldn't happen with polling, but log if it does
      ESP_LOGE(TAG, "Range Error: Data not ready (unexpected)"); break;
    default:
      ESP_LOGE(TAG, "Range Error: Unknown error code: %d", error_code); break;
  }
}


void VL6180XSensor::update() {
  ESP_LOGV(TAG, "Update: Starting measurement cycle (Range Scaling: %dx, ALS Gain: %.2fx)",
           this->scaling_, this->get_als_gain_multiplier());

  // --- Range Measurement ---
  if (this->distance_sensor_ != nullptr) {
    if (!this->write_byte(SYSRANGE__START, 0x01)) {
        ESP_LOGW(TAG, "Update: Failed to write SYSRANGE__START");
        this->status_set_warning(); this->distance_sensor_->publish_state(NAN);
    } else {
        uint8_t status = 0; uint16_t timeout = 0; const uint16_t max_timeout = 100; bool timed_out = false;
        do {
          if (!this->read_byte(RESULT__INTERRUPT_STATUS_GPIO, &status)) {
            ESP_LOGW(TAG, "Update: Failed to read RESULT__INTERRUPT_STATUS_GPIO for Range");
            this->status_set_warning(); this->distance_sensor_->publish_state(NAN);
            this->write_byte(SYSTEM__INTERRUPT_CLEAR, 0x07); timed_out = true; break;
          }
          if ((status & 0x04) != 0) break; // Range ready
          delay(1); timeout++;
          if (timeout > max_timeout) { timed_out = true; break; }
        } while (true);
        this->write_byte(SYSTEM__INTERRUPT_CLEAR, 0x01); // Clear Range interrupt

        if (!timed_out) {
          uint8_t range = this->read_byte_(RESULT__RANGE_VAL);
          uint8_t error_status = this->read_byte_(RESULT__RANGE_STATUS);
          uint8_t error_code = error_status >> 4;
          if (error_code == VL6180X_ERROR_NONE) {
            float range_mm = (float)range * this->scaling_;
            ESP_LOGD(TAG, "Distance: %.0f mm", range_mm);
            this->distance_sensor_->publish_state(range_mm);
            // Clear warning only if ALS is also OK (or not enabled)
            if (this->als_sensor_ == nullptr) this->status_clear_warning();
          } else {
            // --- Call handle_error instead of just logging code ---
            this->handle_error(error_code);
            this->distance_sensor_->publish_state(NAN);
            this->status_set_warning();
          }
        } else {
          ESP_LOGW(TAG, "Update: Timeout waiting for Range measurement.");
          this->distance_sensor_->publish_state(NAN);
          this->status_set_warning();
        }
    }
  } // End Range Measurement

  // --- Ambient Light Measurement ---
  if (this->als_sensor_ != nullptr) {
    if (!this->write_byte(SYSALS__ANALOGUE_GAIN, 0x40 | this->als_gain_)) {
        ESP_LOGW(TAG, "Update: Failed to write SYSALS__ANALOGUE_GAIN");
        this->status_set_warning(); this->als_sensor_->publish_state(NAN);
        return;
    }
    if (!this->write_byte(SYSALS__START, 0x01)) {
        ESP_LOGW(TAG, "Update: Failed to write SYSALS__START");
        this->status_set_warning(); this->als_sensor_->publish_state(NAN);
        return;
    }

    uint8_t status = 0; uint16_t timeout = 0; const uint16_t max_timeout = (uint16_t)ALS_INTEGRATION_PERIOD_MS + 50;
    bool timed_out = false;
    do {
      if (!this->read_byte(RESULT__INTERRUPT_STATUS_GPIO, &status)) {
        ESP_LOGW(TAG, "Update: Failed to read RESULT__INTERRUPT_STATUS_GPIO for ALS");
        this->status_set_warning(); this->als_sensor_->publish_state(NAN);
        this->write_byte(SYSTEM__INTERRUPT_CLEAR, 0x07); timed_out = true; break;
      }
      if ((status & 0x20) != 0) break; // ALS ready (bit 5)
      delay(5); timeout += 5;
      if (timeout > max_timeout) { timed_out = true; break; }
    } while (true);

    this->write_byte(SYSTEM__INTERRUPT_CLEAR, 0x02); // Clear ALS interrupt (bit 1)

    if (!timed_out) {
      uint8_t als_bytes[2];
      if (!this->read_register16(RESULT__ALS_VAL, als_bytes, 2)) {
          ESP_LOGW(TAG, "Update: Failed to read RESULT__ALS_VAL");
          this->status_set_warning(); this->als_sensor_->publish_state(NAN);
      } else {
          uint16_t als_count = ((uint16_t)als_bytes[0] << 8) | als_bytes[1];
          ESP_LOGV(TAG, "Update: Raw ALS value read: %d", als_count);

          float gain_multiplier = this->get_als_gain_multiplier();
          float lux = ALS_LUX_RESOLUTION_FACTOR * ((float)als_count / gain_multiplier) * (100.0f / ALS_INTEGRATION_PERIOD_MS);
          ESP_LOGD(TAG, "ALS: %.2f lx (Raw: %d, Gain: %.2fx)", lux, als_count, gain_multiplier);
          this->als_sensor_->publish_state(lux);
          // Clear warning only if distance sensor is not enabled or was OK
          if (this->distance_sensor_ == nullptr || !this->status_has_warning()) {
              this->status_clear_warning();
          }

          VL6180XAlsGain next_gain = this->als_gain_;
          if (als_count == 65535 && this->als_gain_ != GAIN_1) {
              ESP_LOGD(TAG, "ALS saturated, decreasing gain for next measurement.");
              switch (this->als_gain_) {
                  case GAIN_40:   next_gain = GAIN_20;   break;
                  case GAIN_20:   next_gain = GAIN_10;   break;
                  case GAIN_10:   next_gain = GAIN_5;    break;
                  case GAIN_5:    next_gain = GAIN_2_5;  break;
                  case GAIN_2_5:  next_gain = GAIN_1_67; break;
                  case GAIN_1_67: next_gain = GAIN_1_25; break;
                  case GAIN_1_25: next_gain = GAIN_1;    break;
                  case GAIN_1:    break;
              }
          } else if (als_count < 10000 && this->als_gain_ != GAIN_40) {
              ESP_LOGD(TAG, "ALS signal low, increasing gain for next measurement.");
              switch (this->als_gain_) {
                  case GAIN_1:    next_gain = GAIN_1_25; break;
                  case GAIN_1_25: next_gain = GAIN_1_67; break;
                  case GAIN_1_67: next_gain = GAIN_2_5;  break;
                  case GAIN_2_5:  next_gain = GAIN_5;    break;
                  case GAIN_5:    next_gain = GAIN_10;   break;
                  case GAIN_10:   next_gain = GAIN_20;   break;
                  case GAIN_20:   next_gain = GAIN_40;   break;
                  case GAIN_40:   break;
              }
          }

          if (next_gain != this->als_gain_) {
              float old_mult = this->get_als_gain_multiplier(); // Get multiplier before changing state
              this->als_gain_ = next_gain; // Update gain state for the next cycle
              float new_mult = this->get_als_gain_multiplier(); // Get multiplier after changing state
              ESP_LOGI(TAG, "ALS Gain changing from %.2fx to %.2fx", old_mult, new_mult);
          }
      }
    } else {
      ESP_LOGW(TAG, "Update: Timeout waiting for ALS measurement.");
      this->als_sensor_->publish_state(NAN);
      this->status_set_warning();
    }
  } // End ALS Measurement
}

void VL6180XSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "VL6180X:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with VL6180X failed!");
  }
  ESP_LOGCONFIG(TAG, "  Desired Range Scaling Factor: %dx", this->scaling_);
  ESP_LOGCONFIG(TAG, "  Calculated 1x PTP Offset: %d", this->ptp_offset_);
  ESP_LOGCONFIG(TAG, "  Initial ALS Gain: %.2fx", this->get_als_gain_multiplier());
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "ALS", this->als_sensor_);
}

// --- I2C Helper Method Implementations ---
bool VL6180XSensor::write_byte(uint16_t reg, uint8_t value) {
  uint8_t data[3]; data[0] = (reg >> 8) & 0xFF; data[1] = reg & 0xFF; data[2] = value;
  if (this->write(data, 3) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "write_byte failed for reg 0x%04X", reg); return false;
  } return true;
}
uint8_t VL6180XSensor::read_byte_(uint16_t reg) {
  uint8_t value = 0; this->read_byte(reg, &value); return value;
}
bool VL6180XSensor::read_byte(uint16_t reg, uint8_t *value) {
  uint8_t data[2]; data[0] = (reg >> 8) & 0xFF; data[1] = reg & 0xFF;
  if (this->write(data, 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "read_byte: Failed write reg 0x%04X", reg); return false;
  }
  if (this->read(value, 1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "read_byte: Failed read reg 0x%04X", reg); return false;
  } return true;
}
bool VL6180XSensor::write_register16(uint16_t reg, uint16_t value) {
  uint8_t data[4]; data[0] = (reg >> 8) & 0xFF; data[1] = reg & 0xFF;
  data[2] = (value >> 8) & 0xFF; data[3] = value & 0xFF;
  if (this->write(data, 4) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "write_register16 failed for reg 0x%04X", reg); return false;
  } return true;
}
bool VL6180XSensor::read_register16(uint16_t reg, uint8_t *value, size_t len) {
   if (len != 2) { ESP_LOGE(TAG, "read_register16: Invalid length %zu", len); return false; }
  uint8_t data[2]; data[0] = (reg >> 8) & 0xFF; data[1] = reg & 0xFF;
  if (this->write(data, 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "read_register16: Failed write reg 0x%04X", reg); return false;
  }
  if (this->read(value, len) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "read_register16: Failed read %zu bytes from reg 0x%04X", len, reg); return false;
  } return true;
}

}  // namespace vl6180x
}  // namespace esphome
