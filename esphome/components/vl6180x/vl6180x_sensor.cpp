// vl6180x_sensor.cpp

#include "vl6180x_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstdint>

namespace esphome {
namespace vl6180x {

static const char *TAG = "vl6180x.sensor";

// Define MAX_READINGS
const int MAX_READINGS = 10;

/* static const uint32_t GESTURE_SINGLE_TAP_INTERVAL_MS = 500;
static const uint32_t GESTURE_DOUBLE_TAP_INTERVAL_MS = 500;
static const uint32_t GESTURE_SINGLE_TAP_DELAY_MS = 1000;
static const uint32_t GESTURE_DOUBLE_TAP_DELAY_MS = 1000;
static const uint32_t GESTURE_HOVER_TIME_MS = 3000;
static const uint8_t GESTURE_HAND_PRESENCE_THRESHOLD = 40;
static const uint8_t GESTURE_SINGLE_TAP_THRESHOLD = 40;
static const uint8_t GESTURE_DOUBLE_TAP_THRESHOLD = 40; */

VL6180XSensor::VL6180XSensor(uint8_t address, uint32_t update_interval)
: PollingComponent(update_interval), lux_without_glass_(0.0), als_lux_resolution_without_glass_(0.32) {
    ESP_LOGI("vl6180x", "VL6180X constructor called");
    this->set_i2c_address(address);
  //this->on_swipe_gesture_trigger_ = new esphome::Trigger<int>();
}

void VL6180XSensor::setup() {
    ESP_LOGI("vl6180x", "Setup function called");
    // Check for expected model ID
//    if (this->reading_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_IDENTIFICATION_MODEL_ID)) != 0xB4) {
      // ESP_LOGE(TAG, "VL6180X sensor not found");
//      this->handle_error(0x00);  // Pass a default error code
      //this->mark_failed();
//      return;
//    }

  // Check for expected model ID
  uint8_t model_id = this->reading_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_IDENTIFICATION_MODEL_ID));
  ESP_LOGI("vl6180x", "Model ID: 0x%02X", model_id);

  if (model_id != 0xB4) {
    ESP_LOGE("vl6180x", "VL6180X sensor not found");
    this->mark_failed();
    return;
  }

    // Store part-to-part range offset so it can be adjusted if scaling is changed
    ptp_offset_ = reading_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET));

    // Check if fresh out of reset
    if (this->reading_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET)) & 0x01) {
      scaling_ = 1;
      ESP_LOGD(TAG, "VL6180X fresh out of reset, loading settings");
      // Initialize VL6180X sensor settings
      this->load_settings();
      this->writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET), 0x00);
    } else {
        // Sensor has already been initialized, so try to get scaling settings by reading registers.
        uint16_t s = reading_register16(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RANGE_SCALER));
        if      (s == ScalerValues[3]) { scaling_ = 3; }
        else if (s == ScalerValues[2]) { scaling_ = 2; }
        else                           { scaling_ = 1; }

        // Adjust the part-to-part range offset value read earlier to account for existing scaling.
        ptp_offset_ *= scaling_;
        ESP_LOGD(TAG, "Initial scaling: %ux", scaling_);
    }


    ESP_LOGD(TAG, "VL6180X sensor initialized");

}

void VL6180XSensor::start_measurement() {
  this->writing_register(0x018, 0x01); // Start a new measurement
  this->state_ = SENSOR_STATE_WAITING_FOR_DATA;
}

void VL6180XSensor::check_measurement() {
  if (this->state_ == SENSOR_STATE_WAITING_FOR_DATA &&
      4 == ((reading_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO)) >> 3) & 0x7)) {
    this->state_ = SENSOR_STATE_DATA_READY;
    // Read and publish the data here
    uint16_t range = this->reading_register16(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RESULT_RANGE_VAL));
    float als = read_als();
	if (this->distance_sensor_ != nullptr)
      distance_sensor_->publish_state(static_cast<float>(range));
    if (this->als_sensor_ != nullptr)
      als_sensor_->publish_state(als);
  }
}

void VL6180XSensor::update() {
  ESP_LOGD("vl6180x", "Update function called");
  this->writing_register(0x018, 0x01); // Start a new measurement
  this->data_ready_ = true; // Set the flag to indicate that data is expected

  uint8_t range_status = this->reading_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RESULT_RANGE_STATUS));
  uint16_t raw_range = this->reading_register16(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RESULT_RANGE_VAL));

  ESP_LOGD(TAG, "Range status: 0x%02X, Raw range: %u", range_status, raw_range);

  if ((range_status & 0x01) == 0) {
    // Valid measurement
    float scaled_range = static_cast<float>(raw_range) * scaling_;
    ESP_LOGD(TAG, "Scaled range: %.2f", scaled_range);
    if (this->distance_sensor_ != nullptr)
      distance_sensor_->publish_state(scaled_range);
  } else {
    // Error in measurement
    ESP_LOGW(TAG, "Error in range measurement: status 0x%02X", range_status);
    if (this->distance_sensor_ != nullptr)
      distance_sensor_->publish_state(NAN);  // Or some other indication of invalid reading
  }

  // Read the status register
//  uint8_t status = this->reading_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RESULT_RANGE_STATUS));

  // Check for errors
//  if (status != VL6180XError::VL6180X_ERROR_NONE) {
//    handle_error(status);
//    return;
//  }
//  uint16_t raw_range = this->reading_register16(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RESULT_RANGE_VAL));
//  float scaled_range = static_cast<float>(raw_range) * scaling_;

//  ESP_LOGD(TAG, "Raw range: %u, Scaled range: %.2f", raw_range, scaled_range);

//  if (this->distance_sensor_ != nullptr)
//    distance_sensor_->publish_state(scaled_range);

  // ALS state machine
  switch (this->state_) {
    case SENSOR_STATE_IDLE:
      this->start_measurement();
      break;

    case SENSOR_STATE_WAITING_FOR_DATA:
      this->check_measurement();
      break;

    case SENSOR_STATE_DATA_READY:
      // If no errors, read the sensor data
      uint16_t range = this->reading_register16(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RESULT_RANGE_VAL));
      float als = read_als();

      // Publish the sensor data
	    if (this->distance_sensor_ != nullptr)
        distance_sensor_->publish_state(static_cast<float>(range));
	    if (this->als_sensor_ != nullptr)
        als_sensor_->publish_state(als);

	    // Go back to idle state
      this->state_ = SENSOR_STATE_IDLE;
      break;
  }
}

// Check for errors
void VL6180XSensor::handle_error(uint8_t status) {
  switch (status) {
    case VL6180XError::VL6180X_ERROR_NONE:
      // No error
      break;
    case VL6180XError::VL6180X_ERROR_SYSERR_1:
      ESP_LOGE(TAG, "System error; VCSEL Continuity Test; No measurement possible");
      return;
    case VL6180XError::VL6180X_ERROR_SYSERR_2:
      ESP_LOGE(TAG, "System error; VCSEL Watchdog Test; No measurement possible");
      return;
    case VL6180XError::VL6180X_ERROR_SYSERR_3:
      ESP_LOGE(TAG, "System error; VCSEL Watchdog; No measurement possible");
      return;
    case VL6180XError::VL6180X_ERROR_SYSERR_4:
      ESP_LOGE(TAG, "System error; PLL1 Lock; No measurement possible");
      return;
    case VL6180XError::VL6180X_ERROR_SYSERR_5:
      ESP_LOGE(TAG, "System error; PLL2 Lock; No measurement possible");
      return;
    case VL6180XError::VL6180X_ERROR_ECEFAIL_6:
      ESP_LOGE(TAG, "Early Convergence Estimate failed");
      return;
    case VL6180XError::VL6180X_ERROR_NOCONVERGE_7:
      ESP_LOGE(TAG, "No target detected, system did not converge before the specified max convergence time limit");
      return;
    case VL6180XError::VL6180X_ERROR_RANGEIGNORE_8:
      ESP_LOGE(TAG, "Ignore threshold check failed");
      return;
	// Bits 9-10 Not used.
    case VL6180XError::VL6180X_ERROR_SNR_11:
      ESP_LOGE(TAG, "Ambient conditions too high. Measurement not valid");
      return;
	// 12/14 Range value < 0
    // If the target is very close (0-10mm) and the offset
    // is not correctly calibrated it could lead to a small
    // negative value.
    case VL6180XError::VL6180X_ERROR_RAWUFLOW_12:
      ESP_LOGE(TAG, "Raw range algo underflow; Target too close");
      return;
	// 13/15 Range value out of range. This occurs when the
    // target is detected by the device but is placed at a
    // high distance (> 200mm) resulting in internal
    // variable overflow.
    case VL6180XError::VL6180X_ERROR_RAWOFLOW_13:
      ESP_LOGE(TAG, "Raw range algo overflow; Target too far");
      return;
    case VL6180XError::VL6180X_ERROR_RANGEUFLOW_14:
      ESP_LOGE(TAG, "Range algo underflow; Target too close");
      return;
    case VL6180XError::VL6180X_ERROR_RANGEOFLOW_15:
      ESP_LOGE(TAG, "Range algo overflow; Target too far");
      return;
    case VL6180XError::VL6180X_ERROR_RANGINGFILTERED_16:
      ESP_LOGE(TAG, "Distance filtered by Wrap Around Filter (WAF). Occurs when a high reflectance target is detected between 600mm to 1.2m");
      return;
    // Bit 17 Not used.
    case VL6180XError::VL6180X_ERROR_DATANOTREADY_18:
      ESP_LOGE(TAG, "Error returned by VL6180x_RangeGetMeasurementIfReady() when ranging data is not ready.");
      return;
    // Custom errors not sensor generated
    case VL6180XError::VL6180X_ERROR_SENSORVERSIONNOTFOUND_19:
      ESP_LOGE(TAG, "VL6180X sensor version not found is it properly connected?");
      return;
    default:
      ESP_LOGE(TAG, "Unknown error");
      return;
  }
}

void VL6180XSensor::loop() {
  static bool hand_detected = false;
  static std::vector<uint8_t> readings;
/*   static uint32_t last_tap_time = 0;
  static uint32_t last_double_tap_time = 0;
  static bool is_tap = false;
  static uint32_t hover_start_time = 0;
  static bool hover_detected = false; */

  // If data is expected, check if it's ready
  if (!this->data_ready_)
    return;
  if (!(this->reading_register(0x04f) & 0x04))
    return;
  this->data_ready_ = false; // Reset the flag as we've read the data

  uint8_t range = this->reading_register(0x062);
  this->writing_register(0x015, 0x07);  // Clear the interrupt
  if (this->distance_sensor_ != nullptr)
    distance_sensor_->publish_state(static_cast<float>(range));

  // Add the new reading to the readings vector
  readings.push_back(range);

  // If the readings vector is too large, remove the oldest reading
  if (readings.size() > MAX_READINGS) {
    readings.erase(readings.begin());
  }

  // Read the ALS value from the sensor
  float als = read_als();
  // Publish the ALS value
  if (this->als_sensor_ != nullptr)
    als_sensor_->publish_state(als);

/*   if (this->on_gesture_swipe_toward_trigger_ || on_gesture_swipe_away_trigger_  != nullptr) {
    // Check for a swipe gesture
    if (!readings.empty() && readings.front() < 40) {
      if (std::is_sorted(readings.begin(), readings.end())) {
        // Detected a swipe away from the sensor and the value is less than 40
        // Trigger the swipe away gesture
        this->on_gesture_swipe_toward_trigger_->trigger();
		ESP_LOGD(TAG, "Gesture swipe toward detected");
      } else if (std::is_sorted(readings.rbegin(), readings.rend())) {
        // Detected a swipe towards the sensor and the value is less than 40
        // Trigger the swipe towards gesture
        this->on_gesture_swipe_away_trigger_->trigger();
		ESP_LOGD(TAG, "Gesture swipe away detected");
      }
    }
  }

  if (this->on_gesture_single_tap_trigger_ != nullptr) {
    // Single tap gesture detection
    if (range < GESTURE_SINGLE_TAP_THRESHOLD) {
      if (!is_tap) {
        is_tap = true;
        last_tap_time = millis();
      } else if (millis() - last_tap_time > GESTURE_SINGLE_TAP_DELAY_MS) {
        // Detected a single tap
        this->on_gesture_single_tap_trigger_->trigger();
		ESP_LOGD(TAG, "Gesture single tap detected");
        is_tap = false;
      }
    } else {
      is_tap = false;
    }
  }

  if (this->on_gesture_double_tap_trigger_ != nullptr) {
    // Double tap gesture detection
    if (range < GESTURE_DOUBLE_TAP_THRESHOLD) {
      if (!is_tap) {
        is_tap = true;
        last_tap_time = millis();
      } else if (millis() - last_tap_time < GESTURE_DOUBLE_TAP_INTERVAL_MS && millis() - last_double_tap_time > GESTURE_DOUBLE_TAP_DELAY_MS) {
        // Detected a double tap
        this->on_gesture_double_tap_trigger_->trigger();
		ESP_LOGD(TAG, "Gesture double tap detected");
        last_double_tap_time = millis();
        is_tap = false;
      }
    } else {
      is_tap = false;
    }
  }

  if (this->on_gesture_hover_trigger_ != nullptr) {
    // Hover gesture detection
    if (range >= GESTURE_HAND_PRESENCE_THRESHOLD && range <= 190) {
      if (!hover_detected) {
        hover_start_time = millis();
        hover_detected = true;
      } else if (millis() - hover_start_time > GESTURE_HOVER_TIME_MS) {
        // Hand has been hovering for more than HOVER_TIME_MS
        this->on_gesture_hover_trigger_->trigger();
		ESP_LOGD(TAG, "Gesture hover detected");
      }
    } else {
      hover_detected = false;
    }
  } */
}

// I2C low level interfacing

// Write 1 byte to the VL6180X at 'address'
void VL6180XSensor::writing_register(uint16_t reg, uint8_t data) {
  this->write_register16(reg, &data, 1);
}

// Read 1 byte from the VL6180X at 'address'
uint8_t VL6180XSensor::reading_register(uint16_t reg) {
  uint8_t data = 0;
  this->read_register16(reg, &data, 1);
  return data;
}

// Read 2 byte from the VL6180X at 'address'
uint16_t VL6180XSensor::reading_register16(uint16_t reg) {
  uint8_t data[2];
  this->read_register16(reg, data, 2);
  return (data[0]) << 8 | data[1];
}

// Standard Ranging (SR) settings must be loaded onto VL6180X after the device
// has powered up.
void VL6180XSensor::load_settings() {
  // Mandatory Private Registors from Section 9 page 24 of application note
  // required to be loaded onto the VL6180X during the initialisation of the device.
  // https://www.st.com/resource/en/application_note/an4545-vl6180x-basic-ranging-application-note-stmicroelectronics.pdf
  writing_register(0x0207, 0x01);
  writing_register(0x0208, 0x01);
  writing_register(0x0096, 0x00);
  writing_register(0x0097, 0xfd); // RANGE_SCALER = 253
  writing_register(0x00e3, 0x00);
  writing_register(0x00e4, 0x04);
  writing_register(0x00e5, 0x02);
  writing_register(0x00e6, 0x01);
  writing_register(0x00e7, 0x03);
  writing_register(0x00f5, 0x02);
  writing_register(0x00d9, 0x05);
  writing_register(0x00db, 0xce);
  writing_register(0x00dc, 0x03);
  writing_register(0x00dd, 0xf8);
  writing_register(0x009f, 0x00);
  writing_register(0x00a3, 0x3c);
  writing_register(0x00b7, 0x00);
  writing_register(0x00bb, 0x3c);
  writing_register(0x00b2, 0x09);
  writing_register(0x00ca, 0x09);
  writing_register(0x0198, 0x01);
  writing_register(0x01b0, 0x17);
  writing_register(0x01ad, 0x00);
  writing_register(0x00ff, 0x05);
  writing_register(0x0100, 0x05);
  writing_register(0x0199, 0x05);
  writing_register(0x01a6, 0x1b);
  writing_register(0x01ac, 0x3e);
  writing_register(0x01a7, 0x1f);
  writing_register(0x0030, 0x00);
  // Recommended : Public registers - See data sheet for more detail
  writing_register(0x0011, 0x10); // Enables polling for 'New Sample ready'
                                  // when measurement completes
  writing_register(0x010a, 0x30); // Set the averaging sample period
                                  // (compromise between lower noise and
                                  // increased execution time)
  writing_register(0x003f, 0x46); // Sets the light and dark gain (upper
                                  // nibble). Dark gain should not be
                                  // changed.
  writing_register(0x0031, 0xFF); // Sets the # of range measurements after
                                  // which auto calibration of system is performed
  writing_register(0x0041, 0x63); // Set ALS integration time to 100ms
  writing_register(0x002e, 0x01); // Perform a single temperature calibration of the ranging sensor

  writing_register(0x0014, 0x04); // Enables the notification of a new value being available.
  //writing_register(0x001b, 0x80); // continuous mode interval

  // Optional: Public registers - See data sheet for more detail
  //writing_register(0x001b, 0x09); // Set default ranging inter-measurement
                                    // period to 100ms
  //writing_register(0x003e, 0x31); // Set default ALS inter-measurement period
                                    // to 500ms
  //writing_register(0x0014, 0x24); // Configures interrupt on 'New Sample
                                    // Ready threshold event'
}

// RANGE_SCALER values for 1x, 2x, 3x scaling
const uint16_t VL6180XSensor::ScalerValues[] = {0, 253, 127, 84};

void VL6180XSensor::set_scaling(uint8_t new_scaling) {
  uint8_t const DefaultCrosstalkValidHeight = 20; // default value of SYSRANGE__CROSSTALK_VALID_HEIGHT

  // do nothing if scaling value is invalid
  if (new_scaling < 1 || new_scaling > 3) {
    ESP_LOGE(TAG, "Invalid scaling factor: %u", new_scaling);
    return;
  }

  scaling_ = new_scaling;
  ESP_LOGI(TAG, "Setting scaling to %ux", scaling_);
  // Use the enum values directly
  uint8_t scaler_value;
  switch (scaling_) {
    case 1:
      scaler_value = static_cast<uint8_t>(VL6180XScalerFactor::VL6180X_SCALER_FACTOR_1);
      break;
    case 2:
      scaler_value = static_cast<uint8_t>(VL6180XScalerFactor::VL6180X_SCALER_FACTOR_2);
      break;
    case 3:
      scaler_value = static_cast<uint8_t>(VL6180XScalerFactor::VL6180X_SCALER_FACTOR_3);
      break;
    default:
      // This shouldn't happen due to the earlier check, but it's good practice
      ESP_LOGE(TAG, "Invalid scaling factor: %u", scaling_);
      return;
  }
  // used to be register16
  //writing_register(VL6180XRegister::VL6180X_REG_RANGE_SCALER, scaler_value);
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RANGE_SCALER), scaler_value);
  ESP_LOGI(TAG, "Scaling set. New scaler value: 0x%02X", scaler_value);
  // Apply scaling on part-to-part offset
  //writing_register(VL6180XRegister::VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET, ptp_offset_ / scaling_);
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET), ptp_offset_ / scaling_);

  // Apply scaling on CrossTalkValidHeight
  //writing_register(VL6180XRegister::VL6180X_REG_SYSRANGE_CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / scaling_);
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSRANGE_CROSSTALK_VALID_HEIGHT), DefaultCrosstalkValidHeight / scaling_);

  // Enable early convergence estimate only at 1x scaling
  //uint8_t rce = reading_register(VL6180XRegister::VL6180X_REG_SYSRANGE_RANGE_CHECK_ENABLES);
  //writing_register(VL6180XRegister::VL6180X_REG_SYSRANGE_RANGE_CHECK_ENABLES, (rce & 0xFE) | (scaling_ == 1));
  uint8_t rce = reading_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSRANGE_RANGE_CHECK_ENABLES));
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSRANGE_RANGE_CHECK_ENABLES), (rce & 0xFE) | (scaling_ == 1));
  ESP_LOGD(TAG, "Scaling set. New scaler value: 0x%02X", scaler_value);
  // Log the scaling setting
  ESP_LOGI(TAG, "VL6180X scaling set to %ux", scaling_);
}


// Assuming there the device is in a 200lux environment, by starting a single shot ambient
// light measurement, the example below shows how to interpret the value read.
// When ambient light measurement is complete, the value is read from the
// RESULT__ALS_VAL register (0x0050) = 0x01 (high byte) and read from 0x0051 = 0x38
// (low byte).The values read should be concatenated together giving a values of 0x0138. By
// converting this hex value to decimal, the resulting value is 312. Then this value should be
// multiplied by the ALS calibration value, 0.32 count/lux is the default value, multiplied by 100
// and then divided by the gain and then divided by the ALS integration time. The equation is
// shown below:
// 312 * [ALS calibration value] * 100/ ([ALS gain] * [ALS Integration time]) = 312 * 0.32 * 100 / (1 * 50) = 199.68 Lux.
// The example above uses the default calibration value of the VL6180X from ST
// Microelectronics. If there is cover glass used on top of the VL6180X, the ALS calibration
// value shall be recalculated by the end user to ensure the proper lux measurement.

// Single shot lux measurement
float VL6180XSensor::read_als() {
  // Define the ALS lux resolution and integration time (in milliseconds)
  //float als_lux_resolution = 0.32; // Example value, adjust as needed
  float als_integration_time = 100; // Example value, adjust as needed
  float gainxvalue = 1;
  // Add the code to read the ALS data from the VL6180X sensor
  uint8_t reg;
  reg = reading_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSTEM_INTERRUPT_CONFIG));
  reg &= ~0x38;
  reg |= (0x4 << 3); // IRQ on ALS ready
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSTEM_INTERRUPT_CONFIG), reg);
  // 100 ms integration period
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI), 0);
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO), 100);
  if (static_cast<uint8_t>(gain_) > static_cast<uint8_t>(VL6180XALSGain::VL6180X_ALS_GAIN_40)) {
    gain_ = VL6180XALSGain::VL6180X_ALS_GAIN_40;
  }
  // Analog gain
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSALS_ANALOGUE_GAIN), 0x40 | static_cast<uint8_t>(gain_));
  // Start ALS
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSALS_START), 0x1);
  // Read lux
  float als_count = reading_register16(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_RESULT_ALS_VAL));
  // Clear interrupt
  writing_register(static_cast<uint16_t>(VL6180XRegister::VL6180X_REG_SYSTEM_INTERRUPT_CLEAR), 0x07);

  // Log the ALS value
  if (this->als_sensor_ != nullptr) {
    ESP_LOGD(TAG, "ALS Count: %f", als_count);
    ESP_LOGD(TAG, "ALS Gain: 0x%02X", static_cast<uint8_t>(gain_));
  }
  switch (gain_) {
    case VL6180XALSGain::VL6180X_ALS_GAIN_1:
      gainxvalue = 1;
    break;
    case VL6180XALSGain::VL6180X_ALS_GAIN_1_25:
      gainxvalue = 1.25;
    break;
    case VL6180XALSGain::VL6180X_ALS_GAIN_1_67:
      gainxvalue = 1.67;
	break;
    case VL6180XALSGain::VL6180X_ALS_GAIN_2_5:
      gainxvalue = 2.5;
    break;
    case VL6180XALSGain::VL6180X_ALS_GAIN_5:
      gainxvalue = 5;
    break;
    case VL6180XALSGain::VL6180X_ALS_GAIN_10:
      gainxvalue = 10;
    break;
    case VL6180XALSGain::VL6180X_ALS_GAIN_20:
      gainxvalue = 20;
    break;
    case VL6180XALSGain::VL6180X_ALS_GAIN_40:
      gainxvalue = 40;
    break;
  };
  if (this->als_sensor_ != nullptr){
    ESP_LOGD(TAG, "ALS Gain X Value: %f", gainxvalue);
  }
  // Calculate the light level in lux
  float light_level_lux = als_lux_resolution_without_glass_ * ((float)als_count / (float)gainxvalue) * (100.0f / als_integration_time);

  // If the sensor is behind a glass cover, adjust the light level using the recalibration formula
  if (is_behind_glass_) {
    // Measure the lux value with the glass cover
    float lux_with_glass = light_level_lux;

    // Use the lux value without the glass cover
    // This would need to be done in a controlled environment with the same light conditions
    float lux_without_glass = lux_without_glass_; // Use the member variable here

    // Recalculate the ALS lux resolution
    float als_lux_resolution_with_glass = (lux_without_glass / lux_with_glass) * als_lux_resolution_without_glass_;

    // Recalculate the light level in lux using the new ALS lux resolution
    light_level_lux = als_lux_resolution_with_glass * ((float)als_count / (float)gainxvalue) * (100.0f / als_integration_time);
  }

  return light_level_lux;
}

void VL6180XSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "VL6180X Sensor:");
  LOG_SENSOR("  ", "Range", this);
  LOG_UPDATE_INTERVAL(this);
  LOG_I2C_DEVICE(this);
}

}  // namespace vl6180x
}  // namespace esphome
