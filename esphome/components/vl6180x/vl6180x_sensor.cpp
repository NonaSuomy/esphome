// vl6180x_sensor.cpp
#include "vl6180x_sensor.h"
#include "esphome/core/log.h"

//Automation<int> *automation_id_2;
//esphome::automation::Automation<>:: *automation_id_2;

namespace esphome {
namespace vl6180x {

static const char *TAG = "vl6180x.sensor";
static const uint32_t DOUBLE_TAP_INTERVAL_MS = 500;
static const uint32_t DOUBLE_TAP_DELAY_MS = 1000;
static const uint32_t HOVER_TIME_MS = 3000;
static const uint8_t HAND_PRESENCE_THRESHOLD = 40; // Adjust as needed
static const uint8_t DOUBLE_TAP_THRESHOLD = 40; // Adjust as needed

VL6180XSensor::VL6180XSensor(uint8_t address, uint32_t update_interval) : PollingComponent(update_interval) {
  this->set_i2c_address(address);
}

void VL6180XSensor::setup() {
  // Initialize VL6180X sensor settings
  load_settings();
  //automation_id_2 = new Automation<int>(this->vl6180x_sensor_id);
    //automation_id_2 = new esphome::automation::Automation<>(vl6180x_sensor);
}

void VL6180XSensor::update() {
  this->write_register(0x018, 0x01); // Start a new measurement
  this->data_ready_ = true; // Set the flag to indicate that data is expected
}

//void VL6180XSensor::trigger_swipe_gesture(int direction) {
//  if (this->on_swipe_gesture_trigger_ != nullptr) {
//    this->on_swipe_gesture_trigger_->trigger(direction);
//  }
//}

void VL6180XSensor::loop() {
  static bool hand_detected = false;
  static std::vector<uint8_t> readings;
  static uint32_t last_tap_time = 0;
  static uint32_t last_double_tap_time = 0;
  static bool is_tap = false;
  static uint32_t hover_start_time = 0;
  static bool hover_detected = false;

  // If data is expected, check if it's ready
  if (!this->data_ready_)
    return;
  if (!(this->read_register(0x04f) & 0x04))
    return;
  this->data_ready_ = false; // Reset the flag as we've read the data

  uint8_t range = this->read_register(0x062);
  this->write_register(0x015, 0x07);  // Clear the interrupt
  distance_sensor_->publish_state(range);
	
  // Read the ALS value from the sensor
  float als = read_als(gain_);
  // Publish the ALS value
  als_sensor_->publish_state(als);
	
  // Swipe gesture detection
/*   if (!readings.empty() && readings.front() < 40) {
    if (std::is_sorted(readings.begin(), readings.end())) {
      // Detected a swipe in one direction and the value is less than 40
      // Trigger the swipe gesture with direction 1 (away from the sensor)
      this->on_swipe_gesture_trigger_->trigger(1);
    } else {
      // Detected a swipe in the other direction and the value is less than 40
      // Trigger the swipe gesture with direction 2 (towards the sensor)
      this->on_swipe_gesture_trigger_->trigger(2);
    }
  }

  // Double tap gesture detection
  if (range < DOUBLE_TAP_THRESHOLD) {
    if (!is_tap) {
      is_tap = true;
      last_tap_time = millis();
    } else if (millis() - last_tap_time < DOUBLE_TAP_INTERVAL_MS && millis() - last_double_tap_time > DOUBLE_TAP_DELAY_MS) {
      // Detected a double tap
      this->on_double_tap_gesture_trigger_->trigger();
      last_double_tap_time = millis();
      is_tap = false;
    }
  } else {
    is_tap = false;
  }

  // Hover gesture detection
  if (range >= HAND_PRESENCE_THRESHOLD && range <= 190) {
    if (!hover_detected) {
      hover_start_time = millis();
      hover_detected = true;
    } else if (millis() - hover_start_time > HOVER_TIME_MS) {
      // Hand has been hovering for more than HOVER_TIME_MS
      this->on_hover_gesture_trigger_->trigger();
    }
  } else {
    hover_detected = false;
  } */
}

void VL6180XSensor::write_register(uint16_t reg, uint8_t data) {
  this->write_register16(reg, data, 1);
}

uint8_t VL6180XSensor::read_register(uint8_t reg) {
  uint8_t data = 0;
  this->read_register(reg, data, 1);
  return data;
}

uint16_t VL6180XSensor::read_register16(uint16_t reg) {
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
  write_register(0x0207, 0x01);
  write_register(0x0208, 0x01);
  write_register(0x0096, 0x00);
  write_register(0x0097, 0xfd);
  write_register(0x00e3, 0x00);
  write_register(0x00e4, 0x04);
  write_register(0x00e5, 0x02);
  write_register(0x00e6, 0x01);
  write_register(0x00e7, 0x03);
  write_register(0x00f5, 0x02);
  write_register(0x00d9, 0x05);
  write_register(0x00db, 0xce);
  write_register(0x00dc, 0x03);
  write_register(0x00dd, 0xf8);
  write_register(0x009f, 0x00);
  write_register(0x00a3, 0x3c);
  write_register(0x00b7, 0x00);
  write_register(0x00bb, 0x3c);
  write_register(0x00b2, 0x09);
  write_register(0x00ca, 0x09);
  write_register(0x0198, 0x01);
  write_register(0x01b0, 0x17);
  write_register(0x01ad, 0x00);
  write_register(0x00ff, 0x05);
  write_register(0x0100, 0x05);
  write_register(0x0199, 0x05);
  write_register(0x01a6, 0x1b);
  write_register(0x01ac, 0x3e);
  write_register(0x01a7, 0x1f);
  write_register(0x0030, 0x00);
  // Recommended : Public registers - See data sheet for more detail
  write_register(0x0011, 0x10); // Enables polling for 'New Sample ready'
                                // when measurement completes
  write_register(0x010a, 0x30); // Set the averaging sample period
                                // (compromise between lower noise and
                                // increased execution time)
  write_register(0x003f, 0x46); // Sets the light and dark gain (upper
                                // nibble). Dark gain should not be
                                // changed.
  write_register(0x0031, 0xFF); // Sets the # of range measurements after
                                // which auto calibration of system is performed
  write_register(0x0041, 0x63); // Set ALS integration time to 100ms
  write_register(0x002e, 0x01); // Perform a single temperature calibration of the ranging sensor
  
  write_register(0x0014, 0x04); // Enables the notification of a new value being available.
  //write_register(0x001b, 0x80); // continuous mode interval 
  
  // Optional: Public registers - See data sheet for more detail
  //write_register(0x001b, 0x09); // Set default ranging inter-measurement
                                  // period to 100ms
  //write_register(0x003e, 0x31); // Set default ALS inter-measurement period
                                  // to 500ms
  //write_register(0x0014, 0x24); // Configures interrupt on 'New Sample
                                  // Ready threshold event'
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

float VL6180XSensor::read_als(uint8_t gain) {
  // Define the ALS lux resolution and integration time (in milliseconds)
  float als_lux_resolution = 0.32; // Example value, adjust as needed
  float als_integration_time = 100; // Example value, adjust as needed

  // Add the code to read the ALS data from the VL6180X sensor
  uint16_t reg;
  reg = read_register(VL6180X_REG_SYSTEM_INTERRUPT_CONFIG);
  reg &= ~0x38;
  reg |= (0x4 << 3); 
  write_register(VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, reg);
  write_register(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI, 0);
  write_register(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO, 100);
  if (gain > VL6180X_ALS_GAIN_40) {
    gain = VL6180X_ALS_GAIN_40;
  }
  write_register(VL6180X_REG_SYSALS_ANALOGUE_GAIN, 0x40 | gain);
  write_register(VL6180X_REG_SYSALS_START, 0x1);
  while (4 != ((read_register(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) >> 3) & 0x7))
   ;
  uint16_t als_count = read_register16(VL6180X_REG_RESULT_ALS_VAL);
  write_register(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

  // Calculate the light level in lux
  float light_level_lux = (als_count * als_lux_resolution * als_integration_time) /
                          (als_lux_resolution * gain * als_integration_time);

  // If the sensor is behind a glass cover, adjust the light level using the recalibration formula
  if (is_behind_glass_) {
    // Measure the lux value with the glass cover
    float lux_with_glass = light_level_lux;

    // Measure the lux value without the glass cover
    // This would need to be done in a controlled environment with the same light conditions
    float lux_without_glass = 0.0; // Example value, adjust as needed

    // Recalculate the ALS lux resolution
    als_lux_resolution = (lux_without_glass / lux_with_glass) * als_lux_resolution;

    // Recalculate the light level in lux using the new ALS lux resolution
    light_level_lux = (als_count * als_lux_resolution * als_integration_time) /
                      (als_lux_resolution * gain * als_integration_time);
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
