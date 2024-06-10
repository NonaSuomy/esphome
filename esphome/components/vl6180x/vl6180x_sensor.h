// vl6180x_sensor.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace vl6180x {

// Register addresses
enum VL6180XRegister {
  VL6180X_DEFAULT_I2C_ADDR = 0x29, // The fixed I2C address

  // Device model identification number
  VL6180X_REG_IDENTIFICATION_MODEL_ID = 0x000,
  // Interrupt configuration
  VL6180X_REG_SYSTEM_INTERRUPT_CONFIG = 0x014,
  // Interrupt clear bits
  VL6180X_REG_SYSTEM_INTERRUPT_CLEAR = 0x015,
  // Fresh out of reset bit
  VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET = 0x016,
  // Trigger Ranging
  VL6180X_REG_SYSRANGE_START = 0x018,
  // Part to part range offset
  VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET = 0x024,
  // Trigger Lux Reading
  VL6180X_REG_SYSALS_START = 0x038,
  // Lux reading gain
  VL6180X_REG_SYSALS_ANALOGUE_GAIN = 0x03F,
  // Integration period for ALS mode, high byte
  VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI = 0x040,
  // Integration period for ALS mode, low byte
  VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO = 0x041,
  // Specific error codes
  VL6180X_REG_RESULT_RANGE_STATUS = 0x04d,
  // Interrupt status
  VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO = 0x04f,
  // Light reading value
  VL6180X_REG_RESULT_ALS_VAL = 0x050,
  // Ranging reading value
  VL6180X_REG_RESULT_RANGE_VAL = 0x062,
  // I2C Slave Device Address
  VL6180X_REG_SLAVE_DEVICE_ADDRESS = 0x212,
};

// ALS Sensor gain levels
enum VL6180XALSGain {
  // Sensor gain levels
  // For better results at low light,  use higher gain.
  // For better results at high light, use a lower gain.
  VL6180X_ALS_GAIN_1 = 0x06,    // 1x gain
  VL6180X_ALS_GAIN_1_25 = 0x05, // 1.25x gain
  VL6180X_ALS_GAIN_1_67 = 0x04, // 1.67x gain
  VL6180X_ALS_GAIN_2_5 = 0x03,  // 2.5x gain
  VL6180X_ALS_GAIN_5 = 0x02,    // 5x gain
  VL6180X_ALS_GAIN_10 = 0x01,   // 10x gain
  VL6180X_ALS_GAIN_20 = 0x00,   // 20x gain
  VL6180X_ALS_GAIN_40 = 0x07,   // 40x gain
};

// Error codes
enum VL6180XError {
  VL6180X_ERROR_NONE = 0,                // Success!
  VL6180X_ERROR_SYSERR_1 = 1,            // System error 1
  VL6180X_ERROR_SYSERR_2 = 2,            // System error 2
  VL6180X_ERROR_SYSERR_3 = 3,            // System error 3
  VL6180X_ERROR_SYSERR_4 = 4,            // System error 4
  VL6180X_ERROR_SYSERR_5 = 5,            // System error 5
  VL6180X_ERROR_ECEFAIL_6 = 6,           // Early convergence estimate fail
  VL6180X_ERROR_NOCONVERGE_7 = 7,        // No target detected
  VL6180X_ERROR_RANGEIGNORE_8 = 8,       // Ignore threshold check failed
  // 9                                   // Not used
  // 10                                  // Not used
  VL6180X_ERROR_SNR_11 = 11,             // Ambient conditions too high
  VL6180X_ERROR_RAWUFLOW_12 = 12,        // Raw range algo underflow
  VL6180X_ERROR_RAWOFLOW_13 = 13,        // Raw range algo overflow
  VL6180X_ERROR_RANGEUFLOW_14 = 14,      // Raw range algo underflow
  VL6180X_ERROR_RANGEOFLOW_15 = 15,      // Raw range algo overflow
  VL6180X_ERROR_RANGINGFILTERED_16 = 16, // Distance filtered by Wrap Around Filter (WAF). Occurs when a high reflectance target is detected between 600mm to 1.2m
  // 17                                  // Not used
  VL6180X_ERROR_DATANOTREADY_18 = 18,    // Error returned by VL6180x_RangeGetMeasurementIfReady() when ranging data is not ready.
};

// VL6180X Sensor class
class VL6180XSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
  public:
    // Sensor states
    enum SensorState {
      SENSOR_STATE_IDLE,
      SENSOR_STATE_WAITING_FOR_DATA,
      SENSOR_STATE_DATA_READY,
    };

    float get_setup_priority() const override { return setup_priority::DATA; }

	// Constructor
    VL6180XSensor(uint8_t address = 0x29, uint32_t update_interval = 60000);

	// Overridden methods from base classes
    void setup() override;
    void update() override;
    void loop() override;
    void dump_config() override;

    // Sensor configuration methods
    void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
    void set_als_sensor(sensor::Sensor *als_sensor) { als_sensor_ = als_sensor; }
    void set_gain(int gain) { gain_ = gain; }
    void set_is_behind_glass(bool is_behind_glass) { is_behind_glass_ = is_behind_glass; }
    void set_lux_without_glass(float lux_without_glass) { lux_without_glass_ = lux_without_glass; }

	// Gesture configuration methods
/* 	void set_gesture_single_tap_interval_ms(uint32_t gesture_single_tap_interval_ms) { gesture_single_tap_interval_ms_ = gesture_single_tap_interval_ms; }
    void set_gesture_double_tap_interval_ms(uint32_t gesture_double_tap_interval_ms) { gesture_double_tap_interval_ms_ = gesture_double_tap_interval_ms; }
    void set_gesture_double_tap_delay_ms(uint32_t gesture_double_tap_delay_ms) { gesture_double_tap_delay_ms_ = gesture_double_tap_delay_ms; }
    void set_gesture_hover_time_ms(uint32_t gesture_hover_time_ms) { gesture_hover_time_ms_ = gesture_hover_time_ms; }
    void set_gesture_hand_presence_threshold(uint8_t gesture_hand_presence_threshold) { gesture_hand_presence_threshold_ = gesture_hand_presence_threshold; }
    void set_gesture_double_tap_threshold(uint8_t gesture_double_tap_threshold) { gesture_double_tap_threshold_ = gesture_double_tap_threshold; } */
	
	// Sensor operation methods
    void start_measurement();
    void check_measurement();
	void handle_error(uint8_t status);

    // Declare triggers for the gestures
/*     Trigger<> *get_gesture_swipe_toward_trigger() const { return on_gesture_swipe_toward_trigger_; }
	Trigger<> *get_gesture_swipe_away_trigger() const { return on_gesture_swipe_away_trigger_; }
	Trigger<> *get_gesture_single_tap_trigger() const { return on_gesture_single_tap_trigger_; }
	Trigger<> *get_gesture_double_tap_trigger() const { return on_gesture_double_tap_trigger_; }
	Trigger<> *get_gesture_hover_trigger() const { return on_gesture_hover_trigger_; } */
  
    // Declare a boolean variable to indicate whether the sensor is behind a glass cover
    bool is_behind_glass_ = false;
  
    // Register read/write methods
    void writing_register(uint16_t reg, uint8_t data);
    uint8_t reading_register(uint16_t reg);
    uint16_t reading_register16(uint16_t reg);

	// Sensor settings methods
    void load_settings();
    uint8_t read_range();
    float read_als(uint8_t gain); // Read the ALS value

  protected:
    // Sensor state and configuratoin variables
    bool data_ready_{false};
    int gain_;
    sensor::Sensor *distance_sensor_;
    sensor::Sensor *als_sensor_;
    float lux_without_glass_;
    float als_lux_resolution_without_glass_;
	
/* 	uint32_t gesture_single_tap_interval_ms_;
	uint32_t gesture_double_tap_interval_ms_;
	uint32_t gesture_single_tap_delay_ms_;
	uint32_t gesture_double_tap_delay_ms_;
	uint32_t gesture_hover_time_ms_;
	uint32_t gesture_hand_presence_threshold_;
	uint32_t gesture_single_tap_threshold_;
	uint32_t gesture_double_tap_threshold_; */
	
    SensorState state_;

    // Declare triggers for the gestures
/*     Trigger<> *on_gesture_swipe_toward_trigger_ = new Trigger<>();
	Trigger<> *on_gesture_swipe_away_trigger_ = new Trigger<>();
	Trigger<> *on_gesture_single_tap_trigger_ = new Trigger<>();
	Trigger<> *on_gesture_double_tap_trigger_ = new Trigger<>();
	Trigger<> *on_gesture_hover_trigger_ = new Trigger<>(); */
};

}  // namespace vl6180x
}  // namespace esphome
