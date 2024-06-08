// vl6180x_sensor.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace vl6180x {

enum VL6180XConstants {
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

    VL6180X_ERROR_NONE = 0,        // Success!
    VL6180X_ERROR_SYSERR_1 = 1,    // System error
    VL6180X_ERROR_SYSERR_5 = 5,    // Sysem error
    VL6180X_ERROR_ECEFAIL = 6,     // Early convergence estimate fail
    VL6180X_ERROR_NOCONVERGE = 7,  // No target detected
    VL6180X_ERROR_RANGEIGNORE = 8, // Ignore threshold check failed
    VL6180X_ERROR_SNR = 11,        // Ambient conditions too high
    VL6180X_ERROR_RAWUFLOW = 12,   // Raw range algo underflow
    VL6180X_ERROR_RAWOFLOW = 13,   // Raw range algo overflow
    VL6180X_ERROR_RANGEUFLOW = 14, // Raw range algo underflow
    VL6180X_ERROR_RANGEOFLOW = 15  // Raw range algo overflow
};

class VL6180XSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
  public:
    enum SensorState {
      SENSOR_STATE_IDLE,
      SENSOR_STATE_WAITING_FOR_DATA,
      SENSOR_STATE_DATA_READY,
      // add more states as needed
    };
	  void start_measurement();
    void check_measurement();
    SensorState state_;

    float get_setup_priority() const override { return setup_priority::DATA; }
    VL6180XSensor(uint8_t address = 0x29, uint32_t update_interval = 60000);
    void setup() override;
    void update() override;
    void loop() override;
    void dump_config() override;
  
    void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
    void set_als_sensor(sensor::Sensor *als_sensor) { als_sensor_ = als_sensor; }
    void set_gain(int gain) { gain_ = gain; }
    void set_is_behind_glass(bool is_behind_glass) { is_behind_glass_ = is_behind_glass; }
    void set_lux_without_glass(float lux_without_glass) { lux_without_glass_ = lux_without_glass; }

    //void trigger_swipe_gesture(int direction);

    // Declare triggers for the gestures
    //Trigger<> *get_swipe_gesture_trigger() const { return on_swipe_gesture_trigger_; }
  
    //1Trigger<int> *on_swipe_gesture_trigger_ = new Trigger<int>();
  
    //Trigger<int> *get_swipe_gesture_trigger() const { return on_swipe_gesture_trigger_; }
  
    //1Trigger<> *get_double_tap_gesture_trigger() const { return on_double_tap_gesture_trigger_; }
    //1Trigger<> *get_hover_gesture_trigger() const { return on_hover_gesture_trigger_; }
  
    // Declare a boolean variable to indicate whether the sensor is behind a glass cover
    bool is_behind_glass_ = false;
  
    // Declare functions
    void writing_register(uint16_t reg, uint8_t data);
    uint8_t reading_register(uint16_t reg);
    uint16_t reading_register16(uint16_t reg);
    void load_settings();
    uint8_t read_range();
    float read_als(uint8_t gain); // Read the ALS value

  protected:
    bool data_ready_{false};
    int gain_;
    sensor::Sensor *distance_sensor_;
    sensor::Sensor *als_sensor_;
    float lux_without_glass_;
    float als_lux_resolution_without_glass_;

    // Declare triggers for the gestures
    //1Trigger<int> *on_swipe_gesture_trigger_ = new Trigger<int>();
    //Trigger<> *on_swipe_gesture_trigger_ = new Trigger<>();
    //1Trigger<> *on_double_tap_gesture_trigger_ = new Trigger<>();
    //1Trigger<> *on_hover_gesture_trigger_ = new Trigger<>();
};

}  // namespace vl6180x
}  // namespace esphome
