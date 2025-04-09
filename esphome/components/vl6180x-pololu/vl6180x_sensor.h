#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace vl6180x {

// Error codes from the original code
enum VL6180XError {
  VL6180X_ERROR_NONE = 0,        // No error; Valid measurement
  VL6180X_ERROR_SYSERR_1 = 1,    // System error; VCSEL Continuity Test
  VL6180X_ERROR_SYSERR_2 = 2,    // System error; VCSEL Watchdog Test
  VL6180X_ERROR_SYSERR_3 = 3,    // System error; VCSEL Watchdog
  VL6180X_ERROR_SYSERR_4 = 4,    // System error; PLL1 Lock
  VL6180X_ERROR_SYSERR_5 = 5,    // System error; PLL2 Lock
  VL6180X_ERROR_ECEFAIL = 6,     // Early Convergence Estimate Check fail
  VL6180X_ERROR_NOCONVERGE = 7,  // Max convergence time reached
  VL6180X_ERROR_RANGEIGNORE = 8, // Range ignore threshold check failed
  VL6180X_ERROR_SNR = 11,        // Ambient conditions too high
  VL6180X_ERROR_RAWUFLOW = 12,   // Raw range underflow
  VL6180X_ERROR_RAWOFLOW = 13,   // Raw range overflow
  VL6180X_ERROR_RANGEUFLOW = 14, // Range underflow
  VL6180X_ERROR_RANGEOFLOW = 15  // Range overflow
};

// Register addresses
enum RegAddr {
  IDENTIFICATION__MODEL_ID = 0x000,
  IDENTIFICATION__MODEL_REV_MAJOR = 0x001,
  IDENTIFICATION__MODEL_REV_MINOR = 0x002,
  IDENTIFICATION__MODULE_REV_MAJOR = 0x003,
  IDENTIFICATION__MODULE_REV_MINOR = 0x004,
  IDENTIFICATION__DATE_HI = 0x006,
  IDENTIFICATION__DATE_LO = 0x007,
  IDENTIFICATION__TIME = 0x008,

  SYSTEM__MODE_GPIO0 = 0x010,
  SYSTEM__MODE_GPIO1 = 0x011,
  SYSTEM__HISTORY_CTRL = 0x012,
  SYSTEM__INTERRUPT_CONFIG_GPIO = 0x014,
  SYSTEM__INTERRUPT_CLEAR = 0x015,
  SYSTEM__FRESH_OUT_OF_RESET = 0x016,
  SYSTEM__GROUPED_PARAMETER_HOLD = 0x017,

  SYSRANGE__START = 0x018,
  SYSRANGE__THRESH_HIGH = 0x019,
  SYSRANGE__THRESH_LOW = 0x01A,
  SYSRANGE__INTERMEASUREMENT_PERIOD = 0x01B,
  SYSRANGE__MAX_CONVERGENCE_TIME = 0x01C,
  SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E,
  SYSRANGE__CROSSTALK_VALID_HEIGHT = 0x021,
  SYSRANGE__EARLY_CONVERGENCE_ESTIMATE = 0x022,
  SYSRANGE__PART_TO_PART_RANGE_OFFSET = 0x024,
  SYSRANGE__RANGE_IGNORE_VALID_HEIGHT = 0x025,
  SYSRANGE__RANGE_IGNORE_THRESHOLD = 0x026,
  SYSRANGE__MAX_AMBIENT_LEVEL_MULT = 0x02C,
  SYSRANGE__RANGE_CHECK_ENABLES = 0x02D,
  SYSRANGE__VHV_RECALIBRATE = 0x02E,
  SYSRANGE__VHV_REPEAT_RATE = 0x031,

  SYSALS__START = 0x038,
  SYSALS__THRESH_HIGH = 0x03A,
  SYSALS__THRESH_LOW = 0x03C,
  SYSALS__INTERMEASUREMENT_PERIOD = 0x03E,
  SYSALS__ANALOGUE_GAIN = 0x03F,
  SYSALS__INTEGRATION_PERIOD = 0x040,

  RESULT__RANGE_STATUS = 0x04D,
  RESULT__ALS_STATUS = 0x04E,
  RESULT__INTERRUPT_STATUS_GPIO = 0x04F,
  RESULT__ALS_VAL = 0x050,
  RESULT__HISTORY_BUFFER_0 = 0x052,
  RESULT__HISTORY_BUFFER_1 = 0x054,
  RESULT__HISTORY_BUFFER_2 = 0x056,
  RESULT__HISTORY_BUFFER_3 = 0x058,
  RESULT__HISTORY_BUFFER_4 = 0x05A,
  RESULT__HISTORY_BUFFER_5 = 0x05C,
  RESULT__HISTORY_BUFFER_6 = 0x05E,
  RESULT__HISTORY_BUFFER_7 = 0x060,
  RESULT__RANGE_VAL = 0x062,
  RESULT__RANGE_RAW = 0x064,
  RESULT__RANGE_RETURN_RATE = 0x066,
  RESULT__RANGE_REFERENCE_RATE = 0x068,
  RESULT__RANGE_RETURN_SIGNAL_COUNT = 0x06C,
  RESULT__RANGE_REFERENCE_SIGNAL_COUNT = 0x070,
  RESULT__RANGE_RETURN_AMB_COUNT = 0x074,
  RESULT__RANGE_REFERENCE_AMB_COUNT = 0x078,
  RESULT__RANGE_RETURN_CONV_TIME = 0x07C,
  RESULT__RANGE_REFERENCE_CONV_TIME = 0x080,

  RANGE_SCALER = 0x096,

  READOUT__AVERAGING_SAMPLE_PERIOD = 0x10A,
  FIRMWARE__BOOTUP = 0x119,
  FIRMWARE__RESULT_SCALER = 0x120,
  I2C_SLAVE__DEVICE_ADDRESS = 0x212,
  INTERLEAVED_MODE__ENABLE = 0x2A3,
};

class VL6180XSensor : public PollingComponent, public sensor::Sensor, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_distance_sensor(sensor::Sensor *distance) { distance_sensor_ = distance; }
  void set_als_sensor(sensor::Sensor *als) { als_sensor_ = als; }
  void set_scaling(uint8_t new_scaling);

 protected:
  // I2C register access methods
  void write_reg(uint16_t reg, uint8_t value) {
    uint8_t buffer[3];
    buffer[0] = (reg >> 8) & 0xFF;  // reg high byte
    buffer[1] = reg & 0xFF;         // reg low byte
    buffer[2] = value;
    this->write(buffer, 3);
  }

  void write_reg_16bit(uint16_t reg, uint16_t value) {
    uint8_t buffer[4];
    buffer[0] = (reg >> 8) & 0xFF;    // reg high byte
    buffer[1] = reg & 0xFF;           // reg low byte
    buffer[2] = (value >> 8) & 0xFF;  // value high byte
    buffer[3] = value & 0xFF;         // value low byte
    this->write(buffer, 4);
  }

  void write_reg_32bit(uint16_t reg, uint32_t value) {
    uint8_t buffer[6];
    buffer[0] = (reg >> 8) & 0xFF;     // reg high byte
    buffer[1] = reg & 0xFF;            // reg low byte
    buffer[2] = (value >> 24) & 0xFF;  // value highest byte
    buffer[3] = (value >> 16) & 0xFF;
    buffer[4] = (value >> 8) & 0xFF;
    buffer[5] = value & 0xFF;          // value lowest byte
    this->write(buffer, 6);
  }

  uint8_t read_reg(uint16_t reg) {
    uint8_t buffer[2];
    buffer[0] = (reg >> 8) & 0xFF;  // reg high byte
    buffer[1] = reg & 0xFF;         // reg low byte
    this->write(buffer, 2);
    
    uint8_t value;
    this->read(&value, 1);
    return value;
  }

  uint16_t read_reg_16bit(uint16_t reg) {
    uint8_t buffer[2];
    buffer[0] = (reg >> 8) & 0xFF;  // reg high byte
    buffer[1] = reg & 0xFF;         // reg low byte
    this->write(buffer, 2);
    
    uint8_t value[2];
    this->read(value, 2);
    return ((uint16_t)value[0] << 8) | value[1];
  }

  uint32_t read_reg_32bit(uint16_t reg) {
    uint8_t buffer[2];
    buffer[0] = (reg >> 8) & 0xFF;  // reg high byte
    buffer[1] = reg & 0xFF;         // reg low byte
    this->write(buffer, 2);
    
    uint8_t value[4];
    this->read(value, 4);
    return ((uint32_t)value[0] << 24) | ((uint32_t)value[1] << 16) |
           ((uint32_t)value[2] << 8) | value[3];
  }

  bool init_sensor_();
  uint16_t read_range_single();
  uint16_t read_ambient_single();
  uint8_t read_range_status();

  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *als_sensor_{nullptr};
  uint8_t scaling_{1};
};

}  // namespace vl6180x
}  // namespace esphome
