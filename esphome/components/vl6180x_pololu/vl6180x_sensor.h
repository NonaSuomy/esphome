#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace vl6180x {

// Error codes
enum VL6180XError {
  VL6180X_ERROR_NONE = 0,
  VL6180X_ERROR_SYSERR_1 = 1,
  VL6180X_ERROR_SYSERR_2 = 2,
  VL6180X_ERROR_SYSERR_3 = 3,
  VL6180X_ERROR_SYSERR_4 = 4,
  VL6180X_ERROR_SYSERR_5 = 5,
  VL6180X_ERROR_ECEFAIL = 6,
  VL6180X_ERROR_NOCONVERGE = 7,
  VL6180X_ERROR_RANGEIGNORE = 8,
  // Bits 9-10 Not used
  VL6180X_ERROR_SNR = 11,
  VL6180X_ERROR_RAWUFLOW = 12,
  VL6180X_ERROR_RAWOFLOW = 13,
  VL6180X_ERROR_RANGEUFLOW = 14,
  VL6180X_ERROR_RANGEOFLOW = 15,
  VL6180X_ERROR_RANGINGFILTERED = 16,
  // Bit 17 Not used
  VL6180X_ERROR_DATANOTREADY = 18
};

// ALS Gain Enum
enum VL6180XAlsGain {
  GAIN_20   = 0,
  GAIN_10   = 1,
  GAIN_5    = 2,
  GAIN_2_5  = 3,
  GAIN_1_67 = 4,
  GAIN_1_25 = 5,
  GAIN_1    = 6,
  GAIN_40   = 7
};


class VL6180XSensor : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_distance_sensor(sensor::Sensor *distance) { distance_sensor_ = distance; }
  void set_als_sensor(sensor::Sensor *als) { als_sensor_ = als; }
  void set_scaling(uint8_t new_scaling);

 protected:
  // Register addresses
  enum regAddr {
    IDENTIFICATION__MODEL_ID              = 0x000,
    SYSTEM__MODE_GPIO0                    = 0x010,
    SYSTEM__MODE_GPIO1                    = 0x011,
    SYSTEM__HISTORY_CTRL                  = 0x012,
    SYSTEM__INTERRUPT_CONFIG_GPIO         = 0x014,
    SYSTEM__INTERRUPT_CLEAR               = 0x015,
    SYSTEM__FRESH_OUT_OF_RESET            = 0x016,
    SYSTEM__GROUPED_PARAMETER_HOLD        = 0x017,
    SYSRANGE__START                       = 0x018,
    SYSRANGE__THRESH_HIGH                 = 0x019,
    SYSRANGE__THRESH_LOW                  = 0x01A,
    SYSRANGE__INTERMEASUREMENT_PERIOD     = 0x01B,
    SYSRANGE__MAX_CONVERGENCE_TIME        = 0x01C,
    SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E, // 16-bit
    SYSRANGE__CROSSTALK_VALID_HEIGHT      = 0x021,
    SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  = 0x022, // 16-bit
    SYSRANGE__PART_TO_PART_RANGE_OFFSET   = 0x024,
    SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   = 0x025,
    SYSRANGE__RANGE_IGNORE_THRESHOLD      = 0x026, // 16-bit
    SYSRANGE__MAX_AMBIENT_LEVEL_MULT      = 0x02C,
    SYSRANGE__RANGE_CHECK_ENABLES         = 0x02D,
    SYSRANGE__VHV_RECALIBRATE             = 0x02E,
    SYSRANGE__VHV_REPEAT_RATE             = 0x031,
    SYSALS__START                         = 0x038,
    SYSALS__THRESH_HIGH                   = 0x03A,
    SYSALS__THRESH_LOW                    = 0x03C,
    SYSALS__INTERMEASUREMENT_PERIOD       = 0x03E,
    SYSALS__ANALOGUE_GAIN                 = 0x03F,
    SYSALS__INTEGRATION_PERIOD            = 0x040, // 16-bit
    RESULT__RANGE_STATUS                  = 0x04D,
    RESULT__ALS_STATUS                    = 0x04E,
    RESULT__INTERRUPT_STATUS_GPIO         = 0x04F,
    RESULT__ALS_VAL                       = 0x050, // 16-bit
    RESULT__RANGE_VAL                     = 0x062,
    RESULT__RANGE_RAW                     = 0x064,
    RANGE_SCALING                         = 0x096, // 16-bit
    READOUT__AVERAGING_SAMPLE_PERIOD      = 0x10A,
    I2C_SLAVE__DEVICE_ADDRESS             = 0x212,
    INTERLEAVED_MODE__ENABLE              = 0x2A3,
  };

  bool init_();
  void configure_default_();
  void apply_scaling_dependent_registers();

  // --- Error Handling Helper DECLARATION ---
  void handle_error(uint8_t error_code);

  // --- I2C Helper DECLARATIONS ---
  bool write_byte(uint16_t reg, uint8_t value);
  uint8_t read_byte_(uint16_t reg);
  bool read_byte(uint16_t reg, uint8_t *value);
  bool write_register16(uint16_t reg, uint16_t value);
  bool read_register16(uint16_t reg, uint8_t *value, size_t len);

  // --- ALS Helper DECLARATION ---
  float get_als_gain_multiplier() const;

  // --- Member Variables ---
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *als_sensor_{nullptr};
  uint8_t scaling_{1};
  int8_t ptp_offset_{0};
  VL6180XAlsGain als_gain_{GAIN_1};

 private:
   // --- Static Member DECLARATION ---
  static const char *const TAG;
};

}  // namespace vl6180x
}  // namespace esphome
