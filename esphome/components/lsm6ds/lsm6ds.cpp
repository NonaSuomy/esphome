#include "lsm6ds.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace lsm6ds {

static const char *const TAG = "lsm6ds";

void LSM6DSComponent::setup() {
  uint8_t who_am_i;
  if (!this->read_register(0x0F, &who_am_i, 1)) {
    this->mark_failed();
    ESP_LOGE(TAG, "Communication failed");
    return;
  }

  if (who_am_i == 0x6C) {
    ESP_LOGI(TAG, "Detected LSM6DSOX");
  } else if (who_am_i == 0x6A) {
    ESP_LOGI(TAG, "Detected LSM6DS3TR-C");
  } else {
    this->mark_failed();
    ESP_LOGE(TAG, "Unknown chip ID: 0x%02X", who_am_i);
    return;
  }

  // SW Reset
  this->write_register(0x12, 0x01);
  uint8_t ctrl3;
  uint32_t start = millis();
  do {
    delay(10);
    this->read_register(0x12, &ctrl3, 1);
  } while ((ctrl3 & 0x01) && (millis() - start < 100));

  if (ctrl3 & 0x01) {
    ESP_LOGW(TAG, "SW Reset timed out");
  }

  // Enable BDU (Block Data Update)
  this->write_register(0x12, 0x40 | 0x04); // BDU=1, IF_INC=1

  // Configure Accel
  uint8_t ctrl1_xl = (this->accel_odr_ << 4) | (this->accel_range_ << 2);
  this->write_register(0x10, ctrl1_xl);
  
  switch (this->accel_range_) {
    case LSM6DS_ACCEL_RANGE_2G: accel_sensitivity_ = 0.061e-3f; break;
    case LSM6DS_ACCEL_RANGE_4G: accel_sensitivity_ = 0.122e-3f; break;
    case LSM6DS_ACCEL_RANGE_8G: accel_sensitivity_ = 0.244e-3f; break;
    case LSM6DS_ACCEL_RANGE_16G: accel_sensitivity_ = 0.488e-3f; break;
    default: accel_sensitivity_ = 0; break;
  }

  // Configure Gyro
  uint8_t ctrl2_g = (this->gyro_odr_ << 4);
  if (this->gyro_range_ == LSM6DS_GYRO_RANGE_125DPS) {
    ctrl2_g |= 0x02; // FS_125 = 1
    gyro_sensitivity_ = 4.375e-3f;
  } else {
    ctrl2_g |= (this->gyro_range_ << 2);
    switch (this->gyro_range_) {
      case LSM6DS_GYRO_RANGE_250DPS: gyro_sensitivity_ = 8.75e-3f; break;
      case LSM6DS_GYRO_RANGE_500DPS: gyro_sensitivity_ = 17.50e-3f; break;
      case LSM6DS_GYRO_RANGE_1000DPS: gyro_sensitivity_ = 35.0e-3f; break;
      case LSM6DS_GYRO_RANGE_2000DPS: gyro_sensitivity_ = 70.0e-3f; break;
      default: gyro_sensitivity_ = 0; break;
    }
  }
  this->write_register(0x11, ctrl2_g);
}

void LSM6DSComponent::update() {
  uint8_t status;
  if (!this->read_register(0x1E, &status, 1)) return;

  if (status & 0x01) { // GDA - Gyroscope data available
    uint8_t data[6];
    if (this->read_register(0x22, data, 6)) {
      int16_t gx = encode_uint16(data[1], data[0]);
      int16_t gy = encode_uint16(data[3], data[2]);
      int16_t gz = encode_uint16(data[5], data[4]);
      if (this->gyro_x_sensor_) this->gyro_x_sensor_->publish_state(gx * gyro_sensitivity_);
      if (this->gyro_y_sensor_) this->gyro_y_sensor_->publish_state(gy * gyro_sensitivity_);
      if (this->gyro_z_sensor_) this->gyro_z_sensor_->publish_state(gz * gyro_sensitivity_);
    }
  }

  if (status & 0x02) { // XLDA - Accelerometer data available
    uint8_t data[6];
    if (this->read_register(0x28, data, 6)) {
      int16_t ax = encode_uint16(data[1], data[0]);
      int16_t ay = encode_uint16(data[3], data[2]);
      int16_t az = encode_uint16(data[5], data[4]);
      float grav = 9.80665f;
      if (this->accel_x_sensor_) this->accel_x_sensor_->publish_state(ax * accel_sensitivity_ * grav);
      if (this->accel_y_sensor_) this->accel_y_sensor_->publish_state(ay * accel_sensitivity_ * grav);
      if (this->accel_z_sensor_) this->accel_z_sensor_->publish_state(az * accel_sensitivity_ * grav);
    }
  }

  if (status & 0x04) { // TDA - Temperature data available
    uint8_t data[2];
    if (this->read_register(0x20, data, 2)) {
      int16_t temp = encode_uint16(data[1], data[0]);
      if (this->temperature_sensor_) this->temperature_sensor_->publish_state(25.0f + (temp / 256.0f));
    }
  }
}

void LSM6DSComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LSM6DS (LSM6DSOX/LSM6DS3TR-C):");
  LOG_UPDATE_INTERVAL(this);
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Connection failed!");
    return;
  }
  LOG_SENSOR("  ", "Accel X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Accel Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Accel Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

}  // namespace lsm6ds
}  // namespace esphome
