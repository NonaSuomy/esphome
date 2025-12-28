#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "usb/usb_host.h"
#include <map>
#include <vector>

namespace esphome {
namespace usb_hidx {

class HIDDeviceDriver;
class Xbox360Driver;
class PlayStationDriver;
class SwitchDriver;

struct HIDDevice {
  usb_device_handle_t dev_hdl{nullptr};
  usb_transfer_t *transfer{nullptr};
  usb_transfer_t *media_transfer{nullptr};
  uint8_t interface_num{0};
  uint8_t dev_addr{0};
  uint16_t vid{0};
  uint16_t pid{0};
  uint8_t protocol{0};
  uint8_t out_endpoint{0};
  bool active{false};
  HIDDeviceDriver *driver{nullptr};
};

class HIDDeviceDriver {
 public:
  virtual ~HIDDeviceDriver() = default;
  virtual bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) = 0;
  virtual void process_report(const uint8_t *data, size_t len, HIDDevice *device) = 0;
  virtual const char *get_name() = 0;
};

class USBHIDXComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void register_device_driver(HIDDeviceDriver *driver) { drivers_.push_back(driver); }
  void set_xbox360_driver(Xbox360Driver *driver) { xbox360_driver_ = driver; }
  void register_gamepad_button_a_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_a_sensor_ = sensor; }
  void register_gamepad_button_b_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_b_sensor_ = sensor; }
  void register_keyboard_sensor(text_sensor::TextSensor *sensor) { keyboard_sensor_ = sensor; }

  binary_sensor::BinarySensor *gamepad_button_a_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_b_sensor_{nullptr};
  void register_keyboard_key_sensor(binary_sensor::BinarySensor *sensor, uint8_t keycode);
  void register_mouse_left_sensor(binary_sensor::BinarySensor *sensor) { mouse_left_sensor_ = sensor; }
  void register_mouse_right_sensor(binary_sensor::BinarySensor *sensor) { mouse_right_sensor_ = sensor; }
  void register_mouse_middle_sensor(binary_sensor::BinarySensor *sensor) { mouse_middle_sensor_ = sensor; }
  void register_mouse_x_sensor(sensor::Sensor *sensor) { mouse_x_sensor_ = sensor; }
  void register_mouse_y_sensor(sensor::Sensor *sensor) { mouse_y_sensor_ = sensor; }
  void register_mouse_wheel_sensor(sensor::Sensor *sensor) { mouse_wheel_sensor_ = sensor; }

  text_sensor::TextSensor *get_keyboard_sensor() { return keyboard_sensor_; }
  std::map<uint8_t, binary_sensor::BinarySensor *> &get_keyboard_key_sensors() { return keyboard_key_sensors_; }
  binary_sensor::BinarySensor *get_mouse_left_sensor() { return mouse_left_sensor_; }
  binary_sensor::BinarySensor *get_mouse_right_sensor() { return mouse_right_sensor_; }
  binary_sensor::BinarySensor *get_mouse_middle_sensor() { return mouse_middle_sensor_; }
  sensor::Sensor *get_mouse_x_sensor() { return mouse_x_sensor_; }
  sensor::Sensor *get_mouse_y_sensor() { return mouse_y_sensor_; }
  sensor::Sensor *get_mouse_wheel_sensor() { return mouse_wheel_sensor_; }

  void update_keyboard_leds(HIDDevice *device, uint8_t led_state);
  void send_xbox360_output(HIDDevice *device, const uint8_t *data, size_t len);
  void send_xbox360_interrupt_out(HIDDevice *device, const uint8_t *data, size_t len);
  void send_xbox360_rumble(uint8_t left_motor, uint8_t right_motor);
  void send_playstation_get_report(HIDDevice *device, uint8_t report_id);

  // Generic HID output report (for I2C bridges, etc.)
  esp_err_t send_hid_output_report(HIDDevice *device, const uint8_t *data, size_t len);

  Xbox360Driver *get_xbox360_driver() { return xbox360_driver_; }
  void set_playstation_driver(PlayStationDriver *driver) { playstation_driver_ = driver; }
  PlayStationDriver *get_playstation_driver() { return playstation_driver_; }

 protected:
  usb_host_client_handle_t client_hdl_{nullptr};
  HIDDevice devices_[4];
  int active_channels_{0};
  int connected_devices_{0};
  bool client_registered_{false};

  std::vector<HIDDeviceDriver *> drivers_;
  text_sensor::TextSensor *keyboard_sensor_{nullptr};
  std::map<uint8_t, binary_sensor::BinarySensor *> keyboard_key_sensors_;
  binary_sensor::BinarySensor *mouse_left_sensor_{nullptr};
  binary_sensor::BinarySensor *mouse_right_sensor_{nullptr};
  binary_sensor::BinarySensor *mouse_middle_sensor_{nullptr};
  sensor::Sensor *mouse_x_sensor_{nullptr};
  sensor::Sensor *mouse_y_sensor_{nullptr};
  sensor::Sensor *mouse_wheel_sensor_{nullptr};
  HIDDevice *xbox360_device_{nullptr};
  Xbox360Driver *xbox360_driver_{nullptr};
  PlayStationDriver *playstation_driver_{nullptr};

  static void client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg);
  static void transfer_callback(usb_transfer_t *transfer);
  static void led_control_callback(usb_transfer_t *transfer);
  void handle_new_device(uint8_t address);
  void handle_device_gone(usb_device_handle_t dev_hdl);
  void setup_media_interface(HIDDevice *dev, const usb_config_desc_t *config_desc);
  HIDDevice *find_device_by_handle(usb_device_handle_t dev_hdl);
  HIDDevice *find_device_by_transfer(usb_transfer_t *transfer);
};

}  // namespace usb_hidx
}  // namespace esphome
