#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "usb/usb_host.h"
#include <map>
#include <memory>
#include <string>
#include <vector>

// Implemented by the project-local ESP-IDF DWC HCD override.  It reports
// actual hardware channel usage, including channels held by hub/default
// pipes that are not owned by USB HIDX itself.
extern "C" bool usb_dwc_hcd_get_channel_status(int *allocated, int *total, bool *exhausted) __attribute__((weak));

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#else
// Gamepad drivers keep their report parser usable even in a raw-only build.
// ESPHome does not compile binary_sensor.cpp unless a binary-sensor entity is
// configured, so provide a private no-op type for those optional callbacks.
// When USE_BINARY_SENSOR is set, the real ESPHome class above is used.
namespace esphome {
namespace binary_sensor {
class BinarySensor {
 public:
  void publish_state(bool) {}
};
}  // namespace binary_sensor
}  // namespace esphome
#endif

// Optional component support
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#else
// Keep header-only drivers source-compatible in builds that only use button
// or raw-report entities. ESPHome omits sensor.cpp in that case, so these
// callbacks are deliberately no-ops.
namespace esphome {
namespace sensor {
class Sensor {
 public:
  void publish_state(float) {}
};
}  // namespace sensor
}  // namespace esphome
#endif

namespace esphome {
namespace usb_hidx {

class HIDDeviceDriver;
class GenericGamepadDriver;
class InteractDriver;
class Xbox360Driver;
class PlayStationDriver;
class SwitchDriver;

struct HIDDevice {
  usb_device_handle_t dev_hdl{nullptr};
  usb_transfer_t *transfer{nullptr};
  usb_transfer_t *media_transfer{nullptr};
  // Output/control transfers are short-lived but may still be owned by the
  // USB host after a device-gone event. Keep them associated with the device
  // record until their callbacks release them.
  std::vector<usb_transfer_t *> output_transfers;
  bool transfer_retry_pending{false};
  bool media_transfer_retry_pending{false};
  uint8_t transfer_error_count{0};
  uint8_t media_transfer_error_count{0};
  uint32_t transfer_retry_at{0};
  uint32_t media_transfer_retry_at{0};
  uint8_t interface_num{0};
  bool interface_claimed{false};
  uint8_t media_interface_num{0};
  bool media_interface_claimed{false};
  uint8_t dev_addr{0};
  uint16_t vid{0};
  uint16_t pid{0};
  usb_speed_t speed{USB_SPEED_FULL};
  uint8_t protocol{0};
  uint8_t out_endpoint{0};
  bool active{false};
  HIDDeviceDriver *driver{nullptr};
};

struct HIDDeviceSelector {
  uint16_t vid{0};
  uint16_t pid{0};

  bool matches(const HIDDevice *device) const {
    return device != nullptr && (vid == 0 || vid == device->vid) && (pid == 0 || pid == device->pid);
  }
};

class HIDDeviceDriver {
 public:
  virtual ~HIDDeviceDriver() = default;
  // The registry stores configured driver templates. Every physical USB
  // device receives its own clone so state from two identical devices cannot
  // overwrite one another.
  virtual HIDDeviceDriver *clone() const = 0;
  virtual bool match_device(uint8_t protocol, uint16_t vid, uint16_t pid) = 0;
  virtual void process_report(const uint8_t *data, size_t len, HIDDevice *device) = 0;
  virtual const char *get_name() = 0;
  virtual void on_device_ready(HIDDevice *device) { (void) device; }
  virtual void on_device_removed() {}
  virtual void send_rumble(HIDDevice *device, uint8_t left_motor, uint8_t right_motor) {
    (void) device;
    (void) left_motor;
    (void) right_motor;
  }
};

class USBHIDXComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void register_device_driver(HIDDeviceDriver *driver) { drivers_.emplace_back(driver); }
  void set_xbox360_driver(Xbox360Driver *driver) { xbox360_driver_ = driver; }
  // These pointers are kept available to header-only protocol drivers even
  // when a raw-only build omits ESPHome's binary-sensor component.  In that
  // case BinarySensor is the no-op type declared above and no entity exists.
  void register_gamepad_button_a_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_a_sensor_ = sensor; }
  void register_gamepad_button_b_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_b_sensor_ = sensor; }
  void register_gamepad_button_x_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_x_sensor_ = sensor; }
  void register_gamepad_button_y_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_y_sensor_ = sensor; }
  void register_gamepad_button_l_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_l_sensor_ = sensor; }
  void register_gamepad_button_r_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_r_sensor_ = sensor; }
  void register_gamepad_button_zl_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_zl_sensor_ = sensor; }
  void register_gamepad_button_zr_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_zr_sensor_ = sensor; }
  void register_gamepad_button_minus_sensor(binary_sensor::BinarySensor *sensor) {
    gamepad_button_minus_sensor_ = sensor;
  }
  void register_gamepad_button_plus_sensor(binary_sensor::BinarySensor *sensor) {
    gamepad_button_plus_sensor_ = sensor;
  }
  void register_gamepad_button_home_sensor(binary_sensor::BinarySensor *sensor) {
    gamepad_button_home_sensor_ = sensor;
  }
  void register_gamepad_button_capture_sensor(binary_sensor::BinarySensor *sensor) {
    gamepad_button_capture_sensor_ = sensor;
  }
  void register_gamepad_button_cross_sensor(binary_sensor::BinarySensor *sensor) {
    gamepad_button_cross_sensor_ = sensor;
  }
  void register_gamepad_button_circle_sensor(binary_sensor::BinarySensor *sensor) {
    gamepad_button_circle_sensor_ = sensor;
  }
  void register_gamepad_button_l3_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_l3_sensor_ = sensor; }
  void register_gamepad_button_r3_sensor(binary_sensor::BinarySensor *sensor) { gamepad_button_r3_sensor_ = sensor; }
  void register_gamepad_dpad_up_sensor(binary_sensor::BinarySensor *sensor) { gamepad_dpad_up_sensor_ = sensor; }
  void register_gamepad_dpad_down_sensor(binary_sensor::BinarySensor *sensor) { gamepad_dpad_down_sensor_ = sensor; }
  void register_gamepad_dpad_left_sensor(binary_sensor::BinarySensor *sensor) { gamepad_dpad_left_sensor_ = sensor; }
  void register_gamepad_dpad_right_sensor(binary_sensor::BinarySensor *sensor) { gamepad_dpad_right_sensor_ = sensor; }
  binary_sensor::BinarySensor *get_gamepad_button_a_sensor() { return gamepad_button_a_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_b_sensor() { return gamepad_button_b_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_x_sensor() { return gamepad_button_x_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_y_sensor() { return gamepad_button_y_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_l_sensor() { return gamepad_button_l_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_r_sensor() { return gamepad_button_r_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_zl_sensor() { return gamepad_button_zl_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_zr_sensor() { return gamepad_button_zr_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_minus_sensor() { return gamepad_button_minus_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_plus_sensor() { return gamepad_button_plus_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_home_sensor() { return gamepad_button_home_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_capture_sensor() { return gamepad_button_capture_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_cross_sensor() { return gamepad_button_cross_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_circle_sensor() { return gamepad_button_circle_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_l3_sensor() { return gamepad_button_l3_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_button_r3_sensor() { return gamepad_button_r3_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_dpad_up_sensor() { return gamepad_dpad_up_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_dpad_down_sensor() { return gamepad_dpad_down_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_dpad_left_sensor() { return gamepad_dpad_left_sensor_; }
  binary_sensor::BinarySensor *get_gamepad_dpad_right_sensor() { return gamepad_dpad_right_sensor_; }

#ifdef USE_TEXT_SENSOR
  void register_keyboard_sensor(text_sensor::TextSensor *sensor) { keyboard_sensor_ = sensor; }
  text_sensor::TextSensor *get_keyboard_sensor() { return keyboard_sensor_; }
  void register_device_name_sensor(text_sensor::TextSensor *sensor) { device_name_sensor_ = sensor; }
  text_sensor::TextSensor *get_device_name_sensor() { return device_name_sensor_; }
  void register_device_speed_sensor(text_sensor::TextSensor *sensor) { device_speed_sensor_ = sensor; }
  text_sensor::TextSensor *get_device_speed_sensor() { return device_speed_sensor_; }
  void register_last_input_sensor(text_sensor::TextSensor *sensor) { last_input_sensor_ = sensor; }
  text_sensor::TextSensor *get_last_input_sensor() { return last_input_sensor_; }
  void register_resource_status_sensor(text_sensor::TextSensor *sensor) { resource_status_sensor_ = sensor; }
  text_sensor::TextSensor *get_resource_status_sensor() { return resource_status_sensor_; }
#endif

  void register_mouse_left_sensor(binary_sensor::BinarySensor *sensor) { mouse_left_sensor_ = sensor; }
  void register_mouse_right_sensor(binary_sensor::BinarySensor *sensor) { mouse_right_sensor_ = sensor; }
  void register_mouse_middle_sensor(binary_sensor::BinarySensor *sensor) { mouse_middle_sensor_ = sensor; }

#ifdef USE_BINARY_SENSOR
  void register_keyboard_key_sensor(binary_sensor::BinarySensor *sensor, uint8_t keycode);
  void register_raw_binary_sensor(binary_sensor::BinarySensor *sensor, uint8_t offset, uint8_t mask, uint8_t value,
                                  bool has_value, uint16_t vid, uint16_t pid);
#endif

#ifdef USE_SENSOR
  void register_raw_sensor(sensor::Sensor *sensor, uint8_t offset, uint8_t length, bool is_signed, float scale,
                           float bias, uint16_t vid, uint16_t pid);
  void register_mouse_x_sensor(sensor::Sensor *sensor) { mouse_x_sensor_ = sensor; }
  void register_mouse_y_sensor(sensor::Sensor *sensor) { mouse_y_sensor_ = sensor; }
  void register_mouse_wheel_sensor(sensor::Sensor *sensor) { mouse_wheel_sensor_ = sensor; }
  sensor::Sensor *get_mouse_x_sensor() { return mouse_x_sensor_; }
  sensor::Sensor *get_mouse_y_sensor() { return mouse_y_sensor_; }
  sensor::Sensor *get_mouse_wheel_sensor() { return mouse_wheel_sensor_; }
  void register_gamepad_left_stick_x_sensor(sensor::Sensor *sensor) { gamepad_left_stick_x_sensor_ = sensor; }
  void register_gamepad_left_stick_y_sensor(sensor::Sensor *sensor) { gamepad_left_stick_y_sensor_ = sensor; }
  void register_gamepad_right_stick_x_sensor(sensor::Sensor *sensor) { gamepad_right_stick_x_sensor_ = sensor; }
  void register_gamepad_right_stick_y_sensor(sensor::Sensor *sensor) { gamepad_right_stick_y_sensor_ = sensor; }
  sensor::Sensor *get_gamepad_left_stick_x_sensor() { return gamepad_left_stick_x_sensor_; }
  sensor::Sensor *get_gamepad_left_stick_y_sensor() { return gamepad_left_stick_y_sensor_; }
  sensor::Sensor *get_gamepad_right_stick_x_sensor() { return gamepad_right_stick_x_sensor_; }
  sensor::Sensor *get_gamepad_right_stick_y_sensor() { return gamepad_right_stick_y_sensor_; }
#else
  sensor::Sensor *get_mouse_x_sensor() { return nullptr; }
  sensor::Sensor *get_mouse_y_sensor() { return nullptr; }
  sensor::Sensor *get_mouse_wheel_sensor() { return nullptr; }
  sensor::Sensor *get_gamepad_left_stick_x_sensor() { return nullptr; }
  sensor::Sensor *get_gamepad_left_stick_y_sensor() { return nullptr; }
  sensor::Sensor *get_gamepad_right_stick_x_sensor() { return nullptr; }
  sensor::Sensor *get_gamepad_right_stick_y_sensor() { return nullptr; }
#endif

  binary_sensor::BinarySensor *get_mouse_left_sensor() { return mouse_left_sensor_; }
  binary_sensor::BinarySensor *get_mouse_right_sensor() { return mouse_right_sensor_; }
  binary_sensor::BinarySensor *get_mouse_middle_sensor() { return mouse_middle_sensor_; }
#ifdef USE_BINARY_SENSOR
  std::map<uint8_t, binary_sensor::BinarySensor *> &get_keyboard_key_sensors() { return keyboard_key_sensors_; }
#endif

  void update_keyboard_leds(HIDDevice *device, uint8_t led_state);
  void send_xbox360_output(HIDDevice *device, const uint8_t *data, size_t len);
  void send_xbox360_interrupt_out(HIDDevice *device, const uint8_t *data, size_t len);
  void send_xbox360_rumble(uint8_t left_motor, uint8_t right_motor);
  void send_switch_rumble(uint8_t left_motor, uint8_t right_motor);
  void send_playstation_get_report(HIDDevice *device, uint8_t report_id);

  // Generic HID output report (for I2C bridges, etc.)
  esp_err_t send_hid_output_report(HIDDevice *device, const uint8_t *data, size_t len);

  Xbox360Driver *get_xbox360_driver() { return xbox360_driver_; }
  void set_playstation_driver(PlayStationDriver *driver) { playstation_driver_ = driver; }
  PlayStationDriver *get_playstation_driver() { return playstation_driver_; }
  void set_switch_driver(SwitchDriver *driver) { switch_driver_ = driver; }
  SwitchDriver *get_switch_driver() { return switch_driver_; }

 protected:
  // Legacy protocol drivers are header-only and historically accessed these
  // shared entity pointers directly. Keep those drivers source-compatible
  // while retaining the component's other state as protected.
  friend class GenericGamepadDriver;
  friend class InteractDriver;
  friend class PlayStationDriver;
  friend class SwitchDriver;

  usb_host_client_handle_t client_hdl_{nullptr};
  // Device records are allocated as devices arrive. A fixed array here would
  // impose an arbitrary HIDX limit that is unrelated to the USB controller's
  // actual endpoint/channel and memory resources. unique_ptr keeps each
  // HIDDevice address stable while transfers are in flight.
  std::vector<std::unique_ptr<HIDDevice>> devices_;
  int active_channels_{0};
  int connected_devices_{0};
  bool client_registered_{false};

  // Templates live for the component lifetime; active_driver_instances_ owns
  // one independent stateful instance for each matched physical device.
  std::vector<std::unique_ptr<HIDDeviceDriver>> drivers_;
  std::vector<std::unique_ptr<HIDDeviceDriver>> active_driver_instances_;
  binary_sensor::BinarySensor *gamepad_button_a_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_b_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_x_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_y_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_l_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_r_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_zl_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_zr_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_minus_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_plus_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_home_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_capture_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_cross_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_circle_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_l3_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_button_r3_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_dpad_up_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_dpad_down_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_dpad_left_sensor_{nullptr};
  binary_sensor::BinarySensor *gamepad_dpad_right_sensor_{nullptr};
  binary_sensor::BinarySensor *mouse_left_sensor_{nullptr};
  binary_sensor::BinarySensor *mouse_right_sensor_{nullptr};
  binary_sensor::BinarySensor *mouse_middle_sensor_{nullptr};
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *keyboard_sensor_{nullptr};
  text_sensor::TextSensor *device_name_sensor_{nullptr};
  text_sensor::TextSensor *device_speed_sensor_{nullptr};
  text_sensor::TextSensor *last_input_sensor_{nullptr};
  text_sensor::TextSensor *resource_status_sensor_{nullptr};
  std::string last_resource_status_;
  uint32_t resource_status_last_check_ms_{0};
#endif
#ifdef USE_BINARY_SENSOR
  std::map<uint8_t, binary_sensor::BinarySensor *> keyboard_key_sensors_;
  struct RawBinaryBinding {
    binary_sensor::BinarySensor *sensor;
    uint8_t offset;
    uint8_t mask;
    uint8_t value;
    bool has_value;
    HIDDeviceSelector selector;
  };
  std::vector<RawBinaryBinding> raw_binary_bindings_;
#endif
#ifdef USE_SENSOR
  sensor::Sensor *mouse_x_sensor_{nullptr};
  sensor::Sensor *mouse_y_sensor_{nullptr};
  sensor::Sensor *mouse_wheel_sensor_{nullptr};
  sensor::Sensor *gamepad_left_stick_x_sensor_{nullptr};
  sensor::Sensor *gamepad_left_stick_y_sensor_{nullptr};
  sensor::Sensor *gamepad_right_stick_x_sensor_{nullptr};
  sensor::Sensor *gamepad_right_stick_y_sensor_{nullptr};
  struct RawSensorBinding {
    sensor::Sensor *sensor;
    uint8_t offset;
    uint8_t length;
    bool is_signed;
    float scale;
    float bias;
    HIDDeviceSelector selector;
  };
  std::vector<RawSensorBinding> raw_sensor_bindings_;
#endif
  HIDDevice *xbox360_device_{nullptr};
  HIDDevice *switch_device_{nullptr};
  Xbox360Driver *xbox360_driver_{nullptr};
  PlayStationDriver *playstation_driver_{nullptr};
  SwitchDriver *switch_driver_{nullptr};

  static void client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg);
  static void transfer_callback(usb_transfer_t *transfer);
  static void led_control_callback(usb_transfer_t *transfer);
  void handle_new_device(uint8_t address);
  void handle_device_gone(usb_device_handle_t dev_hdl);
  void setup_media_interface(HIDDevice *dev, const usb_config_desc_t *config_desc);
  HIDDevice *find_device_by_handle(usb_device_handle_t dev_hdl);
  HIDDevice *find_device_by_transfer(usb_transfer_t *transfer);
  void service_transfer_retries();
  void try_finalize_device(HIDDevice *device);
  void track_output_transfer(HIDDevice *device, usb_transfer_t *transfer);
  void untrack_output_transfer(HIDDevice *device, usb_transfer_t *transfer);
  void publish_raw_bindings(HIDDevice *device, const uint8_t *data, size_t len);
  bool has_raw_bindings_for(const HIDDevice *device) const;
#ifdef USE_TEXT_SENSOR
  void update_resource_status();
#endif
};

}  // namespace usb_hidx
}  // namespace esphome
