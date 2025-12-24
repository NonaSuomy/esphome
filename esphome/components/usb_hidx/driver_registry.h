#pragma once

// Only include drivers that are enabled in YAML config

#ifdef USB_HIDX_ENABLE_KEYBOARD
#if __has_include("keyboard_driver.h")
#include "keyboard_driver.h"
#define HAS_KEYBOARD_DRIVER
#endif
#endif

#ifdef USB_HIDX_ENABLE_MOUSE
#if __has_include("mouse_driver.h")
#include "mouse_driver.h"
#define HAS_MOUSE_DRIVER
#endif
#endif

#ifdef USB_HIDX_ENABLE_GAMEPAD
#if __has_include("generic_gamepad_driver.h")
#include "generic_gamepad_driver.h"
#define HAS_GENERIC_GAMEPAD_DRIVER
#endif
#if __has_include("xbox360_driver.h")
#include "xbox360_driver.h"
#define HAS_XBOX360_DRIVER
#endif
#if __has_include("switch_driver.h")
#include "switch_driver.h"
#define HAS_SWITCH_DRIVER
#endif
#endif

namespace esphome {
namespace usb_hidx {

class USBHIDXComponent;  // Forward declaration

inline void register_all_drivers(USBHIDXComponent *component) {
#ifdef HAS_KEYBOARD_DRIVER
  component->register_device_driver(new KeyboardDriver(component));
#endif

#ifdef HAS_MOUSE_DRIVER
  component->register_device_driver(new MouseDriver(component));
#endif

#ifdef HAS_GENERIC_GAMEPAD_DRIVER
  component->register_device_driver(new GenericGamepadDriver(component));
#endif

#ifdef HAS_XBOX360_DRIVER
  auto *xbox360 = new Xbox360Driver(component);
  component->register_device_driver(xbox360);
  component->set_xbox360_driver(xbox360);
  // Pass stored sensors to driver if they were registered
  if (component->gamepad_button_a_sensor_)
    xbox360->set_button_a_sensor(component->gamepad_button_a_sensor_);
  if (component->gamepad_button_b_sensor_)
    xbox360->set_button_b_sensor(component->gamepad_button_b_sensor_);
#endif

#ifdef HAS_SWITCH_DRIVER
  component->register_device_driver(new SwitchDriver(component));
#endif
}

}  // namespace usb_hidx
}  // namespace esphome
