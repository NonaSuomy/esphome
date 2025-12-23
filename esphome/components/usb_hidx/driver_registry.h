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
  component->register_device_driver(new Xbox360Driver(component));
#endif

#ifdef HAS_SWITCH_DRIVER
  component->register_device_driver(new SwitchDriver(component));
#endif
}

}  // namespace usb_hidx
}  // namespace esphome
