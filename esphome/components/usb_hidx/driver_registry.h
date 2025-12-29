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
#if __has_include("devices/playstation/playstation_driver.h")
#include "devices/playstation/playstation_driver.h"
#define HAS_PLAYSTATION_DRIVER
#endif
#if __has_include("devices/xboxone/xboxone_driver.h")
#include "devices/xboxone/xboxone_driver.h"
#define HAS_XBOXONE_DRIVER
#endif
#if __has_include("devices/steam/steam_driver.h")
#include "devices/steam/steam_driver.h"
#define HAS_STEAM_DRIVER
#endif
#if __has_include("devices/stadia/stadia_driver.h")
#include "devices/stadia/stadia_driver.h"
#define HAS_STADIA_DRIVER
#endif
#if __has_include("devices/logitech/logitech_driver.h")
#include "devices/logitech/logitech_driver.h"
#define HAS_LOGITECH_DRIVER
#endif
#if __has_include("devices/wiimote/wiimote_driver.h")
#include "devices/wiimote/wiimote_driver.h"
#define HAS_WIIMOTE_DRIVER
#endif
#if __has_include("devices/thrustmaster/thrustmaster_driver.h")
#include "devices/thrustmaster/thrustmaster_driver.h"
#define HAS_THRUSTMASTER_DRIVER
#endif
#if __has_include("devices/touchscreen/touchscreen_driver.h")
#include "devices/touchscreen/touchscreen_driver.h"
#define HAS_TOUCHSCREEN_DRIVER
#endif
#if __has_include("devices/mcp2221/mcp2221_driver.h")
#include "devices/mcp2221/mcp2221_driver.h"
#define HAS_MCP2221_DRIVER
#endif
#if __has_include("devices/cp2112/cp2112_driver.h")
#include "devices/cp2112/cp2112_driver.h"
#define HAS_CP2112_DRIVER
#endif
#if __has_include("devices/ft260/ft260_driver.h")
#include "devices/ft260/ft260_driver.h"
#define HAS_FT260_DRIVER
#endif
#if __has_include("devices/switch/switch_driver.h")
#include "devices/switch/switch_driver.h"
#define HAS_SWITCH_DRIVER
#endif
#endif

// Include USB I2C header if MCP2221 is enabled
#ifdef HAS_MCP2221_DRIVER
#if __has_include("../usb_i2c/usb_i2c.h")
#include "../usb_i2c/usb_i2c.h"
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

// Register specific gamepad drivers first (they check VID/PID)
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

#ifdef HAS_XBOXONE_DRIVER
  component->register_device_driver(new XboxOneDriver(component));
#endif

#ifdef HAS_PLAYSTATION_DRIVER
  auto *playstation = new PlayStationDriver(component);
  component->register_device_driver(playstation);
  component->set_playstation_driver(playstation);
#endif

#ifdef HAS_STEAM_DRIVER
  component->register_device_driver(new SteamDriver(component));
#endif

#ifdef HAS_STADIA_DRIVER
  component->register_device_driver(new StadiaDriver(component));
#endif

#ifdef HAS_SWITCH_DRIVER
  component->register_device_driver(new SwitchDriver(component));
#endif

#ifdef HAS_LOGITECH_DRIVER
  component->register_device_driver(new LogitechDriver(component));
#endif

#ifdef HAS_WIIMOTE_DRIVER
  component->register_device_driver(new WiimoteDriver(component));
#endif

#ifdef HAS_THRUSTMASTER_DRIVER
  component->register_device_driver(new ThrustmasterDriver(component));
#endif

#ifdef HAS_TOUCHSCREEN_DRIVER
  component->register_device_driver(new TouchscreenDriver(component));
#endif

#ifdef HAS_MCP2221_DRIVER
  auto *mcp2221 = new MCP2221Driver(component);
  component->register_device_driver(mcp2221);
  // Register with USB I2C bus if available
  if (::esphome::usb_i2c::global_usb_i2c_bus) {
    ::esphome::usb_i2c::global_usb_i2c_bus->set_mcp2221_driver(mcp2221);
  }
#endif

#ifdef HAS_CP2112_DRIVER
  component->register_device_driver(new CP2112Driver(component));
#endif

#ifdef HAS_FT260_DRIVER
  component->register_device_driver(new FT260Driver(component));
#endif

// Register generic gamepad last (fallback for unknown gamepads)
#ifdef HAS_GENERIC_GAMEPAD_DRIVER
  component->register_device_driver(new GenericGamepadDriver(component));
#endif
}

}  // namespace usb_hidx
}  // namespace esphome
