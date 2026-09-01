#pragma once

// Every driver is opt-in. The Python component emits one
// USB_HIDX_ENABLE_<DRIVER> define for each driver requested by YAML. Most
// drivers are header-only, so family-wide selection wastes flash/RAM.

#ifdef USB_HIDX_ENABLE_KEYBOARD
#include "devices/keyboard/keyboard_driver.h"
#define HAS_KEYBOARD_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_MOUSE
#include "devices/mouse/mouse_driver.h"
#define HAS_MOUSE_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_GENERIC_GAMEPAD
#include "devices/generic_gamepad/generic_gamepad_driver.h"
#define HAS_GENERIC_GAMEPAD_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_XBOX360
#include "devices/xbox360/xbox360_driver.h"
#define HAS_XBOX360_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_XBOXONE
#include "devices/xboxone/xboxone_driver.h"
#define HAS_XBOXONE_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_PLAYSTATION
#include "devices/playstation/playstation_driver.h"
#define HAS_PLAYSTATION_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_STEAM
#include "devices/steam/steam_driver.h"
#define HAS_STEAM_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_STADIA
#include "devices/stadia/stadia_driver.h"
#define HAS_STADIA_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_SWITCH
#include "devices/switch/switch_driver.h"
#define HAS_SWITCH_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_WIIMOTE
#include "devices/wiimote/wiimote_driver.h"
#define HAS_WIIMOTE_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_THRUSTMASTER
#include "devices/thrustmaster/thrustmaster_driver.h"
#define HAS_THRUSTMASTER_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_TOUCHSCREEN
#include "devices/touchscreen/touchscreen_driver.h"
#define HAS_TOUCHSCREEN_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_INTERACT
#include "devices/interact/interact_driver.h"
#define HAS_INTERACT_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_LOGITECH
#include "devices/logitech/logitech_driver.h"
#define HAS_LOGITECH_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_MCE_REMOTE
#include "devices/mce_remote/mce_remote_driver.h"
#define HAS_MCE_REMOTE_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_MCP2221
#include "devices/mcp2221/mcp2221_driver.h"
#define HAS_MCP2221_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_CP2112
#include "devices/cp2112/cp2112_driver.h"
#define HAS_CP2112_DRIVER
#endif

#ifdef USB_HIDX_ENABLE_FT260
#include "devices/ft260/ft260_driver.h"
#define HAS_FT260_DRIVER
#endif

namespace esphome {
namespace usb_hidx {

class USBHIDXComponent;

inline void register_all_drivers(USBHIDXComponent *component) {
#ifdef HAS_KEYBOARD_DRIVER
  component->register_device_driver(new KeyboardDriver(component));
#endif

#ifdef HAS_MOUSE_DRIVER
  component->register_device_driver(new MouseDriver(component));
#endif

  // Specific gamepad drivers must precede the generic fallback.
#ifdef HAS_XBOX360_DRIVER
  auto *xbox360 = new Xbox360Driver(component);
  component->register_device_driver(xbox360);
  component->set_xbox360_driver(xbox360);
#ifdef USE_BINARY_SENSOR
  if (component->get_gamepad_button_a_sensor())
    xbox360->set_button_a_sensor(component->get_gamepad_button_a_sensor());
  if (component->get_gamepad_button_b_sensor())
    xbox360->set_button_b_sensor(component->get_gamepad_button_b_sensor());
  if (component->get_gamepad_button_x_sensor())
    xbox360->set_button_x_sensor(component->get_gamepad_button_x_sensor());
  if (component->get_gamepad_button_y_sensor())
    xbox360->set_button_y_sensor(component->get_gamepad_button_y_sensor());
  if (component->get_gamepad_button_l_sensor())
    xbox360->set_button_l_sensor(component->get_gamepad_button_l_sensor());
  if (component->get_gamepad_button_r_sensor())
    xbox360->set_button_r_sensor(component->get_gamepad_button_r_sensor());
  if (component->get_gamepad_button_zl_sensor())
    xbox360->set_button_zl_sensor(component->get_gamepad_button_zl_sensor());
  if (component->get_gamepad_button_zr_sensor())
    xbox360->set_button_zr_sensor(component->get_gamepad_button_zr_sensor());
  if (component->get_gamepad_button_minus_sensor())
    xbox360->set_button_minus_sensor(component->get_gamepad_button_minus_sensor());
  if (component->get_gamepad_button_plus_sensor())
    xbox360->set_button_plus_sensor(component->get_gamepad_button_plus_sensor());
  if (component->get_gamepad_button_home_sensor())
    xbox360->set_button_home_sensor(component->get_gamepad_button_home_sensor());
  if (component->get_gamepad_button_l3_sensor())
    xbox360->set_button_l3_sensor(component->get_gamepad_button_l3_sensor());
  if (component->get_gamepad_button_r3_sensor())
    xbox360->set_button_r3_sensor(component->get_gamepad_button_r3_sensor());
  if (component->get_gamepad_dpad_up_sensor())
    xbox360->set_dpad_up_sensor(component->get_gamepad_dpad_up_sensor());
  if (component->get_gamepad_dpad_down_sensor())
    xbox360->set_dpad_down_sensor(component->get_gamepad_dpad_down_sensor());
  if (component->get_gamepad_dpad_left_sensor())
    xbox360->set_dpad_left_sensor(component->get_gamepad_dpad_left_sensor());
  if (component->get_gamepad_dpad_right_sensor())
    xbox360->set_dpad_right_sensor(component->get_gamepad_dpad_right_sensor());
#endif
#ifdef USE_SENSOR
  if (component->get_gamepad_left_stick_x_sensor())
    xbox360->set_axis_lx_sensor(component->get_gamepad_left_stick_x_sensor());
  if (component->get_gamepad_left_stick_y_sensor())
    xbox360->set_axis_ly_sensor(component->get_gamepad_left_stick_y_sensor());
  if (component->get_gamepad_right_stick_x_sensor())
    xbox360->set_axis_rx_sensor(component->get_gamepad_right_stick_x_sensor());
  if (component->get_gamepad_right_stick_y_sensor())
    xbox360->set_axis_ry_sensor(component->get_gamepad_right_stick_y_sensor());
#endif
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
  auto *sw = new SwitchDriver(component);
  component->register_device_driver(sw);
  component->set_switch_driver(sw);
#endif

#ifdef HAS_INTERACT_DRIVER
  component->register_device_driver(new InteractDriver(component));
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

#ifdef HAS_MCE_REMOTE_DRIVER
  component->register_device_driver(new MCERemoteDriver(component));
#endif

#ifdef HAS_MCP2221_DRIVER
  component->register_device_driver(new MCP2221Driver(component));
#endif

#ifdef HAS_CP2112_DRIVER
  component->register_device_driver(new CP2112Driver(component));
#endif

#ifdef HAS_FT260_DRIVER
  component->register_device_driver(new FT260Driver(component));
#endif

#ifdef HAS_GENERIC_GAMEPAD_DRIVER
  component->register_device_driver(new GenericGamepadDriver(component));
#endif
}

}  // namespace usb_hidx
}  // namespace esphome
