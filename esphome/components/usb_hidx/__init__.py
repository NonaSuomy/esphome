from pathlib import Path

import esphome.codegen as cg
from esphome.components import binary_sensor, sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@nonasuomy"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["usb_host"]

CONF_HUB = "hub"
CONF_KEYBOARD = "keyboard"
CONF_MOUSE = "mouse"
CONF_GAMEPAD = "gamepad"
CONF_DRIVERS = "drivers"
CONF_MCE_REMOTE = "mce_remote"
CONF_DEVICE_ID = "device_id"
CONF_VID = "vid"
CONF_PID = "pid"
CONF_LAYOUT = "layout"
CONF_LEFT_BUTTON = "left_button"
CONF_RIGHT_BUTTON = "right_button"
CONF_BUTTON_A = "button_a"
CONF_BUTTON_B = "button_b"
CONF_X_DELTA = "x_delta"
CONF_Y_DELTA = "y_delta"
CONF_TYPE = "type"
CONF_DRIVER = "driver"
CONF_OFFSET = "offset"
CONF_MASK = "mask"
CONF_VALUE = "value"

DRIVER_NAMES = (
    "keyboard",
    "mouse",
    "generic_gamepad",
    "xbox360",
    "xboxone",
    "playstation",
    "steam",
    "stadia",
    "switch",
    "wiimote",
    "thrustmaster",
    "touchscreen",
    "interact",
    "logitech",
    "mce_remote",
    "mcp2221",
    "cp2112",
    "ft260",
)

DRIVER_ALIASES = {
    "generic": "generic_gamepad",
    "generic_gamepad": "generic_gamepad",
    "keyboard": "keyboard",
    "mouse": "mouse",
    "xbox360": "xbox360",
    "xboxone": "xboxone",
    "playstation": "playstation",
    "ps3": "playstation",
    "ps4": "playstation",
    "ps5": "playstation",
    "steam": "steam",
    "stadia": "stadia",
    "switch": "switch",
    "wiimote": "wiimote",
    "thrustmaster": "thrustmaster",
    "touchscreen": "touchscreen",
    "interact": "interact",
    "logitech": "logitech",
    "mce": "mce_remote",
    "mce_remote": "mce_remote",
    "mcp2221": "mcp2221",
    "cp2112": "cp2112",
    "ft260": "ft260",
}

DRIVER_MACROS = {name: "USB_HIDX_ENABLE_" + name.upper() for name in DRIVER_NAMES}

usb_hidx_ns = cg.esphome_ns.namespace("usb_hidx")
USBHIDXComponent = usb_hidx_ns.class_("USBHIDXComponent", cg.Component)

KEYBOARD_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_LAYOUT, default="us"): cv.one_of(
            "us", "uk", "de", "fr", "es", lower=True
        ),
    }
)

MOUSE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_LEFT_BUTTON): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_RIGHT_BUTTON): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_X_DELTA): sensor.sensor_schema(),
        cv.Optional(CONF_Y_DELTA): sensor.sensor_schema(),
    }
)

GAMEPAD_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_TYPE, default="generic"): cv.one_of(
            *DRIVER_ALIASES.keys(), lower=True
        ),
        cv.Optional(CONF_BUTTON_A): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_BUTTON_B): binary_sensor.binary_sensor_schema(),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(USBHIDXComponent),
        cv.Optional(CONF_HUB, default=True): cv.boolean,
        cv.Optional(CONF_DRIVERS, default=[]): cv.ensure_list(
            cv.one_of(*DRIVER_ALIASES.keys(), lower=True)
        ),
        cv.Optional(CONF_MCE_REMOTE, default=False): cv.boolean,
        cv.Optional(CONF_KEYBOARD): KEYBOARD_SCHEMA,
        cv.Optional(CONF_MOUSE): MOUSE_SCHEMA,
        cv.Optional(CONF_GAMEPAD): GAMEPAD_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Keep the driver registry's relative device includes local to this
    # component. This is an include path only; driver source is still pulled
    # in solely by the selected USB_HIDX_ENABLE_* macros below.
    cg.add_build_flag(f"-I{Path(__file__).parent}")

    if config[CONF_HUB]:
        cg.add_define("USB_HIDX_HUB_ENABLED")

    # Select individual drivers at compile time.  The old implementation
    # selected whole families (for example every gamepad driver); keeping the
    # selection at the individual-driver level is important on memory-limited
    # ESP32 targets.
    selected_drivers = {DRIVER_ALIASES[name] for name in config.get(CONF_DRIVERS, [])}

    if CONF_KEYBOARD in config:
        selected_drivers.add("keyboard")
        # Set keyboard layout
        layout = config[CONF_KEYBOARD].get(CONF_LAYOUT, "us")
        if layout == "us":
            cg.add_define("KEYBOARD_LAYOUT_US")
        elif layout == "uk":
            cg.add_define("KEYBOARD_LAYOUT_UK")
        elif layout == "de":
            cg.add_define("KEYBOARD_LAYOUT_DE")
        elif layout == "fr":
            cg.add_define("KEYBOARD_LAYOUT_FR")
        elif layout == "es":
            cg.add_define("KEYBOARD_LAYOUT_ES")
    if CONF_MOUSE in config:
        selected_drivers.add("mouse")
    if CONF_GAMEPAD in config:
        selected_drivers.add(
            DRIVER_ALIASES[config[CONF_GAMEPAD].get(CONF_TYPE, "generic")]
        )
    if config.get(CONF_MCE_REMOTE, False):
        selected_drivers.add("mce_remote")

    for driver in sorted(selected_drivers):
        cg.add_build_flag(f"-D{DRIVER_MACROS[driver]}")


def enable_driver(driver):
    """Enable one driver from a platform declaration.

    Platform entries are allowed to be used without a populated legacy
    ``usb_hidx.keyboard/mouse/gamepad`` block, so they must participate in
    compile-time driver selection themselves.
    """
    canonical = DRIVER_ALIASES.get(driver.lower(), driver.lower())
    if canonical not in DRIVER_MACROS:
        raise cv.Invalid(f"Unknown USB HIDX driver: {driver}")
    cg.add_build_flag(f"-D{DRIVER_MACROS[canonical]}")


def driver_from_type(driver_type):
    canonical = DRIVER_ALIASES.get(driver_type.lower())
    if canonical is None:
        raise cv.Invalid(f"Unknown USB HIDX driver type: {driver_type}")
    return canonical
