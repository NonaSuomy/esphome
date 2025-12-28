from pathlib import Path
import sys

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
CONF_DEVICE_ID = "device_id"
CONF_LAYOUT = "layout"
CONF_LEFT_BUTTON = "left_button"
CONF_RIGHT_BUTTON = "right_button"
CONF_BUTTON_A = "button_a"
CONF_BUTTON_B = "button_b"
CONF_X_DELTA = "x_delta"
CONF_Y_DELTA = "y_delta"
CONF_TYPE = "type"

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
        cv.Optional(CONF_TYPE, default="generic"): cv.string,
        cv.Optional(CONF_BUTTON_A): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_BUTTON_B): binary_sensor.binary_sensor_schema(),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(USBHIDXComponent),
        cv.Optional(CONF_HUB, default=True): cv.boolean,
        cv.Optional(CONF_KEYBOARD): KEYBOARD_SCHEMA,
        cv.Optional(CONF_MOUSE): MOUSE_SCHEMA,
        cv.Optional(CONF_GAMEPAD): GAMEPAD_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if config[CONF_HUB]:
        cg.add_define("USB_HIDX_HUB_ENABLED")

    # Only include drivers for devices configured in YAML
    if CONF_KEYBOARD in config:
        cg.add_define("USB_HIDX_ENABLE_KEYBOARD")
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
        cg.add_define("USB_HIDX_ENABLE_MOUSE")
    if CONF_GAMEPAD in config:
        cg.add_define("USB_HIDX_ENABLE_GAMEPAD")

    # Auto-discover and add device driver include paths
    component_dir = Path(__file__).parent
    devices_dir = component_dir / "devices"

    # Add component directory itself to include path
    cg.add_build_flag(f"-I{component_dir}")

    # Add devices directory to Python path for platform discovery
    if str(devices_dir) not in sys.path:
        sys.path.insert(0, str(devices_dir))

    if devices_dir.exists():
        for device_folder in devices_dir.iterdir():
            if device_folder.is_dir() and not device_folder.name.startswith("_"):
                # Add device folder to include path
                cg.add_build_flag(f"-I{device_folder}")

                # Register device platforms (text_sensor, binary_sensor, sensor)
                for platform_type in ["text_sensor", "binary_sensor", "sensor"]:
                    platform_file = device_folder / f"{platform_type}.py"
                    if platform_file.exists():
                        # Register as usb_hidx.DEVICE_NAME platform
                        cg.add_platformio_option("lib_ldf_mode", "deep+")
