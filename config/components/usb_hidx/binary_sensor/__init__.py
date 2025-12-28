import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv

from .. import USBHIDXComponent

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_TYPE = "type"
CONF_KEY = "key"
CONF_LEFT_BUTTON = "left_button"
CONF_RIGHT_BUTTON = "right_button"
CONF_MIDDLE_BUTTON = "middle_button"
CONF_BUTTON_A = "button_a"
CONF_BUTTON_B = "button_b"
CONF_BUTTON_CROSS = "button_cross"
CONF_BUTTON_CIRCLE = "button_circle"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Required(CONF_TYPE): cv.one_of("keyboard", "mouse", "gamepad", lower=True),
        cv.Optional(CONF_KEY): cv.hex_uint8_t,
        cv.Optional(CONF_LEFT_BUTTON): cv.boolean,
        cv.Optional(CONF_RIGHT_BUTTON): cv.boolean,
        cv.Optional(CONF_MIDDLE_BUTTON): cv.boolean,
        cv.Optional(CONF_BUTTON_A): cv.boolean,
        cv.Optional(CONF_BUTTON_B): cv.boolean,
        cv.Optional(CONF_BUTTON_CROSS): cv.boolean,
        cv.Optional(CONF_BUTTON_CIRCLE): cv.boolean,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await binary_sensor.new_binary_sensor(config)

    device_type = config[CONF_TYPE]

    if device_type == "keyboard":
        if CONF_KEY in config:
            cg.add(parent.register_keyboard_key_sensor(var, config[CONF_KEY]))
    elif device_type == "mouse":
        if config.get(CONF_LEFT_BUTTON):
            cg.add(parent.register_mouse_left_sensor(var))
        elif config.get(CONF_RIGHT_BUTTON):
            cg.add(parent.register_mouse_right_sensor(var))
        elif config.get(CONF_MIDDLE_BUTTON):
            cg.add(parent.register_mouse_middle_sensor(var))
    elif device_type == "gamepad":
        if config.get(CONF_BUTTON_A):
            cg.add(parent.register_gamepad_button_a_sensor(var))
        elif config.get(CONF_BUTTON_B):
            cg.add(parent.register_gamepad_button_b_sensor(var))
        elif config.get(CONF_BUTTON_CROSS) or config.get(CONF_BUTTON_CIRCLE):
            # Store sensors in parent, driver will pick them up when initialized
            if config.get(CONF_BUTTON_CROSS):
                cg.add_define("USB_HIDX_PS_BUTTON_CROSS")
                cg.add(
                    cg.RawExpression(
                        f"auto *ps_driver = id({config[CONF_USB_HIDX_ID]}).get_playstation_driver(); if (ps_driver) ps_driver->set_button_cross_sensor({var})"
                    )
                )
            if config.get(CONF_BUTTON_CIRCLE):
                cg.add_define("USB_HIDX_PS_BUTTON_CIRCLE")
                cg.add(
                    cg.RawExpression(
                        f"auto *ps_driver = id({config[CONF_USB_HIDX_ID]}).get_playstation_driver(); if (ps_driver) ps_driver->set_button_circle_sensor({var})"
                    )
                )
