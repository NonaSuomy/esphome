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
CONF_BUTTON_X = "button_x"
CONF_BUTTON_Y = "button_y"
CONF_BUTTON_L = "button_l"
CONF_BUTTON_R = "button_r"
CONF_BUTTON_ZL = "button_zl"
CONF_BUTTON_ZR = "button_zr"
CONF_BUTTON_MINUS = "button_minus"
CONF_BUTTON_PLUS = "button_plus"
CONF_BUTTON_HOME = "button_home"
CONF_BUTTON_CAPTURE = "button_capture"
CONF_BUTTON_L3 = "button_l3"
CONF_BUTTON_R3 = "button_r3"
CONF_BUTTON_CROSS = "button_cross"
CONF_BUTTON_CIRCLE = "button_circle"
CONF_DPAD_UP = "dpad_up"
CONF_DPAD_LEFT = "dpad_left"
CONF_DPAD_RIGHT = "dpad_right"
CONF_DPAD_DOWN = "dpad_down"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Required(CONF_TYPE): cv.one_of("keyboard", "mouse", "gamepad", "switch", lower=True),
        cv.Optional(CONF_KEY): cv.hex_uint8_t,
        cv.Optional(CONF_LEFT_BUTTON): cv.boolean,
        cv.Optional(CONF_RIGHT_BUTTON): cv.boolean,
        cv.Optional(CONF_MIDDLE_BUTTON): cv.boolean,
        cv.Optional(CONF_BUTTON_A): cv.boolean,
        cv.Optional(CONF_BUTTON_B): cv.boolean,
        cv.Optional(CONF_BUTTON_X): cv.boolean,
        cv.Optional(CONF_BUTTON_Y): cv.boolean,
        cv.Optional(CONF_BUTTON_L): cv.boolean,
        cv.Optional(CONF_BUTTON_R): cv.boolean,
        cv.Optional(CONF_BUTTON_ZL): cv.boolean,
        cv.Optional(CONF_BUTTON_ZR): cv.boolean,
        cv.Optional(CONF_BUTTON_MINUS): cv.boolean,
        cv.Optional(CONF_BUTTON_PLUS): cv.boolean,
        cv.Optional(CONF_BUTTON_HOME): cv.boolean,
        cv.Optional(CONF_BUTTON_CAPTURE): cv.boolean,
        cv.Optional(CONF_BUTTON_L3): cv.boolean,
        cv.Optional(CONF_BUTTON_R3): cv.boolean,
        cv.Optional(CONF_BUTTON_CROSS): cv.boolean,
        cv.Optional(CONF_BUTTON_CIRCLE): cv.boolean,
        cv.Optional(CONF_DPAD_UP): cv.boolean,
        cv.Optional(CONF_DPAD_LEFT): cv.boolean,
        cv.Optional(CONF_DPAD_RIGHT): cv.boolean,
        cv.Optional(CONF_DPAD_DOWN): cv.boolean,
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
    elif device_type == "gamepad" or device_type == "switch":
        if config.get(CONF_BUTTON_A):
            cg.add(parent.register_gamepad_button_a_sensor(var))
        elif config.get(CONF_BUTTON_B):
            cg.add(parent.register_gamepad_button_b_sensor(var))
        elif config.get(CONF_BUTTON_X):
            cg.add(parent.register_gamepad_button_x_sensor(var))
        elif config.get(CONF_BUTTON_Y):
            cg.add(parent.register_gamepad_button_y_sensor(var))
        elif config.get(CONF_BUTTON_L):
            cg.add(parent.register_gamepad_button_l_sensor(var))
        elif config.get(CONF_BUTTON_R):
            cg.add(parent.register_gamepad_button_r_sensor(var))
        elif config.get(CONF_BUTTON_ZL):
            cg.add(parent.register_gamepad_button_zl_sensor(var))
        elif config.get(CONF_BUTTON_ZR):
            cg.add(parent.register_gamepad_button_zr_sensor(var))
        elif config.get(CONF_BUTTON_MINUS):
            cg.add(parent.register_gamepad_button_minus_sensor(var))
        elif config.get(CONF_BUTTON_PLUS):
            cg.add(parent.register_gamepad_button_plus_sensor(var))
        elif config.get(CONF_BUTTON_HOME):
            cg.add(parent.register_gamepad_button_home_sensor(var))
        elif config.get(CONF_BUTTON_CAPTURE):
            cg.add(parent.register_gamepad_button_capture_sensor(var))
        elif config.get(CONF_BUTTON_L3):
            cg.add(parent.register_gamepad_button_l3_sensor(var))
        elif config.get(CONF_BUTTON_R3):
            cg.add(parent.register_gamepad_button_r3_sensor(var))
        elif config.get(CONF_DPAD_UP):
            cg.add(parent.register_gamepad_dpad_up_sensor(var))
        elif config.get(CONF_DPAD_LEFT):
            cg.add(parent.register_gamepad_dpad_left_sensor(var))
        elif config.get(CONF_DPAD_RIGHT):
            cg.add(parent.register_gamepad_dpad_right_sensor(var))
        elif config.get(CONF_DPAD_DOWN):
            cg.add(parent.register_gamepad_dpad_down_sensor(var))
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
