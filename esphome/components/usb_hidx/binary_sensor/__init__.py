import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv

from .. import DRIVER_ALIASES, USBHIDXComponent, enable_driver

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_TYPE = "type"
CONF_DRIVER = "driver"
CONF_KEY = "key"
CONF_VID = "vid"
CONF_PID = "pid"
CONF_OFFSET = "offset"
CONF_MASK = "mask"
CONF_VALUE = "value"
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
        cv.Required(CONF_TYPE): cv.one_of(
            "keyboard", "mouse", "gamepad", "switch", "raw", lower=True
        ),
        cv.Optional(CONF_DRIVER): cv.one_of(*DRIVER_ALIASES.keys(), lower=True),
        cv.Optional(CONF_KEY): cv.hex_uint8_t,
        cv.Optional(CONF_VID): cv.hex_uint16_t,
        cv.Optional(CONF_PID): cv.hex_uint16_t,
        cv.Optional(CONF_OFFSET): cv.int_range(min=0, max=255),
        cv.Optional(CONF_MASK): cv.hex_uint8_t,
        cv.Optional(CONF_VALUE): cv.hex_uint8_t,
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


def validate_config(config):
    device_type = config[CONF_TYPE]
    if device_type == "raw":
        if CONF_DRIVER in config:
            raise cv.Invalid(
                "raw USB HIDX binary sensors cannot select a protocol driver"
            )
        if CONF_OFFSET not in config or CONF_MASK not in config:
            raise cv.Invalid("raw USB HIDX binary sensors require offset and mask")
        if config[CONF_MASK] == 0:
            raise cv.Invalid("raw USB HIDX binary sensor mask must not be zero")
        if any(
            config.get(key)
            for key in (
                CONF_KEY,
                CONF_LEFT_BUTTON,
                CONF_RIGHT_BUTTON,
                CONF_MIDDLE_BUTTON,
                CONF_BUTTON_A,
                CONF_BUTTON_B,
                CONF_BUTTON_X,
                CONF_BUTTON_Y,
                CONF_BUTTON_L,
                CONF_BUTTON_R,
                CONF_BUTTON_ZL,
                CONF_BUTTON_ZR,
                CONF_BUTTON_MINUS,
                CONF_BUTTON_PLUS,
                CONF_BUTTON_HOME,
                CONF_BUTTON_CAPTURE,
                CONF_BUTTON_L3,
                CONF_BUTTON_R3,
                CONF_BUTTON_CROSS,
                CONF_BUTTON_CIRCLE,
                CONF_DPAD_UP,
                CONF_DPAD_LEFT,
                CONF_DPAD_RIGHT,
                CONF_DPAD_DOWN,
            )
        ):
            raise cv.Invalid(
                "raw USB HIDX binary sensors cannot use protocol-specific mappings"
            )
        return config

    if CONF_VID in config or CONF_PID in config:
        raise cv.Invalid("vid and pid selectors are supported only with type: raw")

    if device_type == "keyboard":
        if CONF_KEY not in config:
            raise cv.Invalid("keyboard USB HIDX binary sensors require key")
    elif device_type == "mouse":
        mouse_mappings = sum(
            bool(config.get(key))
            for key in (CONF_LEFT_BUTTON, CONF_RIGHT_BUTTON, CONF_MIDDLE_BUTTON)
        )
        if mouse_mappings != 1:
            raise cv.Invalid(
                "mouse USB HIDX binary sensors require exactly one button mapping"
            )
    elif device_type in ("gamepad", "switch"):
        gamepad_mappings = sum(
            bool(config.get(key))
            for key in (
                CONF_BUTTON_A,
                CONF_BUTTON_B,
                CONF_BUTTON_X,
                CONF_BUTTON_Y,
                CONF_BUTTON_L,
                CONF_BUTTON_R,
                CONF_BUTTON_ZL,
                CONF_BUTTON_ZR,
                CONF_BUTTON_MINUS,
                CONF_BUTTON_PLUS,
                CONF_BUTTON_HOME,
                CONF_BUTTON_CAPTURE,
                CONF_BUTTON_L3,
                CONF_BUTTON_R3,
                CONF_BUTTON_CROSS,
                CONF_BUTTON_CIRCLE,
                CONF_DPAD_UP,
                CONF_DPAD_LEFT,
                CONF_DPAD_RIGHT,
                CONF_DPAD_DOWN,
            )
        )
        if gamepad_mappings != 1:
            raise cv.Invalid(
                "gamepad USB HIDX binary sensors require exactly one button mapping"
            )
    return config


CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, validate_config)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await binary_sensor.new_binary_sensor(config)

    device_type = config[CONF_TYPE]

    if device_type == "raw":
        cg.add(
            parent.register_raw_binary_sensor(
                var,
                config[CONF_OFFSET],
                config[CONF_MASK],
                config.get(CONF_VALUE, 0),
                CONF_VALUE in config,
                config.get(CONF_VID, 0),
                config.get(CONF_PID, 0),
            )
        )
        return

    if device_type == "keyboard":
        enable_driver(config.get(CONF_DRIVER, "keyboard"))
    elif device_type == "mouse":
        enable_driver(config.get(CONF_DRIVER, "mouse"))
    elif device_type == "switch":
        enable_driver(config.get(CONF_DRIVER, "switch"))
    elif device_type == "gamepad" and CONF_DRIVER in config:
        # A top-level ``usb_hidx.gamepad.type`` can select the concrete
        # driver for the standard entities.  Platform-only configurations
        # must name their driver explicitly; silently enabling the generic
        # gamepad here would pull an extra driver into every build.
        enable_driver(config[CONF_DRIVER])

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
    elif device_type in ("gamepad", "switch"):
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
