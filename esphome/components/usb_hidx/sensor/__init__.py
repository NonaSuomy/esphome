import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv

from .. import DRIVER_ALIASES, USBHIDXComponent, enable_driver

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_TYPE = "type"
CONF_DRIVER = "driver"
CONF_VID = "vid"
CONF_PID = "pid"
CONF_OFFSET = "offset"
CONF_LENGTH = "length"
CONF_SIGNED = "signed"
CONF_SCALE = "scale"
CONF_BIAS = "bias"
CONF_X_DELTA = "x_delta"
CONF_Y_DELTA = "y_delta"
CONF_WHEEL = "wheel"
CONF_AXIS = "axis"

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Required(CONF_TYPE): cv.one_of("mouse", "gamepad", "raw", lower=True),
        cv.Optional(CONF_DRIVER): cv.one_of(*DRIVER_ALIASES.keys(), lower=True),
        cv.Optional(CONF_VID): cv.hex_uint16_t,
        cv.Optional(CONF_PID): cv.hex_uint16_t,
        cv.Optional(CONF_OFFSET, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_LENGTH, default=1): cv.int_range(min=1, max=4),
        cv.Optional(CONF_SIGNED, default=False): cv.boolean,
        cv.Optional(CONF_SCALE, default=1.0): cv.float_,
        cv.Optional(CONF_BIAS, default=0.0): cv.float_,
        cv.Optional(CONF_X_DELTA): cv.boolean,
        cv.Optional(CONF_Y_DELTA): cv.boolean,
        cv.Optional(CONF_WHEEL): cv.boolean,
        cv.Optional(CONF_AXIS): cv.one_of("lx", "ly", "rx", "ry", lower=True),
    }
)

AXIS_REGISTERS = {
    "lx": "register_gamepad_left_stick_x_sensor",
    "ly": "register_gamepad_left_stick_y_sensor",
    "rx": "register_gamepad_right_stick_x_sensor",
    "ry": "register_gamepad_right_stick_y_sensor",
}


def validate_config(config):
    device_type = config[CONF_TYPE]
    if device_type == "raw":
        if CONF_DRIVER in config:
            raise cv.Invalid("raw USB HIDX sensors cannot select a protocol driver")
        if any(
            config.get(key)
            for key in (CONF_X_DELTA, CONF_Y_DELTA, CONF_WHEEL, CONF_AXIS)
        ):
            raise cv.Invalid(
                "raw USB HIDX sensors cannot use protocol-specific mappings"
            )
    elif CONF_VID in config or CONF_PID in config:
        raise cv.Invalid("vid and pid selectors are supported only with type: raw")
    elif device_type == "mouse":
        mouse_mappings = sum(
            bool(config.get(key)) for key in (CONF_X_DELTA, CONF_Y_DELTA, CONF_WHEEL)
        )
        if mouse_mappings != 1:
            raise cv.Invalid("mouse USB HIDX sensors require exactly one axis mapping")
    elif device_type == "gamepad" and CONF_AXIS not in config:
        raise cv.Invalid("gamepad USB HIDX sensors require axis")
    return config


CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, validate_config)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await sensor.new_sensor(config)

    if config[CONF_TYPE] == "raw":
        cg.add(
            parent.register_raw_sensor(
                var,
                config[CONF_OFFSET],
                config[CONF_LENGTH],
                config[CONF_SIGNED],
                config[CONF_SCALE],
                config[CONF_BIAS],
                config.get(CONF_VID, 0),
                config.get(CONF_PID, 0),
            )
        )
    elif config[CONF_TYPE] == "mouse":
        enable_driver(config.get(CONF_DRIVER, "mouse"))
        if config.get(CONF_X_DELTA):
            cg.add(parent.register_mouse_x_sensor(var))
        elif config.get(CONF_Y_DELTA):
            cg.add(parent.register_mouse_y_sensor(var))
        elif config.get(CONF_WHEEL):
            cg.add(parent.register_mouse_wheel_sensor(var))
    elif config[CONF_TYPE] == "gamepad":
        # Keep platform-only builds selective.  A top-level gamepad block can
        # select the driver for these standard entities; otherwise the YAML
        # entry should provide ``driver:`` explicitly.
        if CONF_DRIVER in config:
            enable_driver(config[CONF_DRIVER])
        axis = config.get(CONF_AXIS)
        register_fn = AXIS_REGISTERS.get(axis)
        if register_fn:
            cg.add(getattr(parent, register_fn)(var))
