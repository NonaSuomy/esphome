import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv

from .. import USBHIDXComponent

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_TYPE = "type"
CONF_X_DELTA = "x_delta"
CONF_Y_DELTA = "y_delta"
CONF_WHEEL = "wheel"
CONF_AXIS = "axis"

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Required(CONF_TYPE): cv.one_of("mouse", "gamepad", lower=True),
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


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await sensor.new_sensor(config)

    if config[CONF_TYPE] == "mouse":
        if config.get(CONF_X_DELTA):
            cg.add(parent.register_mouse_x_sensor(var))
        elif config.get(CONF_Y_DELTA):
            cg.add(parent.register_mouse_y_sensor(var))
        elif config.get(CONF_WHEEL):
            cg.add(parent.register_mouse_wheel_sensor(var))
    elif config[CONF_TYPE] == "gamepad":
        axis = config.get(CONF_AXIS)
        register_fn = AXIS_REGISTERS.get(axis)
        if register_fn:
            cg.add(getattr(parent, register_fn)(var))
