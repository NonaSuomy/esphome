import esphome.codegen as cg
from esphome.components import sensor
from esphome.components.usb_hidx import USBHIDXComponent
import esphome.config_validation as cv

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_LEFT_STICK_X = "left_stick_x"
CONF_LEFT_STICK_Y = "left_stick_y"
CONF_RIGHT_STICK_X = "right_stick_x"
CONF_RIGHT_STICK_Y = "right_stick_y"

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Optional(CONF_LEFT_STICK_X): cv.boolean,
        cv.Optional(CONF_LEFT_STICK_Y): cv.boolean,
        cv.Optional(CONF_RIGHT_STICK_X): cv.boolean,
        cv.Optional(CONF_RIGHT_STICK_Y): cv.boolean,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await sensor.new_sensor(config)

    if config.get(CONF_LEFT_STICK_X):
        cg.add(parent.register_gamepad_left_stick_x_sensor(var))
    elif config.get(CONF_LEFT_STICK_Y):
        cg.add(parent.register_gamepad_left_stick_y_sensor(var))
    elif config.get(CONF_RIGHT_STICK_X):
        cg.add(parent.register_gamepad_right_stick_x_sensor(var))
    elif config.get(CONF_RIGHT_STICK_Y):
        cg.add(parent.register_gamepad_right_stick_y_sensor(var))
