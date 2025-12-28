import esphome.codegen as cg
from esphome.components import sensor
from esphome.components.usb_hidx import USBHIDXComponent
import esphome.config_validation as cv

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_DEVICE_ID = "device_id"
CONF_X_DELTA = "x_delta"
CONF_Y_DELTA = "y_delta"
CONF_WHEEL = "wheel"

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_X_DELTA): cv.boolean,
        cv.Optional(CONF_Y_DELTA): cv.boolean,
        cv.Optional(CONF_WHEEL): cv.boolean,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await sensor.new_sensor(config)

    if config.get(CONF_X_DELTA):
        cg.add(parent.register_mouse_x_sensor(var))
    elif config.get(CONF_Y_DELTA):
        cg.add(parent.register_mouse_y_sensor(var))
    elif config.get(CONF_WHEEL):
        cg.add(parent.register_mouse_wheel_sensor(var))
