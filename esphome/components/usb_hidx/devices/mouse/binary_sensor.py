import esphome.codegen as cg
from esphome.components import binary_sensor
from esphome.components.usb_hidx import USBHIDXComponent
import esphome.config_validation as cv

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_DEVICE_ID = "device_id"
CONF_LEFT_BUTTON = "left_button"
CONF_RIGHT_BUTTON = "right_button"
CONF_MIDDLE_BUTTON = "middle_button"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_LEFT_BUTTON): cv.boolean,
        cv.Optional(CONF_RIGHT_BUTTON): cv.boolean,
        cv.Optional(CONF_MIDDLE_BUTTON): cv.boolean,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await binary_sensor.new_binary_sensor(config)

    if config.get(CONF_LEFT_BUTTON):
        cg.add(parent.register_mouse_left_sensor(var))
    elif config.get(CONF_RIGHT_BUTTON):
        cg.add(parent.register_mouse_right_sensor(var))
    elif config.get(CONF_MIDDLE_BUTTON):
        cg.add(parent.register_mouse_middle_sensor(var))
