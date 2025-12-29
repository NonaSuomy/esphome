import esphome.codegen as cg
from esphome.components import text_sensor
from esphome.components.usb_hidx import USBHIDXComponent
import esphome.config_validation as cv

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_DEVICE_ID = "device_id"

CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Optional(CONF_DEVICE_ID): cv.string,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await text_sensor.new_text_sensor(config)
    cg.add(parent.register_keyboard_sensor(var))
