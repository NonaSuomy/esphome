import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv

from .. import USBHIDXComponent

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_TYPE = "type"

CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Required(CONF_TYPE): cv.one_of("keyboard", lower=True),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await text_sensor.new_text_sensor(config)

    if config[CONF_TYPE] == "keyboard":
        cg.add(parent.register_keyboard_sensor(var))
