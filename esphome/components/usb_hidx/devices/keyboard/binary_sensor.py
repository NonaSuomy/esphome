import esphome.codegen as cg
from esphome.components import binary_sensor
from esphome.components.usb_hidx import USBHIDXComponent
import esphome.config_validation as cv

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_DEVICE_ID = "device_id"
CONF_KEY = "key"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Required(CONF_KEY): cv.hex_uint8_t,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await binary_sensor.new_binary_sensor(config)
    cg.add(parent.register_keyboard_key_sensor(var, config[CONF_KEY]))
