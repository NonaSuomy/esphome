import esphome.codegen as cg
from esphome.components import binary_sensor
from esphome.components.usb_hidx import USBHIDXComponent
import esphome.config_validation as cv

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_DEVICE_ID = "device_id"
CONF_TYPE = "type"
CONF_BUTTON_A = "button_a"
CONF_BUTTON_B = "button_b"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_TYPE, default="generic"): cv.string,
        cv.Optional(CONF_BUTTON_A): cv.boolean,
        cv.Optional(CONF_BUTTON_B): cv.boolean,
    }
)


async def to_code(config):
    _ = await cg.get_variable(config[CONF_USB_HIDX_ID])
    _ = await binary_sensor.new_binary_sensor(config)
    # Gamepad button registration would go here
