import esphome.codegen as cg
from esphome.components import binary_sensor
from esphome.components.usb_hidx import USBHIDXComponent
import esphome.config_validation as cv

CONF_TYPE = "type"

# Route to the appropriate device driver based on type
def get_device_module(device_type):
    if device_type == "switch":
        from esphome.components.usb_hidx.devices.switch import binary_sensor as device_module
    elif device_type == "generic" or device_type == "generic_gamepad":
        from esphome.components.usb_hidx.devices.generic_gamepad import binary_sensor as device_module
    else:
        from esphome.components.usb_hidx.devices.generic_gamepad import binary_sensor as device_module
    return device_module

async def to_code(config):
    device_type = config.get(CONF_TYPE, "generic")
    device_module = get_device_module(device_type)
    await device_module.to_code(config)

CONFIG_SCHEMA = cv.Schema({
    cv.Optional(CONF_TYPE, default="generic"): cv.string,
}).extend(cv.COMPONENT_SCHEMA)

def validate_config(config):
    device_type = config.get(CONF_TYPE, "generic")
    device_module = get_device_module(device_type)
    return device_module.CONFIG_SCHEMA(config)

CONFIG_SCHEMA = validate_config
