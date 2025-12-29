import esphome.codegen as cg
from esphome.components import binary_sensor
from esphome.components.usb_hidx import USBHIDXComponent
import esphome.config_validation as cv

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_DEVICE_ID = "device_id"
CONF_TYPE = "type"
CONF_BUTTON_A = "button_a"
CONF_BUTTON_B = "button_b"
CONF_BUTTON_CROSS = "button_cross"
CONF_BUTTON_CIRCLE = "button_circle"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_TYPE, default="generic"): cv.string,
        cv.Optional(CONF_BUTTON_A): cv.boolean,
        cv.Optional(CONF_BUTTON_B): cv.boolean,
        cv.Optional(CONF_BUTTON_CROSS): cv.boolean,
        cv.Optional(CONF_BUTTON_CIRCLE): cv.boolean,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_USB_HIDX_ID])
    var = await binary_sensor.new_binary_sensor(config)

    if config.get(CONF_BUTTON_A):
        cg.add(parent.register_gamepad_button_a_sensor(var))
    if config.get(CONF_BUTTON_B):
        cg.add(parent.register_gamepad_button_b_sensor(var))
    if config.get(CONF_BUTTON_CROSS):
        driver = cg.RawExpression(
            f"id({config[CONF_USB_HIDX_ID]}).get_playstation_driver()"
        )
        cg.add(driver.set_button_cross_sensor(var))
    if config.get(CONF_BUTTON_CIRCLE):
        driver = cg.RawExpression(
            f"id({config[CONF_USB_HIDX_ID]}).get_playstation_driver()"
        )
        cg.add(driver.set_button_circle_sensor(var))
