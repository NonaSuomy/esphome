import esphome.codegen as cg
from esphome.components import binary_sensor
from esphome.components.usb_hidx import USBHIDXComponent, usb_hidx_ns
import esphome.config_validation as cv

CONF_USB_HIDX_ID = "usb_hidx_id"
CONF_TYPE = "type"
CONF_BUTTON_CROSS = "button_cross"
CONF_BUTTON_CIRCLE = "button_circle"

PlayStationDriver = usb_hidx_ns.class_("PlayStationDriver")

CONFIG_SCHEMA = cv.All(
    binary_sensor.binary_sensor_schema().extend(
        {
            cv.GenerateID(CONF_USB_HIDX_ID): cv.use_id(USBHIDXComponent),
            cv.Required(CONF_TYPE): cv.string_strict("gamepad"),
            cv.Optional(CONF_BUTTON_CROSS): cv.boolean,
            cv.Optional(CONF_BUTTON_CIRCLE): cv.boolean,
        }
    ),
    cv.has_at_least_one_key(CONF_BUTTON_CROSS, CONF_BUTTON_CIRCLE),
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)

    driver = cg.RawExpression(
        f"id({config[CONF_USB_HIDX_ID]}).get_playstation_driver()"
    )

    if config.get(CONF_BUTTON_CROSS):
        cg.add(driver.set_button_cross_sensor(var))
    if config.get(CONF_BUTTON_CIRCLE):
        cg.add(driver.set_button_circle_sensor(var))
