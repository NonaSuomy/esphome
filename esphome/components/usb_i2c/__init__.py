import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import CONF_FREQUENCY, CONF_ID, CONF_SCAN

DEPENDENCIES = ["usb_hidx"]

usb_i2c_ns = cg.esphome_ns.namespace("usb_i2c")
USBI2CBus = usb_i2c_ns.class_("USBI2CBus", i2c.I2CBus, cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(USBI2CBus),
        cv.Optional(CONF_SCAN, default=True): cv.boolean,
        cv.Optional(CONF_FREQUENCY, default="100kHz"): cv.frequency,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_scan(config[CONF_SCAN]))
    cg.add(var.set_frequency(int(config[CONF_FREQUENCY])))
