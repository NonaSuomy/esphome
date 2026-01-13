import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome import pins

CODEOWNERS = ["@esphome"]

ptttl_ns = cg.esphome_ns.namespace("ptttl")
PTTTLComponent = ptttl_ns.class_("PTTTLComponent", cg.Component)

CONF_BCLK_PIN = "bclk_pin"
CONF_LRCLK_PIN = "lrclk_pin"
CONF_DOUT_PIN = "dout_pin"
CONF_SAMPLE_RATE = "sample_rate"
CONF_VOLUME = "volume"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(PTTTLComponent),
    cv.Required(CONF_BCLK_PIN): pins.internal_gpio_output_pin_schema,
    cv.Required(CONF_LRCLK_PIN): pins.internal_gpio_output_pin_schema,
    cv.Required(CONF_DOUT_PIN): pins.internal_gpio_output_pin_schema,
    cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.int_range(min=8000, max=48000),
    cv.Optional(CONF_VOLUME, default=0.8): cv.float_range(min=0.0, max=1.0),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    bclk = await cg.gpio_pin_expression(config[CONF_BCLK_PIN])
    cg.add(var.set_bclk_pin(bclk))
    
    lrclk = await cg.gpio_pin_expression(config[CONF_LRCLK_PIN])
    cg.add(var.set_lrclk_pin(lrclk))
    
    dout = await cg.gpio_pin_expression(config[CONF_DOUT_PIN])
    cg.add(var.set_dout_pin(dout))
    
    if CONF_SAMPLE_RATE in config:
        cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    
    if CONF_VOLUME in config:
        cg.add(var.set_volume(config[CONF_VOLUME]))
