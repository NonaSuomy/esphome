import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor, pn532
from esphome.const import CONF_ID

DEPENDENCIES = ['pn532']

yubikey_otp_ns = cg.esphome_ns.namespace('yubikey_otp')
YubikeyOtpSensor = yubikey_otp_ns.class_('YubikeyOtpSensor', text_sensor.TextSensor, cg.PollingComponent)

CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend({
    cv.GenerateID(): cv.declare_id(YubikeyOtpSensor),
    cv.Required(pn532.CONF_PN532_ID): cv.use_id(pn532.PN532),
}).extend(cv.polling_component_schema('60s'))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await text_sensor.register_text_sensor(var, config)

    pn532 = await cg.get_variable(config[pn532.CONF_PN532_ID])
    cg.add(var.set_pn532(pn532))
