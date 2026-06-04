import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID

from ..lsm6ds_base import LSM6DS_SCHEMA, LSM6DSComponent, register_lsm6ds

AUTO_LOAD = ["lsm6ds_base"]
CODEOWNERS = ["@NonaSuomy"]
DEPENDENCIES = ["i2c"]

lsm6ds_i2c_ns = cg.esphome_ns.namespace("lsm6ds_i2c")
LSM6DSI2CDevice = lsm6ds_i2c_ns.class_(
    "LSM6DSI2CDevice", LSM6DSComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = LSM6DS_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(LSM6DSI2CDevice),
    }
).extend(i2c.i2c_device_schema(0x6A))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await register_lsm6ds(var, config)
    await i2c.register_i2c_device(var, config)
