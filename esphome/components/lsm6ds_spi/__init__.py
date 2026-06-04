import esphome.codegen as cg
from esphome.components import spi
import esphome.config_validation as cv
from esphome.const import CONF_ID

from ..lsm6ds_base import LSM6DS_SCHEMA, LSM6DSComponent, register_lsm6ds

AUTO_LOAD = ["lsm6ds_base"]
CODEOWNERS = ["@NonaSuomy"]
DEPENDENCIES = ["spi"]

lsm6ds_spi_ns = cg.esphome_ns.namespace("lsm6ds_spi")
LSM6DSSPIDevice = lsm6ds_spi_ns.class_(
    "LSM6DSSPIDevice", LSM6DSComponent, spi.SPIDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LSM6DSSPIDevice),
        }
    )
    .extend(LSM6DS_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await register_lsm6ds(var, config)
    await spi.register_spi_device(var, config)
