import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, spi
from esphome.const import CONF_ID

from .const import (
    lsm6ds_ns,
    LSM6DSComponent,
    CONF_ACCEL_RANGE,
    CONF_GYRO_RANGE,
    CONF_ACCEL_ODR,
    CONF_GYRO_ODR,
    LSM6DS_ACCEL_RANGE_2G,
    LSM6DS_ACCEL_RANGE_16G,
    LSM6DS_ACCEL_RANGE_4G,
    LSM6DS_ACCEL_RANGE_8G,
    LSM6DS_GYRO_RANGE_250DPS,
    LSM6DS_GYRO_RANGE_500DPS,
    LSM6DS_GYRO_RANGE_1000DPS,
    LSM6DS_GYRO_RANGE_2000DPS,
    LSM6DS_GYRO_RANGE_125DPS,
    LSM6DS_ACCEL_ODR_OFF,
    LSM6DS_ACCEL_ODR_12_5HZ,
    LSM6DS_ACCEL_ODR_26HZ,
    LSM6DS_ACCEL_ODR_52HZ,
    LSM6DS_ACCEL_ODR_104HZ,
    LSM6DS_ACCEL_ODR_208HZ,
    LSM6DS_ACCEL_ODR_416HZ,
    LSM6DS_ACCEL_ODR_833HZ,
    LSM6DS_ACCEL_ODR_1_66KHZ,
    LSM6DS_ACCEL_ODR_3_33KHZ,
    LSM6DS_ACCEL_ODR_6_66KHZ,
    LSM6DS_GYRO_ODR_OFF,
    LSM6DS_GYRO_ODR_12_5HZ,
    LSM6DS_GYRO_ODR_26HZ,
    LSM6DS_GYRO_ODR_52HZ,
    LSM6DS_GYRO_ODR_104HZ,
    LSM6DS_GYRO_ODR_208HZ,
    LSM6DS_GYRO_ODR_416HZ,
    LSM6DS_GYRO_ODR_833HZ,
    LSM6DS_GYRO_ODR_1_66KHZ,
    LSM6DS_GYRO_ODR_3_33KHZ,
    LSM6DS_GYRO_ODR_6_66KHZ,
)

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

LSM6DSI2CDevice = lsm6ds_ns.class_("LSM6DSI2CDevice", LSM6DSComponent, i2c.I2CDevice)

ACCEL_RANGES = {
    "2G": LSM6DS_ACCEL_RANGE_2G,
    "16G": LSM6DS_ACCEL_RANGE_16G,
    "4G": LSM6DS_ACCEL_RANGE_4G,
    "8G": LSM6DS_ACCEL_RANGE_8G,
}

GYRO_RANGES = {
    "250DPS": LSM6DS_GYRO_RANGE_250DPS,
    "500DPS": LSM6DS_GYRO_RANGE_500DPS,
    "1000DPS": LSM6DS_GYRO_RANGE_1000DPS,
    "2000DPS": LSM6DS_GYRO_RANGE_2000DPS,
    "125DPS": LSM6DS_GYRO_RANGE_125DPS,
}

ACCEL_ODRS = {
    "OFF": LSM6DS_ACCEL_ODR_OFF,
    "12.5HZ": LSM6DS_ACCEL_ODR_12_5HZ,
    "26HZ": LSM6DS_ACCEL_ODR_26HZ,
    "52HZ": LSM6DS_ACCEL_ODR_52HZ,
    "104HZ": LSM6DS_ACCEL_ODR_104HZ,
    "208HZ": LSM6DS_ACCEL_ODR_208HZ,
    "416HZ": LSM6DS_ACCEL_ODR_416HZ,
    "833HZ": LSM6DS_ACCEL_ODR_833HZ,
    "1.66KHZ": LSM6DS_ACCEL_ODR_1_66KHZ,
    "3.33KHZ": LSM6DS_ACCEL_ODR_3_33KHZ,
    "6.66KHZ": LSM6DS_ACCEL_ODR_6_66KHZ,
}

GYRO_ODRS = {
    "OFF": LSM6DS_GYRO_ODR_OFF,
    "12.5HZ": LSM6DS_GYRO_ODR_12_5HZ,
    "26HZ": LSM6DS_GYRO_ODR_26HZ,
    "52HZ": LSM6DS_GYRO_ODR_52HZ,
    "104HZ": LSM6DS_GYRO_ODR_104HZ,
    "208HZ": LSM6DS_GYRO_ODR_208HZ,
    "416HZ": LSM6DS_GYRO_ODR_416HZ,
    "833HZ": LSM6DS_GYRO_ODR_833HZ,
    "1.66KHZ": LSM6DS_GYRO_ODR_1_66KHZ,
    "3.33KHZ": LSM6DS_GYRO_ODR_3_33KHZ,
    "6.66KHZ": LSM6DS_GYRO_ODR_6_66KHZ,
}

BASE_SCHEMA = cv.Schema({
    cv.Optional(CONF_ACCEL_RANGE, default="2G"): cv.enum(ACCEL_RANGES, upper=True),
    cv.Optional(CONF_GYRO_RANGE, default="250DPS"): cv.enum(GYRO_RANGES, upper=True),
    cv.Optional(CONF_ACCEL_ODR, default="104HZ"): cv.enum(ACCEL_ODRS, upper=True),
    cv.Optional(CONF_GYRO_ODR, default="104HZ"): cv.enum(GYRO_ODRS, upper=True),
}).extend(cv.polling_component_schema("60s"))

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LSM6DSI2CDevice),
}).extend(BASE_SCHEMA).extend(i2c.i2c_device_schema(None))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    accel_range = ACCEL_RANGES[config[CONF_ACCEL_RANGE]]
    gyro_range = GYRO_RANGES[config[CONF_GYRO_RANGE]]
    accel_odr = ACCEL_ODRS[config[CONF_ACCEL_ODR]]
    gyro_odr = GYRO_ODRS[config[CONF_GYRO_ODR]]
        
    cg.add(var.set_accel_range(accel_range))
    cg.add(var.set_gyro_range(gyro_range))
    cg.add(var.set_accel_odr(accel_odr))
    cg.add(var.set_gyro_odr(gyro_odr))
