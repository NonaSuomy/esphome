import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_BRIEFCASE_DOWNLOAD,
    ICON_SCREEN_ROTATION,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREE_PER_SECOND,
    UNIT_METER_PER_SECOND_SQUARED,
)

from .const import (
    CONF_ACCEL_ODR,
    CONF_ACCEL_RANGE,
    CONF_ACCEL_X,
    CONF_ACCEL_Y,
    CONF_ACCEL_Z,
    CONF_GYRO_ODR,
    CONF_GYRO_RANGE,
    CONF_GYRO_X,
    CONF_GYRO_Y,
    CONF_GYRO_Z,
)

CODEOWNERS = ["@NonaSuomy"]

# Define the C++ namespace
lsm6ds_ns = cg.esphome_ns.namespace("lsm6ds_base")
LSM6DSComponent = lsm6ds_ns.class_("LSM6DSComponent", cg.PollingComponent)

LSM6DSAccelRange = lsm6ds_ns.enum("LSM6DSAccelRange")
ACCEL_RANGE_OPTIONS = {
    "2G": LSM6DSAccelRange.LSM6DS_ACCEL_RANGE_2G,
    "4G": LSM6DSAccelRange.LSM6DS_ACCEL_RANGE_4G,
    "8G": LSM6DSAccelRange.LSM6DS_ACCEL_RANGE_8G,
    "16G": LSM6DSAccelRange.LSM6DS_ACCEL_RANGE_16G,
}

LSM6DSGyroRange = lsm6ds_ns.enum("LSM6DSGyroRange")
GYRO_RANGE_OPTIONS = {
    "125DPS": LSM6DSGyroRange.LSM6DS_GYRO_RANGE_125DPS,
    "250DPS": LSM6DSGyroRange.LSM6DS_GYRO_RANGE_250DPS,
    "500DPS": LSM6DSGyroRange.LSM6DS_GYRO_RANGE_500DPS,
    "1000DPS": LSM6DSGyroRange.LSM6DS_GYRO_RANGE_1000DPS,
    "2000DPS": LSM6DSGyroRange.LSM6DS_GYRO_RANGE_2000DPS,
}

LSM6DSODR = lsm6ds_ns.enum("LSM6DSODR")
ODR_OPTIONS = {
    "12.5HZ": LSM6DSODR.LSM6DS_ODR_12_5HZ,
    "26HZ": LSM6DSODR.LSM6DS_ODR_26HZ,
    "52HZ": LSM6DSODR.LSM6DS_ODR_52HZ,
    "104HZ": LSM6DSODR.LSM6DS_ODR_104HZ,
    "208HZ": LSM6DSODR.LSM6DS_ODR_208HZ,
    "416HZ": LSM6DSODR.LSM6DS_ODR_416HZ,
    "833HZ": LSM6DSODR.LSM6DS_ODR_833HZ,
    "1660HZ": LSM6DSODR.LSM6DS_ODR_1660HZ,
    "3330HZ": LSM6DSODR.LSM6DS_ODR_3330HZ,
    "6660HZ": LSM6DSODR.LSM6DS_ODR_6660HZ,
}

accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
gyro_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREE_PER_SECOND,
    icon=ICON_SCREEN_ROTATION,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
temperature_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

LSM6DS_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(LSM6DSComponent),
        cv.Optional(CONF_ACCEL_X): accel_schema,
        cv.Optional(CONF_ACCEL_Y): accel_schema,
        cv.Optional(CONF_ACCEL_Z): accel_schema,
        cv.Optional(CONF_GYRO_X): gyro_schema,
        cv.Optional(CONF_GYRO_Y): gyro_schema,
        cv.Optional(CONF_GYRO_Z): gyro_schema,
        cv.Optional(CONF_TEMPERATURE): temperature_schema,
        cv.Optional(CONF_ACCEL_RANGE, default="2G"): cv.enum(
            ACCEL_RANGE_OPTIONS, upper=True
        ),
        cv.Optional(CONF_GYRO_RANGE, default="250DPS"): cv.enum(
            GYRO_RANGE_OPTIONS, upper=True
        ),
        cv.Optional(CONF_ACCEL_ODR, default="104HZ"): cv.enum(ODR_OPTIONS, upper=True),
        cv.Optional(CONF_GYRO_ODR, default="104HZ"): cv.enum(ODR_OPTIONS, upper=True),
    }
).extend(cv.polling_component_schema("60s"))


async def register_lsm6ds(var, config):
    await cg.register_component(var, config)

    # Set configuration options
    cg.add(var.set_accel_range(config[CONF_ACCEL_RANGE]))
    cg.add(var.set_gyro_range(config[CONF_GYRO_RANGE]))
    cg.add(var.set_accel_odr(config[CONF_ACCEL_ODR]))
    cg.add(var.set_gyro_odr(config[CONF_GYRO_ODR]))

    # Set up sensor outputs
    for d in ["x", "y", "z"]:
        accel_key = f"accel_{d}"
        if accel_key in config:
            sens = await sensor.new_sensor(config[accel_key])
            cg.add(getattr(var, f"set_accel_{d}_sensor")(sens))
        gyro_key = f"gyro_{d}"
        if gyro_key in config:
            sens = await sensor.new_sensor(config[gyro_key])
            cg.add(getattr(var, f"set_gyro_{d}_sensor")(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))
