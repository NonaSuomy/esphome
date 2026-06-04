import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_TEMPERATURE,
    ICON_BRIEFCASE_DOWNLOAD,
    ICON_SCREEN_ROTATION,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREE_PER_SECOND,
    UNIT_METER_PER_SECOND_SQUARED,
)

from ..lsm6ds_base import LSM6DSComponent
from ..lsm6ds_base.const import (
    CONF_ACCEL_X,
    CONF_ACCEL_Y,
    CONF_ACCEL_Z,
    CONF_GYRO_X,
    CONF_GYRO_Y,
    CONF_GYRO_Z,
    CONF_LSM6DS_ID,
)

DEPENDENCIES = ["lsm6ds_spi"]
CODEOWNERS = ["@NonaSuomy"]

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
    icon=ICON_THERMOMETER,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LSM6DS_ID): cv.use_id(LSM6DSComponent),
        cv.Optional(CONF_ACCEL_X): accel_schema,
        cv.Optional(CONF_ACCEL_Y): accel_schema,
        cv.Optional(CONF_ACCEL_Z): accel_schema,
        cv.Optional(CONF_GYRO_X): gyro_schema,
        cv.Optional(CONF_GYRO_Y): gyro_schema,
        cv.Optional(CONF_GYRO_Z): gyro_schema,
        cv.Optional(CONF_TEMPERATURE): temperature_schema,
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_LSM6DS_ID])

    for d in ["x", "y", "z"]:
        accel_key = f"accel_{d}"
        if accel_key in config:
            sens = await sensor.new_sensor(config[accel_key])
            cg.add(getattr(hub, f"set_accel_{d}_sensor")(sens))
        gyro_key = f"gyro_{d}"
        if gyro_key in config:
            sens = await sensor.new_sensor(config[gyro_key])
            cg.add(getattr(hub, f"set_gyro_{d}_sensor")(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(hub.set_temperature_sensor(sens))
