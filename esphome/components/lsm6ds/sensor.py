import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    UNIT_METER_PER_SECOND_SQUARED,
    UNIT_DEGREE_PER_SECOND,
    UNIT_CELSIUS,
    ICON_ACCELERATION,
    ICON_THERMOMETER,
    ICON_SCREEN_ROTATION,
    CONF_ID,
)
from .const import (
    CONF_ACCEL_X, CONF_ACCEL_Y, CONF_ACCEL_Z,
    CONF_GYRO_X, CONF_GYRO_Y, CONF_GYRO_Z,
    LSM6DSComponent,
)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ID): cv.use_id(LSM6DSComponent),
    cv.Optional(CONF_ACCEL_X): sensor.sensor_schema(
        unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
        icon=ICON_ACCELERATION,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_ACCEL_Y): sensor.sensor_schema(
        unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
        icon=ICON_ACCELERATION,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_ACCEL_Z): sensor.sensor_schema(
        unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
        icon=ICON_ACCELERATION,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_GYRO_X): sensor.sensor_schema(
        unit_of_measurement=UNIT_DEGREE_PER_SECOND,
        icon=ICON_SCREEN_ROTATION,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_GYRO_Y): sensor.sensor_schema(
        unit_of_measurement=UNIT_DEGREE_PER_SECOND,
        icon=ICON_SCREEN_ROTATION,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_GYRO_Z): sensor.sensor_schema(
        unit_of_measurement=UNIT_DEGREE_PER_SECOND,
        icon=ICON_SCREEN_ROTATION,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional("temperature"): sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        icon=ICON_THERMOMETER,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_ID])
    
    for key in [CONF_ACCEL_X, CONF_ACCEL_Y, CONF_ACCEL_Z, 
                CONF_GYRO_X, CONF_GYRO_Y, CONF_GYRO_Z, 
                "temperature"]:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
