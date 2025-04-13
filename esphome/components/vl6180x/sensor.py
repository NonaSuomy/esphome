import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_ILLUMINANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLIMETER,
    UNIT_LUX,
)

DEPENDENCIES = ['i2c']

# Define our own constants
CONF_ALS = "als"
CONF_DISTANCE = "distance"
CONF_SCALER = "scaler"

vl6180x_ns = cg.esphome_ns.namespace('vl6180x')
VL6180XSensor = vl6180x_ns.class_('VL6180XSensor', cg.Component, i2c.I2CDevice)

# Define distance sensor schema with scaler
DISTANCE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_MILLIMETER,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_DISTANCE,
    state_class=STATE_CLASS_MEASUREMENT,
).extend({
    cv.Optional(CONF_SCALER, default=1): cv.one_of(1, 2, 3, int=True),
})

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(VL6180XSensor),
        cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
        cv.Optional(CONF_DISTANCE): DISTANCE_SCHEMA,
        cv.Optional(CONF_ALS): sensor.sensor_schema(
            unit_of_measurement=UNIT_LUX,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_ILLUMINANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    })
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x29))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_DISTANCE in config:
        distance_config = config[CONF_DISTANCE]
        sens = await sensor.new_sensor(distance_config)
        cg.add(var.set_distance_sensor(sens))
        cg.add(var.set_scaling(distance_config[CONF_SCALER]))

    if CONF_ALS in config:
        sens = await sensor.new_sensor(config[CONF_ALS])
        cg.add(var.set_als_sensor(sens))
