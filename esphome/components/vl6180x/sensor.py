# sensor.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import sensor, i2c
from esphome.const import (
  CONF_ID,
  CONF_TRIGGER_ID,
  CONF_GAIN,
  DEVICE_CLASS_DISTANCE,
  DEVICE_CLASS_ILLUMINANCE,
  ICON_RULER,
  ICON_BRIGHTNESS_5,
  UNIT_LUX,
)
from esphome.automation import Automation

DEPENDENCIES = ['i2c']

CODEOWNERS = ["@NonaSuomy"]

vl6180x_ns = cg.esphome_ns.namespace('vl6180x')

VL6180XSensor = vl6180x_ns.class_(
  'VL6180XSensor', sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

CONF_DISTANCE = "distance"
CONF_ALS = "als"
CONF_GAIN = "gain"
CONF_ON_SWIPE_GESTURE = "on_swipe_gesture"
CONF_ON_DOUBLE_TAP_GESTURE = "on_double_tap_gesture"
CONF_ON_HOVER_GESTURE = "on_hover_gesture"
CONF_UNDER_GLASS = "under_glass"
CONF_LUX_WITHOUT_GLASS = "lux_without_glass"

def validate(config):
    if CONF_DISTANCE not in config and CONF_ALS not in config:
        raise cv.Invalid("You must select at least one of distance or als.")
    return config
    
CONFIG_SCHEMA = cv.All(
  cv.Schema(
    {
      cv.GenerateID(): cv.declare_id(VL6180XSensor),
      cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
        unit_of_measurement="mm",
        icon=ICON_RULER,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DISTANCE,
      ),
      cv.Optional(CONF_ALS): cv.Schema({
        cv.Required('name'): cv.string,
        cv.Optional('gain', default='5X'): cv.enum({
          '1x': 0x06,
          '1.25x': 0x05,
          '1.67x': 0x04,
          '2.5x': 0x03,
          '5x': 0x02,
          '10x': 0x01,
          '20x': 0x00,
          '40x': 0x07,
        }, lower=True),
        cv.Optional(CONF_UNDER_GLASS): cv.boolean,
        cv.Optional(CONF_LUX_WITHOUT_GLASS): cv.float_, 
      }
      )
      .extend(
        sensor.sensor_schema(
          unit_of_measurement=UNIT_LUX,
          icon=ICON_BRIGHTNESS_5,
          accuracy_decimals=1,
          device_class=DEVICE_CLASS_ILLUMINANCE,
        )
      ),
      cv.Optional(CONF_ON_SWIPE_GESTURE): automation.validate_automation({
          cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(Automation),
      }),
      cv.Optional(CONF_ON_DOUBLE_TAP_GESTURE): automation.validate_automation({
          cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(Automation),
      }),
      cv.Optional(CONF_ON_HOVER_GESTURE): automation.validate_automation({
          cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(Automation),
      }),
    }
  )
  .extend(cv.polling_component_schema('60s'))
  .extend(i2c.i2c_device_schema(0x29)),
  cv.has_at_least_one_key(CONF_DISTANCE, CONF_ALS),
)

async def to_code(config):
  var = cg.new_Pvariable(config[CONF_ID])
  await cg.register_component(var, config)
  await i2c.register_i2c_device(var, config)
  if CONF_DISTANCE in config:
    distance = await sensor.new_sensor(config[CONF_DISTANCE])
    cg.add(var.set_distance_sensor(distance))
  if CONF_ALS in config:
    als = await sensor.new_sensor(config[CONF_ALS])
    cg.add(var.set_als_sensor(als))
    cg.add(var.set_gain(config[CONF_ALS][CONF_GAIN]))
    if CONF_UNDER_GLASS in config[CONF_ALS]:  
      cg.add(var.set_is_behind_glass(config[CONF_ALS][CONF_UNDER_GLASS]))
    if CONF_LUX_WITHOUT_GLASS in config[CONF_ALS]:  # Add this line
      cg.add(var.set_lux_without_glass(config[CONF_ALS][CONF_LUX_WITHOUT_GLASS]))
  # Register the gesture triggers
  #if CONF_ON_SWIPE_GESTURE in config:
  #  for conf in config[CONF_ON_SWIPE_GESTURE]:
  #      trigger = var.get_swipe_gesture_trigger()
  #      await automation.build_automation(trigger, [], conf)
  #if CONF_ON_SWIPE_GESTURE in config:
  #  for conf in config[CONF_ON_SWIPE_GESTURE]:
  #    trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
  #    await automation.build_automation(trigger, [(int, 'x')], conf)
  #if CONF_ON_DOUBLE_TAP_GESTURE in config:
  #  for conf in config[CONF_ON_DOUBLE_TAP_GESTURE]:
  #    trigger = var.get_double_tap_gesture_trigger()
  #    await automation.build_automation(trigger, [], conf)
  #if CONF_ON_HOVER_GESTURE in config:
  #  for conf in config[CONF_ON_HOVER_GESTURE]:
  #    trigger = var.get_hover_gesture_trigger()
  #    await automation.build_automation(trigger, [], conf)
