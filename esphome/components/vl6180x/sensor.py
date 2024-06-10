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
  UNIT_MILLIMETER,
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
CONF_UNDER_GLASS = "under_glass"
CONF_LUX_WITHOUT_GLASS = "lux_without_glass"
# CONF_ON_GESTURE_SWIPE_TOWARD = "on_gesture_swipe_toward"
# CONF_ON_GESTURE_SWIPE_AWAY = "on_gesture_swipe_away"
# CONF_ON_GESTURE_SINGLE_TAP = "on_gesture_single_tap"
# CONF_ON_GESTURE_DOUBLE_TAP = "on_gesture_double_tap"
# CONF_ON_GESTURE_HOVER = "on_gesture_hover"
# CONF_GESTURE_SINGLE_TAP_INTERVAL_MS = "gesture_single_tap_interval_ms"
# CONF_GESTURE_DOUBLE_TAP_INTERVAL_MS = "gesture_double_tap_interval_ms"
# CONF_GESTURE_DOUBLE_TAP_DELAY_MS = "gesture_double_tap_delay_ms"
# CONF_GESTURE_HOVER_TIME_MS = "gesture_hover_time_ms"
# CONF_GESTURE_HAND_PRESENCE_THRESHOLD = "gesture_hand_presence_threshold"
# CONF_GESTURE_DOUBLE_TAP_THRESHOLD = "gesture_double_tap_threshold"

#def validate(config):
#    if CONF_DISTANCE not in config and CONF_ALS not in config:
#        raise cv.Invalid("You must select at least one of distance or als.")
#    return config

CONFIG_SCHEMA = cv.All(
  cv.Schema(
    {
      cv.GenerateID(): cv.declare_id(VL6180XSensor),
      cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
        unit_of_measurement=UNIT_MILLIMETER,
        icon=ICON_RULER,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_DISTANCE,
      ),
      cv.Optional(CONF_ALS): sensor.sensor_schema(
          unit_of_measurement=UNIT_LUX,
          icon=ICON_BRIGHTNESS_5,
          accuracy_decimals=1,
          device_class=DEVICE_CLASS_ILLUMINANCE,
      ).extend(
        {
          cv.Optional('gain', default='1X'): cv.enum({
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
      ),
      # cv.Optional(CONF_ON_GESTURE_SWIPE_TOWARD): automation.validate_automation(single=True),
      # cv.Optional(CONF_ON_GESTURE_SWIPE_AWAY): automation.validate_automation(single=True),
      # cv.Optional(CONF_ON_GESTURE_SINGLE_TAP): automation.validate_automation(single=True),
      # cv.Optional(CONF_ON_GESTURE_DOUBLE_TAP): automation.validate_automation(single=True),
      # cv.Optional(CONF_ON_GESTURE_HOVER): automation.validate_automation(single=True),
      # cv.Optional(CONF_GESTURE_SINGLE_TAP_INTERVAL_MS, default=500): cv.uint32_t,
      # cv.Optional(CONF_GESTURE_DOUBLE_TAP_INTERVAL_MS, default=500): cv.uint32_t,
      # cv.Optional(CONF_GESTURE_DOUBLE_TAP_DELAY_MS, default=1000): cv.uint32_t,
      # cv.Optional(CONF_GESTURE_HOVER_TIME_MS, default=3000): cv.uint32_t,
      # cv.Optional(CONF_GESTURE_HAND_PRESENCE_THRESHOLD, default=40): cv.uint8_t,
      # cv.Optional(CONF_GESTURE_DOUBLE_TAP_THRESHOLD, default=40): cv.uint8_t,
    }
  )
  .extend(cv.polling_component_schema('60s'))
  .extend(i2c.i2c_device_schema(0x29)),
)

async def to_code(config):
  var = cg.new_Pvariable(config[CONF_ID])
  await cg.register_component(var, config)
  await i2c.register_i2c_device(var, config)
  if CONF_DISTANCE in config:
    distance = await sensor.new_sensor(config[CONF_DISTANCE])
    cg.add(var.set_distance_sensor(distance))
  # Register the gesture triggers
  # if CONF_ON_GESTURE_SWIPE_TOWARD in config:
    # await automation.build_automation(
      # var.get_gesture_swipe_toward_trigger(), [], config[CONF_ON_GESTURE_SWIPE_TOWARD]
    # )
  # if CONF_ON_GESTURE_SWIPE_AWAY in config:
    # await automation.build_automation(
      # var.get_gesture_swipe_away_trigger(), [], config[CONF_ON_GESTURE_SWIPE_AWAY]
    # )
  # if CONF_ON_GESTURE_SINGLE_TAP in config:
    # await automation.build_automation(
      # var.get_gesture_single_tap_trigger(), [], config[CONF_ON_GESTURE_SINGLE_TAP]
    # )
  # if CONF_ON_GESTURE_DOUBLE_TAP in config:
    # await automation.build_automation(
      # var.get_gesture_double_tap_trigger(), [], config[CONF_ON_GESTURE_DOUBLE_TAP]
    # )
  # if CONF_ON_GESTURE_HOVER in config:
    # await automation.build_automation(
      # var.get_gesture_hover_trigger(), [], config[CONF_ON_GESTURE_HOVER]
    # )
  if CONF_ALS in config:
    als = await sensor.new_sensor(config[CONF_ALS])
    cg.add(var.set_als_sensor(als))
    cg.add(var.set_gain(config[CONF_ALS][CONF_GAIN]))
    if CONF_UNDER_GLASS in config[CONF_ALS]:
      cg.add(var.set_is_behind_glass(config[CONF_ALS][CONF_UNDER_GLASS]))
    if CONF_LUX_WITHOUT_GLASS in config[CONF_ALS]:
      cg.add(var.set_lux_without_glass(config[CONF_ALS][CONF_LUX_WITHOUT_GLASS]))
