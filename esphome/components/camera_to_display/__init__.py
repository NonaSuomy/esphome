import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import display, esp32_camera
from esphome.const import CONF_ID, CONF_DISPLAY, CONF_UPDATE_INTERVAL

DEPENDENCIES = ['esp32']
AUTO_LOAD = ['display', 'esp32_camera']

camera_to_display_ns = cg.esphome_ns.namespace('camera_to_display')
CameraToDisplayComponent = camera_to_display_ns.class_('CameraToDisplayComponent', cg.Component)

CONF_CAMERA = 'camera'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CameraToDisplayComponent),
    cv.Required(CONF_DISPLAY): cv.use_id(display.DisplayBuffer),
    cv.Required(CONF_CAMERA): cv.use_id(esp32_camera.ESP32Camera),
    cv.Optional(CONF_UPDATE_INTERVAL, default='100ms'): cv.update_interval,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    disp = await cg.get_variable(config[CONF_DISPLAY])
    cg.add(var.set_display(disp))

    cam = await cg.get_variable(config[CONF_CAMERA])
    cg.add(var.set_camera(cam))

    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
