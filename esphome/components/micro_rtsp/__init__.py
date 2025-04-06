# components/micro_rtsp/__init__.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32_camera
from esphome.components import wifi
from esphome.const import CONF_ID

DEPENDENCIES = ['esp32_camera', 'wifi']
AUTO_LOAD = ['esp32_camera']

CONF_CAMERA_ID = 'camera_id'
CONF_RTSP_PORT = 'rtsp_port'
CONF_FRAME_DURATION = 'frame_duration_ms'

micro_rtsp_ns = cg.esphome_ns.namespace('micro_rtsp')
MicroRTSP = micro_rtsp_ns.class_('MicroRTSP', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MicroRTSP),
    cv.Required(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
    cv.Optional(CONF_RTSP_PORT, default=8554): cv.port,
    cv.Optional(CONF_FRAME_DURATION, default=100): cv.positive_int,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    camera = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(camera))
    cg.add(var.set_port(config[CONF_RTSP_PORT]))
    cg.add(var.set_frame_duration(config[CONF_FRAME_DURATION]))
