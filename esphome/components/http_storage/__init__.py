import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_URL, CONF_UPDATE_INTERVAL, CONF_USERNAME, CONF_PASSWORD
from esphome.components import storage

DEPENDENCIES = ['network']
AUTO_LOAD = ['storage']

http_storage_ns = cg.esphome_ns.namespace('http_storage')
HttpStorage = http_storage_ns.class_('HttpStorage', cg.Component, storage.StorageDevice)

CONF_MOUNT_PATH = "mount_path"
CONF_STORAGE_ID = "storage_id"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HttpStorage),
    cv.Required(CONF_STORAGE_ID): cv.use_id(storage.Storage),
    cv.Required(CONF_URL): cv.string,
    cv.Optional(CONF_MOUNT_PATH, default="/http"): cv.string,
    cv.Optional(CONF_USERNAME): cv.string,
    cv.Optional(CONF_PASSWORD): cv.string,
    cv.Optional(CONF_UPDATE_INTERVAL): cv.update_interval,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_url(config[CONF_URL]))
    cg.add(var.set_mount_path(config[CONF_MOUNT_PATH]))
    
    if CONF_USERNAME in config:
        cg.add(var.set_auth(config[CONF_USERNAME], config[CONF_PASSWORD]))
        
    cg.add_library("bblanchon/ArduinoJson", "7.0.4")
        
    cg.add_define("USE_HTTP_STORAGE")
    
    # Register with the storage component
    storage_component = await cg.get_variable(config[CONF_STORAGE_ID])
    cg.add(storage_component.register_device(var))
