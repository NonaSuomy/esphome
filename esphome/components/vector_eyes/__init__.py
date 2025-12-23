import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import display, speaker, font, time
from esphome.const import CONF_ID

DEPENDENCIES = ['display']

vector_eyes_ns = cg.esphome_ns.namespace('vector_eyes')
VectorEyes = vector_eyes_ns.class_('VectorEyes', cg.Component)

CONF_DISPLAY_ID = 'display_id'
CONF_SPEAKER_ID = 'speaker_id'
CONF_STORAGE_ID = 'storage_id'
CONF_MOUNT_PATH = 'mount_path'
CONF_FONT = 'font'
CONF_TIME_ID = 'time_id'


# DEPRECATED: SD card pin (kept for backward compatibility)
CONF_CS_PIN = "cs_pin"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(VectorEyes),
    cv.Required(CONF_DISPLAY_ID): cv.use_id(display.DisplayBuffer),
    cv.Optional(CONF_SPEAKER_ID): cv.use_id(speaker.Speaker),
    # NEW: Storage component integration
    cv.Optional(CONF_STORAGE_ID): cv.use_id(cg.Component),  # Reference to storage component
    cv.Optional(CONF_STORAGE_ID): cv.use_id(cg.Component),  # Reference to storage component
    cv.Optional(CONF_MOUNT_PATH, default="/sd"): cv.string,  # Mount path for animations
    cv.Optional(CONF_FONT): cv.use_id(font.Font),  # Font for loading screen text
    cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock), # Time component for clock
    # DEPRECATED: Direct SD card configuration (kept for backward compatibility)
    cv.Optional(CONF_CS_PIN): pins.gpio_output_pin_schema,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    disp = await cg.get_variable(config[CONF_DISPLAY_ID])
    cg.add(var.set_display(disp))
    
    if CONF_SPEAKER_ID in config:
        spk = await cg.get_variable(config[CONF_SPEAKER_ID])
        cg.add(var.set_speaker(spk))
    
    # NEW: Storage component configuration (preferred method)
    # Note: Storage component integration is optional and requires USE_STORAGE define
    # The C++ code already handles conditional compilation with #ifdef USE_STORAGE
    if CONF_STORAGE_ID in config:
        storage = await cg.get_variable(config[CONF_STORAGE_ID])
        cg.add(var.set_storage(storage))
        
        # NEW: Mount path configuration (only when using storage component)
        if CONF_MOUNT_PATH in config:
            cg.add(var.set_mount_path(config[CONF_MOUNT_PATH]))
        
        # Add USE_STORAGE define as a build flag
        # Add USE_STORAGE define as a build flag
        cg.add_build_flag("-DUSE_STORAGE")
    
    if CONF_FONT in config:
        f = await cg.get_variable(config[CONF_FONT])
        cg.add(var.set_font(f))
        
    if CONF_TIME_ID in config:
        t = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time(t))
    
    # DEPRECATED: Direct SD card configuration (kept for backward compatibility)
    # If both storage_id and cs_pin are provided, storage_id takes precedence
    if CONF_CS_PIN in config:
        if CONF_STORAGE_ID not in config:
            # Only use cs_pin if storage_id is not provided
            cs = await cg.gpio_pin_expression(config[CONF_CS_PIN])
            cg.add(var.set_sd_cs_pin(cs))
        else:
            # Log a warning that cs_pin is ignored when storage_id is present
            cg.add(cg.RawExpression(
                'ESP_LOGW("vector_eyes", "cs_pin is deprecated and ignored when storage_id is provided")'
            ))
    
    # Add ArduinoJson for JSON animation parsing
    cg.add_library("bblanchon/ArduinoJson", "7.4.2")
