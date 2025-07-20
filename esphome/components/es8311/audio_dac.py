import esphome.codegen as cg
from esphome.components import i2c
from esphome.components.audio_dac import AudioDac
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_I2C_ID,
    CONF_ADDRESS,
    CONF_SAMPLE_RATE,
    CONF_BITS_PER_SAMPLE,
    CONF_MIC_GAIN,
)

CODEOWNERS = ["@kroimon", "@kahrendt", "@NonaSuomy"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["audio_dac"]

es8311_ns = cg.esphome_ns.namespace("es8311")
ES8311 = es8311_ns.class_("ES8311", AudioDac, cg.Component, i2c.I2CDevice)

CONF_USE_MCLK = "use_mclk"
CONF_USE_MICROPHONE = "use_microphone"
CONF_MICROPHONE_TYPE = "microphone_type"

es8311_resolution = es8311_ns.enum("ES8311Resolution")
ES8311_BITS_PER_SAMPLE_ENUM = {
    16: es8311_resolution.ES8311_RESOLUTION_16,
    "16bit": es8311_resolution.ES8311_RESOLUTION_16,
    18: es8311_resolution.ES8311_RESOLUTION_18,
    "18bit": es8311_resolution.ES8311_RESOLUTION_18,
    20: es8311_resolution.ES8311_RESOLUTION_20,
    "20bit": es8311_resolution.ES8311_RESOLUTION_20,
    24: es8311_resolution.ES8311_RESOLUTION_24,
    "24bit": es8311_resolution.ES8311_RESOLUTION_24,
    32: es8311_resolution.ES8311_RESOLUTION_32,
    "32bit": es8311_resolution.ES8311_RESOLUTION_32,
}

es8311_mic_gain = es8311_ns.enum("ES8311MicGain")
ES8311_MIC_GAIN_ENUM = {
    "MIN": es8311_mic_gain.ES8311_MIC_GAIN_MIN,
    "0DB": es8311_mic_gain.ES8311_MIC_GAIN_0DB,
    "3DB": es8311_mic_gain.ES8311_MIC_GAIN_3DB,
    "6DB": es8311_mic_gain.ES8311_MIC_GAIN_6DB,
    "9DB": es8311_mic_gain.ES8311_MIC_GAIN_9DB,
    "12DB": es8311_mic_gain.ES8311_MIC_GAIN_12DB,
    "15DB": es8311_mic_gain.ES8311_MIC_GAIN_15DB,
    "18DB": es8311_mic_gain.ES8311_MIC_GAIN_18DB,
    "21DB": es8311_mic_gain.ES8311_MIC_GAIN_21DB,
    "24DB": es8311_mic_gain.ES8311_MIC_GAIN_24DB,
    "27DB": es8311_mic_gain.ES8311_MIC_GAIN_27DB,
    "30DB": es8311_mic_gain.ES8311_MIC_GAIN_30DB,
    "MAX": es8311_mic_gain.ES8311_MIC_GAIN_MAX,
}

es8311_microphone_type = es8311_ns.enum("ES8311MicrophoneType")
ES8311_MICROPHONE_TYPE_ENUM = {
    "analog": es8311_microphone_type.ES8311_MICROPHONE_ANALOG,
    "digital": es8311_microphone_type.ES8311_MICROPHONE_DIGITAL,
}

_validate_bits = cv.float_with_unit("bits", "bit")

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ES8311),
            cv.Optional(CONF_I2C_ID): cv.use_id(i2c.I2CBus),
            cv.Optional(CONF_ADDRESS, default=0x18): cv.i2c_address,
            cv.Optional(CONF_BITS_PER_SAMPLE, default=16): cv.enum(ES8311_BITS_PER_SAMPLE_ENUM),
            cv.Optional(CONF_MIC_GAIN, default="30DB"): cv.enum(
                ES8311_MIC_GAIN_ENUM, upper=True
            ),
			cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.All(cv.frequency, cv.Range(min=8000, max=96000)),
            cv.Optional(CONF_USE_MCLK, default=True): cv.boolean,
			cv.Optional(CONF_USE_MICROPHONE, default=True): cv.boolean,
            cv.Optional(CONF_MICROPHONE_TYPE, default="analog"): cv.enum(ES8311_MICROPHONE_TYPE_ENUM),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x18))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Set sample rate and bits per sample first
    cg.add(var.set_sample_frequency(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_bits_per_sample(config[CONF_BITS_PER_SAMPLE]))
    
    # Set other configuration
    cg.add(var.set_use_mclk(config[CONF_USE_MCLK]))
    cg.add(var.set_use_mic(config[CONF_USE_MICROPHONE]))
    cg.add(var.set_microphone_type(config[CONF_MICROPHONE_TYPE]))

    gain_str = config[CONF_MIC_GAIN]
    if gain_str == "0DB":
        cg.add(var.set_mic_gain(0))
    elif gain_str == "3DB":
        cg.add(var.set_mic_gain(1))
    elif gain_str == "6DB":
        cg.add(var.set_mic_gain(2))
    elif gain_str == "9DB":
        cg.add(var.set_mic_gain(3))
    elif gain_str == "12DB":
        cg.add(var.set_mic_gain(4))
    elif gain_str == "15DB":
        cg.add(var.set_mic_gain(5))
    elif gain_str == "18DB":
        cg.add(var.set_mic_gain(6))
    elif gain_str == "21DB":
        cg.add(var.set_mic_gain(7))
    elif gain_str == "24DB":
        cg.add(var.set_mic_gain(8))
    elif gain_str == "27DB":
        cg.add(var.set_mic_gain(9))
    elif gain_str == "30DB":
        cg.add(var.set_mic_gain(10))
    else:
        raise cv.Invalid(f"Invalid gain value: {gain_str}")
