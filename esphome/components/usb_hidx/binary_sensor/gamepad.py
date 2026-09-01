CONF_TYPE = "type"


# Route to the appropriate device driver based on type.
def get_device_module(device_type):
    if device_type == "switch":
        from esphome.components.usb_hidx.devices.switch import (
            binary_sensor as device_module,
        )
    elif device_type in {"generic", "generic_gamepad"}:
        from esphome.components.usb_hidx.devices.generic_gamepad import (
            binary_sensor as device_module,
        )
    else:
        from esphome.components.usb_hidx.devices.generic_gamepad import (
            binary_sensor as device_module,
        )
    return device_module


def validate_config(config):
    device_type = config.get(CONF_TYPE, "generic")
    device_module = get_device_module(device_type)
    return device_module.CONFIG_SCHEMA(config)


async def to_code(config):
    device_type = config.get(CONF_TYPE, "generic")
    device_module = get_device_module(device_type)
    await device_module.to_code(config)


CONFIG_SCHEMA = validate_config
