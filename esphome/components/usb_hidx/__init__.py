from pathlib import Path

import esphome.codegen as cg
from esphome.components import (
    binary_sensor as esphome_binary_sensor,
    sensor as esphome_sensor,
)
from esphome.components.esp32 import add_idf_component, idf_version
import esphome.config_validation as cv
from esphome.const import CONF_DEVICE_ID, CONF_ID, CONF_TYPE
from esphome.core import CORE
from esphome.coroutine import CoroPriority, coroutine_with_priority

CODEOWNERS = ["@nonasuomy"]
DEPENDENCIES = ["esp32"]
# The legacy nested ``mouse``/``gamepad`` blocks create entities from this
# component's ``to_code`` function. Load only the entity runtimes those blocks
# actually use; platform-style entries load their own runtimes normally.
SOURCE_DIRS = ("devices",)

# This is an include-only IDF component.  It lets stock ESPHome's native IDF
# build see the driver headers below this Python component, even though stock
# ESPHome intentionally does not stage nested Python subpackages as C++ source.
# The matching CMakeLists.txt lives beside this module and is referenced by
# path at code-generation time; no ESPHome core patch is required.
_IDF_DRIVER_COMPONENT = "usb_hidx"

CONF_HUB = "hub"
CONF_KEYBOARD = "keyboard"
CONF_MOUSE = "mouse"
CONF_GAMEPAD = "gamepad"
CONF_DRIVERS = "drivers"
CONF_MCE_REMOTE = "mce_remote"
CONF_VID = "vid"
CONF_PID = "pid"
CONF_LAYOUT = "layout"
CONF_LEFT_BUTTON = "left_button"
CONF_RIGHT_BUTTON = "right_button"
CONF_MIDDLE_BUTTON = "middle_button"
CONF_BUTTON_A = "button_a"
CONF_BUTTON_B = "button_b"
CONF_X_DELTA = "x_delta"
CONF_Y_DELTA = "y_delta"
CONF_WHEEL = "wheel"
CONF_DRIVER = "driver"
CONF_MASK = "mask"

DRIVER_NAMES = (
    "keyboard",
    "mouse",
    "generic_gamepad",
    "xbox360",
    "xboxone",
    "playstation",
    "steam",
    "stadia",
    "switch",
    "wiimote",
    "thrustmaster",
    "touchscreen",
    "interact",
    "logitech",
    "mce_remote",
    "mcp2221",
    "cp2112",
    "ft260",
)

DRIVER_ALIASES = {
    "generic": "generic_gamepad",
    "generic_gamepad": "generic_gamepad",
    "keyboard": "keyboard",
    "mouse": "mouse",
    "xbox360": "xbox360",
    "xboxone": "xboxone",
    "playstation": "playstation",
    "ps3": "playstation",
    "ps4": "playstation",
    "ps5": "playstation",
    "steam": "steam",
    "stadia": "stadia",
    "switch": "switch",
    "wiimote": "wiimote",
    "thrustmaster": "thrustmaster",
    "touchscreen": "touchscreen",
    "interact": "interact",
    "logitech": "logitech",
    "mce": "mce_remote",
    "mce_remote": "mce_remote",
    "mcp2221": "mcp2221",
    "cp2112": "cp2112",
    "ft260": "ft260",
}

DRIVER_MACROS = {name: "USB_HIDX_ENABLE_" + name.upper() for name in DRIVER_NAMES}

usb_hidx_ns = cg.esphome_ns.namespace("usb_hidx")
USBHIDXComponent = usb_hidx_ns.class_("USBHIDXComponent", cg.Component)

KEYBOARD_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_LAYOUT, default="us"): cv.one_of(
            "us", "uk", "de", "fr", "es", lower=True
        ),
    }
)

MOUSE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_LEFT_BUTTON): esphome_binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_RIGHT_BUTTON): esphome_binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_MIDDLE_BUTTON): esphome_binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_X_DELTA): esphome_sensor.sensor_schema(),
        cv.Optional(CONF_Y_DELTA): esphome_sensor.sensor_schema(),
        cv.Optional(CONF_WHEEL): esphome_sensor.sensor_schema(),
    }
)

GAMEPAD_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_DEVICE_ID): cv.string,
        cv.Optional(CONF_TYPE, default="generic"): cv.one_of(
            *DRIVER_ALIASES.keys(), lower=True
        ),
        cv.Optional(CONF_BUTTON_A): esphome_binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_BUTTON_B): esphome_binary_sensor.binary_sensor_schema(),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(USBHIDXComponent),
        cv.Optional(CONF_HUB, default=True): cv.boolean,
        cv.Optional(CONF_DRIVERS, default=[]): cv.ensure_list(
            cv.one_of(*DRIVER_ALIASES.keys(), lower=True)
        ),
        cv.Optional(CONF_MCE_REMOTE, default=False): cv.boolean,
        cv.Optional(CONF_KEYBOARD): KEYBOARD_SCHEMA,
        cv.Optional(CONF_MOUSE): MOUSE_SCHEMA,
        cv.Optional(CONF_GAMEPAD): GAMEPAD_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)


def AUTO_LOAD(config):
    loads = ["usb_host"]
    mouse = config.get(CONF_MOUSE, {})
    gamepad = config.get(CONF_GAMEPAD, {})
    if any(
        key in mouse
        for key in (CONF_LEFT_BUTTON, CONF_RIGHT_BUTTON, CONF_MIDDLE_BUTTON)
    ) or any(key in gamepad for key in (CONF_BUTTON_A, CONF_BUTTON_B)):
        loads.append("binary_sensor")
    if any(key in mouse for key in (CONF_X_DELTA, CONF_Y_DELTA, CONF_WHEEL)):
        loads.append("sensor")
    return loads


@coroutine_with_priority(CoroPriority.FINAL)
def _register_standalone_idf_components():
    """Register the bundled native-IDF pieces after all component code runs.

    The final priority is intentional.  It allows this external component to
    replace the stock ``espressif/usb`` 1.4.x dependency that the auto-loaded
    ``usb_host`` component registers, while still honoring a project-local
    ``config/idf_components/usb`` override used for development.
    """
    if not CORE.using_toolchain_esp_idf:
        return

    component_dir = Path(__file__).resolve().parent
    if (component_dir / "CMakeLists.txt").is_file():
        add_idf_component(
            name=_IDF_DRIVER_COMPONENT,
            path=str(component_dir),
        )

    # ESP-IDF 6 moved USB host out of the framework.  Use the bundled HCD
    # implementation so P4 split/TT transfers work without modifying the
    # user's ESP-IDF or ESPHome installation.
    if idf_version() >= cv.Version(6, 0, 0):
        local_usb = CORE.config_dir / "idf_components" / "usb"
        packaged_usb = component_dir.parents[2] / "usb_hidx_idf" / "usb"
        usb_override = local_usb if local_usb.is_dir() else packaged_usb
        if usb_override.is_dir():
            add_idf_component(name="espressif/usb", path=str(usb_override))


async def to_code(config):
    component = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(component, config)

    # Schedule after the auto-loaded usb_host component so this component is
    # usable from stock ESPHome with only an external usb_hidx checkout.
    CORE.add_job(_register_standalone_idf_components)

    # Keep the driver registry's relative device includes local to this
    # component for PlatformIO/Arduino builds. Native ESP-IDF uses the
    # include-only managed component registered above because its framework
    # helper intentionally filters -I from global build flags.
    cg.add_build_flag(f"-I{Path(__file__).parent}")

    if config[CONF_HUB]:
        cg.add_define("USB_HIDX_HUB_ENABLED")

    # Select individual drivers at compile time.  The old implementation
    # selected whole families (for example every gamepad driver); keeping the
    # selection at the individual-driver level is important on memory-limited
    # ESP32 targets.
    selected_drivers = {DRIVER_ALIASES[name] for name in config.get(CONF_DRIVERS, [])}

    if CONF_KEYBOARD in config:
        selected_drivers.add("keyboard")
        # Set keyboard layout
        layout = config[CONF_KEYBOARD].get(CONF_LAYOUT, "us")
        if layout == "us":
            cg.add_define("KEYBOARD_LAYOUT_US")
        elif layout == "uk":
            cg.add_define("KEYBOARD_LAYOUT_UK")
        elif layout == "de":
            cg.add_define("KEYBOARD_LAYOUT_DE")
        elif layout == "fr":
            cg.add_define("KEYBOARD_LAYOUT_FR")
        elif layout == "es":
            cg.add_define("KEYBOARD_LAYOUT_ES")
    if CONF_MOUSE in config:
        selected_drivers.add("mouse")
    if CONF_GAMEPAD in config:
        selected_drivers.add(
            DRIVER_ALIASES[config[CONF_GAMEPAD].get(CONF_TYPE, "generic")]
        )
    if config.get(CONF_MCE_REMOTE, False):
        selected_drivers.add("mce_remote")

    for driver in sorted(selected_drivers):
        cg.add_build_flag(f"-D{DRIVER_MACROS[driver]}")

    # Keep the legacy nested blocks functional. These schemas predate the
    # platform-style entities, but they still appear in existing user YAML.
    # They must be materialized here; otherwise validation succeeds while the
    # requested mappings are silently discarded.
    mouse = config.get(CONF_MOUSE)
    if mouse is not None:
        if CONF_LEFT_BUTTON in mouse:
            entity = await esphome_binary_sensor.new_binary_sensor(
                mouse[CONF_LEFT_BUTTON]
            )
            cg.add(component.register_mouse_left_sensor(entity))
        if CONF_RIGHT_BUTTON in mouse:
            entity = await esphome_binary_sensor.new_binary_sensor(
                mouse[CONF_RIGHT_BUTTON]
            )
            cg.add(component.register_mouse_right_sensor(entity))
        if CONF_MIDDLE_BUTTON in mouse:
            entity = await esphome_binary_sensor.new_binary_sensor(
                mouse[CONF_MIDDLE_BUTTON]
            )
            cg.add(component.register_mouse_middle_sensor(entity))
        if CONF_X_DELTA in mouse:
            entity = await esphome_sensor.new_sensor(mouse[CONF_X_DELTA])
            cg.add(component.register_mouse_x_sensor(entity))
        if CONF_Y_DELTA in mouse:
            entity = await esphome_sensor.new_sensor(mouse[CONF_Y_DELTA])
            cg.add(component.register_mouse_y_sensor(entity))
        if CONF_WHEEL in mouse:
            entity = await esphome_sensor.new_sensor(mouse[CONF_WHEEL])
            cg.add(component.register_mouse_wheel_sensor(entity))

    gamepad = config.get(CONF_GAMEPAD)
    if gamepad is not None:
        if CONF_BUTTON_A in gamepad:
            entity = await esphome_binary_sensor.new_binary_sensor(
                gamepad[CONF_BUTTON_A]
            )
            cg.add(component.register_gamepad_button_a_sensor(entity))
        if CONF_BUTTON_B in gamepad:
            entity = await esphome_binary_sensor.new_binary_sensor(
                gamepad[CONF_BUTTON_B]
            )
            cg.add(component.register_gamepad_button_b_sensor(entity))


def enable_driver(driver):
    """Enable one driver from a platform declaration.

    Platform entries are allowed to be used without a populated legacy
    ``usb_hidx.keyboard/mouse/gamepad`` block, so they must participate in
    compile-time driver selection themselves.
    """
    canonical = DRIVER_ALIASES.get(driver.lower(), driver.lower())
    if canonical not in DRIVER_MACROS:
        raise cv.Invalid(f"Unknown USB HIDX driver: {driver}")
    cg.add_build_flag(f"-D{DRIVER_MACROS[canonical]}")


def driver_from_type(driver_type):
    canonical = DRIVER_ALIASES.get(driver_type.lower())
    if canonical is None:
        raise cv.Invalid(f"Unknown USB HIDX driver type: {driver_type}")
    return canonical
