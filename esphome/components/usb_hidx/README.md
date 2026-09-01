# USB HIDX

USB HIDX is an ESPHome USB HID host component for ESP32 targets with USB host
support. Drivers are selected at code-generation time; a build only includes
drivers requested by the component configuration or by a USB HIDX platform.

## Using the GitHub test branch

To use the current HIDX and matching USB host component without checking out
the fork, add this to an ESPHome YAML configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/NonaSuomy/esphome
      ref: hidx-testing-004
    components:
      - usb_hidx
      - usb_host
    refresh: 1d
```

The `usb_host` entry is intentional: it provides the matching host-component
configuration used by the HIDX branch, including the selectable USB
peripheral map.

## Driver selection

The legacy category blocks remain supported:

```yaml
usb_hidx:
  id: usb_hidx_component
  hub: true
  keyboard:
    layout: us
  mouse: {}
  gamepad:
    type: xbox360
```

Platform entries select the driver they need. Gamepad entries default to the
generic parser; set `driver:` when a protocol-specific parser is required. A
build can also explicitly select drivers:

```yaml
usb_hidx:
  id: usb_hidx_component
  drivers: [keyboard, mouse, xbox360]
```

Supported driver names include `keyboard`, `mouse`, `generic_gamepad`,
`xbox360`, `xboxone`, `playstation`, `steam`, `stadia`, `switch`, `wiimote`,
`thrustmaster`, `touchscreen`, `interact`, `logitech`, `mce_remote`, `mcp2221`,
`cp2112`, and `ft260`.

## Standard entities

Use normal ESPHome binary sensors, sensors, and text sensors. Their regular
`on_press`, `on_release`, and `on_value` automation blocks work without driver
changes. For example:

```yaml
binary_sensor:
  - platform: usb_hidx
    usb_hidx_id: usb_hidx_component
    type: keyboard
    key: 0x04  # HID usage for A
    name: Keyboard A
    on_press:
      - light.turn_on: status_light
```

## Raw report mappings

When a device has a report layout that is not covered by a dedicated driver,
map a byte and mask directly from YAML. Offsets are zero-based and include the
HID report ID when the device provides one. `vid` and `pid` are optional and
zero means “any device”.

```yaml
binary_sensor:
  - platform: usb_hidx
    usb_hidx_id: usb_hidx_component
    type: raw
    vid: 0x1234
    pid: 0x5678
    offset: 2
    mask: 0x04
    name: Device Button
    on_press:
      - logger.log: Device button pressed

sensor:
  - platform: usb_hidx
    usb_hidx_id: usb_hidx_component
    type: raw
    vid: 0x1234
    pid: 0x5678
    offset: 3
    length: 2
    signed: true
    scale: 0.01
    name: Device Axis
```

Raw binary sensors are active when any masked bit is set. Add `value` to match
a specific masked value. Raw sensors decode one to four little-endian bytes,
then apply `value * scale + bias`.

`vid` and `pid` selectors are currently supported for `type: raw` mappings.
This is the deterministic way to separate identical devices or multiple
devices of the same class; the legacy standard keyboard/mouse/gamepad outputs
are shared by their selected driver.

The legacy `device_id` fields are accepted for configuration compatibility but
are metadata only; they do not currently route standard driver reports to
separate entity sets. Use `type: raw` with `vid`/`pid` when per-device routing
is required.

Driver headers remain grouped under `devices/<driver>/`. The fork's source
packaging preserves that directory tree, so adding a driver does not require
flattening or duplicating its files.

When this component is fetched from GitHub, use the matching ESPHome fork
(`hidx-testing-004`) as the ESPHome runtime, or use a local component checkout.
The directory-preserving `SOURCE_DIRS` packaging hook is part of this fork;
stock ESPHome loaders may omit headers below Python subpackage directories.

Input entities are binary sensors/sensors because they represent reports from
the USB device. Use regular ESPHome `button` or `switch` entities with HIDX
output actions for device output reports such as rumble or LEDs.

## USB host resources

The component does not impose a fixed number of connected HID devices. Each
device record is allocated when a device arrives and released after unplug.
The actual limit is the ESP32-P4 USB host/HCD resources (channels, FIFO/DMA
memory, and the configured hub topology). `resource_status` exposes the HCD
channel usage so an installation can report a real rejection instead of a
guessed device count.

The ESP32-P4 DWC configuration in this project includes the local split/TT HCD
override. For native ESP-IDF 6.x builds, `usb_host` selects
`config/idf_components/usb` through the generated IDF dependency manifest when
that directory is present. Keep that directory with the project when moving
the configuration to another checkout. `tt_usb_override.py` is retained for
PlatformIO/legacy-IDF builds; native ESP-IDF builds do not execute
`platformio_options.extra_scripts`.
