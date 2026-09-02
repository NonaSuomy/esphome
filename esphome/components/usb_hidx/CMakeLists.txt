# Include-only bridge for native ESP-IDF builds.
#
# ESPHome's stock external-component loader stages source files directly in a
# component package but skips nested Python subpackages.  The HIDX drivers
# intentionally remain grouped under devices/<driver>/, so register this
# directory as a managed IDF component to preserve those include paths without
# changing ESPHome's loader or copying the driver headers into a flat tree.
idf_component_register(INCLUDE_DIRS ".")
