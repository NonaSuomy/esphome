#pragma once

// FatFs disk I/O interface for raw SPI SD card access
// This connects the SdSpiRaw driver to ESP-IDF's FatFs diskio layer

#include "sd_spi_raw.h"

namespace esphome {
namespace sd_storage {

/**
 * Set the SdSpiRaw instance to use for FatFs disk I/O.
 * Must be called before any FatFs operations.
 */
void fatfs_set_sd_driver(SdSpiRaw *driver);

/**
 * Get the current SdSpiRaw instance.
 */
SdSpiRaw *fatfs_get_sd_driver();

/**
 * Register the raw SPI driver with ESP-IDF's diskio layer.
 * @param pdrv Physical drive number (0-9)
 */
void fatfs_register_raw_driver(uint8_t pdrv);

}  // namespace sd_storage
}  // namespace esphome
