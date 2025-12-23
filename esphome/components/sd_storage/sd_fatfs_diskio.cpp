#include "sd_fatfs_diskio.h"
#include "esphome/core/log.h"
#include "diskio_impl.h"

namespace esphome {
namespace sd_storage {

static const char *TAG = "sd_fatfs_diskio";

// Global pointer to the SD driver instance
static SdSpiRaw *g_sd_driver = nullptr;

void fatfs_set_sd_driver(SdSpiRaw *driver) {
  g_sd_driver = driver;
}

SdSpiRaw *fatfs_get_sd_driver() {
  return g_sd_driver;
}

// ESP-IDF diskio callback implementations
static DSTATUS sd_raw_initialize(BYTE pdrv) {
  // Driver should already be initialized via SdSpiRaw::init()
  if (g_sd_driver == nullptr || !g_sd_driver->is_initialized()) {
    return STA_NOINIT;
  }
  return 0;  // OK
}

static DSTATUS sd_raw_status(BYTE pdrv) {
  if (g_sd_driver == nullptr || !g_sd_driver->is_initialized()) {
    return STA_NOINIT;
  }
  return 0;  // OK
}

static DRESULT sd_raw_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
  if (g_sd_driver == nullptr || !g_sd_driver->is_initialized()) {
    return RES_NOTRDY;
  }
  
  if (!g_sd_driver->read_blocks(sector, buff, count)) {
    return RES_ERROR;
  }
  
  return RES_OK;
}

static DRESULT sd_raw_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {
  if (g_sd_driver == nullptr || !g_sd_driver->is_initialized()) {
    return RES_NOTRDY;
  }
  
  if (!g_sd_driver->write_blocks(sector, buff, count)) {
    return RES_ERROR;
  }
  
  return RES_OK;
}

static DRESULT sd_raw_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
  if (g_sd_driver == nullptr || !g_sd_driver->is_initialized()) {
    return RES_NOTRDY;
  }
  
  switch (cmd) {
    case CTRL_SYNC:
      return RES_OK;
      
    case GET_SECTOR_COUNT:
      *(DWORD *)buff = g_sd_driver->get_sector_count();
      return RES_OK;
      
    case GET_SECTOR_SIZE:
      *(WORD *)buff = 512;
      return RES_OK;
      
    case GET_BLOCK_SIZE:
      *(DWORD *)buff = 1;
      return RES_OK;
      
    default:
      return RES_PARERR;
  }
}

// Register our raw SPI driver with ESP-IDF's diskio layer
void fatfs_register_raw_driver(uint8_t pdrv) {
  static const ff_diskio_impl_t raw_diskio_impl = {
    .init = sd_raw_initialize,
    .status = sd_raw_status,
    .read = sd_raw_read,
    .write = sd_raw_write,
    .ioctl = sd_raw_ioctl,
  };
  
  ff_diskio_register(pdrv, &raw_diskio_impl);
  ESP_LOGI(TAG, "Registered raw SPI SD driver as pdrv %d", pdrv);
}

}  // namespace sd_storage
}  // namespace esphome
