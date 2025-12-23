#pragma once

#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "sd_storage_base.h"  // For CardType enum

namespace esphome {
namespace sd_storage {

// SD Card Commands
static const uint8_t CMD0 = 0;    // GO_IDLE_STATE
static const uint8_t CMD8 = 8;    // SEND_IF_COND
static const uint8_t CMD9 = 9;    // SEND_CSD
static const uint8_t CMD10 = 10;  // SEND_CID
static const uint8_t CMD12 = 12;  // STOP_TRANSMISSION
static const uint8_t CMD16 = 16;  // SET_BLOCKLEN
static const uint8_t CMD17 = 17;  // READ_SINGLE_BLOCK
static const uint8_t CMD18 = 18;  // READ_MULTIPLE_BLOCK
static const uint8_t CMD24 = 24;  // WRITE_BLOCK
static const uint8_t CMD25 = 25;  // WRITE_MULTIPLE_BLOCK
static const uint8_t CMD55 = 55;  // APP_CMD
static const uint8_t CMD58 = 58;  // READ_OCR
static const uint8_t ACMD41 = 41; // SD_SEND_OP_COND (after CMD55)

// Use CardType from sd_storage_base.h, not a local definition

/**
 * Raw SPI SD Card driver that uses ESPHome's SPIDevice infrastructure.
 * This allows the SD card to share the SPI bus with other devices (like displays)
 * without the conflicts caused by ESP-IDF's VFS-based SD mount.
 */
class SdSpiRaw {
 public:
  /**
   * Initialize the SD card using raw SPI commands.
   * @param delegate The SPI delegate from ESPHome's SPI component
   * @param cs_pin The chip select pin for the SD card
   * @param frequency_khz The SPI frequency in kHz (typically 400 for init, then higher)
   * @return true if initialization succeeded
   */
  bool init(spi::SPIDelegate *delegate, GPIOPin *cs_pin, uint32_t frequency_khz = 400);

  /**
   * Read a single 512-byte block from the SD card.
   * @param lba Logical block address (sector number)
   * @param buffer Buffer to receive the 512 bytes
   * @return true if read succeeded
   */
  bool read_block(uint32_t lba, uint8_t *buffer);

  /**
   * Read multiple 512-byte blocks from the SD card.
   * @param lba Starting logical block address
   * @param buffer Buffer to receive the blocks
   * @param count Number of blocks to read
   * @return true if read succeeded
   */
  bool read_blocks(uint32_t lba, uint8_t *buffer, size_t count);

  /**
   * Write a single 512-byte block to the SD card.
   * @param lba Logical block address (sector number)
   * @param buffer Data to write (512 bytes)
   * @return true if write succeeded
   */
  bool write_block(uint32_t lba, const uint8_t *buffer);

  /**
   * Write multiple 512-byte blocks to the SD card.
   * @param lba Starting logical block address
   * @param buffer Data to write
   * @param count Number of blocks to write
   * @return true if write succeeded
   */
  bool write_blocks(uint32_t lba, const uint8_t *buffer, size_t count);

  /**
   * Get the total number of sectors on the card.
   * @return Number of 512-byte sectors
   */
  uint32_t get_sector_count() { return this->sector_count_; }

  /**
   * Get the card type.
   */
  CardType get_card_type() { return this->card_type_; }

  /**
   * Check if the card is high capacity (SDHC/SDXC).
   */
  bool is_high_capacity() { return this->card_type_ == CardType::SDHC; }

  /**
   * Check if initialization was successful.
   */
  bool is_initialized() { return this->initialized_; }

 protected:
  /**
   * Send a command to the SD card and get response.
   * @param cmd Command number (0-63)
   * @param arg 32-bit command argument
   * @return R1 response byte, or 0xFF on error
   */
  uint8_t send_cmd(uint8_t cmd, uint32_t arg);

  /**
   * Send ACMD (application-specific command).
   * This sends CMD55 first, then the actual command.
   */
  uint8_t send_acmd(uint8_t cmd, uint32_t arg);

  /**
   * Wait for the card to be ready (not busy).
   * @param timeout_ms Timeout in milliseconds
   * @return true if card is ready
   */
  bool wait_ready(uint32_t timeout_ms = 500);

  /**
   * Select the SD card (pull CS low).
   */
  void select();

  /**
   * Deselect the SD card (pull CS high).
   */
  void deselect();

  /**
   * Read CSD register to get card capacity.
   */
  bool read_csd();

  spi::SPIDelegate *delegate_{nullptr};
  GPIOPin *cs_pin_{nullptr};
  CardType card_type_{CardType::UNKNOWN};
  uint32_t sector_count_{0};
  bool initialized_{false};
};

}  // namespace sd_storage
}  // namespace esphome
