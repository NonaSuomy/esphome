#include "sd_spi_raw.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sd_storage {

static const char *TAG = "sd_spi_raw";

// Block size is always 512 bytes for SD cards
static const size_t BLOCK_SIZE = 512;

// Timeouts
static const uint32_t INIT_TIMEOUT_MS = 1000;
static const uint32_t CMD_TIMEOUT_MS = 100;

bool SdSpiRaw::init(spi::SPIDelegate *delegate, GPIOPin *cs_pin, uint32_t frequency_khz) {
  this->delegate_ = delegate;
  this->cs_pin_ = cs_pin;
  this->initialized_ = false;
  this->card_type_ = CardType::UNKNOWN;

  if (this->delegate_ == nullptr || this->cs_pin_ == nullptr) {
    ESP_LOGE(TAG, "Invalid delegate or CS pin");
    return false;
  }

  // Setup CS pin as output, initially high (deselected)
  this->cs_pin_->setup();
  this->cs_pin_->digital_write(true);

  ESP_LOGI(TAG, "Initializing SD card via raw SPI");

  // Send 80+ clock cycles with CS high to initialize card
  this->delegate_->begin_transaction();
  uint8_t dummy = 0xFF;
  for (int i = 0; i < 10; i++) {
    this->delegate_->transfer(&dummy, &dummy, 1);
  }
  this->delegate_->end_transaction();

  // CMD0 - Go to idle state
  this->select();
  uint8_t r1 = this->send_cmd(CMD0, 0);
  this->deselect();

  if (r1 != 0x01) {
    ESP_LOGE(TAG, "CMD0 failed, response: 0x%02X", r1);
    return false;
  }
  ESP_LOGD(TAG, "CMD0 OK - card in idle state");

  // CMD8 - Check for SDv2
  this->select();
  r1 = this->send_cmd(CMD8, 0x000001AA);
  
  if (r1 == 0x01) {
    // SDv2 card - read remaining 4 bytes of R7 response
    uint8_t r7[4];
    this->delegate_->begin_transaction();
    this->delegate_->read_array(r7, 4);
    this->delegate_->end_transaction();
    this->deselect();

    if (r7[2] != 0x01 || r7[3] != 0xAA) {
      ESP_LOGE(TAG, "CMD8 returned invalid voltage range");
      return false;
    }
    ESP_LOGD(TAG, "CMD8 OK - SDv2 card detected");

    // ACMD41 with HCS bit set for SDHC support
    uint32_t start_time = millis();
    do {
      this->select();
      r1 = this->send_acmd(ACMD41, 0x40000000);  // HCS bit set
      this->deselect();
      if (millis() - start_time > INIT_TIMEOUT_MS) {
        ESP_LOGE(TAG, "ACMD41 timeout");
        return false;
      }
      delay(10);
    } while (r1 != 0x00);

    ESP_LOGD(TAG, "ACMD41 OK - card ready");

    // CMD58 - Read OCR to check for SDHC
    this->select();
    r1 = this->send_cmd(CMD58, 0);
    if (r1 == 0x00) {
      uint8_t ocr[4];
      this->delegate_->begin_transaction();
      this->delegate_->read_array(ocr, 4);
      this->delegate_->end_transaction();
      
      if (ocr[0] & 0x40) {
        this->card_type_ = CardType::SDHC;
        ESP_LOGI(TAG, "SDHC/SDXC card detected");
      } else {
        this->card_type_ = CardType::SDSC;
        ESP_LOGI(TAG, "SDSC v2 card detected");
      }
    }
    this->deselect();

  } else if (r1 == 0x05) {
    // SDv1 or MMC card (illegal command response)
    this->deselect();
    ESP_LOGD(TAG, "No CMD8 support - SDv1/MMC card");

    // Try ACMD41 for SDv1
    uint32_t start_time = millis();
    do {
      this->select();
      r1 = this->send_acmd(ACMD41, 0);
      this->deselect();
      if (millis() - start_time > INIT_TIMEOUT_MS) {
        // ACMD41 failed, try CMD1 for MMC
        ESP_LOGD(TAG, "ACMD41 failed, trying CMD1 for MMC");
        start_time = millis();
        do {
          this->select();
          r1 = this->send_cmd(1, 0);  // CMD1 for MMC
          this->deselect();
          if (millis() - start_time > INIT_TIMEOUT_MS) {
            ESP_LOGE(TAG, "Card init timeout");
            return false;
          }
          delay(10);
        } while (r1 != 0x00);
        this->card_type_ = CardType::MMC;
        ESP_LOGI(TAG, "MMC card detected");
        break;
      }
      delay(10);
    } while (r1 != 0x00);

    if (this->card_type_ == CardType::UNKNOWN) {
      this->card_type_ = CardType::SDSC;
      ESP_LOGI(TAG, "SDv1 card detected");
    }
  } else {
    this->deselect();
    ESP_LOGE(TAG, "CMD8 returned unexpected response: 0x%02X", r1);
    return false;
  }

  // For non-SDHC cards, set block size to 512
  if (this->card_type_ != CardType::SDHC) {
    this->select();
    r1 = this->send_cmd(CMD16, BLOCK_SIZE);
    this->deselect();
    if (r1 != 0x00) {
      ESP_LOGW(TAG, "CMD16 failed: 0x%02X (ignoring)", r1);
    }
  }

  // Read CSD to get card capacity
  if (!this->read_csd()) {
    ESP_LOGW(TAG, "Failed to read CSD, capacity unknown");
  }

  this->initialized_ = true;
  ESP_LOGI(TAG, "SD card initialized successfully, type=%d, sectors=%u", 
           this->card_type_, this->sector_count_);
  
  return true;
}

void SdSpiRaw::select() {
  this->delegate_->begin_transaction(); // Lock bus (and toggle dummy CS)
  this->cs_pin_->digital_write(false);  // Active low
  delayMicroseconds(1);
}

void SdSpiRaw::deselect() {
  this->cs_pin_->digital_write(true);  // Inactive high
  
  // Send dummy byte to allow card to release MISO (bus is still locked)
  uint8_t dummy = 0xFF;
  this->delegate_->transfer(&dummy, &dummy, 1);
  
  this->delegate_->end_transaction();   // Unlock bus
}

bool SdSpiRaw::wait_ready(uint32_t timeout_ms) {
  uint32_t start = millis();
  this->delegate_->begin_transaction();
  uint8_t response;
  do {
    this->delegate_->read_array(&response, 1);
    if (response == 0xFF) {
      this->delegate_->end_transaction();
      return true;
    }
  } while (millis() - start < timeout_ms);
  this->delegate_->end_transaction();
  return false;
}

uint8_t SdSpiRaw::send_cmd(uint8_t cmd, uint32_t arg) {
  // Wait for card to be ready
  if (!this->wait_ready(CMD_TIMEOUT_MS)) {
    return 0xFF;
  }

  this->delegate_->begin_transaction();

  // Send command packet
  uint8_t packet[6];
  packet[0] = 0x40 | cmd;  // Command with start bit and transmission bit
  packet[1] = (arg >> 24) & 0xFF;
  packet[2] = (arg >> 16) & 0xFF;
  packet[3] = (arg >> 8) & 0xFF;
  packet[4] = arg & 0xFF;
  
  // CRC (required for CMD0 and CMD8, others don't check)
  if (cmd == CMD0) {
    packet[5] = 0x95;  // Valid CRC for CMD0
  } else if (cmd == CMD8) {
    packet[5] = 0x87;  // Valid CRC for CMD8 with arg 0x1AA
  } else {
    packet[5] = 0x01;  // Dummy CRC with stop bit
  }

  this->delegate_->write_array(packet, 6);

  // Wait for response (R1 format)
  uint8_t response;
  int attempts = 10;
  do {
    this->delegate_->read_array(&response, 1);
    if ((response & 0x80) == 0) {
      break;  // Valid response (MSB is 0)
    }
  } while (--attempts > 0);

  this->delegate_->end_transaction();

  return response;
}

uint8_t SdSpiRaw::send_acmd(uint8_t cmd, uint32_t arg) {
  // Send CMD55 first
  uint8_t r1 = this->send_cmd(CMD55, 0);
  if (r1 > 0x01) {
    return r1;
  }
  // Then send the actual application command
  return this->send_cmd(cmd, arg);
}

bool SdSpiRaw::read_csd() {
  this->select();
  uint8_t r1 = this->send_cmd(CMD9, 0);
  
  if (r1 != 0x00) {
    this->deselect();
    return false;
  }

  // Wait for data token (0xFE)
  this->delegate_->begin_transaction();
  uint8_t token;
  uint32_t start = millis();
  do {
    this->delegate_->read_array(&token, 1);
    if (millis() - start > CMD_TIMEOUT_MS) {
      this->delegate_->end_transaction();
      this->deselect();
      return false;
    }
  } while (token == 0xFF);

  if (token != 0xFE) {
    this->delegate_->end_transaction();
    this->deselect();
    return false;
  }

  // Read 16 bytes of CSD data
  uint8_t csd[16];
  this->delegate_->read_array(csd, 16);

  // Read 2 bytes CRC (and discard)
  uint8_t crc[2];
  this->delegate_->read_array(crc, 2);

  this->delegate_->end_transaction();
  this->deselect();

  // Parse CSD to get capacity
  uint8_t csd_version = (csd[0] >> 6) & 0x03;
  
  if (csd_version == 0) {
    // CSD v1.0 (SDSC)
    uint32_t read_bl_len = csd[5] & 0x0F;
    uint32_t c_size = ((csd[6] & 0x03) << 10) | (csd[7] << 2) | ((csd[8] >> 6) & 0x03);
    uint32_t c_size_mult = ((csd[9] & 0x03) << 1) | ((csd[10] >> 7) & 0x01);
    
    uint32_t mult = 1 << (c_size_mult + 2);
    uint32_t blocknr = (c_size + 1) * mult;
    uint32_t block_len = 1 << read_bl_len;
    
    this->sector_count_ = (blocknr * block_len) / BLOCK_SIZE;
  } else if (csd_version == 1) {
    // CSD v2.0 (SDHC/SDXC)
    uint32_t c_size = ((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9];
    this->sector_count_ = (c_size + 1) * 1024;  // Each unit is 512KB
  }

  ESP_LOGD(TAG, "CSD v%d: sector_count=%u", csd_version + 1, this->sector_count_);
  return true;
}

bool SdSpiRaw::read_block(uint32_t lba, uint8_t *buffer) {
  if (!this->initialized_ || buffer == nullptr) {
    return false;
  }

  // For SDSC cards, address is byte address; for SDHC, it's block address
  uint32_t addr = this->is_high_capacity() ? lba : (lba * BLOCK_SIZE);

  this->select();
  uint8_t r1 = this->send_cmd(CMD17, addr);
  
  if (r1 != 0x00) {
    this->deselect();
    ESP_LOGE(TAG, "CMD17 failed: 0x%02X", r1);
    return false;
  }

  // Wait for data token (0xFE)
  this->delegate_->begin_transaction();
  uint8_t token;
  uint32_t start = millis();
  do {
    this->delegate_->read_array(&token, 1);
    if (millis() - start > CMD_TIMEOUT_MS) {
      this->delegate_->end_transaction();
      this->deselect();
      ESP_LOGE(TAG, "Read timeout waiting for data token");
      return false;
    }
  } while (token == 0xFF);

  if (token != 0xFE) {
    this->delegate_->end_transaction();
    this->deselect();
    ESP_LOGE(TAG, "Invalid data token: 0x%02X", token);
    return false;
  }

  // Read data
  this->delegate_->read_array(buffer, BLOCK_SIZE);

  // Read 2-byte CRC (and discard)
  uint8_t crc[2];
  this->delegate_->read_array(crc, 2);

  this->delegate_->end_transaction();
  this->deselect();

  return true;
}

bool SdSpiRaw::read_blocks(uint32_t lba, uint8_t *buffer, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (!this->read_block(lba + i, buffer + (i * BLOCK_SIZE))) {
      return false;
    }
  }
  return true;
}

bool SdSpiRaw::write_block(uint32_t lba, const uint8_t *buffer) {
  if (!this->initialized_ || buffer == nullptr) {
    return false;
  }

  // For SDSC cards, address is byte address; for SDHC, it's block address
  uint32_t addr = this->is_high_capacity() ? lba : (lba * BLOCK_SIZE);

  this->select();
  uint8_t r1 = this->send_cmd(CMD24, addr);
  
  if (r1 != 0x00) {
    this->deselect();
    ESP_LOGE(TAG, "CMD24 failed: 0x%02X", r1);
    return false;
  }

  this->delegate_->begin_transaction();

  // Send data token
  uint8_t token = 0xFE;
  this->delegate_->write_array(&token, 1);

  // Send data
  this->delegate_->write_array(buffer, BLOCK_SIZE);

  // Send dummy CRC
  uint8_t crc[2] = {0xFF, 0xFF};
  this->delegate_->write_array(crc, 2);

  // Get data response
  uint8_t response;
  this->delegate_->read_array(&response, 1);
  
  this->delegate_->end_transaction();

  // Response should be xxx0 0101 for data accepted
  if ((response & 0x1F) != 0x05) {
    this->deselect();
    ESP_LOGE(TAG, "Write rejected: 0x%02X", response);
    return false;
  }

  // Wait for write to complete (card busy)
  if (!this->wait_ready(500)) {
    this->deselect();
    ESP_LOGE(TAG, "Write timeout");
    return false;
  }

  this->deselect();
  return true;
}

bool SdSpiRaw::write_blocks(uint32_t lba, const uint8_t *buffer, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (!this->write_block(lba + i, buffer + (i * BLOCK_SIZE))) {
      return false;
    }
  }
  return true;
}

}  // namespace sd_storage
}  // namespace esphome
