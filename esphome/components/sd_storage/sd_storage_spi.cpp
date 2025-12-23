#include "sd_storage_spi.h"

#ifdef USE_SD_STORAGE_SPI

#include "esphome/core/log.h"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <dirent.h>

extern "C" {
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
}
#include "ff.h"

#ifdef USE_STORAGE
#include "esphome/components/storage/storage.h"
#endif

#ifndef VFS_FAT_MOUNT_DEFAULT_CONFIG
#define VFS_FAT_MOUNT_DEFAULT_CONFIG() \
  { .format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 0, .disk_status_check_enable = false, }
#endif  // VFS_FAT_MOUNT_DEFAULT_CONFIG

// SD OCR SDHC capability bit
int constexpr SD_OCR_SDHC_CAP = (1 << 30);

namespace esphome {
namespace sd_storage {

static constexpr size_t FILE_PATH_MAX = ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN;

std::string SdSpi::build_full_path(const char *path) {
  std::string full_path = this->mount_path_;
  if (path[0] != '/') {
    full_path += "/";
  }
  full_path += path;
  return full_path;
}

void SdSpi::setup() {
  ESP_LOGI(TAG_SPI, "Initializing SD card in SPI mode");

  // Setup DATA1/DATA2 pins as pullup inputs if specified
  auto setup_input_pullup = [](GPIOPin *pin) {
    pin->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
    pin->setup();
  };
  if (this->data1_pin_ != nullptr)
    setup_input_pullup(this->data1_pin_);
  if (this->data2_pin_ != nullptr)
    setup_input_pullup(this->data2_pin_);

  // Initialize SPI bus
  // Hack: Temporarily set cs_ to nullptr so SPIDevice registers with NullPin
  // This prevents the SPIDelegate from automatically managing the real CS pin
  GPIOPin *real_cs = this->cs_;
  this->cs_ = nullptr;
  this->spi_setup();
  this->cs_ = real_cs;

  if (!this->mount_card()) {
    ESP_LOGE(TAG_SPI, "Failed to mount SD card");
    this->mark_failed();
    return;
  }
}

void SdSpi::loop() {
  // Nothing to do in loop
}

void SdSpi::dump_config() {
  ESP_LOGCONFIG(TAG_SPI, "SD Storage (SPI):");
  ESP_LOGCONFIG(TAG_SPI, "  Mounted: %s", this->is_mounted_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG_SPI, "  Mount path: %s", this->mount_path_.c_str());
  ESP_LOGCONFIG(TAG_SPI, "  Mode 1 bit: %s", YESNO(this->mode_1bit_));
  ESP_LOGCONFIG(TAG_SPI, "  CS Pin: %d", spi::Utility::get_pin_no(this->cs_));

  if (this->is_mounted_) {
    ESP_LOGCONFIG(TAG_SPI, "  Card Type: %d", static_cast<uint8_t>(this->card_type_));
    ESP_LOGCONFIG(TAG_SPI, "  Total bytes: %" PRIu64, this->total_bytes_);
    ESP_LOGCONFIG(TAG_SPI, "  Used bytes: %" PRIu64, this->used_bytes_);
  }

  if (this->is_failed()) {
    ESP_LOGE(TAG_SPI, "Setup failed: %s", SdSpi::error_code_to_string(this->init_error_).c_str());
  }
}

std::string SdSpi::error_code_to_string(SdSpi::ErrorCode code) {
  switch (code) {
    case ErrorCode::ERR_MOUNT:
      return "Failed to mount card";
    case ErrorCode::ERR_NO_CARD:
      return "No card found";
    default:
      return "Unknown error";
  }
}

bool SdSpi::mount_card() {
  ESP_LOGI(TAG_SPI, "Mounting SD card via Raw SPI (FatFs)");

  // 1. Initialize SD card via raw SPI driver
  // Use configure frequency, defaulting to 400kHz for init
  if (!this->raw_driver_.init(this->delegate_, this->cs_, 400)) {
    ESP_LOGE(TAG_SPI, "Raw SD init failed");
    this->init_error_ = ErrorCode::ERR_MOUNT;
    return false;
  }
  
  // 2. Register driver with ESP-IDF diskio layer and set global pointer
  fatfs_set_sd_driver(&this->raw_driver_);
  fatfs_register_raw_driver(0);  // Register as drive 0
  
  // 3. Mount filesystem
  FRESULT res = f_mount(&this->fatfs_, "", 1);
  if (res != FR_OK) {
    ESP_LOGE(TAG_SPI, "f_mount failed: %d", res);
    this->init_error_ = ErrorCode::ERR_MOUNT;
    return false;
  }
  
  this->is_mounted_ = true;
  this->card_type_ = this->raw_driver_.get_card_type();
  
  this->update_card_info();

  ESP_LOGI(TAG_SPI, "SD card mounted successfully via Raw SPI/FatFs");
  
  // Notify callbacks
  for (const auto &callback : this->mount_ready_callbacks_) {
    callback(this->mount_path_);
  }

#ifdef USE_STORAGE
  // Register with storage registry
  if (storage::global_storage != nullptr) {
    storage::global_storage->register_device(this);
  }
#endif

  return true;
}

void SdSpi::unmount_card() {
  if (this->is_mounted_) {
#ifdef USE_STORAGE
    // Unregister from storage registry if available
    if (storage::global_storage != nullptr) {
      storage::global_storage->unregister_device(this);
    }
#endif

    f_mount(NULL, "", 0);  // Unmount
    this->is_mounted_ = false;
    ESP_LOGI(TAG_SPI, "SD card unmounted");
  }
}

bool SdSpi::update_card_info() {
  if (!this->is_mounted_) return false;

  this->total_bytes_ = this->raw_driver_.get_sector_count() * 512;
  
  uint64_t total, free_bytes;
  if (this->get_space_info(&total, &free_bytes)) {
    this->used_bytes_ = total - free_bytes;
  }
  return true;
}

uint64_t SdSpi::get_free_bytes() const {
  if (!this->is_mounted_)
    return 0;

  FATFS *fs;
  DWORD fre_clust;

  FRESULT res = f_getfree("", &fre_clust, &fs);
  if (res != FR_OK) return 0;

  DWORD fre_sect = fre_clust * fs->csize;
  return (uint64_t) fre_sect * fs->ssize;
}

// Original string-based methods
bool SdSpi::write_file(const std::string &path, const std::string &data) {
  if (!this->is_mounted_) {
    ESP_LOGW(TAG_SPI, "Card not mounted, cannot write file");
    return false;
  }

  FIL fp;
  FRESULT res = f_open(&fp, path.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
  if (res != FR_OK) {
    ESP_LOGW(TAG_SPI, "Failed to open file for writing: %s (res: %d)", path.c_str(), res);
    return false;
  }

  UINT written;
  res = f_write(&fp, data.c_str(), data.length(), &written);
  f_close(&fp);

  if (written != data.length()) {
    ESP_LOGW(TAG_SPI, "Failed to write all data to file");
    return false;
  }

  ESP_LOGD(TAG_SPI, "Wrote %d bytes to %s", (int)data.length(), path.c_str());
  return true;
}

bool SdSpi::append_file(const std::string &path, const std::string &data) {
  if (!this->is_mounted_) {
    ESP_LOGW(TAG_SPI, "Card not mounted, cannot append to file");
    return false;
  }

  FIL fp;
  FRESULT res = f_open(&fp, path.c_str(), FA_WRITE | FA_OPEN_APPEND);
  if (res != FR_OK) {
    ESP_LOGW(TAG_SPI, "Failed to open file for appending: %s (res: %d)", path.c_str(), res);
    return false;
  }

  UINT written;
  res = f_write(&fp, data.c_str(), data.length(), &written);
  f_close(&fp);

  if (written != data.length()) {
    ESP_LOGW(TAG_SPI, "Failed to append all data to file");
    return false;
  }

  ESP_LOGD(TAG_SPI, "Appended %d bytes to %s", (int)data.length(), path.c_str());
  return true;
}

std::string SdSpi::read_file(const std::string &path) {
  if (!this->is_mounted_) {
    ESP_LOGW(TAG_SPI, "Card not mounted, cannot read file");
    return "";
  }

  FIL fp;
  FRESULT res = f_open(&fp, path.c_str(), FA_READ);
  if (res != FR_OK) {
    ESP_LOGW(TAG_SPI, "Failed to open file for reading: %s (res: %d)", path.c_str(), res);
    return "";
  }

  // Get file size
  FILINFO fno;
  f_stat(path.c_str(), &fno);
  size_t file_size = fno.fsize;

  // Read file content
  std::string content;
  content.resize(file_size);
  UINT bytes_read;
  res = f_read(&fp, &content[0], file_size, &bytes_read);
  f_close(&fp);

  if (bytes_read != file_size) {
    ESP_LOGW(TAG_SPI, "Failed to read entire file");
    return "";
  }

  ESP_LOGD(TAG_SPI, "Read %d bytes from %s", bytes_read, path.c_str());
  return content;
}

bool SdSpi::delete_file(const std::string &path) {
  if (!this->is_mounted_) {
    ESP_LOGW(TAG_SPI, "Card not mounted, cannot delete file");
    return false;
  }

  if (f_unlink(path.c_str()) == FR_OK) {
    ESP_LOGD(TAG_SPI, "Deleted file: %s", path.c_str());
    return true;
  } else {
    ESP_LOGW(TAG_SPI, "Failed to delete file: %s", path.c_str());
    return false;
  }
}

bool SdSpi::create_directory(const std::string &path) {
  if (!this->is_mounted_) {
    ESP_LOGW(TAG_SPI, "Card not mounted, cannot create directory");
    return false;
  }

  if (f_mkdir(path.c_str()) == FR_OK) {
    ESP_LOGD(TAG_SPI, "Created directory: %s", path.c_str());
    return true;
  } else {
    ESP_LOGW(TAG_SPI, "Failed to create directory: %s", path.c_str());
    return false;
  }
}

bool SdSpi::remove_directory(const std::string &path) {
  if (!this->is_mounted_) {
    ESP_LOGW(TAG_SPI, "Card not mounted, cannot remove directory");
    return false;
  }

  if (f_unlink(path.c_str()) == FR_OK) { // f_unlink removes dirs too if empty
    ESP_LOGD(TAG_SPI, "Removed directory: %s", path.c_str());
    return true;
  } else {
    ESP_LOGW(TAG_SPI, "Failed to remove directory: %s", path.c_str());
    return false;
  }
}

bool SdSpi::is_directory(const std::string &path) {
  if (!this->is_mounted_) {
    return false;
  }

  FILINFO fno;
  if (f_stat(path.c_str(), &fno) != FR_OK) {
    return false;
  }
  return fno.fattrib & AM_DIR;
}

uint32_t SdSpi::file_size(const std::string &path) {
  if (!this->is_mounted_) {
    return 0;
  }

  FILINFO fno;
  if (f_stat(path.c_str(), &fno) != FR_OK) {
    return 0;
  }
  return fno.fsize;
}

std::vector<FileInfo> SdSpi::list_directory(const std::string &path) {
  std::vector<FileInfo> files;

  if (!this->is_mounted_) {
    ESP_LOGW(TAG_SPI, "Card not mounted, cannot list directory");
    return files;
  }

  if (!this->is_mounted_) return files;

  FF_DIR dir;
  FILINFO fno;
  FRESULT res = f_opendir(&dir, path.c_str());
  
  if (res != FR_OK) {
    ESP_LOGW(TAG_SPI, "Failed to open directory: %s (res: %d)", path.c_str(), res);
    return files;
  }

  while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0) {
    if (fno.fname[0] == '.') continue;
    
    FileInfo info;
    info.path = fno.fname;
    info.is_directory = fno.fattrib & AM_DIR;
    info.size = fno.fsize;
    
    files.push_back(info);
  }
  
  f_closedir(&dir);
  ESP_LOGD(TAG_SPI, "Listed %d entries in %s", files.size(), path.c_str());
  return files;
}

#ifdef USE_STORAGE
//========================================================================
// StorageDevice Interface Implementation
//========================================================================

storage::StorageInfo SdSpi::get_info() {
  storage::StorageInfo info;
  info.id = this->id_.empty() ? "sd_storage_spi" : this->id_;
  info.name = "SD Card (SPI)";
  info.type = storage::StorageType::SD_CARD;
  info.filesystem = storage::FilesystemType::FAT;
  info.mount_path = this->mount_path_;
  info.total_bytes = this->total_bytes_;
  info.free_bytes = this->get_free_bytes();
  info.block_size = 512;  // Standard SD card sector size
  info.is_mounted = this->is_mounted_;
  info.is_removable = true;
  info.is_read_only = false;
  info.supports_raw_access = false;
  info.supports_filesystem = true;
  return info;
}

bool SdSpi::file_exists(const char *path) {
  if (!this->is_mounted_)
    return false;

  FILINFO fno;
  return f_stat(path, &fno) == FR_OK && !(fno.fattrib & AM_DIR);
}

bool SdSpi::get_file_size(const char *path, size_t *size) {
  if (!this->is_mounted_)
    return false;

  FILINFO fno;
  if (f_stat(path, &fno) != FR_OK) return false;
  *size = fno.fsize;
  return true;
}

bool SdSpi::read_file(const char *path, uint8_t *data, size_t *length) {
  if (!this->is_mounted_)
    return false;

  FIL fp;
  if (f_open(&fp, path, FA_READ) != FR_OK) return false;
  UINT bytes_read;
  f_read(&fp, data, *length, &bytes_read);
  f_close(&fp);
  *length = bytes_read;
  return true;
}

bool SdSpi::write_file(const char *path, const uint8_t *data, size_t length) {
  if (!this->is_mounted_)
    return false;

  FIL fp;
  if (f_open(&fp, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) return false;
  UINT written;
  f_write(&fp, data, length, &written);
  f_close(&fp);
  return written == length;
}

bool SdSpi::append_file(const char *path, const uint8_t *data, size_t length) {
  if (!this->is_mounted_)
    return false;

  FIL fp;
  if (f_open(&fp, path, FA_WRITE | FA_OPEN_APPEND) != FR_OK) return false;
  UINT written;
  f_write(&fp, data, length, &written);
  f_close(&fp);
  return written == length;
}

bool SdSpi::delete_file(const char *path) {
  if (!this->is_mounted_)
    return false;

  return f_unlink(path) == FR_OK;
}

bool SdSpi::rename_file(const char *old_path, const char *new_path) {
  if (!this->is_mounted_)
    return false;

  return f_rename(old_path, new_path) == FR_OK;
}

bool SdSpi::copy_file(const char *src_path, const char *dst_path) {
  if (!this->is_mounted_)
    return false;

  FIL src, dst;
  if (f_open(&src, src_path, FA_READ) != FR_OK) return false;
  if (f_open(&dst, dst_path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
    f_close(&src);
    return false;
  }

  uint8_t buffer[512];
  UINT bytes_read, written;
  bool success = true;

  while (f_read(&src, buffer, sizeof(buffer), &bytes_read) == FR_OK && bytes_read > 0) {
    if (f_write(&dst, buffer, bytes_read, &written) != FR_OK || written != bytes_read) {
      success = false;
      break;
    }
  }

  f_close(&src);
  f_close(&dst);
  return success;
}

bool SdSpi::dir_exists(const char *path) {
  if (!this->is_mounted_)
    return false;

  FILINFO fno;
  return f_stat(path, &fno) == FR_OK && (fno.fattrib & AM_DIR);
}

bool SdSpi::create_dir(const char *path) {
  if (!this->is_mounted_)
    return false;

  return f_mkdir(path) == FR_OK;
}

bool SdSpi::delete_dir(const char *path, bool recursive) {
  if (!this->is_mounted_)
    return false;

  std::string full_path = this->build_full_path(path);

  if (recursive) {
    // Recursive delete logic with FatFs
    FF_DIR dir;
    FILINFO fno;
    if (f_opendir(&dir, path) != FR_OK) return false;

    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0) {
      if (fno.fname[0] == '.') continue;
      
      std::string entry_path = std::string(path) + "/" + fno.fname;
      if (fno.fattrib & AM_DIR) {
          if (!this->delete_dir(entry_path.c_str(), true)) { f_closedir(&dir); return false; }
      } else {
          if (f_unlink(entry_path.c_str()) != FR_OK) { f_closedir(&dir); return false; }
      }
    }
    f_closedir(&dir);
  }
  return f_unlink(path) == FR_OK;
}

bool SdSpi::list_dir(const char *path, std::vector<storage::StorageFileInfo> *entries) {
  if (!this->is_mounted_)
    return false;

  std::string full_path = this->build_full_path(path);
  DIR *dir = opendir(full_path.c_str());
  if (dir == nullptr)
    return false;

  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
      continue;

    storage::StorageFileInfo info;
    info.name = entry->d_name;
    info.path = std::string(path) + "/" + entry->d_name;
    info.is_directory = entry->d_type == DT_DIR;

    // Get file size and modification time
    std::string entry_full_path = full_path + "/" + entry->d_name;
    struct stat st;
    if (stat(entry_full_path.c_str(), &st) == 0) {
      info.size = info.is_directory ? 0 : st.st_size;
      info.modified_time = st.st_mtime;
    } else {
      info.size = 0;
      info.modified_time = 0;
    }

    entries->push_back(info);
  }

  closedir(dir);
  return true;
}

bool SdSpi::get_space_info(uint64_t *total, uint64_t *free) {
  if (!this->is_mounted_)
    return false;

  FATFS *fs;
  DWORD fre_clust;

  // Get volume information and free clusters
  // Need to append "/" to mount path for f_getfree
  // No need to append / for f_getfree with empty path (which means root in 0:)
  FRESULT res = f_getfree("", &fre_clust, &fs);
  if (res != FR_OK) {
    ESP_LOGW(TAG_SPI, "Failed to get filesystem info: %d", res);
    return false;
  }

  // Calculate total and free bytes
  // Cast to uint64_t before multiplication to avoid overflow on large cards
  DWORD tot_sect = (fs->n_fatent - 2) * fs->csize;
  DWORD fre_sect = fre_clust * fs->csize;

  // Sector size is typically 512 bytes for SD cards
  *total = (uint64_t) tot_sect * fs->ssize;
  *free = (uint64_t) fre_sect * fs->ssize;
  return true;
}

bool SdSpi::can_write_file(const char *path, size_t size) {
  if (!this->is_mounted_)
    return false;

  uint64_t total, free_space;
  if (!this->get_space_info(&total, &free_space))
    return false;

  return free_space >= size;
}

void *SdSpi::open_file(const char *path, const char *mode) {
  if (!this->is_mounted_)
    return nullptr;

  FIL *fp = new FIL();
  BYTE fatfs_mode = FA_READ;
  if (strcmp(mode, "w") == 0 || strcmp(mode, "wb") == 0) {
    fatfs_mode = FA_WRITE | FA_CREATE_ALWAYS;
  } else if (strcmp(mode, "a") == 0 || strcmp(mode, "ab") == 0) {
    fatfs_mode = FA_WRITE | FA_OPEN_APPEND;
  }
  if (f_open(fp, path, fatfs_mode) != FR_OK) {
      delete fp;
      return nullptr;
  }
  return fp;
}

size_t SdSpi::read_file_chunk(void *handle, uint8_t *buffer, size_t size) {
  if (handle == nullptr) return 0;
  UINT bytes_read;
  f_read(static_cast<FIL*>(handle), buffer, size, &bytes_read);
  return bytes_read;
}

size_t SdSpi::write_file_chunk(void *handle, const uint8_t *data, size_t size) {
  if (handle == nullptr) return 0;
  UINT written;
  f_write(static_cast<FIL*>(handle), data, size, &written);
  return written;
}

bool SdSpi::seek_file(void *handle, size_t offset) {
  if (handle == nullptr)
    return false;
  return f_lseek(static_cast<FIL*>(handle), offset) == FR_OK;
}

size_t SdSpi::tell_file(void *handle) {
  if (handle == nullptr)
    return 0;
  return f_tell(static_cast<FIL*>(handle));
}

bool SdSpi::close_file(void *handle) {
  if (handle == nullptr)
    return false;
  FRESULT res = f_close(static_cast<FIL*>(handle));
  delete static_cast<FIL*>(handle);
  return res == FR_OK;
}

bool SdSpi::format() {
  // Not implemented - would need to unmount, format, and remount
  ESP_LOGW(TAG_SPI, "Format not implemented for SD cards");
  return false;
}

bool SdSpi::sync() {
  // FAT filesystem syncs on file close
  return true;
}

#endif  // USE_STORAGE

}  // namespace sd_storage
}  // namespace esphome

#endif  // USE_SD_STORAGE_SPI
