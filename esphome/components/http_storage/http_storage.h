#pragma once

#include "esphome/core/component.h"
#include "esphome/components/storage/storage_device.h"
#include "esphome/core/defines.h"

#ifdef USE_HTTP_STORAGE

#include <vector>
#include <string>

#ifdef ESP32
#include <HTTPClient.h>
#endif

namespace esphome {
namespace http_storage {

class HttpStorage : public Component, public storage::StorageDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_url(const std::string &url);
  void set_mount_path(const std::string &path) { this->mount_path_ = path; }
  void set_auth(const std::string &username, const std::string &password);

  // StorageDevice implementation
  storage::StorageInfo get_info() override;
  bool is_available() override;
  bool supports_filesystem() override { return true; }
  std::string get_mount_path() override { return this->mount_path_; }

  bool file_exists(const char *path) override;
  bool get_file_size(const char *path, size_t *size) override;
  bool read_file(const char *path, uint8_t *data, size_t *length) override;
  bool list_dir(const char *path, std::vector<storage::StorageFileInfo> *entries) override;
  
  // Stubs for unsupported write operations
  bool write_file(const char *path, const uint8_t *data, size_t length) override { return false; }
  bool append_file(const char *path, const uint8_t *data, size_t length) override { return false; }
  bool delete_file(const char *path) override { return false; }
  bool create_dir(const char *path) override { return false; }
  bool delete_dir(const char *path, bool recursive) override { return false; }
  bool rename_file(const char *old_path, const char *new_path) override { return false; }

  // Streaming access
  void *open_file(const char *path, const char *mode) override;
  size_t read_file_chunk(void *handle, uint8_t *buffer, size_t size) override;
  bool close_file(void *handle) override;

 protected:
  std::string base_url_;
  std::string mount_path_;
  std::string username_;
  std::string password_;
  bool has_auth_{false};

  std::string get_url_path(const char *path);
  
  // Helper to perform HTTP request
  // Returns http code (e.g. 200) or negative on error
  int perform_head_request(const std::string &url, size_t *content_length = nullptr);
};

}  // namespace http_storage
}  // namespace esphome

#endif  // USE_HTTP_STORAGE
