#include "storage_adapter.h"

#ifdef USE_STORAGE

#include "vector_eyes.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <algorithm>
#include <map>

// VFS includes
#include <cstdio>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

namespace esphome {
namespace vector_eyes {

static const char *const TAG = "vector_eyes.storage_adapter";

// Animation directory path (relative to mount point)
// static const char *const ANIMATIONS_DIR = "/assets"; // Now using animations_dir_ member
// static const char *const AUDIO_MAPPINGS_FILE = "/assets/cladToFileMaps/AnimationTriggerMap.json";
// (We build paths dynamically now)

// Retry configuration constants
static constexpr int DEFAULT_MAX_RETRIES = 3;
static constexpr int RETRY_DELAY_MS = 100;  // Initial delay in milliseconds

StorageAdapter::StorageAdapter(VectorEyes *parent) : parent_(parent) {
  initialize_buffer_pools();
}

StorageAdapter::~StorageAdapter() {
  close_all_handles();
  cleanup_buffer_pools();
}

//========================================================================
// Initialization
//========================================================================

bool StorageAdapter::initialize(storage::Storage *storage_component) {
  if (storage_component == nullptr) {
    ESP_LOGW(TAG, "Storage component is null, storage adapter will not be available");
    return false;
  }
  
  storage_ = storage_component;
  
  // Try to find a suitable storage device
  primary_device_ = find_device_with_animations();
  
  if (primary_device_ != nullptr) {
    log_device_info(primary_device_);
    ESP_LOGI(TAG, "Storage adapter initialized successfully");
    initialized_ = true;
    return true;
  } else if (using_direct_vfs_) {
    ESP_LOGI(TAG, "Storage adapter initialized (Direct VFS)");
    initialized_ = true;
    return true;
  } else {
    ESP_LOGW(TAG, "No storage device with animations found, will fall back to internal animations");
    initialized_ = false;
    return false;
  }
}

bool StorageAdapter::is_available() {
  if (initialized_) {
      if (using_direct_vfs_) return true;
      if (primary_device_ != nullptr && primary_device_->is_available()) return true;
  }

  // Lazy initialization: Check if a new device appeared (e.g. late mounting network storage)
  // Throttle checks to once per second
  uint32_t now = millis();
  if (storage_ != nullptr && (now - last_check_ms_ > 1000)) {
    last_check_ms_ = now;
    
    // Don't spam logs unless debugging
    ESP_LOGI(TAG, "Re-checking for storage device with animations... (Storage component set)");
    
    storage::StorageDevice *device = find_device_with_animations();
    if (device != nullptr) {
      primary_device_ = device;
      log_device_info(primary_device_);
      ESP_LOGI(TAG, "Storage adapter lazily initialized with device: %s", device->get_info().id.c_str());
      initialized_ = true;
      return true;
    } else if (using_direct_vfs_) {
       ESP_LOGI(TAG, "Storage adapter lazily initialized (Direct VFS)");
       initialized_ = true;
       return true;
    } else {
        ESP_LOGW(TAG, "No device with animations found during lazy init");
    }
  } else if (storage_ == nullptr) {
      static uint32_t last_null_log = 0;
      if (now - last_null_log > 5000) {
          ESP_LOGW(TAG, "Storage component is NULL in is_available check");
          last_null_log = now;
      }
  }

  return false;
}

//========================================================================
// File Operations
//========================================================================

bool StorageAdapter::file_exists(const std::string &path) {
  if (!is_available()) {
    return false;
  }
  
  std::string full_path = build_full_path(path);

  // VFS Direct Mode
  if (using_direct_vfs_) {
      struct stat st;
      bool exists = (stat(full_path.c_str(), &st) == 0);
      if (exists) ESP_LOGD(TAG, "File found via VFS: %s", full_path.c_str());
      return exists;
  }
  
  // Try primary device first with retry logic
  bool exists = retry_operation([this, &full_path]() {
    return primary_device_->file_exists(full_path.c_str());
  }, "file_exists (primary device)", DEFAULT_MAX_RETRIES);
  
  if (exists) {
    return true;
  }
  
  // Try all other available devices as fallback (with retry for each)
  if (storage_ != nullptr) {
    auto devices = storage_->get_all_devices();
    for (auto *device : devices) {
      if (device != primary_device_ && validate_device(device)) {
        exists = retry_operation([device, &full_path]() {
          return device->file_exists(full_path.c_str());
        }, "file_exists (alternative device)", DEFAULT_MAX_RETRIES);
        
        if (exists) {
          ESP_LOGD(TAG, "File found on alternative device: %s", full_path.c_str());
          return true;
        }
      }
    }
  }
  
  return false;
}

bool StorageAdapter::read_file(const std::string &path, std::vector<uint8_t> &data) {
  uint32_t start_ms = millis();
  
  if (!is_available()) {
    ESP_LOGW(TAG, "Storage not available, cannot read file: %s", path.c_str());
    return false;
  }
  
  std::string full_path = build_full_path(path);
  
  // Try primary device first with retry logic
  bool success = retry_operation([this, &full_path, &data]() {
    return try_read_from_device(primary_device_, full_path, data);
  }, "read_file (primary device)", DEFAULT_MAX_RETRIES);
  
  if (success) {
    uint32_t duration_ms = millis() - start_ms;
    if (duration_ms > SLOW_OPERATION_THRESHOLD_MS) {
      ESP_LOGW(TAG, "Slow read_file operation: %s took %d ms", path.c_str(), duration_ms);
    }
    return true;
  }
  
  ESP_LOGW(TAG, "Failed to read from primary device after retries, trying alternatives");
  
  // Try all available devices as fallback (with retry for each)
  if (storage_ != nullptr) {
    auto devices = storage_->get_all_devices();
    for (auto *device : devices) {
      if (device != primary_device_ && validate_device(device)) {
        success = retry_operation([this, device, &full_path, &data]() {
          return try_read_from_device(device, full_path, data);
        }, "read_file (alternative device)", DEFAULT_MAX_RETRIES);
        
        if (success) {
          uint32_t duration_ms = millis() - start_ms;
          ESP_LOGI(TAG, "Successfully read from alternative device: %s (took %d ms)", 
                   device->get_info().id.c_str(), duration_ms);
          if (duration_ms > SLOW_OPERATION_THRESHOLD_MS) {
            ESP_LOGW(TAG, "Slow read_file operation with fallback: %s took %d ms", path.c_str(), duration_ms);
          }
          return true;
        }
      }
    }
  }
  
  uint32_t duration_ms = millis() - start_ms;
  ESP_LOGE(TAG, "Failed to read file from any storage device: %s (took %d ms)", full_path.c_str(), duration_ms);
  return false;
}

bool StorageAdapter::list_animations(std::vector<std::string> &animation_names) {
  uint32_t start_ms = millis();
  
  if (!is_available()) {
    ESP_LOGW(TAG, "Storage not available, cannot list animations");
    return false;
  }
  
  std::string animations_path = build_full_path(this->animations_dir_);
  std::vector<storage::StorageFileInfo> entries;
  
  // Try to list directory with retry logic
  bool success = retry_operation([this, &animations_path, &entries]() {
    return primary_device_->list_dir(animations_path.c_str(), &entries);
  }, "list_animations", DEFAULT_MAX_RETRIES);
  
  if (!success) {
    uint32_t duration_ms = millis() - start_ms;
    ESP_LOGW(TAG, "Failed to list animations directory after retries: %s (took %d ms)", 
             animations_path.c_str(), duration_ms);
    return false;
  }
  
  animation_names.clear();
  for (const auto &entry : entries) {
    // Only include .json files
    if (!entry.is_directory && entry.name.length() > 5) {
      std::string ext = entry.name.substr(entry.name.length() - 5);
      if (ext == ".json") {
        animation_names.push_back(entry.name);
      }
    }
  }
  
  uint32_t duration_ms = millis() - start_ms;
  ESP_LOGD(TAG, "Found %d animation files (took %d ms)", animation_names.size(), duration_ms);
  if (duration_ms > SLOW_OPERATION_THRESHOLD_MS) {
    ESP_LOGW(TAG, "Slow list_animations operation took %d ms", duration_ms);
  }
  return true;
}

bool StorageAdapter::list_files_recursive(const std::string &path, std::vector<std::string> &file_paths, const std::string &extension) {
  uint32_t start_ms = millis();
  
  if (!is_available()) {
    return false;
  }
  
  std::string full_path = build_full_path(path);
  
  // Use a lambda for the recursive operation to support retry top-level
  auto list_op = [this, &full_path, &file_paths, &extension]() {
    file_paths.clear();
    std::vector<std::string> dirs_to_visit = {full_path};
    
    while (!dirs_to_visit.empty()) {
      std::string current_dir = dirs_to_visit.back();
      dirs_to_visit.pop_back();
      
      std::vector<storage::StorageFileInfo> entries;
      if (!primary_device_->list_dir(current_dir.c_str(), &entries)) {
        ESP_LOGW(TAG, "Failed to list dir: %s", current_dir.c_str());
        continue;
      }

      
      for (const auto &entry : entries) {
        std::string entry_path = current_dir + "/" + entry.name;
        // Clean up double slashes just in case
        if (current_dir.length() == 1 && current_dir[0] == '/') {
             entry_path = "/" + entry.name;
        }

        if (entry.is_directory) {
          dirs_to_visit.push_back(entry_path);
        } else {
          if (extension.empty() || 
              (entry.name.length() >= extension.length() && 
               entry.name.compare(entry.name.length() - extension.length(), extension.length(), extension) == 0)) {
            // Store as relative path to mount point (remove mount prefix if needed? 
            // Wait, build_full_path adds slash. We want paths we can pass to read_file.
            // primary_device file methods take device-relative paths. 
            // entry_path here is what we passed to list_dir, which was built with build_full_path.
            // So it should be fine.
            file_paths.push_back(entry_path);
          }
        }
      }
    }
    return true;
  };

  bool success = retry_operation(list_op, "list_files_recursive", DEFAULT_MAX_RETRIES);
  
  uint32_t duration_ms = millis() - start_ms;
  ESP_LOGD(TAG, "Listed %d files in %s (recursive) took %d ms", file_paths.size(), path.c_str(), duration_ms);
  
  return success;

}

//========================================================================
// Streaming Operations
//========================================================================

void *StorageAdapter::open_file(const std::string &path) {
  uint32_t start_ms = millis();
  
  if (!is_available()) {
    ESP_LOGW(TAG, "Storage not available, cannot open file: %s", path.c_str());
    return nullptr;
  }
  
  std::string full_path = build_full_path(path);
  void *handle = nullptr;
  
  if (using_direct_vfs_) {
      FILE *f = fopen(full_path.c_str(), "rb");
      if (f) {
           ESP_LOGD(TAG, "Opened file via VFS: %s", full_path.c_str());
           track_file_handle((void*)f, nullptr, full_path);
           return (void*)f;
      }
      ESP_LOGW(TAG, "Failed to open file via VFS: %s", full_path.c_str());
      return nullptr;
  }
  
  // Try to open file with retry logic
  retry_operation([this, &full_path, &handle]() {
    handle = primary_device_->open_file(full_path.c_str(), "r");
    return handle != nullptr;
  }, "open_file", DEFAULT_MAX_RETRIES);
  
  uint32_t duration_ms = millis() - start_ms;
  
  if (handle == nullptr) {
    ESP_LOGW(TAG, "Failed to open file after retries: %s (took %d ms)", full_path.c_str(), duration_ms);
  } else {
    ESP_LOGD(TAG, "Opened file for streaming: %s (took %d ms)", full_path.c_str(), duration_ms);
    if (duration_ms > SLOW_OPERATION_THRESHOLD_MS) {
      ESP_LOGW(TAG, "Slow open_file operation took %d ms", duration_ms);
    }
    // Track the opened handle
    track_file_handle(handle, primary_device_, full_path);
  }
  
  return handle;
}

size_t StorageAdapter::read_chunk(void *handle, uint8_t *buffer, size_t size) {
  if (!is_available() || handle == nullptr) {
    return 0;
  }
  
  // Verify handle is tracked
  if (!is_handle_tracked(handle)) {
    ESP_LOGW(TAG, "Attempting to read from untracked file handle");
    return 0;
  }
  
  size_t bytes_read = 0;
  if (using_direct_vfs_) {
      // Direct VFS read
      bytes_read = fread(buffer, 1, size, (FILE*)handle);
  } else {
      bytes_read = primary_device_->read_file_chunk(handle, buffer, size);
  }
  
  // Update position in tracked handle
  if (bytes_read > 0) {
    auto it = open_handles_.find(handle);
    if (it != open_handles_.end()) {
      it->second.position += bytes_read;
    }
  }
  
  return bytes_read;
}

bool StorageAdapter::close_file(void *handle) {
  if (!is_available() || handle == nullptr) {
    return false;
  }
  
  // Verify handle is tracked
  if (!is_handle_tracked(handle)) {
    ESP_LOGW(TAG, "Attempting to close untracked file handle");
    return false;
  }
  
  bool result = false;
  if (using_direct_vfs_) {
      result = (fclose((FILE*)handle) == 0);
  } else {
      result = primary_device_->close_file(handle);
  }
  if (result) {
    ESP_LOGD(TAG, "Closed file handle");
    // Untrack the closed handle
    untrack_file_handle(handle);
  } else {
    ESP_LOGW(TAG, "Failed to close file handle");
  }
  
  return result;
}

//========================================================================
// Device Management
//========================================================================

storage::StorageDevice *StorageAdapter::get_primary_device() {
  return primary_device_;
}

void StorageAdapter::set_preferred_mount_path(const std::string &path) {
  preferred_mount_path_ = path;
  ESP_LOGI(TAG, "Set preferred mount path: %s", path.c_str());
  
  // Re-discover devices with new preference
  if (storage_ != nullptr) {
    primary_device_ = find_device_with_animations();
    if (primary_device_ != nullptr) {
      log_device_info(primary_device_);
    }
  }
}

//========================================================================
// Event Callbacks
//========================================================================

void StorageAdapter::on_device_added(storage::StorageDevice *device) {
  if (device == nullptr) {
    return;
  }
  
  ESP_LOGI(TAG, "Storage device added: %s", device->get_info().id.c_str());
  log_device_info(device);
  
  // If we don't have a primary device yet, try to use this one
  if (primary_device_ == nullptr && validate_device(device)) {
    // Check if it has animations
    std::string animations_path = this->animations_dir_;
    if (device->dir_exists(animations_path.c_str())) {
      primary_device_ = device;
      initialized_ = true;
      ESP_LOGI(TAG, "Set new device as primary: %s", device->get_info().id.c_str());
    }
  }
}

void StorageAdapter::on_device_removed(storage::StorageDevice *device) {
  if (device == nullptr) {
    return;
  }
  
  ESP_LOGI(TAG, "Storage device removed: %s", device->get_info().id.c_str());
  
  // Close any open handles on this device
  std::vector<void*> handles_to_close;
  for (const auto &pair : open_handles_) {
    if (pair.second.device == device) {
      handles_to_close.push_back(pair.first);
    }
  }
  
  for (void *handle : handles_to_close) {
    ESP_LOGW(TAG, "Force closing handle due to device removal: %s", 
             open_handles_[handle].path.c_str());
    untrack_file_handle(handle);
  }
  
  // If this was our primary device, try to find a replacement
  if (device == primary_device_) {
    ESP_LOGW(TAG, "Primary storage device removed, searching for replacement");
    primary_device_ = nullptr;
    initialized_ = false;
    using_direct_vfs_ = false; // NEW: Reset VFS flag
    
    if (storage_ != nullptr) {
      primary_device_ = find_device_with_animations();
      if (primary_device_ != nullptr) {
        initialized_ = true;
        ESP_LOGI(TAG, "Found replacement device: %s", primary_device_->get_info().id.c_str());
      } else {
        ESP_LOGW(TAG, "No replacement device found, falling back to internal animations");
      }
    }
  }
}

//========================================================================
// Internal Helper Methods
//========================================================================

storage::StorageDevice *StorageAdapter::find_device_with_animations() {
  if (storage_ == nullptr) {
    return nullptr;
  }
  
  auto devices = storage_->get_all_devices();
  
  // First, try to find device matching preferred mount path 
  // OR check if the path is accessible via VFS directly (e.g. NFS mount)
  if (!preferred_mount_path_.empty()) {

    // VFS Direct Check
    std::string test_file_vfs = preferred_mount_path_ + this->animations_dir_ + "/cladToFileMaps/AnimationTriggerMap.json";
    
    struct stat st;
    ESP_LOGI(TAG, "Checking VFS path: %s", test_file_vfs.c_str());
    if (stat(test_file_vfs.c_str(), &st) == 0) {
        ESP_LOGI(TAG, "Found animations via Direct VFS at: %s%s", preferred_mount_path_.c_str(), this->animations_dir_.c_str());
        this->using_direct_vfs_ = true;
        return nullptr;
    } else {
        ESP_LOGW(TAG, "VFS path failed. Errno: %d", errno);
    }
    
    // Fallback: Check root if assets folder not found
    std::string test_file_vfs_root = preferred_mount_path_ + "/cladToFileMaps/AnimationTriggerMap.json";
    ESP_LOGI(TAG, "Checking VFS ROOT path: %s", test_file_vfs_root.c_str());
    if (stat(test_file_vfs_root.c_str(), &st) == 0) {
        ESP_LOGI(TAG, "Found animations via Direct VFS at ROOT: %s", preferred_mount_path_.c_str());
        this->using_direct_vfs_ = true;
        this->animations_dir_ = ""; // Set root
        return nullptr;
    } else {
         ESP_LOGW(TAG, "VFS ROOT path failed. Errno: %d", errno);
    }

    for (auto *device : devices) {
      if (validate_device(device)) {
        if (device->get_info().mount_path == preferred_mount_path_) {
          std::string animations_path = this->animations_dir_;
          ESP_LOGI(TAG, "Checking for animations at: %s", animations_path.c_str());
          
          // WORKAROUND: list_dir() and dir_exists() fail on ESP32 SD cards
          // Instead, check for a known animation file to verify directory exists
          // We check for the Trigger Map which is essential
          std::string test_file = this->animations_dir_ + "/cladToFileMaps/AnimationTriggerMap.json";
          ESP_LOGI(TAG, "Testing for specific file: %s", test_file.c_str());
          
          if (device->file_exists(test_file.c_str())) {
            ESP_LOGI(TAG, "Found preferred device with animations at %s (verified via file check)", 
                     preferred_mount_path_.c_str());
            return device;
          } else {
             // Fallback: Check root
             std::string test_file_root = "/cladToFileMaps/AnimationTriggerMap.json";
             ESP_LOGI(TAG, "Testing for specific file at ROOT: %s", test_file_root.c_str());
             
             if (device->file_exists(test_file_root.c_str())) {
                 ESP_LOGI(TAG, "Found preferred device with animations at ROOT of %s", preferred_mount_path_.c_str());
                 this->animations_dir_ = "";
                 return device;
             }
          }
        }
      }
    }
    ESP_LOGD(TAG, "Preferred mount path not found or has no animations, trying all devices");
  }
  
  // Try all available devices
  for (auto *device : devices) {
    if (validate_device(device)) {
      std::string mount_path = device->get_info().mount_path;
      if (!mount_path.empty()) {
        std::string animations_path = this->animations_dir_;
        ESP_LOGI(TAG, "Checking device %s for animations at: %s", 
                 device->get_info().id.c_str(), animations_path.c_str());
        
        // WORKAROUND: list_dir() and dir_exists() fail on ESP32 SD cards
        // Instead, check for a known animation file to verify directory exists
        std::string test_file = this->animations_dir_ + "/cladToFileMaps/AnimationTriggerMap.json";
        ESP_LOGI(TAG, "Testing for specific file: %s", test_file.c_str());
        
        if (device->file_exists(test_file.c_str())) {
          ESP_LOGI(TAG, "Found device with animations: %s at %s (verified via file check)", 
                   device->get_info().id.c_str(), mount_path.c_str());
          return device;
        } else {
           // Fallback Check Root
           std::string test_file_root = "/cladToFileMaps/AnimationTriggerMap.json";
           if (device->file_exists(test_file_root.c_str())) {
               ESP_LOGI(TAG, "Found device with animations at ROOT: %s", device->get_info().id.c_str());
               this->animations_dir_ = "";
               return device;
           }
        }
      }
    }
  }
  
  ESP_LOGD(TAG, "No storage device with animations directory found");
  return nullptr;
}

bool StorageAdapter::validate_device(storage::StorageDevice *device) {
  if (using_direct_vfs_ && device == nullptr) return true;
  if (device == nullptr) {
    return false;
  }
  
  if (!device->is_available()) {
    return false;
  }
  
  if (!device->supports_filesystem()) {
    return false;
  }
  
  return true;
}

bool StorageAdapter::try_read_from_device(storage::StorageDevice *device, 
                                          const std::string &path,
                                          std::vector<uint8_t> &data) {
  if (!validate_device(device)) {
    return false;
  }

  // VFS Direct Read
  if (using_direct_vfs_ && device == nullptr) {
      FILE *f = fopen(path.c_str(), "rb");
      if (!f) return false;
      
      fseek(f, 0, SEEK_END);
      long size = ftell(f);
      rewind(f);
      
      if (size <= 0) { fclose(f); return false; }
      
      data.resize(size);
      size_t read = fread(data.data(), 1, size, f);
      fclose(f);
      return read == (size_t)size;
  }
  
  // Get file size first
  size_t file_size = 0;
  if (!device->get_file_size(path.c_str(), &file_size)) {
    ESP_LOGD(TAG, "Could not get file size: %s", path.c_str());
    return false;
  }
  
  if (file_size == 0) {
    ESP_LOGW(TAG, "File is empty: %s", path.c_str());
    data.clear();
    return true;  // Empty file is valid
  }
  
  // Check if file should be streamed instead of loaded entirely
  if (should_stream_file(file_size)) {
    ESP_LOGW(TAG, "File %s is large (%d bytes), consider using streaming API", 
             path.c_str(), file_size);
    // Continue anyway, but log the warning
  }
  
  // Enforce memory limits before allocation
  if (!enforce_memory_limit(file_size)) {
    ESP_LOGE(TAG, "Cannot allocate %d bytes for file: %s (memory limit exceeded)", 
             file_size, path.c_str());
    return false;
  }
  
  // Try to use buffer pool for small files
  uint8_t *buffer = acquire_buffer(file_size);
  bool using_pool = (buffer != nullptr);
  
  if (!using_pool) {
    // Allocate directly if pool is unavailable or file is too large
    data.resize(file_size);
    buffer = data.data();
  }
  
  // Read file
  size_t bytes_read = file_size;
  if (!device->read_file(path.c_str(), buffer, &bytes_read)) {
    ESP_LOGW(TAG, "Failed to read file: %s", path.c_str());
    if (using_pool) {
      release_buffer(buffer);
    } else {
      data.clear();
    }
    return false;
  }
  
  // If we used the pool, copy data to output vector
  if (using_pool) {
    data.assign(buffer, buffer + bytes_read);
    release_buffer(buffer);
  } else {
    // Adjust size if less was read
    if (bytes_read < file_size) {
      data.resize(bytes_read);
    }
  }
  
  // Update cached bytes tracking
  // FIX: Caller takes ownership of data, so it's not "cached" in StorageAdapter
  // total_cached_bytes_ += bytes_read;
  
  ESP_LOGD(TAG, "Successfully read %d bytes from: %s (using pool: %s, total cached: %d)", 
           bytes_read, path.c_str(), using_pool ? "yes" : "no", total_cached_bytes_);
  return true;
}

std::string StorageAdapter::build_full_path(const std::string &relative_path) {
  // Pass the path directly to the device (device-relative)
  // Ensure we have a leading slash for consistency
  if (!relative_path.empty() && relative_path[0] != '/') {
    return "/" + relative_path;
  }
  return relative_path;
}

void StorageAdapter::log_device_info(storage::StorageDevice *device) {
  if (device == nullptr) {
    return;
  }
  
  auto info = device->get_info();
  ESP_LOGI(TAG, "Device: %s", info.id.c_str());
  ESP_LOGI(TAG, "  Name: %s", info.name.c_str());
  ESP_LOGI(TAG, "  Mount: %s", info.mount_path.c_str());
  ESP_LOGI(TAG, "  Type: %d", static_cast<int>(info.type));
  ESP_LOGI(TAG, "  Available: %s", info.is_mounted ? "yes" : "no");
  ESP_LOGI(TAG, "  Capacity: %llu bytes", info.total_bytes);
  ESP_LOGI(TAG, "  Free: %llu bytes", info.free_bytes);
}

//========================================================================
// File Handle Lifecycle Management
//========================================================================

void StorageAdapter::track_file_handle(void *native_handle, storage::StorageDevice *device, 
                                       const std::string &path) {
  if (native_handle == nullptr) {
    return;
  }
  
  FileHandle fh;
  fh.native_handle = native_handle;
  fh.device = device;
  fh.path = path;
  fh.position = 0;
  fh.open_time_ms = millis();
  
  // Get file size if possible
  if (device != nullptr) {
    size_t file_size = 0;
    if (device->get_file_size(path.c_str(), &file_size)) {
      fh.size = file_size;
    }
  }
  
  open_handles_[native_handle] = fh;
  ESP_LOGD(TAG, "Tracking file handle: %s (total open: %d)", path.c_str(), open_handles_.size());
}

void StorageAdapter::untrack_file_handle(void *native_handle) {
  if (native_handle == nullptr) {
    return;
  }
  
  auto it = open_handles_.find(native_handle);
  if (it != open_handles_.end()) {
    uint32_t duration_ms = millis() - it->second.open_time_ms;
    ESP_LOGD(TAG, "Untracking file handle: %s (open for %d ms, read %d/%d bytes)", 
             it->second.path.c_str(), duration_ms, it->second.position, it->second.size);
    open_handles_.erase(it);
  }
}

bool StorageAdapter::is_handle_tracked(void *native_handle) const {
  return open_handles_.find(native_handle) != open_handles_.end();
}

void StorageAdapter::close_all_handles() {
  if (open_handles_.empty()) {
    return;
  }
  
  ESP_LOGW(TAG, "Closing %d open file handles", open_handles_.size());
  
  // Close all tracked handles
  for (auto &pair : open_handles_) {
    void *handle = pair.first;
    FileHandle &fh = pair.second;
    
    if (fh.device != nullptr && fh.device->is_available()) {
      ESP_LOGD(TAG, "Force closing: %s", fh.path.c_str());
      fh.device->close_file(handle);
    }
  }
  
  open_handles_.clear();
}

//========================================================================
// Memory Management (Task 10)
//========================================================================

void StorageAdapter::initialize_buffer_pools() {
  buffer_pools_.resize(BUFFER_POOL_SIZE);
  for (auto &pool : buffer_pools_) {
    pool.buffer.reserve(BUFFER_SIZE);
    pool.in_use = false;
    pool.last_used_ms = 0;
  }
  ESP_LOGD(TAG, "Initialized %d buffer pools of %d bytes each", BUFFER_POOL_SIZE, BUFFER_SIZE);
}

void StorageAdapter::cleanup_buffer_pools() {
  for (auto &pool : buffer_pools_) {
    pool.buffer.clear();
    pool.buffer.shrink_to_fit();
    pool.in_use = false;
  }
  buffer_pools_.clear();
  ESP_LOGD(TAG, "Cleaned up buffer pools");
}

uint8_t *StorageAdapter::acquire_buffer(size_t size) {
  // If size is larger than our pool buffers, we can't use pooling
  if (size > BUFFER_SIZE) {
    ESP_LOGD(TAG, "Requested buffer size %d exceeds pool size, allocating directly", size);
    return nullptr;  // Caller should allocate directly
  }
  
  // Try to find an available buffer in the pool
  for (auto &pool : buffer_pools_) {
    if (!pool.in_use) {
      pool.in_use = true;
      pool.last_used_ms = millis();
      
      // Ensure buffer is large enough
      if (pool.buffer.capacity() < size) {
        pool.buffer.reserve(size);
      }
      pool.buffer.resize(size);
      
      ESP_LOGV(TAG, "Acquired buffer from pool (%d bytes)", size);
      return pool.buffer.data();
    }
  }
  
  ESP_LOGD(TAG, "No available buffers in pool, all %d buffers in use", BUFFER_POOL_SIZE);
  return nullptr;  // No available buffers
}

void StorageAdapter::release_buffer(uint8_t *buffer) {
  if (buffer == nullptr) {
    return;
  }
  
  // Find the buffer in our pools
  for (auto &pool : buffer_pools_) {
    if (pool.buffer.data() == buffer) {
      pool.in_use = false;
      pool.last_used_ms = millis();
      ESP_LOGV(TAG, "Released buffer back to pool");
      return;
    }
  }
  
  ESP_LOGW(TAG, "Attempted to release buffer not from pool");
}

bool StorageAdapter::should_stream_file(size_t file_size) const {
  return file_size > LARGE_FILE_THRESHOLD;
}

bool StorageAdapter::enforce_memory_limit(size_t bytes_needed) {
  // Check if we're within limits
  if (total_cached_bytes_ + bytes_needed <= MAX_CACHED_DATA) {
    return true;  // We have enough space
  }
  
  ESP_LOGW(TAG, "Memory limit approaching: %d bytes cached, need %d more (limit: %d)",
           total_cached_bytes_, bytes_needed, MAX_CACHED_DATA);
  
  // For now, we don't have a cache to evict from
  // In a future enhancement, we could cache parsed animations and evict them here
  // For now, just check if the allocation would exceed limits
  if (bytes_needed > MAX_CACHED_DATA) {
    ESP_LOGE(TAG, "Single allocation of %d bytes exceeds memory limit of %d bytes",
             bytes_needed, MAX_CACHED_DATA);
    return false;
  }
  
  // If we get here, we need to free some memory but don't have a cache yet
  // Log a warning and allow the allocation (better than failing completely)
  ESP_LOGW(TAG, "Memory limit exceeded but no cache to evict, allowing allocation");
  return true;
}

}  // namespace vector_eyes
}  // namespace esphome

#endif  // USE_STORAGE
