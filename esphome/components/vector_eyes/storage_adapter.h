#pragma once

#include "esphome/core/component.h"

// Only include storage headers if storage component is available
#ifdef USE_STORAGE
#include "esphome/components/storage/storage.h"
#include "esphome/components/storage/storage_device.h"
#else
// Forward declarations when storage component is not available
namespace esphome {
namespace storage {
class Storage;
class StorageDevice;
}  // namespace storage
}  // namespace esphome
#endif

#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

// ESP-IDF includes for task watchdog
#ifdef USE_ESP_IDF
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#endif

namespace esphome {
namespace vector_eyes {

// Forward declaration
class VectorEyes;

#ifdef USE_STORAGE

/**
 * FileHandle - Wrapper for tracking open file handles
 * 
 * This structure tracks metadata about open files to ensure proper
 * lifecycle management and debugging.
 */
struct FileHandle {
  void *native_handle;                    // Native handle from StorageDevice
  storage::StorageDevice *device;         // Device that owns this handle
  std::string path;                       // Full path to the file
  size_t size;                            // File size in bytes
  size_t position;                        // Current read position
  uint32_t open_time_ms;                  // Timestamp when file was opened
  
  FileHandle() : native_handle(nullptr), device(nullptr), size(0), position(0), open_time_ms(0) {}
};

/**
 * StorageAdapter - Abstraction layer for storage operations in vector_eyes
 * 
 * This class provides a unified interface for accessing animation and audio files
 * from various storage backends (SD card, USB, network storage) through the
 * ESPHome storage component.
 */
class StorageAdapter {
 public:
  explicit StorageAdapter(VectorEyes *parent);
  ~StorageAdapter();
  
  //========================================================================
  // Initialization
  //========================================================================
  
  /// Initialize the storage adapter with a storage component
  /// @param storage_component Pointer to the storage component instance
  /// @return true if initialization succeeded
  bool initialize(storage::Storage *storage_component);
  
  /// Check if storage is available and initialized
  /// @return true if storage is ready for use
  bool is_available();
  
  //========================================================================
  // File Operations
  //========================================================================
  
  /// Check if a file exists on any available storage device
  /// @param path File path (relative to mount point)
  /// @return true if file exists
  bool file_exists(const std::string &path);
  
  /// Read entire file into a buffer
  /// @param path File path (relative to mount point)
  /// @param data Output buffer for file contents
  /// @return true if file was read successfully
  bool read_file(const std::string &path, std::vector<uint8_t> &data);
  
  /// List all animation files in the animations directory
  /// @param animation_names Output vector of animation filenames
  /// @return true if listing succeeded
  bool list_animations(std::vector<std::string> &animation_names);
  
  //========================================================================
  // Streaming Operations (for large files like audio)
  //========================================================================
  
  /// Open a file for streaming access
  /// @param path File path (relative to mount point)
  /// @return File handle (nullptr on failure)
  void *open_file(const std::string &path);
  
  /// Read a chunk from an open file
  /// @param handle File handle from open_file()
  /// @param buffer Buffer to store data
  /// @param size Maximum bytes to read
  /// @return Number of bytes actually read
  size_t read_chunk(void *handle, uint8_t *buffer, size_t size);
  
  /// Close an open file handle
  /// @param handle File handle from open_file()
  /// @return true if close succeeded
  bool close_file(void *handle);
  
  //========================================================================
  // Device Management
  //========================================================================
  
  /// Get the primary storage device being used
  /// @return Pointer to primary device (nullptr if none available)
  storage::StorageDevice *get_primary_device();
  
  /// Set preferred mount path for finding animations
  /// @param path Mount path (e.g., "/sd", "/usb")
  void set_preferred_mount_path(const std::string &path);
  
  /// Get the current preferred mount path
  /// @return Preferred mount path string
  const std::string &get_preferred_mount_path() const { return preferred_mount_path_; }
  
  /// Get memory usage statistics
  /// @return Total bytes currently cached in memory
  size_t get_cached_bytes() const { return total_cached_bytes_; }
  
  /// Get maximum allowed cached bytes
  /// @return Maximum cache size in bytes
  size_t get_max_cached_bytes() const { return MAX_CACHED_DATA; }
  
  //========================================================================
  // Event Callbacks (for hot-plug support)
  //========================================================================
  
  /// Called when a new storage device is added
  /// @param device The newly added device
  void on_device_added(storage::StorageDevice *device);
  
  /// Called when a storage device is removed
  /// @param device The removed device
  void on_device_removed(storage::StorageDevice *device);
  
  /// Recursively list all files in a directory
  /// @param path Directory path
  /// @param file_paths Output vector of full file paths (relative to mount)
  /// @param extension Optional filter by extension (e.g. ".json")
  /// @return true if successful
  bool list_files_recursive(const std::string &path, std::vector<std::string> &file_paths, const std::string &extension = "");

  
 protected:
  VectorEyes *parent_;                              // Parent component
  storage::Storage *storage_{nullptr};              // Storage component reference
  storage::StorageDevice *primary_device_{nullptr}; // Currently selected device
  std::string preferred_mount_path_;                // Preferred mount path
  std::string animations_dir_{"/assets"};           // Path to animations directory (can be updated to "")
  bool initialized_{false};                         // Initialization state
  bool using_direct_vfs_{false};                    // NEW: Use direct VFS access
  uint32_t last_check_ms_{0};                       // Last time we checked for devices
  
  // File handle lifecycle management
  std::map<void*, FileHandle> open_handles_;        // Track all open file handles
  uint32_t next_handle_id_{1};                      // Counter for handle IDs
  
  //========================================================================
  // Internal Helper Methods
  //========================================================================
  
  /// Find a storage device that contains animation files
  /// @return Pointer to device with animations (nullptr if none found)
  storage::StorageDevice *find_device_with_animations();
  
  /// Validate that a device is suitable for use
  /// @param device Device to validate
  /// @return true if device is valid and available
  bool validate_device(storage::StorageDevice *device);
  
  /// Try to read a file from a specific device
  /// @param device Device to read from
  /// @param path File path
  /// @param data Output buffer
  /// @return true if read succeeded
  bool try_read_from_device(storage::StorageDevice *device, const std::string &path, 
                           std::vector<uint8_t> &data);
  
  /// Build full path by combining mount path and relative path
  /// @param relative_path Relative file path
  /// @return Full path including mount point
  std::string build_full_path(const std::string &relative_path);
  
  /// Log storage device information for debugging
  /// @param device Device to log
  void log_device_info(storage::StorageDevice *device);
  
  /// Track an opened file handle
  /// @param native_handle Native handle from StorageDevice
  /// @param device Device that owns the handle
  /// @param path Full path to the file
  void track_file_handle(void *native_handle, storage::StorageDevice *device, const std::string &path);
  
  /// Untrack a closed file handle
  /// @param native_handle Native handle to untrack
  void untrack_file_handle(void *native_handle);
  
  /// Check if a handle is currently tracked
  /// @param native_handle Handle to check
  /// @return true if handle is tracked
  bool is_handle_tracked(void *native_handle) const;
  
  /// Get the number of currently open handles
  /// @return Number of open handles
  size_t get_open_handle_count() const { return open_handles_.size(); }
  
  /// Close all open file handles (cleanup)
  void close_all_handles();
  
  /// Retry a file operation with exponential backoff
  /// @param operation Lambda function to retry
  /// @param operation_name Name of operation for logging
  /// @param max_retries Maximum number of retry attempts
  /// @return true if operation succeeded within retry limit
  template<typename Func>
  bool retry_operation(Func operation, const char *operation_name, int max_retries = 3) {
    int attempt = 0;
    int delay_ms = RETRY_DELAY_MS;
    
    while (attempt < max_retries) {
      attempt++;
      
      // Try the operation
      if (operation()) {
        if (attempt > 1) {
          ESP_LOGI("vector_eyes.storage_adapter", "%s succeeded on attempt %d", operation_name, attempt);
        }
        return true;
      }
      
      // If this was the last attempt, give up
      if (attempt >= max_retries) {
        ESP_LOGE("vector_eyes.storage_adapter", "%s failed after %d attempts", operation_name, max_retries);
        return false;
      }
      
      // Log retry and wait before next attempt
      ESP_LOGW("vector_eyes.storage_adapter", "%s failed (attempt %d/%d), retrying in %d ms...", 
               operation_name, attempt, max_retries, delay_ms);
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
      
      // Exponential backoff (double the delay each time, up to 1 second)
      delay_ms = std::min(delay_ms * 2, 1000);
    }
    
    return false;
  }
  
  // Retry configuration
  static constexpr int DEFAULT_MAX_RETRIES = 3;
  static constexpr int RETRY_DELAY_MS = 100;  // Initial delay
  
  //========================================================================
  // Memory Management (Task 10)
  //========================================================================
  
  /// Buffer pool for file reads to reduce allocations
  struct BufferPool {
    std::vector<uint8_t> buffer;
    bool in_use{false};
    uint32_t last_used_ms{0};
  };
  
  std::vector<BufferPool> buffer_pools_;
  static constexpr size_t BUFFER_POOL_SIZE = 2;        // Number of pooled buffers
  static constexpr size_t BUFFER_SIZE = 4096;          // 4KB buffers
  static constexpr size_t LARGE_FILE_THRESHOLD = 10240; // 10KB threshold for streaming
  static constexpr size_t MAX_CACHED_DATA = 51200;     // 50KB max cached data
  static constexpr uint32_t SLOW_OPERATION_THRESHOLD_MS = 100; // Log if operation takes > 100ms
  
  size_t total_cached_bytes_{0};  // Track total cached memory
  
  /// Get a buffer from the pool or allocate a new one
  /// @param size Minimum buffer size needed
  /// @return Pointer to buffer (nullptr if allocation fails)
  uint8_t *acquire_buffer(size_t size);
  
  /// Return a buffer to the pool
  /// @param buffer Pointer to buffer to release
  void release_buffer(uint8_t *buffer);
  
  /// Check if we should use streaming for a file
  /// @param file_size Size of the file in bytes
  /// @return true if file should be streamed
  bool should_stream_file(size_t file_size) const;
  
  /// Enforce memory limits by evicting cached data
  /// @param bytes_needed Number of bytes we need to allocate
  /// @return true if enough memory was freed
  bool enforce_memory_limit(size_t bytes_needed);
  
  /// Initialize buffer pools
  void initialize_buffer_pools();
  
  /// Clean up buffer pools
  void cleanup_buffer_pools();
};

/**
 * StorageAdapterStream - Stream wrapper for StorageAdapter
 * Allows reading from storage using Arduino Stream-like interface (duck typing for ArduinoJson)
 */
class StorageAdapterStream {
public:
    static const size_t BUFFER_SIZE = 512;

    StorageAdapterStream(StorageAdapter *adapter, const std::string &path) : adapter_(adapter) {
        handle_ = adapter_->open_file(path);
    }
    ~StorageAdapterStream() {
        if (handle_) adapter_->close_file(handle_);
    }

    bool isOpen() const { return handle_ != nullptr; }

    int available() { return isOpen() ? 1 : 0; }
    
    int read() {
        if (buffer_pos_ >= buffer_available_) {
            if (!fillBuffer()) return -1;
        }
        return buffer_[buffer_pos_++];
    }
    int peek() { return -1; }
    
    size_t readBytes(char *user_buf, size_t length) {
        size_t bytes_read = 0;
        while (length > 0) {
            if (buffer_pos_ >= buffer_available_) {
                if (!fillBuffer()) break;
            }
            size_t available = buffer_available_ - buffer_pos_;
            size_t to_copy = std::min(available, length);
            memcpy(user_buf, &buffer_[buffer_pos_], to_copy);
            buffer_pos_ += to_copy;
            user_buf += to_copy;
            length -= to_copy;
            bytes_read += to_copy;
        }
        return bytes_read;
    }
    size_t write(uint8_t) { return 0; }
    void flush() {}
    
private:
    bool fillBuffer() {
        if (!isOpen()) return false;
        buffer_pos_ = 0;
        buffer_available_ = adapter_->read_chunk(handle_, buffer_, BUFFER_SIZE);
        
        // Feed the task watchdog and yield to prevent timeout during long reads
        fill_count_++;
        if (fill_count_ % 8 == 0) {  // Every 8 buffer fills (~4KB)
#ifdef USE_ESP_IDF
            esp_task_wdt_reset();
            vTaskDelay(1);  // Give other tasks a chance to run
#endif
        }
        
        return buffer_available_ > 0;
    }
    
    size_t fill_count_{0};

    StorageAdapter *adapter_;
    void *handle_{nullptr};
    uint8_t buffer_[BUFFER_SIZE];
    size_t buffer_pos_{0};
    size_t buffer_available_{0};
};


#endif  // USE_STORAGE

}  // namespace vector_eyes
}  // namespace esphome
