// Simple compilation test for StorageAdapter
// This file verifies that the StorageAdapter class compiles correctly

#ifdef USE_STORAGE

#include "storage_adapter.h"
#include "vector_eyes.h"

namespace esphome {
namespace vector_eyes {

// Test that we can instantiate the class
void test_storage_adapter_instantiation() {
  VectorEyes *parent = nullptr;
  StorageAdapter adapter(parent);
  
  // Test basic methods exist
  bool available = adapter.is_available();
  (void)available; // Suppress unused warning
  
  std::string path = adapter.get_preferred_mount_path();
  (void)path; // Suppress unused warning
}

// Test that we can call initialization
void test_storage_adapter_initialization() {
  VectorEyes *parent = nullptr;
  StorageAdapter adapter(parent);
  
  storage::Storage *storage = nullptr;
  bool result = adapter.initialize(storage);
  (void)result; // Suppress unused warning
}

// Test file operations
void test_storage_adapter_file_operations() {
  VectorEyes *parent = nullptr;
  StorageAdapter adapter(parent);
  
  std::string path = "/test.json";
  bool exists = adapter.file_exists(path);
  (void)exists;
  
  std::vector<uint8_t> data;
  bool read_result = adapter.read_file(path, data);
  (void)read_result;
  
  std::vector<std::string> animations;
  bool list_result = adapter.list_animations(animations);
  (void)list_result;
}

// Test streaming operations
void test_storage_adapter_streaming() {
  VectorEyes *parent = nullptr;
  StorageAdapter adapter(parent);
  
  std::string path = "/test.wav";
  void *handle = adapter.open_file(path);
  
  if (handle != nullptr) {
    uint8_t buffer[1024];
    size_t bytes_read = adapter.read_chunk(handle, buffer, sizeof(buffer));
    (void)bytes_read;
    
    bool close_result = adapter.close_file(handle);
    (void)close_result;
  }
}

// Test device management
void test_storage_adapter_device_management() {
  VectorEyes *parent = nullptr;
  StorageAdapter adapter(parent);
  
  storage::StorageDevice *device = adapter.get_primary_device();
  (void)device;
  
  adapter.set_preferred_mount_path("/sd");
}

// Test callbacks
void test_storage_adapter_callbacks() {
  VectorEyes *parent = nullptr;
  StorageAdapter adapter(parent);
  
  storage::StorageDevice *device = nullptr;
  adapter.on_device_added(device);
  adapter.on_device_removed(device);
}

}  // namespace vector_eyes
}  // namespace esphome

// Main function for compilation test
int main() {
  esphome::vector_eyes::test_storage_adapter_instantiation();
  esphome::vector_eyes::test_storage_adapter_initialization();
  esphome::vector_eyes::test_storage_adapter_file_operations();
  esphome::vector_eyes::test_storage_adapter_streaming();
  esphome::vector_eyes::test_storage_adapter_device_management();
  esphome::vector_eyes::test_storage_adapter_callbacks();
  return 0;
}

#endif  // USE_STORAGE
