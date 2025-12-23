#include "http_storage.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

#ifdef USE_HTTP_STORAGE

#include <ArduinoJson.h>

namespace esphome {
namespace http_storage {

static const char *const TAG = "http_storage";

struct HttpFileHandle {
  std::string full_url;
  size_t position;
  size_t size;
};

void HttpStorage::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HTTP Storage...");
}

void HttpStorage::dump_config() {
  ESP_LOGCONFIG(TAG, "HTTP Storage:");
  ESP_LOGCONFIG(TAG, "  Base URL: %s", this->base_url_.c_str());
  ESP_LOGCONFIG(TAG, "  Mount Path: %s", this->mount_path_.c_str());
  if (this->has_auth_) {
    ESP_LOGCONFIG(TAG, "  Authentication: Enabled");
  }
}

void HttpStorage::set_url(const std::string &url) {
  this->base_url_ = url;
  if (!this->base_url_.empty() && this->base_url_.back() != '/') {
    this->base_url_ += '/';
  }
}

void HttpStorage::set_auth(const std::string &username, const std::string &password) {
  this->username_ = username;
  this->password_ = password;
  this->has_auth_ = true;
}

storage::StorageInfo HttpStorage::get_info() {
  storage::StorageInfo info;
  info.id = "http_storage";
  info.name = "HTTP Storage";
  info.type = storage::StorageType::NETWORK;
  info.filesystem = storage::FilesystemType::NONE; // Virtual
  info.mount_path = this->mount_path_;
  info.is_mounted = network::is_connected();
  info.is_read_only = true;
  return info;
}

bool HttpStorage::is_available() {
  return network::is_connected();
}

std::string HttpStorage::get_url_path(const char *path) {
  std::string s_path = path;
  
  // Strip mount path prefix if present
  if (s_path.find(this->mount_path_) == 0) {
    s_path = s_path.substr(this->mount_path_.length());
  }
  
  // Ensure no leading slash for appending to base_url which has trailing slash
  if (!s_path.empty() && s_path[0] == '/') {
    s_path = s_path.substr(1);
  }
  
  return this->base_url_ + s_path;
}

int HttpStorage::perform_head_request(const std::string &url, size_t *content_length) {
#ifdef ESP32
  HTTPClient http;
  http.begin(url.c_str());
  if (this->has_auth_) {
    http.setAuthorization(this->username_.c_str(), this->password_.c_str());
  }
  
  const char *headerNames[] = {"Content-Length"};
  http.collectHeaders(headerNames, 1);
  
  // Use GET with Range: bytes=0-0 effectively or just HEAD if server supports it.
  // Standard HTTPClient uses GET. Let's try native HEAD if possible, but 
  // ESP32 HTTPClient sendRequest can do HEAD.
  int code = http.sendRequest("HEAD");
  
  if (code == 200 && content_length != nullptr) {
    if (http.hasHeader("Content-Length")) {
      *content_length = http.header("Content-Length").toInt();
    }
  }
  
  http.end();
  return code;
#else
  return -1;
#endif
}

bool HttpStorage::file_exists(const char *path) {
  std::string url = this->get_url_path(path);
  int code = this->perform_head_request(url);
  ESP_LOGV(TAG, "HEAD %s -> %d", url.c_str(), code);
  return code == 200;
}

bool HttpStorage::get_file_size(const char *path, size_t *size) {
  std::string url = this->get_url_path(path);
  return this->perform_head_request(url, size) == 200;
}

bool HttpStorage::read_file(const char *path, uint8_t *data, size_t *length) {
  std::string url = this->get_url_path(path);
#ifdef ESP32
  HTTPClient http;
  http.begin(url.c_str());
  if (this->has_auth_) {
    http.setAuthorization(this->username_.c_str(), this->password_.c_str());
  }
  
  int code = http.GET();
  if (code != 200) {
    ESP_LOGW(TAG, "GET %s failed: %d", url.c_str(), code);
    http.end();
    return false;
  }
  
  size_t size = http.getSize();
  if (size > *length) {
     ESP_LOGW(TAG, "Buffer too small for %s: %d > %d", url.c_str(), size, *length);
     // Still read what we can? No, typically fail or resize if it was a vector (but here it is raw pointer)
     // Actually storage interface passes *length as in/out. 
     // We should respect input length.
     size = *length; 
  }
  
  // Read stream
  WiFiClient *stream = http.getStreamPtr();
  size_t bytes_read = 0;
  while (http.connected() && (size > 0 || size == -1) && bytes_read < *length) {
    size_t available = stream->available();
    if (available) {
      size_t to_read = std::min(available, *length - bytes_read);
      int c = stream->readBytes(data + bytes_read, to_read);
      if (c > 0) {
        bytes_read += c;
        if (size != -1) size -= c;
      }
    } else {
      delay(1);
    }
  }
  
  *length = bytes_read;
  http.end();
  return true;
#else
  return false;
#endif
}

bool HttpStorage::list_dir(const char *path, std::vector<storage::StorageFileInfo> *entries) {
  // Try to find manifest.json in the directory
  std::string dir_url = this->get_url_path(path);
  if (dir_url.back() != '/') dir_url += '/';
  
  std::string manifest_url = dir_url + "manifest.json";
  
#ifdef ESP32
  HTTPClient http;
  http.begin(manifest_url.c_str());
  if (this->has_auth_) {
    http.setAuthorization(this->username_.c_str(), this->password_.c_str());
  }
  
  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    // Parse JSON
    // Format expected: [{"name": "file.json", "size": 123, "is_dir": false}, ...]
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error && doc.is<JsonArray>()) {
      for (JsonObject item : doc.as<JsonArray>()) {
        storage::StorageFileInfo info;
        info.name = item["name"].as<std::string>();
        std::string p = path;
        if (p.back() != '/') p += '/';
        info.path = p + info.name;
        info.size = item["size"] | 0;
        info.is_directory = item["is_dir"] | false;
        entries->push_back(info);
      }
      http.end();
      return true;
    } else {
        ESP_LOGw(TAG, "Failed to parse manifest.json: %s", error.c_str());
    }
  } else {
     // If manifest missing, maybe try file_list.json? Or just log warning.
     ESP_LOGD(TAG, "No manifest.json found at %s (%d)", manifest_url.c_str(), code);
  }
  http.end();
#endif
  
  return false;
}

// Streaming
void *HttpStorage::open_file(const char *path, const char *mode) {
  if (strcmp(mode, "r") != 0) return nullptr; // Only read supported
  
  size_t size = 0;
  if (!this->get_file_size(path, &size)) {
    return nullptr;
  }
  
  HttpFileHandle *handle = new HttpFileHandle();
  handle->full_url = this->get_url_path(path);
  handle->position = 0;
  handle->size = size;
  
  return (void*)handle;
}

size_t HttpStorage::read_file_chunk(void *handle_ptr, uint8_t *buffer, size_t size) {
  HttpFileHandle *handle = (HttpFileHandle*)handle_ptr;
  if (!handle) return 0;
  
#ifdef ESP32
  HTTPClient http;
  http.begin(handle->full_url.c_str());
  if (this->has_auth_) {
    http.setAuthorization(this->username_.c_str(), this->password_.c_str());
  }
  
  // Use Range header to fetch specific chunk
  char range_header[64];
  size_t end_pos = handle->position + size - 1;
  snprintf(range_header, sizeof(range_header), "bytes=%u-%u", handle->position, end_pos);
  http.addHeader("Range", range_header);
  
  const char *headerNames[] = {"Content-Range"};
  http.collectHeaders(headerNames, 1);
  
  int code = http.GET();
  // 206 Partial Content is expected, but some servers might send 200 (full file) if they don't support range
  // processing 200 is harder here as we'd need to skip.
  
  size_t bytes_read = 0;
  if (code == 206) {
      WiFiClient *stream = http.getStreamPtr();
      
      // Read up to 'size' bytes
      size_t available = http.getSize(); // Should match requested range size approx
      size_t to_read = std::min(size, available);
      
      // stream->readBytes is blocking with timeout
      bytes_read = stream->readBytes(buffer, to_read);
      
      handle->position += bytes_read;
  } else if (code == 200) {
      // Server ignored Range, sent whole file. 
      // This is inefficient but we can skip.
      WiFiClient *stream = http.getStreamPtr();
      
      // Skip to position
      size_t to_skip = handle->position;
      while (to_skip > 0) {
          size_t skip_now = std::min((size_t)256, to_skip);
          uint8_t skip_buf[256];
          stream->readBytes(skip_buf, skip_now);
          to_skip -= skip_now;
      }
      
      bytes_read = stream->readBytes(buffer, size);
      handle->position += bytes_read;
  } else {
      ESP_LOGW(TAG, "Chunk read failed %d", code);
  }
  
  http.end();
  return bytes_read;
#else
  return 0;
#endif
}

bool HttpStorage::close_file(void *handle_ptr) {
  HttpFileHandle *handle = (HttpFileHandle*)handle_ptr;
  if (handle) {
    delete handle;
    return true;
  }
  return false;
}

}  // namespace http_storage
}  // namespace esphome

#endif  // USE_HTTP_STORAGE
