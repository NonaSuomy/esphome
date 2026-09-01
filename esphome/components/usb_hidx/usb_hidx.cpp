#include "usb_hidx.h"
#include "esphome/core/log.h"
#include "driver_registry.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>

#ifdef USB_HIDX_ENABLE_XBOX360
#include "devices/xbox360/xbox360_driver.h"
#endif
#ifdef USB_HIDX_ENABLE_PLAYSTATION
#include "devices/playstation/playstation_driver.h"
#endif
#ifdef USB_HIDX_ENABLE_SWITCH
#include "devices/switch/switch_driver.h"
#endif

#ifdef USB_HIDX_ENABLE_MCE_REMOTE
#include "devices/mce_remote/mce_remote_driver.h"
#endif

namespace esphome {
namespace usb_hidx {

static const char *TAG = "usb_hidx";

static constexpr uint8_t MAX_TRANSFER_RETRIES = 5;
static constexpr uint32_t TRANSFER_RETRY_BASE_MS = 25;

static uint32_t transfer_retry_delay(uint8_t failures) {
  // Keep the host responsive while preventing a failed TT transfer from
  // becoming a tight submit/log loop. The last retry is delayed by 400 ms.
  const uint8_t shift = failures > 4 ? 4 : failures;
  return TRANSFER_RETRY_BASE_MS << shift;
}

static const char *usb_speed_name(usb_speed_t speed) {
  switch (speed) {
    case USB_SPEED_LOW:
      return "LS";
    case USB_SPEED_FULL:
      return "FS";
    case USB_SPEED_HIGH:
      return "HS";
    default:
      return "?";
  }
}

#ifdef USE_TEXT_SENSOR
static std::string device_display_name(const HIDDevice *device) {
  if (device == nullptr) {
    return "Unknown";
  }
  char identity[64];
  snprintf(identity, sizeof(identity), "%04X:%04X @%u %s", device->vid, device->pid, device->dev_addr,
           usb_speed_name(device->speed));
  std::string result = device->driver ? device->driver->get_name() : "HID";
  result += " ";
  result += identity;
  return result;
}

static std::string combined_device_names(const std::vector<std::unique_ptr<HIDDevice>> &devices) {
  std::string combined;
  for (const auto &record : devices) {
    if (record->active) {
      if (!combined.empty()) {
        combined += "+";
      }
      combined += device_display_name(record.get());
    }
  }
  return combined;
}

static std::string combined_device_speeds(const std::vector<std::unique_ptr<HIDDevice>> &devices) {
  std::string combined;
  for (const auto &record : devices) {
    if (record->active) {
      if (!combined.empty()) {
        combined += "+";
      }
      combined += usb_speed_name(record->speed);
    }
  }
  return combined;
}
#endif

void USBHIDXComponent::setup() {
  ESP_LOGI(TAG, "Setting up USB HIDX component");

  // Auto-register all available device drivers
  register_all_drivers(this);
  ESP_LOGI(TAG, "Registered %u device drivers", static_cast<unsigned>(this->drivers_.size()));

  // Register USB client
  usb_host_client_config_t client_config = {.is_synchronous = false,
                                            // One event is needed for each
                                            // hot-plug transition.  This is
                                            // queue depth, not a device limit;
                                            // leave enough room for a burst
                                            // from a populated hub tree.
                                            .max_num_event_msg = 16,
                                            .async = {
                                                .client_event_callback = USBHIDXComponent::client_event_callback,
                                                .callback_arg = this,
                                            }};

  esp_err_t err = usb_host_client_register(&client_config, &this->client_hdl_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register USB client: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  this->client_registered_ = true;
  ESP_LOGI(TAG, "USB HIDX client registered successfully");
}

#ifdef USE_BINARY_SENSOR
void USBHIDXComponent::register_keyboard_key_sensor(binary_sensor::BinarySensor *sensor, uint8_t keycode) {
  keyboard_key_sensors_[keycode] = sensor;
}

void USBHIDXComponent::register_raw_binary_sensor(binary_sensor::BinarySensor *sensor, uint8_t offset, uint8_t mask,
                                                  uint8_t value, bool has_value, uint16_t vid, uint16_t pid) {
  raw_binary_bindings_.push_back({sensor, offset, mask, value, has_value, {vid, pid}});
}
#endif

#ifdef USE_SENSOR
void USBHIDXComponent::register_raw_sensor(sensor::Sensor *sensor, uint8_t offset, uint8_t length, bool is_signed,
                                           float scale, float bias, uint16_t vid, uint16_t pid) {
  raw_sensor_bindings_.push_back({sensor, offset, length, is_signed, scale, bias, {vid, pid}});
}
#endif

void USBHIDXComponent::publish_raw_bindings(HIDDevice *device, const uint8_t *data, size_t len) {
#ifdef USE_BINARY_SENSOR
  for (const auto &binding : raw_binary_bindings_) {
    if (binding.sensor == nullptr || !binding.selector.matches(device) || binding.offset >= len) {
      continue;
    }
    const uint8_t masked = data[binding.offset] & binding.mask;
    const bool active = binding.has_value ? masked == (binding.value & binding.mask) : masked != 0;
    binding.sensor->publish_state(active);
  }
#endif

#ifdef USE_SENSOR
  for (const auto &binding : raw_sensor_bindings_) {
    if (binding.sensor == nullptr || !binding.selector.matches(device) || binding.length == 0 || binding.length > 4 ||
        binding.offset >= len || binding.length > len - binding.offset) {
      continue;
    }

    uint32_t raw = 0;
    for (uint8_t i = 0; i < binding.length; i++) {
      raw |= static_cast<uint32_t>(data[binding.offset + i]) << (8U * i);
    }

    int32_t numeric = static_cast<int32_t>(raw);
    if (binding.is_signed && binding.length < 4 && (raw & (1UL << (binding.length * 8U - 1U)))) {
      numeric -= static_cast<int32_t>(1UL << (binding.length * 8U));
    }
    binding.sensor->publish_state(static_cast<float>(numeric) * binding.scale + binding.bias);
  }
#endif
}

bool USBHIDXComponent::has_raw_bindings_for(const HIDDevice *device) const {
  if (device == nullptr) {
    return false;
  }
#ifdef USE_BINARY_SENSOR
  for (const auto &binding : this->raw_binary_bindings_) {
    if (binding.sensor != nullptr && binding.selector.matches(device)) {
      return true;
    }
  }
#endif
#ifdef USE_SENSOR
  for (const auto &binding : this->raw_sensor_bindings_) {
    if (binding.sensor != nullptr && binding.selector.matches(device)) {
      return true;
    }
  }
#endif
  return false;
}

void USBHIDXComponent::loop() {
  if (this->client_hdl_) {
    esp_err_t err = usb_host_client_handle_events(this->client_hdl_, 0);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "USB client event error: %s", esp_err_to_name(err));
    }
  }

  this->service_transfer_retries();

  // A device-gone callback can arrive before the HCD has completed the
  // canceled transfer callbacks. Retry finalization from the main loop after
  // those callbacks release their transfer objects. Do not use a range-based
  // loop here: try_finalize_device() erases a fully released record.
  for (size_t index = 0; index < this->devices_.size();) {
    HIDDevice *device = this->devices_[index].get();
    if (device->active) {
      index++;
      continue;
    }
    this->try_finalize_device(device);
    // If finalization erased this record, the next record shifted into the
    // same index. Otherwise transfers are still pending, so advance safely.
    if (index < this->devices_.size() && this->devices_[index].get() == device) {
      index++;
    }
  }

#ifdef USE_TEXT_SENSOR
  this->update_resource_status();
#endif
}

#ifdef USE_TEXT_SENSOR
void USBHIDXComponent::update_resource_status() {
  if (this->resource_status_sensor_ == nullptr) {
    return;
  }

  const uint32_t now = esphome::millis();
  if (now - this->resource_status_last_check_ms_ < 250) {
    return;
  }
  this->resource_status_last_check_ms_ = now;

  int allocated = 0;
  int total = 0;
  bool exhausted = false;
  std::string status;
  if (usb_dwc_hcd_get_channel_status == nullptr || !usb_dwc_hcd_get_channel_status(&allocated, &total, &exhausted)) {
    status = "USB: host starting";
  } else if (exhausted) {
    char text[80];
    snprintf(text, sizeof(text), "USB: %d/%d - DEVICE REJECTED", allocated, total);
    status = text;
  } else if (total > 0 && allocated >= total) {
    char text[64];
    snprintf(text, sizeof(text), "USB: %d/%d FULL", allocated, total);
    status = text;
  } else {
    char text[64];
    snprintf(text, sizeof(text), "USB: %d/%d channels", allocated, total);
    status = text;
  }

  if (status != this->last_resource_status_) {
    this->last_resource_status_ = status;
    this->resource_status_sensor_->publish_state(status);
  }
}
#endif

HIDDevice *USBHIDXComponent::find_device_by_handle(usb_device_handle_t dev_hdl) {
  for (const auto &record : devices_) {
    if (record->dev_hdl == dev_hdl) {
      return record.get();
    }
  }
  return nullptr;
}

HIDDevice *USBHIDXComponent::find_device_by_transfer(usb_transfer_t *transfer) {
  for (const auto &record : devices_) {
    if (record->transfer == transfer || record->media_transfer == transfer ||
        std::find(record->output_transfers.begin(), record->output_transfers.end(), transfer) !=
            record->output_transfers.end()) {
      return record.get();
    }
  }
  return nullptr;
}

void USBHIDXComponent::track_output_transfer(HIDDevice *device, usb_transfer_t *transfer) {
  if (device != nullptr && transfer != nullptr) {
    device->output_transfers.push_back(transfer);
  }
}

void USBHIDXComponent::untrack_output_transfer(HIDDevice *device, usb_transfer_t *transfer) {
  if (device == nullptr || transfer == nullptr) {
    return;
  }
  auto it = std::find(device->output_transfers.begin(), device->output_transfers.end(), transfer);
  if (it != device->output_transfers.end()) {
    device->output_transfers.erase(it);
  }
}

void USBHIDXComponent::service_transfer_retries() {
  const uint32_t now = esphome::millis();
  std::vector<HIDDevice *> devices_to_remove;

  auto retry = [this, now, &devices_to_remove](HIDDevice *device, usb_transfer_t *&slot, bool &pending,
                                               uint8_t &failures, uint32_t &retry_at, const char *label,
                                               bool retire_device) {
    if (!pending || slot == nullptr || device == nullptr || !device->active ||
        static_cast<int32_t>(now - retry_at) < 0) {
      return;
    }

    if (slot->status == USB_TRANSFER_STATUS_STALL) {
      const esp_err_t clear_err = usb_host_endpoint_clear(device->dev_hdl, slot->bEndpointAddress);
      if (clear_err != ESP_OK && clear_err != ESP_ERR_NOT_FOUND) {
        failures++;
        if (failures >= MAX_TRANSFER_RETRIES) {
          ESP_LOGW(TAG, "%s endpoint disabled after %u retries: %s", label, failures, esp_err_to_name(clear_err));
          usb_host_transfer_free(slot);
          slot = nullptr;
          pending = false;
          failures = 0;
          if (retire_device) {
            // Finalization must happen after the device-vector iteration.
            devices_to_remove.push_back(device);
          }
          return;
        }
        retry_at = now + transfer_retry_delay(failures);
        ESP_LOGW(TAG, "%s endpoint clear failed: %s (retry %u/%u)", label, esp_err_to_name(clear_err), failures,
                 MAX_TRANSFER_RETRIES);
        return;
      }
    }

    const esp_err_t submit_err = usb_host_transfer_submit(slot);
    if (submit_err == ESP_OK) {
      pending = false;
      failures = 0;
      ESP_LOGD(TAG, "%s transfer resubmitted", label);
      return;
    }

    failures++;
    if (failures >= MAX_TRANSFER_RETRIES) {
      ESP_LOGW(TAG, "%s transfer disabled after %u retries: %s", label, failures, esp_err_to_name(submit_err));
      usb_host_transfer_free(slot);
      slot = nullptr;
      pending = false;
      failures = 0;
      if (retire_device) {
        devices_to_remove.push_back(device);
      }
      return;
    }

    retry_at = now + transfer_retry_delay(failures);
    ESP_LOGW(TAG, "%s transfer retry %u/%u failed: %s", label, failures, MAX_TRANSFER_RETRIES,
             esp_err_to_name(submit_err));
  };

  for (const auto &record : this->devices_) {
    HIDDevice *device = record.get();
    retry(device, device->transfer, device->transfer_retry_pending, device->transfer_error_count,
          device->transfer_retry_at, "HID", true);
    retry(device, device->media_transfer, device->media_transfer_retry_pending, device->media_transfer_error_count,
          device->media_transfer_retry_at, "media", false);
  }

  for (HIDDevice *device : devices_to_remove) {
    if (device != nullptr && device->active && device->dev_hdl != nullptr) {
      this->handle_device_gone(device->dev_hdl);
    }
  }
}

void USBHIDXComponent::try_finalize_device(HIDDevice *device) {
  if (device == nullptr || device->active || device->transfer != nullptr || device->media_transfer != nullptr ||
      !device->output_transfers.empty()) {
    return;
  }

  if (device->dev_hdl != nullptr) {
    if (device->media_interface_claimed) {
      const esp_err_t err = usb_host_interface_release(this->client_hdl_, device->dev_hdl, device->media_interface_num);
      if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Release media interface %u deferred: %s", device->media_interface_num, esp_err_to_name(err));
        return;
      }
      device->media_interface_claimed = false;
    }

    if (device->interface_claimed) {
      const esp_err_t err = usb_host_interface_release(this->client_hdl_, device->dev_hdl, device->interface_num);
      if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Release HID interface %u deferred: %s", device->interface_num, esp_err_to_name(err));
        return;
      }
      device->interface_claimed = false;
    }

    const esp_err_t close_err = usb_host_device_close(this->client_hdl_, device->dev_hdl);
    if (close_err != ESP_OK && close_err != ESP_ERR_NOT_FOUND) {
      ESP_LOGW(TAG, "USB device close deferred: %s", esp_err_to_name(close_err));
      return;
    }
    device->dev_hdl = nullptr;
  }

  HIDDeviceDriver *driver = device->driver;
  this->active_driver_instances_.erase(
      std::remove_if(this->active_driver_instances_.begin(), this->active_driver_instances_.end(),
                     [driver](const std::unique_ptr<HIDDeviceDriver> &candidate) { return candidate.get() == driver; }),
      this->active_driver_instances_.end());

  auto it = std::find_if(this->devices_.begin(), this->devices_.end(),
                         [device](const std::unique_ptr<HIDDevice> &record) { return record.get() == device; });
  if (it != this->devices_.end()) {
    this->devices_.erase(it);
  }
}

void USBHIDXComponent::client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg) {
  auto *component = static_cast<USBHIDXComponent *>(arg);

  if (component == nullptr || event_msg == nullptr) {
    return;
  }

  switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
      component->handle_new_device(event_msg->new_dev.address);
      break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
      component->handle_device_gone(event_msg->dev_gone.dev_hdl);
      break;
    default:
      break;
  }
}

void USBHIDXComponent::handle_new_device(uint8_t address) {
  ESP_LOGI(TAG, "New USB device detected at address %d", address);

  devices_.emplace_back(new (std::nothrow) HIDDevice());
  if (devices_.empty() || devices_.back() == nullptr) {
    ESP_LOGE(TAG, "Unable to allocate HID device record");
    if (!devices_.empty() && devices_.back() == nullptr) {
      devices_.pop_back();
    }
    return;
  }

  HIDDevice *dev = devices_.back().get();
  dev->dev_addr = address;

  esp_err_t err = usb_host_device_open(this->client_hdl_, address, &dev->dev_hdl);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(err));
    devices_.pop_back();
    return;
  }

  usb_device_info_t dev_info{};
  err = usb_host_device_info(dev->dev_hdl, &dev_info);
  if (err == ESP_OK) {
    dev->speed = dev_info.speed;
    ESP_LOGI(TAG, "Device bus speed: %s", usb_speed_name(dev->speed));
  } else {
    ESP_LOGW(TAG, "Failed to get device bus speed: %s", esp_err_to_name(err));
  }

  const usb_device_desc_t *dev_desc;
  err = usb_host_get_device_descriptor(dev->dev_hdl, &dev_desc);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get device descriptor");
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    devices_.pop_back();
    return;
  }

  dev->vid = dev_desc->idVendor;
  dev->pid = dev_desc->idProduct;

  ESP_LOGI(TAG, "Device VID:PID = %04X:%04X, Class=0x%02X", dev->vid, dev->pid, dev_desc->bDeviceClass);

  // Check for Xbox 360 devices (vendor-specific class 0xFF)
  bool is_xbox360 = (dev->vid == 0x045E && (dev->pid == 0x028E || dev->pid == 0x0719)) ||
                    (dev->vid == 0x2DC8 && dev->pid == 0x310B) ||  // 8BitDo (Xbox mode)
                    (dev->vid == 0x1BAD) ||                        // Mad Catz / Rock Band instruments
                    (dev->vid == 0x1430);                          // RedOctane / Guitar Hero instruments

  // Check for 8BitDo devices (may use vendor-specific class)
  bool is_8bitdo = (dev->vid == 0x2DC8 && dev->pid != 0x310B);  // Exclude Xbox mode

  // Check for MCE IR receivers (may use vendor-specific interface class)
  bool is_mce =
      (dev->vid == 0x1784 || dev->vid == 0x0471 || dev->vid == 0x0609 || dev->vid == 0x1460 || dev->vid == 0x1308 ||
       dev->vid == 0x051C || dev->vid == 0x107B || dev->vid == 0x03EE || dev->vid == 0x179D || dev->vid == 0x195D ||
       dev->vid == 0x1509 || dev->vid == 0x043E || dev->vid == 0x147A || dev->vid == 0x1934 || dev->vid == 0x2304 ||
       dev->vid == 0x1019 || dev->vid == 0x0FB8 || dev->vid == 0x185B || dev->vid == 0x04EB || dev->vid == 0x105A ||
       dev->vid == 0x0572 || dev->vid == 0x0BDA || dev->vid == 0x2596 || dev->vid == 0x03F3 ||
       (dev->vid == 0x045E && (dev->pid == 0x006D || dev->pid == 0x00A0 || dev->pid == 0x00F2)));

  if (dev_desc->bDeviceClass != 0x03 && dev_desc->bDeviceClass != 0x00 && !is_xbox360 && !is_8bitdo && !is_mce) {
    ESP_LOGD(TAG, "Not a HID device (class 0x%02X), ignoring", dev_desc->bDeviceClass);
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    devices_.pop_back();
    return;
  }

  // Get config descriptor
  const usb_config_desc_t *config_desc;
  err = usb_host_get_active_config_descriptor(dev->dev_hdl, &config_desc);
  if (err != ESP_OK) {
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    devices_.pop_back();
    return;
  }

  ESP_LOGI(TAG, "Config: wTotalLength=%d, bNumInterfaces=%d", config_desc->wTotalLength, config_desc->bNumInterfaces);

  // Find the first usable HID interface. Composite devices are allowed to
  // expose several HID interfaces, and the first matching interface is not
  // necessarily the one with an interrupt-IN endpoint. Keep the current
  // interface while walking its endpoint descriptors and only select a
  // candidate once a valid interrupt-IN endpoint is found.
  const usb_intf_desc_t *intf_desc = nullptr;
  const usb_ep_desc_t *ep_desc = nullptr;
  const usb_intf_desc_t *current_intf = nullptr;
  bool current_intf_accepted = false;
  uint8_t out_ep = 0;
  int offset = 0;

  if (is_8bitdo) {
    ESP_LOGI(TAG, "8BitDo - scanning all interfaces:");
  }

  while (offset < config_desc->wTotalLength) {
    const usb_standard_desc_t *desc = (const usb_standard_desc_t *) ((uint8_t *) config_desc + offset);
    if (desc->bLength < sizeof(usb_standard_desc_t) || offset + desc->bLength > config_desc->wTotalLength) {
      ESP_LOGW(TAG, "Malformed USB descriptor at offset %d", offset);
      break;
    }

    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      if (desc->bLength < sizeof(usb_intf_desc_t)) {
        ESP_LOGW(TAG, "Short interface descriptor at offset %d", offset);
        break;
      }
      const usb_intf_desc_t *temp_intf = (const usb_intf_desc_t *) desc;
      current_intf = temp_intf;
      current_intf_accepted =
          temp_intf->bAlternateSetting == 0 &&
          (temp_intf->bInterfaceClass == 0x03 ||
           (is_xbox360 && (temp_intf->bInterfaceClass == 0xFF || temp_intf->bInterfaceClass == 0x03)) ||
           (is_8bitdo && temp_intf->bInterfaceClass == 0xFF) ||
           (is_mce && (temp_intf->bInterfaceClass == 0xFF || temp_intf->bInterfaceClass == 0x03)));

      if (is_8bitdo) {
        ESP_LOGI(TAG, "  Intf %d: Class=0x%02X Sub=0x%02X Proto=0x%02X EPs=%d", temp_intf->bInterfaceNumber,
                 temp_intf->bInterfaceClass, temp_intf->bInterfaceSubClass, temp_intf->bInterfaceProtocol,
                 temp_intf->bNumEndpoints);
      }
      if (is_xbox360) {
        ESP_LOGI(TAG, "  Intf %d: Class=0x%02X Sub=0x%02X Proto=0x%02X EPs=%d", temp_intf->bInterfaceNumber,
                 temp_intf->bInterfaceClass, temp_intf->bInterfaceSubClass, temp_intf->bInterfaceProtocol,
                 temp_intf->bNumEndpoints);
      }

      // Log rejected interfaces for MCE to help debug
      if (is_mce && !current_intf_accepted) {
        ESP_LOGI(TAG, "MCE intf %d: Class=0x%02X Sub=0x%02X Proto=0x%02X", temp_intf->bInterfaceNumber,
                 temp_intf->bInterfaceClass, temp_intf->bInterfaceSubClass, temp_intf->bInterfaceProtocol);
      }
    } else if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && current_intf_accepted) {
      if (desc->bLength < sizeof(usb_ep_desc_t)) {
        ESP_LOGW(TAG, "Short endpoint descriptor at offset %d", offset);
        break;
      }
      const usb_ep_desc_t *temp_ep = (const usb_ep_desc_t *) desc;
      if ((temp_ep->bmAttributes & 0x03) == 0x03) {
        if ((temp_ep->bEndpointAddress & 0x80) && ep_desc == nullptr) {
          // Fix invalid bInterval=0 (ESP-IDF requires 1-255 for interrupt endpoints)
          if (temp_ep->bInterval == 0) {
            ESP_LOGW(TAG, "Fixing invalid bInterval=0 on IN endpoint 0x%02X", temp_ep->bEndpointAddress);
            const_cast<usb_ep_desc_t *>(temp_ep)->bInterval = 1;
          }
          intf_desc = current_intf;
          dev->protocol = current_intf->bInterfaceProtocol;
          ep_desc = temp_ep;
          ESP_LOGI(TAG, "Found HID interface %d, protocol %d, class 0x%02X", intf_desc->bInterfaceNumber, dev->protocol,
                   intf_desc->bInterfaceClass);
          ESP_LOGI(TAG, "Found interrupt IN endpoint: 0x%02X (interval=%d)", ep_desc->bEndpointAddress,
                   ep_desc->bInterval);
        } else if (!(temp_ep->bEndpointAddress & 0x80) && intf_desc == current_intf && out_ep == 0) {
          if (temp_ep->bInterval == 0) {
            ESP_LOGW(TAG, "Fixing invalid bInterval=0 on OUT endpoint 0x%02X", temp_ep->bEndpointAddress);
            const_cast<usb_ep_desc_t *>(temp_ep)->bInterval = 1;
          }
          out_ep = temp_ep->bEndpointAddress;
          ESP_LOGI(TAG, "Found interrupt OUT endpoint: 0x%02X", out_ep);
        }
      }
    }
    offset += desc->bLength;
  }

  if (!intf_desc || !ep_desc) {
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    devices_.pop_back();
    return;
  }

  // Match before claiming an interface or allocating a long-lived transfer.
  // A configured registry is also a device filter: an unrelated HID device
  // must not consume a host channel merely because it happens to enumerate.
  HIDDeviceDriver *matched_driver_template = nullptr;
  ESP_LOGI(TAG, "Attempting to match device (protocol=%d, VID=%04X, PID=%04X) to %u drivers", dev->protocol, dev->vid,
           dev->pid, static_cast<unsigned>(this->drivers_.size()));
  for (const auto &driver_template : this->drivers_) {
    auto *driver = driver_template.get();
    ESP_LOGD(TAG, "Checking driver: %s", driver->get_name());
    if (driver->match_device(dev->protocol, dev->vid, dev->pid)) {
      matched_driver_template = driver;
      break;
    }
  }
  if (matched_driver_template == nullptr && !this->has_raw_bindings_for(dev)) {
    ESP_LOGI(TAG, "No selected driver matched %04X:%04X; ignoring device", dev->vid, dev->pid);
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    devices_.pop_back();
    return;
  }

  // Claim interface
  err = usb_host_interface_claim(this->client_hdl_, dev->dev_hdl, intf_desc->bInterfaceNumber, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to claim interface: %s", esp_err_to_name(err));
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    devices_.pop_back();
    return;
  }

  dev->interface_num = intf_desc->bInterfaceNumber;
  dev->out_endpoint = out_ep;

  // Allocate transfer
  usb_transfer_t *transfer;
  const uint16_t packet_size = ep_desc->wMaxPacketSize & 0x07FF;
  if (packet_size == 0) {
    ESP_LOGE(TAG, "Invalid interrupt endpoint packet size");
    usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->interface_num);
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    devices_.pop_back();
    return;
  }
  err = usb_host_transfer_alloc(packet_size, 0, &transfer);
  if (err != ESP_OK) {
    usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->interface_num);
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    devices_.pop_back();
    return;
  }

  // Clone the matched template before the transfer is submitted. A clone is
  // the per-physical-device state holder; templates never receive reports.
  if (matched_driver_template != nullptr) {
    auto *instance = matched_driver_template->clone();
    if (instance == nullptr) {
      ESP_LOGE(TAG, "Matched %s but could not allocate its device state", matched_driver_template->get_name());
      usb_host_transfer_free(transfer);
      usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->interface_num);
      usb_host_device_close(this->client_hdl_, dev->dev_hdl);
      devices_.pop_back();
      return;
    }
    this->active_driver_instances_.emplace_back(instance);
    dev->driver = instance;
  }

  // Count a channel only after the transfer object exists. Failed descriptor
  // parsing/allocation must not consume accounting capacity permanently.
  this->active_channels_++;

  transfer->device_handle = dev->dev_hdl;
  transfer->bEndpointAddress = ep_desc->bEndpointAddress;
  transfer->context = this;
  transfer->num_bytes = packet_size;
  transfer->callback = USBHIDXComponent::transfer_callback;

  dev->transfer = transfer;
  dev->active = true;
  dev->interface_claimed = true;
  this->connected_devices_++;

  // Put the long-lived input transfer on the host before any driver startup
  // hook can submit an output command. This prevents a failed input submit
  // from leaving driver-owned output callbacks pointing at a freed record.
  err = usb_host_transfer_submit(transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to submit transfer: %s", esp_err_to_name(err));
    usb_host_transfer_free(transfer);
    dev->transfer = nullptr;
    HIDDeviceDriver *driver = dev->driver;
    this->active_driver_instances_.erase(
        std::remove_if(
            this->active_driver_instances_.begin(), this->active_driver_instances_.end(),
            [driver](const std::unique_ptr<HIDDeviceDriver> &candidate) { return candidate.get() == driver; }),
        this->active_driver_instances_.end());
    dev->driver = nullptr;
    usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->interface_num);
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    dev->active = false;
    dev->interface_claimed = false;
    this->active_channels_--;
    this->connected_devices_--;
    dev->dev_hdl = nullptr;
    devices_.pop_back();
    return;
  }

  if (dev->driver != nullptr) {
    ESP_LOGI(TAG, "Matched device to %s driver", dev->driver->get_name());
    // Store Xbox 360 device reference
    if (strcmp(dev->driver->get_name(), "Xbox360") == 0) {
      this->xbox360_device_ = dev;
    }
    if (strcmp(dev->driver->get_name(), "Switch") == 0) {
      this->switch_device_ = dev;
    }
    // Initialization is part of the driver contract, so every selected
    // protocol gets the same lifecycle regardless of its name.
    dev->driver->on_device_ready(dev);
  }

  if (!dev->driver) {
    ESP_LOGW(TAG, "No driver found for protocol %d", dev->protocol);
  }

  ESP_LOGI(TAG, "Device monitoring started (protocol %d)", dev->protocol);
#ifdef USE_TEXT_SENSOR
  if (this->device_name_sensor_) {
    // Build a deterministic inventory from all active devices.  Include
    // VID:PID, host address and bus speed so identical receivers can be
    // distinguished while debugging a populated hub.
    const std::string combined = combined_device_names(this->devices_);
    this->device_name_sensor_->publish_state(combined.empty() ? "Unknown" : combined);
  }
  if (this->device_speed_sensor_) {
    const std::string combined = combined_device_speeds(this->devices_);
    this->device_speed_sensor_->publish_state(combined.empty() ? "None" : combined);
  }
#endif
  // If keyboard (and not an Xbox360/8BitDo in Xbox mode), try to set up media interface
  if (dev->driver != nullptr && strcmp(dev->driver->get_name(), "Keyboard") == 0 && !is_xbox360) {
    ESP_LOGI(TAG, "Keyboard detected, attempting to set up media interface");
    setup_media_interface(dev, config_desc);
  }
}

void USBHIDXComponent::handle_device_gone(usb_device_handle_t dev_hdl) {
  ESP_LOGI(TAG, "USB device disconnected");

  HIDDevice *dev = find_device_by_handle(dev_hdl);
  if (!dev || !dev->active) {
    return;
  }

  dev->active = false;
  if (this->xbox360_device_ == dev) {
    this->xbox360_device_ = nullptr;
    for (const auto &record : this->devices_) {
      if (record->active && record->driver != nullptr && strcmp(record->driver->get_name(), "Xbox360") == 0) {
        this->xbox360_device_ = record.get();
        break;
      }
    }
  }
  if (this->switch_device_ == dev) {
    this->switch_device_ = nullptr;
    for (const auto &record : this->devices_) {
      if (record->active && record->driver != nullptr && strcmp(record->driver->get_name(), "Switch") == 0) {
        this->switch_device_ = record.get();
        break;
      }
    }
  }
  if (dev->driver != nullptr) {
    dev->driver->on_device_removed();
  }
  if (this->connected_devices_ > 0) {
    this->connected_devices_--;
  }
  if (this->active_channels_ > 0) {
    this->active_channels_--;
  }

  // A transfer that already completed with an error is not in the HCD queue,
  // so it can be freed immediately. Transfers still owned by the HCD are
  // released by transfer_callback with NO_DEVICE/CANCELED.
  if (dev->transfer_retry_pending && dev->transfer != nullptr) {
    usb_host_transfer_free(dev->transfer);
    dev->transfer = nullptr;
  }
  dev->transfer_retry_pending = false;
  if (dev->media_transfer_retry_pending && dev->media_transfer != nullptr) {
    usb_host_transfer_free(dev->media_transfer);
    dev->media_transfer = nullptr;
  }
  dev->media_transfer_retry_pending = false;

  ESP_LOGI(TAG, "Device removed, %d devices remaining", this->connected_devices_);
#ifdef USE_TEXT_SENSOR
  if (this->device_name_sensor_) {
    if (this->connected_devices_ == 0) {
      this->device_name_sensor_->publish_state("None");
    } else {
      const std::string combined = combined_device_names(this->devices_);
      this->device_name_sensor_->publish_state(combined.empty() ? "None" : combined);
    }
  }
  if (this->device_speed_sensor_) {
    const std::string combined = combined_device_speeds(this->devices_);
    this->device_speed_sensor_->publish_state(combined.empty() ? "None" : combined);
  }
#endif

  this->try_finalize_device(dev);
}

void USBHIDXComponent::transfer_callback(usb_transfer_t *transfer) {
  if (transfer == nullptr) {
    return;
  }
  if (transfer->context == nullptr) {
    // A transfer with no owner cannot be routed back into the component, but
    // the callback still owns the completed transfer object.
    usb_host_transfer_free(transfer);
    return;
  }
  auto *component = static_cast<USBHIDXComponent *>(transfer->context);
  HIDDevice *dev = component->find_device_by_transfer(transfer);

  if (dev == nullptr) {
    // This can only happen for a stale callback after a programming error or
    // an unexpected host teardown. Do not leak the transfer in that case.
    usb_host_transfer_free(transfer);
    return;
  }

  const bool is_media = transfer == dev->media_transfer;
  const bool is_main = transfer == dev->transfer;
  if (!is_media && !is_main) {
    usb_host_transfer_free(transfer);
    component->untrack_output_transfer(dev, transfer);
    component->try_finalize_device(dev);
    return;
  }

  usb_transfer_t *&slot = is_media ? dev->media_transfer : dev->transfer;
  bool &retry_pending = is_media ? dev->media_transfer_retry_pending : dev->transfer_retry_pending;
  uint8_t &error_count = is_media ? dev->media_transfer_error_count : dev->transfer_error_count;
  uint32_t &retry_at = is_media ? dev->media_transfer_retry_at : dev->transfer_retry_at;
  const char *label = is_media ? "Media" : "HID";

  if (!dev->active) {
    slot = nullptr;
    retry_pending = false;
    usb_host_transfer_free(transfer);
    component->try_finalize_device(dev);
    return;
  }

  if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
    if (transfer->status == USB_TRANSFER_STATUS_NO_DEVICE || transfer->status == USB_TRANSFER_STATUS_CANCELED) {
      const bool no_device = transfer->status == USB_TRANSFER_STATUS_NO_DEVICE;
      const int status = transfer->status;
      ESP_LOGD(TAG, "%s transfer ended without a device (status=%d)", label, status);
      slot = nullptr;
      retry_pending = false;
      error_count = 0;
      usb_host_transfer_free(transfer);
      if (no_device && dev->active) {
        // The HCD can report NO_DEVICE before the client DEV_GONE event is
        // delivered. Complete the same teardown path here so a lost device
        // cannot leave a live record and channel count behind.
        component->handle_device_gone(dev->dev_hdl);
      } else {
        component->try_finalize_device(dev);
      }
      return;
    }
    error_count++;
    if (error_count >= MAX_TRANSFER_RETRIES) {
      ESP_LOGW(TAG, "%s transfer disabled after %u errors (status=%d)", label, error_count, transfer->status);
      slot = nullptr;
      retry_pending = false;
      error_count = 0;
      const bool retire_device = is_main;
      usb_host_transfer_free(transfer);
      if (retire_device && dev->active && dev->dev_hdl != nullptr) {
        component->handle_device_gone(dev->dev_hdl);
      }
      return;
    }
    retry_pending = true;
    retry_at = esphome::millis() + transfer_retry_delay(error_count);
    ESP_LOGW(TAG, "%s transfer status=%d; retry %u/%u scheduled", label, transfer->status, error_count,
             MAX_TRANSFER_RETRIES);
    return;
  }

  retry_pending = false;
  error_count = 0;

  if (transfer->actual_num_bytes > transfer->data_buffer_size) {
    ESP_LOGW(TAG, "%s report length %u exceeds buffer size %u", label, transfer->actual_num_bytes,
             transfer->data_buffer_size);
  } else if (transfer->actual_num_bytes > 0) {
    // Raw YAML mappings are independent of the selected protocol driver and
    // must see every completed report, including reports for devices that have
    // no dedicated driver.
    component->publish_raw_bindings(dev, transfer->data_buffer, transfer->actual_num_bytes);

    // Check if this is an idle report
    bool is_idle = false;

    const bool is_ps_device = dev->vid == 0x054C && dev->pid == 0x0268;
    if (is_ps_device && transfer->actual_num_bytes == 49) {
      // PlayStation idle: no buttons (bytes 2-3 = 0), sticks centered
      uint16_t buttons = transfer->data_buffer[2] | (transfer->data_buffer[3] << 8);
      uint8_t lx = transfer->data_buffer[6];
      uint8_t ly = transfer->data_buffer[7];
      uint8_t rx = transfer->data_buffer[8];
      uint8_t ry = transfer->data_buffer[9];
      bool sticks_centered = (abs((int) lx - 128) < 10 && abs((int) ly - 128) < 10 && abs((int) rx - 128) < 10 &&
                              abs((int) ry - 128) < 10);
      is_idle = (buttons == 0 && (transfer->data_buffer[4] & 0x01) == 0 && sticks_centered);
    } else if (transfer->actual_num_bytes == 6 && transfer->data_buffer[0] == 0x01) {
      // Interact flight stick idle: [01 55 7F 7F 00 00] - buttons 0x55, centered sticks, neutral HAT
      is_idle =
          (transfer->data_buffer[1] == 0x55 && transfer->data_buffer[2] == 0x7F && transfer->data_buffer[3] == 0x7F &&
           transfer->data_buffer[4] == 0x00 && transfer->data_buffer[5] == 0x00);
    } else {
      // Generic idle check for other devices
      is_idle = (transfer->bEndpointAddress == 0x81 && transfer->actual_num_bytes == 8 &&
                 transfer->data_buffer[0] == 0 && transfer->data_buffer[1] == 0 && transfer->data_buffer[2] == 0x0F &&
                 transfer->data_buffer[3] == 0x80 && transfer->data_buffer[4] == 0x80 &&
                 transfer->data_buffer[5] == 0x80 && transfer->data_buffer[6] == 0x80 && transfer->data_buffer[7] == 0);
    }

    if (!is_idle) {
#ifdef USE_TEXT_SENSOR
      if (component->last_input_sensor_) {
        // Identify the physical USB device that produced this non-idle report.
        // The address is useful when two identical receivers/keyboards share
        // the same VID:PID; it is assigned by the host and may change after
        // a reconnect.
        char input_source[96];
        snprintf(input_source, sizeof(input_source), "%s %04X:%04X @%u %s",
                 dev->driver ? dev->driver->get_name() : "HID", dev->vid, dev->pid, dev->dev_addr,
                 usb_speed_name(dev->speed));
        component->last_input_sensor_->publish_state(input_source);
      }
#endif

      // Optional protocol-specific diagnostic logging is kept in the driver.
      // Do not format every interrupt report here: high-rate devices can make
      // logging starve the USB host task and hide the actual hot-plug events.
    }

    if (dev->driver) {
      if (is_media) {
        // Force media report processing by setting a flag in the data
        // We'll use a temporary buffer with a marker
        // Keyboard media reports are currently passed through the legacy
        // 0xFF marker used by KeyboardDriver. Never copy past the fixed
        // buffer: a malformed or high-speed report must be rejected rather
        // than corrupting the USB host task stack.
        if (transfer->actual_num_bytes > 64) {
          ESP_LOGW(TAG, "Media report too large (%u bytes), dropping", transfer->actual_num_bytes);
        } else {
          uint8_t temp_data[65];
          temp_data[0] = 0xFF;  // Marker for media report
          memcpy(&temp_data[1], transfer->data_buffer, transfer->actual_num_bytes);
          dev->driver->process_report(temp_data, transfer->actual_num_bytes + 1, dev);
        }
      } else {
        dev->driver->process_report(transfer->data_buffer, transfer->actual_num_bytes, dev);
      }
    }
  }

  if (!dev->active) {
    slot = nullptr;
    usb_host_transfer_free(transfer);
    component->try_finalize_device(dev);
    return;
  }

  const esp_err_t submit_err = usb_host_transfer_submit(transfer);
  if (submit_err == ESP_OK) {
    return;
  }

  error_count++;
  if (error_count >= MAX_TRANSFER_RETRIES) {
    ESP_LOGW(TAG, "%s transfer disabled after submit errors: %s", label, esp_err_to_name(submit_err));
    slot = nullptr;
    retry_pending = false;
    error_count = 0;
    const bool retire_device = is_main;
    usb_host_transfer_free(transfer);
    if (retire_device && dev->active && dev->dev_hdl != nullptr) {
      component->handle_device_gone(dev->dev_hdl);
    }
    return;
  }
  retry_pending = true;
  retry_at = esphome::millis() + transfer_retry_delay(error_count);
  ESP_LOGW(TAG, "%s transfer submit failed; retry %u/%u scheduled: %s", label, error_count, MAX_TRANSFER_RETRIES,
           esp_err_to_name(submit_err));
}

void USBHIDXComponent::setup_media_interface(HIDDevice *dev, const usb_config_desc_t *config_desc) {
  ESP_LOGI(TAG, "Searching for secondary HID interface...");
  // Interface numbers are not required to be contiguous or to start at zero.
  const usb_intf_desc_t *intf_desc = nullptr;
  const usb_ep_desc_t *ep_desc = nullptr;
  const usb_intf_desc_t *candidate_intf = nullptr;
  int offset = 0;

  while (offset < config_desc->wTotalLength) {
    const usb_standard_desc_t *desc = (const usb_standard_desc_t *) ((uint8_t *) config_desc + offset);
    if (desc->bLength < sizeof(usb_standard_desc_t) || offset + desc->bLength > config_desc->wTotalLength) {
      ESP_LOGW(TAG, "Malformed media descriptor at offset %d", offset);
      break;
    }
    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      if (desc->bLength < sizeof(usb_intf_desc_t)) {
        ESP_LOGW(TAG, "Short media interface descriptor at offset %d", offset);
        break;
      }
      const usb_intf_desc_t *temp_intf = (const usb_intf_desc_t *) desc;
      ESP_LOGD(TAG, "Found interface %d: Class=0x%02X", temp_intf->bInterfaceNumber, temp_intf->bInterfaceClass);
      if (candidate_intf != nullptr) {
        // The preceding secondary HID interface had no usable interrupt-IN
        // endpoint. Continue scanning rather than making that interface block
        // a later media interface in a composite keyboard.
        candidate_intf = nullptr;
      }
      if (temp_intf->bInterfaceNumber != dev->interface_num && temp_intf->bInterfaceClass == 0x03 &&
          temp_intf->bAlternateSetting == 0) {
        candidate_intf = temp_intf;
        ESP_LOGI(TAG, "Found candidate secondary HID interface %d", temp_intf->bInterfaceNumber);
      }
      if (intf_desc != nullptr) {
        // A valid endpoint was already found on the preceding interface.
        break;
      }
    } else if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && candidate_intf) {
      if (desc->bLength < sizeof(usb_ep_desc_t)) {
        ESP_LOGW(TAG, "Short media endpoint descriptor at offset %d", offset);
        break;
      }
      const usb_ep_desc_t *temp_ep = (const usb_ep_desc_t *) desc;
      if ((temp_ep->bEndpointAddress & 0x80) && ((temp_ep->bmAttributes & 0x03) == 0x03)) {
        // A few composite keyboard receivers expose an invalid zero polling
        // interval on the secondary interface. ESP-IDF rejects that value;
        // use the minimum legal interval just as we do for the primary HID
        // interface.
        if (temp_ep->bInterval == 0) {
          ESP_LOGW(TAG, "Fixing invalid bInterval=0 on media endpoint 0x%02X", temp_ep->bEndpointAddress);
          const_cast<usb_ep_desc_t *>(temp_ep)->bInterval = 1;
        }
        intf_desc = candidate_intf;
        ep_desc = temp_ep;
        ESP_LOGI(TAG, "Found media endpoint: 0x%02X", ep_desc->bEndpointAddress);
        break;
      }
    }
    offset += desc->bLength;
  }

  if (!intf_desc || !ep_desc) {
    ESP_LOGD(TAG, "No media interface found");
    return;
  }

  esp_err_t err = usb_host_interface_claim(this->client_hdl_, dev->dev_hdl, intf_desc->bInterfaceNumber, 0);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to claim media interface: %s", esp_err_to_name(err));
    return;
  }

  dev->media_interface_num = intf_desc->bInterfaceNumber;
  dev->media_interface_claimed = true;

  usb_transfer_t *transfer;
  const uint16_t packet_size = ep_desc->wMaxPacketSize & 0x07FF;
  if (packet_size == 0) {
    ESP_LOGW(TAG, "Invalid media endpoint packet size");
    usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->media_interface_num);
    dev->media_interface_claimed = false;
    return;
  }
  err = usb_host_transfer_alloc(packet_size, 0, &transfer);
  if (err != ESP_OK) {
    esp_err_t release_err = usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->media_interface_num);
    ESP_LOGD(TAG, "Release media interface after allocation failure: %s", esp_err_to_name(release_err));
    dev->media_interface_claimed = false;
    return;
  }

  transfer->device_handle = dev->dev_hdl;
  transfer->bEndpointAddress = ep_desc->bEndpointAddress;
  transfer->context = this;
  transfer->num_bytes = packet_size;
  transfer->callback = USBHIDXComponent::transfer_callback;

  dev->media_transfer = transfer;

  err = usb_host_transfer_submit(transfer);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Media key monitoring started on endpoint 0x%02X", ep_desc->bEndpointAddress);
  } else {
    ESP_LOGW(TAG, "Failed to submit media transfer: %s", esp_err_to_name(err));
    usb_host_transfer_free(transfer);
    esp_err_t release_err = usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->media_interface_num);
    ESP_LOGD(TAG, "Release media interface after submit failure: %s", esp_err_to_name(release_err));
    dev->media_interface_claimed = false;
    dev->media_transfer = nullptr;
  }
}

void USBHIDXComponent::led_control_callback(usb_transfer_t *transfer) {
  if (transfer == nullptr) {
    return;
  }
  auto *component = static_cast<USBHIDXComponent *>(transfer->context);
  HIDDevice *device = component != nullptr ? component->find_device_by_transfer(transfer) : nullptr;
  if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGD(TAG, "LED command completed");
  } else {
    ESP_LOGW(TAG, "LED command failed: %d", transfer->status);
  }
  if (component != nullptr) {
    component->untrack_output_transfer(device, transfer);
  }
  usb_host_transfer_free(transfer);
  if (component != nullptr) {
    component->try_finalize_device(device);
  }
}

void USBHIDXComponent::update_keyboard_leds(HIDDevice *device, uint8_t led_state) {
  if (!device || !device->dev_hdl || !this->client_hdl_) {
    ESP_LOGW(TAG, "Cannot update LEDs - device not available");
    return;
  }

  usb_transfer_t *ctrl_transfer;
  esp_err_t err = usb_host_transfer_alloc(16, 0, &ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to allocate LED transfer");
    return;
  }

  usb_setup_packet_t setup_pkt = {
      .bmRequestType = 0x21, .bRequest = 0x09, .wValue = 0x0200, .wIndex = device->interface_num, .wLength = 1};

  ctrl_transfer->device_handle = device->dev_hdl;
  ctrl_transfer->callback = USBHIDXComponent::led_control_callback;
  ctrl_transfer->context = this;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  ctrl_transfer->data_buffer[sizeof(usb_setup_packet_t)] = led_state;
  ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + 1;
  this->track_output_transfer(device, ctrl_transfer);

  err = usb_host_transfer_submit_control(this->client_hdl_, ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit LED command: %s", esp_err_to_name(err));
    this->untrack_output_transfer(device, ctrl_transfer);
    usb_host_transfer_free(ctrl_transfer);
  }
}

void USBHIDXComponent::send_xbox360_output(HIDDevice *device, const uint8_t *data, size_t len) {
  if (!device || !device->dev_hdl || !this->client_hdl_) {
    ESP_LOGW(TAG, "Cannot send Xbox 360 command - device not available");
    return;
  }
  if ((data == nullptr && len != 0) || len > std::numeric_limits<uint16_t>::max()) {
    ESP_LOGW(TAG, "Invalid Xbox 360 output report length: %u", len);
    return;
  }

  usb_transfer_t *ctrl_transfer;
  if (len > std::numeric_limits<size_t>::max() - sizeof(usb_setup_packet_t)) {
    ESP_LOGW(TAG, "Xbox 360 output report length overflow: %u", len);
    return;
  }
  esp_err_t err = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + len, 0, &ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to allocate Xbox 360 transfer");
    return;
  }

  usb_setup_packet_t setup_pkt = {.bmRequestType = 0x21,  // Host-to-device, Class, Interface
                                  .bRequest = 0x09,       // SET_REPORT
                                  .wValue = 0x0200,       // Output report
                                  .wIndex = device->interface_num,
                                  .wLength = (uint16_t) len};

  ctrl_transfer->device_handle = device->dev_hdl;
  ctrl_transfer->callback = USBHIDXComponent::led_control_callback;
  ctrl_transfer->context = this;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  if (len != 0) {
    memcpy(ctrl_transfer->data_buffer + sizeof(usb_setup_packet_t), data, len);
  }
  ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + len;
  this->track_output_transfer(device, ctrl_transfer);

  err = usb_host_transfer_submit_control(this->client_hdl_, ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit Xbox 360 command: %s", esp_err_to_name(err));
    this->untrack_output_transfer(device, ctrl_transfer);
    usb_host_transfer_free(ctrl_transfer);
  }
}

void USBHIDXComponent::send_xbox360_rumble(uint8_t left_motor, uint8_t right_motor) {
#ifdef USB_HIDX_ENABLE_XBOX360
  if (!xbox360_device_ || !xbox360_device_->driver) {
    ESP_LOGW(TAG, "No Xbox 360 controller is connected");
    return;
  }
  xbox360_device_->driver->send_rumble(xbox360_device_, left_motor, right_motor);
#else
  ESP_LOGW(TAG, "Xbox 360 gamepad support not enabled");
#endif
}

void USBHIDXComponent::send_switch_rumble(uint8_t left_motor, uint8_t right_motor) {
#ifdef USB_HIDX_ENABLE_SWITCH
  if (!switch_device_ || !switch_device_->driver) {
    ESP_LOGW(TAG, "No Switch controller is connected");
    return;
  }
  switch_device_->driver->send_rumble(switch_device_, left_motor, right_motor);
#else
  ESP_LOGW(TAG, "Switch gamepad support not enabled");
#endif
}

void USBHIDXComponent::send_playstation_get_report(HIDDevice *device, uint8_t report_id) {
  if (!device || !device->dev_hdl || !this->client_hdl_) {
    ESP_LOGW(TAG, "Cannot send GET_REPORT - device not available");
    return;
  }

  usb_transfer_t *ctrl_transfer;
  esp_err_t err = usb_host_transfer_alloc(64, 0, &ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to allocate GET_REPORT transfer");
    return;
  }

  usb_setup_packet_t setup_pkt = {.bmRequestType = 0xA1,                           // Device-to-host, Class, Interface
                                  .bRequest = 0x01,                                // GET_REPORT
                                  .wValue = (uint16_t) ((0x03 << 8) | report_id),  // Feature report
                                  .wIndex = device->interface_num,
                                  .wLength = 49};

  ctrl_transfer->device_handle = device->dev_hdl;
  ctrl_transfer->callback = USBHIDXComponent::led_control_callback;
  ctrl_transfer->context = this;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + 49;
  this->track_output_transfer(device, ctrl_transfer);

  err = usb_host_transfer_submit_control(this->client_hdl_, ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit GET_REPORT: %s", esp_err_to_name(err));
    this->untrack_output_transfer(device, ctrl_transfer);
    usb_host_transfer_free(ctrl_transfer);
  } else {
    ESP_LOGI(TAG, "Sent GET_REPORT for PlayStation");
  }
}

void USBHIDXComponent::send_xbox360_interrupt_out(HIDDevice *device, const uint8_t *data, size_t len) {
  if (!device || !device->dev_hdl || !device->out_endpoint) {
    ESP_LOGW(TAG, "Cannot send interrupt OUT - device or endpoint not available");
    return;
  }
  if (data == nullptr || len == 0 || len > std::numeric_limits<uint16_t>::max()) {
    ESP_LOGW(TAG, "Invalid interrupt OUT report length: %u", len);
    return;
  }

  usb_transfer_t *out_transfer;
  esp_err_t err = usb_host_transfer_alloc(len, 0, &out_transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to allocate OUT transfer");
    return;
  }

  out_transfer->device_handle = device->dev_hdl;
  out_transfer->bEndpointAddress = device->out_endpoint;
  out_transfer->callback = USBHIDXComponent::led_control_callback;
  out_transfer->context = this;
  out_transfer->num_bytes = len;
  memcpy(out_transfer->data_buffer, data, len);
  this->track_output_transfer(device, out_transfer);

  err = usb_host_transfer_submit(out_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit OUT transfer: %s", esp_err_to_name(err));
    this->untrack_output_transfer(device, out_transfer);
    usb_host_transfer_free(out_transfer);
  }
}

esp_err_t USBHIDXComponent::send_hid_output_report(HIDDevice *device, const uint8_t *data, size_t len) {
  if (!device || !device->dev_hdl || !this->client_hdl_) {
    ESP_LOGW(TAG, "Cannot send HID output - device not available");
    return ESP_ERR_INVALID_STATE;
  }
  if ((data == nullptr && len != 0) || len > std::numeric_limits<uint16_t>::max() ||
      len > std::numeric_limits<size_t>::max() - sizeof(usb_setup_packet_t)) {
    ESP_LOGW(TAG, "Invalid HID output report length: %u", len);
    return ESP_ERR_INVALID_SIZE;
  }

  usb_transfer_t *ctrl_transfer;
  esp_err_t err = usb_host_transfer_alloc(len + sizeof(usb_setup_packet_t), 0, &ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to allocate HID output transfer");
    return err;
  }

  usb_setup_packet_t setup_pkt = {.bmRequestType = 0x21,  // Host-to-device, Class, Interface
                                  .bRequest = 0x09,       // SET_REPORT
                                  .wValue = 0x0200,       // Output report
                                  .wIndex = device->interface_num,
                                  .wLength = (uint16_t) len};

  ctrl_transfer->device_handle = device->dev_hdl;
  ctrl_transfer->callback = USBHIDXComponent::led_control_callback;
  ctrl_transfer->context = this;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  if (len != 0) {
    memcpy(ctrl_transfer->data_buffer + sizeof(usb_setup_packet_t), data, len);
  }
  ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + len;
  this->track_output_transfer(device, ctrl_transfer);

  err = usb_host_transfer_submit_control(this->client_hdl_, ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit HID output: %s", esp_err_to_name(err));
    this->untrack_output_transfer(device, ctrl_transfer);
    usb_host_transfer_free(ctrl_transfer);
    return err;
  }

  return ESP_OK;
}

}  // namespace usb_hidx
}  // namespace esphome
