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

void USBHIDXComponent::setup() {
  ESP_LOGI(TAG, "Setting up USB HIDX component");

  // Auto-register all available device drivers
  register_all_drivers(this);
  ESP_LOGI(TAG, "Registered %d device drivers", this->drivers_.size());

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

void USBHIDXComponent::loop() {
  if (this->client_hdl_) {
    esp_err_t err = usb_host_client_handle_events(this->client_hdl_, 0);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "USB client event error: %s", esp_err_to_name(err));
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
    if (record->active && record->dev_hdl == dev_hdl) {
      return record.get();
    }
  }
  return nullptr;
}

HIDDevice *USBHIDXComponent::find_device_by_transfer(usb_transfer_t *transfer) {
  for (const auto &record : devices_) {
    if (record->active && (record->transfer == transfer || record->media_transfer == transfer)) {
      return record.get();
    }
  }
  return nullptr;
}

void USBHIDXComponent::client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg) {
  auto *component = static_cast<USBHIDXComponent *>(arg);

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

  // Small delay to allow device to stabilize after connect/reconnect
  vTaskDelay(pdMS_TO_TICKS(100));

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

  // Find HID interface - log ALL interfaces for 8BitDo
  const usb_intf_desc_t *intf_desc = nullptr;
  const usb_ep_desc_t *ep_desc = nullptr;
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

      // Accept HID class (0x03), Xbox 360 vendor-specific (0xFF), 8BitDo, or MCE vendor-specific
      if ((temp_intf->bInterfaceClass == 0x03 ||
           (is_xbox360 && (temp_intf->bInterfaceClass == 0xFF || temp_intf->bInterfaceClass == 0x03)) ||
           (is_8bitdo && temp_intf->bInterfaceClass == 0xFF) ||
           (is_mce && (temp_intf->bInterfaceClass == 0xFF || temp_intf->bInterfaceClass == 0x03)))) {
        intf_desc = temp_intf;
        dev->protocol = intf_desc->bInterfaceProtocol;
        ESP_LOGI(TAG, "Found HID interface, protocol %d, class 0x%02X", dev->protocol, temp_intf->bInterfaceClass);
        break;
      }
      // Log rejected interfaces for MCE to help debug
      if (is_mce) {
        ESP_LOGI(TAG, "MCE intf %d: Class=0x%02X Sub=0x%02X Proto=0x%02X", temp_intf->bInterfaceNumber,
                 temp_intf->bInterfaceClass, temp_intf->bInterfaceSubClass, temp_intf->bInterfaceProtocol);
      }
      // For 8BitDo, also accept standard HID on interface 0
      if (is_8bitdo && temp_intf->bInterfaceClass == 0x03 && temp_intf->bInterfaceNumber == 0) {
        intf_desc = temp_intf;
        dev->protocol = intf_desc->bInterfaceProtocol;
        ESP_LOGI(TAG, "Found 8BitDo HID interface, protocol %d", dev->protocol);
        break;
      }
    }
    offset += desc->bLength;
  }

  if (!intf_desc) {
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    devices_.pop_back();
    return;
  }

  // Find interrupt IN endpoint
  offset = (uint8_t *) intf_desc - (uint8_t *) config_desc + intf_desc->bLength;
  uint8_t out_ep = 0;
  while (offset < config_desc->wTotalLength) {
    const usb_standard_desc_t *desc = (const usb_standard_desc_t *) ((uint8_t *) config_desc + offset);
    if (desc->bLength < sizeof(usb_standard_desc_t) || offset + desc->bLength > config_desc->wTotalLength) {
      ESP_LOGW(TAG, "Malformed endpoint descriptor at offset %d", offset);
      break;
    }

    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
      if (desc->bLength < sizeof(usb_ep_desc_t)) {
        ESP_LOGW(TAG, "Short endpoint descriptor at offset %d", offset);
        break;
      }
      const usb_ep_desc_t *temp_ep = (const usb_ep_desc_t *) desc;
      if ((temp_ep->bEndpointAddress & 0x80) && ((temp_ep->bmAttributes & 0x03) == 0x03)) {
        // Fix invalid bInterval=0 (ESP-IDF requires 1-255 for interrupt endpoints)
        if (temp_ep->bInterval == 0) {
          ESP_LOGW(TAG, "Fixing invalid bInterval=0 on IN endpoint 0x%02X", temp_ep->bEndpointAddress);
          const_cast<usb_ep_desc_t *>(temp_ep)->bInterval = 1;
        }
        ep_desc = temp_ep;
        ESP_LOGI(TAG, "Found interrupt IN endpoint: 0x%02X (interval=%d)", ep_desc->bEndpointAddress,
                 ep_desc->bInterval);
      } else if (!(temp_ep->bEndpointAddress & 0x80) && ((temp_ep->bmAttributes & 0x03) == 0x03)) {
        if (temp_ep->bInterval == 0) {
          ESP_LOGW(TAG, "Fixing invalid bInterval=0 on OUT endpoint 0x%02X", temp_ep->bEndpointAddress);
          const_cast<usb_ep_desc_t *>(temp_ep)->bInterval = 1;
        }
        out_ep = temp_ep->bEndpointAddress;
        ESP_LOGI(TAG, "Found interrupt OUT endpoint: 0x%02X", out_ep);
      }
    } else if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      break;
    }
    offset += desc->bLength;
  }

  if (!ep_desc) {
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
  this->active_channels_++;

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

  transfer->device_handle = dev->dev_hdl;
  transfer->bEndpointAddress = ep_desc->bEndpointAddress;
  transfer->context = this;
  transfer->num_bytes = packet_size;
  transfer->callback = USBHIDXComponent::transfer_callback;

  dev->transfer = transfer;
  dev->active = true;
  this->connected_devices_++;

  // Match device to driver
  ESP_LOGI(TAG, "Attempting to match device (protocol=%d, VID=%04X, PID=%04X) to %d drivers", dev->protocol, dev->vid,
           dev->pid, this->drivers_.size());
  for (auto *driver : this->drivers_) {
    ESP_LOGD(TAG, "Checking driver: %s", driver->get_name());
    if (driver->match_device(dev->protocol, dev->vid, dev->pid)) {
      dev->driver = driver;
      ESP_LOGI(TAG, "Matched device to %s driver", driver->get_name());
      // Store Xbox 360 device reference
      if (strcmp(driver->get_name(), "Xbox360") == 0) {
        this->xbox360_device_ = dev;
#ifdef USB_HIDX_ENABLE_XBOX360
        // Initialize immediately after match so controller starts sending reports
        auto *xbox_driver = static_cast<Xbox360Driver *>(driver);
        xbox_driver->set_device(dev);
        xbox_driver->init_controller(dev);
#endif
      }
#ifdef USB_HIDX_ENABLE_PLAYSTATION
      // Call on_device_ready for drivers that need initialization
      if (strcmp(driver->get_name(), "PlayStation") == 0 || strcmp(driver->get_name(), "PS3") == 0 ||
          strcmp(driver->get_name(), "PS4") == 0 || strcmp(driver->get_name(), "PS5") == 0) {
        auto *ps_driver = static_cast<PlayStationDriver *>(driver);
        ps_driver->on_device_ready(dev);
      }
#endif
#ifdef USB_HIDX_ENABLE_SWITCH
      if (strcmp(driver->get_name(), "Switch") == 0) {
        auto *switch_driver = static_cast<SwitchDriver *>(driver);
        switch_driver->on_device_ready(dev);
      }
#endif
#ifdef USB_HIDX_ENABLE_MCE_REMOTE
      if (strcmp(driver->get_name(), "MCERemote") == 0) {
        auto *mce_driver = static_cast<MCERemoteDriver *>(driver);
        mce_driver->on_device_ready(dev);
      }
#endif
      break;
    }
  }

  if (!dev->driver) {
    ESP_LOGW(TAG, "No driver found for protocol %d", dev->protocol);
  }

  err = usb_host_transfer_submit(transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to submit transfer: %s", esp_err_to_name(err));
    usb_host_transfer_free(transfer);
    usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->interface_num);
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    dev->active = false;
    this->active_channels_--;
    this->connected_devices_--;
    dev->dev_hdl = nullptr;
    devices_.pop_back();
  } else {
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
    if (dev->protocol == 0x01 && !is_xbox360) {
      ESP_LOGI(TAG, "Keyboard detected, attempting to set up media interface");
      setup_media_interface(dev, config_desc);
    }
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
  }
#ifdef USB_HIDX_ENABLE_SWITCH
  if (this->switch_driver_ != nullptr) {
    this->switch_driver_->set_device(nullptr);
  }
#endif
#ifdef USB_HIDX_ENABLE_PLAYSTATION
  if (this->playstation_driver_ != nullptr) {
    this->playstation_driver_->set_device(nullptr);
  }
#endif
  this->connected_devices_--;
  if (this->active_channels_ > 0) {
    this->active_channels_--;
  }

  if (dev->transfer) {
    dev->transfer = nullptr;
  }

  if (dev->dev_hdl) {
    // A composite keyboard normally has two claimed interfaces: the main
    // keyboard interface and a second media-key interface.  ESP-IDF refuses
    // to close a device while *any* interface remains claimed.  Releasing
    // only interface 0 strands the upstream port after unplug/replug.
    if (dev->media_interface_claimed) {
      esp_err_t err = usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->media_interface_num);
      ESP_LOGD(TAG, "Release media interface %d: %s", dev->media_interface_num, esp_err_to_name(err));
      if (err == ESP_OK || err == ESP_ERR_NOT_FOUND) {
        dev->media_interface_claimed = false;
      }
    }

    esp_err_t err = usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->interface_num);
    ESP_LOGD(TAG, "Release HID interface %d: %s", dev->interface_num, esp_err_to_name(err));

    err = usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    ESP_LOGD(TAG, "Close USB device: %s", esp_err_to_name(err));
    if (err == ESP_OK || err == ESP_ERR_NOT_FOUND) {
      dev->dev_hdl = nullptr;
    } else {
      ESP_LOGW(TAG, "USB device close deferred; the device still owns a host resource");
    }
  }

  dev->media_transfer = nullptr;

  // Remove the record so a later hot-plug can reclaim its memory. The record
  // is erased only after the device is inactive and its interfaces are closed,
  // so in-flight transfer callbacks can safely observe the inactive state.
  for (auto it = devices_.begin(); it != devices_.end(); ++it) {
    if (it->get() == dev) {
      devices_.erase(it);
      break;
    }
  }

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
}

void USBHIDXComponent::transfer_callback(usb_transfer_t *transfer) {
  if (transfer == nullptr || transfer->context == nullptr) {
    return;
  }
  auto *component = static_cast<USBHIDXComponent *>(transfer->context);
  HIDDevice *dev = component->find_device_by_transfer(transfer);

  // Only log PlayStation transfers if not idle
  bool is_ps_device = (dev && dev->vid == 0x054C && dev->pid == 0x0268);

  if (!dev || !dev->active) {
    if (transfer)
      usb_host_transfer_free(transfer);
    return;
  }

  if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
    // Raw YAML mappings are independent of the selected protocol driver and
    // must see every completed report, including reports for devices that have
    // no dedicated driver.
    component->publish_raw_bindings(dev, transfer->data_buffer, transfer->actual_num_bytes);

    // Check if this is an idle report
    bool is_idle = false;

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
      // Check if this is from the media interface (0x82) or keyboard interface (0x81)
      bool is_media = (transfer == dev->media_transfer);
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

  if (dev->active && usb_host_transfer_submit(transfer) != ESP_OK) {
    usb_host_transfer_free(transfer);
    if (transfer == dev->media_transfer) {
      dev->media_transfer = nullptr;
    }
    if (transfer == dev->transfer) {
      dev->transfer = nullptr;
    }
  }
}

void USBHIDXComponent::setup_media_interface(HIDDevice *dev, const usb_config_desc_t *config_desc) {
  ESP_LOGI(TAG, "Searching for secondary HID interface...");
  // Interface numbers are not required to be contiguous or to start at zero.
  const usb_intf_desc_t *intf_desc = nullptr;
  const usb_ep_desc_t *ep_desc = nullptr;
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
      if (temp_intf->bInterfaceNumber != dev->interface_num && temp_intf->bInterfaceClass == 0x03 &&
          intf_desc == nullptr) {
        intf_desc = temp_intf;
        ESP_LOGI(TAG, "Found secondary HID interface %d", temp_intf->bInterfaceNumber);
      }
    } else if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && intf_desc) {
      if (desc->bLength < sizeof(usb_ep_desc_t)) {
        ESP_LOGW(TAG, "Short media endpoint descriptor at offset %d", offset);
        break;
      }
      const usb_ep_desc_t *temp_ep = (const usb_ep_desc_t *) desc;
      if ((temp_ep->bEndpointAddress & 0x80) && ((temp_ep->bmAttributes & 0x03) == 0x03)) {
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
  if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGD(TAG, "LED command completed");
  } else {
    ESP_LOGW(TAG, "LED command failed: %d", transfer->status);
  }
  usb_host_transfer_free(transfer);
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
  ctrl_transfer->context = nullptr;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  ctrl_transfer->data_buffer[sizeof(usb_setup_packet_t)] = led_state;
  ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + 1;

  err = usb_host_transfer_submit_control(this->client_hdl_, ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit LED command: %s", esp_err_to_name(err));
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
  ctrl_transfer->context = nullptr;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  if (len != 0) {
    memcpy(ctrl_transfer->data_buffer + sizeof(usb_setup_packet_t), data, len);
  }
  ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + len;

  err = usb_host_transfer_submit_control(this->client_hdl_, ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit Xbox 360 command: %s", esp_err_to_name(err));
    usb_host_transfer_free(ctrl_transfer);
  }
}

void USBHIDXComponent::send_xbox360_rumble(uint8_t left_motor, uint8_t right_motor) {
#ifdef USB_HIDX_ENABLE_XBOX360
  if (!xbox360_driver_) {
    ESP_LOGW(TAG, "Xbox 360 driver not initialized");
    return;
  }
  xbox360_driver_->send_rumble(xbox360_device_, left_motor, right_motor);
#else
  ESP_LOGW(TAG, "Xbox 360 gamepad support not enabled");
#endif
}

void USBHIDXComponent::send_switch_rumble(uint8_t left_motor, uint8_t right_motor) {
#ifdef USB_HIDX_ENABLE_SWITCH
  if (!switch_driver_) {
    ESP_LOGW(TAG, "Switch driver not initialized");
    return;
  }
  switch_driver_->send_rumble(left_motor, right_motor);
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
  ctrl_transfer->context = nullptr;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + 49;

  err = usb_host_transfer_submit_control(this->client_hdl_, ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit GET_REPORT: %s", esp_err_to_name(err));
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
  out_transfer->context = nullptr;
  out_transfer->num_bytes = len;
  memcpy(out_transfer->data_buffer, data, len);

  err = usb_host_transfer_submit(out_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit OUT transfer: %s", esp_err_to_name(err));
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
  ctrl_transfer->context = nullptr;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  if (len != 0) {
    memcpy(ctrl_transfer->data_buffer + sizeof(usb_setup_packet_t), data, len);
  }
  ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + len;

  err = usb_host_transfer_submit_control(this->client_hdl_, ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit HID output: %s", esp_err_to_name(err));
    usb_host_transfer_free(ctrl_transfer);
    return err;
  }

  return ESP_OK;
}

}  // namespace usb_hidx
}  // namespace esphome
