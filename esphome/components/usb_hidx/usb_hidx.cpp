#include "usb_hidx.h"
#include "esphome/core/log.h"
#include "driver_registry.h"

namespace esphome {
namespace usb_hidx {

static const char *TAG = "usb_hidx";

void USBHIDXComponent::setup() {
  ESP_LOGI(TAG, "Setting up USB HIDX component");

  // Auto-register all available device drivers
  register_all_drivers(this);

  // Initialize device array
  for (int i = 0; i < 4; i++) {
    devices_[i].active = false;
  }

  // Register USB client
  usb_host_client_config_t client_config = {.is_synchronous = false,
                                            .max_num_event_msg = 5,
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

void USBHIDXComponent::register_keyboard_key_sensor(binary_sensor::BinarySensor *sensor, uint8_t keycode) {
  keyboard_key_sensors_[keycode] = sensor;
}

void USBHIDXComponent::loop() {
  if (this->client_hdl_) {
    esp_err_t err = usb_host_client_handle_events(this->client_hdl_, 0);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "USB client event error: %s", esp_err_to_name(err));
    }
  }
}

HIDDevice *USBHIDXComponent::find_device_by_handle(usb_device_handle_t dev_hdl) {
  for (int i = 0; i < 4; i++) {
    if (devices_[i].active && devices_[i].dev_hdl == dev_hdl) {
      return &devices_[i];
    }
  }
  return nullptr;
}

HIDDevice *USBHIDXComponent::find_device_by_transfer(usb_transfer_t *transfer) {
  for (int i = 0; i < 4; i++) {
    if (devices_[i].active && (devices_[i].transfer == transfer || devices_[i].media_transfer == transfer)) {
      return &devices_[i];
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

  int slot = -1;
  for (int i = 0; i < 4; i++) {
    if (!devices_[i].active) {
      slot = i;
      break;
    }
  }

  if (slot == -1) {
    ESP_LOGW(TAG, "No free device slots available");
    return;
  }

  HIDDevice *dev = &devices_[slot];
  dev->dev_addr = address;

  esp_err_t err = usb_host_device_open(this->client_hdl_, address, &dev->dev_hdl);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(err));
    return;
  }

  const usb_device_desc_t *dev_desc;
  err = usb_host_get_device_descriptor(dev->dev_hdl, &dev_desc);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get device descriptor");
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    return;
  }

  dev->vid = dev_desc->idVendor;
  dev->pid = dev_desc->idProduct;

  ESP_LOGI(TAG, "Device VID:PID = %04X:%04X", dev->vid, dev->pid);

  // Check for Xbox 360 devices (vendor-specific class 0xFF)
  bool is_xbox360 = (dev->vid == 0x045E && (dev->pid == 0x028E || dev->pid == 0x0719));

  if (dev_desc->bDeviceClass != 0x03 && dev_desc->bDeviceClass != 0x00 && !is_xbox360) {
    ESP_LOGD(TAG, "Not a HID device, ignoring");
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    return;
  }

  // Get config descriptor
  const usb_config_desc_t *config_desc;
  err = usb_host_get_active_config_descriptor(dev->dev_hdl, &config_desc);
  if (err != ESP_OK) {
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    return;
  }

  // Find HID interface
  const usb_intf_desc_t *intf_desc = nullptr;
  const usb_ep_desc_t *ep_desc = nullptr;
  int offset = 0;

  while (offset < config_desc->wTotalLength) {
    const usb_standard_desc_t *desc = (const usb_standard_desc_t *) ((uint8_t *) config_desc + offset);

    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      const usb_intf_desc_t *temp_intf = (const usb_intf_desc_t *) desc;
      // Accept HID class (0x03) or Xbox 360 vendor-specific (0xFF)
      if ((temp_intf->bInterfaceClass == 0x03 || (is_xbox360 && temp_intf->bInterfaceClass == 0xFF)) &&
          temp_intf->bInterfaceNumber == 0) {
        intf_desc = temp_intf;
        dev->protocol = intf_desc->bInterfaceProtocol;
        ESP_LOGI(TAG, "Found HID interface, protocol %d", dev->protocol);
        break;
      }
    }
    offset += desc->bLength;
  }

  if (!intf_desc) {
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    return;
  }

  // Find interrupt IN endpoint
  offset = (uint8_t *) intf_desc - (uint8_t *) config_desc + intf_desc->bLength;
  uint8_t out_ep = 0;
  while (offset < config_desc->wTotalLength) {
    const usb_standard_desc_t *desc = (const usb_standard_desc_t *) ((uint8_t *) config_desc + offset);

    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
      const usb_ep_desc_t *temp_ep = (const usb_ep_desc_t *) desc;
      if ((temp_ep->bEndpointAddress & 0x80) && ((temp_ep->bmAttributes & 0x03) == 0x03)) {
        ep_desc = temp_ep;
        ESP_LOGI(TAG, "Found interrupt IN endpoint: 0x%02X", ep_desc->bEndpointAddress);
      } else if (!(temp_ep->bEndpointAddress & 0x80) && ((temp_ep->bmAttributes & 0x03) == 0x03)) {
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
    return;
  }

  // Claim interface
  err = usb_host_interface_claim(this->client_hdl_, dev->dev_hdl, intf_desc->bInterfaceNumber, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to claim interface: %s", esp_err_to_name(err));
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    return;
  }

  dev->interface_num = intf_desc->bInterfaceNumber;
  dev->out_endpoint = out_ep;
  this->active_channels_++;

  // Allocate transfer
  usb_transfer_t *transfer;
  err = usb_host_transfer_alloc(ep_desc->wMaxPacketSize, 0, &transfer);
  if (err != ESP_OK) {
    usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->interface_num);
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    return;
  }

  transfer->device_handle = dev->dev_hdl;
  transfer->bEndpointAddress = ep_desc->bEndpointAddress;
  transfer->context = this;
  transfer->num_bytes = ep_desc->wMaxPacketSize;
  transfer->callback = USBHIDXComponent::transfer_callback;

  dev->transfer = transfer;
  dev->active = true;
  this->connected_devices_++;

  // Match device to driver
  for (auto *driver : this->drivers_) {
    if (driver->match_device(dev->protocol, dev->vid, dev->pid)) {
      dev->driver = driver;
      ESP_LOGI(TAG, "Matched device to %s driver", driver->get_name());
      // Store Xbox 360 device reference
      if (strcmp(driver->get_name(), "Xbox360") == 0) {
        this->xbox360_device_ = dev;
      }
      // Call on_device_ready for drivers that need initialization
      if (strcmp(driver->get_name(), "PlayStation") == 0) {
        auto *ps_driver = static_cast<PlayStationDriver *>(driver);
        ps_driver->on_device_ready(dev);
      } else if (strcmp(driver->get_name(), "Switch") == 0) {
        auto *switch_driver = static_cast<SwitchDriver *>(driver);
        switch_driver->on_device_ready(dev);
      }
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
  } else {
    ESP_LOGI(TAG, "Device monitoring started (protocol %d)", dev->protocol);

    // If keyboard, try to set up media interface (interface 1)
    if (dev->protocol == 0x01) {
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
  this->connected_devices_--;

  if (dev->transfer) {
    dev->transfer = nullptr;
  }

  if (dev->dev_hdl) {
    usb_host_interface_release(this->client_hdl_, dev->dev_hdl, dev->interface_num);
    usb_host_device_close(this->client_hdl_, dev->dev_hdl);
    dev->dev_hdl = nullptr;
  }

  ESP_LOGI(TAG, "Device removed, %d devices remaining", this->connected_devices_);
}

void USBHIDXComponent::transfer_callback(usb_transfer_t *transfer) {
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
    } else {
      // Generic idle check for other devices
      is_idle = (transfer->bEndpointAddress == 0x81 && transfer->actual_num_bytes == 8 &&
                 transfer->data_buffer[0] == 0 && transfer->data_buffer[1] == 0 && transfer->data_buffer[2] == 0x0F &&
                 transfer->data_buffer[3] == 0x80 && transfer->data_buffer[4] == 0x80 &&
                 transfer->data_buffer[5] == 0x80 && transfer->data_buffer[6] == 0x80 && transfer->data_buffer[7] == 0);
    }

    if (!is_idle) {
      // Log PlayStation transfers
      if (is_ps_device) {
        ESP_LOGI(TAG, "PlayStation transfer: status=%d, bytes=%d, EP=0x%02X", transfer->status,
                 transfer->actual_num_bytes, transfer->bEndpointAddress);
      }

      ESP_LOGD(TAG, "Transfer from EP 0x%02X: %d bytes", transfer->bEndpointAddress, transfer->actual_num_bytes);

      // Log raw data for all report sizes
      if (transfer->actual_num_bytes == 8) {
        ESP_LOGD(TAG, "RAW: [%02X %02X %02X %02X %02X %02X %02X %02X]", transfer->data_buffer[0],
                 transfer->data_buffer[1], transfer->data_buffer[2], transfer->data_buffer[3], transfer->data_buffer[4],
                 transfer->data_buffer[5], transfer->data_buffer[6], transfer->data_buffer[7]);
      } else if (transfer->actual_num_bytes == 5) {
        ESP_LOGD(TAG, "RAW: [%02X %02X %02X %02X %02X]", transfer->data_buffer[0], transfer->data_buffer[1],
                 transfer->data_buffer[2], transfer->data_buffer[3], transfer->data_buffer[4]);
      } else if (transfer->actual_num_bytes > 0) {
        // Log any other size
        std::string hex_str = "RAW: [";
        for (int i = 0; i < transfer->actual_num_bytes; i++) {
          char buf[8];
          snprintf(buf, sizeof(buf), "%02X%s", transfer->data_buffer[i], i < transfer->actual_num_bytes - 1 ? " " : "");
          hex_str += buf;
        }
        hex_str += "]";
        ESP_LOGD(TAG, "%s", hex_str.c_str());
      }
    }

    if (dev->driver) {
      // Check if this is from the media interface (0x82) or keyboard interface (0x81)
      bool is_media = (transfer == dev->media_transfer);
      if (is_media) {
        // Force media report processing by setting a flag in the data
        // We'll use a temporary buffer with a marker
        uint8_t temp_data[65];  // Max HID report size + 1 for marker
        temp_data[0] = 0xFF;    // Marker for media report
        memcpy(&temp_data[1], transfer->data_buffer, transfer->actual_num_bytes);
        dev->driver->process_report(temp_data, transfer->actual_num_bytes + 1, dev);
      } else {
        dev->driver->process_report(transfer->data_buffer, transfer->actual_num_bytes, dev);
      }
    }
  }

  if (dev->active && usb_host_transfer_submit(transfer) != ESP_OK) {
    usb_host_transfer_free(transfer);
    dev->transfer = nullptr;
  }
}

void USBHIDXComponent::setup_media_interface(HIDDevice *dev, const usb_config_desc_t *config_desc) {
  ESP_LOGI(TAG, "Searching for media interface (interface 1)...");
  // Find interface 1 for media keys
  const usb_intf_desc_t *intf_desc = nullptr;
  const usb_ep_desc_t *ep_desc = nullptr;
  int offset = 0;

  while (offset < config_desc->wTotalLength) {
    const usb_standard_desc_t *desc = (const usb_standard_desc_t *) ((uint8_t *) config_desc + offset);
    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      const usb_intf_desc_t *temp_intf = (const usb_intf_desc_t *) desc;
      ESP_LOGD(TAG, "Found interface %d: Class=0x%02X", temp_intf->bInterfaceNumber, temp_intf->bInterfaceClass);
      if (temp_intf->bInterfaceNumber == 1 && temp_intf->bInterfaceClass == 0x03) {
        intf_desc = temp_intf;
        ESP_LOGI(TAG, "Found media interface 1");
      }
    } else if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && intf_desc) {
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

  esp_err_t err = usb_host_interface_claim(this->client_hdl_, dev->dev_hdl, 1, 0);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to claim media interface: %s", esp_err_to_name(err));
    return;
  }

  usb_transfer_t *transfer;
  err = usb_host_transfer_alloc(ep_desc->wMaxPacketSize, 0, &transfer);
  if (err != ESP_OK) {
    usb_host_interface_release(this->client_hdl_, dev->dev_hdl, 1);
    return;
  }

  transfer->device_handle = dev->dev_hdl;
  transfer->bEndpointAddress = ep_desc->bEndpointAddress;
  transfer->context = this;
  transfer->num_bytes = ep_desc->wMaxPacketSize;
  transfer->callback = USBHIDXComponent::transfer_callback;

  dev->media_transfer = transfer;

  err = usb_host_transfer_submit(transfer);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Media key monitoring started on endpoint 0x%02X", ep_desc->bEndpointAddress);
  } else {
    ESP_LOGW(TAG, "Failed to submit media transfer: %s", esp_err_to_name(err));
    usb_host_transfer_free(transfer);
    usb_host_interface_release(this->client_hdl_, dev->dev_hdl, 1);
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

  usb_setup_packet_t setup_pkt = {.bmRequestType = 0x21, .bRequest = 0x09, .wValue = 0x0200, .wIndex = 0, .wLength = 1};

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

  usb_transfer_t *ctrl_transfer;
  esp_err_t err = usb_host_transfer_alloc(64, 0, &ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to allocate Xbox 360 transfer");
    return;
  }

  usb_setup_packet_t setup_pkt = {.bmRequestType = 0x21,  // Host-to-device, Class, Interface
                                  .bRequest = 0x09,       // SET_REPORT
                                  .wValue = 0x0200,       // Output report
                                  .wIndex = 0,            // Interface 0
                                  .wLength = (uint16_t) len};

  ctrl_transfer->device_handle = device->dev_hdl;
  ctrl_transfer->callback = USBHIDXComponent::led_control_callback;
  ctrl_transfer->context = nullptr;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  memcpy(ctrl_transfer->data_buffer + sizeof(usb_setup_packet_t), data, len);
  ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + len;

  err = usb_host_transfer_submit_control(this->client_hdl_, ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to submit Xbox 360 command: %s", esp_err_to_name(err));
    usb_host_transfer_free(ctrl_transfer);
  }
}

void USBHIDXComponent::send_xbox360_rumble(uint8_t left_motor, uint8_t right_motor) {
  if (!xbox360_driver_) {
    ESP_LOGW(TAG, "Xbox 360 driver not initialized");
    return;
  }
  xbox360_driver_->send_rumble(xbox360_device_, left_motor, right_motor);
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
                                  .wIndex = 0,
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

  usb_transfer_t *ctrl_transfer;
  esp_err_t err = usb_host_transfer_alloc(len + sizeof(usb_setup_packet_t), 0, &ctrl_transfer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to allocate HID output transfer");
    return err;
  }

  usb_setup_packet_t setup_pkt = {.bmRequestType = 0x21,  // Host-to-device, Class, Interface
                                  .bRequest = 0x09,       // SET_REPORT
                                  .wValue = 0x0200,       // Output report
                                  .wIndex = 0,            // Interface 0
                                  .wLength = (uint16_t) len};

  ctrl_transfer->device_handle = device->dev_hdl;
  ctrl_transfer->callback = USBHIDXComponent::led_control_callback;
  ctrl_transfer->context = nullptr;
  memcpy(ctrl_transfer->data_buffer, &setup_pkt, sizeof(usb_setup_packet_t));
  memcpy(ctrl_transfer->data_buffer + sizeof(usb_setup_packet_t), data, len);
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
