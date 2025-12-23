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

  if (dev_desc->bDeviceClass != 0x03 && dev_desc->bDeviceClass != 0x00) {
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
      if (temp_intf->bInterfaceClass == 0x03 && temp_intf->bInterfaceNumber == 0) {
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
  while (offset < config_desc->wTotalLength) {
    const usb_standard_desc_t *desc = (const usb_standard_desc_t *) ((uint8_t *) config_desc + offset);

    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
      ep_desc = (const usb_ep_desc_t *) desc;
      if ((ep_desc->bEndpointAddress & 0x80) && ((ep_desc->bmAttributes & 0x03) == 0x03)) {
        ESP_LOGI(TAG, "Found interrupt IN endpoint: 0x%02X", ep_desc->bEndpointAddress);
        break;
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

  if (!dev || !dev->active) {
    if (transfer)
      usb_host_transfer_free(transfer);
    return;
  }

  if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
    // Log which endpoint this came from
    ESP_LOGD(TAG, "Transfer from EP 0x%02X: %d bytes", transfer->bEndpointAddress, transfer->actual_num_bytes);

    // Log ALL 8-byte reports for debugging
    if (transfer->actual_num_bytes == 8) {
      ESP_LOGD(TAG, "RAW: [%02X %02X %02X %02X %02X %02X %02X %02X]", transfer->data_buffer[0],
               transfer->data_buffer[1], transfer->data_buffer[2], transfer->data_buffer[3], transfer->data_buffer[4],
               transfer->data_buffer[5], transfer->data_buffer[6], transfer->data_buffer[7]);
    }

    if (dev->driver) {
      // Check if this is from the media interface (0x82) or keyboard interface (0x81)
      bool is_media = (transfer == dev->media_transfer);
      if (is_media) {
        // Force media report processing by setting a flag in the data
        // We'll use a temporary buffer with a marker
        uint8_t temp_data[9];
        temp_data[0] = 0xFF;  // Marker for media report
        memcpy(&temp_data[1], transfer->data_buffer, 8);
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

}  // namespace usb_hidx
}  // namespace esphome
