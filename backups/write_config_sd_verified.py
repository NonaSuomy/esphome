
import os

content = r"""substitutions:
  # Wifi
  wifi_ssid: !secret wifi_ssid2
  wifi_password: !secret wifi_password2
  use_address: !secret use_address_wifi015
  
  # TTGO Camera Plus Pins
  # SPI
  spi_clk: GPIO21
  spi_mosi: GPIO19
  spi_miso: GPIO22
  
  # Display (ST7789)
  dis_cs: GPIO12
  dis_dc: GPIO15
  dis_bk: GPIO2
  
  # Camera (Not used by Vector Eyes but pins reserved)
  cam_xclk: GPIO4
  cam_sda: GPIO18
  cam_scl: GPIO23
  
  # Audio
  # Mic: WS=32, SCK=14, SD=33
  # Speaker: We will reuse WS/SCK and use GPIO16 for DIN (RGB LED pin)
  # User must connect I2S DAC DIN to GPIO16, LRC to GPIO32, BCLK to GPIO14
  mic_ws: GPIO32
  mic_sck: GPIO14
  mic_sd: GPIO33
  
  # Speaker (I2S)
  spk_din: GPIO16
  
  # SD Card (SPI CS)
  sd_cs: GPIO0 # Correct pin per TTGO Camera Plus datasheet 

esphome:
  name: vector-eyes-ttgo
  friendly_name: Vector Eyes TTGO
  includes:
    - includes.h
  platformio_options:
    board_build.partitions: /home/nonasuomy/code/esphome003/esphome/huge_app.csv

  on_boot:
    priority: 1200 # Run BEFORE SPI bus init (1000)
    then:
      - lambda: |-
          // Deselect display specific to TTGO Camera Plus using ESP-IDF calls
          gpio_config_t io_conf = {};
          io_conf.intr_type = GPIO_INTR_DISABLE;
          io_conf.mode = GPIO_MODE_OUTPUT;
          io_conf.pin_bit_mask = (1ULL << 12) | (1ULL << 15);
          io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
          io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
          gpio_config(&io_conf);
          
          gpio_set_level(GPIO_NUM_12, 1);
          gpio_set_level(GPIO_NUM_15, 1);

esp32:
  board: esp32dev
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_FATFS_LFN_HEAP: "y"
      CONFIG_FATFS_MAX_LFN: "255"
  
psram:
  mode: quad
  speed: 80MHz

logger:
  level: DEBUG

api:
  services:
    - service: play_animation
      variables:
        name: string
      then:
        - lambda: 'id(my_vector_eyes).play_animation(name);'
    - service: look_at
      variables:
        x: float
        y: float
      then:
        - lambda: 'id(my_vector_eyes).look_at(x, y);'
    - service: reset_face
      then:
        - lambda: 'id(my_vector_eyes).reset_face();'
  encryption:
    key: !secret encryption_key015

ota:
  - platform: esphome
    password: !secret ota_pass015

wifi:
  ssid: ${wifi_ssid}
  password: ${wifi_password}
  # use_address: ${use_address} # Optional static IP

# I2C (Shared with Camera/Sensors)
i2c:
  sda: ${cam_sda}
  scl: ${cam_scl}
  scan: true
  id: bus_a

# SPI for Display
spi:
  clk_pin: ${spi_clk}
  mosi_pin: ${spi_mosi}
  miso_pin: ${spi_miso}
  interface: hardware
  
# Display Backlight
output:
  - platform: ledc
    pin: ${dis_bk}
    id: gpio_dis_bk_backlight_pwm

light:
  - platform: monochromatic
    output: gpio_dis_bk_backlight_pwm
    name: "Display Backlight"
    id: back_light
    restore_mode: ALWAYS_ON

# Display
display:
  - platform: ili9xxx
    model: st7789v
    id: display1
    dimensions:
      height: 240
      width: 240
      offset_width: 0
      offset_height: 0
    invert_colors: true 
    cs_pin: ${dis_cs}
    dc_pin: ${dis_dc}
    
    # Performance settings
    update_interval: never # Stop auto-updates to prevent error flood
    data_rate: 40MHz # High speed SPI
    
    # lambda: |-
    #   id(my_vector_eyes).draw();

# Storage Component (replaces direct SD card access)
storage:
  id: main_storage

# SD Card Storage Device (SPI mode for TTGO Camera Plus)
# Uses the SPI bus defined above (CLK=21, MOSI=19, MISO=22)
sd_storage:
  type: sd_spi
  id: sd_card
  cs_pin: ${sd_cs}
  path: "/sd"
  frequency: 400kHz

interval:
  - interval: 10s
    then:
      - lambda: |-
          static bool tested = false;
          if (!tested && id(sd_card).is_mounted()) {
            tested = true;
            ESP_LOGI("json_test", "Reading JSON file from SD card...");
            
            // Open and read the JSON file
            FILE *f = fopen("/sd/audio_mappings.json", "r");
            if (f) {
              // Get file size
              fseek(f, 0, SEEK_END);
              long size = ftell(f);
              fseek(f, 0, SEEK_SET);
              
              ESP_LOGI("json_test", "File size: %ld bytes", size);
              
              // Read first 512 bytes (or less if file is smaller)
              char buffer[513] = {0};
              size_t bytes_to_read = (size > 512) ? 512 : size;
              size_t bytes_read = fread(buffer, 1, bytes_to_read, f);
              fclose(f);
              
              ESP_LOGI("json_test", "Read %d bytes from file", (int)bytes_read);
              ESP_LOGI("json_test", "Content preview: %.200s", buffer);
              
              // Parse JSON using ArduinoJson
              JsonDocument doc;
              DeserializationError error = deserializeJson(doc, buffer);
              
              if (error) {
                ESP_LOGE("json_test", "JSON parse error: %s", error.c_str());
              } else {
                ESP_LOGI("json_test", "JSON parsed successfully!");
                // Log some keys if they exist
                if (doc.containsKey("version")) {
                  ESP_LOGI("json_test", "  version: %s", doc["version"].as<const char*>());
                }
                if (doc.is<JsonArray>()) {
                  ESP_LOGI("json_test", "  Array with %d elements", doc.size());
                }
              }
            } else {
              ESP_LOGE("json_test", "Failed to open /sd/audio_mappings.json");
            }
          }

# Audio
i2s_audio:
  - id: i2s_bus
    i2s_lrclk_pin: ${mic_ws}
    i2s_bclk_pin: ${mic_sck}

speaker:
  - platform: i2s_audio
    id: vector_speaker
    i2s_audio_id: i2s_bus
    dac_type: external
    i2s_dout_pin: ${spk_din}

# Vector Eyes Component
vector_eyes:
  id: my_vector_eyes
  display_id: display1
  speaker_id: vector_speaker
  storage_id: main_storage
  mount_path: "/sd"

# Controls
switch:
  - platform: template
    name: "Autonomous Mode"
    id: autonomous_mode_switch
    restore_mode: RESTORE_DEFAULT_ON
    optimistic: true
    turn_on_action:
      - lambda: 'id(my_vector_eyes).set_autonomous_mode(true);'
    turn_off_action:
      - lambda: 'id(my_vector_eyes).set_autonomous_mode(false);'

button:
  - platform: template
    name: "Play Blink"
    on_press:
      - lambda: 'id(my_vector_eyes).play_blink();'
  - platform: template
    name: "Play Happy"
    on_press:
      - lambda: 'id(my_vector_eyes).play_anim_happy();'
  - platform: template
    name: "Play Angry"
    on_press:
      - lambda: 'id(my_vector_eyes).play_anim_angry();'
  - platform: template
    name: "Play Awe"
    on_press:
      - lambda: 'id(my_vector_eyes).play_anim_awe();'
  - platform: template
    name: "Look Right"
    on_press:
      - lambda: 'id(my_vector_eyes).play_anim_look_right();'
  - platform: template
    name: "Look Left"
    on_press:
      - lambda: 'id(my_vector_eyes).play_anim_look_left();'
  - platform: template
    name: "Reset Eyes"
    on_press:
      - lambda: 'id(my_vector_eyes).reset_face();'

number:
  - platform: template
    name: "Volume"
    id: vector_volume
    min_value: 0
    max_value: 100
    step: 1
    lambda: "return id(my_vector_eyes).get_volume() * 100.0;"
    set_action:
      - lambda: 'id(my_vector_eyes).set_volume(x / 100.0);'
"""

with open("/home/nonasuomy/code/esphome003/esphome/config/vector-eyes-ttgo.yaml", "w") as f:
    f.write(content)
