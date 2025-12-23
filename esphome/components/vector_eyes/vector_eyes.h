#pragma once

#include "esphome/core/defines.h"

#include "esphome/core/component.h"
#include "esphome/components/display/display_buffer.h"
#include "esphome/components/speaker/speaker.h"
#include "esphome/components/font/font.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/hal.h"
#include "esphome/components/api/custom_api_device.h"

// Arduino SD library only needed for fallback when NOT using storage component
// (ESP-IDF framework doesn't have Arduino SD library)
#ifndef USE_STORAGE
#include <SD.h>
#include <SPI.h>
#endif

#include <unistd.h>
#include <vector>
#include <map>
#include <ArduinoJson.h>
#include "esphome/core/preferences.h"





#include "procedural_face.h"
#include "animation_player.h"
#include "animation_triggers.h"
#include "mood_manager.h"
#include "storage_adapter.h"

namespace esphome {

namespace storage {
class Storage;
}

namespace vector_eyes {

class BehaviorEngine;

class VectorEyes : public Component, public api::CustomAPIDevice {

 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_display(display::DisplayBuffer *display);
  void set_speaker(speaker::Speaker *speaker);
  
  // Storage configuration (NEW)
  void set_storage(storage::Storage *storage);
  void set_mount_path(const std::string &path);
  void set_font(font::Font *font) { font_ = font; }
  
  // DEPRECATED: SD card pin (kept for backward compatibility)
  void set_sd_cs_pin(GPIOPin *pin);

  // Animation control
  void play_animation(const std::string &name);
  void play_trigger(AnimationTrigger trigger);
  void play_anim_type(AnimationTrigger type);
  
  // Manual triggers (also update mood)
  void play_anim_happy();
  void play_anim_angry();
  void play_anim_awe();
  void play_anim_neutral();
  void play_anim_look_right();
  void play_anim_look_left();
  void play_blink();
  
  // Control methods
  void stop_animation();
  void reset_face();
  void look_at(float x, float y);
  void draw();
  void draw_clock();  // Draw clock display (time from ESPHome time component)
  void set_clock_mode(bool enabled) { 
      clock_mode_ = enabled; 
      if (enabled) {
          clock_show_start_time_ = millis();
      }
  }
  void set_24h_mode(bool enabled) { is_24h_mode_ = enabled; }
  void set_time(esphome::time::RealTimeClock *time) { time_ = time; }
  
  // Audio
  void play_zelda();
  void set_volume(float volume);
  float get_volume() const;
  void play_animation_from_file(const std::string &filename);
  void play_animation_from_json(const std::string &filename);
  void load_audio_mappings();
  
  // Settings
  void set_autonomous_mode(bool enabled);

 protected:
  esphome::time::RealTimeClock *time_{nullptr};
  display::DisplayBuffer *display_{nullptr};
  speaker::Speaker *speaker_{nullptr};
  
  // Storage integration (NEW)
  storage::Storage *storage_component_{nullptr};
  StorageAdapter *storage_adapter_{nullptr};
  BehaviorEngine *behavior_engine_{nullptr};
  font::Font *font_{nullptr};

  std::string mount_path_{"/sd"};

  
  // DEPRECATED: Direct SD card access (kept for backward compatibility)
  GPIOPin *sd_cs_pin_{nullptr};
  bool sd_card_initialized_{false};
  
  ProceduralFace face_;
  AnimationPlayer animation_player_;
  MoodManager mood_manager_;
  
  bool autonomous_mode_{true};
  bool clock_mode_{false};  // When true, display clock instead of eyes
  bool is_24h_mode_{false};
  uint32_t clock_show_start_time_{0};
  static constexpr uint32_t CLOCK_SHOW_DURATION = 5000;
  
  uint32_t boot_start_time_{0};
  static constexpr uint32_t MIN_BOOT_DURATION = 6000; // Force loading screen for at least 6s
  
  std::vector<AnimationKeyframe> dynamic_anim_buffer_;
  std::vector<std::string> sound_string_buffer_;
  std::map<std::string, std::string> audio_event_map_; // Maps audio event names to WAV files
  
  float volume_{1.0f};
  ESPPreferenceObject volume_pref_;
  bool save_pending_{false};
  uint32_t last_volume_change_time_{0};
  uint32_t last_behavior_time_{0};
  uint32_t next_behavior_delay_{5000};
  uint32_t hunting_complete_time_{0};
  uint32_t recalling_complete_time_{0};

  void apply_keyframe(ProceduralFace &face, const AnimationKeyframe &kf);
  void update_behavior();
  void play_wav_file(const std::string &filename);
  const char* get_animation_for_trigger(AnimationTrigger trigger);
#ifndef USE_STORAGE
  bool parse_json_animation(File &file);  // Arduino SD library only
#else
  bool parse_json_animation_from_buffer(std::vector<uint8_t> &buffer);
  bool parse_json_animation_doc(JsonDocument &doc);
#endif
  std::string map_audio_event_to_wav(const std::string &event_name);
  void match_audio_to_keyframes();
  
  // Helper methods for storage abstraction
  bool file_exists_any(const std::string &path);
  
  // Procedural Animation Fallback
  void trigger_procedural_blink();
  
  // Async Audio State
  void update_audio();
  void *audio_handle_{nullptr};
  bool audio_playing_{false};
  uint16_t audio_channels_{1};
  uint32_t audio_rate_{16000};
  uint16_t audio_bits_{16};
  uint32_t audio_data_size_{0};
  uint32_t audio_bytes_read_{0};
  
  // Audio Buffers
  uint8_t audio_raw_buffer_[1024];
  int16_t audio_sample_buffer_[512]; // 1024 bytes / 2
  size_t audio_buffer_valid_bytes_{0}; // Count of SAMPLES
  size_t audio_buffer_sent_bytes_{0};  // Count of SAMPLES
};

}  // namespace vector_eyes
}  // namespace esphome
