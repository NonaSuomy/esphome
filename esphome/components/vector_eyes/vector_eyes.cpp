#include "vector_eyes.h"
#include "behavior_engine.h"
#include "esphome/core/log.h"
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>

// ESP-IDF GPIO for SPI bus CS management
#ifdef USE_ESP_IDF
#include <driver/gpio.h>
#endif
#include "esphome/components/time/real_time_clock.h"
#include "clock_sprites.h"

namespace esphome {
namespace vector_eyes {



static const char *TAG = "vector_eyes";

// Fix setup: instantiate behavior engine first
void VectorEyes::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Vector Eyes...");
  
  // Initialize Behavior Engine
  this->behavior_engine_ = new BehaviorEngine();

  // NEW: Try to initialize storage component first
  bool storage_available = false;
#ifdef USE_STORAGE
  if (this->storage_component_ != nullptr) {
      ESP_LOGI(TAG, "Initializing storage component... (ptr=%p)", this->storage_component_);
      
      this->storage_adapter_ = new StorageAdapter(this);
      
      // Set preferred mount path if configured
      if (!this->mount_path_.empty()) {
          this->storage_adapter_->set_preferred_mount_path(this->mount_path_);
      }
      
      // Initialize storage adapter (may fail if NFS not mounted yet)
      if (this->storage_adapter_->initialize(this->storage_component_)) {
          ESP_LOGI(TAG, "Storage adapter initialized successfully");
          storage_available = true;
      } else {
          ESP_LOGW(TAG, "Storage adapter initialization failed, will retry lazily in loop");
      }
      
      // ALWAYS pass storage adapter to behavior engine, even if init failed
      // BehaviorEngine::loop() will retry loading maps when storage becomes available
      this->behavior_engine_->setup(this->storage_adapter_);
      
      if (storage_available) {
          // Try loading maps now if storage is ready
          this->behavior_engine_->load_maps();
          
          // Load audio mappings from storage
          this->load_audio_mappings();
      }
  } else {
      ESP_LOGD(TAG, "No storage component configured, will use SD card if available");
  }
#endif
  
#ifndef USE_STORAGE
  // FALLBACK: Try direct SD card initialization if storage not available
  // (Only available with Arduino framework, not ESP-IDF)
  if (!storage_available && this->sd_cs_pin_ != nullptr) {
      this->sd_cs_pin_->setup();
      int pin_no = ((esphome::InternalGPIOPin *)this->sd_cs_pin_)->get_pin();
      ESP_LOGI(TAG, "Initializing SD card on CS pin %d (fallback mode)...", pin_no);
      ESP_LOGI(TAG, "Using ESPHome's SPI bus (CLK=21, MISO=22, MOSI=19)");
      
      // Try SD init with 4MHz (don't call SPI.begin() - ESPHome already did it)
      ESP_LOGI(TAG, "Trying SD init with 4MHz...");
      if (SD.begin(pin_no, SPI, 4000000)) {
          ESP_LOGI(TAG, "SD Card initialized at 4MHz");
          this->sd_card_initialized_ = true;
          
          // Get card info
          uint8_t cardType = SD.cardType();
          if (cardType == CARD_NONE) {
              ESP_LOGE(TAG, "No SD card attached");
              this->sd_card_initialized_ = false;
          } else {
              // Print card type
              const char* type_str = "UNKNOWN";
              if (cardType == CARD_MMC) type_str = "MMC";
              else if (cardType == CARD_SD) type_str = "SDSC";
              else if (cardType == CARD_SDHC) type_str = "SDHC";
              
              uint64_t cardSize = SD.cardSize() / (1024 * 1024);
              ESP_LOGI(TAG, "SD Card Type: %s, Size: %lluMB", type_str, cardSize);
              
              // Try to load audio mappings (will check file existence internally)
              load_audio_mappings();
          }
      } else {
          ESP_LOGE(TAG, "SD Card initialization failed");
          ESP_LOGE(TAG, "Check: 1) SD card inserted, 2) FAT32 formatted, 3) Wiring correct");
      }
  }
#endif
  
  // Log final storage status
  if (storage_available) {
      ESP_LOGI(TAG, "Using storage component for file access");
  } else if (this->sd_card_initialized_) {
      ESP_LOGI(TAG, "Using direct SD card access (fallback mode)");
  } else {
      ESP_LOGW(TAG, "No storage available, using internal animations only");
  }

  this->animation_player_.set_sound_callback([this](const char* sound) {
      if (sound) {
          std::string s(sound); // Copy to string
          this->play_wav_file(s);
      }
  });
  
  // Initialize with neutral face
  // this->face_.set_eye_color(0.0f, 1.0f, 0.6f); // Cyan - method missing currently
  
  // Load volume
  this->volume_pref_ = global_preferences->make_preference<float>(0x45594553); // 'EYES'
  float loaded_volume;
  if (this->volume_pref_.load(&loaded_volume)) {
      this->volume_ = loaded_volume;
      ESP_LOGI(TAG, "Loaded volume: %.2f", this->volume_);
  }
  
  this->reset_face();
  this->boot_start_time_ = millis();
}

void VectorEyes::dump_config() {
  ESP_LOGCONFIG(TAG, "Vector Eyes:");
  
  // Storage status
#ifdef USE_STORAGE
  if (this->storage_adapter_ && this->storage_adapter_->is_available()) {
      ESP_LOGCONFIG(TAG, "  Storage: Available (via storage component)");
      if (!this->mount_path_.empty()) {
          ESP_LOGCONFIG(TAG, "    Mount Path: %s", this->mount_path_.c_str());
      }
  } else
#endif
  if (this->sd_card_initialized_) {
      ESP_LOGCONFIG(TAG, "  Storage: Available (direct SD card - fallback mode)");
  } else {
      ESP_LOGCONFIG(TAG, "  Storage: Not available (using internal animations)");
  }
  
  ESP_LOGCONFIG(TAG, "  Volume: %.2f", this->volume_);
}

#ifndef USE_STORAGE
// Arduino SD library fallback (only available with Arduino framework)
void VectorEyes::play_animation_from_file(const std::string &filename) {
    if (!this->sd_card_initialized_) {
        ESP_LOGD(TAG, "SD Card not initialized, cannot play %s", filename.c_str());
        return;
    }

    // Arduino SD library: use filename without leading slash
    File file = SD.open(filename.c_str(), FILE_READ);
    if (!file) {
        ESP_LOGD(TAG, "Could not open CSV animation: %s", filename.c_str());
        return;
    }
    
    ESP_LOGI(TAG, "Opened CSV animation: %s (%d bytes)", filename.c_str(), file.size());

    // Clear existing buffers
    this->dynamic_anim_buffer_.clear();
    this->sound_string_buffer_.clear();

    // Skip header line
    if (file.available()) {
        String line = file.readStringUntil('\n'); 
    }

    while (file.available()) {
        String line_str = file.readStringUntil('\n');
        line_str.trim();
        if (line_str.length() == 0) continue;

        AnimationKeyframe kf;
        
        // Simple manual CSV parsing
        int pos = 0;
        int next_pos = 0;
        
        auto get_next_float = [&](float &val) {
            next_pos = line_str.indexOf(',', pos);
            if (next_pos == -1) next_pos = line_str.length();
            String sub = line_str.substring(pos, next_pos);
            val = sub.toFloat();
            pos = next_pos + 1;
        };
        auto get_next_int = [&](uint32_t &val) {
            next_pos = line_str.indexOf(',', pos);
            if (next_pos == -1) next_pos = line_str.length();
            String sub = line_str.substring(pos, next_pos);
            val = sub.toInt();
            pos = next_pos + 1;
        };

        // Format: time, dur, sx, sy, ang, cx, cy, llt, llb, rlt, rlb, sound
        // Indices: 0     1    2   3   4    5   6   7    8    9    10   11
        
        get_next_int(kf.trigger_time);
        get_next_int(kf.duration);
        get_next_float(kf.face.scale_x);
        get_next_float(kf.face.scale_y);
        get_next_float(kf.face.angle);
        get_next_float(kf.face.center_x);
        get_next_float(kf.face.center_y);
        get_next_float(kf.face.l_lid_top);
        get_next_float(kf.face.l_lid_bottom);
        get_next_float(kf.face.r_lid_top);
        get_next_float(kf.face.r_lid_bottom);
        
        // Sound is last (11)
        next_pos = line_str.indexOf(',', pos); // Should be -1 or end
        String sound_val;
        if (pos < line_str.length()) {
             sound_val = line_str.substring(pos);
             sound_val.trim();
        }
        
        if (sound_val.length() > 0 && sound_val != "nullptr") {
            this->sound_string_buffer_.push_back(sound_val.c_str());
            kf.sound_name = this->sound_string_buffer_.back().c_str();
        } else {
            kf.sound_name = nullptr;
        }
        
        this->dynamic_anim_buffer_.push_back(kf);
    }
    
    file.close();
    
    ESP_LOGI(TAG, "Loaded %d frames from %s", this->dynamic_anim_buffer_.size(), filename.c_str());
    if (!this->dynamic_anim_buffer_.empty()) {
        this->animation_player_.play(this->dynamic_anim_buffer_.data(), this->dynamic_anim_buffer_.size());
    }
}
#endif // USE_STORAGE

void VectorEyes::play_animation_from_json(const std::string &filename) {
#ifdef USE_STORAGE
    // NEW: Try storage adapter first
    if (this->storage_adapter_ && this->storage_adapter_->is_available()) {
        std::string path = filename;
        // Only prepend /animations/ if filename is bare (no slashes)
        // This allows passing full paths like "assets/animations/foo.json"
        if (filename.find('/') == std::string::npos) {
             path = "/animations/" + filename;
        }
        
        // Check if file exists
        if (!this->storage_adapter_->file_exists(path)) {
            ESP_LOGD(TAG, "Animation file not found via storage: %s", path.c_str());
            return;
        }

        // STREAMING PARSING STRATEGY
        // Instead of reading the whole file into RAM (which causes OOM for 26KB+ files),
        // we stream it directly into JsonDocument.
        
        ESP_LOGD(TAG, "Opening animation stream: %s", path.c_str());
        StorageAdapterStream stream(this->storage_adapter_, path);
        
        if (!stream.isOpen()) {
            ESP_LOGW(TAG, "Failed to open stream for: %s", path.c_str());
            return;
        }
        
        JsonDocument doc;
        // deserializeJson will read from stream as needed
        DeserializationError error = deserializeJson(doc, stream);
        
        if (error) {
            ESP_LOGE(TAG, "Failed to parse JSON animation from stream: %s", error.c_str());
            return;
        }
        
        if (parse_json_animation_doc(doc)) {
            ESP_LOGI(TAG, "Loaded animation via stream: %s", path.c_str());
            if (!this->dynamic_anim_buffer_.empty()) {
                this->animation_player_.play(this->dynamic_anim_buffer_.data(), this->dynamic_anim_buffer_.size());
            }
        }
    }
#endif
    
#ifndef USE_ESP_IDF
    // FALLBACK: SD library directly
    File file = SD.open(filename.c_str());
    if (file) {
        bool success = parse_json_animation(file);
        file.close();
        if (success && !this->dynamic_anim_buffer_.empty()) {
             this->animation_player_.play(this->dynamic_anim_buffer_.data(), this->dynamic_anim_buffer_.size());
        }
    }
#endif
}

#ifdef USE_STORAGE
bool VectorEyes::parse_json_animation_doc(JsonDocument &doc) {
    // Clear existing buffers
    this->dynamic_anim_buffer_.clear();
    this->sound_string_buffer_.clear();

    // Get the animation array (first key in the object)
    JsonArray anim_array;
    for (JsonPair kv : doc.as<JsonObject>()) {
        anim_array = kv.value().as<JsonArray>();
        break; // Take the first (and usually only) key
    }
    
    if (anim_array.isNull()) {
        ESP_LOGE(TAG, "No animation array found in JSON");
        return false;
    }
    
    // Memory Optimization: Reserve vectors
    size_t count = anim_array.size();
    ESP_LOGI(TAG, "Found %d keyframes in JSON", count);
    
    // Cap reservation to 50 items to avoid allocating huge contiguous blocks (OOM protection)
    // The vector will grow if needed, but in smaller steps
    size_t safe_reserve = count > 50 ? 50 : count;
    this->dynamic_anim_buffer_.reserve(safe_reserve);
    
    // Heuristic: assume ~10% have audio
    this->sound_string_buffer_.reserve(safe_reserve / 10 + 5); 

    // First pass: collect audio events
    struct AudioEvent {
        uint32_t trigger_time;
        std::string event_name;
        std::string wav_file;
    };
    std::vector<AudioEvent> audio_events;
    audio_events.reserve(safe_reserve / 10 + 5);
    

                

    
    // Parse each keyframe
    for (JsonObject keyframe : anim_array) {
        const char* name = keyframe["Name"] | "";
        
        // Process ProceduralFaceKeyFrame entries
        if (strcmp(name, "ProceduralFaceKeyFrame") == 0) {
            AnimationKeyframe kf;
            
            kf.trigger_time = keyframe["triggerTime_ms"] | 0;
            kf.duration = keyframe["durationTime_ms"] | 0;
            
            // Face scale and position
            // Face scale and position (scaled by 1000 for int16_t storage)
            kf.face.scale_x = (int16_t)((keyframe["faceScaleX"] | 1.0f) * ProceduralFace::SCALE);
            kf.face.scale_y = (int16_t)((keyframe["faceScaleY"] | 1.0f) * ProceduralFace::SCALE);
            kf.face.angle = (int16_t)((keyframe["faceAngle"] | 0.0f) * ProceduralFace::SCALE);
            kf.face.center_x = (int16_t)((keyframe["faceCenterX"] | 0.0f) * ProceduralFace::SCALE);
            kf.face.center_y = (int16_t)((keyframe["faceCenterY"] | 0.0f) * ProceduralFace::SCALE);
            
            // Left eye array - extract lid values
            JsonArray leftEye = keyframe["leftEye"];
            if (!leftEye.isNull() && leftEye.size() >= 8) {
                kf.face.l_lid_top = (int16_t)((leftEye[5] | 0.5f) * ProceduralFace::SCALE);
                kf.face.l_lid_bottom = (int16_t)((leftEye[6] | 0.5f) * ProceduralFace::SCALE);
            } else {
                kf.face.l_lid_top = 0;
                kf.face.l_lid_bottom = 0;
            }
            
            // Right eye array
            JsonArray rightEye = keyframe["rightEye"];
            if (!rightEye.isNull() && rightEye.size() >= 8) {
                kf.face.r_lid_top = (int16_t)((rightEye[5] | 0.5f) * ProceduralFace::SCALE);
                kf.face.r_lid_bottom = (int16_t)((rightEye[6] | 0.5f) * ProceduralFace::SCALE);
            } else {
                kf.face.r_lid_top = 0;
                kf.face.r_lid_bottom = 0;
            }
            
            kf.sound_name = nullptr; // Will be assigned during audio matching
            
            this->dynamic_anim_buffer_.push_back(kf);
        }
        // Handle audio keyframes
        else if (strcmp(name, "RobotAudioKeyFrame") == 0) {
            uint32_t trigger_time = keyframe["triggerTime_ms"] | 0;
            
            // Extract audio event names from eventGroups
            JsonArray eventGroups = keyframe["eventGroups"];
            if (!eventGroups.isNull() && eventGroups.size() > 0) {
                for (JsonObject group : eventGroups) {
                    JsonArray audioNames = group["audioName"];
                    if (!audioNames.isNull() && audioNames.size() > 0) {
                        for (JsonVariant audioNameVar : audioNames) {
                            const char* audioName = audioNameVar.as<const char*>();
                            if (audioName && strlen(audioName) > 0) {
                                // Map audio event to WAV file
                                std::string wav_file = map_audio_event_to_wav(audioName);
                                if (!wav_file.empty()) {
                                    AudioEvent evt;
                                    evt.trigger_time = trigger_time;
                                    evt.event_name = audioName;
                                    evt.wav_file = wav_file;
                                    audio_events.push_back(evt);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    if (this->dynamic_anim_buffer_.empty()) {
        ESP_LOGW(TAG, "No procedural face keyframes found");
        return false;
    }
    
    // Second pass: match audio events to closest visual keyframes (100ms tolerance)
    for (const auto& audio : audio_events) {
        int closest_idx = -1;
        uint32_t min_distance = 100; // 100ms tolerance
        
        for (size_t i = 0; i < this->dynamic_anim_buffer_.size(); i++) {
            uint32_t kf_time = this->dynamic_anim_buffer_[i].trigger_time;
            uint32_t distance = (audio.trigger_time > kf_time) ? 
                (audio.trigger_time - kf_time) : (kf_time - audio.trigger_time);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        if (closest_idx >= 0) {
             this->sound_string_buffer_.push_back(audio.wav_file);
             this->dynamic_anim_buffer_[closest_idx].sound_name = this->sound_string_buffer_.back().c_str();
        }
    }
    return true;
}

bool VectorEyes::parse_json_animation_from_buffer(std::vector<uint8_t> &buffer) {
    if (buffer.empty()) {
        ESP_LOGE(TAG, "Empty JSON buffer");
        return false;
    }

    JsonDocument doc;
    // Cast to char* (mutable) and length - ArduinoJson will use zero-copy mode
    DeserializationError error = deserializeJson(doc, (char*)buffer.data(), buffer.size());
    
    if (error) {
        ESP_LOGE(TAG, "JSON parse error: %s", error.c_str());
        return false;
    }
    
    return parse_json_animation_doc(doc);
}
#endif

#ifndef USE_STORAGE
// Arduino SD library fallback (only available with Arduino framework)
bool VectorEyes::parse_json_animation(File &file) {
    // Clear existing buffers
    this->dynamic_anim_buffer_.clear();
    this->sound_string_buffer_.clear();
    
    // Use streaming parser for large files
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    
    if (error) {
        ESP_LOGE(TAG, "JSON parse error: %s", error.c_str());
        return false;
    }
    
    // Get the animation array (first key in the object)
    JsonArray anim_array;
    for (JsonPair kv : doc.as<JsonObject>()) {
        anim_array = kv.value().as<JsonArray>();
        break; // Take the first (and usually only) key
    }
    
    if (anim_array.isNull()) {
        ESP_LOGE(TAG, "No animation array found in JSON");
        return false;
    }
    
    ESP_LOGD(TAG, "Found %d keyframes in JSON", anim_array.size());
    
    // First pass: collect audio events with their trigger times
    struct AudioEvent {
        uint32_t trigger_time;
        std::string event_name;
        std::string wav_file;
    };
    std::vector<AudioEvent> audio_events;
    
    // Parse each keyframe
    for (JsonObject keyframe : anim_array) {
        const char* name = keyframe["Name"] | "";
        
        // Process ProceduralFaceKeyFrame entries
        if (strcmp(name, "ProceduralFaceKeyFrame") == 0) {
            AnimationKeyframe kf;
            
            kf.trigger_time = keyframe["triggerTime_ms"] | 0;
            kf.duration = keyframe["durationTime_ms"] | 0;
            
            // Face scale and position
            kf.face.scale_x = keyframe["faceScaleX"] | 1.0f;
            kf.face.scale_y = keyframe["faceScaleY"] | 1.0f;
            kf.face.angle = keyframe["faceAngle"] | 0.0f;
            kf.face.center_x = keyframe["faceCenterX"] | 0.0f;
            kf.face.center_y = keyframe["faceCenterY"] | 0.0f;
            
            // Left eye array - extract lid values
            JsonArray leftEye = keyframe["leftEye"];
            if (!leftEye.isNull() && leftEye.size() >= 8) {
                kf.face.l_lid_top = leftEye[5] | 0.5f;
                kf.face.l_lid_bottom = leftEye[6] | 0.5f;
            } else {
                kf.face.l_lid_top = 0.0f;
                kf.face.l_lid_bottom = 0.0f;
            }
            
            // Right eye array
            JsonArray rightEye = keyframe["rightEye"];
            if (!rightEye.isNull() && rightEye.size() >= 8) {
                kf.face.r_lid_top = rightEye[5] | 0.5f;
                kf.face.r_lid_bottom = rightEye[6] | 0.5f;
            } else {
                kf.face.r_lid_top = 0.0f;
                kf.face.r_lid_bottom = 0.0f;
            }
            
            kf.sound_name = nullptr; // Will be assigned during audio matching
            
            this->dynamic_anim_buffer_.push_back(kf);
        }
        // Handle audio keyframes
        else if (strcmp(name, "RobotAudioKeyFrame") == 0) {
            uint32_t trigger_time = keyframe["triggerTime_ms"] | 0;
            
            // Extract audio event names from eventGroups
            JsonArray eventGroups = keyframe["eventGroups"];
            if (!eventGroups.isNull() && eventGroups.size() > 0) {
                for (JsonObject group : eventGroups) {
                    JsonArray audioNames = group["audioName"];
                    if (!audioNames.isNull() && audioNames.size() > 0) {
                        for (JsonVariant audioNameVar : audioNames) {
                            const char* audioName = audioNameVar.as<const char*>();
                            if (audioName && strlen(audioName) > 0) {
                                // Map audio event to WAV file
                                std::string wav_file = map_audio_event_to_wav(audioName);
                                if (!wav_file.empty()) {
                                    AudioEvent evt;
                                    evt.trigger_time = trigger_time;
                                    evt.event_name = audioName;
                                    evt.wav_file = wav_file;
                                    audio_events.push_back(evt);
                                    
                                    ESP_LOGD(TAG, "Audio event '%s' -> '%s' at %dms", 
                                             audioName, wav_file.c_str(), trigger_time);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    if (this->dynamic_anim_buffer_.empty()) {
        ESP_LOGW(TAG, "No procedural face keyframes found");
        return false;
    }
    
    // Second pass: match audio events to closest visual keyframes (100ms tolerance)
    for (const auto& audio : audio_events) {
        int closest_idx = -1;
        uint32_t min_distance = 100; // 100ms tolerance
        
        for (size_t i = 0; i < this->dynamic_anim_buffer_.size(); i++) {
            uint32_t kf_time = this->dynamic_anim_buffer_[i].trigger_time;
            uint32_t distance = (audio.trigger_time > kf_time) ? 
                (audio.trigger_time - kf_time) : (kf_time - audio.trigger_time);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        // Assign audio to closest keyframe if within tolerance and not already assigned
        if (closest_idx >= 0 && this->dynamic_anim_buffer_[closest_idx].sound_name == nullptr) {
            // Store the wav filename and point to it
            this->sound_string_buffer_.push_back(audio.wav_file);
            this->dynamic_anim_buffer_[closest_idx].sound_name = 
                this->sound_string_buffer_.back().c_str();
            
            ESP_LOGI(TAG, "Synced audio '%s' to keyframe at %dms (offset: %dms)",
                     audio.wav_file.c_str(), 
                     this->dynamic_anim_buffer_[closest_idx].trigger_time,
                     min_distance);
        } else if (closest_idx >= 0) {
            ESP_LOGW(TAG, "Keyframe at %dms already has audio, skipping '%s'",
                     this->dynamic_anim_buffer_[closest_idx].trigger_time,
                     audio.wav_file.c_str());
        } else {
            ESP_LOGW(TAG, "No keyframe within 100ms for audio '%s' at %dms",
                     audio.wav_file.c_str(), audio.trigger_time);
        }
    }
    
    ESP_LOGI(TAG, "Loaded %d keyframes, %d with audio", 
             this->dynamic_anim_buffer_.size(), audio_events.size());
    
    return true;
}
#endif  // USE_STORAGE

void VectorEyes::loop() {
  static uint32_t last_loop_log = 0;
  if (millis() - last_loop_log > 2000) {
      last_loop_log = millis();
      ESP_LOGD(TAG, "VectorEyes::loop running");
      if (this->behavior_engine_) ESP_LOGD(TAG, "BehaviorEngine ptr is valid");
      else ESP_LOGW(TAG, "BehaviorEngine ptr is NULL");
  }

  // Update animation player
  this->animation_player_.update(this->face_);
  
  
  // Update behavior engine
  if (this->behavior_engine_) {
      this->behavior_engine_->loop();
  }
  
  // Update audio streaming
  this->update_audio();
  
  // Only auto-play if autonomous mode enabled


  if (this->autonomous_mode_) {
      if (!this->animation_player_.is_playing()) {
          // Check random behavior
          update_behavior();
      }
  }
  
  // Clean up clock mode if it's been showing too long
  if (this->clock_mode_ && millis() - this->clock_show_start_time_ > CLOCK_SHOW_DURATION) {
      this->set_clock_mode(false);
  }
  
  // SPI Bus Locking: Deselect SD card (GPIO0) before display SPI transaction
#ifdef USE_ESP_IDF
  gpio_set_level(GPIO_NUM_0, 1);  // SD CS HIGH = deselected
#endif
  
  this->face_.Draw(this->display_);
  
  if (this->save_pending_) {
      uint32_t now = millis();
      if (now - this->last_volume_change_time_ > 2000) {
          this->save_pending_ = false;
          this->volume_pref_.save(&this->volume_);
          ESP_LOGD(TAG, "Volume saved: %.2f", this->volume_);
      }
  }
}

// Restoration of missing public API methods
void VectorEyes::play_animation(const std::string &name) {
    // Check if file exists on storage (storage adapter or SD card)
    std::string filename = name + ".csv";
    std::string path = "/animations/" + filename;
    
    if (file_exists_any(path)) {
#ifndef USE_STORAGE
        play_animation_from_file(filename);
#else
        // TODO: Implement storage-aware CSV animation playback
        ESP_LOGW(TAG, "CSV animation playback not yet implemented with storage component");
#endif
        return;
    }
    
    ESP_LOGW(TAG, "Animation %s not found on storage, checking internal...", name.c_str());
    // Fallback? Internal ones? For now just log
}

void VectorEyes::stop_animation() {
    animation_player_.stop();
}

void VectorEyes::reset_face() {
    face_.Reset();
    face_.center_x = 0;
    face_.center_y = 0;
    face_.l_lid_top = 1000;
    face_.l_lid_bottom = 1000;
    face_.r_lid_top = 1000;
    face_.r_lid_bottom = 1000;
}

void VectorEyes::look_at(float x, float y) {
    face_.center_x = x;
    face_.center_y = y;
}

void VectorEyes::draw() {
    ESP_LOGV(TAG, "draw() called, face: sx=%.2f sy=%.2f cx=%.2f cy=%.2f", 
             face_.scale_x, face_.scale_y, face_.center_x, face_.center_y);
    
    // Auto-hide clock after 5 seconds
    if (this->clock_mode_ && millis() - clock_show_start_time_ > CLOCK_SHOW_DURATION) {
        this->clock_mode_ = false;
        // Don't redraw eyes immediately, let next loop handle it naturally
    }
    
    // SPI Bus Locking: Deselect SD card (GPIO0) before display SPI transaction
#ifdef USE_ESP_IDF
    gpio_set_level(GPIO_NUM_0, 1);  // SD CS HIGH = deselected
#endif
    
    // Check state first to determine what to draw
    bool is_ready = (this->behavior_engine_ && this->behavior_engine_->is_ready());
    bool min_boot_time_passed = (millis() - this->boot_start_time_ > MIN_BOOT_DURATION);
    
    // Only draw main content (Clock or Eyes) if we are ready OR if it's the clock (which overrides everything)
    // If not ready, we clear the screen for the loading bar
    if (this->clock_mode_) {
        this->draw_clock();
    } else if (is_ready && min_boot_time_passed) {
        face_.Draw(this->display_);
    } else {
        // Clear screen for loading bar if not drawing eyes
        this->display_->fill(display::COLOR_OFF);
    }
    
    if (!is_ready || !min_boot_time_passed) {
        // Draw loading bar at bottom
        int width = 128;
        int height = 64;
        int bar_height = 8;
        
        // ESTIMATED_DURATION_MS = 30000 (30 seconds)
        // Calculate width based on time, capped at full width
        int bar_width = (millis() * width) / 30000; 
        if (bar_width > width) bar_width = width;
        
        // Progress bar background
        this->display_->filled_rectangle(0, height - bar_height, width, bar_height, esphome::display::COLOR_OFF); // Black background
        this->display_->rectangle(0, height - bar_height, width, bar_height, esphome::display::COLOR_ON); // White border
        
        // Fill
        this->display_->filled_rectangle(2, height - bar_height + 2, bar_width, bar_height - 4, esphome::display::COLOR_ON);
        
        // Draw text above loading bar
        if (this->font_) {
            // Draw Peering Eyes above text (Two half circles looking over the text)
            // Center of screen is 64. Text is at ~46 (height - 18)
            // User requested higher and bigger
            int eye_y = height - bar_height - 35; // Moved up (was 28)
            int eye_radius = 8; // Bigger (was 5)
            
            // Left Eye (Offset 12 from center)
            this->display_->filled_circle(52, eye_y, eye_radius, esphome::display::COLOR_ON);
            // Right Eye
            this->display_->filled_circle(76, eye_y, eye_radius, esphome::display::COLOR_ON);
            
            // "Mask" the bottom half to make them look like half-moons/peering
            // Cover spanning both eyes: 52-8=44 to 76+8=84 -> 40 to 88 width 48+
            this->display_->filled_rectangle(40, eye_y, 48, eye_radius + 1, esphome::display::COLOR_OFF);


            // Title moved up to make room for status text
            this->display_->print(64, height - bar_height - 18, this->font_, esphome::display::COLOR_ON, esphome::display::TextAlign::BOTTOM_CENTER, "ESPHoMeeko");
            
            // Whimsical status messages based on state
            const char* status = "Waking up neurons...";
            uint32_t now = millis();
            
            // Enforce "Waking up" state for first 4 seconds (Increased from 2s)
            if (now - this->boot_start_time_ < 4000) {
                 status = "Waking up neurons...";
            } else if (this->storage_adapter_) {
                bool storage_ready = this->storage_adapter_->is_available();
                
                // Timeout for hunting check (e.g. 7 seconds FROM BOOT)
                bool hunting_timeout = (now - this->boot_start_time_ > 7000);

                // State 1: Hunting for memories (Storage connection)
                // Proceed if ready OR if timed out
                if (!storage_ready && !hunting_timeout) {
                    status = "Hunting for memories...";
                    this->hunting_complete_time_ = 0;
                } 
                // State 2: Storage ready (or timed out), hold message for a bit
                else {
                    if (this->hunting_complete_time_ == 0) {
                        this->hunting_complete_time_ = now;
                    }
                    
                    // Increased delay to 3 seconds (was 2s)
                    if (now - this->hunting_complete_time_ < 3000) {
                        status = "Hunting for memories...";
                    } 
                    // State 3: Recalling personality (Loading maps)
                    else if (!is_ready) {
                         status = "Recalling personality..."; 
                         this->recalling_complete_time_ = 0;
                    } 
                    // State 4: Ready, hold message for a bit
                    else {
                         if (this->recalling_complete_time_ == 0) {
                             this->recalling_complete_time_ = now;
                         }

                         // Increased delay to 3 seconds (was 2s)
                         if (now - this->recalling_complete_time_ < 3000) {
                             status = "Recalling personality...";
                             // Force loading bar to stay visible even if is_ready is true
                             min_boot_time_passed = false; 
                         }
                    }
                }
            }
            
            // Status text
            this->display_->print(64, height - bar_height - 3, this->font_, esphome::display::COLOR_ON, esphome::display::TextAlign::BOTTOM_CENTER, status);
        }
    }
}

void VectorEyes::update_behavior() {
    uint32_t now = millis();
    
    // Debug logging for behavior state
    static uint32_t last_debug = 0;
    if (now - last_debug > 5000) {
        last_debug = now;
        ESP_LOGD(TAG, "Behavior State: Playing=%d, Clock=%d, Auto=%d, NextDelay=%d, SinceLast=%d",
                 this->animation_player_.is_playing(), this->clock_mode_, this->autonomous_mode_, 
                 this->next_behavior_delay_, now - this->last_behavior_time_);
    }

    // Only run behavior if we are not currently playing an animation AND not in clock mode
    if (!this->animation_player_.is_playing() && !this->clock_mode_ &&
        this->autonomous_mode_ && 
        now - last_behavior_time_ > next_behavior_delay_) {
        // Pick random behavior
        last_behavior_time_ = now;
        next_behavior_delay_ = 3000 + ((random_uint32() % 10) * 1000); // 3-13s
        
        ESP_LOGI(TAG, "Behavior tick: Triggering NothingToDoBoredIdle");
        play_trigger(AnimationTrigger::NothingToDoBoredIdle);
    }
}

void VectorEyes::play_anim_type(AnimationTrigger type) {
    if (this->clock_mode_) this->clock_mode_ = false; // Cancel clock if animation plays
    play_trigger(type);
}

void VectorEyes::play_trigger(AnimationTrigger trigger) {
    if (this->clock_mode_) this->clock_mode_ = false; // Cancel clock if animation plays
    ESP_LOGD(TAG, "play_trigger called with trigger ID %d", (int)trigger);

    std::string anim_file;
    bool animation_found = false;

    if (this->behavior_engine_ != nullptr) {
        // Convert enum to string for Behavior Engine
        std::string trigger_name = BehaviorEngine::trigger_to_string((int32_t)trigger);
        anim_file = this->behavior_engine_->get_animation_for_trigger(trigger_name);
        
        if (!anim_file.empty()) {
            // Auto-append .json if extension is missing (Common in TriggerMap)
            if (anim_file.find(".") == std::string::npos) {
                anim_file += ".json";
            }

            ESP_LOGI(TAG, "Trigger %s -> Playing %s", trigger_name.c_str(), anim_file.c_str());
            animation_found = true;
            
            // Simple extension check
            if (anim_file.length() > 5 && anim_file.substr(anim_file.length() - 5) == ".json") {
                 this->play_animation_from_json(anim_file);
            } else {
                 // Fallback to CSV or just try playing it
#ifndef USE_STORAGE
                 this->play_animation_from_file(anim_file);
#else
                 ESP_LOGW(TAG, "Non-JSON animation '%s' not supported with storage component yet", anim_file.c_str());
#endif
            }
        } else {
             ESP_LOGW(TAG, "No animation found for trigger: %s", trigger_name.c_str());
        }
    } else {
        ESP_LOGW(TAG, "Behavior engine not initialized");
    }
    
    // Fallback: If no animation found (e.g. storage missing), do a procedural blink
    // But only for "Bored" triggers to avoid spamming blinks on specific events
    if (!animation_found && 
        trigger == AnimationTrigger::NothingToDoBoredIdle) {
         
         ESP_LOGI(TAG, "No animation found, using procedural blink fallback");
         this->trigger_procedural_blink();
    }
}

void VectorEyes::trigger_procedural_blink() {
    // Generate a simple blink animation (Open -> Closed -> Open)
    this->dynamic_anim_buffer_.clear();
    this->sound_string_buffer_.clear();
    
    // Frame 1: Eyes Open (Current State)
    AnimationKeyframe k1;
    k1.trigger_time = 0;
    k1.duration = 100;
    k1.face.Reset(); // 1000 (Open)
    k1.sound_name = nullptr;
    this->dynamic_anim_buffer_.push_back(k1);
    
    // Frame 2: Eyes Closed
    AnimationKeyframe k2;
    k2.trigger_time = 100;
    k2.duration = 150;
    k2.face.Reset();
    k2.face.l_lid_top = 0; k2.face.l_lid_bottom = 0;
    k2.face.r_lid_top = 0; k2.face.r_lid_bottom = 0;
    k2.sound_name = nullptr;
    this->dynamic_anim_buffer_.push_back(k2);
    
    // Frame 3: Eyes Open
    AnimationKeyframe k3;
    k3.trigger_time = 250;
    k3.duration = 100;
    k3.face.Reset(); // 1000
    k3.sound_name = nullptr;
    this->dynamic_anim_buffer_.push_back(k3);
    
    // Play it
    this->animation_player_.play(this->dynamic_anim_buffer_.data(), this->dynamic_anim_buffer_.size());
}

void VectorEyes::draw_clock() {
    if (this->display_ == nullptr) return;
    if (this->time_ == nullptr) {
        static uint32_t last_log = 0;
        if (millis() - last_log > 5000) {
           ESP_LOGW(TAG, "Time component not set, cannot display clock");
           last_log = millis();
        }
        return;
    }

    auto time = this->time_->now();
    if (!time.is_valid()) {
        static uint32_t last_log = 0;
        if (millis() - last_log > 5000) {
           ESP_LOGW(TAG, "Time not valid, cannot display clock");
           last_log = millis();
        }
        return;
    }

    int hour = time.hour;
    int minute = time.minute;
    bool pm = false;
    
    if (!this->is_24h_mode_) {
        // 12-hour format
        if (hour >= 12) {
            pm = true;
            if (hour > 12) hour -= 12;
        }
        if (hour == 0) hour = 12;
    }

    int hour_tens = hour / 10;
    int hour_ones = hour % 10;
    int min_tens = minute / 10;
    int min_ones = minute % 10;

    // Sprite Dimensions
    const int W = CLOCK_DIGIT_WIDTH;    // 18
    const int H = CLOCK_DIGIT_HEIGHT;   // 27
    const int COL_W = CLOCK_COLON_WIDTH; // 6
    const int SPACING = 3;              // Space between digits

    // Calculate total width explicitly for centering
    // Normal: [H10] [H1] [COL] [M10] [M1]
    // Single digit Hour: [H1] [COL] [M10] [M1]
    
    int total_width = 0;
    bool show_tens = (hour_tens > 0); // Always show tens for 10, 11, 12, etc.
    
    if (this->is_24h_mode_) {
       // In 24h mode, typically always show leading zero? Or singular?
       // Let's stick to standard digital watch: 
       // Often 08:30 is shown as 8:30 or 08:30. Let's do 08:30 for 24h consistency usually,
       // but user requested centering if single digit. So we'll skip leading zero if desired?
       // Actually 24h usually requires leading zero for sorting, but for display:
       // "09:00" vs "9:00". User said "if there is only a single hour digit".
       // So [0-9] hours -> single digit. [10-23] -> double.
       show_tens = (hour_tens > 0); 
    }
    
    if (show_tens) {
        // 4 digits + colon + 4 spaces (between items? no, usually tighter)
        // [D] [D] [C] [D] [D]
        // W+S + W+S + CW+S + W+S + W
        total_width = (W * 4) + COL_W + (SPACING * 4);
    } else {
        // 3 digits + colon
        // [D] [C] [D] [D]
        total_width = (W * 3) + COL_W + (SPACING * 3);
    }
    
    // Center X
    // Display is 128 wide
    int start_x = (128 - total_width) / 2;
    int current_x = start_x;
    
    // Vertical Center
    // Display 64, Digit 27
    const int Y_POS = (64 - H) / 2; // (64-27)/2 = 18.5 -> 18

    // Helper lambda to draw sprite
    auto draw_sprite = [&](int x, int y, int w, int h, const uint8_t* data) {
        for (int py = 0; py < h; py++) {
            for (int px = 0; px < w; px++) {
                uint8_t pixel = data[py * w + px];
                if (pixel > 127) { // Simple threshold for 1-bit rendering
                    this->display_->draw_pixel_at(x + px, y + py, esphome::display::COLOR_ON);
                }
            }
        }
    };

    this->display_->clear();

    // Draw Hour Tens (skip if 0)
    if (show_tens) {
        draw_sprite(current_x, Y_POS, W, H, CLOCK_DIGITS[hour_tens]);
        current_x += W + SPACING;
    }
    
    // Draw Hour Ones
    draw_sprite(current_x, Y_POS, W, H, CLOCK_DIGITS[hour_ones]);
    current_x += W + SPACING;

    // Draw Colon
    draw_sprite(current_x, Y_POS, COL_W, H, CLOCK_COLON);
    current_x += COL_W + SPACING;

    // Draw Minute Tens
    draw_sprite(current_x, Y_POS, W, H, CLOCK_DIGITS[min_tens]);
    current_x += W + SPACING;

    // Draw Minute Ones
    draw_sprite(current_x, Y_POS, W, H, CLOCK_DIGITS[min_ones]);

    // Update loop handles actual display refresh
    // Removing recursive call to prevent stack overflow
    // this->display_->update();
}



// play_blink now uses manual mappings to closest available trigger
void VectorEyes::play_blink() { play_trigger(AnimationTrigger::ObservingIdleEyesOnly); }
void VectorEyes::play_anim_happy() { play_trigger(AnimationTrigger::Feedback_GoodRobot); }
void VectorEyes::play_anim_angry() { play_trigger(AnimationTrigger::DriveEndAngry); }
void VectorEyes::play_anim_awe() { play_trigger(AnimationTrigger::ReactToUnexpectedMovement); } 
void VectorEyes::play_anim_neutral() { play_trigger(AnimationTrigger::NeutralFace); }
void VectorEyes::play_anim_look_right() { play_trigger(AnimationTrigger::ObservingLookStraight); } // Fallback
void VectorEyes::play_anim_look_left() { play_trigger(AnimationTrigger::ObservingLookStraight); } // Fallback

// Async Audio Update Loop
void VectorEyes::update_audio() {
#ifdef USE_SPEAKER
    if (!this->audio_playing_ || this->audio_handle_ == nullptr || this->storage_adapter_ == nullptr) {
        return;
    }
  
    // static uint32_t last_log = 0;
    // if (millis() - last_log > 1000) {
    //    last_log = millis();
    //    ESP_LOGV(TAG, "Audio update: read=%u total=%u", this->audio_bytes_read_, this->audio_data_size_);
    // }

    // Process audio in chunks
    // 8KB buffer (matches new play_wav_file buffer size)
    // We want to read enough to keep I2S busy, but not block main loop too long
    // If we read 4KB at 16khz/16bit, that's ~128ms. 
    // We should try to read smaller chunks more often, or rely on I2S buffering.
    // Let's read 1KB chunks to be responsive (approx 32ms)
    // Persistent buffer state
    // Check if speaker is ready for more data (basic backpressure check if available)
    // Most Speaker components block if buffer full, so we rely on that but with small writes.
    
    // 1. Retry sending remaining data from previous cycle
    if (this->audio_buffer_valid_bytes_ > this->audio_buffer_sent_bytes_) {
        size_t samples_remaining = this->audio_buffer_valid_bytes_ - this->audio_buffer_sent_bytes_;
        size_t bytes_remaining = samples_remaining * 2; // 16-bit samples
        
        // Align to 4 bytes for ESP32 I2S
        if (bytes_remaining % 4 != 0) bytes_remaining -= (bytes_remaining % 4);
        
        if (bytes_remaining > 0) {
            size_t bytes_written = this->speaker_->play((const uint8_t*)(this->audio_sample_buffer_ + this->audio_buffer_sent_bytes_), bytes_remaining);
            this->audio_buffer_sent_bytes_ += (bytes_written / 2);
            
            if (this->audio_buffer_sent_bytes_ < this->audio_buffer_valid_bytes_) {
                // Still full
                return;
            }
        } else {
             this->audio_buffer_sent_bytes_ = this->audio_buffer_valid_bytes_;
        }
    }

    // 2. Read new chunk
    this->audio_buffer_sent_bytes_ = 0;
    this->audio_buffer_valid_bytes_ = 0;
    
    // Use raw buffer from heap/stack - declare here or use member?
    // Using stack is fine for 1KB if stack is large enough. ESP32 default stack is 8KB? 
    // 8KB stack is tight for 1KB buffer + overhead. 
    // Let's use a member buffer for safety or static?
    // Member buffer is safer.
    
    size_t bytes_read = this->storage_adapter_->read_chunk(this->audio_handle_, (uint8_t*)this->audio_raw_buffer_, 1024);
    
    if (bytes_read == 0) {
        // EOF or Error
        this->storage_adapter_->close_file(this->audio_handle_);
        this->audio_handle_ = nullptr;
        this->audio_playing_ = false;
        ESP_LOGD(TAG, "Audio playback finished (Async)");
        return;
    }
    
    this->audio_bytes_read_ += bytes_read;
    
    // Process audio (Convert/Volume)
    // Note: This logic duplicates play_wav_file processing. ideally refactor, but for now inline.
    
    // Check bits per sample
    size_t samples_ready = 0;
    
    if (this->audio_bits_ == 24) {
        // Convert 24-bit to 16-bit
        size_t sample_count = bytes_read / 3;
        for (size_t i = 0; i < sample_count; i++) {
             // 24-bit Little Endian: LSB, MID, MSB (0, 1, 2)
             // 16-bit target:        MID, MSB (1, 2)
             uint8_t* raw = (uint8_t*)this->audio_raw_buffer_;
             int16_t s = (raw[i*3 + 2] << 8) | raw[i*3 + 1];
             this->audio_sample_buffer_[i] = s;
        }
        samples_ready = sample_count;
    } else {
        // Assume 16-bit
        int16_t* src = (int16_t*)this->audio_raw_buffer_;
        size_t sample_count = bytes_read / 2;
        for(size_t i=0; i<sample_count; i++) {
            this->audio_sample_buffer_[i] = src[i];
        }
        samples_ready = sample_count;
    }
    
    // Apply Volume
    if (this->volume_ < 0.99f) {
        for (size_t i = 0; i < samples_ready; i++) {
            this->audio_sample_buffer_[i] = (int16_t)(this->audio_sample_buffer_[i] * this->volume_);
        }
    }
    
    this->audio_buffer_valid_bytes_ = samples_ready;
    
    size_t bytes_to_write = this->audio_buffer_valid_bytes_ * 2;
    // Align to 4 bytes for ESP32 I2S
    if (bytes_to_write % 4 != 0) bytes_to_write -= (bytes_to_write % 4);
    
    // Initial Write Attempt
    if (bytes_to_write > 0) {
        size_t written = this->speaker_->play((const uint8_t*)this->audio_sample_buffer_, bytes_to_write);
        this->audio_buffer_sent_bytes_ += (written / 2);
    }
    
    // Check if we hit data limit (if RIFF header size was parsed)
    if (this->audio_data_size_ > 0 && this->audio_bytes_read_ >= this->audio_data_size_) {
        // We reached end of declared data chunk
        // Close immediately? Or wait for buffer to drain?
        // If we close now, valid_samples might still have data to play next loop.
        // But we shouldn't read more.
        // Let's NOT close yet if valid_samples > samples_sent? 
        // Actually, if we just read the last chunk, valid_samples is set. 
        // Next loop will try to drain it.
        // Only close if we are actually done AND buffer is empty?
        // For simplicity: Mark as done reading, but keep handle until drained?
        // Or just let the EOF check handle it next time? 
        // read_chunk might fail or return 0 next time effectively.
        // But let's close explicitly to be safe as before.
        this->storage_adapter_->close_file(this->audio_handle_);
        this->audio_handle_ = nullptr;
        this->audio_playing_ = false;
        ESP_LOGD(TAG, "Audio playback finished (Data limit reached)");
    }
#endif
}

void VectorEyes::play_wav_file(const std::string &filename) {

#ifdef USE_SPEAKER
    if (this->speaker_ == nullptr) return;

    // Stop current playback if any
    if (this->audio_playing_ && this->audio_handle_ != nullptr) {
        ESP_LOGD(TAG, "Stopping previous audio playback before starting new file");
        // The audio_handle_ could be a File* or a void* from StorageAdapter
        // We need to close it appropriately.
#ifdef USE_STORAGE
        if (this->storage_adapter_ != nullptr) {
            this->storage_adapter_->close_file(this->audio_handle_);
        }
#else
        // If not using storage adapter, assume it's an Arduino File*
        ((File *)this->audio_handle_)->close();
        delete (File *)this->audio_handle_; // Assuming it was allocated with new
#endif
        this->audio_handle_ = nullptr;
        this->audio_playing_ = false;
    }

#ifdef USE_STORAGE
    // Try storage adapter first
    if (this->storage_adapter_ && this->storage_adapter_->is_available()) {
        std::string path = "/audio/" + filename;
        
        // Open file
        void *handle = this->storage_adapter_->open_file(path);
        if (handle != nullptr) {
            ESP_LOGD(TAG, "Starting async audio: %s", filename.c_str());
            
            // Read RIFF Header (12 bytes)
            uint8_t riff_header[12];
            if (this->storage_adapter_->read_chunk(handle, riff_header, 12) < 12) {
                 ESP_LOGE(TAG, "Invalid WAV: Too short");
                 this->storage_adapter_->close_file(handle);
                 return;
            }
            
            if (memcmp(riff_header, "RIFF", 4) != 0 || memcmp(riff_header + 8, "WAVE", 4) != 0) {
                 ESP_LOGE(TAG, "Invalid WAV: Not a RIFF/WAVE file");
                 this->storage_adapter_->close_file(handle);
                 return;
            }

            // Parse Header asynchronously-ish (fast enough to do here)
            // We need to find 'fmt ' and 'data' chunks to set up state
            bool data_found = false;
            uint8_t chunk_header[8];
            
            // Limit header scan to prevent locking up
            int scan_limit = 100; 
            
            while (!data_found && scan_limit-- > 0) {
                if (this->storage_adapter_->read_chunk(handle, chunk_header, 8) < 8) break;
                
                uint32_t chunk_size = chunk_header[4] | (chunk_header[5] << 8) | 
                                      (chunk_header[6] << 16) | (chunk_header[7] << 24);
                                      
                if (memcmp(chunk_header, "fmt ", 4) == 0) {
                    uint8_t fmt_data[16];
                    if (chunk_size >= 16) {
                        this->storage_adapter_->read_chunk(handle, fmt_data, 16);
                        this->audio_channels_ = fmt_data[2] | (fmt_data[3] << 8);
                        this->audio_rate_ = fmt_data[4] | (fmt_data[5] << 8) | (fmt_data[6] << 16) | (fmt_data[7] << 24);
                        this->audio_bits_ = fmt_data[14] | (fmt_data[15] << 8);
                        
                        // Skip rest of fmt
                        if (chunk_size > 16) {
                            size_t to_skip = chunk_size - 16;
                            uint8_t temp;
                            for (size_t k=0; k<to_skip; k++) this->storage_adapter_->read_chunk(handle, &temp, 1);
                        }
                    } else {
                         // Malformed
                         this->storage_adapter_->close_file(handle);
                         return;
                    }
                } 
                else if (memcmp(chunk_header, "data", 4) == 0) {
                    data_found = true;
                    this->audio_data_size_ = chunk_size;
                    this->audio_bytes_read_ = 0;
                    this->audio_handle_ = handle;
                    this->audio_playing_ = true;
                    // Reset buffer state
                    this->audio_buffer_valid_bytes_ = 0;
                    this->audio_buffer_sent_bytes_ = 0;
                    
                    ESP_LOGI(TAG, "Async WAV Started: %s | %u Hz | %u bit | %u ch", 
                             filename.c_str(), this->audio_rate_, this->audio_bits_, this->audio_channels_);
                             
                    // Ready to play! return and let loop() handle it.
                    return; 
                }
                else {
                    // Skip unknown chunk
                    size_t remaining = chunk_size;
                    uint8_t skip_buf[64];
                    while (remaining > 0) {
                        size_t to_read = remaining > sizeof(skip_buf) ? sizeof(skip_buf) : remaining;
                        if (this->storage_adapter_->read_chunk(handle, skip_buf, to_read) == 0) break;
                        remaining -= to_read;
                    }
                }
            }
            
            // If we got here, we failed to find data or scanned too much
            this->storage_adapter_->close_file(handle);
            ESP_LOGW(TAG, "Failed to start audio: Header parse failed or no data chunk");
            return;
        }
    }
#endif

    // Fallback?
    ESP_LOGW(TAG, "Audio file not found or storage unavailable: %s", filename.c_str());
#endif
}

void VectorEyes::play_zelda() {
    play_wav_file("zelda.wav");
}

void VectorEyes::set_volume(float volume) {
    this->volume_ = volume;
    this->last_volume_change_time_ = millis();
    this->save_pending_ = true;
}

void VectorEyes::set_autonomous_mode(bool enabled) {
    this->autonomous_mode_ = enabled;
}

void VectorEyes::set_display(display::DisplayBuffer *display) { this->display_ = display; }

#ifdef USE_SPEAKER
void VectorEyes::set_speaker(speaker::Speaker *speaker) { this->speaker_ = speaker; }
#endif

#ifdef USE_STORAGE
void VectorEyes::set_storage(storage::Storage *storage) { 
    this->storage_component_ = storage; 
    printf("DEBUG: set_storage called. storage=%p, this=%p\n", storage, this);
    ESP_LOGI(TAG, "set_storage called. storage_component_=%p", this->storage_component_);
}
void VectorEyes::set_mount_path(const std::string &path) { 
    this->mount_path_ = path; 
    ESP_LOGI(TAG, "set_mount_path called. path=%s", this->mount_path_.c_str());
}
#endif

void VectorEyes::set_sd_cs_pin(GPIOPin *pin) { this->sd_cs_pin_ = pin; }

float VectorEyes::get_volume() const { return this->volume_; }



void VectorEyes::apply_keyframe(ProceduralFace &face, const AnimationKeyframe &kf) {
  face.scale_x = kf.face.scale_x;
  face.scale_y = kf.face.scale_y;
  face.angle = kf.face.angle;
  face.center_x = kf.face.center_x;
  face.center_y = kf.face.center_y;
  face.l_lid_top = kf.face.l_lid_top;
  face.l_lid_bottom = kf.face.l_lid_bottom;
  face.r_lid_top = kf.face.r_lid_top;
  face.r_lid_bottom = kf.face.r_lid_bottom;
}

// Arduino SD library fallback (only available with Arduino framework)
// for generic storage, this function is now enabled
void VectorEyes::load_audio_mappings() {
#ifdef USE_STORAGE
    // Try storage adapter first
    if (this->storage_adapter_ && this->storage_adapter_->is_available()) {
        ESP_LOGI(TAG, "Attempting to load audio mappings via storage adapter...");
        std::vector<uint8_t> data;
        // Try root path
        if (this->storage_adapter_->read_file("/audio_mappings.json", data)) {
            // Null terminate
            data.push_back(0);
            
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, (char*)data.data());
            
            if (!error) {
                JsonObject mappings = doc["mappings"].as<JsonObject>();
                for (JsonPair kv : mappings) {
                    this->audio_event_map_[kv.key().c_str()] = kv.value().as<std::string>();
                }
                ESP_LOGI(TAG, "Loaded %d audio event mappings from storage", this->audio_event_map_.size());
                return;
            } else {
                ESP_LOGE(TAG, "Failed to parse audio mappings from storage: %s", error.c_str());
            }
        } else {
             ESP_LOGW(TAG, "audio_mappings.json not found on storage");
        }
    }
#endif

#ifndef USE_STORAGE
    if (!this->sd_card_initialized_) {
        ESP_LOGW(TAG, "SD card not initialized, cannot load audio mappings");
        return;
    }
    
    // CRITICAL DEBUG: Use POSIX API to list directory since Arduino SD library is broken
    ESP_LOGI(TAG, "=== SD CARD DEBUG: Listing root directory with POSIX API ===");
    
    // Try to find the SD card mount point
    const char* mount_points[] = {"/sd", "/sdcard", "/mnt/sd", "/SD"};
    const char* sd_mount = nullptr;
    
    for (const char* mp : mount_points) {
        DIR* dir = opendir(mp);
        if (dir) {
            sd_mount = mp;
            closedir(dir);
            ESP_LOGI(TAG, "Found SD card mounted at: %s", sd_mount);
            break;
        }
    }
    
    if (!sd_mount) {
        ESP_LOGW(TAG, "Could not find SD card mount point, trying common paths...");
        // List what's available at root
        DIR* root_dir = opendir("/");
        if (root_dir) {
            ESP_LOGI(TAG, "Contents of /:");
            struct dirent* entry;
            while ((entry = readdir(root_dir)) != nullptr) {
                ESP_LOGI(TAG, "  - %s", entry->d_name);
            }
            closedir(root_dir);
        }
        
        // Arduino SD library claims to work without mount point, try current directory
        ESP_LOGI(TAG, "Trying to list files in current directory:");
        DIR* cwd = opendir(".");
        if (cwd) {
            struct dirent* entry;
            int count = 0;
            while ((entry = readdir(cwd)) != nullptr && count < 20) {
                struct stat st;
                if (stat(entry->d_name, &st) == 0) {
                    ESP_LOGI(TAG, "  [%d] %s %s (size: %ld bytes)", 
                             count,
                             S_ISDIR(st.st_mode) ? "DIR " : "FILE",
                             entry->d_name,
                             st.st_size);
                    count++;
                }
            }
            closedir(cwd);
            ESP_LOGI(TAG, "=== Found %d items ===", count);
        } else {
            ESP_LOGE(TAG, "Failed to open current directory!");
        }
    } else {
        // List files in the SD mount point
        DIR* dir = opendir(sd_mount);
        if (dir) {
            struct dirent* entry;
            int count = 0;
            while ((entry = readdir(dir)) != nullptr && count < 20) {
                char full_path[256];
                snprintf(full_path, sizeof(full_path), "%s/%s", sd_mount, entry->d_name);
                struct stat st;
                if (stat(full_path, &st) == 0) {
                    ESP_LOGI(TAG, "  [%d] %s %s (size: %ld bytes)", 
                             count,
                             S_ISDIR(st.st_mode) ? "DIR " : "FILE",
                             entry->d_name,
                             st.st_size);
                    count++;
                }
            }
            closedir(dir);
            ESP_LOGI(TAG, "=== Found %d items in %s ===", count, sd_mount);
        }
    }
    
    // Arduino SD library: use filename without leading slash
    const char* mapping_file = "audio_mappings.json";
    
    // Try to open directly (SD.exists() is unreliable)
    ESP_LOGI(TAG, "Attempting to load audio mappings from %s", mapping_file);
    File file2 = SD.open(mapping_file, FILE_READ);
    if (!file2) {
        ESP_LOGW(TAG, "Could not open audio mappings file: %s", mapping_file);
        return;
    }
    
    ESP_LOGI(TAG, "Successfully opened audio_mappings.json (%d bytes)", file2.size());
    
    // Read file into string
    size_t fsize = file2.size();
    char *json_str = (char*)malloc(fsize + 1);
    file2.read((uint8_t*)json_str, fsize);
    json_str[fsize] = 0;
    file2.close();
    
    // Parse JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json_str);
    free(json_str);
    
    if (error) {
        ESP_LOGE(TAG, "Failed to parse audio mappings: %s", error.c_str());
        return;
    }
    
    // Load mappings from JSON
    JsonObject mappings = doc["mappings"].as<JsonObject>();
    for (JsonPair kv : mappings) {
        this->audio_event_map_[kv.key().c_str()] = kv.value().as<std::string>();
    }
    
    ESP_LOGI(TAG, "Loaded %d audio event mappings", this->audio_event_map_.size());
#endif  // USE_STORAGE (End of fallback logic)
}

std::string VectorEyes::map_audio_event_to_wav(const std::string &event_name) {
    // Lazy load audio mappings if not yet loaded and storage is now available
#ifdef USE_STORAGE
    if (this->audio_event_map_.empty() && this->storage_adapter_ && this->storage_adapter_->is_available()) {
        ESP_LOGI(TAG, "Audio mappings not loaded yet, loading now (lazy init)...");
        this->load_audio_mappings();
    }
#endif

    // Try exact match first
    auto it = this->audio_event_map_.find(event_name);
    if (it != this->audio_event_map_.end()) {
        return it->second;
    }
    
    // Try pattern matching (substring match)
    for (const auto& pair : this->audio_event_map_) {
        if (event_name.find(pair.first) != std::string::npos) {
            ESP_LOGD(TAG, "Pattern matched '%s' -> '%s'", event_name.c_str(), pair.second.c_str());
            return pair.second;
        }
    }
    
    ESP_LOGW(TAG, "No audio mapping found for event: %s", event_name.c_str());
    return "";
}

void VectorEyes::match_audio_to_keyframes() {
    // This function matches audio events to the closest visual keyframes
    // within a 100ms tolerance window
    
    struct AudioEvent {
        uint32_t trigger_time;
        std::string wav_file;
    };
    
    std::vector<AudioEvent> audio_events;
    
    // Extract audio events from sound_string_buffer_ (populated during JSON parse)
    // Note: In the improved parse_json_animation, we'll populate this properly
    
    // For each audio event, find the closest visual keyframe
    for (const auto& audio : audio_events) {
        int closest_idx = -1;
        uint32_t min_distance = 100; // 100ms tolerance
        
        for (size_t i = 0; i < this->dynamic_anim_buffer_.size(); i++) {
            uint32_t distance = abs((int32_t)(this->dynamic_anim_buffer_[i].trigger_time - audio.trigger_time));
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        // Assign audio to closest keyframe if within tolerance
        if (closest_idx >= 0 && this->dynamic_anim_buffer_[closest_idx].sound_name == nullptr) {
            // Store the wav filename in sound_string_buffer_ and point to it
            this->sound_string_buffer_.push_back(audio.wav_file);
            this->dynamic_anim_buffer_[closest_idx].sound_name = 
                this->sound_string_buffer_.back().c_str();
            
            ESP_LOGD(TAG, "Matched audio '%s' to keyframe at %dms (distance: %dms)",
                     audio.wav_file.c_str(), 
                     this->dynamic_anim_buffer_[closest_idx].trigger_time,
                     min_distance);
        }
    }
}

} // namespace vector_eyes
} // namespace esphome

namespace esphome {
namespace vector_eyes {

//========================================================================
// Helper Methods for Storage Abstraction
//========================================================================

bool VectorEyes::file_exists_any(const std::string &path) {
#ifdef USE_STORAGE
    // Try storage adapter first
    if (this->storage_adapter_ && this->storage_adapter_->is_available()) {
        return this->storage_adapter_->file_exists(path);
    }
#endif
    
#ifndef USE_STORAGE
    // Fallback to SD card (Arduino framework only)
    if (this->sd_card_initialized_) {
        // SD card expects leading slash
        std::string sd_path = path;
        if (!sd_path.empty() && sd_path[0] != '/') {
            sd_path = "/" + sd_path;
        }
        return SD.exists(sd_path.c_str());
    }
#endif
    
    return false;
}




} // namespace vector_eyes
} // namespace esphome
