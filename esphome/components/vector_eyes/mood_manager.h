#pragma once

#include <algorithm>
#include <cstdlib>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace vector_eyes {

enum class MoodType {
  Neutral,
  Happy,
  Angry,
  Sad,
  Surprised,
  Curious,
  Sleepy
};

class MoodManager {
 public:
  MoodManager() = default;

  void setup() {
    // Start neutral
    current_mood_ = MoodType::Neutral;
    last_mood_change_time_ = millis();
  }

  void update() {
    // Decay emotions over time back to neutral
    uint32_t now = millis();
    if (current_mood_ != MoodType::Neutral && 
        now - last_mood_change_time_ > mood_duration_ms_) {
      set_mood(MoodType::Neutral);
    }
  }

  void set_mood(MoodType new_mood, uint32_t duration_ms = 10000) {
    if (current_mood_ != new_mood) {
      current_mood_ = new_mood;
      last_mood_change_time_ = millis();
      mood_duration_ms_ = duration_ms;
      ESP_LOGD("MoodManager", "Mood changed to %d", (int)current_mood_);
    } else {
      // Extend duration if same mood triggered
      mood_duration_ms_ = std::max(mood_duration_ms_, duration_ms);
      last_mood_change_time_ = millis(); // Reset timer
    }
  }

  MoodType get_mood() const { return current_mood_; }

  // Get a stimulation level (0.0 to 1.0) - placeholder for more complex logic
  float get_stimulation_level() const {
    if (current_mood_ == MoodType::Happy || current_mood_ == MoodType::Angry) {
      return 0.8f;
    }
    return 0.2f;
  }

 protected:
  MoodType current_mood_{MoodType::Neutral};
  uint32_t last_mood_change_time_{0};
  uint32_t mood_duration_ms_{10000};
};

}  // namespace vector_eyes
}  // namespace esphome
