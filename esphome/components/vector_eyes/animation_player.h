#pragma once
#include "procedural_face.h"
#include <cstdint>
#include <cstddef>
#include <functional>

namespace esphome {
namespace vector_eyes {

// Compact animation keyframe structure optimized for PROGMEM storage
// Total size: 60 bytes on ESP32 (4 + 4 + 48 + 4)
// - trigger_time: 4 bytes (when to execute this keyframe)
// - duration: 4 bytes (how long until next keyframe)
// - face: 48 bytes (ProceduralFace with 9 floats + 3 dimension floats)
// - sound_name: 4 bytes (pointer to audio filename or nullptr)
struct AnimationKeyframe {
  uint32_t trigger_time;      // Milliseconds from animation start
  uint32_t duration;          // Duration until next keyframe
  ProceduralFace face;        // Eye/face parameters
  const char* sound_name{nullptr};  // WAV filename or nullptr
};

class AnimationPlayer {
 public:
  void update(ProceduralFace &face);
  void play(const AnimationKeyframe *frames, size_t length);
  bool is_playing() const { return playing_; }
  void stop() { playing_ = false; transitioning_ = false; }
  
  void transition_to(ProceduralFace &current, const ProceduralFace &target, uint32_t duration);

  void set_sound_callback(std::function<void(const char*)> callback) { on_sound_trigger_ = callback; }
  
 private:
  const AnimationKeyframe *current_anim_{nullptr};
  size_t current_anim_length_{0};
  uint32_t anim_start_time_{0};
  bool playing_{false};
  int last_played_kf_idx_{-1};
  
  std::function<void(const char*)> on_sound_trigger_;
  
  bool transitioning_{false};
  ProceduralFace start_face_;
  ProceduralFace target_face_;
  uint32_t transition_start_time_{0};
  uint32_t transition_duration_{0};
  
  void interpolate(ProceduralFace &out, const ProceduralFace &start, const ProceduralFace &end, float t);
};

} // namespace vector_eyes
} // namespace esphome
