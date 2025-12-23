#include "animation_player.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace vector_eyes {

static const char *TAG = "vector_eyes.animation";

void AnimationPlayer::play(const AnimationKeyframe *frames, size_t length) {
  if (frames == nullptr || length == 0) {
    return;
  }
  
  current_anim_ = frames;
  current_anim_length_ = length;
  anim_start_time_ = millis();
  playing_ = true;
  transitioning_ = false;
  last_played_kf_idx_ = -1;
  
  ESP_LOGD(TAG, "Playing animation with %d keyframes", length);
}

void AnimationPlayer::update(ProceduralFace &face) {
  if (transitioning_) {
    uint32_t now = millis();
    uint32_t elapsed = now - transition_start_time_;
    
    if (elapsed >= transition_duration_) {
      face = target_face_;
      transitioning_ = false;
      ESP_LOGD(TAG, "Transition finished");
    } else {
      float t = (float)elapsed / (float)transition_duration_;
      interpolate(face, start_face_, target_face_, t);
    }
    return;
  }
  
  if (!playing_ || current_anim_ == nullptr) {
    return;
  }
  
  uint32_t now = millis();
  uint32_t elapsed = now - anim_start_time_;
  
  // Find current keyframe
  size_t current_kf_idx = 0;
  for (size_t i = 0; i < current_anim_length_; i++) {
    if (current_anim_[i].trigger_time <= elapsed) {
      current_kf_idx = i;
    } else {
      break;
    }
  }
  
  // Check for sound triggers (Handle skipping safely)
  // If we jumped multiple frames (e.g. slowed down loop), make sure we trigger sounds
  // for the skipped keyframes too.
  if ((int)current_kf_idx > last_played_kf_idx_) {
    // Iterate from last_played_kf_idx_ + 1 up to current_kf_idx
    int start_check = last_played_kf_idx_ + 1;
    for (int i = start_check; i <= (int)current_kf_idx; i++) {
        const auto &kf = current_anim_[i];
        if (kf.sound_name != nullptr) {
            if (this->on_sound_trigger_) {
                ESP_LOGD("AnimationPlayer", "Triggering sound: %s", kf.sound_name);
                this->on_sound_trigger_(kf.sound_name);
            }
        }
    }
    last_played_kf_idx_ = current_kf_idx;
  }
  
  // Check if animation finished
  if (current_kf_idx >= current_anim_length_ - 1) {
    const auto &last_kf = current_anim_[current_anim_length_ - 1];
    if (elapsed >= last_kf.trigger_time + last_kf.duration) {
      face = last_kf.face;
      playing_ = false;
      current_anim_ = nullptr;
      ESP_LOGD(TAG, "Animation finished");
      return;
    }
  }
  
  // Interpolate between current and next keyframe
  const auto &current_kf = current_anim_[current_kf_idx];
  
  if (current_kf_idx < current_anim_length_ - 1) {
    const auto &next_kf = current_anim_[current_kf_idx + 1];
    uint32_t kf_elapsed = elapsed - current_kf.trigger_time;
    
    if (kf_elapsed < current_kf.duration) {
      float t = (float)kf_elapsed / (float)current_kf.duration;
      interpolate(face, current_kf.face, next_kf.face, t);
    } else {
      face = current_kf.face;
    }
  } else {
    face = current_kf.face;
  }
}

void AnimationPlayer::transition_to(ProceduralFace &current, const ProceduralFace &target, uint32_t duration) {
  start_face_ = current;
  target_face_ = target;
  transition_start_time_ = millis();
  transition_duration_ = duration;
  transitioning_ = true;
  playing_ = false;
  current_anim_ = nullptr;
  
  ESP_LOGD(TAG, "Starting transition (duration: %dms)", duration);
}

void AnimationPlayer::interpolate(ProceduralFace &out, const ProceduralFace &start, const ProceduralFace &end, float t) {
  // Linear interpolation
  out.scale_x = start.scale_x + (end.scale_x - start.scale_x) * t;
  out.scale_y = start.scale_y + (end.scale_y - start.scale_y) * t;
  out.angle = start.angle + (end.angle - start.angle) * t;
  out.center_x = start.center_x + (end.center_x - start.center_x) * t;
  out.center_y = start.center_y + (end.center_y - start.center_y) * t;
  
  out.l_lid_top = start.l_lid_top + (end.l_lid_top - start.l_lid_top) * t;
  out.l_lid_bottom = start.l_lid_bottom + (end.l_lid_bottom - start.l_lid_bottom) * t;
  out.r_lid_top = start.r_lid_top + (end.r_lid_top - start.r_lid_top) * t;
  out.r_lid_bottom = start.r_lid_bottom + (end.r_lid_bottom - start.r_lid_bottom) * t;
}

}  // namespace vector_eyes
}  // namespace esphome
