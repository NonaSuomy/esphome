// Mapping from animation triggers to actual animation names
// Based on Vector's AnimationTriggerMap.json system
#pragma once

#include "animation_triggers.h"
#include <vector>
#include <cstdlib>

namespace esphome {
namespace vector_eyes {

struct TriggerMapping {
  AnimationTrigger trigger;
  std::vector<const char*> animation_names;  // Multiple options for variety
};

// Map triggers to animation names (supports multiple animations per trigger for variety)
static const TriggerMapping TRIGGER_MAP[] = {
  // Core expressions
  {AnimationTrigger::NeutralFace, {"anim_eyes_neutral"}},
  {AnimationTrigger::Blink, {"anim_keepalive_blink_01"}},
  
  // Directional looks
  {AnimationTrigger::LookLeft, {"anim_eyes_look_left"}},
  {AnimationTrigger::LookRight, {"anim_eyes_look_right"}},
  {AnimationTrigger::LookUp, {"anim_lookinplaceforfaces_bodymovepause"}},
  {AnimationTrigger::LookDown, {"anim_onboarding_lookdown_loop"}},
  
  // Emotions
  {AnimationTrigger::Happy, {"anim_eyes_look_happy", "anim_explorer_huh_01"}},
  {AnimationTrigger::Angry, {"anim_eyes_angry"}},
  {AnimationTrigger::Curious, {"anim_eyes_awe", "anim_explorer_huh_01"}},
  {AnimationTrigger::Sad, {"anim_rtmotion_sad_01"}},
  {AnimationTrigger::Surprised, {"anim_observer_lookinsurprised_01"}},
  
  // Idle behaviors
  {AnimationTrigger::ExploringLookAround, {"anim_explorer_lookaround_01", "anim_hiking_lookaround_01"}},
  {AnimationTrigger::ExploringQuickScan, {"anim_explorer_scan_short_01"}},
  {AnimationTrigger::NothingToDoBoredIdle, {"anim_hiking_lookaround_01"}},
  {AnimationTrigger::ObservingIdleWithHeadLookingStraight, {"anim_observing_far_subtle_01", "anim_keepalive_eyesonly_loop_01"}},
  
  // Reactions
  {AnimationTrigger::ReactToMotion, {"anim_rtmotion_greet_01", "anim_rtmotion_happy_01"}},
  {AnimationTrigger::ReactToMotionLeft, {"anim_turn_left_01"}},
  {AnimationTrigger::ReactToMotionRight, {"anim_turn_right_01"}},
  
  // Weather
  {AnimationTrigger::WeatherSunny, {"anim_weather_sunny_01"}},
  {AnimationTrigger::WeatherCloudy, {"anim_weather_cloud_01"}},
  {AnimationTrigger::WeatherRain, {"anim_weather_rain_01"}},
  {AnimationTrigger::WeatherSnow, {"anim_weather_snow_01"}},
};

// Get a random animation name for a given trigger
inline const char* get_animation_for_trigger(AnimationTrigger trigger) {
  for (const auto& mapping : TRIGGER_MAP) {
    if (mapping.trigger == trigger) {
      if (mapping.animation_names.empty()) {
        return nullptr;
      }
      // Pick random from list
      size_t idx = rand() % mapping.animation_names.size();
      return mapping.animation_names[idx];
    }
  }
  return nullptr;
}

}  // namespace vector_eyes
}  // namespace esphome
