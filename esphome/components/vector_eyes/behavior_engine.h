#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <map>
#include <vector>
#include <string>
#include <ArduinoJson.h>

#include "storage_adapter.h"
#include "animation_triggers.h"

namespace esphome {
namespace vector_eyes {

 

struct AnimationGroupEntry {
    std::string name;
    float weight;
    float cooldown; // seconds
    std::string mood; 
    // float headAngleMin/Max; 
};

struct AnimationGroup {
    std::string name;
    std::vector<AnimationGroupEntry> entries;
    // We don't track cooldowns in the Group struct structure-wise, usually it is runtime state.
    // The engine tracks cooldowns.
};

class BehaviorEngine {
 public:
  BehaviorEngine() {}

#ifdef USE_STORAGE
  void setup(StorageAdapter *storage_adapter);
#endif
  void loop();
  
  // Returns the filename of the animation to play for the given trigger
  // Returns empty string if no valid animation found
  std::string get_animation_for_trigger(const std::string &trigger_name);
  
  // Overload for enum if we can resolve it
  // std::string get_animation_for_trigger(AnimationTrigger trigger);

  // Reload maps from storage
  void load_maps();

  // Helper to convert Enum to String (needs implementation in cpp with big switch)
  static std::string trigger_to_string(int32_t trigger);

  bool is_ready() const { return maps_loaded_; }

 protected:
#ifdef USE_STORAGE
  StorageAdapter *storage_adapter_{nullptr};
#endif

  struct CompactTriggerEntry {
      uint32_t trigger_hash;
      uint16_t group_name_offset;
  };

  // Compact Map: Trigger Hash -> Offset in group_name_pool_
  std::vector<CompactTriggerEntry> trigger_map_;
  
  // Compact pool of group name strings (null terminated)
  std::vector<char> group_name_pool_;
  
  uint32_t hash_trigger(const char* str);

  // Cache of loaded Animation Groups: Group Name -> Group Data
  std::map<std::string, AnimationGroup> loaded_groups_;
  
  // Runtime cooldowns: Animation Name -> Cooldown Expiry Time (millis)
  std::map<std::string, uint32_t> cooldowns_;

  bool maps_loaded_{false};

  void load_trigger_map();
  // void scan_animation_groups(); // REMOVED
  bool load_animation_group(const std::string &group_name);
  
  std::string select_animation_from_group(AnimationGroup &group);
  bool is_on_cooldown(const std::string &anim_name);
  void set_cooldown(const std::string &anim_name, float cooldown_sec);
};

}  // namespace vector_eyes
}  // namespace esphome
