#include "behavior_engine.h"
#include "animation_triggers.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <random>

namespace esphome {
namespace vector_eyes {

static const char *TAG = "behavior_engine";

#ifdef USE_STORAGE
void BehaviorEngine::setup(StorageAdapter *storage_adapter) {
    this->storage_adapter_ = storage_adapter;
}
#endif

void BehaviorEngine::loop() {
#ifdef USE_STORAGE
    // Retry loading maps if not yet loaded and storage is available
    static uint32_t last_loop_log = 0;
    if (millis() - last_loop_log > 2000) {
        last_loop_log = millis();
        ESP_LOGD(TAG, "BehaviorEngine::loop running, maps_loaded_=%d", maps_loaded_);
        if (this->storage_adapter_) ESP_LOGD(TAG, "Storage adapter is set, available=%d", this->storage_adapter_->is_available());
        else ESP_LOGW(TAG, "Storage adapter is NULL in loop");
    }

    if (!maps_loaded_ && this->storage_adapter_ != nullptr && this->storage_adapter_->is_available()) {
        static uint32_t last_retry_ms = 0;
        uint32_t now = millis();
        if (now - last_retry_ms > 5000) {
            last_retry_ms = now;
            ESP_LOGI(TAG, "Retrying Behavior Engine map loading...");
            this->load_maps();
        }
    }
#endif
}

void BehaviorEngine::load_maps() {
#ifdef USE_STORAGE
    if (this->storage_adapter_ == nullptr || !this->storage_adapter_->is_available()) {
        ESP_LOGW(TAG, "Storage adapter not ready, cannot load behavior maps");
        return;
    }
    
    // 1. Load Trigger Map FIRST
    this->load_trigger_map();
    
    // 2. Scan Animation Groups - REMOVED
    // We now use direct path access (flattened structure) to save memory
    // this->scan_animation_groups();
     
    this->maps_loaded_ = true;
    ESP_LOGI(TAG, "Behavior Engine maps loaded. Triggers: %d", 
             trigger_map_.size());
#else
    ESP_LOGW(TAG, "Storage not enabled, behavior engine disabled");
#endif
}

// void BehaviorEngine::scan_animation_groups() {
//      REMOVED - Using direct file access strategy
// }

// FNV-1a Hash
uint32_t BehaviorEngine::hash_trigger(const char* str) {
    uint32_t hash = 2166136261u;
    while (*str) {
        hash ^= (uint8_t)*str++;
        hash *= 16777619u;
    }
    return hash;
}

void BehaviorEngine::load_trigger_map() {
    trigger_map_.clear();
    group_name_pool_.clear();

#ifdef USE_STORAGE
    if (!this->storage_adapter_) return;

    std::string map_path = "/assets/cladToFileMaps/AnimationTriggerMap.json";
    
    // Use OOM-free streaming loader
    StorageAdapterStream stream(storage_adapter_, map_path);
    if (!stream.isOpen()) {
        ESP_LOGE(TAG, "Failed to open trigger map at: %s", map_path.c_str());
        return;
    }

    ESP_LOGI(TAG, "Loading trigger map from %s (Streaming)...", map_path.c_str());
    
    // Using streaming parser prevents allocating 50KB buffer + separate JsonDocument
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, stream);

    if (error) {
        ESP_LOGE(TAG, "Failed to parse AnimationTriggerMap.json: %s", error.c_str());
        return;
    }

    if (!doc.is<JsonArray>()) {
        ESP_LOGE(TAG, "AnimationTriggerMap root is not an array");
        return;
    }

    // Reserve approximate sizes
    trigger_map_.reserve(600); 
    group_name_pool_.reserve(15000); // Avg 25 chars * 600

    int loaded_count = 0;
    for (JsonObject item : doc.as<JsonArray>()) {
        const char *event = item["CladEvent"];
        const char *anim = item["AnimName"];
        
        if (event && anim) {
            uint32_t hash = hash_trigger(event);
            
            // Add string to pool
            uint16_t offset = group_name_pool_.size();
            size_t len = strlen(anim);
            
            // Allow pool to grow
            for(size_t i=0; i<len; i++) group_name_pool_.push_back(anim[i]);
            group_name_pool_.push_back(0); // Null terminator

            trigger_map_.push_back({hash, offset});
            loaded_count++;
        }
    }
    
    ESP_LOGI(TAG, "Compact Map Loaded: %d entries. Pool size: %d bytes. Streaming used.", 
             loaded_count, group_name_pool_.size());
             
    // Shrink persistent structures
    trigger_map_.shrink_to_fit();
    group_name_pool_.shrink_to_fit();

#endif
}

std::string BehaviorEngine::get_animation_for_trigger(const std::string &trigger_name) {
    if (!maps_loaded_) return "";
    
    // 1. Find Group Name for Trigger (Hash Lookup)
    uint32_t target_hash = hash_trigger(trigger_name.c_str());
    std::string group_name = "";
    
    for (const auto &entry : trigger_map_) {
        // Fast hash comparison
        if (entry.trigger_hash == target_hash) {
            // Retrieve string from pool
            if (entry.group_name_offset < group_name_pool_.size()) {
                group_name = &group_name_pool_[entry.group_name_offset];
            }
            break;
        }
    }
    
    if (group_name.empty()) {
        // Trigger not found in map
        // Fallback: Check if trigger name itself works as a group name (Direct Load)
        if (load_animation_group(trigger_name)) {
             ESP_LOGD(TAG, "Trigger '%s' used directly as group name", trigger_name.c_str());
             return select_animation_from_group(loaded_groups_[trigger_name]);
        }
        
        return ""; 
    }
    
    ESP_LOGD(TAG, "Trigger '%s' mapped to group '%s'", trigger_name.c_str(), group_name.c_str());
    
    // 2. Load Group if not loaded
    if (loaded_groups_.find(group_name) == loaded_groups_.end()) {
        if (!load_animation_group(group_name)) {
            ESP_LOGW(TAG, "Failed to load animation group '%s' for trigger '%s'", group_name.c_str(), trigger_name.c_str());
            return "";
        }
    }
    
    // 3. Select Animation from Group
    return select_animation_from_group(loaded_groups_[group_name]);
}

bool BehaviorEngine::load_animation_group(const std::string &group_name) {
    if (loaded_groups_.count(group_name)) return true; // Already loaded

    // Flattened structure strategy: Look directly in assets/animationGroups/
    std::string path = "assets/animationGroups/" + group_name + ".json";
    
#ifdef USE_STORAGE
    if (!this->storage_adapter_) return false;

    // Check if file exists directly
    if (!this->storage_adapter_->file_exists(path)) {
        ESP_LOGW(TAG, "Group file not found: %s (Make sure animationGroups is flattened)", path.c_str());
        return false;
    }

    std::vector<uint8_t> data;
    if (!this->storage_adapter_->read_file(path, data)) {
        ESP_LOGE(TAG, "Failed to read group file: %s", path.c_str());
        return false;
    }
    data.push_back(0); 
    
    // Use JsonDocument for auto-sizing (ArduinoJson 7)
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, (char*)data.data());

    if (error) {
        ESP_LOGE(TAG, "Failed to parse group file %s: %s", path.c_str(), error.c_str());
        return false;
    }
    
    AnimationGroup group;
    group.name = group_name;
    
    JsonArray anims = doc["Animations"];
    if (anims.isNull()) {
         ESP_LOGW(TAG, "Group %s has no Animations array", group_name.c_str());
         return false;
    }
    
    for (JsonObject anim : anims) {
        AnimationGroupEntry entry;
        entry.name = anim["Name"] | "";
        entry.weight = anim["Weight"] | 1; // Default weight 1
        entry.mood = anim["Mood"] | "Default";
        entry.cooldown = anim["CooldownTime_Sec"] | 0.0f;
        
        if (!entry.name.empty()) {
            group.entries.push_back(entry);
        }
    }
    
    // Move cleanup to AFTER parsing is done (avoid use-after-free)
    data.clear();
    data.shrink_to_fit();
    
    loaded_groups_[group_name] = group;
    ESP_LOGD(TAG, "Loaded group '%s' with %d entries", group_name.c_str(), group.entries.size());
    
    return true;
#else
    return false;
#endif
}

std::string BehaviorEngine::select_animation_from_group(AnimationGroup &group) {
    if (group.entries.empty()) return "";
    
    // Filter available animations (check cooldowns)
    std::vector<const AnimationGroupEntry*> candidates;
    float total_weight = 0;
    
    uint32_t now = millis();
    
    for (const auto &entry : group.entries) {
        // Check cooldown
        if (is_on_cooldown(entry.name)) {
            continue;
        }
        
        // Check mood (Optional: For now ignoring mood filtering or assume 'Default' is okay)
        // If we want to be fancy, we'd need MoodManager.
        
        candidates.push_back(&entry);
        total_weight += entry.weight;
    }
    
    if (candidates.empty()) {
        // All on cooldown? Or empty.
        // Fallback: Pick random one ignoring cooldown?
         ESP_LOGD(TAG, "All animations in group %s on cooldown or filtered. Picking random fallback.", group.name.c_str());
         if (group.entries.empty()) return "";
         int idx = random() % group.entries.size();
         const auto &entry = group.entries[idx];
         // Don't set cooldown on fallback?
         return entry.name;
    }
    
    // Weighted selection
    float r = (float)random() / (float)RAND_MAX * total_weight;
    
    const AnimationGroupEntry *selected = nullptr;
    for (const auto *entry : candidates) {
        r -= entry->weight;
        if (r <= 0) {
            selected = entry;
            break;
        }
    }
    
    if (!selected) selected = candidates.back();
    
    // Set cooldown
    if (selected->cooldown > 0) {
        set_cooldown(selected->name, selected->cooldown);
    }
    
    return selected->name;
}

bool BehaviorEngine::is_on_cooldown(const std::string &anim_name) {
    if (cooldowns_.count(anim_name)) {
        if (millis() < cooldowns_[anim_name]) {
            return true;
        } else {
             cooldowns_.erase(anim_name);
        }
    }
    return false;
}

void BehaviorEngine::set_cooldown(const std::string &anim_name, float cooldown_sec) {
    if (cooldown_sec > 0) {
        cooldowns_[anim_name] = millis() + (uint32_t)(cooldown_sec * 1000);
    }
}

// Convert Enum to String
std::string BehaviorEngine::trigger_to_string(int32_t trigger) {
    // Generate giant switch
    switch ((AnimationTrigger)trigger) {
        case AnimationTrigger::InvalidAnimTrigger: return "InvalidAnimTrigger";
        case AnimationTrigger::AlexaErrorLoop: return "AlexaErrorLoop";
        case AnimationTrigger::AlexaError2Idle: return "AlexaError2Idle";
        case AnimationTrigger::AlexaIdle2Error: return "AlexaIdle2Error";
        case AnimationTrigger::AlexaIdle2Listen: return "AlexaIdle2Listen";
        case AnimationTrigger::AlexaIdle2Speak: return "AlexaIdle2Speak";
        case AnimationTrigger::AlexaListenLoop: return "AlexaListenLoop";
        case AnimationTrigger::AlexaListen2Error: return "AlexaListen2Error";
        case AnimationTrigger::AlexaListen2Idle: return "AlexaListen2Idle";
        case AnimationTrigger::AlexaListen2Speak: return "AlexaListen2Speak";
        case AnimationTrigger::AlexaListen2Think: return "AlexaListen2Think";
        case AnimationTrigger::AlexaSignOut: return "AlexaSignOut";
        case AnimationTrigger::AlexaSpeakLoop: return "AlexaSpeakLoop";
        case AnimationTrigger::AlexaSpeak2Error: return "AlexaSpeak2Error";
        case AnimationTrigger::AlexaSpeak2Idle: return "AlexaSpeak2Idle";
        case AnimationTrigger::AlexaSpeak2Listen: return "AlexaSpeak2Listen";
        case AnimationTrigger::AlexaThinkLoop: return "AlexaThinkLoop";
        case AnimationTrigger::AlexaThink2Error: return "AlexaThink2Error";
        case AnimationTrigger::AlexaThink2Idle: return "AlexaThink2Idle";
        case AnimationTrigger::AlexaThink2Speak: return "AlexaThink2Speak";
        case AnimationTrigger::AlreadyAtFace: return "AlreadyAtFace";
        case AnimationTrigger::AudioOnlyHuh: return "AudioOnlyHuh";
        case AnimationTrigger::BlackJack_Deal: return "BlackJack_Deal";
        case AnimationTrigger::BlackJack_GoodLuck: return "BlackJack_GoodLuck";
        case AnimationTrigger::BlackJack_GetIn: return "BlackJack_GetIn";
        case AnimationTrigger::BlackJack_Idle: return "BlackJack_Idle";
        case AnimationTrigger::BlackJack_Quit: return "BlackJack_Quit";
        case AnimationTrigger::BlackJack_Response: return "BlackJack_Response";
        case AnimationTrigger::BlackJack_RtpIdle: return "BlackJack_RtpIdle";
        case AnimationTrigger::BlackJack_RtpPlayerNo: return "BlackJack_RtpPlayerNo";
        case AnimationTrigger::BlackJack_RtpPlayerYes: return "BlackJack_RtpPlayerYes";
        case AnimationTrigger::BlackJack_RtpRequest: return "BlackJack_RtpRequest";
        case AnimationTrigger::BlackJack_RtpTimeOut: return "BlackJack_RtpTimeOut";
        case AnimationTrigger::BlackJack_SpeechGetIn: return "BlackJack_SpeechGetIn";
        case AnimationTrigger::BlackJack_SpeechShortStatement: return "BlackJack_SpeechShortStatement";
        case AnimationTrigger::BlackJack_Spread: return "BlackJack_Spread";
        case AnimationTrigger::BlackJack_Swipe: return "BlackJack_Swipe";
        case AnimationTrigger::BlackJack_VictorBlackJackLose: return "BlackJack_VictorBlackJackLose";
        case AnimationTrigger::BlackJack_VictorBlackJackWin: return "BlackJack_VictorBlackJackWin";
        case AnimationTrigger::BlackJack_VictorBust: return "BlackJack_VictorBust";
        case AnimationTrigger::BlackJack_VictorLose: return "BlackJack_VictorLose";
        case AnimationTrigger::BlackJack_VictorPush: return "BlackJack_VictorPush";
        case AnimationTrigger::BlackJack_VictorWin: return "BlackJack_VictorWin";
        case AnimationTrigger::BumpObjectSlowGetIn: return "BumpObjectSlowGetIn";
        case AnimationTrigger::BumpObjectSlowLoop: return "BumpObjectSlowLoop";
        case AnimationTrigger::BumpObjectSlowGetOut: return "BumpObjectSlowGetOut";
        case AnimationTrigger::BumpObjectFastGetIn: return "BumpObjectFastGetIn";
        case AnimationTrigger::BumpObjectFastLoop: return "BumpObjectFastLoop";
        case AnimationTrigger::BumpObjectFastGetOut: return "BumpObjectFastGetOut";
        case AnimationTrigger::ChargerDockingAlreadyHere: return "ChargerDockingAlreadyHere";
        case AnimationTrigger::ChargerDockingDrivingEnd: return "ChargerDockingDrivingEnd";
        case AnimationTrigger::ChargerDockingDrivingLoop: return "ChargerDockingDrivingLoop";
        case AnimationTrigger::ChargerDockingDrivingStart: return "ChargerDockingDrivingStart";
        case AnimationTrigger::ChargerDockingFailure: return "ChargerDockingFailure";
        case AnimationTrigger::ChargerDockingLeftTurn: return "ChargerDockingLeftTurn";
        case AnimationTrigger::ChargerDockingRaiseLift: return "ChargerDockingRaiseLift";
        case AnimationTrigger::ChargerDockingRequest: return "ChargerDockingRequest";
        case AnimationTrigger::ChargerDockingRequestGetout: return "ChargerDockingRequestGetout";
        case AnimationTrigger::ChargerDockingRequestPickup: return "ChargerDockingRequestPickup";
        case AnimationTrigger::ChargerDockingRequestWaitLoop: return "ChargerDockingRequestWaitLoop";
        case AnimationTrigger::ChargerDockingRightTurn: return "ChargerDockingRightTurn";
        case AnimationTrigger::ChargerDockingSearchAfterCompletedSearch: return "ChargerDockingSearchAfterCompletedSearch";
        case AnimationTrigger::ChargerDockingSearchSingleTurn: return "ChargerDockingSearchSingleTurn";
        case AnimationTrigger::ChargerDockingSearchSingleTurnEnd: return "ChargerDockingSearchSingleTurnEnd";
        case AnimationTrigger::ChargerDockingSearchWaitForImages: return "ChargerDockingSearchWaitForImages";
        case AnimationTrigger::ChargerDockingSettle: return "ChargerDockingSettle";
        case AnimationTrigger::ChargerDockingSevereRequest: return "ChargerDockingSevereRequest";
        case AnimationTrigger::ChargerDockingSevereRequestGetout: return "ChargerDockingSevereRequestGetout";
        case AnimationTrigger::ChargerDockingSorryButLowBattery: return "ChargerDockingSorryButLowBattery";
        case AnimationTrigger::ChargerReaction: return "ChargerReaction";
        case AnimationTrigger::ClockGetIn: return "ClockGetIn";
        case AnimationTrigger::ClockGetOut: return "ClockGetOut";
        case AnimationTrigger::ComeHereStart: return "ComeHereStart";
        case AnimationTrigger::ComeHereSuccess: return "ComeHereSuccess";
        case AnimationTrigger::ConnectWakeUp: return "ConnectWakeUp";
        case AnimationTrigger::ConnectWakeUpLights: return "ConnectWakeUpLights";
        case AnimationTrigger::ConnectToCubeFailure: return "ConnectToCubeFailure";
        case AnimationTrigger::ConnectToCubeGetIn: return "ConnectToCubeGetIn";
        case AnimationTrigger::ConnectToCubeLoop: return "ConnectToCubeLoop";
        case AnimationTrigger::ConnectToCubeLostConnection: return "ConnectToCubeLostConnection";
        case AnimationTrigger::ConnectToCubeSuccess: return "ConnectToCubeSuccess";
        case AnimationTrigger::CountingFastLoop: return "CountingFastLoop";
        case AnimationTrigger::CountingGetInEven: return "CountingGetInEven";
        case AnimationTrigger::CountingGetInOdd: return "CountingGetInOdd";
        case AnimationTrigger::CountingGetOut: return "CountingGetOut";
        case AnimationTrigger::CountingSlowLoop: return "CountingSlowLoop";
        case AnimationTrigger::CubePounceBackup: return "CubePounceBackup";
        case AnimationTrigger::CubePounceDriveGetIn: return "CubePounceDriveGetIn";
        case AnimationTrigger::CubePounceDriveLoop: return "CubePounceDriveLoop";
        case AnimationTrigger::CubePounceDriveGetOut: return "CubePounceDriveGetOut";
        case AnimationTrigger::CubePounceFake: return "CubePounceFake";
        case AnimationTrigger::CubePounceGetIn: return "CubePounceGetIn";
        case AnimationTrigger::CubePounceGetOutBored: return "CubePounceGetOutBored";
        case AnimationTrigger::CubePounceGetReady: return "CubePounceGetReady";
        case AnimationTrigger::CubePounceGetUnready: return "CubePounceGetUnready";
        case AnimationTrigger::CubePounceIdleLiftDown: return "CubePounceIdleLiftDown";
        case AnimationTrigger::CubePounceIdleLiftUp: return "CubePounceIdleLiftUp";
        case AnimationTrigger::CubePounceLoseHand: return "CubePounceLoseHand";
        case AnimationTrigger::CubePounceLoseSession: return "CubePounceLoseSession";
        case AnimationTrigger::CubePouncePounceClose: return "CubePouncePounceClose";
        case AnimationTrigger::CubePouncePounceNormal: return "CubePouncePounceNormal";
        case AnimationTrigger::CubePounceReactToCube: return "CubePounceReactToCube";
        case AnimationTrigger::CubePounceWinHand: return "CubePounceWinHand";
        case AnimationTrigger::CubePounceWinSession: return "CubePounceWinSession";
        case AnimationTrigger::DanceBeatCantDoThat: return "DanceBeatCantDoThat";
        case AnimationTrigger::DanceBeatEyeHold: return "DanceBeatEyeHold";
        case AnimationTrigger::DanceBeatGetIn: return "DanceBeatGetIn";
        case AnimationTrigger::DanceBeatGetOut: return "DanceBeatGetOut";
        case AnimationTrigger::DanceBeatGetReady: return "DanceBeatGetReady";
        case AnimationTrigger::DanceBeatListening: return "DanceBeatListening";
        case AnimationTrigger::DanceBeatNoBeatDetected: return "DanceBeatNoBeatDetected";
        case AnimationTrigger::DockEndDefault: return "DockEndDefault";
        case AnimationTrigger::DockLoopDefault: return "DockLoopDefault";
        case AnimationTrigger::DockStartDefault: return "DockStartDefault";
        case AnimationTrigger::DriveEndAngry: return "DriveEndAngry";
        case AnimationTrigger::DriveEndDefault: return "DriveEndDefault";
        case AnimationTrigger::DriveEndHappy: return "DriveEndHappy";
        case AnimationTrigger::DriveEndLaunch: return "DriveEndLaunch";
        case AnimationTrigger::DriveLoopAngry: return "DriveLoopAngry";
        case AnimationTrigger::DriveLoopDefault: return "DriveLoopDefault";
        case AnimationTrigger::DriveLoopHappy: return "DriveLoopHappy";
        case AnimationTrigger::DriveLoopLaunch: return "DriveLoopLaunch";
        case AnimationTrigger::DriveOffChargerFarLeft: return "DriveOffChargerFarLeft";
        case AnimationTrigger::DriveOffChargerFarRight: return "DriveOffChargerFarRight";
        case AnimationTrigger::DriveOffChargerLeft: return "DriveOffChargerLeft";
        case AnimationTrigger::DriveOffChargerRight: return "DriveOffChargerRight";
        case AnimationTrigger::DriveOffChargerStraight: return "DriveOffChargerStraight";
        case AnimationTrigger::DriveStartAngry: return "DriveStartAngry";
        case AnimationTrigger::DriveStartDefault: return "DriveStartDefault";
        case AnimationTrigger::DriveStartHappy: return "DriveStartHappy";
        case AnimationTrigger::DriveStartLaunch: return "DriveStartLaunch";
        case AnimationTrigger::EyeColorIdle: return "EyeColorIdle";
        case AnimationTrigger::EyeColorGetIn: return "EyeColorGetIn";
        case AnimationTrigger::EyeColorGetOut: return "EyeColorGetOut";
        case AnimationTrigger::EyeColorSwitch: return "EyeColorSwitch";
        case AnimationTrigger::EyeContactLookLoop: return "EyeContactLookLoop";
        case AnimationTrigger::ExploringHuhClose: return "ExploringHuhClose";
        case AnimationTrigger::ExploringHuhFar: return "ExploringHuhFar";
        case AnimationTrigger::ExploringLookAround: return "ExploringLookAround";
        case AnimationTrigger::ExploringLookAtHuman: return "ExploringLookAtHuman";
        case AnimationTrigger::ExploringQuickScan: return "ExploringQuickScan";
        case AnimationTrigger::ExploringReactToHandGetIn: return "ExploringReactToHandGetIn";
        case AnimationTrigger::ExploringReactToHandDrive: return "ExploringReactToHandDrive";
        case AnimationTrigger::ExploringReactToHandLift: return "ExploringReactToHandLift";
        case AnimationTrigger::ExploringReactToHandReaction: return "ExploringReactToHandReaction";
        case AnimationTrigger::ExploringReactToHandGetOut: return "ExploringReactToHandGetOut";
        case AnimationTrigger::ExploringScanToLeft: return "ExploringScanToLeft";
        case AnimationTrigger::ExploringScanToRight: return "ExploringScanToRight";
        case AnimationTrigger::ExploringScanCenterFromLeft: return "ExploringScanCenterFromLeft";
        case AnimationTrigger::ExploringScanCenterFromRight: return "ExploringScanCenterFromRight";
        case AnimationTrigger::FacePlantRoll: return "FacePlantRoll";
        case AnimationTrigger::FacePlantRollArmUp: return "FacePlantRollArmUp";
        case AnimationTrigger::FailedToRightFromFace: return "FailedToRightFromFace";
        case AnimationTrigger::FetchCubeFailure: return "FetchCubeFailure";
        case AnimationTrigger::FetchCubeSetDown: return "FetchCubeSetDown";
        case AnimationTrigger::FetchCubeSuccess: return "FetchCubeSuccess";
        case AnimationTrigger::FoundFace: return "FoundFace";
        case AnimationTrigger::FindCubeReactToCube: return "FindCubeReactToCube";
        case AnimationTrigger::FindCubeTurns: return "FindCubeTurns";
        case AnimationTrigger::FindCubeWaitLoop: return "FindCubeWaitLoop";
        case AnimationTrigger::FistBumpIdle: return "FistBumpIdle";
        case AnimationTrigger::FistBumpRequestOnce: return "FistBumpRequestOnce";
        case AnimationTrigger::FistBumpRequestRetry: return "FistBumpRequestRetry";
        case AnimationTrigger::FistBumpSuccess: return "FistBumpSuccess";
        case AnimationTrigger::FistBumpLeftHanging: return "FistBumpLeftHanging";
        case AnimationTrigger::FlipDownFromBack: return "FlipDownFromBack";
        case AnimationTrigger::FrustratedByFailureMajor: return "FrustratedByFailureMajor";
        case AnimationTrigger::Feedback_GoodRobot: return "Feedback_GoodRobot";
        case AnimationTrigger::Feedback_BadRobot: return "Feedback_BadRobot";
        case AnimationTrigger::Feedback_ShutUp: return "Feedback_ShutUp";
        case AnimationTrigger::Feedback_BeQuiet: return "Feedback_BeQuiet";
        case AnimationTrigger::Feedback_Apology: return "Feedback_Apology";
        case AnimationTrigger::Feedback_ILoveYou: return "Feedback_ILoveYou";
        case AnimationTrigger::Feedback_MeanWords: return "Feedback_MeanWords";
        case AnimationTrigger::GoToSleepGetIn: return "GoToSleepGetIn";
        case AnimationTrigger::GoToSleepOff: return "GoToSleepOff";
        case AnimationTrigger::GoToSleepSleeping: return "GoToSleepSleeping";
        case AnimationTrigger::GazingLookAtFacesGetInLeft: return "GazingLookAtFacesGetInLeft";
        case AnimationTrigger::GazingLookAtFacesGetInRight: return "GazingLookAtFacesGetInRight";
        case AnimationTrigger::GazingLookAtFacesTurnLeft: return "GazingLookAtFacesTurnLeft";
        case AnimationTrigger::GazingLookAtFacesTurnRight: return "GazingLookAtFacesTurnRight";
        case AnimationTrigger::GazingLookAtSurfacesGetInLeft: return "GazingLookAtSurfacesGetInLeft";
        case AnimationTrigger::GazingLookAtSurfacesGetInRight: return "GazingLookAtSurfacesGetInRight";
        case AnimationTrigger::GazingLookAtSurfaceReaction: return "GazingLookAtSurfaceReaction";
        case AnimationTrigger::GazingLookAtSurfaceTurnLeft: return "GazingLookAtSurfaceTurnLeft";
        case AnimationTrigger::GazingLookAtSurfacesTurnRight: return "GazingLookAtSurfacesTurnRight";
        case AnimationTrigger::GazingLookAtVectorReaction: return "GazingLookAtVectorReaction";
        case AnimationTrigger::HighTemperatureWarningFace: return "HighTemperatureWarningFace";
        case AnimationTrigger::HeldOnPalmEdgeNervous: return "HeldOnPalmEdgeNervous";
        case AnimationTrigger::HeldOnPalmEdgeRelaxed: return "HeldOnPalmEdgeRelaxed";
        case AnimationTrigger::HeldOnPalmGetInNervous: return "HeldOnPalmGetInNervous";
        case AnimationTrigger::HeldOnPalmGetInRelaxed: return "HeldOnPalmGetInRelaxed";
        case AnimationTrigger::HeldOnPalmReactToJolt: return "HeldOnPalmReactToJolt";
        case AnimationTrigger::HeldOnPalmLookingNervous: return "HeldOnPalmLookingNervous";
        case AnimationTrigger::HeldOnPalmNestling: return "HeldOnPalmNestling";
        case AnimationTrigger::HeldOnPalmPickupNervous: return "HeldOnPalmPickupNervous";
        case AnimationTrigger::HeldOnPalmPickupRelaxed: return "HeldOnPalmPickupRelaxed";
        case AnimationTrigger::HeldOnPalmPutDownNervous: return "HeldOnPalmPutDownNervous";
        case AnimationTrigger::HeldOnPalmPutDownRelaxed: return "HeldOnPalmPutDownRelaxed";
        case AnimationTrigger::HeldOnPalmRollOff: return "HeldOnPalmRollOff";
        case AnimationTrigger::HeldOnPalmTransitionToRelaxed: return "HeldOnPalmTransitionToRelaxed";
        case AnimationTrigger::SeasonalHappyHolidays: return "SeasonalHappyHolidays";
        case AnimationTrigger::SeasonalHappyNewYear: return "SeasonalHappyNewYear";
        case AnimationTrigger::ICantDoThat: return "ICantDoThat";
        case AnimationTrigger::InitialWakeUp: return "InitialWakeUp";
        case AnimationTrigger::IntentionalPerformance: return "IntentionalPerformance";
        case AnimationTrigger::InteractWithFaceTrackingIdle: return "InteractWithFaceTrackingIdle";
        case AnimationTrigger::InteractWithFacesInitialNamed: return "InteractWithFacesInitialNamed";
        case AnimationTrigger::InteractWithFacesInitialUnnamed: return "InteractWithFacesInitialUnnamed";
        case AnimationTrigger::InvestigateHeldCubeGetIn: return "InvestigateHeldCubeGetIn";
        case AnimationTrigger::InvestigateHeldCubeGetOutBored: return "InvestigateHeldCubeGetOutBored";
        case AnimationTrigger::InvestigateHeldCubeGetOutCubeLost: return "InvestigateHeldCubeGetOutCubeLost";
        case AnimationTrigger::InvestigateHeldCubeOnSetDown: return "InvestigateHeldCubeOnSetDown";
        case AnimationTrigger::InvestigateHeldCubeTrackingLoop: return "InvestigateHeldCubeTrackingLoop";
        case AnimationTrigger::KnowledgeGraphGetIn: return "KnowledgeGraphGetIn";
        case AnimationTrigger::KnowledgeGraphListening: return "KnowledgeGraphListening";
        case AnimationTrigger::KnowledgeGraphSearchingGetIn: return "KnowledgeGraphSearchingGetIn";
        case AnimationTrigger::KnowledgeGraphSearching: return "KnowledgeGraphSearching";
        case AnimationTrigger::KnowledgeGraphSearchingGetOutSuccess: return "KnowledgeGraphSearchingGetOutSuccess";
        case AnimationTrigger::KnowledgeGraphSearchingFail: return "KnowledgeGraphSearchingFail";
        case AnimationTrigger::KnowledgeGraphSearchingFailGetOut: return "KnowledgeGraphSearchingFailGetOut";
        case AnimationTrigger::KnowledgeGraphAnswer: return "KnowledgeGraphAnswer";
        case AnimationTrigger::KnowledgeGraphGetOut: return "KnowledgeGraphGetOut";
        case AnimationTrigger::KnowledgeGraphSuccessReaction: return "KnowledgeGraphSuccessReaction";
        case AnimationTrigger::LookAround: return "LookAround";
        case AnimationTrigger::LookAtDevice: return "LookAtDevice";
        case AnimationTrigger::LookAtDeviceGetIn: return "LookAtDeviceGetIn";
        case AnimationTrigger::LookAtDeviceGetOut: return "LookAtDeviceGetOut";
        case AnimationTrigger::LookInPlaceForFacesBodyPause_Active: return "LookInPlaceForFacesBodyPause_Active";
        case AnimationTrigger::LookInPlaceForFacesBodyPause: return "LookInPlaceForFacesBodyPause";
        case AnimationTrigger::LookInPlaceForFacesHeadMovePause: return "LookInPlaceForFacesHeadMovePause";
        case AnimationTrigger::LookAtUserEndearingly: return "LookAtUserEndearingly";
        case AnimationTrigger::LowlightChargerSearchGetin: return "LowlightChargerSearchGetin";
        case AnimationTrigger::LowlightChargerSearchLoop: return "LowlightChargerSearchLoop";
        case AnimationTrigger::LowlightChargerSearchGetout: return "LowlightChargerSearchGetout";
        case AnimationTrigger::MeetVictorConfusion: return "MeetVictorConfusion";
        case AnimationTrigger::MeetVictorGetIn: return "MeetVictorGetIn";
        case AnimationTrigger::MeetVictorLookFace: return "MeetVictorLookFace";
        case AnimationTrigger::MeetVictorLookFaceInterrupt: return "MeetVictorLookFaceInterrupt";
        case AnimationTrigger::MeetVictorSayName: return "MeetVictorSayName";
        case AnimationTrigger::MeetVictorSayNameAgain: return "MeetVictorSayNameAgain";
        case AnimationTrigger::MeetVictorSawWrongFace: return "MeetVictorSawWrongFace";
        case AnimationTrigger::MeetVictorDuplicateName: return "MeetVictorDuplicateName";
        case AnimationTrigger::MessagingMessageGetIn: return "MessagingMessageGetIn";
        case AnimationTrigger::MessagingMessageLoop: return "MessagingMessageLoop";
        case AnimationTrigger::MessagingMessageGetOut: return "MessagingMessageGetOut";
        case AnimationTrigger::MessagingMessageRecordReaction: return "MessagingMessageRecordReaction";
        case AnimationTrigger::MessagingMessageDeletedShort: return "MessagingMessageDeletedShort";
        case AnimationTrigger::MessagingMessageRewind: return "MessagingMessageRewind";
        case AnimationTrigger::MovementDriveBackward: return "MovementDriveBackward";
        case AnimationTrigger::MovementTurnLeft: return "MovementTurnLeft";
        case AnimationTrigger::MovementTurnRight: return "MovementTurnRight";
        case AnimationTrigger::MovementTurnAround: return "MovementTurnAround";
        case AnimationTrigger::NoCloudGetIn: return "NoCloudGetIn";
        case AnimationTrigger::NoCloudIcon: return "NoCloudIcon";
        case AnimationTrigger::NoWifiGetIn: return "NoWifiGetIn";
        case AnimationTrigger::NoWifiSearching: return "NoWifiSearching";
        case AnimationTrigger::NoWifiIcon: return "NoWifiIcon";
        case AnimationTrigger::NeutralFace: return "NeutralFace";
        case AnimationTrigger::NothingToDoBoredIdle: return "NothingToDoBoredIdle";
        case AnimationTrigger::ObservingIdleEyesOnly: return "ObservingIdleEyesOnly";
        case AnimationTrigger::ObservingIdleWithHeadLookingStraight: return "ObservingIdleWithHeadLookingStraight";
        case AnimationTrigger::ObservingIdleWithHeadLookingUp: return "ObservingIdleWithHeadLookingUp";
        case AnimationTrigger::ObservingLookStraight: return "ObservingLookStraight";
        case AnimationTrigger::ObservingLookUp: return "ObservingLookUp";
        case AnimationTrigger::ObservingOnCharger: return "ObservingOnCharger";
        case AnimationTrigger::ObservingOnChargerGetIn: return "ObservingOnChargerGetIn";
        case AnimationTrigger::ObservingOnChargerGetOut: return "ObservingOnChargerGetOut";
        case AnimationTrigger::OnboardingComeHere: return "OnboardingComeHere";
        case AnimationTrigger::OnboardingComeHereGetOut: return "OnboardingComeHereGetOut";
        case AnimationTrigger::OnboardingCubeDriveGetIn: return "OnboardingCubeDriveGetIn";
        case AnimationTrigger::OnboardingCubeDriveGetOut: return "OnboardingCubeDriveGetOut";
        case AnimationTrigger::OnboardingCubeDriveLoop: return "OnboardingCubeDriveLoop";
        case AnimationTrigger::OnboardingCubeHuh: return "OnboardingCubeHuh";
        case AnimationTrigger::OnboardingDriveOffCharger: return "OnboardingDriveOffCharger";
        case AnimationTrigger::OnboardingDriveOffCharger_1p0: return "OnboardingDriveOffCharger_1p0";
        case AnimationTrigger::OnboardingLookAtUserGetOut_1p0: return "OnboardingLookAtUserGetOut_1p0";
        case AnimationTrigger::OnboardingLookAround: return "OnboardingLookAround";
        case AnimationTrigger::OnboardingLookDown: return "OnboardingLookDown";
        case AnimationTrigger::OnboardingLookForCube: return "OnboardingLookForCube";
        case AnimationTrigger::OnboardingLookAtPhoneUp: return "OnboardingLookAtPhoneUp";
        case AnimationTrigger::OnboardingLookAtPhoneLoop: return "OnboardingLookAtPhoneLoop";
        case AnimationTrigger::OnboardingLookAtPhoneDown: return "OnboardingLookAtPhoneDown";
        case AnimationTrigger::OnboardingLookAtUser: return "OnboardingLookAtUser";
        case AnimationTrigger::OnboardingListenGetIn: return "OnboardingListenGetIn";
        case AnimationTrigger::OnboardingListenGetOut: return "OnboardingListenGetOut";
        case AnimationTrigger::OnboardingReactToFaceHappy: return "OnboardingReactToFaceHappy";
        case AnimationTrigger::OnboardingWakeUp: return "OnboardingWakeUp";
        case AnimationTrigger::OnboardingWakeWordGetIn: return "OnboardingWakeWordGetIn";
        case AnimationTrigger::OnboardingWakeWordSuccess: return "OnboardingWakeWordSuccess";
        case AnimationTrigger::PettingLevel1: return "PettingLevel1";
        case AnimationTrigger::PettingLevel2: return "PettingLevel2";
        case AnimationTrigger::PettingLevel3: return "PettingLevel3";
        case AnimationTrigger::PettingLevel4: return "PettingLevel4";
        case AnimationTrigger::PettingLevel1Getout: return "PettingLevel1Getout";
        case AnimationTrigger::PettingLevel2Getout: return "PettingLevel2Getout";
        case AnimationTrigger::PettingLevel3Getout: return "PettingLevel3Getout";
        case AnimationTrigger::PettingLevel4Getout: return "PettingLevel4Getout";
        case AnimationTrigger::PettingBlissLoop: return "PettingBlissLoop";
        case AnimationTrigger::PettingBlissGetout: return "PettingBlissGetout";
        case AnimationTrigger::PickupCubePreperation: return "PickupCubePreperation";
        case AnimationTrigger::PickupCubeRetry: return "PickupCubeRetry";
        case AnimationTrigger::PickupCubeSuccess: return "PickupCubeSuccess";
        case AnimationTrigger::PlaceCubeByChargerFail: return "PlaceCubeByChargerFail";
        case AnimationTrigger::PlaceCubeByChargerReactToCharger: return "PlaceCubeByChargerReactToCharger";
        case AnimationTrigger::PlaceCubeByChargerSuccess: return "PlaceCubeByChargerSuccess";
        case AnimationTrigger::PlanningGetIn: return "PlanningGetIn";
        case AnimationTrigger::PlanningLoop: return "PlanningLoop";
        case AnimationTrigger::PlanningGetOut: return "PlanningGetOut";
        case AnimationTrigger::PokeObjectGetIn: return "PokeObjectGetIn";
        case AnimationTrigger::PokeObjectDriveLoop: return "PokeObjectDriveLoop";
        case AnimationTrigger::PokeObjectGetOut: return "PokeObjectGetOut";
        case AnimationTrigger::PopAWheelieInitial: return "PopAWheelieInitial";
        case AnimationTrigger::PopAWheeliePreActionNamedFace: return "PopAWheeliePreActionNamedFace";
        case AnimationTrigger::PopAWheeliePreActionUnnamedFace: return "PopAWheeliePreActionUnnamedFace";
        case AnimationTrigger::PopAWheelieRealign: return "PopAWheelieRealign";
        case AnimationTrigger::PopAWheelieRetry: return "PopAWheelieRetry";
        case AnimationTrigger::PounceFail: return "PounceFail";
        case AnimationTrigger::PounceSuccess: return "PounceSuccess";
        case AnimationTrigger::PounceWProxForward: return "PounceWProxForward";
        case AnimationTrigger::PRDemoGreeting: return "PRDemoGreeting";
        case AnimationTrigger::PutDownBlockKeepAlive: return "PutDownBlockKeepAlive";
        case AnimationTrigger::PutDownBlockPutDown: return "PutDownBlockPutDown";
        case AnimationTrigger::ReactToCliff: return "ReactToCliff";
        case AnimationTrigger::ReactToCliffBack: return "ReactToCliffBack";
        case AnimationTrigger::ReactToCliffBackLeft: return "ReactToCliffBackLeft";
        case AnimationTrigger::ReactToCliffBackRight: return "ReactToCliffBackRight";
        case AnimationTrigger::ReactToCliffFront: return "ReactToCliffFront";
        case AnimationTrigger::ReactToCliffFrontLeft: return "ReactToCliffFrontLeft";
        case AnimationTrigger::ReactToCliffFrontRight: return "ReactToCliffFrontRight";
        case AnimationTrigger::ReactToCliffTurnLeft60: return "ReactToCliffTurnLeft60";
        case AnimationTrigger::ReactToCliffTurnLeft120: return "ReactToCliffTurnLeft120";
        case AnimationTrigger::ReactToCliffTurnLeft180: return "ReactToCliffTurnLeft180";
        case AnimationTrigger::ReactToCliffTurnRight60: return "ReactToCliffTurnRight60";
        case AnimationTrigger::ReactToCliffTurnRight120: return "ReactToCliffTurnRight120";
        case AnimationTrigger::ReactToCliffTurnRight180: return "ReactToCliffTurnRight180";
        case AnimationTrigger::ReactToCubeSearchForCubeLvl1: return "ReactToCubeSearchForCubeLvl1";
        case AnimationTrigger::ReactToCubeSearchForCubeLvl2: return "ReactToCubeSearchForCubeLvl2";
        case AnimationTrigger::ReactToCubeSearchForCubeLvl3: return "ReactToCubeSearchForCubeLvl3";
        case AnimationTrigger::ReactToCubeTapCubeFound: return "ReactToCubeTapCubeFound";
        case AnimationTrigger::ReactToCubeTapCubeNotFound: return "ReactToCubeTapCubeNotFound";
        case AnimationTrigger::ReactToCubeTapCubeTappedLvl1: return "ReactToCubeTapCubeTappedLvl1";
        case AnimationTrigger::ReactToCubeTapCubeTappedLvl2: return "ReactToCubeTapCubeTappedLvl2";
        case AnimationTrigger::ReactToCubeTapCubeTappedLvl3: return "ReactToCubeTapCubeTappedLvl3";
        case AnimationTrigger::ReactToCubeTapInteractionGetOut: return "ReactToCubeTapInteractionGetOut";
        case AnimationTrigger::ReactToCubeTapInteractionLoop: return "ReactToCubeTapInteractionLoop";
        case AnimationTrigger::ReactToDarkness: return "ReactToDarkness";
        case AnimationTrigger::ReactToGoodBye: return "ReactToGoodBye";
        case AnimationTrigger::ReactToGoodMorning: return "ReactToGoodMorning";
        case AnimationTrigger::ReactToGoodNight: return "ReactToGoodNight";
        case AnimationTrigger::ReactToGreeting: return "ReactToGreeting";
        case AnimationTrigger::ReactToHabitat: return "ReactToHabitat";
        case AnimationTrigger::ReactToMotionLeft: return "ReactToMotionLeft";
        case AnimationTrigger::ReactToMotionTurnLeft: return "ReactToMotionTurnLeft";
        case AnimationTrigger::ReactToMotionRight: return "ReactToMotionRight";
        case AnimationTrigger::ReactToMotionTurnRight: return "ReactToMotionTurnRight";
        case AnimationTrigger::ReactToMotionUp: return "ReactToMotionUp";
        case AnimationTrigger::ReactToMotionTurnUp: return "ReactToMotionTurnUp";
        case AnimationTrigger::ReactToMotionLeftGetout: return "ReactToMotionLeftGetout";
        case AnimationTrigger::ReactToMotionRightGetout: return "ReactToMotionRightGetout";
        case AnimationTrigger::ReactToMotionUpGetout: return "ReactToMotionUpGetout";
        case AnimationTrigger::ReactToObstacle: return "ReactToObstacle";
        case AnimationTrigger::ReactToOnLeftSideGetIn: return "ReactToOnLeftSideGetIn";
        case AnimationTrigger::ReactToOnLeftSideLoop: return "ReactToOnLeftSideLoop";
        case AnimationTrigger::ReactToOnRightSideGetIn: return "ReactToOnRightSideGetIn";
        case AnimationTrigger::ReactToOnRightSideLoop: return "ReactToOnRightSideLoop";
        case AnimationTrigger::ReactToOnSideEffort: return "ReactToOnSideEffort";
        case AnimationTrigger::ReactToOnSideGetOut: return "ReactToOnSideGetOut";
        case AnimationTrigger::ReactToPerchedOnBlock: return "ReactToPerchedOnBlock";
        case AnimationTrigger::ReactToPickupInitial: return "ReactToPickupInitial";
        case AnimationTrigger::ReactToPickupLoop: return "ReactToPickupLoop";
        case AnimationTrigger::ReactToPutDown: return "ReactToPutDown";
        case AnimationTrigger::ReactToShake_GetIn: return "ReactToShake_GetIn";
        case AnimationTrigger::ReactToShake_Lvl1Loop: return "ReactToShake_Lvl1Loop";
        case AnimationTrigger::ReactToShake_Lvl1Waiting: return "ReactToShake_Lvl1Waiting";
        case AnimationTrigger::ReactToShake_Lvl1InHand: return "ReactToShake_Lvl1InHand";
        case AnimationTrigger::ReactToShake_Lvl1OnGround: return "ReactToShake_Lvl1OnGround";
        case AnimationTrigger::ReactToShake_Lvl2Loop: return "ReactToShake_Lvl2Loop";
        case AnimationTrigger::ReactToShake_Lvl2Waiting: return "ReactToShake_Lvl2Waiting";
        case AnimationTrigger::ReactToShake_Lvl2InHand: return "ReactToShake_Lvl2InHand";
        case AnimationTrigger::ReactToShake_Lvl2OnGround: return "ReactToShake_Lvl2OnGround";
        case AnimationTrigger::ReactToShake_Lvl3Loop: return "ReactToShake_Lvl3Loop";
        case AnimationTrigger::ReactToShake_Lvl3Waiting: return "ReactToShake_Lvl3Waiting";
        case AnimationTrigger::ReactToShake_Lvl3InHand: return "ReactToShake_Lvl3InHand";
        case AnimationTrigger::ReactToShake_Lvl3OnGround: return "ReactToShake_Lvl3OnGround";
        case AnimationTrigger::ReactToShakeSnowGlobe_GetIn: return "ReactToShakeSnowGlobe_GetIn";
        case AnimationTrigger::ReactToShakeSnowGlobe_Lvl1Loop: return "ReactToShakeSnowGlobe_Lvl1Loop";
        case AnimationTrigger::ReactToShakeSnowGlobe_Lvl1Waiting: return "ReactToShakeSnowGlobe_Lvl1Waiting";
        case AnimationTrigger::ReactToShakeSnowGlobe_Lvl1InHand: return "ReactToShakeSnowGlobe_Lvl1InHand";
        case AnimationTrigger::ReactToShakeSnowGlobe_Lvl1OnGround: return "ReactToShakeSnowGlobe_Lvl1OnGround";
        case AnimationTrigger::RTS_OffCharger_Sleep_Ambient: return "RTS_OffCharger_Sleep_Ambient";
        case AnimationTrigger::RTS_OffCharger_Sleep_Front: return "RTS_OffCharger_Sleep_Front";
        case AnimationTrigger::RTS_OffCharger_Sleep_30Left: return "RTS_OffCharger_Sleep_30Left";
        case AnimationTrigger::RTS_OffCharger_Sleep_30Right: return "RTS_OffCharger_Sleep_30Right";
        case AnimationTrigger::RTS_OffCharger_Sleep_60Left: return "RTS_OffCharger_Sleep_60Left";
        case AnimationTrigger::RTS_OffCharger_Sleep_60Right: return "RTS_OffCharger_Sleep_60Right";
        case AnimationTrigger::RTS_OffCharger_Sleep_Left: return "RTS_OffCharger_Sleep_Left";
        case AnimationTrigger::RTS_OffCharger_Sleep_Right: return "RTS_OffCharger_Sleep_Right";
        case AnimationTrigger::RTS_OffCharger_Sleep_120Left: return "RTS_OffCharger_Sleep_120Left";
        case AnimationTrigger::RTS_OffCharger_Sleep_120Right: return "RTS_OffCharger_Sleep_120Right";
        case AnimationTrigger::RTS_OffCharger_Sleep_150Left: return "RTS_OffCharger_Sleep_150Left";
        case AnimationTrigger::RTS_OffCharger_Sleep_150Right: return "RTS_OffCharger_Sleep_150Right";
        case AnimationTrigger::RTS_OffCharger_Sleep_Back: return "RTS_OffCharger_Sleep_Back";
        case AnimationTrigger::RTS_OnCharger_Sleep_Ambient: return "RTS_OnCharger_Sleep_Ambient";
        case AnimationTrigger::RTS_OnCharger_Sleep_Front: return "RTS_OnCharger_Sleep_Front";
        case AnimationTrigger::RTS_OnCharger_Sleep_30Left: return "RTS_OnCharger_Sleep_30Left";
        case AnimationTrigger::RTS_OnCharger_Sleep_30Right: return "RTS_OnCharger_Sleep_30Right";
        case AnimationTrigger::RTS_OnCharger_Sleep_60Left: return "RTS_OnCharger_Sleep_60Left";
        case AnimationTrigger::RTS_OnCharger_Sleep_60Right: return "RTS_OnCharger_Sleep_60Right";
        case AnimationTrigger::RTS_OnCharger_Sleep_Left: return "RTS_OnCharger_Sleep_Left";
        case AnimationTrigger::RTS_OnCharger_Sleep_Right: return "RTS_OnCharger_Sleep_Right";
        case AnimationTrigger::RTS_OnCharger_Sleep_120Left: return "RTS_OnCharger_Sleep_120Left";
        case AnimationTrigger::RTS_OnCharger_Sleep_120Right: return "RTS_OnCharger_Sleep_120Right";
        case AnimationTrigger::RTS_OnCharger_Sleep_150Left: return "RTS_OnCharger_Sleep_150Left";
        case AnimationTrigger::RTS_OnCharger_Sleep_150Right: return "RTS_OnCharger_Sleep_150Right";
        case AnimationTrigger::RTS_OnCharger_Sleep_Back: return "RTS_OnCharger_Sleep_Back";
        case AnimationTrigger::RTS_OffCharger_Awake_Ambient: return "RTS_OffCharger_Awake_Ambient";
        case AnimationTrigger::RTS_OffCharger_Awake_Front: return "RTS_OffCharger_Awake_Front";
        case AnimationTrigger::RTS_OffCharger_Awake_30Left: return "RTS_OffCharger_Awake_30Left";
        case AnimationTrigger::RTS_OffCharger_Awake_30Right: return "RTS_OffCharger_Awake_30Right";
        case AnimationTrigger::RTS_OffCharger_Awake_60Left: return "RTS_OffCharger_Awake_60Left";
        case AnimationTrigger::RTS_OffCharger_Awake_60Right: return "RTS_OffCharger_Awake_60Right";
        case AnimationTrigger::RTS_OffCharger_Awake_Left: return "RTS_OffCharger_Awake_Left";
        case AnimationTrigger::RTS_OffCharger_Awake_Right: return "RTS_OffCharger_Awake_Right";
        case AnimationTrigger::RTS_OffCharger_Awake_120Left: return "RTS_OffCharger_Awake_120Left";
        case AnimationTrigger::RTS_OffCharger_Awake_120Right: return "RTS_OffCharger_Awake_120Right";
        case AnimationTrigger::RTS_OffCharger_Awake_150Left: return "RTS_OffCharger_Awake_150Left";
        case AnimationTrigger::RTS_OffCharger_Awake_150Right: return "RTS_OffCharger_Awake_150Right";
        case AnimationTrigger::RTS_OffCharger_Awake_Back: return "RTS_OffCharger_Awake_Back";
        case AnimationTrigger::RTS_OnCharger_Awake_Ambient: return "RTS_OnCharger_Awake_Ambient";
        case AnimationTrigger::RTS_OnCharger_Awake_Front: return "RTS_OnCharger_Awake_Front";
        case AnimationTrigger::RTS_OnCharger_Awake_30Left: return "RTS_OnCharger_Awake_30Left";
        case AnimationTrigger::RTS_OnCharger_Awake_30Right: return "RTS_OnCharger_Awake_30Right";
        case AnimationTrigger::RTS_OnCharger_Awake_60Left: return "RTS_OnCharger_Awake_60Left";
        case AnimationTrigger::RTS_OnCharger_Awake_60Right: return "RTS_OnCharger_Awake_60Right";
        case AnimationTrigger::RTS_OnCharger_Awake_Left: return "RTS_OnCharger_Awake_Left";
        case AnimationTrigger::RTS_OnCharger_Awake_Right: return "RTS_OnCharger_Awake_Right";
        case AnimationTrigger::RTS_OnCharger_Awake_120Left: return "RTS_OnCharger_Awake_120Left";
        case AnimationTrigger::RTS_OnCharger_Awake_120Right: return "RTS_OnCharger_Awake_120Right";
        case AnimationTrigger::RTS_OnCharger_Awake_150Left: return "RTS_OnCharger_Awake_150Left";
        case AnimationTrigger::RTS_OnCharger_Awake_150Right: return "RTS_OnCharger_Awake_150Right";
        case AnimationTrigger::RTS_OnCharger_Awake_Back: return "RTS_OnCharger_Awake_Back";
        case AnimationTrigger::ReactToTouchInitial: return "ReactToTouchInitial";
        case AnimationTrigger::ReactToTriggerWordOffChargerFrontLeft: return "ReactToTriggerWordOffChargerFrontLeft";
        case AnimationTrigger::ReactToTriggerWordOffChargerFrontRight: return "ReactToTriggerWordOffChargerFrontRight";
        case AnimationTrigger::ReactToTriggerWordOffChargerLeft: return "ReactToTriggerWordOffChargerLeft";
        case AnimationTrigger::ReactToTriggerWordOffChargerRight: return "ReactToTriggerWordOffChargerRight";
        case AnimationTrigger::ReactToTriggerWordOffChargerBehind: return "ReactToTriggerWordOffChargerBehind";
        case AnimationTrigger::ReactToTriggerWordOffChargerBehindLeft: return "ReactToTriggerWordOffChargerBehindLeft";
        case AnimationTrigger::ReactToTriggerWordOffChargerBehindRight: return "ReactToTriggerWordOffChargerBehindRight";
        case AnimationTrigger::ReactToUnclaimedIntent: return "ReactToUnclaimedIntent";
        case AnimationTrigger::ReactToUnclaimedIntentInAir: return "ReactToUnclaimedIntentInAir";
        case AnimationTrigger::ReactToUnexpectedMovement: return "ReactToUnexpectedMovement";
        case AnimationTrigger::RollBlockRealign: return "RollBlockRealign";
        case AnimationTrigger::RollBlockRetry: return "RollBlockRetry";
        case AnimationTrigger::RollBlockSuccess: return "RollBlockSuccess";
        case AnimationTrigger::SoundOnlyLiftEffortPickup: return "SoundOnlyLiftEffortPickup";
        case AnimationTrigger::SoundOnlyLiftEffortPlaceHigh: return "SoundOnlyLiftEffortPlaceHigh";
        case AnimationTrigger::SoundOnlyLiftEffortPlaceLow: return "SoundOnlyLiftEffortPlaceLow";
        case AnimationTrigger::SoundOnlyLiftEffortPlaceRoll: return "SoundOnlyLiftEffortPlaceRoll";
        case AnimationTrigger::StuckOnEdgeGetIn: return "StuckOnEdgeGetIn";
        case AnimationTrigger::StuckOnEdgeIdle: return "StuckOnEdgeIdle";
        case AnimationTrigger::StuckOnEdgeLeftGetIn: return "StuckOnEdgeLeftGetIn";
        case AnimationTrigger::StuckOnEdgeLeftIdle: return "StuckOnEdgeLeftIdle";
        case AnimationTrigger::StuckOnEdgeRightGetIn: return "StuckOnEdgeRightGetIn";
        case AnimationTrigger::StuckOnEdgeRightIdle: return "StuckOnEdgeRightIdle";
        case AnimationTrigger::SuccessfulWheelie: return "SuccessfulWheelie";
        case AnimationTrigger::TimerCancelGetIn: return "TimerCancelGetIn";
        case AnimationTrigger::TimerCancelTimer: return "TimerCancelTimer";
        case AnimationTrigger::TimerCheckTimeGetIn: return "TimerCheckTimeGetIn";
        case AnimationTrigger::TimerCheckTimeGetOut: return "TimerCheckTimeGetOut";
        case AnimationTrigger::TimerSetGetIn: return "TimerSetGetIn";
        case AnimationTrigger::TimerSetGetOut: return "TimerSetGetOut";
        case AnimationTrigger::TimerRingGetIn: return "TimerRingGetIn";
        case AnimationTrigger::TimerRing: return "TimerRing";
        case AnimationTrigger::TimerRingGetOut: return "TimerRingGetOut";
        case AnimationTrigger::TakeAPictureCapture: return "TakeAPictureCapture";
        case AnimationTrigger::TakeAPictureFocusing: return "TakeAPictureFocusing";
        case AnimationTrigger::TextToSpeechGetIn: return "TextToSpeechGetIn";
        case AnimationTrigger::TextToSpeechGetLoop: return "TextToSpeechGetLoop";
        case AnimationTrigger::TextToSpeechGetOut: return "TextToSpeechGetOut";
        case AnimationTrigger::UnintentionalPerformance: return "UnintentionalPerformance";
        case AnimationTrigger::UnitTestAnim: return "UnitTestAnim";
        case AnimationTrigger::VC_ListeningGetIn: return "VC_ListeningGetIn";
        case AnimationTrigger::VC_ListeningLoop: return "VC_ListeningLoop";
        case AnimationTrigger::VC_ListeningGetOut: return "VC_ListeningGetOut";
        case AnimationTrigger::VC_SleepingToListeningGetIn: return "VC_SleepingToListeningGetIn";
        case AnimationTrigger::VC_SleepingToListeningLoop: return "VC_SleepingToListeningLoop";
        case AnimationTrigger::VC_SleepingToListeningGetOut: return "VC_SleepingToListeningGetOut";
        case AnimationTrigger::VC_IntentNeutral: return "VC_IntentNeutral";
        case AnimationTrigger::VolumeLevel1: return "VolumeLevel1";
        case AnimationTrigger::VolumeLevel2: return "VolumeLevel2";
        case AnimationTrigger::VolumeLevel3: return "VolumeLevel3";
        case AnimationTrigger::VolumeLevel4: return "VolumeLevel4";
        case AnimationTrigger::VolumeLevel5: return "VolumeLevel5";
        case AnimationTrigger::WakeupGetout: return "WakeupGetout";
        case AnimationTrigger::GreetAfterLongTime: return "GreetAfterLongTime";
        case AnimationTrigger::DEPRECATED_AcknowledgeFaceNamed: return "DEPRECATED_AcknowledgeFaceNamed";
        case AnimationTrigger::DEPRECATED_AcknowledgeFaceUnnamed: return "DEPRECATED_AcknowledgeFaceUnnamed";
        case AnimationTrigger::DEPRECATED_AcknowledgeObject: return "DEPRECATED_AcknowledgeObject";
        case AnimationTrigger::DEPRECATED_ComeHere_SearchForFace: return "DEPRECATED_ComeHere_SearchForFace";
        case AnimationTrigger::DEPRECATED_CubeMovedSense: return "DEPRECATED_CubeMovedSense";
        case AnimationTrigger::DEPRECATED_CubeMovedUpset: return "DEPRECATED_CubeMovedUpset";
        case AnimationTrigger::DEPRECATED_DizzyReactionHard: return "DEPRECATED_DizzyReactionHard";
        case AnimationTrigger::DEPRECATED_DizzyReactionMedium: return "DEPRECATED_DizzyReactionMedium";
        case AnimationTrigger::DEPRECATED_DizzyReactionSoft: return "DEPRECATED_DizzyReactionSoft";
        case AnimationTrigger::DEPRECATED_DizzyShakeLoop: return "DEPRECATED_DizzyShakeLoop";
        case AnimationTrigger::DEPRECATED_DizzyShakeStop: return "DEPRECATED_DizzyShakeStop";
        case AnimationTrigger::DEPRECATED_DizzyStillPickedUp: return "DEPRECATED_DizzyStillPickedUp";
        case AnimationTrigger::DEPRECATED_LaserAcknowledge: return "DEPRECATED_LaserAcknowledge";
        case AnimationTrigger::DEPRECATED_LaserDriveEnd: return "DEPRECATED_LaserDriveEnd";
        case AnimationTrigger::DEPRECATED_LaserDriveLoop: return "DEPRECATED_LaserDriveLoop";
        case AnimationTrigger::DEPRECATED_LaserDriveStart: return "DEPRECATED_LaserDriveStart";
        case AnimationTrigger::DEPRECATED_LaserGetOut: return "DEPRECATED_LaserGetOut";
        case AnimationTrigger::DEPRECATED_LaserPounce: return "DEPRECATED_LaserPounce";
        case AnimationTrigger::DEPRECATED_LookDownForLaser: return "DEPRECATED_LookDownForLaser";
        case AnimationTrigger::DEPRECATED_NamedFaceInitialGreeting: return "DEPRECATED_NamedFaceInitialGreeting";
        case AnimationTrigger::DEPRECATED_SearchForFace_Search: return "DEPRECATED_SearchForFace_Search";
        case AnimationTrigger::DEPRECATED_SearchForFace_FoundFace: return "DEPRECATED_SearchForFace_FoundFace";
        case AnimationTrigger::DEPRECATED_StackBlocksSuccess: return "DEPRECATED_StackBlocksSuccess";
        case AnimationTrigger::Count: return "Count";
        default: return "";
    }
}

} // namespace vector_eyes
} // namespace esphome
