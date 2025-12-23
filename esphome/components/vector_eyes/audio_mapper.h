// Audio event mapping from Wwise event names to embedded WAV files
#pragma once
#include <string>

namespace esphome {
namespace vector_eyes {

// Map Wwise audio event names to WAV filenames
inline const char* map_audio_event(const std::string& event_name) {
  // Screen sounds (most common)
  if (event_name == "Play__Robot_Vic_Sfx__Blink") return "blink.wav";
  if (event_name == "Play__Robot_Vic_Sfx__Scrn_Curious") return "curious.wav";
  if (event_name == "Play__Robot_Vic_Sfx__Scrn_Curious_Short") return "curious.wav";  // Reuse
  if (event_name == "Play__Robot_Vic_Sfx__Scrn_Curious_Long") return "curious.wav";
  
  if (event_name == "Play__Robot_Vic_Sfx__Scrn_Happy") return "scrn_happy_01.wav";
  if (event_name == "Play__Robot_Vic_Sfx__Scrn_Happy_Short") return "happy_short.wav";
  if (event_name == "Play__Robot_Vic_Sfx__Scrn_Happy_Long") return "happy.wav";
  
  if (event_name == "Play__Robot_Vic_Sfx__Scrn_Neutral") return "neutral.wav";
  if (event_name == "Play__Robot_Vic_Sfx__Scrn_Neutral_Short") return "neutral_short.wav";
  if (event_name == "Play__Robot_Vic_Sfx__Scrn_Neutral_Long") return "neutral.wav";  // Reuse
  
  // TODO: Add more mappings for Scrn_Angry, Scrn_Sad, Scrn_Surprised
  // TODO: Add Head and Lift movement sounds when available
  
  // Unmapped events
  return nullptr;
}

}  // namespace vector_eyes
}  // namespace esphome
