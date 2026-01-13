#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "driver/i2s.h"
#include <cmath>
#include <cstring>
#include <vector>

namespace esphome {
namespace ptttl {

#define MAX_CHANNELS 16
#define MAX_NAME_LEN 256

struct VibratoSettings {
  uint16_t frequency;  // Hz
  uint16_t variance;   // Hz
};

struct Note {
  uint8_t note_value;      // 0 = pause, 1-88 = piano keys
  uint16_t duration_ms;    // Duration in milliseconds
  VibratoSettings vibrato;
};

struct Channel {
  const char *data_start;  // Pointer to start of channel data
  uint16_t position;       // Current position in channel data
  float samples_played;    // Samples played for current note (float for accuracy)
  float note_duration_samples; // Duration of current note in samples (float)
  Note current_note;       // Current note being played
  uint16_t phase;          // Phase accumulator for waveform
  float vibrato_phase;     // Phase for vibrato LFO
  bool active;             // Is this channel active
  bool finished;           // Has this channel finished playing
};

class PTTTLComponent : public Component {
 public:
  void set_bclk_pin(InternalGPIOPin *pin) { bclk_pin_ = pin; }
  void set_lrclk_pin(InternalGPIOPin *pin) { lrclk_pin_ = pin; }
  void set_dout_pin(InternalGPIOPin *pin) { dout_pin_ = pin; }
  
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::LATE; }
  
  void play(const char *ptttl);
  void stop();
  bool is_playing() { return playing_; }
  
  // Configuration
  void set_sample_rate(uint32_t rate) { sample_rate_ = rate; }
  void set_volume(float volume) { volume_ = std::max(0.0f, std::min(1.0f, volume)); }
  
 protected:
  InternalGPIOPin *bclk_pin_;
  InternalGPIOPin *lrclk_pin_;
  InternalGPIOPin *dout_pin_;
  
  bool playing_{false};
  const char *ptttl_string_{nullptr};
  char name_[MAX_NAME_LEN];
  
  // Channels
  Channel channels_[MAX_CHANNELS];
  uint8_t num_channels_{0};
  
  // Default values from settings section
  uint8_t default_duration_{4};
  uint8_t default_octave_{6};
  uint16_t bpm_{63};
  uint16_t default_vibrato_freq_{7};
  uint16_t default_vibrato_var_{10};
  
  // Audio settings
  uint32_t sample_rate_{16000};
  float volume_{0.8f};
  
  // Audio task
  TaskHandle_t audio_task_handle_{nullptr};
  static void audio_task(void *param);
  
  // Parsing helpers
  bool parse_ptttl_header();
  bool parse_settings_section(const char *&pos);
  bool find_channel_starts(const char *data_section);
  bool parse_note(const char *&pos, Note &note);
  float note_to_frequency(uint8_t note_value);
  
  // Playback
  void play_next_note(uint8_t channel_idx);
  void generate_audio();
  int16_t generate_sample(uint8_t channel_idx);
  
  // Utility
  void skip_whitespace_and_comments(const char *&pos);
  bool parse_uint(const char *&pos, unsigned int &value);
  uint8_t note_name_to_value(char note, bool sharp, bool flat, uint8_t octave);
};

}  // namespace ptttl
}  // namespace esphome
