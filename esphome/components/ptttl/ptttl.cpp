#include "ptttl.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace ptttl {

static const char *TAG = "ptttl";

// Piano key frequencies (A0 = 27.5 Hz is key 1, C8 = 4186 Hz is key 88)
static const float PIANO_KEY_FREQUENCIES[88] = {
  27.50, 29.14, 30.87, 32.70, 34.65, 36.71, 38.89, 41.20, 43.65, 46.25, 49.00, 51.91,  // Octave 0
  55.00, 58.27, 61.74, 65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.83, // Octave 1
  110.00, 116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 196.00, 207.65, // Octave 2
  220.00, 233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, // Octave 3
  466.16, 493.88, 523.25, 554.37, 587.33, 622.25, 659.25, 698.46, 739.99, 783.99, 830.61, 880.00, // Octave 4
  932.33, 987.77, 1046.50, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760.00, // Octave 5
  1864.66, 1975.53, 2093.00, 2217.46, 2349.32, 2489.02, 2637.02, 2793.83, 2959.96, 3135.96, 3322.44, 3520.00, // Octave 6
  3729.31, 3951.07, 4186.01 // Octave 7 (partial)
};

// Note names to piano key offsets within an octave
static const uint8_t NOTE_OFFSETS[12] = {
  0,  // C
  1,  // C# / Db
  2,  // D
  3,  // D# / Eb
  4,  // E
  5,  // F
  6,  // F# / Gb
  7,  // G
  8,  // G# / Ab
  9,  // A
  10, // A# / Bb
  11  // B
};

// Octave starting positions (piano key numbers, 0-based)
static const uint8_t OCTAVE_STARTS[9] = {0, 3, 15, 27, 39, 51, 63, 75, 87};

void PTTTLComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PTTTL...");
  
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = sample_rate_,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = this->bclk_pin_->get_pin(),
    .ws_io_num = this->lrclk_pin_->get_pin(),
    .data_out_num = this->dout_pin_->get_pin(),
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install I2S driver: %d", err);
    this->mark_failed();
    return;
  }
  
  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set I2S pins: %d", err);
    this->mark_failed();
    return;
  }
  
  i2s_zero_dma_buffer(I2S_NUM_0);
  
  // Create audio task on core 1
  xTaskCreatePinnedToCore(
    audio_task,
    "PTTTLAudio",
    4096,
    this,
    2,
    &this->audio_task_handle_,
    1
  );
  
  ESP_LOGCONFIG(TAG, "PTTTL initialized - Sample rate: %d Hz, Volume: %.2f", sample_rate_, volume_);
}

void PTTTLComponent::loop() {
  // Note timing is now handled in the audio generation task
}

void PTTTLComponent::play(const char *ptttl) {
  if (!ptttl || strlen(ptttl) == 0) {
    ESP_LOGE(TAG, "Invalid PTTTL string");
    return;
  }
  
  ESP_LOGI(TAG, "Playing PTTTL");
  
  // Reset state
  ptttl_string_ = ptttl;
  playing_ = false;
  num_channels_ = 0;
  
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    channels_[i].active = false;
    channels_[i].finished = false;
    channels_[i].phase = 0;
    channels_[i].vibrato_phase = 0.0f;
  }
  
  // Parse the PTTTL string
  if (!parse_ptttl_header()) {
    ESP_LOGE(TAG, "Failed to parse PTTTL header");
    return;
  }
  
  ESP_LOGI(TAG, "Parsed '%s': %d channels, d=%d, o=%d, b=%d, f=%d, v=%d", 
           name_, num_channels_, default_duration_, default_octave_, bpm_, 
           default_vibrato_freq_, default_vibrato_var_);
  
  // Start playing all channels
  playing_ = true;
  for (uint8_t i = 0; i < num_channels_; i++) {
    channels_[i].active = true;
    channels_[i].samples_played = 0.0f;
    play_next_note(i);
  }
}

void PTTTLComponent::stop() {
  ESP_LOGD(TAG, "Stopping playback");
  playing_ = false;
  
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    channels_[i].active = false;
    channels_[i].finished = true;
  }
}

bool PTTTLComponent::parse_ptttl_header() {
  const char *pos = ptttl_string_;
  
  // Parse name (first field before ':')
  uint16_t name_len = 0;
  while (*pos && *pos != ':' && name_len < MAX_NAME_LEN - 1) {
    name_[name_len++] = *pos++;
  }
  name_[name_len] = '\0';
  
  if (*pos != ':') {
    ESP_LOGE(TAG, "Missing ':' after name");
    return false;
  }
  pos++; // Skip ':'
  
  // Parse settings section
  if (!parse_settings_section(pos)) {
    return false;
  }
  
  if (*pos != ':') {
    ESP_LOGE(TAG, "Missing ':' after settings");
    return false;
  }
  pos++; // Skip ':'
  
  // Find channel starts in data section
  return find_channel_starts(pos);
}

bool PTTTLComponent::parse_settings_section(const char *&pos) {
  // Set defaults
  default_duration_ = 4;
  default_octave_ = 6;
  bpm_ = 63;
  default_vibrato_freq_ = 7;
  default_vibrato_var_ = 10;
  
  while (*pos && *pos != ':') {
    // Skip whitespace
    while (*pos == ' ' || *pos == '\t') pos++;
    
    if (*pos == ':') break;
    
    // Parse setting key
    char key = *pos++;
    
    // Expect '='
    while (*pos == ' ' || *pos == '\t') pos++;
    if (*pos != '=') {
      ESP_LOGE(TAG, "Expected '=' after setting key '%c'", key);
      return false;
    }
    pos++; // Skip '='
    
    // Parse value
    unsigned int value;
    if (!parse_uint(pos, value)) {
      ESP_LOGE(TAG, "Failed to parse value for setting '%c'", key);
      return false;
    }
    
    // Apply setting
    switch (key) {
      case 'd':
        default_duration_ = value;
        break;
      case 'o':
        if (value > 8) {
          ESP_LOGE(TAG, "Invalid octave %d (must be 0-8)", value);
          return false;
        }
        default_octave_ = value;
        break;
      case 'b':
        if (value == 0) {
          ESP_LOGE(TAG, "BPM cannot be zero");
          return false;
        }
        bpm_ = value;
        break;
      case 'f':
        default_vibrato_freq_ = value;
        break;
      case 'v':
        default_vibrato_var_ = value;
        break;
      default:
        ESP_LOGW(TAG, "Unknown setting key '%c'", key);
        break;
    }
    
    // Skip comma
    while (*pos == ' ' || *pos == '\t') pos++;
    if (*pos == ',') pos++;
  }
  
  return true;
}

bool PTTTLComponent::find_channel_starts(const char *data_section) {
  const char *pos = data_section;
  skip_whitespace_and_comments(pos);
  
  // Find all channel starts (separated by '|')
  num_channels_ = 0;
  channels_[num_channels_].data_start = pos;
  channels_[num_channels_].position = 0;
  num_channels_++;
  
  // Scan through first block to find channel separators
  bool in_first_block = true;
  while (*pos && *pos != '\0' && in_first_block) {
    if (*pos == '|') {
      if (num_channels_ >= MAX_CHANNELS) {
        ESP_LOGE(TAG, "Too many channels (max %d)", MAX_CHANNELS);
        return false;
      }
      
      pos++; // Skip '|'
      skip_whitespace_and_comments(pos);
      
      channels_[num_channels_].data_start = pos;
      channels_[num_channels_].position = 0;
      num_channels_++;
    } else if (*pos == ';') {
      // End of first block - channels are defined
      in_first_block = false;
    } else if (*pos == '#') {
      // Skip comment
      while (*pos && *pos != '\n') pos++;
    } else {
      pos++;
    }
  }
  
  return num_channels_ > 0;
}

void PTTTLComponent::play_next_note(uint8_t channel_idx) {
  if (channel_idx >= num_channels_) return;
  
  Channel &ch = channels_[channel_idx];
  const char *pos = ch.data_start + ch.position;
  
  skip_whitespace_and_comments(pos);
  
  // Check for end of channel or block separator
  if (!*pos) {
    ch.active = false;
    ch.finished = true;
    ESP_LOGD(TAG, "Channel %d finished (EOF)", channel_idx);
    return;
  }
  
  if (*pos == '|' || *pos == ';') {
    // Find start of next block
    while (*pos && *pos != ';') pos++;
    if (!*pos) {
      ch.active = false;
      ch.finished = true;
      ESP_LOGD(TAG, "Channel %d finished (EOF)", channel_idx);
      return;
    }
    pos++; // Skip ';'
    skip_whitespace_and_comments(pos);
    
    // Skip to this channel's position in the new block
    for (uint8_t i = 0; i < channel_idx; i++) {
      while (*pos && *pos != '|') pos++;
      if (!*pos) {
        ch.active = false;
        ch.finished = true;
        ESP_LOGD(TAG, "Channel %d finished (no more blocks)", channel_idx);
        return;
      }
      pos++; // Skip '|'
      skip_whitespace_and_comments(pos);
    }
    
    if (!*pos) {
      ch.active = false;
      ch.finished = true;
      ESP_LOGD(TAG, "Channel %d finished (end of blocks)", channel_idx);
      return;
    }
  }
  
  // Parse the note
  if (!parse_note(pos, ch.current_note)) {
    ESP_LOGE(TAG, "Failed to parse note on channel %d at position %d", channel_idx, ch.position);
    ch.active = false;
    ch.finished = true;
    return;
  }
  
  // Update position
  ch.position = pos - ch.data_start;
  ch.samples_played = 0.0f;
  ch.note_duration_samples = (ch.current_note.duration_ms * (float)sample_rate_) / 1000.0f;
  ch.phase = 0;
  ch.vibrato_phase = 0.0f;
  
  ESP_LOGV(TAG, "Ch%d: note=%d, dur=%dms (%.1f samples), vib=%d/%d", 
           channel_idx, ch.current_note.note_value, ch.current_note.duration_ms,
           ch.note_duration_samples, ch.current_note.vibrato.frequency, ch.current_note.vibrato.variance);
}

bool PTTTLComponent::parse_note(const char *&pos, Note &note) {
  // Initialize with defaults
  uint8_t duration = default_duration_;
  uint8_t octave = default_octave_;
  bool dotted = false;
  bool has_vibrato = false;
  uint16_t vibrato_freq = default_vibrato_freq_;
  uint16_t vibrato_var = default_vibrato_var_;
  
  // Parse duration (optional)
  if (*pos >= '0' && *pos <= '9') {
    unsigned int dur;
    if (!parse_uint(pos, dur)) return false;
    duration = dur;
  }
  
  // Parse note name
  char note_char = *pos++;
  if (note_char >= 'a' && note_char <= 'z') {
    note_char = note_char - 32; // Convert to uppercase
  }
  
  // Check for sharp or flat
  bool sharp = false;
  bool flat = false;
  if (*pos == '#') {
    sharp = true;
    pos++;
  } else if (*pos == 'b' && (note_char != 'P')) {
    flat = true;
    pos++;
  }
  
  // Check for dot (before or after octave)
  if (*pos == '.') {
    dotted = true;
    pos++;
  }
  
  // Parse octave (optional)
  if (*pos >= '0' && *pos <= '9') {
    octave = *pos - '0';
    pos++;
  }
  
  // Check for dot again (after octave)
  if (*pos == '.') {
    dotted = true;
    pos++;
  }
  
  // Check for vibrato
  if (*pos == 'v') {
    has_vibrato = true;
    pos++;
    
    // Parse vibrato frequency (optional)
    if (*pos >= '0' && *pos <= '9') {
      unsigned int freq;
      if (!parse_uint(pos, freq)) return false;
      vibrato_freq = freq;
      
      // Parse vibrato variance (optional, after '-')
      if (*pos == '-') {
        pos++;
        unsigned int var;
        if (!parse_uint(pos, var)) return false;
        vibrato_var = var;
      }
    }
  }
  
  // Convert note to value
  if (note_char == 'P' || note_char == 'p') {
    note.note_value = 0; // Pause
  } else {
    note.note_value = note_name_to_value(note_char, sharp, flat, octave);
    if (note.note_value == 0) {
      ESP_LOGE(TAG, "Invalid note: %c", note_char);
      return false;
    }
  }
  
  // Calculate duration in milliseconds
  float whole_note_duration = (60.0f / bpm_) * 4.0f * 1000.0f;
  note.duration_ms = whole_note_duration / duration;
  if (dotted) {
    note.duration_ms = note.duration_ms * 1.5f;
  }
  
  // Set vibrato
  if (has_vibrato) {
    note.vibrato.frequency = vibrato_freq;
    note.vibrato.variance = vibrato_var;
  } else {
    note.vibrato.frequency = 0;
    note.vibrato.variance = 0;
  }
  
  // Skip comma if present
  skip_whitespace_and_comments(pos);
  if (*pos == ',') pos++;
  
  return true;
}

uint8_t PTTTLComponent::note_name_to_value(char note, bool sharp, bool flat, uint8_t octave) {
  if (octave > 8) return 0;
  
  uint8_t note_offset;
  switch (note) {
    case 'C': note_offset = sharp ? 1 : (flat ? 11 : 0); break;  // Cb = B
    case 'D': note_offset = sharp ? 3 : (flat ? 1 : 2); break;   // Db = C#
    case 'E': note_offset = sharp ? 5 : (flat ? 3 : 4); break;   // Eb = D#
    case 'F': note_offset = sharp ? 6 : (flat ? 4 : 5); break;   // Fb = E
    case 'G': note_offset = sharp ? 8 : (flat ? 6 : 7); break;   // Gb = F#
    case 'A': note_offset = sharp ? 10 : (flat ? 8 : 9); break;  // Ab = G#
    case 'B': note_offset = sharp ? 0 : (flat ? 10 : 11); break; // Bb = A#, B# = C
    default: return 0;
  }
  
  // Handle Cb wrapping to previous octave
  if (note == 'C' && flat) {
    if (octave == 0) return 0;  // Cb0 doesn't exist
    octave--;
  }
  
  // Handle B# wrapping to next octave
  if (note == 'B' && sharp) {
    octave++;
    if (octave > 8) return 0;
  }
  
  // Calculate piano key number (1-88)
  if (octave == 0) {
    // Octave 0 only has A, A#, B
    if (note_offset < 9) return 0;
    return note_offset - 9 + 1;
  }
  
  uint8_t key = OCTAVE_STARTS[octave] + note_offset + 1;
  if (key > 88) return 0;
  
  return key;
}

float PTTTLComponent::note_to_frequency(uint8_t note_value) {
  if (note_value == 0 || note_value > 88) return 0.0f;
  return PIANO_KEY_FREQUENCIES[note_value - 1];
}

int16_t PTTTLComponent::generate_sample(uint8_t channel_idx) {
  Channel &ch = channels_[channel_idx];
  
  // If channel finished, output silence
  if (!ch.active || ch.finished || ch.current_note.note_value == 0) {
    return 0;
  }
  
  // Get base frequency
  float freq = note_to_frequency(ch.current_note.note_value);
  
  // Apply vibrato if enabled
  if (ch.current_note.vibrato.frequency > 0 && ch.current_note.vibrato.variance > 0) {
    float vibrato_lfo = sinf(ch.vibrato_phase);
    freq += vibrato_lfo * ch.current_note.vibrato.variance;
    
    // Update vibrato phase
    float vibrato_phase_delta = (2.0f * M_PI * ch.current_note.vibrato.frequency) / sample_rate_;
    ch.vibrato_phase += vibrato_phase_delta;
    if (ch.vibrato_phase >= 2.0f * M_PI) {
      ch.vibrato_phase -= 2.0f * M_PI;
    }
  }
  
  // Generate sine wave for smoother sound
  float phase_float = (float)ch.phase / 65536.0f;  // Convert to 0.0-1.0
  float sine_value = sinf(phase_float * 2.0f * M_PI);
  int16_t raw_sample = sine_value * 8000.0f;
  
  // Update phase
  uint16_t phase_increment = (freq * 65536.0f) / sample_rate_;
  ch.phase += phase_increment;
  
  // Apply attack/decay envelope (matching original ptttl defaults)
  float samples_elapsed = ch.samples_played;
  float samples_remaining = ch.note_duration_samples - samples_elapsed;
  float attack_samples = 100.0f;  // Original default: 100 samples (~2.3ms at 44.1kHz)
  float decay_samples = 500.0f;   // Original default: 500 samples (~11.3ms at 44.1kHz)
  
  float envelope = 1.0f;
  if (samples_elapsed < attack_samples) {
    // Attack phase - ramp up from 0 to 1
    envelope = samples_elapsed / attack_samples;
  } else if (samples_remaining < decay_samples) {
    // Decay phase - ramp down from 1 to 0
    envelope = samples_remaining / decay_samples;
  }
  
  return raw_sample * envelope;
}

void PTTTLComponent::generate_audio() {
  const int samples = 64;
  int16_t buffer[samples * 2];
  
  for (int i = 0; i < samples; i++) {
    // First, check if any channels need to advance to next note
    for (uint8_t ch = 0; ch < num_channels_; ch++) {
      if (playing_ && !channels_[ch].finished && channels_[ch].active) {
        if (channels_[ch].samples_played >= channels_[ch].note_duration_samples) {
          play_next_note(ch);
        }
      }
    }
    
    // Then generate and mix samples from all channels
    int32_t mixed = 0;
    for (uint8_t ch = 0; ch < num_channels_; ch++) {
      if (playing_) {
        mixed += generate_sample(ch);
        
        // Increment sample counter for active channels
        if (!channels_[ch].finished && channels_[ch].active) {
          channels_[ch].samples_played += 1.0f;
        }
      }
    }
    
    // Check if all channels finished
    if (playing_) {
      bool any_active = false;
      for (uint8_t ch = 0; ch < num_channels_; ch++) {
        if (!channels_[ch].finished) {
          any_active = true;
          break;
        }
      }
      if (!any_active) {
        ESP_LOGI(TAG, "All channels finished - stopping playback");
        playing_ = false;
      }
    }
    
    // Average by total channel count and apply volume
    int16_t sample = (mixed / num_channels_) * volume_;
    
    buffer[i * 2] = sample;
    buffer[i * 2 + 1] = sample;
  }
  
  size_t bytes_written;
  i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
}

void PTTTLComponent::audio_task(void *param) {
  PTTTLComponent *comp = (PTTTLComponent *)param;
  
  while (true) {
    comp->generate_audio();
  }
}

void PTTTLComponent::skip_whitespace_and_comments(const char *&pos) {
  while (*pos) {
    if (*pos == ' ' || *pos == '\t' || *pos == '\n' || *pos == '\r') {
      pos++;
    } else if (*pos == '#') {
      // Skip comment line
      while (*pos && *pos != '\n') pos++;
      if (*pos == '\n') pos++;
    } else {
      break;
    }
  }
}

bool PTTTLComponent::parse_uint(const char *&pos, unsigned int &value) {
  if (*pos < '0' || *pos > '9') return false;
  
  value = 0;
  while (*pos >= '0' && *pos <= '9') {
    value = value * 10 + (*pos - '0');
    pos++;
  }
  
  return true;
}

}  // namespace ptttl
}  // namespace esphome
