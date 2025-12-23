# AudioMapper Implementation

## Overview

The AudioMapper system provides a robust, configurable solution for mapping Vector's Wwise audio event names to WAV file identifiers. It supports exact matching, pattern-based matching with specificity precedence, and hot-reload of configuration files.

## Features

### 1. Configuration-Based Mapping
- External JSON configuration file (`audio_mappings.json`)
- Separates mapping logic from code
- Easy to update without code changes

### 2. Dual Matching Strategy
- **Exact Match**: Direct event name to WAV file mapping
- **Pattern Match**: Substring-based matching for variants
- Exact matches are tried first for performance

### 3. Specificity Precedence
- When multiple patterns match, the most specific (longest) pattern wins
- Example: `Scrn_Happy_Short` takes precedence over `Scrn_Happy`
- Ensures correct variant selection

### 4. Hot-Reload Support
- Reload mappings without restarting the application
- Graceful error handling for syntax errors
- Preserves existing mappings if reload fails

### 5. Comprehensive Logging
- INFO level: Successful operations and statistics
- WARNING level: Unmapped events
- ERROR level: Configuration errors
- DEBUG level: Detailed matching information

## Configuration File Format

```json
{
  "exact_matches": {
    "Play__Robot_Vic_Sfx__Blink": "blink",
    "Play__Robot_Vic_Sfx__Wake_Word_On": "zelda"
  },
  "pattern_matches": {
    "Scrn_Happy_Short": "happy_short",
    "Scrn_Happy": "happy",
    "Emote_Curious": "emote_curious"
  }
}
```

### Exact Matches
Use for events that should map to a specific WAV file with no ambiguity.

### Pattern Matches
Use for families of related events (e.g., all "Happy" variants). The system automatically handles specificity by sorting patterns by length.

## Usage Examples

### Basic Usage

```python
from audio_mapper import AudioMapper

# Create mapper with configuration file
mapper = AudioMapper("audio_mappings.json")

# Map an audio event
wav_file = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Blink")
print(f"Mapped to: {wav_file}.wav")  # Output: Mapped to: blink.wav
```

### Integration with Animation Conversion

```python
from audio_mapper import AudioMapper
from animation_data import AudioEventData

# Initialize mapper
mapper = AudioMapper("audio_mappings.json")

# Process audio events from animation
for audio_event in animation.audio_events:
    event_name = audio_event.event_names[0]
    wav_file = mapper.map_event_to_wav(event_name)
    
    if wav_file:
        audio_event.wav_file = wav_file
        print(f"Mapped: {event_name} -> {wav_file}.wav")
    else:
        print(f"Warning: No mapping for {event_name}")
```

### Hot-Reload

```python
# Initial load
mapper = AudioMapper("audio_mappings.json")

# ... user edits audio_mappings.json ...

# Reload mappings
if mapper.reload_mappings():
    print("Mappings reloaded successfully")
else:
    print("Failed to reload (old mappings preserved)")
```

### Programmatic Mapping

```python
# Create mapper without config file
mapper = AudioMapper()

# Add mappings programmatically
mapper.add_exact_mapping("Play__Robot_Vic_Sfx__Blink", "blink")
mapper.add_pattern_mapping("Scrn_Happy", "happy")

# Use as normal
wav_file = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Blink")
```

## Implementation Details

### Specificity Algorithm

Pattern mappings are sorted by length (descending) when loaded:
1. Longer patterns are checked first
2. First match wins
3. This ensures most specific pattern takes precedence

Example:
```
Patterns: ["Scrn_Happy_Short", "Scrn_Happy", "Happy"]
Event: "Play__Robot_Vic_Sfx__Scrn_Happy_Short"
Result: Matches "Scrn_Happy_Short" (longest match)
```

### Error Handling

1. **Missing Configuration File**: Logs error, returns False on reload
2. **JSON Syntax Error**: Logs error with details, preserves old mappings
3. **Unmapped Event**: Logs warning, returns None
4. **Empty Event Name**: Returns None silently

### Performance Considerations

- Exact matches use dictionary lookup: O(1)
- Pattern matches use linear search: O(n) where n = number of patterns
- Patterns are pre-sorted, so first match is always most specific
- Typical performance: <1ms per lookup

## Testing

Comprehensive test suite included:
- `test_audio_mapper.py`: Core functionality tests
- `test_audio_mapper_reload.py`: Reload and error handling tests
- `example_audio_mapper_usage.py`: Usage examples

Run tests:
```bash
python test_audio_mapper.py
python test_audio_mapper_reload.py
python example_audio_mapper_usage.py
```

## Requirements Validation

This implementation satisfies the following requirements:

### Requirement 2.1
✓ Audio event names are looked up in the mapping table

### Requirement 2.2
✓ Unmapped events log warnings and return None

### Requirement 2.5
✓ Variant suffixes match most specific mapping available

### Requirement 8.1
✓ Mappings are read from external configuration file

### Requirement 8.2
✓ Mapping file can be reloaded without code changes

### Requirement 8.3
✓ Pattern matching is supported for audio event names

### Requirement 8.4
✓ Most specific pattern takes precedence when multiple match

### Requirement 8.5
✓ Syntax errors are reported and handled gracefully

## Future Enhancements

Possible improvements for future versions:
1. Regular expression support for more complex patterns
2. Wildcard matching (e.g., `*_Happy_*`)
3. Mapping validation on load (check WAV files exist)
4. Mapping statistics and coverage reports
5. Multiple configuration file support (layered configs)
6. Caching for frequently accessed mappings

## Migration from Hardcoded Mappings

To migrate from the hardcoded `AUDIO_EVENT_MAP` in `convert_anims.py`:

1. Extract all mappings to `audio_mappings.json`
2. Replace hardcoded dictionary with AudioMapper instance
3. Update conversion logic to use `mapper.map_event_to_wav()`
4. Remove hardcoded dictionary from code

Example migration:
```python
# OLD CODE
AUDIO_EVENT_MAP = {
    "Play__Robot_Vic_Sfx__Blink": "blink",
    # ... 50+ more entries ...
}
mapped_sound = AUDIO_EVENT_MAP.get(event_name)

# NEW CODE
mapper = AudioMapper("audio_mappings.json")
mapped_sound = mapper.map_event_to_wav(event_name)
```

## Files

- `audio_mapper.py`: Main AudioMapper class implementation
- `audio_mappings.json`: Default configuration file with 42 mappings
- `test_audio_mapper.py`: Core functionality tests
- `test_audio_mapper_reload.py`: Reload and error handling tests
- `example_audio_mapper_usage.py`: Usage examples and demonstrations
- `AUDIO_MAPPER_README.md`: This documentation file
