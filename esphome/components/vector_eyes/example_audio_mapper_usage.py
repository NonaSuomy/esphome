"""
Example usage of AudioMapper in the animation conversion pipeline.

This demonstrates how to integrate the AudioMapper class with the existing
conversion script to map audio events to WAV files.
"""

from audio_mapper import AudioMapper
from animation_data import AnimationData, AudioEventData
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(name)s: %(message)s')


def example_basic_usage():
    """Basic usage example."""
    print("\n=== Basic Usage Example ===\n")
    
    # Create mapper with configuration file
    mapper = AudioMapper("audio_mappings.json")
    
    # Map some audio events
    events = [
        "Play__Robot_Vic_Sfx__Blink",
        "Play__Robot_Vic_Sfx__Scrn_Happy_Short",
        "Play__Robot_Vic_Sfx__Scrn_Curious_Long",
        "Play__Robot_Vic_Sfx__Unknown_Event"
    ]
    
    for event in events:
        wav_file = mapper.map_event_to_wav(event)
        if wav_file:
            print(f"✓ {event} -> {wav_file}.wav")
        else:
            print(f"✗ {event} -> (unmapped)")


def example_with_animation_data():
    """Example showing integration with AnimationData."""
    print("\n=== Integration with AnimationData Example ===\n")
    
    # Create mapper
    mapper = AudioMapper("audio_mappings.json")
    
    # Simulate audio events from an animation
    audio_events = [
        AudioEventData(
            trigger_time=0,
            event_names=["Play__Robot_Vic_Sfx__Blink"]
        ),
        AudioEventData(
            trigger_time=500,
            event_names=["Play__Robot_Vic_Sfx__Scrn_Happy_Short"]
        ),
        AudioEventData(
            trigger_time=1000,
            event_names=["Play__Robot_Vic_Sfx__Emote_Curious_Long"]
        )
    ]
    
    # Map each audio event to WAV file
    for audio_event in audio_events:
        event_name = audio_event.event_names[0]
        wav_file = mapper.map_event_to_wav(event_name)
        
        if wav_file:
            audio_event.wav_file = wav_file
            print(f"✓ t={audio_event.trigger_time}ms: {event_name} -> {wav_file}.wav")
        else:
            print(f"✗ t={audio_event.trigger_time}ms: {event_name} -> (unmapped)")


def example_hot_reload():
    """Example showing hot-reload functionality."""
    print("\n=== Hot-Reload Example ===\n")
    
    # Create mapper
    mapper = AudioMapper("audio_mappings.json")
    
    # Initial mapping
    result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Blink")
    print(f"Initial mapping: Blink -> {result}")
    
    # Simulate configuration update (in real usage, user would edit the file)
    print("\n(Simulating configuration file update...)")
    
    # Reload mappings
    success = mapper.reload_mappings()
    if success:
        print("✓ Mappings reloaded successfully")
        
        # Test mapping again
        result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Blink")
        print(f"After reload: Blink -> {result}")
    else:
        print("✗ Failed to reload mappings")


def example_pattern_matching():
    """Example showing pattern matching with specificity."""
    print("\n=== Pattern Matching Example ===\n")
    
    mapper = AudioMapper("audio_mappings.json")
    
    # Test various patterns
    test_cases = [
        ("Play__Robot_Vic_Sfx__Scrn_Happy", "Should match 'Scrn_Happy' pattern"),
        ("Play__Robot_Vic_Sfx__Scrn_Happy_Short", "Should match more specific 'Scrn_Happy_Short' pattern"),
        ("Play__Robot_Vic_Sfx__Scrn_Happy_Long", "Should match more specific 'Scrn_Happy_Long' pattern"),
        ("Play__Robot_Vic_Sfx__Head_Down_Micro_Curious", "Should match 'Head_Down_Micro_Curious' pattern"),
    ]
    
    for event, description in test_cases:
        result = mapper.map_event_to_wav(event)
        print(f"{description}")
        print(f"  {event} -> {result}.wav\n")


def example_statistics():
    """Example showing mapping statistics."""
    print("\n=== Mapping Statistics Example ===\n")
    
    mapper = AudioMapper("audio_mappings.json")
    
    stats = mapper.get_mapping_stats()
    print(f"Exact mappings loaded: {stats['exact_count']}")
    print(f"Pattern mappings loaded: {stats['pattern_count']}")
    print(f"Total mappings: {stats['exact_count'] + stats['pattern_count']}")


if __name__ == "__main__":
    print("=" * 60)
    print("AudioMapper Usage Examples")
    print("=" * 60)
    
    example_basic_usage()
    example_with_animation_data()
    example_hot_reload()
    example_pattern_matching()
    example_statistics()
    
    print("\n" + "=" * 60)
    print("Examples completed!")
    print("=" * 60)
