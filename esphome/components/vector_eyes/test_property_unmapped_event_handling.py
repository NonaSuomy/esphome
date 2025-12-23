"""
Property-based tests for unmapped event handling.

Feature: vector-animation-audio-sync, Property 6: Unmapped event handling
Validates: Requirements 2.2
"""
import json
import tempfile
import os
import logging
from io import StringIO
from hypothesis import given, strategies as st, settings
from hypothesis.strategies import composite

from audio_mapper import AudioMapper


@composite
def audio_event_name_strategy(draw):
    """
    Generate valid audio event names following Vector's naming convention.
    
    Vector audio events typically follow patterns like:
    - Play__Robot_Vic_Sfx__EventName
    - Play__Robot_Vic__EventName
    - Scrn_EventName
    - Emote_EventName
    - Head_EventName
    - Lift_EventName
    - Tread_EventName
    """
    prefixes = [
        "Play__Robot_Vic_Sfx__",
        "Play__Robot_Vic__",
        "Scrn_",
        "Emote_",
        "Head_",
        "Lift_",
        "Tread_"
    ]
    
    prefix = draw(st.sampled_from(prefixes))
    
    # Generate event name components
    components = []
    num_components = draw(st.integers(min_value=1, max_value=4))
    
    for _ in range(num_components):
        component = draw(st.text(
            min_size=3,
            max_size=15,
            alphabet=st.characters(whitelist_categories=('Lu', 'Ll'), min_codepoint=65, max_codepoint=122)
        ))
        components.append(component)
    
    event_name = prefix + "_".join(components)
    return event_name


@composite
def wav_file_name_strategy(draw):
    """Generate valid WAV file identifiers (without extension)."""
    # WAV file names are typically lowercase with underscores
    components = []
    num_components = draw(st.integers(min_value=1, max_value=3))
    
    for _ in range(num_components):
        component = draw(st.text(
            min_size=3,
            max_size=12,
            alphabet=st.characters(whitelist_categories=('Ll',), min_codepoint=97, max_codepoint=122)
        ))
        components.append(component)
    
    wav_file = "_".join(components)
    return wav_file


@composite
def unmapped_event_strategy(draw):
    """
    Generate a set of mappings and unmapped event names.
    
    Returns a tuple of (mappings_dict, list_of_unmapped_event_names)
    """
    # Generate some mappings
    num_mappings = draw(st.integers(min_value=1, max_value=10))
    
    mappings = {}
    
    for i in range(num_mappings):
        # Use a predictable pattern to ensure uniqueness
        event_name = f"Play__Robot_Vic_Sfx__Mapped_{i}_{draw(st.text(min_size=3, max_size=8, alphabet='abcdefghijklmnopqrstuvwxyz'))}"
        wav_file = draw(wav_file_name_strategy())
        mappings[event_name] = wav_file
    
    # Generate unmapped event names (guaranteed to not be in mappings)
    # Use a different prefix to ensure they don't match
    num_unmapped = draw(st.integers(min_value=1, max_value=5))
    unmapped_events = []
    
    for i in range(num_unmapped):
        # Use "Unmapped" prefix to ensure it's different from "Mapped"
        event_name = f"Play__Robot_Vic_Sfx__Unmapped_{i}_{draw(st.text(min_size=3, max_size=8, alphabet='abcdefghijklmnopqrstuvwxyz'))}"
        unmapped_events.append(event_name)
    
    return mappings, unmapped_events


@given(test_data=unmapped_event_strategy())
@settings(max_examples=100, deadline=None)
def test_unmapped_event_handling(test_data):
    """
    Feature: vector-animation-audio-sync, Property 6: Unmapped event handling
    
    Property: For any audio event name not in the mapping table, the system should 
    log a warning and continue without assigning audio to that event (return None).
    
    Validates: Requirements 2.2
    """
    mappings, unmapped_events = test_data
    
    # Create a temporary configuration file with the mappings
    config = {
        "exact_matches": mappings,
        "pattern_matches": {}
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Set up logging to capture warnings
        log_stream = StringIO()
        handler = logging.StreamHandler(log_stream)
        handler.setLevel(logging.WARNING)
        
        # Get the audio_mapper logger
        logger = logging.getLogger('audio_mapper')
        original_level = logger.level
        logger.setLevel(logging.WARNING)
        logger.addHandler(handler)
        
        try:
            # Create AudioMapper with the configuration
            mapper = AudioMapper(temp_file)
            
            # Property: For every unmapped event name,
            # looking it up should return None and log a warning
            for event_name in unmapped_events:
                # Clear the log stream
                log_stream.truncate(0)
                log_stream.seek(0)
                
                # Attempt to map the unmapped event
                result = mapper.map_event_to_wav(event_name)
                
                # Should return None for unmapped events
                assert result is None, \
                    f"Unmapped event '{event_name}' should return None, but returned '{result}'"
                
                # Should log a warning
                log_output = log_stream.getvalue()
                assert "No mapping found" in log_output or "warning" in log_output.lower(), \
                    f"Unmapped event '{event_name}' should log a warning, but log was: '{log_output}'"
                
                assert event_name in log_output, \
                    f"Warning log should mention the event name '{event_name}', but log was: '{log_output}'"
            
            # Additional verification: Mapped events should still work
            for mapped_event, expected_wav in mappings.items():
                result = mapper.map_event_to_wav(mapped_event)
                assert result == expected_wav, \
                    f"Mapped event '{mapped_event}' should return '{expected_wav}', but returned '{result}'"
            
            # Verify that we tested at least one unmapped event
            assert len(unmapped_events) > 0, "Test should generate at least one unmapped event"
        
        finally:
            # Clean up logging
            logger.removeHandler(handler)
            logger.setLevel(original_level)
            handler.close()
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def unmapped_pattern_strategy(draw):
    """
    Generate pattern mappings and event names that don't match any pattern.
    
    Returns a tuple of (pattern_mappings_dict, list_of_unmapped_event_names)
    """
    # Generate some pattern mappings with predictable patterns
    num_patterns = draw(st.integers(min_value=1, max_value=8))
    
    pattern_mappings = {}
    
    for i in range(num_patterns):
        # Use predictable patterns that won't accidentally match unmapped events
        pattern = f"PATTERN_{i}_MATCH"
        wav_file = draw(wav_file_name_strategy())
        pattern_mappings[pattern] = wav_file
    
    # Generate event names that don't contain any of the patterns
    # Use numeric-only event names to ensure they don't match the letter-based patterns
    num_unmapped = draw(st.integers(min_value=1, max_value=5))
    unmapped_events = []
    
    for i in range(num_unmapped):
        # Use numeric event names that won't match "PATTERN_X_MATCH"
        event_name = f"Event_{draw(st.integers(min_value=1000, max_value=9999))}"
        unmapped_events.append(event_name)
    
    return pattern_mappings, unmapped_events


@given(test_data=unmapped_pattern_strategy())
@settings(max_examples=100, deadline=None)
def test_unmapped_pattern_handling(test_data):
    """
    Feature: vector-animation-audio-sync, Property 6: Unmapped event handling (patterns)
    
    Property: For any audio event name that doesn't match any pattern in the mapping table, 
    the system should log a warning and return None.
    
    Validates: Requirements 2.2
    """
    pattern_mappings, unmapped_events = test_data
    
    # Create a temporary configuration file with pattern mappings
    config = {
        "exact_matches": {},
        "pattern_matches": pattern_mappings
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Set up logging to capture warnings
        log_stream = StringIO()
        handler = logging.StreamHandler(log_stream)
        handler.setLevel(logging.WARNING)
        
        # Get the audio_mapper logger
        logger = logging.getLogger('audio_mapper')
        original_level = logger.level
        logger.setLevel(logging.WARNING)
        logger.addHandler(handler)
        
        try:
            # Create AudioMapper with the configuration
            mapper = AudioMapper(temp_file)
            
            # Property: For every event name that doesn't match any pattern,
            # looking it up should return None and log a warning
            for event_name in unmapped_events:
                # Clear the log stream
                log_stream.truncate(0)
                log_stream.seek(0)
                
                # Attempt to map the unmapped event
                result = mapper.map_event_to_wav(event_name)
                
                # Should return None for unmapped events
                assert result is None, \
                    f"Unmapped event '{event_name}' should return None, but returned '{result}'"
                
                # Should log a warning
                log_output = log_stream.getvalue()
                assert "No mapping found" in log_output or "warning" in log_output.lower(), \
                    f"Unmapped event '{event_name}' should log a warning, but log was: '{log_output}'"
            
            # Verify that we tested at least one unmapped event
            assert len(unmapped_events) > 0, "Test should generate at least one unmapped event"
        
        finally:
            # Clean up logging
            logger.removeHandler(handler)
            logger.setLevel(original_level)
            handler.close()
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


if __name__ == "__main__":
    import sys
    import traceback
    
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running property-based tests for unmapped event handling...\n", flush=True)
    
    try:
        print("Starting test 1: Unmapped event handling (exact matches)...", flush=True)
        test_unmapped_event_handling()
        print("✓ Property test passed: Unmapped event handling (exact matches)", flush=True)
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Unmapped event handling (exact matches)", flush=True)
        print(f"Error: {e}", flush=True)
        traceback.print_exc()
        tests_failed += 1
    
    try:
        print("\nStarting test 2: Unmapped event handling (patterns)...", flush=True)
        test_unmapped_pattern_handling()
        print("✓ Property test passed: Unmapped event handling (patterns)", flush=True)
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Unmapped event handling (patterns)", flush=True)
        print(f"Error: {e}", flush=True)
        traceback.print_exc()
        tests_failed += 1
    
    print(f"\n{tests_passed} passed, {tests_failed} failed", flush=True)
    sys.exit(1 if tests_failed > 0 else 0)
