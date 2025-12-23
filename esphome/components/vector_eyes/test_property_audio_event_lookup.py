"""
Property-based tests for audio event lookup.

Feature: vector-animation-audio-sync, Property 5: Audio event lookup
Validates: Requirements 2.1
"""
import json
import tempfile
import os
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
def exact_mapping_strategy(draw):
    """
    Generate a dictionary of exact audio event to WAV file mappings.
    
    Returns a tuple of (mappings_dict, list_of_event_names)
    """
    num_mappings = draw(st.integers(min_value=1, max_value=20))
    
    mappings = {}
    event_names = []
    
    for _ in range(num_mappings):
        event_name = draw(audio_event_name_strategy())
        wav_file = draw(wav_file_name_strategy())
        
        # Ensure unique event names
        if event_name not in mappings:
            mappings[event_name] = wav_file
            event_names.append(event_name)
    
    return mappings, event_names


@given(mapping_data=exact_mapping_strategy())
@settings(max_examples=100, deadline=None)
def test_audio_event_lookup(mapping_data):
    """
    Feature: vector-animation-audio-sync, Property 5: Audio event lookup
    
    Property: For any audio event name in the mapping table, looking up that event 
    should return the corresponding WAV file identifier.
    
    Validates: Requirements 2.1
    """
    mappings, event_names = mapping_data
    
    # Create a temporary configuration file with the mappings
    config = {
        "exact_matches": mappings,
        "pattern_matches": {}
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: For every event name in the mapping table,
        # looking it up should return the corresponding WAV file
        for event_name in event_names:
            expected_wav = mappings[event_name]
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"Lookup for '{event_name}' should not return None (expected '{expected_wav}')"
            
            assert actual_wav == expected_wav, \
                f"Lookup for '{event_name}' returned '{actual_wav}', expected '{expected_wav}'"
        
        # Additional verification: All mappings should be accessible
        assert len(event_names) > 0, "Test should generate at least one mapping"
        
        # Verify that the mapper loaded the correct number of mappings
        stats = mapper.get_mapping_stats()
        assert stats['exact_count'] == len(mappings), \
            f"Mapper should have loaded {len(mappings)} exact mappings, but has {stats['exact_count']}"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def pattern_mapping_strategy(draw):
    """
    Generate a dictionary of pattern-based audio event to WAV file mappings.
    
    Returns a tuple of (pattern_mappings_dict, list_of_test_cases)
    where test_cases is a list of (event_name, expected_wav_file) tuples
    """
    num_patterns = draw(st.integers(min_value=1, max_value=15))
    
    pattern_mappings = {}
    test_cases = []
    
    for _ in range(num_patterns):
        # Generate a pattern (substring that will be matched)
        pattern = draw(st.text(
            min_size=5,
            max_size=20,
            alphabet=st.characters(whitelist_categories=('Lu', 'Ll'), min_codepoint=65, max_codepoint=122)
        ))
        
        wav_file = draw(wav_file_name_strategy())
        
        # Ensure unique patterns
        if pattern not in pattern_mappings:
            pattern_mappings[pattern] = wav_file
            
            # Generate an event name that contains this pattern
            prefix = draw(st.text(
                min_size=0,
                max_size=10,
                alphabet=st.characters(whitelist_categories=('Lu', 'Ll'), min_codepoint=65, max_codepoint=122)
            ))
            suffix = draw(st.text(
                min_size=0,
                max_size=10,
                alphabet=st.characters(whitelist_categories=('Lu', 'Ll'), min_codepoint=65, max_codepoint=122)
            ))
            
            event_name = f"{prefix}{pattern}{suffix}"
            test_cases.append((event_name, wav_file))
    
    return pattern_mappings, test_cases


@given(mapping_data=pattern_mapping_strategy())
@settings(max_examples=100, deadline=None)
def test_pattern_based_lookup(mapping_data):
    """
    Feature: vector-animation-audio-sync, Property 5: Audio event lookup (pattern matching)
    
    Property: For any audio event name that matches a pattern in the mapping table, 
    looking up that event should return the corresponding WAV file identifier.
    
    Validates: Requirements 2.1
    """
    pattern_mappings, test_cases = mapping_data
    
    # Create a temporary configuration file with the pattern mappings
    config = {
        "exact_matches": {},
        "pattern_matches": pattern_mappings
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: For every event name that contains a pattern,
        # looking it up should return the corresponding WAV file
        for event_name, expected_wav in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"Pattern lookup for '{event_name}' should not return None (expected '{expected_wav}')"
            
            assert actual_wav == expected_wav, \
                f"Pattern lookup for '{event_name}' returned '{actual_wav}', expected '{expected_wav}'"
        
        # Verify that the mapper loaded the correct number of pattern mappings
        stats = mapper.get_mapping_stats()
        assert stats['pattern_count'] == len(pattern_mappings), \
            f"Mapper should have loaded {len(pattern_mappings)} pattern mappings, but has {stats['pattern_count']}"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def combined_mapping_strategy(draw):
    """
    Generate both exact and pattern mappings together.
    
    Returns a tuple of (exact_mappings, pattern_mappings, test_cases)
    where test_cases is a list of (event_name, expected_wav_file, match_type) tuples
    """
    # Generate exact mappings
    num_exact = draw(st.integers(min_value=1, max_value=10))
    exact_mappings = {}
    test_cases = []
    
    for _ in range(num_exact):
        event_name = draw(audio_event_name_strategy())
        wav_file = draw(wav_file_name_strategy())
        
        if event_name not in exact_mappings:
            exact_mappings[event_name] = wav_file
            test_cases.append((event_name, wav_file, "exact"))
    
    # Generate pattern mappings
    num_patterns = draw(st.integers(min_value=1, max_value=10))
    pattern_mappings = {}
    
    for _ in range(num_patterns):
        pattern = draw(st.text(
            min_size=5,
            max_size=15,
            alphabet=st.characters(whitelist_categories=('Lu', 'Ll'), min_codepoint=65, max_codepoint=122)
        ))
        
        wav_file = draw(wav_file_name_strategy())
        
        if pattern not in pattern_mappings:
            pattern_mappings[pattern] = wav_file
            
            # Generate an event name that contains this pattern
            # but is NOT in exact_mappings
            prefix = draw(st.text(
                min_size=0,
                max_size=8,
                alphabet=st.characters(whitelist_categories=('Lu', 'Ll'), min_codepoint=65, max_codepoint=122)
            ))
            suffix = draw(st.text(
                min_size=0,
                max_size=8,
                alphabet=st.characters(whitelist_categories=('Lu', 'Ll'), min_codepoint=65, max_codepoint=122)
            ))
            
            event_name = f"{prefix}{pattern}{suffix}"
            
            # Only add if not in exact mappings (to avoid conflicts)
            if event_name not in exact_mappings:
                test_cases.append((event_name, wav_file, "pattern"))
    
    return exact_mappings, pattern_mappings, test_cases


@given(mapping_data=combined_mapping_strategy())
@settings(max_examples=100, deadline=None)
def test_combined_exact_and_pattern_lookup(mapping_data):
    """
    Feature: vector-animation-audio-sync, Property 5: Audio event lookup (combined)
    
    Property: For any audio event name in the mapping table (exact or pattern), 
    looking up that event should return the corresponding WAV file identifier.
    
    Validates: Requirements 2.1
    """
    exact_mappings, pattern_mappings, test_cases = mapping_data
    
    # Create a temporary configuration file with both types of mappings
    config = {
        "exact_matches": exact_mappings,
        "pattern_matches": pattern_mappings
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: For every event name (exact or pattern match),
        # looking it up should return the corresponding WAV file
        for event_name, expected_wav, match_type in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"{match_type.capitalize()} lookup for '{event_name}' should not return None (expected '{expected_wav}')"
            
            assert actual_wav == expected_wav, \
                f"{match_type.capitalize()} lookup for '{event_name}' returned '{actual_wav}', expected '{expected_wav}'"
        
        # Verify that the mapper loaded the correct number of mappings
        stats = mapper.get_mapping_stats()
        assert stats['exact_count'] == len(exact_mappings), \
            f"Mapper should have loaded {len(exact_mappings)} exact mappings, but has {stats['exact_count']}"
        assert stats['pattern_count'] == len(pattern_mappings), \
            f"Mapper should have loaded {len(pattern_mappings)} pattern mappings, but has {stats['pattern_count']}"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running property-based tests for audio event lookup...\n")
    
    try:
        test_audio_event_lookup()
        print("✓ Property test passed: Audio event lookup (exact matches)")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Audio event lookup (exact matches) - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_pattern_based_lookup()
        print("✓ Property test passed: Audio event lookup (pattern matching)")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Audio event lookup (pattern matching) - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_combined_exact_and_pattern_lookup()
        print("✓ Property test passed: Audio event lookup (combined)")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Audio event lookup (combined) - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print(f"\n{tests_passed} passed, {tests_failed} failed")
    exit(1 if tests_failed > 0 else 0)
