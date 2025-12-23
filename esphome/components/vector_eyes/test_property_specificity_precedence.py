"""
Property-based tests for specificity precedence.

Feature: vector-animation-audio-sync, Property 26: Specificity precedence
Validates: Requirements 8.4
"""
import json
import tempfile
import os
from hypothesis import given, strategies as st, settings, assume
from hypothesis.strategies import composite

from audio_mapper import AudioMapper


@composite
def multiple_pattern_match_strategy(draw):
    """
    Generate test cases where multiple patterns match the same event.
    
    Creates scenarios with overlapping patterns of different specificity levels
    to test that the most specific (longest) pattern wins.
    
    Returns:
        Tuple of (pattern_mappings, test_cases)
        where test_cases is a list of (event_name, expected_wav, matching_patterns) tuples
    """
    # Generate a base event structure with multiple levels
    components = []
    
    # Level 1: Base prefix
    prefix = draw(st.sampled_from(["Play__Robot_Vic_Sfx", "Scrn", "Emote", "Head", "Lift"]))
    components.append(prefix)
    
    # Level 2: Add emotion/action
    emotion = draw(st.sampled_from(["Curious", "Happy", "Sad", "Angry", "Neutral", "Excited"]))
    components.append(emotion)
    
    # Level 3: Add variant (optional)
    add_variant = draw(st.booleans())
    if add_variant:
        variant = draw(st.sampled_from(["Short", "Long", "Micro", "Medium", "Up", "Down"]))
        components.append(variant)
    
    # Build event name with separators
    separator = draw(st.sampled_from(["__", "_"]))
    event_name = separator.join(components)
    
    # Create patterns with increasing specificity
    pattern_mappings = {}
    matching_patterns = []
    
    # Pattern 1: Just the prefix (least specific)
    pattern1 = components[0]
    wav1 = "level1_wav"
    pattern_mappings[pattern1] = wav1
    matching_patterns.append((pattern1, wav1, len(pattern1)))
    
    # Pattern 2: Prefix + emotion (more specific)
    pattern2 = separator.join(components[:2])
    wav2 = "level2_wav"
    pattern_mappings[pattern2] = wav2
    matching_patterns.append((pattern2, wav2, len(pattern2)))
    
    # Pattern 3: Full event (most specific) - only if we have a variant
    if len(components) >= 3:
        pattern3 = event_name
        wav3 = "level3_wav"
        pattern_mappings[pattern3] = wav3
        matching_patterns.append((pattern3, wav3, len(pattern3)))
        expected_wav = wav3
    else:
        expected_wav = wav2
    
    # Sort by length to verify our expectation
    matching_patterns.sort(key=lambda x: x[2], reverse=True)
    
    test_cases = [
        (event_name, expected_wav, [p[0] for p in matching_patterns])
    ]
    
    return pattern_mappings, test_cases


@given(test_data=multiple_pattern_match_strategy())
@settings(max_examples=100, deadline=None)
def test_specificity_precedence_longest_match(test_data):
    """
    Feature: vector-animation-audio-sync, Property 26: Specificity precedence
    
    Property: For any audio event that matches multiple mapping patterns, 
    the system should use the most specific pattern (longest match).
    
    Validates: Requirements 8.4
    """
    pattern_mappings, test_cases = test_data
    
    # Create a temporary configuration file
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
        
        # Property: The longest (most specific) matching pattern should be used
        for event_name, expected_wav, matching_patterns in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"Lookup for '{event_name}' should not return None (expected '{expected_wav}')"
            
            assert actual_wav == expected_wav, \
                f"Specificity precedence failed: '{event_name}' returned '{actual_wav}', expected '{expected_wav}' " \
                f"(matching patterns: {matching_patterns})"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def nested_pattern_strategy(draw):
    """
    Generate test cases with nested patterns where one pattern is a substring of another.
    
    Returns:
        Tuple of (pattern_mappings, test_cases)
    """
    # Create a hierarchy of patterns
    base = draw(st.sampled_from(["Scrn", "Emote", "Head", "Lift"]))
    emotion = draw(st.sampled_from(["Curious", "Happy", "Sad"]))
    variant = draw(st.sampled_from(["Short", "Long", "Micro"]))
    
    # Build nested patterns
    pattern1 = base  # e.g., "Scrn"
    pattern2 = f"{base}_{emotion}"  # e.g., "Scrn_Curious"
    pattern3 = f"{base}_{emotion}_{variant}"  # e.g., "Scrn_Curious_Short"
    
    # Create mappings
    pattern_mappings = {
        pattern1: "base_wav",
        pattern2: "emotion_wav",
        pattern3: "variant_wav"
    }
    
    # Test cases
    test_cases = [
        # Event that matches all three patterns - should use most specific
        (pattern3, "variant_wav", [pattern1, pattern2, pattern3]),
        # Event that matches first two patterns - should use more specific
        (pattern2, "emotion_wav", [pattern1, pattern2]),
        # Event that matches only first pattern
        (pattern1, "base_wav", [pattern1])
    ]
    
    return pattern_mappings, test_cases


@given(test_data=nested_pattern_strategy())
@settings(max_examples=100, deadline=None)
def test_specificity_precedence_nested_patterns(test_data):
    """
    Feature: vector-animation-audio-sync, Property 26: Specificity precedence
    
    Property: When patterns are nested (one is a substring of another), 
    the longest matching pattern should be used.
    
    Validates: Requirements 8.4
    """
    pattern_mappings, test_cases = test_data
    
    # Create a temporary configuration file
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
        
        # Property: Longest matching pattern should win
        for event_name, expected_wav, matching_patterns in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"Lookup for '{event_name}' should not return None (expected '{expected_wav}')"
            
            assert actual_wav == expected_wav, \
                f"Nested pattern precedence failed: '{event_name}' returned '{actual_wav}', expected '{expected_wav}' " \
                f"(matching patterns: {matching_patterns})"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def overlapping_patterns_strategy(draw):
    """
    Generate test cases with overlapping patterns that match different parts of an event.
    
    Returns:
        Tuple of (pattern_mappings, test_cases)
    """
    # Create an event with multiple components
    parts = []
    parts.append(draw(st.sampled_from(["Play", "Stop", "Pause"])))
    parts.append(draw(st.sampled_from(["Robot", "Vic", "Sfx"])))
    parts.append(draw(st.sampled_from(["Blink", "Happy", "Curious"])))
    parts.append(draw(st.sampled_from(["Short", "Long"])))
    
    event_name = "__".join(parts)
    
    # Create overlapping patterns of different lengths
    pattern_mappings = {}
    
    # Short patterns (2 parts)
    pattern1 = "__".join(parts[:2])  # e.g., "Play__Robot"
    pattern_mappings[pattern1] = "short1_wav"
    
    pattern2 = "__".join(parts[1:3])  # e.g., "Robot__Vic"
    pattern_mappings[pattern2] = "short2_wav"
    
    # Medium pattern (3 parts)
    pattern3 = "__".join(parts[:3])  # e.g., "Play__Robot__Vic"
    pattern_mappings[pattern3] = "medium_wav"
    
    # Long pattern (4 parts - full event)
    pattern4 = event_name
    pattern_mappings[pattern4] = "long_wav"
    
    # The longest pattern should win
    test_cases = [
        (event_name, "long_wav", [pattern1, pattern2, pattern3, pattern4])
    ]
    
    return pattern_mappings, test_cases


@given(test_data=overlapping_patterns_strategy())
@settings(max_examples=100, deadline=None)
def test_specificity_precedence_overlapping_patterns(test_data):
    """
    Feature: vector-animation-audio-sync, Property 26: Specificity precedence
    
    Property: When multiple overlapping patterns match different parts of an event,
    the longest matching pattern should be used.
    
    Validates: Requirements 8.4
    """
    pattern_mappings, test_cases = test_data
    
    # Create a temporary configuration file
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
        
        # Property: Longest matching pattern should win
        for event_name, expected_wav, matching_patterns in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"Lookup for '{event_name}' should not return None (expected '{expected_wav}')"
            
            assert actual_wav == expected_wav, \
                f"Overlapping pattern precedence failed: '{event_name}' returned '{actual_wav}', expected '{expected_wav}' " \
                f"(matching patterns: {matching_patterns})"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def real_world_specificity_strategy(draw):
    """
    Generate test cases based on real Vector audio event patterns with multiple matches.
    
    Returns:
        Tuple of (pattern_mappings, test_cases)
    """
    # Real-world scenarios where multiple patterns could match
    test_scenarios = [
        {
            "patterns": {
                "Scrn": "generic_screen",
                "Scrn_Curious": "curious_short",
                "Scrn_Curious_Short": "curious_short_specific"
            },
            "tests": [
                ("Scrn_Curious_Short", "curious_short_specific", ["Scrn", "Scrn_Curious", "Scrn_Curious_Short"]),
                ("Scrn_Curious_Long", "curious_short", ["Scrn", "Scrn_Curious"]),
                ("Scrn_Happy", "generic_screen", ["Scrn"])
            ]
        },
        {
            "patterns": {
                "Head": "generic_head",
                "Head_Down": "head_down",
                "Head_Down_Micro": "head_down_micro",
                "Head_Down_Micro_Curious": "head_down_micro_curious"
            },
            "tests": [
                ("Head_Down_Micro_Curious", "head_down_micro_curious", 
                 ["Head", "Head_Down", "Head_Down_Micro", "Head_Down_Micro_Curious"]),
                ("Head_Down_Micro_Happy", "head_down_micro", 
                 ["Head", "Head_Down", "Head_Down_Micro"]),
                ("Head_Down_Short", "head_down", ["Head", "Head_Down"]),
                ("Head_Up", "generic_head", ["Head"])
            ]
        },
        {
            "patterns": {
                "Play__Robot": "robot_sound",
                "Play__Robot_Vic": "vic_sound",
                "Play__Robot_Vic_Sfx": "sfx_sound",
                "Play__Robot_Vic_Sfx__Blink": "blink_sound"
            },
            "tests": [
                ("Play__Robot_Vic_Sfx__Blink", "blink_sound", 
                 ["Play__Robot", "Play__Robot_Vic", "Play__Robot_Vic_Sfx", "Play__Robot_Vic_Sfx__Blink"]),
                ("Play__Robot_Vic_Sfx__Wake", "sfx_sound", 
                 ["Play__Robot", "Play__Robot_Vic", "Play__Robot_Vic_Sfx"]),
                ("Play__Robot_Vic_Other", "vic_sound", 
                 ["Play__Robot", "Play__Robot_Vic"]),
                ("Play__Robot_Other", "robot_sound", ["Play__Robot"])
            ]
        }
    ]
    
    scenario = draw(st.sampled_from(test_scenarios))
    
    return scenario["patterns"], scenario["tests"]


@given(test_data=real_world_specificity_strategy())
@settings(max_examples=100, deadline=None)
def test_specificity_precedence_real_world(test_data):
    """
    Feature: vector-animation-audio-sync, Property 26: Specificity precedence
    
    Property: Real-world Vector audio events with multiple matching patterns
    should use the most specific (longest) pattern.
    
    Validates: Requirements 8.4
    """
    pattern_mappings, test_cases = test_data
    
    # Create a temporary configuration file
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
        
        # Property: Most specific pattern should win
        for event_name, expected_wav, matching_patterns in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"Real-world lookup for '{event_name}' should not return None (expected '{expected_wav}')"
            
            assert actual_wav == expected_wav, \
                f"Real-world specificity precedence failed: '{event_name}' returned '{actual_wav}', expected '{expected_wav}' " \
                f"(matching patterns: {matching_patterns})"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def equal_length_patterns_strategy(draw):
    """
    Generate test cases where multiple patterns of equal length match an event.
    
    In this case, the first match in the sorted order should be used.
    
    Returns:
        Tuple of (pattern_mappings, test_cases)
    """
    # Create an event that contains multiple patterns of the same length
    base = draw(st.sampled_from(["Test", "Demo", "Sample"]))
    suffix = draw(st.sampled_from(["Alpha", "Beta", "Gamma"]))
    
    event_name = f"{base}_Event_{suffix}"
    
    # Create patterns of equal length that both match
    pattern1 = f"{base}_Event"  # Length: len(base) + 6
    pattern2 = f"Event_{suffix}"  # Length: 6 + len(suffix)
    
    # Ensure they're actually equal length
    assume(len(pattern1) == len(pattern2))
    
    # Both patterns match, but pattern_mappings is sorted by length (descending)
    # When lengths are equal, the order in the dict matters
    # AudioMapper sorts by length, so equal-length patterns maintain their order
    pattern_mappings = {
        pattern1: "first_wav",
        pattern2: "second_wav"
    }
    
    # The first pattern in the list should be used (since they're equal length)
    test_cases = [
        (event_name, "first_wav", [pattern1, pattern2])
    ]
    
    return pattern_mappings, test_cases


@given(test_data=equal_length_patterns_strategy())
@settings(max_examples=100, deadline=None)
def test_specificity_precedence_equal_length(test_data):
    """
    Feature: vector-animation-audio-sync, Property 26: Specificity precedence
    
    Property: When multiple patterns of equal length match an event,
    the first matching pattern (in sorted order) should be used.
    
    Validates: Requirements 8.4
    """
    pattern_mappings, test_cases = test_data
    
    # Create a temporary configuration file
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
        
        # Property: First matching pattern of equal length should be used
        for event_name, expected_wav, matching_patterns in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"Lookup for '{event_name}' should not return None (expected '{expected_wav}')"
            
            assert actual_wav == expected_wav, \
                f"Equal-length pattern precedence failed: '{event_name}' returned '{actual_wav}', expected '{expected_wav}' " \
                f"(matching patterns: {matching_patterns})"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running property-based tests for specificity precedence...\n")
    
    try:
        test_specificity_precedence_longest_match()
        print("✓ Property test passed: Specificity precedence - longest match")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Specificity precedence - longest match - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_specificity_precedence_nested_patterns()
        print("✓ Property test passed: Specificity precedence - nested patterns")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Specificity precedence - nested patterns - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_specificity_precedence_overlapping_patterns()
        print("✓ Property test passed: Specificity precedence - overlapping patterns")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Specificity precedence - overlapping patterns - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_specificity_precedence_real_world()
        print("✓ Property test passed: Specificity precedence - real world")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Specificity precedence - real world - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_specificity_precedence_equal_length()
        print("✓ Property test passed: Specificity precedence - equal length")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Specificity precedence - equal length - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print(f"\n{tests_passed} passed, {tests_failed} failed")
    exit(1 if tests_failed > 0 else 0)
