"""
Property-based tests for pattern matching support.

Feature: vector-animation-audio-sync, Property 25: Pattern matching support
Validates: Requirements 8.3
"""
import json
import tempfile
import os
from hypothesis import given, strategies as st, settings
from hypothesis.strategies import composite

from audio_mapper import AudioMapper


@composite
def pattern_and_events_strategy(draw):
    """
    Generate a pattern and multiple events that should match it.
    
    Returns:
        Tuple of (pattern, matching_events, non_matching_events)
    """
    # Generate a pattern (substring to match)
    pattern_parts = []
    pattern_parts.append(draw(st.sampled_from(["Scrn", "Emote", "Head", "Lift", "Play"])))
    pattern_parts.append(draw(st.sampled_from(["Curious", "Happy", "Sad", "Angry"])))
    
    separator = draw(st.sampled_from(["_", "__"]))
    pattern = separator.join(pattern_parts)
    
    # Generate events that contain the pattern
    num_matching = draw(st.integers(min_value=1, max_value=5))
    matching_events = []
    
    for _ in range(num_matching):
        # Add prefix and/or suffix to the pattern
        add_prefix = draw(st.booleans())
        add_suffix = draw(st.booleans())
        
        event = pattern
        if add_prefix:
            prefix = draw(st.sampled_from(["Robot", "Vic", "Sfx", "Test"]))
            event = prefix + separator + event
        if add_suffix:
            suffix = draw(st.sampled_from(["Short", "Long", "Micro", "Up", "Down"]))
            event = event + separator + suffix
        
        matching_events.append(event)
    
    # Generate events that don't contain the pattern
    num_non_matching = draw(st.integers(min_value=1, max_value=3))
    non_matching_events = []
    
    for _ in range(num_non_matching):
        # Create events with different components
        different_parts = []
        different_parts.append(draw(st.sampled_from(["Other", "Different", "Unrelated"])))
        different_parts.append(draw(st.sampled_from(["Neutral", "Excited", "Calm"])))
        
        non_matching_event = separator.join(different_parts)
        non_matching_events.append(non_matching_event)
    
    return pattern, matching_events, non_matching_events


@given(test_data=pattern_and_events_strategy())
@settings(max_examples=100, deadline=None)
def test_pattern_matching_identifies_all_matches(test_data):
    """
    Feature: vector-animation-audio-sync, Property 25: Pattern matching support
    
    Property: For any audio event name and a set of mapping patterns, the system 
    should correctly identify all patterns that match the event name.
    
    Validates: Requirements 8.3
    """
    pattern, matching_events, non_matching_events = test_data
    
    # Create a configuration with the pattern
    config = {
        "exact_matches": {},
        "pattern_matches": {
            pattern: "pattern_wav"
        }
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: All events containing the pattern should match
        for event in matching_events:
            actual_wav = mapper.map_event_to_wav(event)
            
            assert actual_wav is not None, \
                f"Event '{event}' contains pattern '{pattern}' but returned None"
            
            assert actual_wav == "pattern_wav", \
                f"Event '{event}' should match pattern '{pattern}', got '{actual_wav}'"
        
        # Property: Events not containing the pattern should not match
        for event in non_matching_events:
            actual_wav = mapper.map_event_to_wav(event)
            
            assert actual_wav is None, \
                f"Event '{event}' does not contain pattern '{pattern}' but returned '{actual_wav}'"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def multiple_patterns_strategy(draw):
    """
    Generate multiple patterns and events that match different combinations.
    
    Returns:
        Tuple of (patterns_dict, test_cases)
    """
    # Create multiple patterns
    patterns = {}
    
    pattern1 = draw(st.sampled_from(["Scrn", "Emote", "Head"]))
    patterns[pattern1] = "pattern1_wav"
    
    pattern2 = draw(st.sampled_from(["Curious", "Happy", "Sad"]))
    patterns[pattern2] = "pattern2_wav"
    
    pattern3 = draw(st.sampled_from(["Short", "Long", "Micro"]))
    patterns[pattern3] = "pattern3_wav"
    
    separator = draw(st.sampled_from(["_", "__"]))
    
    # Create test cases
    test_cases = []
    
    # Event matching pattern1 only
    event1 = pattern1
    test_cases.append((event1, [pattern1], "Should match pattern1 only"))
    
    # Event matching pattern1 and pattern2
    event2 = f"{pattern1}{separator}{pattern2}"
    test_cases.append((event2, [pattern1, pattern2], "Should match pattern1 and pattern2"))
    
    # Event matching all three patterns
    event3 = f"{pattern1}{separator}{pattern2}{separator}{pattern3}"
    test_cases.append((event3, [pattern1, pattern2, pattern3], "Should match all three patterns"))
    
    # Event matching pattern2 and pattern3 only
    event4 = f"Other{separator}{pattern2}{separator}{pattern3}"
    test_cases.append((event4, [pattern2, pattern3], "Should match pattern2 and pattern3"))
    
    # Event matching none
    event5 = "Unrelated_Event"
    test_cases.append((event5, [], "Should match no patterns"))
    
    return patterns, test_cases


@given(test_data=multiple_patterns_strategy())
@settings(max_examples=100, deadline=None)
def test_pattern_matching_multiple_patterns(test_data):
    """
    Feature: vector-animation-audio-sync, Property 25: Pattern matching support
    
    Property: The system should correctly identify all patterns that match an event,
    even when multiple patterns match the same event.
    
    Validates: Requirements 8.3
    """
    patterns, test_cases = test_data
    
    # Create a configuration with multiple patterns
    config = {
        "exact_matches": {},
        "pattern_matches": patterns
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: System should identify all matching patterns
        for event, expected_patterns, description in test_cases:
            # Use find_best_match to check which patterns match
            all_patterns = list(patterns.keys())
            
            # Find all patterns that match this event
            matching_patterns = [p for p in all_patterns if p in event]
            
            # Verify the expected patterns match
            assert set(matching_patterns) == set(expected_patterns), \
                f"{description}: Event '{event}' - expected patterns {expected_patterns}, found {matching_patterns}"
            
            # Verify map_event_to_wav returns a result if any pattern matches
            actual_wav = mapper.map_event_to_wav(event)
            
            if expected_patterns:
                assert actual_wav is not None, \
                    f"{description}: Event '{event}' should match at least one pattern"
            else:
                assert actual_wav is None, \
                    f"{description}: Event '{event}' should not match any pattern"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def substring_pattern_strategy(draw):
    """
    Generate patterns that are substrings of each other.
    
    Returns:
        Tuple of (patterns_dict, test_cases)
    """
    # Create nested substring patterns
    base = draw(st.sampled_from(["Scrn", "Head", "Lift"]))
    middle = draw(st.sampled_from(["Curious", "Happy"]))
    suffix = draw(st.sampled_from(["Short", "Long"]))
    
    separator = "_"
    
    # Create patterns of increasing length
    pattern1 = base
    pattern2 = f"{base}{separator}{middle}"
    pattern3 = f"{base}{separator}{middle}{separator}{suffix}"
    
    patterns = {
        pattern1: "base_wav",
        pattern2: "middle_wav",
        pattern3: "full_wav"
    }
    
    # Test cases
    test_cases = [
        (pattern1, [pattern1], "Base pattern should match itself"),
        (pattern2, [pattern1, pattern2], "Middle pattern should match base and itself"),
        (pattern3, [pattern1, pattern2, pattern3], "Full pattern should match all three"),
        (f"{pattern2}_Extra", [pattern1, pattern2], "Extended pattern should match base and middle"),
        ("Unrelated", [], "Unrelated event should match none")
    ]
    
    return patterns, test_cases


@given(test_data=substring_pattern_strategy())
@settings(max_examples=100, deadline=None)
def test_pattern_matching_substring_patterns(test_data):
    """
    Feature: vector-animation-audio-sync, Property 25: Pattern matching support
    
    Property: When patterns are substrings of each other, the system should 
    correctly identify all matching patterns.
    
    Validates: Requirements 8.3
    """
    patterns, test_cases = test_data
    
    # Create a configuration with substring patterns
    config = {
        "exact_matches": {},
        "pattern_matches": patterns
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: All substring patterns should be identified
        for event, expected_patterns, description in test_cases:
            # Find all patterns that match this event
            all_patterns = list(patterns.keys())
            matching_patterns = [p for p in all_patterns if p in event]
            
            # Verify the expected patterns match
            assert set(matching_patterns) == set(expected_patterns), \
                f"{description}: Event '{event}' - expected patterns {expected_patterns}, found {matching_patterns}"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def case_sensitive_pattern_strategy(draw):
    """
    Generate patterns with different case variations.
    
    Returns:
        Tuple of (pattern, events_dict)
    """
    # Create a pattern with mixed case
    base = draw(st.sampled_from(["Scrn", "Head", "Lift"]))
    emotion = draw(st.sampled_from(["Curious", "Happy", "Sad"]))
    
    pattern = f"{base}_{emotion}"
    
    # Create events with different case variations
    events = {
        pattern: True,  # Exact case - should match
        pattern.lower(): False,  # All lowercase - should not match (case sensitive)
        pattern.upper(): False,  # All uppercase - should not match (case sensitive)
        f"{base.lower()}_{emotion}": False,  # Mixed case - should not match
    }
    
    return pattern, events


@given(test_data=case_sensitive_pattern_strategy())
@settings(max_examples=100, deadline=None)
def test_pattern_matching_case_sensitive(test_data):
    """
    Feature: vector-animation-audio-sync, Property 25: Pattern matching support
    
    Property: Pattern matching should be case-sensitive, only matching events
    with the exact case.
    
    Validates: Requirements 8.3
    """
    pattern, events = test_data
    
    # Create a configuration with the pattern
    config = {
        "exact_matches": {},
        "pattern_matches": {
            pattern: "pattern_wav"
        }
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: Only exact case should match
        for event, should_match in events.items():
            actual_wav = mapper.map_event_to_wav(event)
            
            if should_match:
                assert actual_wav is not None, \
                    f"Event '{event}' with exact case should match pattern '{pattern}'"
                assert actual_wav == "pattern_wav", \
                    f"Event '{event}' should return 'pattern_wav', got '{actual_wav}'"
            else:
                assert actual_wav is None, \
                    f"Event '{event}' with different case should not match pattern '{pattern}'"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def real_world_pattern_strategy(draw):
    """
    Generate test cases based on real Vector audio event patterns.
    
    Returns:
        Tuple of (patterns_dict, test_cases)
    """
    # Real-world patterns from Vector's audio system
    scenarios = [
        {
            "patterns": {
                "Play__Robot": "robot_wav",
                "Vic_Sfx": "sfx_wav",
                "Blink": "blink_wav"
            },
            "tests": [
                ("Play__Robot_Vic_Sfx__Blink", ["Play__Robot", "Vic_Sfx", "Blink"], "Should match all three patterns"),
                ("Play__Robot_Other", ["Play__Robot"], "Should match Play__Robot only"),
                ("Something_Vic_Sfx", ["Vic_Sfx"], "Should match Vic_Sfx only"),
                ("Blink_Short", ["Blink"], "Should match Blink only"),
                ("Unrelated_Event", [], "Should match no patterns")
            ]
        },
        {
            "patterns": {
                "Scrn": "screen_wav",
                "Curious": "curious_wav",
                "Short": "short_wav"
            },
            "tests": [
                ("Scrn_Curious_Short", ["Scrn", "Curious", "Short"], "Should match all patterns"),
                ("Scrn_Happy", ["Scrn"], "Should match Scrn only"),
                ("Emote_Curious", ["Curious"], "Should match Curious only"),
                ("Head_Down_Short", ["Short"], "Should match Short only"),
                ("Other_Event", [], "Should match no patterns")
            ]
        },
        {
            "patterns": {
                "Head_Down": "head_down_wav",
                "Micro": "micro_wav"
            },
            "tests": [
                ("Head_Down_Micro_Curious", ["Head_Down", "Micro"], "Should match both patterns"),
                ("Head_Down_Short", ["Head_Down"], "Should match Head_Down only"),
                ("Scrn_Micro", ["Micro"], "Should match Micro only"),
                ("Head_Up", [], "Should match no patterns")
            ]
        }
    ]
    
    scenario = draw(st.sampled_from(scenarios))
    
    return scenario["patterns"], scenario["tests"]


@given(test_data=real_world_pattern_strategy())
@settings(max_examples=100, deadline=None)
def test_pattern_matching_real_world(test_data):
    """
    Feature: vector-animation-audio-sync, Property 25: Pattern matching support
    
    Property: Real-world Vector audio event patterns should correctly identify
    all matching patterns for each event.
    
    Validates: Requirements 8.3
    """
    patterns, test_cases = test_data
    
    # Create a configuration with real-world patterns
    config = {
        "exact_matches": {},
        "pattern_matches": patterns
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: All matching patterns should be identified
        for event, expected_patterns, description in test_cases:
            # Find all patterns that match this event
            all_patterns = list(patterns.keys())
            matching_patterns = [p for p in all_patterns if p in event]
            
            # Verify the expected patterns match
            assert set(matching_patterns) == set(expected_patterns), \
                f"{description}: Event '{event}' - expected patterns {expected_patterns}, found {matching_patterns}"
            
            # Verify map_event_to_wav returns a result if any pattern matches
            actual_wav = mapper.map_event_to_wav(event)
            
            if expected_patterns:
                assert actual_wav is not None, \
                    f"{description}: Event '{event}' should match at least one pattern"
            else:
                assert actual_wav is None, \
                    f"{description}: Event '{event}' should not match any pattern"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def special_character_pattern_strategy(draw):
    """
    Generate test cases with special characters in patterns.
    
    Returns:
        Tuple of (patterns_dict, test_cases)
    """
    # Create patterns with special characters commonly found in Vector events
    patterns = {
        "Play__Robot": "robot_wav",  # Double underscore
        "Vic_Sfx": "sfx_wav",  # Single underscore
        "Head-Down": "head_down_wav",  # Hyphen
    }
    
    # Test cases
    test_cases = [
        ("Play__Robot_Vic_Sfx", ["Play__Robot", "Vic_Sfx"], "Should match patterns with underscores"),
        ("Head-Down_Micro", ["Head-Down"], "Should match pattern with hyphen"),
        ("Play__Robot", ["Play__Robot"], "Should match exact pattern"),
        ("Other_Event", [], "Should not match unrelated event"),
    ]
    
    return patterns, test_cases


@given(test_data=special_character_pattern_strategy())
@settings(max_examples=100, deadline=None)
def test_pattern_matching_special_characters(test_data):
    """
    Feature: vector-animation-audio-sync, Property 25: Pattern matching support
    
    Property: Patterns with special characters (underscores, hyphens) should 
    match correctly.
    
    Validates: Requirements 8.3
    """
    patterns, test_cases = test_data
    
    # Create a configuration with patterns
    config = {
        "exact_matches": {},
        "pattern_matches": patterns
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: Special characters should be handled correctly
        for event, expected_patterns, description in test_cases:
            # Find all patterns that match this event
            all_patterns = list(patterns.keys())
            matching_patterns = [p for p in all_patterns if p in event]
            
            # Verify the expected patterns match
            assert set(matching_patterns) == set(expected_patterns), \
                f"{description}: Event '{event}' - expected patterns {expected_patterns}, found {matching_patterns}"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running property-based tests for pattern matching support...\n")
    
    try:
        test_pattern_matching_identifies_all_matches()
        print("✓ Property test passed: Pattern matching - identifies all matches")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Pattern matching - identifies all matches - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_pattern_matching_multiple_patterns()
        print("✓ Property test passed: Pattern matching - multiple patterns")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Pattern matching - multiple patterns - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_pattern_matching_substring_patterns()
        print("✓ Property test passed: Pattern matching - substring patterns")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Pattern matching - substring patterns - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_pattern_matching_case_sensitive()
        print("✓ Property test passed: Pattern matching - case sensitive")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Pattern matching - case sensitive - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_pattern_matching_real_world()
        print("✓ Property test passed: Pattern matching - real world")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Pattern matching - real world - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_pattern_matching_special_characters()
        print("✓ Property test passed: Pattern matching - special characters")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Pattern matching - special characters - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print(f"\n{tests_passed} passed, {tests_failed} failed")
    exit(1 if tests_failed > 0 else 0)
