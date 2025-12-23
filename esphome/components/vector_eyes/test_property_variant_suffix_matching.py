"""
Property-based tests for variant suffix matching.

Feature: vector-animation-audio-sync, Property 7: Variant suffix matching
Validates: Requirements 2.5
"""
import json
import tempfile
import os
from hypothesis import given, strategies as st, settings
from hypothesis.strategies import composite

from audio_mapper import AudioMapper


# Common variant suffixes used in Vector audio events
VARIANT_SUFFIXES = ["_Short", "_Long", "_Micro", "_Medium", "_Up", "_Down", "_Fast", "_Slow"]


@composite
def variant_event_strategy(draw):
    """
    Generate audio event names with variant suffixes.
    
    Returns:
        Tuple of (base_event, variant_event, suffix)
    """
    # Generate a base event name
    components = []
    components.append(draw(st.sampled_from(["Play__Robot_Vic_Sfx", "Scrn", "Emote", "Head", "Lift"])))
    components.append(draw(st.sampled_from(["Curious", "Happy", "Sad", "Angry", "Neutral", "Excited", "Blink"])))
    
    separator = draw(st.sampled_from(["__", "_"]))
    base_event = separator.join(components)
    
    # Add a variant suffix
    suffix = draw(st.sampled_from(VARIANT_SUFFIXES))
    variant_event = base_event + suffix
    
    return base_event, variant_event, suffix


@given(test_data=variant_event_strategy())
@settings(max_examples=100, deadline=None)
def test_variant_suffix_exact_match_preferred(test_data):
    """
    Feature: vector-animation-audio-sync, Property 7: Variant suffix matching
    
    Property: For any audio event with variant suffixes, the system should match 
    the most specific mapping available, preferring exact matches over partial matches.
    
    Test case: When both exact and partial matches exist, exact match should win.
    
    Validates: Requirements 2.5
    """
    base_event, variant_event, suffix = test_data
    
    # Create mappings with both exact and partial matches
    pattern_mappings = {
        base_event: "base_wav",  # Partial match (pattern)
    }
    
    exact_mappings = {
        variant_event: "variant_wav"  # Exact match
    }
    
    # Create a temporary configuration file
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
        
        # Property: Exact match should be preferred over pattern match
        actual_wav = mapper.map_event_to_wav(variant_event)
        
        assert actual_wav is not None, \
            f"Lookup for '{variant_event}' should not return None"
        
        assert actual_wav == "variant_wav", \
            f"Exact match should be preferred: '{variant_event}' returned '{actual_wav}', expected 'variant_wav'"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@given(test_data=variant_event_strategy())
@settings(max_examples=100, deadline=None)
def test_variant_suffix_fallback_to_base(test_data):
    """
    Feature: vector-animation-audio-sync, Property 7: Variant suffix matching
    
    Property: When no exact match exists for a variant, the system should fall back
    to the base event pattern match.
    
    Validates: Requirements 2.5
    """
    base_event, variant_event, suffix = test_data
    
    # Create mappings with only base pattern (no exact match for variant)
    pattern_mappings = {
        base_event: "base_wav"
    }
    
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
        
        # Property: Should fall back to base pattern match
        actual_wav = mapper.map_event_to_wav(variant_event)
        
        assert actual_wav is not None, \
            f"Lookup for '{variant_event}' should fall back to base pattern"
        
        assert actual_wav == "base_wav", \
            f"Should fall back to base pattern: '{variant_event}' returned '{actual_wav}', expected 'base_wav'"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def multiple_variant_strategy(draw):
    """
    Generate test cases with multiple variant levels.
    
    Returns:
        Tuple of (base_event, variants, mappings)
    """
    # Generate a base event
    base = draw(st.sampled_from(["Scrn_Curious", "Head_Down", "Lift_Up", "Emote_Happy"]))
    
    # Generate multiple variants with increasing specificity
    variant1_suffix = draw(st.sampled_from(["_Short", "_Long", "_Micro"]))
    variant1 = base + variant1_suffix
    
    variant2_suffix = draw(st.sampled_from(["_Up", "_Down", "_Fast"]))
    variant2 = variant1 + variant2_suffix
    
    # Create mappings for different specificity levels
    mappings = {
        "exact_matches": {
            variant2: "most_specific_wav"  # Most specific: exact match for full variant
        },
        "pattern_matches": {
            variant1: "medium_specific_wav",  # Medium: pattern match for first variant
            base: "least_specific_wav"  # Least: pattern match for base
        }
    }
    
    test_cases = [
        (variant2, "most_specific_wav", "Full variant should use exact match"),
        (variant1, "medium_specific_wav", "First variant should use its pattern match"),
        (base, "least_specific_wav", "Base should use its pattern match")
    ]
    
    return mappings, test_cases


@given(test_data=multiple_variant_strategy())
@settings(max_examples=100, deadline=None)
def test_variant_suffix_multiple_levels(test_data):
    """
    Feature: vector-animation-audio-sync, Property 7: Variant suffix matching
    
    Property: With multiple variant levels, the most specific match should be used
    at each level.
    
    Validates: Requirements 2.5
    """
    mappings, test_cases = test_data
    
    # Create a temporary configuration file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(mappings, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: Each variant level should use its most specific match
        for event_name, expected_wav, description in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"{description}: Lookup for '{event_name}' should not return None"
            
            assert actual_wav == expected_wav, \
                f"{description}: '{event_name}' returned '{actual_wav}', expected '{expected_wav}'"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def real_world_variant_strategy(draw):
    """
    Generate test cases based on real Vector audio event variant patterns.
    
    Returns:
        Tuple of (mappings, test_cases)
    """
    # Real-world scenarios from Vector's audio system
    scenarios = [
        {
            "mappings": {
                "exact_matches": {
                    "Scrn_Curious_Short": "curious_short_exact",
                    "Scrn_Curious_Long": "curious_long_exact"
                },
                "pattern_matches": {
                    "Scrn_Curious": "curious_base"
                }
            },
            "tests": [
                ("Scrn_Curious_Short", "curious_short_exact", "Exact match for Short variant"),
                ("Scrn_Curious_Long", "curious_long_exact", "Exact match for Long variant"),
                ("Scrn_Curious_Medium", "curious_base", "Fallback to base for unmapped Medium variant"),
                ("Scrn_Curious", "curious_base", "Base event uses base mapping")
            ]
        },
        {
            "mappings": {
                "exact_matches": {
                    "Head_Down_Micro_Curious": "head_down_micro_curious_exact"
                },
                "pattern_matches": {
                    "Head_Down_Micro": "head_down_micro_pattern",
                    "Head_Down": "head_down_pattern"
                }
            },
            "tests": [
                ("Head_Down_Micro_Curious", "head_down_micro_curious_exact", "Exact match for full variant"),
                ("Head_Down_Micro_Happy", "head_down_micro_pattern", "Pattern match for Micro + different emotion"),
                ("Head_Down_Short", "head_down_pattern", "Pattern match for Down + different variant"),
                ("Head_Down", "head_down_pattern", "Base pattern match")
            ]
        },
        {
            "mappings": {
                "exact_matches": {},
                "pattern_matches": {
                    "Play__Robot_Vic_Sfx__Blink": "blink_pattern",
                    "Play__Robot_Vic_Sfx": "sfx_pattern"
                }
            },
            "tests": [
                ("Play__Robot_Vic_Sfx__Blink_Short", "blink_pattern", "Variant falls back to more specific pattern"),
                ("Play__Robot_Vic_Sfx__Blink_Long", "blink_pattern", "Another variant falls back to more specific pattern"),
                ("Play__Robot_Vic_Sfx__Blink", "blink_pattern", "Base event uses its pattern"),
                ("Play__Robot_Vic_Sfx__Wake", "sfx_pattern", "Different event uses less specific pattern")
            ]
        }
    ]
    
    scenario = draw(st.sampled_from(scenarios))
    
    return scenario["mappings"], scenario["tests"]


@given(test_data=real_world_variant_strategy())
@settings(max_examples=100, deadline=None)
def test_variant_suffix_real_world_patterns(test_data):
    """
    Feature: vector-animation-audio-sync, Property 7: Variant suffix matching
    
    Property: Real-world Vector audio events with variants should match the most
    specific mapping available, with proper fallback behavior.
    
    Validates: Requirements 2.5
    """
    mappings, test_cases = test_data
    
    # Create a temporary configuration file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(mappings, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: Most specific match should be used for each variant
        for event_name, expected_wav, description in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"{description}: Lookup for '{event_name}' should not return None"
            
            assert actual_wav == expected_wav, \
                f"{description}: '{event_name}' returned '{actual_wav}', expected '{expected_wav}'"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def variant_with_no_base_strategy(draw):
    """
    Generate test cases where a variant exists but no base mapping exists.
    
    Returns:
        Tuple of (base_event, variant_event, suffix)
    """
    # Generate a base event name
    base = draw(st.sampled_from(["Scrn_Test", "Head_Test", "Lift_Test"]))
    
    # Add a variant suffix
    suffix = draw(st.sampled_from(VARIANT_SUFFIXES))
    variant = base + suffix
    
    return base, variant, suffix


@given(test_data=variant_with_no_base_strategy())
@settings(max_examples=100, deadline=None)
def test_variant_suffix_no_base_mapping(test_data):
    """
    Feature: vector-animation-audio-sync, Property 7: Variant suffix matching
    
    Property: When a variant has an exact match but no base mapping exists,
    the exact match should be used. When neither exists, return None.
    
    Validates: Requirements 2.5
    """
    base_event, variant_event, suffix = test_data
    
    # Test case 1: Variant has exact match, base has no mapping
    config1 = {
        "exact_matches": {
            variant_event: "variant_wav"
        },
        "pattern_matches": {}
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config1, f)
        temp_file1 = f.name
    
    try:
        mapper1 = AudioMapper(temp_file1)
        
        # Variant should use exact match
        actual_wav = mapper1.map_event_to_wav(variant_event)
        assert actual_wav == "variant_wav", \
            f"Variant with exact match should return 'variant_wav', got '{actual_wav}'"
        
        # Base should return None (no mapping)
        base_wav = mapper1.map_event_to_wav(base_event)
        assert base_wav is None, \
            f"Base with no mapping should return None, got '{base_wav}'"
    
    finally:
        if os.path.exists(temp_file1):
            os.unlink(temp_file1)
    
    # Test case 2: Neither variant nor base has mapping
    config2 = {
        "exact_matches": {},
        "pattern_matches": {}
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config2, f)
        temp_file2 = f.name
    
    try:
        mapper2 = AudioMapper(temp_file2)
        
        # Both should return None
        variant_wav = mapper2.map_event_to_wav(variant_event)
        assert variant_wav is None, \
            f"Variant with no mapping should return None, got '{variant_wav}'"
        
        base_wav = mapper2.map_event_to_wav(base_event)
        assert base_wav is None, \
            f"Base with no mapping should return None, got '{base_wav}'"
    
    finally:
        if os.path.exists(temp_file2):
            os.unlink(temp_file2)


@composite
def suffix_only_pattern_strategy(draw):
    """
    Generate test cases where patterns match only the suffix part.
    
    Returns:
        Tuple of (events, mappings, expected_results)
    """
    # Create events with different bases but same suffix
    suffix = draw(st.sampled_from(VARIANT_SUFFIXES))
    
    base1 = draw(st.sampled_from(["Scrn_Curious", "Head_Down", "Lift_Up"]))
    base2 = draw(st.sampled_from(["Scrn_Happy", "Head_Up", "Lift_Down"]))
    
    event1 = base1 + suffix
    event2 = base2 + suffix
    
    # Create a pattern that matches the suffix
    mappings = {
        "exact_matches": {},
        "pattern_matches": {
            suffix: "suffix_wav"  # Pattern matches just the suffix
        }
    }
    
    test_cases = [
        (event1, "suffix_wav", f"Event with {suffix} should match suffix pattern"),
        (event2, "suffix_wav", f"Different event with {suffix} should also match suffix pattern")
    ]
    
    return mappings, test_cases


@given(test_data=suffix_only_pattern_strategy())
@settings(max_examples=100, deadline=None)
def test_variant_suffix_pattern_matching(test_data):
    """
    Feature: vector-animation-audio-sync, Property 7: Variant suffix matching
    
    Property: Patterns that match variant suffixes should work correctly,
    matching any event that contains that suffix.
    
    Validates: Requirements 2.5
    """
    mappings, test_cases = test_data
    
    # Create a temporary configuration file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(mappings, f)
        temp_file = f.name
    
    try:
        # Create AudioMapper with the configuration
        mapper = AudioMapper(temp_file)
        
        # Property: Suffix pattern should match all events with that suffix
        for event_name, expected_wav, description in test_cases:
            actual_wav = mapper.map_event_to_wav(event_name)
            
            assert actual_wav is not None, \
                f"{description}: Lookup for '{event_name}' should not return None"
            
            assert actual_wav == expected_wav, \
                f"{description}: '{event_name}' returned '{actual_wav}', expected '{expected_wav}'"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running property-based tests for variant suffix matching...\n")
    
    try:
        test_variant_suffix_exact_match_preferred()
        print("✓ Property test passed: Variant suffix - exact match preferred")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Variant suffix - exact match preferred - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_variant_suffix_fallback_to_base()
        print("✓ Property test passed: Variant suffix - fallback to base")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Variant suffix - fallback to base - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_variant_suffix_multiple_levels()
        print("✓ Property test passed: Variant suffix - multiple levels")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Variant suffix - multiple levels - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_variant_suffix_real_world_patterns()
        print("✓ Property test passed: Variant suffix - real world patterns")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Variant suffix - real world patterns - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_variant_suffix_no_base_mapping()
        print("✓ Property test passed: Variant suffix - no base mapping")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Variant suffix - no base mapping - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_variant_suffix_pattern_matching()
        print("✓ Property test passed: Variant suffix - pattern matching")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Variant suffix - pattern matching - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print(f"\n{tests_passed} passed, {tests_failed} failed")
    exit(1 if tests_failed > 0 else 0)
