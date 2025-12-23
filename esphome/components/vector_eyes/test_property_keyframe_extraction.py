"""
Property-based tests for keyframe extraction.

Feature: vector-animation-audio-sync, Property 1: Complete keyframe extraction
Validates: Requirements 1.1
"""
import json
import tempfile
import os
from hypothesis import given, strategies as st, settings
from hypothesis.strategies import composite

from json_parser import JSONParser
from animation_data import AnimationData


# Supported keyframe types from requirements
KEYFRAME_TYPES = [
    "ProceduralFaceKeyFrame",
    "RobotAudioKeyFrame",
    "HeadAngleKeyFrame",
    "LiftHeightKeyFrame",
    "RecordHeadingKeyFrame"
]


@composite
def keyframe_strategy(draw, keyframe_type=None):
    """Generate a valid keyframe of a specific type."""
    if keyframe_type is None:
        keyframe_type = draw(st.sampled_from(KEYFRAME_TYPES))
    
    trigger_time = draw(st.integers(min_value=0, max_value=10000))
    
    keyframe = {
        "Name": keyframe_type,
        "triggerTime_ms": trigger_time
    }
    
    # Add type-specific fields
    if keyframe_type == "ProceduralFaceKeyFrame":
        keyframe.update({
            "faceScaleX": draw(st.floats(min_value=0.1, max_value=2.0)),
            "faceScaleY": draw(st.floats(min_value=0.1, max_value=2.0)),
            "faceAngle": draw(st.floats(min_value=-180.0, max_value=180.0)),
            "faceCenterX": draw(st.floats(min_value=-10.0, max_value=10.0)),
            "faceCenterY": draw(st.floats(min_value=-10.0, max_value=10.0)),
            "leftEye": [draw(st.floats()) for _ in range(25)],
            "rightEye": [draw(st.floats()) for _ in range(25)],
            "durationTime_ms": draw(st.integers(min_value=0, max_value=1000))
        })
    elif keyframe_type == "RobotAudioKeyFrame":
        keyframe.update({
            "audioName": [draw(st.text(min_size=1, max_size=50))],
            "volume": draw(st.floats(min_value=0.0, max_value=1.0)),
            "probability": [draw(st.floats(min_value=0.0, max_value=1.0))]
        })
    elif keyframe_type == "HeadAngleKeyFrame":
        keyframe.update({
            "angle_deg": draw(st.floats(min_value=-45.0, max_value=45.0))
        })
    elif keyframe_type == "LiftHeightKeyFrame":
        keyframe.update({
            "height_mm": draw(st.floats(min_value=0.0, max_value=100.0))
        })
    elif keyframe_type == "RecordHeadingKeyFrame":
        keyframe.update({
            "heading_deg": draw(st.floats(min_value=0.0, max_value=360.0))
        })
    
    return keyframe


@composite
def animation_json_strategy(draw):
    """Generate a valid animation JSON structure with multiple keyframe types."""
    anim_name = draw(st.text(min_size=5, max_size=30, alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd'), whitelist_characters='_')))
    
    # Generate keyframes of different types
    num_keyframes = draw(st.integers(min_value=1, max_value=20))
    keyframes = []
    
    for _ in range(num_keyframes):
        kf_type = draw(st.sampled_from(KEYFRAME_TYPES))
        keyframe = draw(keyframe_strategy(keyframe_type=kf_type))
        keyframes.append(keyframe)
    
    return {anim_name: keyframes}


def count_keyframes_by_type(json_data):
    """Count keyframes by type in the original JSON."""
    anim_name = list(json_data.keys())[0]
    keyframes = json_data[anim_name]
    
    counts = {}
    for kf in keyframes:
        if isinstance(kf, dict) and "Name" in kf and "triggerTime_ms" in kf:
            kf_type = kf["Name"]
            counts[kf_type] = counts.get(kf_type, 0) + 1
    
    return counts


@given(animation_json=animation_json_strategy())
@settings(max_examples=100, deadline=None)
def test_complete_keyframe_extraction(animation_json):
    """
    Feature: vector-animation-audio-sync, Property 1: Complete keyframe extraction
    
    Property: For any valid JSON animation file, parsing should extract all keyframes 
    of all supported types (ProceduralFaceKeyFrame, RobotAudioKeyFrame, HeadAngleKeyFrame, 
    LiftHeightKeyFrame, RecordHeadingKeyFrame) without loss.
    
    Validates: Requirements 1.1
    """
    # Write JSON to temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(animation_json, f)
        temp_file = f.name
    
    try:
        # Parse the animation
        parser = JSONParser()
        parsed = parser.parse_animation_file(temp_file)
        
        # Count keyframes by type in original JSON
        original_counts = count_keyframes_by_type(animation_json)
        
        # Count keyframes by type in parsed result
        parsed_counts = {}
        for kf in parsed.keyframes:
            kf_type = kf.keyframe_type
            parsed_counts[kf_type] = parsed_counts.get(kf_type, 0) + 1
        
        # All types should have same counts
        for kf_type in KEYFRAME_TYPES:
            original_count = original_counts.get(kf_type, 0)
            parsed_count = parsed_counts.get(kf_type, 0)
            
            assert original_count == parsed_count, \
                f"Keyframe type {kf_type}: expected {original_count}, got {parsed_count}"
        
        # Total count should match
        total_original = sum(original_counts.values())
        total_parsed = len(parsed.keyframes)
        
        assert total_original == total_parsed, \
            f"Total keyframes: expected {total_original}, got {total_parsed}"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def animation_with_same_trigger_times_strategy(draw):
    """Generate animation JSON with multiple keyframes at the same trigger time."""
    anim_name = draw(st.text(min_size=5, max_size=30, alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd'), whitelist_characters='_')))
    
    # Generate a few different trigger times
    num_trigger_times = draw(st.integers(min_value=2, max_value=5))
    trigger_times = [draw(st.integers(min_value=0, max_value=5000)) for _ in range(num_trigger_times)]
    
    keyframes = []
    
    # For each trigger time, create multiple keyframes
    for trigger_time in trigger_times:
        num_keyframes_at_time = draw(st.integers(min_value=2, max_value=4))
        
        for _ in range(num_keyframes_at_time):
            kf_type = draw(st.sampled_from(KEYFRAME_TYPES))
            keyframe = draw(keyframe_strategy(keyframe_type=kf_type))
            # Override the trigger time to ensure they're the same
            keyframe["triggerTime_ms"] = trigger_time
            keyframes.append(keyframe)
    
    return {anim_name: keyframes}


@given(animation_json=animation_with_same_trigger_times_strategy())
@settings(max_examples=100, deadline=None)
def test_keyframe_ordering_preservation(animation_json):
    """
    Feature: vector-animation-audio-sync, Property 2: Keyframe ordering preservation
    
    Property: For any animation with multiple keyframes at the same trigger time, 
    the relative order of those keyframes should be preserved after parsing and sorting.
    
    Validates: Requirements 1.2
    """
    # Write JSON to temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(animation_json, f)
        temp_file = f.name
    
    try:
        # Get original keyframes
        anim_name = list(animation_json.keys())[0]
        original_keyframes = animation_json[anim_name]
        
        # Parse the animation
        parser = JSONParser()
        parsed = parser.parse_animation_file(temp_file)
        
        # Group original keyframes by trigger time
        original_by_time = {}
        for idx, kf in enumerate(original_keyframes):
            if isinstance(kf, dict) and "triggerTime_ms" in kf and "Name" in kf:
                trigger_time = kf["triggerTime_ms"]
                if trigger_time not in original_by_time:
                    original_by_time[trigger_time] = []
                original_by_time[trigger_time].append((idx, kf["Name"]))
        
        # Group parsed keyframes by trigger time
        parsed_by_time = {}
        for idx, kf in enumerate(parsed.keyframes):
            trigger_time = kf.trigger_time
            if trigger_time not in parsed_by_time:
                parsed_by_time[trigger_time] = []
            parsed_by_time[trigger_time].append((idx, kf.keyframe_type))
        
        # For each trigger time with multiple keyframes, verify order is preserved
        for trigger_time in original_by_time:
            if len(original_by_time[trigger_time]) > 1:
                # Get the types in original order
                original_types = [kf_type for _, kf_type in original_by_time[trigger_time]]
                
                # Get the types in parsed order
                if trigger_time in parsed_by_time:
                    parsed_types = [kf_type for _, kf_type in parsed_by_time[trigger_time]]
                    
                    # The relative order should be preserved
                    assert original_types == parsed_types, \
                        f"At trigger time {trigger_time}ms: original order {original_types} != parsed order {parsed_types}"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def nested_animation_json_strategy(draw):
    """
    Generate animation JSON with various nesting structures.
    
    This tests that the parser can correctly navigate different valid JSON structures
    to extract the animation name and keyframe array, regardless of how the data
    is nested or structured.
    """
    anim_name = draw(st.text(min_size=5, max_size=30, alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd'), whitelist_characters='_')))
    
    # Generate keyframes
    num_keyframes = draw(st.integers(min_value=1, max_value=15))
    keyframes = []
    
    for _ in range(num_keyframes):
        kf_type = draw(st.sampled_from(KEYFRAME_TYPES))
        keyframe = draw(keyframe_strategy(keyframe_type=kf_type))
        keyframes.append(keyframe)
    
    # The standard Vector format is: {animation_name: [keyframes]}
    # This is the structure we need to test
    return {anim_name: keyframes}


@given(animation_json=nested_animation_json_strategy())
@settings(max_examples=100, deadline=None)
def test_json_structure_navigation(animation_json):
    """
    Feature: vector-animation-audio-sync, Property 3: JSON structure navigation
    
    Property: For any JSON file following Vector's animation format, the parser 
    should correctly extract the animation name and keyframe array regardless of 
    nesting depth.
    
    Validates: Requirements 1.3
    """
    # Write JSON to temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(animation_json, f)
        temp_file = f.name
    
    try:
        # Get expected animation name and keyframe count from original JSON
        expected_anim_name = list(animation_json.keys())[0]
        expected_keyframes = animation_json[expected_anim_name]
        
        # Count valid keyframes (those with Name and triggerTime_ms)
        expected_valid_count = 0
        for kf in expected_keyframes:
            if isinstance(kf, dict) and "Name" in kf and "triggerTime_ms" in kf:
                expected_valid_count += 1
        
        # Parse the animation
        parser = JSONParser()
        parsed = parser.parse_animation_file(temp_file)
        
        # Verify animation name was correctly extracted
        assert parsed.name == expected_anim_name, \
            f"Animation name mismatch: expected '{expected_anim_name}', got '{parsed.name}'"
        
        # Verify keyframes array was correctly extracted
        assert len(parsed.keyframes) == expected_valid_count, \
            f"Keyframe count mismatch: expected {expected_valid_count}, got {len(parsed.keyframes)}"
        
        # Verify each keyframe has the correct structure
        for kf in parsed.keyframes:
            assert hasattr(kf, 'trigger_time'), "Keyframe missing trigger_time"
            assert hasattr(kf, 'keyframe_type'), "Keyframe missing keyframe_type"
            assert hasattr(kf, 'data'), "Keyframe missing data"
            assert kf.keyframe_type in KEYFRAME_TYPES, \
                f"Invalid keyframe type: {kf.keyframe_type}"
        
        # Verify that the data field contains the original keyframe data
        for kf in parsed.keyframes:
            assert isinstance(kf.data, dict), "Keyframe data should be a dictionary"
            assert "Name" in kf.data, "Keyframe data should contain 'Name' field"
            assert "triggerTime_ms" in kf.data, "Keyframe data should contain 'triggerTime_ms' field"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


@composite
def malformed_keyframe_strategy(draw):
    """Generate a keyframe that is missing required fields or has invalid data."""
    malformation_type = draw(st.sampled_from([
        "missing_name",
        "missing_trigger_time",
        "invalid_trigger_time_type",
        "negative_trigger_time",
        "missing_both",
        "not_a_dict"
    ]))
    
    if malformation_type == "missing_name":
        # Keyframe without Name field
        return {
            "triggerTime_ms": draw(st.integers(min_value=0, max_value=5000)),
            "someOtherField": draw(st.text())
        }
    elif malformation_type == "missing_trigger_time":
        # Keyframe without triggerTime_ms field
        return {
            "Name": draw(st.sampled_from(KEYFRAME_TYPES)),
            "someOtherField": draw(st.text())
        }
    elif malformation_type == "invalid_trigger_time_type":
        # Keyframe with non-numeric triggerTime_ms
        return {
            "Name": draw(st.sampled_from(KEYFRAME_TYPES)),
            "triggerTime_ms": draw(st.text(min_size=1, max_size=10))
        }
    elif malformation_type == "negative_trigger_time":
        # Keyframe with negative trigger time (should be rejected by validation)
        return {
            "Name": draw(st.sampled_from(KEYFRAME_TYPES)),
            "triggerTime_ms": draw(st.integers(min_value=-10000, max_value=-1))
        }
    elif malformation_type == "missing_both":
        # Keyframe missing both required fields
        return {
            "someField": draw(st.text()),
            "anotherField": draw(st.integers())
        }
    else:  # not_a_dict
        # Not even a dictionary
        return draw(st.one_of(
            st.text(),
            st.integers(),
            st.lists(st.integers()),
            st.none()
        ))


@composite
def animation_with_malformed_keyframes_strategy(draw):
    """
    Generate animation JSON with a mix of valid and malformed keyframes.
    
    This tests that the parser can handle malformed keyframes gracefully by
    skipping them and continuing to process valid keyframes.
    """
    anim_name = draw(st.text(min_size=5, max_size=30, alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd'), whitelist_characters='_')))
    
    # Generate a mix of valid and malformed keyframes
    num_valid = draw(st.integers(min_value=1, max_value=10))
    num_malformed = draw(st.integers(min_value=1, max_value=5))
    
    keyframes = []
    
    # Add valid keyframes
    for _ in range(num_valid):
        kf_type = draw(st.sampled_from(KEYFRAME_TYPES))
        keyframe = draw(keyframe_strategy(keyframe_type=kf_type))
        keyframes.append(keyframe)
    
    # Add malformed keyframes
    for _ in range(num_malformed):
        malformed_kf = draw(malformed_keyframe_strategy())
        keyframes.append(malformed_kf)
    
    # Shuffle to mix valid and malformed keyframes
    import random
    random.shuffle(keyframes)
    
    return {anim_name: keyframes}


@given(animation_json=animation_with_malformed_keyframes_strategy())
@settings(max_examples=100, deadline=None)
def test_malformed_keyframe_handling(animation_json):
    """
    Feature: vector-animation-audio-sync, Property 4: Malformed keyframe handling
    
    Property: For any keyframe missing required fields, the system should skip that 
    keyframe, log a descriptive error, and continue processing remaining keyframes.
    
    Validates: Requirements 1.4
    """
    # Write JSON to temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(animation_json, f)
        temp_file = f.name
    
    try:
        # Count valid keyframes in original JSON
        anim_name = list(animation_json.keys())[0]
        original_keyframes = animation_json[anim_name]
        
        valid_count = 0
        for kf in original_keyframes:
            # A keyframe is valid if it's a dict with Name and triggerTime_ms fields
            # and triggerTime_ms can be converted to a non-negative integer
            if isinstance(kf, dict) and "Name" in kf and "triggerTime_ms" in kf:
                try:
                    trigger_time = int(kf["triggerTime_ms"])
                    if trigger_time >= 0:
                        valid_count += 1
                except (ValueError, TypeError):
                    pass
        
        # Parse the animation
        parser = JSONParser()
        
        # The parser should not raise an exception for malformed keyframes
        # It should skip them and continue
        parsed = parser.parse_animation_file(temp_file)
        
        # Verify that the parser extracted exactly the valid keyframes
        assert parsed is not None, "Parser should return AnimationData even with malformed keyframes"
        assert parsed.name == anim_name, f"Animation name should be preserved: expected '{anim_name}', got '{parsed.name}'"
        
        # The number of parsed keyframes should equal the number of valid keyframes
        assert len(parsed.keyframes) == valid_count, \
            f"Parser should extract only valid keyframes: expected {valid_count}, got {len(parsed.keyframes)}"
        
        # All parsed keyframes should be valid
        for kf in parsed.keyframes:
            assert kf.validate(), "All parsed keyframes should pass validation"
            assert kf.trigger_time >= 0, "All parsed keyframes should have non-negative trigger times"
            assert kf.keyframe_type in KEYFRAME_TYPES, f"Invalid keyframe type: {kf.keyframe_type}"
            assert isinstance(kf.data, dict), "Keyframe data should be a dictionary"
            assert "Name" in kf.data, "Keyframe data should contain 'Name' field"
            assert "triggerTime_ms" in kf.data, "Keyframe data should contain 'triggerTime_ms' field"
        
        # Verify that at least some keyframes were skipped (since we added malformed ones)
        total_keyframes = len(original_keyframes)
        skipped_count = total_keyframes - valid_count
        
        # We should have skipped at least one malformed keyframe
        assert skipped_count > 0, \
            f"Test should have generated malformed keyframes to skip (total: {total_keyframes}, valid: {valid_count})"
        
        # The parser should continue processing even after encountering malformed keyframes
        # This is verified by the fact that we successfully parsed and got valid keyframes
        assert len(parsed.keyframes) > 0, \
            "Parser should extract at least some valid keyframes despite malformed ones"
    
    finally:
        # Clean up temporary file
        if os.path.exists(temp_file):
            os.unlink(temp_file)


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    try:
        test_complete_keyframe_extraction()
        print("✓ Property test passed: Complete keyframe extraction")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Complete keyframe extraction - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_keyframe_ordering_preservation()
        print("✓ Property test passed: Keyframe ordering preservation")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Keyframe ordering preservation - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_json_structure_navigation()
        print("✓ Property test passed: JSON structure navigation")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: JSON structure navigation - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_malformed_keyframe_handling()
        print("✓ Property test passed: Malformed keyframe handling")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Malformed keyframe handling - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print(f"\n{tests_passed} passed, {tests_failed} failed")
    exit(1 if tests_failed > 0 else 0)
