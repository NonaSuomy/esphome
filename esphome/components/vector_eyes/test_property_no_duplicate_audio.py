"""
Property-based tests for no duplicate audio assignment.

Feature: vector-animation-audio-sync, Property 22: No duplicate audio assignment
Validates: Requirements 6.4
"""
from hypothesis import given, strategies as st, settings
from hypothesis.strategies import composite

from keyframe_matcher import KeyframeMatcher
from keyframe_extractor import ProceduralFaceData
from animation_data import AudioEventData


def create_face_keyframe(trigger_time: int) -> ProceduralFaceData:
    """Helper to create a ProceduralFaceData with default values."""
    return ProceduralFaceData(
        trigger_time=trigger_time,
        duration=100,
        scale_x=1.0,
        scale_y=1.0,
        angle=0.0,
        center_x=0.0,
        center_y=0.0,
        left_lid_top=0.6,
        left_lid_bottom=0.6,
        right_lid_top=0.6,
        right_lid_bottom=0.6
    )


@composite
def animation_strategy(draw):
    """
    Generate a complete animation with visual keyframes and audio events.
    
    Returns:
        Tuple of (visual_keyframes, audio_events)
    """
    # Generate 5-30 visual keyframes
    num_visual = draw(st.integers(min_value=5, max_value=30))
    visual_keyframes = []
    
    for i in range(num_visual):
        trigger_time = i * 100  # Space them 100ms apart
        visual_keyframes.append(create_face_keyframe(trigger_time))
    
    # Generate 1-50 audio events
    num_audio = draw(st.integers(min_value=1, max_value=50))
    audio_events = []
    
    for i in range(num_audio):
        # Random trigger time within the animation range
        trigger_time = draw(st.integers(min_value=0, max_value=num_visual * 100))
        
        audio_events.append(
            AudioEventData(
                trigger_time=trigger_time,
                event_names=[f"event{i}"],
                wav_file=f"sound{i}.wav"
            )
        )
    
    return visual_keyframes, audio_events


@given(test_data=animation_strategy())
@settings(max_examples=100, deadline=None)
def test_no_duplicate_audio_assignment(test_data):
    """
    Feature: vector-animation-audio-sync, Property 22: No duplicate audio assignment
    
    Property: For any converted animation, no visual keyframe should have more 
    than one audio event assigned to it.
    
    Validates: Requirements 6.4
    """
    visual_keyframes, audio_events = test_data
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Property: Each keyframe index should appear exactly once in the mapping
    keyframe_indices = list(keyframe_sound_map.keys())
    unique_indices = set(keyframe_indices)
    
    assert len(keyframe_indices) == len(unique_indices), \
        f"Duplicate keyframe assignments found: {len(keyframe_indices)} assignments, " \
        f"but only {len(unique_indices)} unique keyframes"
    
    # Property: No keyframe should have more than one audio event
    for idx in keyframe_indices:
        count = keyframe_indices.count(idx)
        assert count == 1, \
            f"Keyframe {idx} has {count} audio events assigned (should be 1)"


@composite
def multiple_audio_per_keyframe_strategy(draw):
    """
    Generate scenarios where multiple audio events are near the same keyframe.
    
    This specifically tests that even when multiple audio events could match
    the same keyframe, only one is assigned.
    
    Returns:
        Tuple of (visual_keyframes, audio_events)
    """
    # Generate a few visual keyframes
    num_visual = draw(st.integers(min_value=3, max_value=8))
    visual_keyframes = []
    
    for i in range(num_visual):
        trigger_time = i * 500  # Space them 500ms apart
        visual_keyframes.append(create_face_keyframe(trigger_time))
    
    # For each keyframe, generate multiple audio events very close to it
    audio_events = []
    audio_counter = 0
    
    for i in range(num_visual):
        keyframe_time = i * 500
        # Generate 2-6 audio events within 20ms of this keyframe
        num_near = draw(st.integers(min_value=2, max_value=6))
        
        for j in range(num_near):
            offset = draw(st.integers(min_value=-20, max_value=20))
            trigger_time = keyframe_time + offset
            
            audio_events.append(
                AudioEventData(
                    trigger_time=trigger_time,
                    event_names=[f"event{audio_counter}"],
                    wav_file=f"sound{audio_counter}.wav"
                )
            )
            audio_counter += 1
    
    return visual_keyframes, audio_events


@given(test_data=multiple_audio_per_keyframe_strategy())
@settings(max_examples=100, deadline=None)
def test_no_duplicate_audio_assignment_clustered(test_data):
    """
    Feature: vector-animation-audio-sync, Property 22: No duplicate audio assignment
    
    Property: Even when multiple audio events are clustered near the same keyframe,
    that keyframe should receive at most one audio assignment.
    
    Validates: Requirements 6.4
    """
    visual_keyframes, audio_events = test_data
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Property: No keyframe should have multiple audio events
    keyframe_indices = list(keyframe_sound_map.keys())
    
    for idx in set(keyframe_indices):
        count = keyframe_indices.count(idx)
        assert count == 1, \
            f"Keyframe {idx} has {count} audio events (should be 1 max), " \
            f"even with clustered audio events"
    
    # Property: Some audio events must be unmatched (since we have clusters)
    assert len(unmatched) > 0, \
        "With clustered audio events, some should be unmatched"


@composite
def validation_test_strategy(draw):
    """
    Generate test cases for validation method.
    
    Returns:
        Tuple of (visual_keyframes, audio_events)
    """
    # Generate visual keyframes
    num_visual = draw(st.integers(min_value=5, max_value=15))
    visual_keyframes = [create_face_keyframe(i * 150) for i in range(num_visual)]
    
    # Generate audio events
    num_audio = draw(st.integers(min_value=5, max_value=25))
    audio_events = []
    
    for i in range(num_audio):
        trigger_time = draw(st.integers(min_value=0, max_value=num_visual * 150))
        audio_events.append(
            AudioEventData(
                trigger_time=trigger_time,
                event_names=[f"event{i}"],
                wav_file=f"sound{i}.wav"
            )
        )
    
    return visual_keyframes, audio_events


@given(test_data=validation_test_strategy())
@settings(max_examples=100, deadline=None)
def test_no_duplicate_audio_assignment_validation(test_data):
    """
    Feature: vector-animation-audio-sync, Property 22: No duplicate audio assignment
    
    Property: The validation method should confirm that no keyframe has
    multiple audio events assigned.
    
    Validates: Requirements 6.4
    """
    visual_keyframes, audio_events = test_data
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Use the validation method
    is_valid, errors = matcher.validate_assignments(keyframe_sound_map, audio_events)
    
    # Property: Validation should pass (no duplicates)
    assert is_valid, \
        f"Validation failed with errors: {errors}"
    
    # Property: No errors should be reported
    assert len(errors) == 0, \
        f"Validation reported errors when none expected: {errors}"


@composite
def edge_case_strategy(draw):
    """
    Generate edge cases for testing.
    
    Returns:
        Tuple of (visual_keyframes, audio_events)
    """
    case_type = draw(st.sampled_from([
        "no_audio",
        "no_keyframes",
        "single_keyframe",
        "single_audio",
        "all_beyond_tolerance"
    ]))
    
    if case_type == "no_audio":
        num_visual = draw(st.integers(min_value=1, max_value=10))
        visual_keyframes = [create_face_keyframe(i * 100) for i in range(num_visual)]
        audio_events = []
    elif case_type == "no_keyframes":
        visual_keyframes = []
        num_audio = draw(st.integers(min_value=1, max_value=10))
        audio_events = [
            AudioEventData(trigger_time=i * 100, event_names=[f"event{i}"], wav_file=f"sound{i}.wav")
            for i in range(num_audio)
        ]
    elif case_type == "single_keyframe":
        visual_keyframes = [create_face_keyframe(500)]
        num_audio = draw(st.integers(min_value=1, max_value=5))
        audio_events = [
            AudioEventData(trigger_time=500 + i * 10, event_names=[f"event{i}"], wav_file=f"sound{i}.wav")
            for i in range(num_audio)
        ]
    elif case_type == "single_audio":
        num_visual = draw(st.integers(min_value=1, max_value=10))
        visual_keyframes = [create_face_keyframe(i * 100) for i in range(num_visual)]
        audio_events = [AudioEventData(trigger_time=250, event_names=["event1"], wav_file="sound1.wav")]
    else:  # all_beyond_tolerance
        visual_keyframes = [create_face_keyframe(0), create_face_keyframe(1000)]
        audio_events = [AudioEventData(trigger_time=500, event_names=["event1"], wav_file="sound1.wav")]
    
    return visual_keyframes, audio_events


@given(test_data=edge_case_strategy())
@settings(max_examples=100, deadline=None)
def test_no_duplicate_audio_assignment_edge_cases(test_data):
    """
    Feature: vector-animation-audio-sync, Property 22: No duplicate audio assignment
    
    Property: Edge cases should maintain the no-duplicate property.
    
    Validates: Requirements 6.4
    """
    visual_keyframes, audio_events = test_data
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Property: No duplicates even in edge cases
    keyframe_indices = list(keyframe_sound_map.keys())
    unique_indices = set(keyframe_indices)
    
    assert len(keyframe_indices) == len(unique_indices), \
        f"Edge case: Duplicate keyframe assignments found"
    
    # Property: Validation should pass
    is_valid, errors = matcher.validate_assignments(keyframe_sound_map, audio_events)
    assert is_valid, f"Edge case: Validation failed with errors: {errors}"


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running property-based tests for no duplicate audio assignment...\n")
    
    try:
        test_no_duplicate_audio_assignment()
        print("✓ Property test passed: No duplicate audio assignment")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: No duplicate audio assignment - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_no_duplicate_audio_assignment_clustered()
        print("✓ Property test passed: No duplicate audio assignment - clustered")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: No duplicate audio assignment - clustered - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_no_duplicate_audio_assignment_validation()
        print("✓ Property test passed: No duplicate audio assignment - validation")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: No duplicate audio assignment - validation - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_no_duplicate_audio_assignment_edge_cases()
        print("✓ Property test passed: No duplicate audio assignment - edge cases")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: No duplicate audio assignment - edge cases - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print(f"\n{tests_passed} passed, {tests_failed} failed")
    exit(1 if tests_failed > 0 else 0)
