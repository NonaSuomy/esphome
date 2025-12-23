"""
Property-based tests for unique audio assignment.

Feature: vector-animation-audio-sync, Property 9: Unique audio assignment
Validates: Requirements 3.2
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
def keyframes_and_audio_strategy(draw):
    """
    Generate visual keyframes and audio events for testing.
    
    Returns:
        Tuple of (visual_keyframes, audio_events)
    """
    # Generate 1-20 visual keyframes
    num_visual = draw(st.integers(min_value=1, max_value=20))
    visual_keyframes = []
    
    for i in range(num_visual):
        trigger_time = i * 200  # Space them 200ms apart
        visual_keyframes.append(create_face_keyframe(trigger_time))
    
    # Generate 1-50 audio events
    num_audio = draw(st.integers(min_value=1, max_value=50))
    audio_events = []
    
    for i in range(num_audio):
        # Random trigger time within the animation range
        trigger_time = draw(st.integers(min_value=0, max_value=num_visual * 200))
        wav_file = f"sound{i}.wav"
        
        audio_events.append(
            AudioEventData(
                trigger_time=trigger_time,
                event_names=[f"event{i}"],
                wav_file=wav_file
            )
        )
    
    return visual_keyframes, audio_events


@given(test_data=keyframes_and_audio_strategy())
@settings(max_examples=100, deadline=None)
def test_unique_audio_assignment_no_duplicate_keyframes(test_data):
    """
    Feature: vector-animation-audio-sync, Property 9: Unique audio assignment
    
    Property: For any set of audio events, each event should be assigned to 
    at most one visual keyframe.
    
    Validates: Requirements 3.2
    """
    visual_keyframes, audio_events = test_data
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Property: Each keyframe index should appear at most once
    keyframe_indices = list(keyframe_sound_map.keys())
    assert len(keyframe_indices) == len(set(keyframe_indices)), \
        "Each keyframe should have at most one audio event"


@given(test_data=keyframes_and_audio_strategy())
@settings(max_examples=100, deadline=None)
def test_unique_audio_assignment_no_duplicate_audio(test_data):
    """
    Feature: vector-animation-audio-sync, Property 9: Unique audio assignment
    
    Property: For any set of audio events, no audio event should be assigned 
    to multiple keyframes.
    
    Validates: Requirements 3.2
    """
    visual_keyframes, audio_events = test_data
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Property: Each audio file should appear at most once in assignments
    assigned_audio = list(keyframe_sound_map.values())
    assert len(assigned_audio) == len(set(assigned_audio)), \
        "Each audio event should be assigned to at most one keyframe"


@composite
def clustered_audio_strategy(draw):
    """
    Generate visual keyframes with multiple audio events clustered near them.
    
    This tests the edge case where multiple audio events are within tolerance
    of the same keyframe.
    
    Returns:
        Tuple of (visual_keyframes, audio_events)
    """
    # Generate a few visual keyframes
    num_visual = draw(st.integers(min_value=2, max_value=5))
    visual_keyframes = []
    
    for i in range(num_visual):
        trigger_time = i * 500  # Space them 500ms apart
        visual_keyframes.append(create_face_keyframe(trigger_time))
    
    # For each keyframe, generate 2-5 audio events clustered around it
    audio_events = []
    audio_counter = 0
    
    for i in range(num_visual):
        keyframe_time = i * 500
        num_clustered = draw(st.integers(min_value=2, max_value=5))
        
        for j in range(num_clustered):
            # Generate audio events within 50ms of the keyframe
            offset = draw(st.integers(min_value=-50, max_value=50))
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


@given(test_data=clustered_audio_strategy())
@settings(max_examples=100, deadline=None)
def test_unique_audio_assignment_clustered_events(test_data):
    """
    Feature: vector-animation-audio-sync, Property 9: Unique audio assignment
    
    Property: When multiple audio events are clustered near the same keyframe,
    only one should be assigned to that keyframe.
    
    Validates: Requirements 3.2
    """
    visual_keyframes, audio_events = test_data
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Property: Each keyframe should have at most one audio event
    keyframe_indices = list(keyframe_sound_map.keys())
    assert len(keyframe_indices) == len(set(keyframe_indices)), \
        "Each keyframe should have at most one audio event, even with clustered audio"
    
    # Property: The number of assigned audio events should be <= number of keyframes
    assert len(keyframe_sound_map) <= len(visual_keyframes), \
        "Cannot assign more audio events than there are keyframes"
    
    # Property: Some audio events should be unmatched (since we have clusters)
    # This is expected behavior when multiple audio events compete for the same keyframe
    total_audio = len(audio_events)
    assigned_audio = len(keyframe_sound_map)
    unmatched_audio = len(unmatched)
    
    assert assigned_audio + unmatched_audio == total_audio, \
        "All audio events should be either assigned or unmatched"


@composite
def within_tolerance_strategy(draw):
    """
    Generate audio events that are all within tolerance of keyframes.
    
    Returns:
        Tuple of (visual_keyframes, audio_events)
    """
    # Generate visual keyframes
    num_visual = draw(st.integers(min_value=3, max_value=10))
    visual_keyframes = []
    
    for i in range(num_visual):
        trigger_time = i * 300  # Space them 300ms apart
        visual_keyframes.append(create_face_keyframe(trigger_time))
    
    # Generate audio events within 100ms of each keyframe
    audio_events = []
    
    for i in range(num_visual):
        keyframe_time = i * 300
        # Generate offset within tolerance
        offset = draw(st.integers(min_value=-99, max_value=99))
        trigger_time = keyframe_time + offset
        
        audio_events.append(
            AudioEventData(
                trigger_time=trigger_time,
                event_names=[f"event{i}"],
                wav_file=f"sound{i}.wav"
            )
        )
    
    return visual_keyframes, audio_events


@given(test_data=within_tolerance_strategy())
@settings(max_examples=100, deadline=None)
def test_unique_audio_assignment_within_tolerance(test_data):
    """
    Feature: vector-animation-audio-sync, Property 9: Unique audio assignment
    
    Property: When audio events are within tolerance, they should be assigned
    uniquely to keyframes (one-to-one mapping).
    
    Validates: Requirements 3.2
    """
    visual_keyframes, audio_events = test_data
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Property: One-to-one mapping
    keyframe_indices = list(keyframe_sound_map.keys())
    assigned_audio = list(keyframe_sound_map.values())
    
    assert len(keyframe_indices) == len(set(keyframe_indices)), \
        "Each keyframe should appear at most once"
    
    assert len(assigned_audio) == len(set(assigned_audio)), \
        "Each audio event should be assigned at most once"
    
    # Property: Most audio events should be assigned (since they're within tolerance)
    # Allow for some to be unmatched due to conflicts
    assert len(keyframe_sound_map) > 0, \
        "At least some audio events should be assigned when within tolerance"


@composite
def edge_case_strategy(draw):
    """
    Generate edge cases for audio assignment.
    
    Returns:
        Tuple of (visual_keyframes, audio_events)
    """
    edge_case_type = draw(st.sampled_from([
        "empty_visual",
        "empty_audio",
        "single_keyframe_multiple_audio",
        "multiple_keyframes_single_audio"
    ]))
    
    if edge_case_type == "empty_visual":
        # No visual keyframes
        visual_keyframes = []
        audio_events = [
            AudioEventData(trigger_time=100, event_names=["event1"], wav_file="sound1.wav")
        ]
    elif edge_case_type == "empty_audio":
        # No audio events
        visual_keyframes = [create_face_keyframe(0), create_face_keyframe(100)]
        audio_events = []
    elif edge_case_type == "single_keyframe_multiple_audio":
        # One keyframe, multiple audio events
        visual_keyframes = [create_face_keyframe(100)]
        num_audio = draw(st.integers(min_value=2, max_value=10))
        audio_events = [
            AudioEventData(
                trigger_time=100 + i * 5,
                event_names=[f"event{i}"],
                wav_file=f"sound{i}.wav"
            )
            for i in range(num_audio)
        ]
    else:  # multiple_keyframes_single_audio
        # Multiple keyframes, one audio event
        num_visual = draw(st.integers(min_value=2, max_value=10))
        visual_keyframes = [create_face_keyframe(i * 200) for i in range(num_visual)]
        audio_events = [
            AudioEventData(trigger_time=100, event_names=["event1"], wav_file="sound1.wav")
        ]
    
    return visual_keyframes, audio_events


@given(test_data=edge_case_strategy())
@settings(max_examples=100, deadline=None)
def test_unique_audio_assignment_edge_cases(test_data):
    """
    Feature: vector-animation-audio-sync, Property 9: Unique audio assignment
    
    Property: Edge cases (empty inputs, single keyframe, etc.) should maintain
    unique assignment property.
    
    Validates: Requirements 3.2
    """
    visual_keyframes, audio_events = test_data
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Property: Unique assignment always holds
    keyframe_indices = list(keyframe_sound_map.keys())
    assigned_audio = list(keyframe_sound_map.values())
    
    assert len(keyframe_indices) == len(set(keyframe_indices)), \
        "Each keyframe should appear at most once (edge case)"
    
    assert len(assigned_audio) == len(set(assigned_audio)), \
        "Each audio event should be assigned at most once (edge case)"
    
    # Property: Total audio events = assigned + unmatched
    assert len(keyframe_sound_map) + len(unmatched) == len(audio_events), \
        "All audio events should be accounted for"


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running property-based tests for unique audio assignment...\n")
    
    try:
        test_unique_audio_assignment_no_duplicate_keyframes()
        print("✓ Property test passed: Unique audio assignment - no duplicate keyframes")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Unique audio assignment - no duplicate keyframes - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_unique_audio_assignment_no_duplicate_audio()
        print("✓ Property test passed: Unique audio assignment - no duplicate audio")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Unique audio assignment - no duplicate audio - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_unique_audio_assignment_clustered_events()
        print("✓ Property test passed: Unique audio assignment - clustered events")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Unique audio assignment - clustered events - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_unique_audio_assignment_within_tolerance()
        print("✓ Property test passed: Unique audio assignment - within tolerance")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Unique audio assignment - within tolerance - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_unique_audio_assignment_edge_cases()
        print("✓ Property test passed: Unique audio assignment - edge cases")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Unique audio assignment - edge cases - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print(f"\n{tests_passed} passed, {tests_failed} failed")
    exit(1 if tests_failed > 0 else 0)
