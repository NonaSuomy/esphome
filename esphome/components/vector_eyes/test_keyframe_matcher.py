"""
Unit tests for keyframe-audio matching algorithm.

Tests the basic functionality of the KeyframeMatcher class.
"""
import pytest
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


def test_basic_matching_within_tolerance():
    """Test that audio events within tolerance are matched to closest keyframe."""
    # Create visual keyframes at 0ms, 100ms, 200ms
    visual_keyframes = [
        create_face_keyframe(0),
        create_face_keyframe(100),
        create_face_keyframe(200),
    ]
    
    # Create audio events near the keyframes
    audio_events = [
        AudioEventData(trigger_time=10, event_names=["sound1"], wav_file="sound1.wav"),
        AudioEventData(trigger_time=105, event_names=["sound2"], wav_file="sound2.wav"),
    ]
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Audio at 10ms should match keyframe at 0ms (distance: 10ms)
    assert 0 in keyframe_sound_map
    assert keyframe_sound_map[0] == "sound1.wav"
    
    # Audio at 105ms should match keyframe at 100ms (distance: 5ms)
    assert 1 in keyframe_sound_map
    assert keyframe_sound_map[1] == "sound2.wav"
    
    # No unmatched events
    assert len(unmatched) == 0


def test_matching_beyond_tolerance():
    """Test that audio events beyond tolerance are not matched."""
    # Create visual keyframes at 0ms, 500ms
    visual_keyframes = [
        create_face_keyframe(0),
        create_face_keyframe(500),
    ]
    
    # Create audio event far from any keyframe
    audio_events = [
        AudioEventData(trigger_time=250, event_names=["sound1"], wav_file="sound1.wav"),
    ]
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Audio at 250ms is 250ms from keyframe at 0ms and 250ms from keyframe at 500ms
    # Both are beyond 100ms tolerance, so it should not be matched
    assert len(keyframe_sound_map) == 0
    assert len(unmatched) == 1
    assert unmatched[0].trigger_time == 250


def test_one_to_one_assignment():
    """Test that each keyframe gets at most one audio event."""
    # Create one visual keyframe
    visual_keyframes = [
        create_face_keyframe(100),
    ]
    
    # Create two audio events both close to the same keyframe
    audio_events = [
        AudioEventData(trigger_time=95, event_names=["sound1"], wav_file="sound1.wav"),
        AudioEventData(trigger_time=105, event_names=["sound2"], wav_file="sound2.wav"),
    ]
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Only one audio event should be assigned to the keyframe
    assert len(keyframe_sound_map) == 1
    assert 0 in keyframe_sound_map
    
    # The first audio event (95ms) is closer (5ms away) than the second (5ms away too)
    # But since they're equally close, the first one processed wins
    assert keyframe_sound_map[0] == "sound1.wav"
    
    # The second audio event should be unmatched
    assert len(unmatched) == 1
    assert unmatched[0].wav_file == "sound2.wav"


def test_closest_keyframe_selection():
    """Test that audio events are matched to the closest keyframe."""
    # Create visual keyframes at 0ms, 100ms, 200ms
    visual_keyframes = [
        create_face_keyframe(0),
        create_face_keyframe(100),
        create_face_keyframe(200),
    ]
    
    # Create audio event at 110ms (closer to 100ms than to 200ms)
    audio_events = [
        AudioEventData(trigger_time=110, event_names=["sound1"], wav_file="sound1.wav"),
    ]
    
    matcher = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, audio_events)
    
    # Audio at 110ms should match keyframe at 100ms (distance: 10ms)
    # not keyframe at 200ms (distance: 90ms)
    assert 1 in keyframe_sound_map
    assert keyframe_sound_map[1] == "sound1.wav"
    assert 2 not in keyframe_sound_map


def test_empty_inputs():
    """Test handling of empty input lists."""
    matcher = KeyframeMatcher(tolerance_ms=100)
    
    # Empty visual keyframes
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes([], [])
    assert len(keyframe_sound_map) == 0
    assert len(unmatched) == 0
    
    # Empty audio events
    visual_keyframes = [create_face_keyframe(0)]
    keyframe_sound_map, unmatched = matcher.match_audio_to_keyframes(visual_keyframes, [])
    assert len(keyframe_sound_map) == 0
    assert len(unmatched) == 0


def test_custom_tolerance():
    """Test that custom tolerance values work correctly."""
    # Create visual keyframes
    visual_keyframes = [
        create_face_keyframe(0),
        create_face_keyframe(500),
    ]
    
    # Create audio event at 150ms
    audio_events = [
        AudioEventData(trigger_time=150, event_names=["sound1"], wav_file="sound1.wav"),
    ]
    
    # With 100ms tolerance, should not match (150ms away from nearest keyframe)
    matcher_100 = KeyframeMatcher(tolerance_ms=100)
    keyframe_sound_map, unmatched = matcher_100.match_audio_to_keyframes(visual_keyframes, audio_events)
    assert len(keyframe_sound_map) == 0
    assert len(unmatched) == 1
    
    # With 200ms tolerance, should match
    matcher_200 = KeyframeMatcher(tolerance_ms=200)
    keyframe_sound_map, unmatched = matcher_200.match_audio_to_keyframes(visual_keyframes, audio_events)
    assert len(keyframe_sound_map) == 1
    assert 0 in keyframe_sound_map
    assert len(unmatched) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])



def test_validate_assignments_valid():
    """Test validation of valid audio assignments."""
    matcher = KeyframeMatcher(tolerance_ms=100)
    
    # Create valid assignments
    keyframe_sound_map = {
        0: "sound1.wav",
        1: "sound2.wav",
        2: "sound3.wav"
    }
    
    audio_events = [
        AudioEventData(trigger_time=0, event_names=["sound1"], wav_file="sound1.wav"),
        AudioEventData(trigger_time=100, event_names=["sound2"], wav_file="sound2.wav"),
        AudioEventData(trigger_time=200, event_names=["sound3"], wav_file="sound3.wav"),
    ]
    
    is_valid, errors = matcher.validate_assignments(keyframe_sound_map, audio_events)
    
    assert is_valid
    assert len(errors) == 0


def test_validate_assignments_no_duplicates():
    """Test validation catches duplicate audio assignments."""
    matcher = KeyframeMatcher(tolerance_ms=100)
    
    # Create invalid assignments (same sound assigned to multiple keyframes)
    keyframe_sound_map = {
        0: "sound1.wav",
        1: "sound1.wav",  # Duplicate!
    }
    
    audio_events = [
        AudioEventData(trigger_time=0, event_names=["sound1"], wav_file="sound1.wav"),
    ]
    
    is_valid, errors = matcher.validate_assignments(keyframe_sound_map, audio_events)
    
    assert not is_valid
    assert len(errors) > 0
    assert any("multiple keyframes" in err for err in errors)


def test_validate_assignments_invalid_sounds():
    """Test validation catches sounds not in audio events."""
    matcher = KeyframeMatcher(tolerance_ms=100)
    
    # Create assignments with sound not in audio events
    keyframe_sound_map = {
        0: "unknown_sound.wav",  # Not in audio events!
    }
    
    audio_events = [
        AudioEventData(trigger_time=0, event_names=["sound1"], wav_file="sound1.wav"),
    ]
    
    is_valid, errors = matcher.validate_assignments(keyframe_sound_map, audio_events)
    
    assert not is_valid
    assert len(errors) > 0
    assert any("not in audio events" in err for err in errors)


def test_validate_assignments_empty():
    """Test validation of empty assignments."""
    matcher = KeyframeMatcher(tolerance_ms=100)
    
    # Empty assignments should be valid
    is_valid, errors = matcher.validate_assignments({}, [])
    
    assert is_valid
    assert len(errors) == 0
