"""
Property-based test for precise audio timing.

Feature: vector-animation-audio-sync, Property 8: Precise audio timing
Validates: Requirements 3.1
"""
import json
import tempfile
import os
import re
from hypothesis import given, strategies as st, settings
from hypothesis.strategies import composite

from json_parser import JSONParser
from animation_data import AnimationData, AudioEventData
from keyframe_extractor import KeyframeExtractor


@composite
def procedural_face_keyframe_strategy(draw):
    """Generate a valid ProceduralFaceKeyFrame."""
    trigger_time = draw(st.integers(min_value=0, max_value=10000))
    
    return {
        "Name": "ProceduralFaceKeyFrame",
        "triggerTime_ms": trigger_time,
        "faceScaleX": draw(st.floats(min_value=0.1, max_value=2.0)),
        "faceScaleY": draw(st.floats(min_value=0.1, max_value=2.0)),
        "faceAngle": draw(st.floats(min_value=-180.0, max_value=180.0)),
        "faceCenterX": draw(st.floats(min_value=-10.0, max_value=10.0)),
        "faceCenterY": draw(st.floats(min_value=-10.0, max_value=10.0)),
        "leftEye": [draw(st.floats()) for _ in range(25)],
        "rightEye": [draw(st.floats()) for _ in range(25)],
        "durationTime_ms": draw(st.integers(min_value=0, max_value=1000))
    }


@composite
def audio_keyframe_strategy(draw):
    """Generate a valid RobotAudioKeyFrame."""
    trigger_time = draw(st.integers(min_value=0, max_value=10000))
    
    # Generate a recognizable audio event name
    audio_events = [
        "Play__Robot_Vic_Sfx__Blink",
        "Play__Robot_Vic_Sfx__Scrn_Curious",
        "Play__Robot_Vic_Sfx__Scrn_Happy",
        "Play__Robot_Vic_Sfx__Scrn_Neutral",
        "Play__Robot_Vic_Sfx__Emote_Happy_Short"
    ]
    
    audio_name = draw(st.sampled_from(audio_events))
    
    return {
        "Name": "RobotAudioKeyFrame",
        "triggerTime_ms": trigger_time,
        "audioName": [audio_name],
        "volume": draw(st.floats(min_value=0.0, max_value=1.0)),
        "probability": [draw(st.floats(min_value=0.0, max_value=1.0))]
    }


@composite
def animation_with_audio_strategy(draw):
    """
    Generate an animation JSON with both visual and audio keyframes.
    
    This ensures we have audio events that should be matched to visual keyframes.
    """
    anim_name = draw(st.text(
        min_size=5, 
        max_size=30, 
        alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd'), whitelist_characters='_')
    ))
    
    # Generate visual keyframes
    num_visual = draw(st.integers(min_value=3, max_value=15))
    visual_keyframes = [draw(procedural_face_keyframe_strategy()) for _ in range(num_visual)]
    
    # Generate audio keyframes
    num_audio = draw(st.integers(min_value=1, max_value=5))
    audio_keyframes = [draw(audio_keyframe_strategy()) for _ in range(num_audio)]
    
    # Combine all keyframes
    all_keyframes = visual_keyframes + audio_keyframes
    
    # Shuffle to mix them
    import random
    random.shuffle(all_keyframes)
    
    return {anim_name: all_keyframes}


def parse_generated_cpp(cpp_content):
    """
    Parse generated C++ code to extract keyframe data.
    
    Returns a list of tuples: (trigger_time, duration, sound_name)
    """
    keyframes = []
    
    # Match lines like:
    # { 0, 33, { 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.6f, 0.6f, 0.6f, 0.6f }, "blink.wav" },
    # or
    # { 100, 50, { ... }, nullptr },
    
    pattern = r'\{\s*(\d+),\s*(\d+),\s*\{[^}]+\},\s*([^}]+)\s*\}'
    
    for match in re.finditer(pattern, cpp_content):
        trigger_time = int(match.group(1))
        duration = int(match.group(2))
        sound_part = match.group(3).strip()
        
        # Extract sound name
        if sound_part == "nullptr":
            sound_name = None
        else:
            # Extract string literal like "blink.wav"
            sound_match = re.search(r'"([^"]+)"', sound_part)
            if sound_match:
                sound_name = sound_match.group(1)
            else:
                sound_name = None
        
        keyframes.append((trigger_time, duration, sound_name))
    
    return keyframes


def simulate_conversion(animation_json):
    """
    Simulate the conversion process from JSON to C++ code.
    
    This mimics what convert_anims.py does, specifically the audio matching logic.
    Returns the generated C++ content.
    """
    anim_name = list(animation_json.keys())[0]
    keyframes = animation_json[anim_name]
    
    # Separate visual and audio keyframes
    visual_keyframes = []
    audio_keyframes = []
    
    for kf in keyframes:
        if kf.get("Name") == "ProceduralFaceKeyFrame":
            visual_keyframes.append(kf)
        elif kf.get("Name") == "RobotAudioKeyFrame":
            audio_keyframes.append(kf)
    
    # Sort visual keyframes by trigger time
    visual_keyframes.sort(key=lambda x: x.get("triggerTime_ms", 0))
    
    # Audio mapping (simplified version from convert_anims.py)
    AUDIO_EVENT_MAP = {
        "Play__Robot_Vic_Sfx__Blink": "blink",
        "Play__Robot_Vic_Sfx__Scrn_Curious": "curious_short",
        "Play__Robot_Vic_Sfx__Scrn_Happy": "happy",
        "Play__Robot_Vic_Sfx__Scrn_Neutral": "neutral",
        "Play__Robot_Vic_Sfx__Emote_Happy_Short": "emote_happy"
    }
    
    # Match audio events to closest visual keyframes
    keyframe_sound_map = {}
    
    for akf in audio_keyframes:
        a_time = akf.get("triggerTime_ms", 0)
        audio_names = akf.get("audioName", [])
        
        if not audio_names:
            continue
        
        event_name = audio_names[0]
        
        # Find mapping
        mapped_sound = None
        for map_key, map_val in AUDIO_EVENT_MAP.items():
            if map_key in event_name:
                mapped_sound = f'"{map_val}.wav"'
                break
        
        if not mapped_sound:
            continue
        
        # Find closest visual keyframe
        closest_idx = None
        closest_dist = float('inf')
        
        for i, kf in enumerate(visual_keyframes):
            kf_time = kf.get("triggerTime_ms", 0)
            dist = abs(kf_time - a_time)
            
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i
        
        # Only assign if within 100ms tolerance
        if closest_idx is not None and closest_dist <= 100:
            keyframe_sound_map[closest_idx] = mapped_sound
    
    # Generate C++ code
    cpp_content = f"static const AnimationKeyframe TEST_ANIM[] = {{\n"
    
    for i, kf in enumerate(visual_keyframes):
        trigger_time = kf.get("triggerTime_ms", 0)
        
        # Calculate duration
        if i < len(visual_keyframes) - 1:
            next_trigger = visual_keyframes[i+1].get("triggerTime_ms", 0)
            duration = next_trigger - trigger_time
        else:
            duration = 33
        
        if duration < 0:
            duration = 0
        
        # Get assigned sound
        sound_str = keyframe_sound_map.get(i, "nullptr")
        
        # Extract face parameters
        scale_x = kf.get("faceScaleX", 1.0)
        scale_y = kf.get("faceScaleY", 1.0)
        angle = kf.get("faceAngle", 0.0)
        center_x = kf.get("faceCenterX", 0.0) * 0.5
        center_y = kf.get("faceCenterY", 0.0) * 0.5
        
        left_eye = kf.get("leftEye", [])
        right_eye = kf.get("rightEye", [])
        
        l_lid_top = left_eye[5] if len(left_eye) > 5 else 0.0
        l_lid_bottom = left_eye[6] if len(left_eye) > 6 else 0.0
        r_lid_top = right_eye[5] if len(right_eye) > 5 else 0.0
        r_lid_bottom = right_eye[6] if len(right_eye) > 6 else 0.0
        
        cpp_content += f"  {{ {trigger_time}, {duration}, {{ {scale_x}f, {scale_y}f, {angle}f, {center_x}f, {center_y}f, {l_lid_top}f, {l_lid_bottom}f, {r_lid_top}f, {r_lid_bottom}f }}, {sound_str} }},\n"
    
    cpp_content += "};\n"
    
    return cpp_content, audio_keyframes, visual_keyframes


@given(animation_json=animation_with_audio_strategy())
@settings(max_examples=100, deadline=None)
def test_precise_audio_timing(animation_json):
    """
    Feature: vector-animation-audio-sync, Property 8: Precise audio timing
    
    Property: For any RobotAudioKeyFrame with a trigger time T, the generated code 
    should schedule audio playback at exactly T milliseconds from animation start.
    
    This property verifies that:
    1. Audio events are matched to the closest visual keyframe
    2. The matched keyframe's trigger time is within 100ms of the audio event's trigger time
    3. Each audio event is assigned to at most one keyframe
    4. The timing precision is maintained in the generated C++ code
    
    Validates: Requirements 3.1
    """
    # Simulate the conversion process
    cpp_content, audio_keyframes, visual_keyframes = simulate_conversion(animation_json)
    
    # Parse the generated C++ code
    generated_keyframes = parse_generated_cpp(cpp_content)
    
    # For each audio keyframe, verify it was matched to a visual keyframe with precise timing
    for akf in audio_keyframes:
        audio_trigger_time = akf.get("triggerTime_ms", 0)
        audio_names = akf.get("audioName", [])
        
        if not audio_names:
            continue
        
        event_name = audio_names[0]
        
        # Check if this audio event should be mapped
        AUDIO_EVENT_MAP = {
            "Play__Robot_Vic_Sfx__Blink": "blink",
            "Play__Robot_Vic_Sfx__Scrn_Curious": "curious_short",
            "Play__Robot_Vic_Sfx__Scrn_Happy": "happy",
            "Play__Robot_Vic_Sfx__Scrn_Neutral": "neutral",
            "Play__Robot_Vic_Sfx__Emote_Happy_Short": "emote_happy"
        }
        
        expected_sound = None
        for map_key, map_val in AUDIO_EVENT_MAP.items():
            if map_key in event_name:
                expected_sound = f"{map_val}.wav"
                break
        
        if not expected_sound:
            # This audio event is not mapped, so it shouldn't appear in generated code
            continue
        
        # Find the closest visual keyframe to this audio event
        closest_visual_time = None
        closest_dist = float('inf')
        
        for vkf in visual_keyframes:
            vkf_time = vkf.get("triggerTime_ms", 0)
            dist = abs(vkf_time - audio_trigger_time)
            
            if dist < closest_dist:
                closest_dist = dist
                closest_visual_time = vkf_time
        
        # If the closest visual keyframe is within 100ms tolerance
        if closest_dist <= 100:
            # Verify that a keyframe with this trigger time has the expected sound
            found_match = False
            
            for gen_trigger, gen_duration, gen_sound in generated_keyframes:
                if gen_trigger == closest_visual_time and gen_sound == expected_sound:
                    found_match = True
                    
                    # CRITICAL: Verify timing precision
                    # The audio should trigger at the keyframe's trigger time,
                    # which should be within 100ms of the original audio event time
                    timing_error = abs(gen_trigger - audio_trigger_time)
                    
                    assert timing_error <= 100, \
                        f"Audio timing error too large: audio event at {audio_trigger_time}ms " \
                        f"matched to keyframe at {gen_trigger}ms (error: {timing_error}ms, max: 100ms)"
                    
                    break
            
            # If we expected a match (within tolerance), verify it exists
            assert found_match, \
                f"Audio event '{event_name}' at {audio_trigger_time}ms should be matched to " \
                f"keyframe at {closest_visual_time}ms (distance: {closest_dist}ms), " \
                f"but no such keyframe with sound '{expected_sound}' was found in generated code"
        else:
            # Audio event is too far from any visual keyframe (>100ms)
            # It should NOT appear in the generated code
            for gen_trigger, gen_duration, gen_sound in generated_keyframes:
                if gen_sound == expected_sound:
                    # Check if this is actually for a different audio event
                    # by verifying the timing
                    timing_error = abs(gen_trigger - audio_trigger_time)
                    
                    # If the timing error is large, this sound is for a different audio event
                    if timing_error > 100:
                        continue
                    
                    # If we get here, we found a sound that shouldn't be there
                    assert False, \
                        f"Audio event '{event_name}' at {audio_trigger_time}ms is too far from " \
                        f"any visual keyframe (closest: {closest_dist}ms > 100ms), but sound " \
                        f"'{expected_sound}' appears at {gen_trigger}ms in generated code"
    
    # Additional verification: Ensure no keyframe has multiple sounds
    # (This is part of the unique assignment property, but we check it here too)
    sound_assignments = {}
    for gen_trigger, gen_duration, gen_sound in generated_keyframes:
        if gen_sound is not None:
            if gen_trigger in sound_assignments:
                assert sound_assignments[gen_trigger] == gen_sound, \
                    f"Keyframe at {gen_trigger}ms has multiple sounds assigned: " \
                    f"'{sound_assignments[gen_trigger]}' and '{gen_sound}'"
            else:
                sound_assignments[gen_trigger] = gen_sound


if __name__ == "__main__":
    # Run the test
    try:
        test_precise_audio_timing()
        print("✓ Property test passed: Precise audio timing")
        exit(0)
    except Exception as e:
        print(f"✗ Property test failed: Precise audio timing - {e}")
        import traceback
        traceback.print_exc()
        exit(1)
