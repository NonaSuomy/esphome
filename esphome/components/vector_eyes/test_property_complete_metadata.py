"""
Property-based tests for complete metadata generation.

Feature: vector-animation-audio-sync, Property 13: Complete metadata generation
Validates: Requirements 4.4
"""
from hypothesis import given, strategies as st, settings
from hypothesis.strategies import composite

from code_generator import CodeGenerator, AnimationMetadata
from keyframe_extractor import ProceduralFaceData


@composite
def procedural_face_keyframe_strategy(draw):
    """Generate a valid ProceduralFaceData keyframe."""
    return ProceduralFaceData(
        trigger_time=draw(st.integers(min_value=0, max_value=10000)),
        duration=draw(st.integers(min_value=0, max_value=1000)),
        scale_x=draw(st.floats(min_value=0.1, max_value=2.0)),
        scale_y=draw(st.floats(min_value=0.1, max_value=2.0)),
        angle=draw(st.floats(min_value=-180.0, max_value=180.0)),
        center_x=draw(st.floats(min_value=-10.0, max_value=10.0)),
        center_y=draw(st.floats(min_value=-10.0, max_value=10.0)),
        left_lid_top=draw(st.floats(min_value=0.0, max_value=1.0)),
        left_lid_bottom=draw(st.floats(min_value=0.0, max_value=1.0)),
        right_lid_top=draw(st.floats(min_value=0.0, max_value=1.0)),
        right_lid_bottom=draw(st.floats(min_value=0.0, max_value=1.0))
    )


@composite
def animation_data_strategy(draw):
    """
    Generate animation data with keyframes and audio mappings.
    
    Returns:
        Tuple of (animation_name, keyframes, keyframe_sound_map, duration_ms)
    """
    # Generate animation name
    anim_name = draw(st.text(
        min_size=5, 
        max_size=30, 
        alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd'), whitelist_characters='_')
    ))
    
    # Generate keyframes
    num_keyframes = draw(st.integers(min_value=1, max_value=50))
    keyframes = [draw(procedural_face_keyframe_strategy()) for _ in range(num_keyframes)]
    
    # Sort keyframes by trigger time
    keyframes.sort(key=lambda kf: kf.trigger_time)
    
    # Calculate duration from keyframes
    if keyframes:
        duration_ms = max(kf.trigger_time + kf.duration for kf in keyframes)
    else:
        duration_ms = 1000
    
    # Generate audio mappings for some keyframes
    audio_file_pool = [
        "blink.wav",
        "curious.wav",
        "happy.wav",
        "sad.wav",
        "excited.wav",
        "confused.wav"
    ]
    
    keyframe_sound_map = {}
    num_audio_keyframes = draw(st.integers(min_value=0, max_value=min(num_keyframes, 20)))
    
    # Assign audio files to random keyframes (no duplicates)
    available_indices = list(range(num_keyframes))
    for _ in range(num_audio_keyframes):
        if not available_indices:
            break
        kf_idx = draw(st.sampled_from(available_indices))
        available_indices.remove(kf_idx)
        audio_file = draw(st.sampled_from(audio_file_pool))
        keyframe_sound_map[kf_idx] = audio_file
    
    return (anim_name, keyframes, keyframe_sound_map, duration_ms)


@given(animation_data=animation_data_strategy())
@settings(max_examples=100, deadline=None)
def test_complete_metadata_generation(animation_data):
    """
    Feature: vector-animation-audio-sync, Property 13: Complete metadata generation
    
    Property: For any converted animation, the generated metadata should include 
    frame count, total duration, and audio event count matching the original data.
    
    Validates: Requirements 4.4
    """
    anim_name, keyframes, keyframe_sound_map, duration_ms = animation_data
    
    # Create a fresh code generator
    gen = CodeGenerator()
    
    # Generate the animation
    cpp_code, metadata = gen.generate_keyframe_array(
        anim_name,
        keyframes,
        keyframe_sound_map,
        duration_ms
    )
    
    # Property 1: Metadata should include correct frame count
    assert metadata.frame_count == len(keyframes), \
        f"Frame count mismatch: expected {len(keyframes)}, got {metadata.frame_count}"
    
    # Property 2: Metadata should include correct duration
    assert metadata.duration_ms == duration_ms, \
        f"Duration mismatch: expected {duration_ms}ms, got {metadata.duration_ms}ms"
    
    # Property 3: Metadata should include correct audio event count
    expected_audio_count = len(keyframe_sound_map)
    assert metadata.audio_event_count == expected_audio_count, \
        f"Audio event count mismatch: expected {expected_audio_count}, got {metadata.audio_event_count}"
    
    # Property 4: Metadata should include the animation name
    assert metadata.name == anim_name, \
        f"Animation name mismatch: expected '{anim_name}', got '{metadata.name}'"
    
    # Property 5: Metadata should include a valid C identifier
    assert metadata.c_identifier, \
        "C identifier should not be empty"
    
    # Property 6: C identifier should be a valid C++ identifier
    # (alphanumeric and underscores, not starting with digit)
    c_id = metadata.c_identifier
    assert c_id.replace('_', '').isalnum(), \
        f"C identifier '{c_id}' contains invalid characters"
    assert not c_id[0].isdigit(), \
        f"C identifier '{c_id}' starts with a digit"


@given(animation_data=animation_data_strategy())
@settings(max_examples=100, deadline=None)
def test_metadata_in_generated_code(animation_data):
    """
    Feature: vector-animation-audio-sync, Property 13: Complete metadata generation
    
    Property: The generated C++ code should include comments with the complete
    metadata (frame count, duration, audio event count).
    
    Validates: Requirements 4.4
    """
    anim_name, keyframes, keyframe_sound_map, duration_ms = animation_data
    
    # Create a fresh code generator
    gen = CodeGenerator()
    
    # Generate the animation
    cpp_code, metadata = gen.generate_keyframe_array(
        anim_name,
        keyframes,
        keyframe_sound_map,
        duration_ms
    )
    
    # Property 1: Generated code should contain frame count comment
    expected_frame_comment = f"Frames: {len(keyframes)}"
    assert expected_frame_comment in cpp_code, \
        f"Generated code should contain '{expected_frame_comment}'"
    
    # Property 2: Generated code should contain duration comment
    expected_duration_comment = f"Duration: {duration_ms}ms"
    assert expected_duration_comment in cpp_code, \
        f"Generated code should contain '{expected_duration_comment}'"
    
    # Property 3: Generated code should contain audio event count comment
    expected_audio_comment = f"Audio Events: {len(keyframe_sound_map)}"
    assert expected_audio_comment in cpp_code, \
        f"Generated code should contain '{expected_audio_comment}'"
    
    # Property 4: Generated code should contain animation name comment
    expected_name_comment = f"Animation: {anim_name}"
    assert expected_name_comment in cpp_code, \
        f"Generated code should contain '{expected_name_comment}'"


@composite
def multiple_animations_strategy(draw):
    """
    Generate multiple animations with varying metadata.
    
    Returns:
        List of (animation_name, keyframes, keyframe_sound_map, duration_ms) tuples
    """
    num_animations = draw(st.integers(min_value=2, max_value=10))
    animations = []
    
    for i in range(num_animations):
        animation = draw(animation_data_strategy())
        animations.append(animation)
    
    return animations


@given(animations=multiple_animations_strategy())
@settings(max_examples=50, deadline=None)
def test_metadata_tracking_across_animations(animations):
    """
    Feature: vector-animation-audio-sync, Property 13: Complete metadata generation
    
    Property: When generating multiple animations, the code generator should
    track metadata for each animation separately and maintain accurate counts.
    
    Validates: Requirements 4.4
    """
    # Create a fresh code generator
    gen = CodeGenerator()
    
    # Track expected metadata
    expected_metadata = []
    
    # Generate code for each animation
    for anim_name, keyframes, keyframe_sound_map, duration_ms in animations:
        cpp_code, metadata = gen.generate_keyframe_array(
            anim_name,
            keyframes,
            keyframe_sound_map,
            duration_ms
        )
        
        expected_metadata.append({
            'name': anim_name,
            'frame_count': len(keyframes),
            'duration_ms': duration_ms,
            'audio_event_count': len(keyframe_sound_map)
        })
    
    # Property 1: Generator should track metadata for all animations
    assert len(gen.animation_metadata) == len(animations), \
        f"Should track metadata for {len(animations)} animations, got {len(gen.animation_metadata)}"
    
    # Property 2: Each animation's metadata should be accurate
    for i, metadata in enumerate(gen.animation_metadata):
        expected = expected_metadata[i]
        
        assert metadata.name == expected['name'], \
            f"Animation {i}: name mismatch"
        assert metadata.frame_count == expected['frame_count'], \
            f"Animation {i}: frame count mismatch"
        assert metadata.duration_ms == expected['duration_ms'], \
            f"Animation {i}: duration mismatch"
        assert metadata.audio_event_count == expected['audio_event_count'], \
            f"Animation {i}: audio event count mismatch"


@given(animations=multiple_animations_strategy())
@settings(max_examples=50, deadline=None)
def test_lookup_table_includes_metadata(animations):
    """
    Feature: vector-animation-audio-sync, Property 13: Complete metadata generation
    
    Property: The generated lookup table should include complete metadata for
    each animation (name, frame count, duration, audio event count).
    
    Validates: Requirements 4.4
    """
    # Create a fresh code generator
    gen = CodeGenerator()
    
    # Generate code for each animation
    for anim_name, keyframes, keyframe_sound_map, duration_ms in animations:
        cpp_code, metadata = gen.generate_keyframe_array(
            anim_name,
            keyframes,
            keyframe_sound_map,
            duration_ms
        )
    
    # Generate the lookup table
    lookup_table = gen.generate_lookup_table()
    
    # Property 1: Lookup table should contain entries for all animations
    for metadata in gen.animation_metadata:
        # Check that the animation name appears in the lookup table
        assert f'"{metadata.name}"' in lookup_table, \
            f"Lookup table should contain animation '{metadata.name}'"
        
        # Check that the C identifier appears
        assert metadata.c_identifier in lookup_table, \
            f"Lookup table should contain C identifier '{metadata.c_identifier}'"
        
        # Check that frame count appears
        assert str(metadata.frame_count) in lookup_table, \
            f"Lookup table should contain frame count {metadata.frame_count}"
        
        # Check that duration appears
        assert str(metadata.duration_ms) in lookup_table, \
            f"Lookup table should contain duration {metadata.duration_ms}"
        
        # Check that audio event count appears
        assert str(metadata.audio_event_count) in lookup_table, \
            f"Lookup table should contain audio event count {metadata.audio_event_count}"


@composite
def animation_with_no_audio_strategy(draw):
    """
    Generate animation data with no audio events.
    
    Returns:
        Tuple of (animation_name, keyframes, keyframe_sound_map, duration_ms)
    """
    # Generate animation name
    anim_name = draw(st.text(
        min_size=5, 
        max_size=30, 
        alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd'), whitelist_characters='_')
    ))
    
    # Generate keyframes
    num_keyframes = draw(st.integers(min_value=1, max_value=30))
    keyframes = [draw(procedural_face_keyframe_strategy()) for _ in range(num_keyframes)]
    keyframes.sort(key=lambda kf: kf.trigger_time)
    
    # Calculate duration
    duration_ms = max(kf.trigger_time + kf.duration for kf in keyframes) if keyframes else 1000
    
    # No audio mappings
    keyframe_sound_map = {}
    
    return (anim_name, keyframes, keyframe_sound_map, duration_ms)


@given(animation_data=animation_with_no_audio_strategy())
@settings(max_examples=50, deadline=None)
def test_metadata_with_no_audio(animation_data):
    """
    Feature: vector-animation-audio-sync, Property 13: Complete metadata generation
    
    Property: For animations with no audio events, the metadata should correctly
    indicate zero audio events while still including accurate frame count and duration.
    
    Validates: Requirements 4.4
    """
    anim_name, keyframes, keyframe_sound_map, duration_ms = animation_data
    
    # Create a fresh code generator
    gen = CodeGenerator()
    
    # Generate the animation
    cpp_code, metadata = gen.generate_keyframe_array(
        anim_name,
        keyframes,
        keyframe_sound_map,
        duration_ms
    )
    
    # Property 1: Audio event count should be zero
    assert metadata.audio_event_count == 0, \
        f"Expected 0 audio events, got {metadata.audio_event_count}"
    
    # Property 2: Frame count should still be accurate
    assert metadata.frame_count == len(keyframes), \
        f"Frame count should be {len(keyframes)}, got {metadata.frame_count}"
    
    # Property 3: Duration should still be accurate
    assert metadata.duration_ms == duration_ms, \
        f"Duration should be {duration_ms}ms, got {metadata.duration_ms}ms"
    
    # Property 4: Generated code should indicate zero audio events
    assert "Audio Events: 0" in cpp_code, \
        "Generated code should indicate 0 audio events"


@composite
def animation_with_all_audio_strategy(draw):
    """
    Generate animation where every keyframe has audio.
    
    Returns:
        Tuple of (animation_name, keyframes, keyframe_sound_map, duration_ms)
    """
    # Generate animation name
    anim_name = draw(st.text(
        min_size=5, 
        max_size=30, 
        alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd'), whitelist_characters='_')
    ))
    
    # Generate keyframes
    num_keyframes = draw(st.integers(min_value=1, max_value=20))
    keyframes = [draw(procedural_face_keyframe_strategy()) for _ in range(num_keyframes)]
    keyframes.sort(key=lambda kf: kf.trigger_time)
    
    # Calculate duration
    duration_ms = max(kf.trigger_time + kf.duration for kf in keyframes) if keyframes else 1000
    
    # Audio file pool
    audio_file_pool = ["sound1.wav", "sound2.wav", "sound3.wav", "sound4.wav"]
    
    # Assign audio to every keyframe
    keyframe_sound_map = {}
    for i in range(num_keyframes):
        audio_file = draw(st.sampled_from(audio_file_pool))
        keyframe_sound_map[i] = audio_file
    
    return (anim_name, keyframes, keyframe_sound_map, duration_ms)


@given(animation_data=animation_with_all_audio_strategy())
@settings(max_examples=50, deadline=None)
def test_metadata_with_all_audio(animation_data):
    """
    Feature: vector-animation-audio-sync, Property 13: Complete metadata generation
    
    Property: For animations where every keyframe has audio, the metadata should
    correctly indicate that audio event count equals frame count.
    
    Validates: Requirements 4.4
    """
    anim_name, keyframes, keyframe_sound_map, duration_ms = animation_data
    
    # Create a fresh code generator
    gen = CodeGenerator()
    
    # Generate the animation
    cpp_code, metadata = gen.generate_keyframe_array(
        anim_name,
        keyframes,
        keyframe_sound_map,
        duration_ms
    )
    
    # Property 1: Audio event count should equal frame count
    assert metadata.audio_event_count == len(keyframes), \
        f"Expected {len(keyframes)} audio events, got {metadata.audio_event_count}"
    
    # Property 2: Audio event count should equal the number of mappings
    assert metadata.audio_event_count == len(keyframe_sound_map), \
        f"Audio event count should match mapping count"
    
    # Property 3: Frame count should be accurate
    assert metadata.frame_count == len(keyframes), \
        f"Frame count should be {len(keyframes)}, got {metadata.frame_count}"


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running complete metadata generation property tests...")
    print("=" * 60)
    
    try:
        test_complete_metadata_generation()
        print("✓ Property test passed: Complete metadata generation")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Complete metadata generation - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_metadata_in_generated_code()
        print("✓ Property test passed: Metadata in generated code")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Metadata in generated code - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_metadata_tracking_across_animations()
        print("✓ Property test passed: Metadata tracking across animations")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Metadata tracking across animations - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_lookup_table_includes_metadata()
        print("✓ Property test passed: Lookup table includes metadata")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Lookup table includes metadata - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_metadata_with_no_audio()
        print("✓ Property test passed: Metadata with no audio")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Metadata with no audio - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_metadata_with_all_audio()
        print("✓ Property test passed: Metadata with all audio")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Metadata with all audio - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print("=" * 60)
    print(f"RESULTS: {tests_passed} passed, {tests_failed} failed")
    
    exit(0 if tests_failed == 0 else 1)
