"""
Property-based tests for audio deduplication.

Feature: vector-animation-audio-sync, Property 12: Audio deduplication
Validates: Requirements 4.3
"""
from hypothesis import given, strategies as st, settings
from hypothesis.strategies import composite

from code_generator import CodeGenerator
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
def animation_with_audio_strategy(draw):
    """
    Generate an animation with keyframes and audio mappings.
    
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
    
    # Sort keyframes by trigger time
    keyframes.sort(key=lambda kf: kf.trigger_time)
    
    # Calculate duration
    if keyframes:
        duration_ms = max(kf.trigger_time + kf.duration for kf in keyframes)
    else:
        duration_ms = 1000
    
    # Generate audio mappings for some keyframes
    # Use a pool of audio files that can be shared
    audio_file_pool = [
        "blink.wav",
        "curious.wav",
        "happy.wav",
        "sad.wav",
        "excited.wav"
    ]
    
    keyframe_sound_map = {}
    num_audio_keyframes = draw(st.integers(min_value=0, max_value=min(num_keyframes, 10)))
    
    # Randomly assign audio files to keyframes
    # Some audio files will be reused across keyframes
    for _ in range(num_audio_keyframes):
        kf_idx = draw(st.integers(min_value=0, max_value=num_keyframes - 1))
        audio_file = draw(st.sampled_from(audio_file_pool))
        keyframe_sound_map[kf_idx] = audio_file
    
    return (anim_name, keyframes, keyframe_sound_map, duration_ms)


@composite
def multiple_animations_strategy(draw):
    """
    Generate multiple animations that may share audio files.
    
    Returns:
        List of (animation_name, keyframes, keyframe_sound_map, duration_ms) tuples
    """
    num_animations = draw(st.integers(min_value=2, max_value=10))
    animations = []
    
    for _ in range(num_animations):
        animation = draw(animation_with_audio_strategy())
        animations.append(animation)
    
    return animations


@given(animations=multiple_animations_strategy())
@settings(max_examples=10, deadline=None)
def test_audio_deduplication(animations):
    """
    Feature: vector-animation-audio-sync, Property 12: Audio deduplication
    
    Property: For any set of animations that reference the same WAV file, the 
    generated code should contain only one copy of that audio data, with all 
    animations referencing the shared data.
    
    Validates: Requirements 4.3
    """
    # Create a fresh code generator
    gen = CodeGenerator()
    
    # Track all audio files used across all animations
    all_audio_files_used = set()
    
    # Generate code for each animation
    for anim_name, keyframes, keyframe_sound_map, duration_ms in animations:
        # Track audio files in this animation
        for sound_file in keyframe_sound_map.values():
            if sound_file:
                all_audio_files_used.add(sound_file)
        
        # Generate the animation
        cpp_code, metadata = gen.generate_keyframe_array(
            anim_name,
            keyframes,
            keyframe_sound_map,
            duration_ms
        )
    
    # Property: The generator should track unique audio files
    # The number of unique audio files tracked should equal the number of unique files used
    assert len(gen.used_audio_files) == len(all_audio_files_used), \
        f"Audio deduplication tracking failed: expected {len(all_audio_files_used)} unique files, got {len(gen.used_audio_files)}"
    
    # Property: All audio files used should be in the tracked set
    for audio_file in all_audio_files_used:
        assert audio_file in gen.used_audio_files, \
            f"Audio file '{audio_file}' was used but not tracked for deduplication"
    
    # Property: No extra audio files should be tracked
    for audio_file in gen.used_audio_files:
        assert audio_file in all_audio_files_used, \
            f"Audio file '{audio_file}' was tracked but never used"
    
    # Property: The deduplication report should list all unique audio files
    report = gen.generate_audio_deduplication_report()
    
    # Verify report contains the correct count
    expected_count_str = f"Total unique audio files used: {len(all_audio_files_used)}"
    assert expected_count_str in report, \
        f"Deduplication report should contain '{expected_count_str}'"
    
    # Verify report lists all audio files
    for audio_file in all_audio_files_used:
        assert audio_file in report, \
            f"Deduplication report should list audio file '{audio_file}'"


@composite
def animations_with_shared_audio_strategy(draw):
    """
    Generate multiple animations that explicitly share audio files.
    
    This strategy ensures that we test the deduplication property with
    animations that definitely share audio files.
    """
    num_animations = draw(st.integers(min_value=2, max_value=5))
    
    # Create a small pool of shared audio files
    shared_audio_files = [
        "shared_sound_1.wav",
        "shared_sound_2.wav",
        "shared_sound_3.wav"
    ]
    
    animations = []
    
    for i in range(num_animations):
        # Generate animation name
        anim_name = f"anim_{i}_{draw(st.text(min_size=3, max_size=10, alphabet='abcdefghijklmnopqrstuvwxyz'))}"
        
        # Generate keyframes
        num_keyframes = draw(st.integers(min_value=1, max_value=10))
        keyframes = [draw(procedural_face_keyframe_strategy()) for _ in range(num_keyframes)]
        keyframes.sort(key=lambda kf: kf.trigger_time)
        
        # Calculate duration
        duration_ms = max(kf.trigger_time + kf.duration for kf in keyframes) if keyframes else 1000
        
        # Assign shared audio files to keyframes
        # Ensure at least one keyframe uses a shared audio file
        keyframe_sound_map = {}
        num_audio_keyframes = draw(st.integers(min_value=1, max_value=min(num_keyframes, 5)))
        
        for _ in range(num_audio_keyframes):
            kf_idx = draw(st.integers(min_value=0, max_value=num_keyframes - 1))
            # Use shared audio files
            audio_file = draw(st.sampled_from(shared_audio_files))
            keyframe_sound_map[kf_idx] = audio_file
        
        animations.append((anim_name, keyframes, keyframe_sound_map, duration_ms))
    
    return animations


@given(animations=animations_with_shared_audio_strategy())
@settings(max_examples=10, deadline=None)
def test_audio_deduplication_with_shared_files(animations):
    """
    Feature: vector-animation-audio-sync, Property 12: Audio deduplication
    
    Property: When multiple animations explicitly share the same audio files,
    the code generator should track each unique audio file exactly once,
    regardless of how many times it appears across animations.
    
    Validates: Requirements 4.3
    """
    # Create a fresh code generator
    gen = CodeGenerator()
    
    # Track how many times each audio file is used across all animations
    audio_file_usage_count = {}
    
    # Generate code for each animation
    for anim_name, keyframes, keyframe_sound_map, duration_ms in animations:
        # Count audio file usage
        for sound_file in keyframe_sound_map.values():
            if sound_file:
                audio_file_usage_count[sound_file] = audio_file_usage_count.get(sound_file, 0) + 1
        
        # Generate the animation
        cpp_code, metadata = gen.generate_keyframe_array(
            anim_name,
            keyframes,
            keyframe_sound_map,
            duration_ms
        )
    
    # Property: Each unique audio file should be tracked exactly once
    unique_audio_files = set(audio_file_usage_count.keys())
    
    assert len(gen.used_audio_files) == len(unique_audio_files), \
        f"Deduplication failed: expected {len(unique_audio_files)} unique files, got {len(gen.used_audio_files)}"
    
    # Property: All unique audio files should be tracked
    for audio_file in unique_audio_files:
        assert audio_file in gen.used_audio_files, \
            f"Audio file '{audio_file}' was used but not tracked"
    
    # Property: If an audio file is used multiple times, it should still only appear once in the tracking
    for audio_file, usage_count in audio_file_usage_count.items():
        if usage_count > 1:
            # This file is shared across multiple animations or keyframes
            # It should still only appear once in the deduplication tracking
            count_in_tracking = sum(1 for f in gen.used_audio_files if f == audio_file)
            assert count_in_tracking == 1, \
                f"Audio file '{audio_file}' used {usage_count} times but appears {count_in_tracking} times in tracking"


@composite
def animations_with_no_audio_strategy(draw):
    """
    Generate animations with no audio mappings.
    
    This tests that deduplication works correctly when some animations
    have no audio at all.
    """
    num_animations = draw(st.integers(min_value=1, max_value=5))
    animations = []
    
    for i in range(num_animations):
        anim_name = f"silent_anim_{i}"
        
        # Generate keyframes
        num_keyframes = draw(st.integers(min_value=1, max_value=10))
        keyframes = [draw(procedural_face_keyframe_strategy()) for _ in range(num_keyframes)]
        keyframes.sort(key=lambda kf: kf.trigger_time)
        
        # Calculate duration
        duration_ms = max(kf.trigger_time + kf.duration for kf in keyframes) if keyframes else 1000
        
        # No audio mappings
        keyframe_sound_map = {}
        
        animations.append((anim_name, keyframes, keyframe_sound_map, duration_ms))
    
    return animations


@given(animations=animations_with_no_audio_strategy())
@settings(max_examples=10, deadline=None)
def test_audio_deduplication_with_no_audio(animations):
    """
    Feature: vector-animation-audio-sync, Property 12: Audio deduplication
    
    Property: When animations have no audio mappings, the deduplication
    tracking should remain empty, and the report should indicate zero
    unique audio files.
    
    Validates: Requirements 4.3
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
    
    # Property: No audio files should be tracked
    assert len(gen.used_audio_files) == 0, \
        f"Expected no audio files to be tracked, but found {len(gen.used_audio_files)}"
    
    # Property: The deduplication report should indicate zero files
    report = gen.generate_audio_deduplication_report()
    assert "Total unique audio files used: 0" in report, \
        "Deduplication report should indicate zero audio files"


@composite
def mixed_animations_strategy(draw):
    """
    Generate a mix of animations: some with audio, some without, some sharing audio.
    
    This tests the most realistic scenario where we have a variety of animations.
    """
    num_animations = draw(st.integers(min_value=3, max_value=10))
    
    # Pool of audio files
    audio_file_pool = [
        "blink.wav",
        "curious.wav",
        "happy.wav",
        "sad.wav"
    ]
    
    animations = []
    
    for i in range(num_animations):
        anim_name = f"mixed_anim_{i}"
        
        # Generate keyframes
        num_keyframes = draw(st.integers(min_value=1, max_value=10))
        keyframes = [draw(procedural_face_keyframe_strategy()) for _ in range(num_keyframes)]
        keyframes.sort(key=lambda kf: kf.trigger_time)
        
        # Calculate duration
        duration_ms = max(kf.trigger_time + kf.duration for kf in keyframes) if keyframes else 1000
        
        # Randomly decide if this animation has audio
        has_audio = draw(st.booleans())
        
        keyframe_sound_map = {}
        if has_audio:
            # Assign audio to some keyframes
            num_audio_keyframes = draw(st.integers(min_value=1, max_value=min(num_keyframes, 5)))
            for _ in range(num_audio_keyframes):
                kf_idx = draw(st.integers(min_value=0, max_value=num_keyframes - 1))
                audio_file = draw(st.sampled_from(audio_file_pool))
                keyframe_sound_map[kf_idx] = audio_file
        
        animations.append((anim_name, keyframes, keyframe_sound_map, duration_ms))
    
    return animations


@given(animations=mixed_animations_strategy())
@settings(max_examples=10, deadline=None)
def test_audio_deduplication_mixed_animations(animations):
    """
    Feature: vector-animation-audio-sync, Property 12: Audio deduplication
    
    Property: In a realistic scenario with a mix of animations (some with audio,
    some without, some sharing audio), the deduplication should correctly track
    only the unique audio files used across all animations.
    
    Validates: Requirements 4.3
    """
    # Create a fresh code generator
    gen = CodeGenerator()
    
    # Track all unique audio files used
    all_audio_files = set()
    
    # Generate code for each animation
    for anim_name, keyframes, keyframe_sound_map, duration_ms in animations:
        # Track audio files
        for sound_file in keyframe_sound_map.values():
            if sound_file:
                all_audio_files.add(sound_file)
        
        # Generate the animation
        cpp_code, metadata = gen.generate_keyframe_array(
            anim_name,
            keyframes,
            keyframe_sound_map,
            duration_ms
        )
    
    # Property: The tracked audio files should match the unique files used
    assert len(gen.used_audio_files) == len(all_audio_files), \
        f"Deduplication failed: expected {len(all_audio_files)} unique files, got {len(gen.used_audio_files)}"
    
    # Property: All used audio files should be tracked
    assert gen.used_audio_files == all_audio_files, \
        f"Tracked audio files don't match used files"
    
    # Property: The report should be accurate
    report = gen.generate_audio_deduplication_report()
    expected_count_str = f"Total unique audio files used: {len(all_audio_files)}"
    assert expected_count_str in report, \
        f"Report should contain '{expected_count_str}'"


if __name__ == "__main__":
    # Run the tests
    tests_passed = 0
    tests_failed = 0
    
    print("Running audio deduplication property tests...")
    print("=" * 60)
    
    try:
        test_audio_deduplication()
        print("✓ Property test passed: Audio deduplication")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Audio deduplication - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_audio_deduplication_with_shared_files()
        print("✓ Property test passed: Audio deduplication with shared files")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Audio deduplication with shared files - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_audio_deduplication_with_no_audio()
        print("✓ Property test passed: Audio deduplication with no audio")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Audio deduplication with no audio - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        test_audio_deduplication_mixed_animations()
        print("✓ Property test passed: Audio deduplication mixed animations")
        tests_passed += 1
    except Exception as e:
        print(f"✗ Property test failed: Audio deduplication mixed animations - {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    print("=" * 60)
    print(f"RESULTS: {tests_passed} passed, {tests_failed} failed")
    
    exit(0 if tests_failed == 0 else 1)
