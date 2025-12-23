"""
Test code generator improvements for task 4.1.

Verifies:
- PROGMEM declarations are present
- Compact AnimationKeyframe structs are generated
- Efficient lookup tables are created
"""
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from code_generator import CodeGenerator
from keyframe_extractor import ProceduralFaceData


def test_progmem_declarations():
    """Test that PROGMEM declarations are present in generated code."""
    gen = CodeGenerator()
    
    # Create sample keyframes
    keyframes = [
        ProceduralFaceData(
            trigger_time=0,
            duration=100,
            scale_x=1.0,
            scale_y=1.0,
            angle=0.0,
            center_x=0.0,
            center_y=0.0,
            left_lid_top=0.0,
            left_lid_bottom=0.0,
            right_lid_top=0.0,
            right_lid_bottom=0.0
        ),
        ProceduralFaceData(
            trigger_time=100,
            duration=100,
            scale_x=1.2,
            scale_y=1.2,
            angle=0.0,
            center_x=0.0,
            center_y=0.0,
            left_lid_top=0.5,
            left_lid_bottom=0.0,
            right_lid_top=0.5,
            right_lid_bottom=0.0
        )
    ]
    
    # Generate code
    cpp_code, metadata = gen.generate_keyframe_array(
        "test_animation",
        keyframes,
        {},
        200
    )
    
    # Verify PROGMEM is present
    assert "PROGMEM" in cpp_code, "PROGMEM declaration missing"
    assert "static const AnimationKeyframe" in cpp_code, "AnimationKeyframe declaration missing"
    
    print("✓ PROGMEM declarations present")


def test_compact_struct_generation():
    """Test that compact AnimationKeyframe structs are generated."""
    gen = CodeGenerator()
    
    keyframes = [
        ProceduralFaceData(
            trigger_time=0,
            duration=100,
            scale_x=1.0,
            scale_y=1.0,
            angle=0.0,
            center_x=0.0,
            center_y=0.0,
            left_lid_top=0.0,
            left_lid_bottom=0.0,
            right_lid_top=0.0,
            right_lid_bottom=0.0
        )
    ]
    
    # Generate code with audio
    cpp_code, metadata = gen.generate_keyframe_array(
        "test_animation",
        keyframes,
        {0: "test_sound.wav"},
        100
    )
    
    # Verify compact struct format: { trigger_time, duration, { face_params }, sound_name }
    assert "{ 0, 100," in cpp_code, "Compact struct format missing"
    assert "1.0f, 1.0f, 0.0f" in cpp_code, "Face parameters missing"
    assert '"test_sound.wav"' in cpp_code, "Sound name missing"
    
    # Verify memory size comment
    assert "60 bytes per keyframe" in cpp_code, "Memory size comment incorrect"
    
    print("✓ Compact struct generation correct")


def test_efficient_lookup_table():
    """Test that efficient lookup tables are created."""
    gen = CodeGenerator()
    
    # Generate multiple animations
    for i in range(3):
        keyframes = [
            ProceduralFaceData(
                trigger_time=0,
                duration=100,
                scale_x=1.0,
                scale_y=1.0,
                angle=0.0,
                center_x=0.0,
                center_y=0.0,
                left_lid_top=0.0,
                left_lid_bottom=0.0,
                right_lid_top=0.0,
                right_lid_bottom=0.0
            )
        ]
        gen.generate_keyframe_array(f"anim_{i}", keyframes, {}, 100)
    
    # Generate lookup table
    lookup_code = gen.generate_lookup_table()
    
    # Verify PROGMEM storage
    assert "PROGMEM" in lookup_code, "PROGMEM missing from lookup table"
    assert "static const AnimationData ANIMATIONS[]" in lookup_code, "Lookup table declaration missing"
    
    # Verify animation count constant
    assert "ANIMATION_COUNT" in lookup_code, "Animation count constant missing"
    assert "static constexpr size_t ANIMATION_COUNT = 3" in lookup_code, "Animation count incorrect"
    
    # Verify animations are sorted (anim_0, anim_1, anim_2)
    anim_0_pos = lookup_code.find('"anim_0"')
    anim_1_pos = lookup_code.find('"anim_1"')
    anim_2_pos = lookup_code.find('"anim_2"')
    assert anim_0_pos < anim_1_pos < anim_2_pos, "Animations not sorted alphabetically"
    
    print("✓ Efficient lookup table created")


def test_helper_functions():
    """Test that helper functions are generated correctly."""
    gen = CodeGenerator()
    
    helper_code = gen.generate_helper_functions()
    
    # Verify all helper functions are present
    assert "get_animation_data" in helper_code, "get_animation_data missing"
    assert "get_animation_metadata" in helper_code, "get_animation_metadata missing"
    assert "get_animation_count" in helper_code, "get_animation_count missing"
    
    # Verify they use ANIMATION_COUNT constant
    assert "ANIMATION_COUNT" in helper_code, "Helper functions don't use ANIMATION_COUNT"
    
    print("✓ Helper functions generated correctly")


def test_metadata_generation():
    """Test that complete metadata is generated."""
    gen = CodeGenerator()
    
    keyframes = [
        ProceduralFaceData(
            trigger_time=0,
            duration=100,
            scale_x=1.0,
            scale_y=1.0,
            angle=0.0,
            center_x=0.0,
            center_y=0.0,
            left_lid_top=0.0,
            left_lid_bottom=0.0,
            right_lid_top=0.0,
            right_lid_bottom=0.0
        ),
        ProceduralFaceData(
            trigger_time=100,
            duration=100,
            scale_x=1.0,
            scale_y=1.0,
            angle=0.0,
            center_x=0.0,
            center_y=0.0,
            left_lid_top=0.0,
            left_lid_bottom=0.0,
            right_lid_top=0.0,
            right_lid_bottom=0.0
        )
    ]
    
    cpp_code, metadata = gen.generate_keyframe_array(
        "test_animation",
        keyframes,
        {0: "sound1.wav", 1: "sound2.wav"},
        200
    )
    
    # Verify metadata
    assert metadata.name == "test_animation", "Animation name incorrect"
    assert metadata.frame_count == 2, "Frame count incorrect"
    assert metadata.duration_ms == 200, "Duration incorrect"
    assert metadata.audio_event_count == 2, "Audio event count incorrect"
    
    # Verify metadata comments in code
    assert "Frames: 2" in cpp_code, "Frame count comment missing"
    assert "Duration: 200ms" in cpp_code, "Duration comment missing"
    assert "Audio Events: 2" in cpp_code, "Audio events comment missing"
    
    print("✓ Complete metadata generated")


def test_audio_deduplication():
    """Test that audio deduplication tracking works."""
    gen = CodeGenerator()
    
    # Generate animations with shared audio
    keyframes = [
        ProceduralFaceData(
            trigger_time=0,
            duration=100,
            scale_x=1.0,
            scale_y=1.0,
            angle=0.0,
            center_x=0.0,
            center_y=0.0,
            left_lid_top=0.0,
            left_lid_bottom=0.0,
            right_lid_top=0.0,
            right_lid_bottom=0.0
        )
    ]
    
    # Use same audio file in multiple animations
    gen.generate_keyframe_array("anim1", keyframes, {0: "shared_sound.wav"}, 100)
    gen.generate_keyframe_array("anim2", keyframes, {0: "shared_sound.wav"}, 100)
    gen.generate_keyframe_array("anim3", keyframes, {0: "unique_sound.wav"}, 100)
    
    # Verify deduplication
    assert len(gen.used_audio_files) == 2, "Audio deduplication failed"
    assert "shared_sound.wav" in gen.used_audio_files, "Shared audio not tracked"
    assert "unique_sound.wav" in gen.used_audio_files, "Unique audio not tracked"
    
    # Generate report
    report = gen.generate_audio_deduplication_report()
    assert "Total unique audio files used: 2" in report, "Deduplication report incorrect"
    
    print("✓ Audio deduplication working")


def main():
    """Run all tests."""
    print("Testing code generator improvements (Task 4.1)...\n")
    
    try:
        test_progmem_declarations()
        test_compact_struct_generation()
        test_efficient_lookup_table()
        test_helper_functions()
        test_metadata_generation()
        test_audio_deduplication()
        
        print("\n✅ All tests passed! Task 4.1 implementation verified.")
        return 0
    except AssertionError as e:
        print(f"\n❌ Test failed: {e}")
        return 1
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
