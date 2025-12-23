#!/usr/bin/env python3
"""
Verification script for Task 4.3: Add metadata generation

This script verifies that the AnimationData struct includes:
1. Frame count (length field)
2. Total duration (duration_ms field)
3. Audio event count (audio_event_count field)

Requirements: 4.4
"""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from code_generator import CodeGenerator, AnimationMetadata
from keyframe_extractor import ProceduralFaceData


def verify_metadata_dataclass():
    """Verify AnimationMetadata dataclass has all required fields."""
    print("Testing AnimationMetadata dataclass...")
    
    # Create a sample metadata instance
    metadata = AnimationMetadata(
        name="test_anim",
        c_identifier="TEST_ANIM",
        frame_count=10,
        duration_ms=1000,
        audio_event_count=3
    )
    
    # Verify all fields are present
    assert hasattr(metadata, 'name'), "Missing 'name' field"
    assert hasattr(metadata, 'c_identifier'), "Missing 'c_identifier' field"
    assert hasattr(metadata, 'frame_count'), "Missing 'frame_count' field"
    assert hasattr(metadata, 'duration_ms'), "Missing 'duration_ms' field"
    assert hasattr(metadata, 'audio_event_count'), "Missing 'audio_event_count' field"
    
    # Verify field values
    assert metadata.frame_count == 10, "frame_count not set correctly"
    assert metadata.duration_ms == 1000, "duration_ms not set correctly"
    assert metadata.audio_event_count == 3, "audio_event_count not set correctly"
    
    print("  ✓ AnimationMetadata dataclass has all required fields")


def verify_struct_generation():
    """Verify AnimationData struct includes all metadata fields."""
    print("\nTesting AnimationData struct generation...")
    
    gen = CodeGenerator()
    struct_code = gen.generate_animation_data_struct()
    
    # Verify struct definition is present
    assert "struct AnimationData" in struct_code, "AnimationData struct not found"
    
    # Verify all required fields are in the struct
    assert "const char* name" in struct_code, "Missing 'name' field in struct"
    assert "const AnimationKeyframe* frames" in struct_code, "Missing 'frames' field in struct"
    assert "size_t length" in struct_code, "Missing 'length' field (frame count) in struct"
    assert "uint32_t duration_ms" in struct_code, "Missing 'duration_ms' field in struct"
    assert "size_t audio_event_count" in struct_code, "Missing 'audio_event_count' field in struct"
    
    # Verify comments document the fields
    assert "Number of keyframes" in struct_code or "Frame count" in struct_code.lower(), \
        "Missing documentation for frame count"
    assert "Total animation duration" in struct_code or "duration" in struct_code.lower(), \
        "Missing documentation for duration"
    assert "Number of audio events" in struct_code or "audio event" in struct_code.lower(), \
        "Missing documentation for audio event count"
    
    print("  ✓ AnimationData struct includes all required metadata fields")
    print("  ✓ Struct fields are properly documented")


def verify_metadata_population():
    """Verify metadata is correctly populated during code generation."""
    print("\nTesting metadata population...")
    
    gen = CodeGenerator()
    
    # Create sample keyframes
    keyframes = [
        ProceduralFaceData(
            trigger_time=0,
            duration=100,
            scale_x=1.0,
            scale_y=1.0,
            angle=0.0,
            center_x=64.0,
            center_y=32.0,
            left_lid_top=0.0,
            left_lid_bottom=1.0,
            right_lid_top=0.0,
            right_lid_bottom=1.0
        ),
        ProceduralFaceData(
            trigger_time=100,
            duration=100,
            scale_x=1.0,
            scale_y=1.0,
            angle=0.0,
            center_x=64.0,
            center_y=32.0,
            left_lid_top=0.5,
            left_lid_bottom=0.5,
            right_lid_top=0.5,
            right_lid_bottom=0.5
        ),
        ProceduralFaceData(
            trigger_time=200,
            duration=100,
            scale_x=1.0,
            scale_y=1.0,
            angle=0.0,
            center_x=64.0,
            center_y=32.0,
            left_lid_top=0.0,
            left_lid_bottom=1.0,
            right_lid_top=0.0,
            right_lid_bottom=1.0
        )
    ]
    
    # Create audio mapping (2 audio events)
    keyframe_sound_map = {
        0: "sound1.wav",
        2: "sound2.wav"
    }
    
    duration_ms = 300
    
    # Generate code
    cpp_code, metadata = gen.generate_keyframe_array(
        "test_animation",
        keyframes,
        keyframe_sound_map,
        duration_ms
    )
    
    # Verify metadata is populated correctly
    assert metadata.name == "test_animation", "Animation name not set correctly"
    assert metadata.frame_count == 3, f"Expected frame_count=3, got {metadata.frame_count}"
    assert metadata.duration_ms == 300, f"Expected duration_ms=300, got {metadata.duration_ms}"
    assert metadata.audio_event_count == 2, f"Expected audio_event_count=2, got {metadata.audio_event_count}"
    
    # Verify metadata is included in generated comments
    assert "Frames: 3" in cpp_code, "Frame count not in generated comments"
    assert "Duration: 300ms" in cpp_code, "Duration not in generated comments"
    assert "Audio Events: 2" in cpp_code, "Audio event count not in generated comments"
    
    print("  ✓ Metadata correctly populated with frame_count=3")
    print("  ✓ Metadata correctly populated with duration_ms=300")
    print("  ✓ Metadata correctly populated with audio_event_count=2")
    print("  ✓ Metadata included in generated code comments")


def verify_lookup_table_includes_metadata():
    """Verify lookup table includes all metadata fields."""
    print("\nTesting lookup table generation...")
    
    gen = CodeGenerator()
    
    # Create sample keyframes and generate code
    keyframes = [
        ProceduralFaceData(
            trigger_time=0,
            duration=100,
            scale_x=1.0,
            scale_y=1.0,
            angle=0.0,
            center_x=64.0,
            center_y=32.0,
            left_lid_top=0.0,
            left_lid_bottom=1.0,
            right_lid_top=0.0,
            right_lid_bottom=1.0
        )
    ]
    
    # Generate two animations
    gen.generate_keyframe_array("anim_one", keyframes, {0: "sound.wav"}, 100)
    gen.generate_keyframe_array("anim_two", keyframes, {}, 150)
    
    # Generate lookup table
    lookup_table = gen.generate_lookup_table()
    
    # Verify lookup table structure
    assert "static const AnimationData ANIMATIONS[]" in lookup_table, \
        "ANIMATIONS array not found"
    assert "PROGMEM" in lookup_table, "PROGMEM storage not used"
    
    # Verify metadata fields are in the lookup table
    # The format should be: { "name", IDENTIFIER, frame_count, duration_ms, audio_event_count }
    assert "anim_one" in lookup_table, "Animation name not in lookup table"
    assert "ANIM_ONE" in lookup_table, "C identifier not in lookup table"
    
    # Verify the lookup table entries include all 5 fields
    # Count commas in each entry (should be 4 commas for 5 fields)
    lines = lookup_table.split('\n')
    for line in lines:
        if '"anim_one"' in line or '"anim_two"' in line:
            # Should have format: { "name", IDENTIFIER, count, duration, audio_count },
            comma_count = line.count(',')
            assert comma_count >= 4, f"Lookup table entry missing fields (found {comma_count} commas, expected 4+)"
    
    print("  ✓ Lookup table includes all metadata fields")
    print("  ✓ Lookup table uses PROGMEM storage")
    print("  ✓ Lookup table entries have correct format")


def verify_helper_functions():
    """Verify helper functions can access metadata."""
    print("\nTesting helper functions...")
    
    gen = CodeGenerator()
    helper_code = gen.generate_helper_functions()
    
    # Verify get_animation_metadata function exists
    assert "get_animation_metadata" in helper_code, \
        "get_animation_metadata function not found"
    
    # Verify it returns AnimationData pointer
    assert "const AnimationData*" in helper_code, \
        "get_animation_metadata doesn't return AnimationData pointer"
    
    # Verify documentation mentions metadata
    assert "metadata" in helper_code.lower(), \
        "Helper functions don't mention metadata"
    
    print("  ✓ get_animation_metadata function exists")
    print("  ✓ Function returns AnimationData pointer with metadata")


def main():
    """Run all verification tests."""
    print("=" * 70)
    print("Task 4.3 Verification: Add metadata generation")
    print("=" * 70)
    print("\nRequirement 4.4: Include frame count, duration, and audio event count")
    print()
    
    try:
        verify_metadata_dataclass()
        verify_struct_generation()
        verify_metadata_population()
        verify_lookup_table_includes_metadata()
        verify_helper_functions()
        
        print("\n" + "=" * 70)
        print("✅ ALL TESTS PASSED - Task 4.3 is complete!")
        print("=" * 70)
        print("\nSummary:")
        print("  ✓ AnimationMetadata dataclass includes all required fields")
        print("  ✓ AnimationData C++ struct includes frame_count, duration_ms, audio_event_count")
        print("  ✓ Metadata is correctly populated during code generation")
        print("  ✓ Lookup table includes all metadata fields")
        print("  ✓ Helper functions provide access to metadata")
        print("\nRequirement 4.4 is fully satisfied.")
        
        return 0
        
    except AssertionError as e:
        print(f"\n❌ TEST FAILED: {e}")
        return 1
    except Exception as e:
        print(f"\n❌ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
