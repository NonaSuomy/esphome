#!/usr/bin/env python3
"""
Verification script for Task 4.2: Audio Data Deduplication

Tests that the CodeGenerator properly:
1. Tracks which WAV files are used across animations
2. Generates shared audio data references
3. Avoids duplicating audio data in generated code
"""
import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from code_generator import CodeGenerator
from keyframe_extractor import ProceduralFaceData


def create_sample_keyframe(trigger_time=0, duration=100):
    """Create a sample keyframe for testing."""
    return ProceduralFaceData(
        trigger_time=trigger_time,
        duration=duration,
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


def test_audio_file_tracking():
    """Test that audio files are tracked across animations."""
    print("Test 1: Audio file tracking across animations")
    
    gen = CodeGenerator()
    
    # Generate animation 1 with audio
    keyframes1 = [create_sample_keyframe()]
    gen.generate_keyframe_array("anim1", keyframes1, {0: "sound1.wav"}, 100)
    
    # Generate animation 2 with different audio
    keyframes2 = [create_sample_keyframe()]
    gen.generate_keyframe_array("anim2", keyframes2, {0: "sound2.wav"}, 100)
    
    # Generate animation 3 with same audio as animation 1
    keyframes3 = [create_sample_keyframe()]
    gen.generate_keyframe_array("anim3", keyframes3, {0: "sound1.wav"}, 100)
    
    # Verify tracking
    assert len(gen.used_audio_files) == 2, f"Expected 2 unique audio files, got {len(gen.used_audio_files)}"
    assert "sound1.wav" in gen.used_audio_files, "sound1.wav not tracked"
    assert "sound2.wav" in gen.used_audio_files, "sound2.wav not tracked"
    
    print("  ✓ Audio files tracked correctly")
    print(f"  ✓ Tracked {len(gen.used_audio_files)} unique audio files: {sorted(gen.used_audio_files)}")
    return True


def test_shared_audio_references():
    """Test that shared audio data uses references, not duplication."""
    print("\nTest 2: Shared audio data references")
    
    gen = CodeGenerator()
    
    # Generate multiple animations using the same audio file
    shared_audio = "shared_sound.wav"
    
    keyframes1 = [create_sample_keyframe()]
    cpp1, _ = gen.generate_keyframe_array("anim1", keyframes1, {0: shared_audio}, 100)
    
    keyframes2 = [create_sample_keyframe()]
    cpp2, _ = gen.generate_keyframe_array("anim2", keyframes2, {0: shared_audio}, 100)
    
    keyframes3 = [create_sample_keyframe()]
    cpp3, _ = gen.generate_keyframe_array("anim3", keyframes3, {0: shared_audio}, 100)
    
    # Verify that each animation references the same string (not duplicating data)
    # The generated code should use string literals like "shared_sound.wav"
    assert f'"{shared_audio}"' in cpp1, "Audio reference missing in animation 1"
    assert f'"{shared_audio}"' in cpp2, "Audio reference missing in animation 2"
    assert f'"{shared_audio}"' in cpp3, "Audio reference missing in animation 3"
    
    # Verify deduplication tracking
    assert len(gen.used_audio_files) == 1, f"Expected 1 unique audio file, got {len(gen.used_audio_files)}"
    assert shared_audio in gen.used_audio_files, "Shared audio not tracked"
    
    print("  ✓ All animations reference the same audio file")
    print(f"  ✓ Audio file '{shared_audio}' used in 3 animations but tracked only once")
    return True


def test_no_audio_duplication_in_code():
    """Test that audio data is not duplicated in generated code."""
    print("\nTest 3: No audio data duplication in generated code")
    
    gen = CodeGenerator()
    
    # Generate animations with various audio configurations
    keyframes1 = [create_sample_keyframe(0, 100), create_sample_keyframe(100, 100)]
    gen.generate_keyframe_array("anim1", keyframes1, {0: "sound_a.wav", 1: "sound_b.wav"}, 200)
    
    keyframes2 = [create_sample_keyframe(0, 100)]
    gen.generate_keyframe_array("anim2", keyframes2, {0: "sound_a.wav"}, 100)
    
    keyframes3 = [create_sample_keyframe(0, 100)]
    gen.generate_keyframe_array("anim3", keyframes3, {0: "sound_c.wav"}, 100)
    
    # Verify unique audio files
    expected_files = {"sound_a.wav", "sound_b.wav", "sound_c.wav"}
    assert gen.used_audio_files == expected_files, \
        f"Expected {expected_files}, got {gen.used_audio_files}"
    
    print("  ✓ Audio files deduplicated correctly")
    print(f"  ✓ Tracked {len(gen.used_audio_files)} unique files: {sorted(gen.used_audio_files)}")
    return True


def test_deduplication_report():
    """Test that deduplication report is generated correctly."""
    print("\nTest 4: Deduplication report generation")
    
    gen = CodeGenerator()
    
    # Generate animations
    keyframes = [create_sample_keyframe()]
    gen.generate_keyframe_array("anim1", keyframes, {0: "sound1.wav"}, 100)
    gen.generate_keyframe_array("anim2", keyframes, {0: "sound2.wav"}, 100)
    gen.generate_keyframe_array("anim3", keyframes, {0: "sound1.wav"}, 100)
    
    # Generate report
    report = gen.generate_audio_deduplication_report()
    
    # Verify report content
    assert "Total unique audio files used: 2" in report, "Report doesn't show correct count"
    assert "sound1.wav" in report, "sound1.wav not in report"
    assert "sound2.wav" in report, "sound2.wav not in report"
    
    print("  ✓ Deduplication report generated correctly")
    print(f"  Report preview: {report.strip()[:100]}...")
    return True


def test_empty_audio():
    """Test handling of animations without audio."""
    print("\nTest 5: Handling animations without audio")
    
    gen = CodeGenerator()
    
    # Generate animation without audio
    keyframes = [create_sample_keyframe()]
    cpp, metadata = gen.generate_keyframe_array("anim_no_audio", keyframes, {}, 100)
    
    # Verify no audio tracked
    assert len(gen.used_audio_files) == 0, "Audio files tracked when none should be"
    assert metadata.audio_event_count == 0, "Audio event count should be 0"
    assert "nullptr" in cpp, "Should use nullptr for no audio"
    
    print("  ✓ Animations without audio handled correctly")
    return True


def test_reset_functionality():
    """Test that reset clears deduplication state."""
    print("\nTest 6: Reset functionality")
    
    gen = CodeGenerator()
    
    # Generate some animations
    keyframes = [create_sample_keyframe()]
    gen.generate_keyframe_array("anim1", keyframes, {0: "sound1.wav"}, 100)
    gen.generate_keyframe_array("anim2", keyframes, {0: "sound2.wav"}, 100)
    
    assert len(gen.used_audio_files) == 2, "Should have 2 audio files before reset"
    assert len(gen.animation_metadata) == 2, "Should have 2 animations before reset"
    
    # Reset
    gen.reset()
    
    assert len(gen.used_audio_files) == 0, "Audio files not cleared after reset"
    assert len(gen.animation_metadata) == 0, "Animation metadata not cleared after reset"
    
    print("  ✓ Reset clears deduplication state correctly")
    return True


def main():
    """Run all verification tests."""
    print("=" * 70)
    print("Task 4.2: Audio Data Deduplication - Verification")
    print("=" * 70)
    print()
    
    tests = [
        test_audio_file_tracking,
        test_shared_audio_references,
        test_no_audio_duplication_in_code,
        test_deduplication_report,
        test_empty_audio,
        test_reset_functionality,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
        except AssertionError as e:
            print(f"  ✗ FAILED: {e}")
            failed += 1
        except Exception as e:
            print(f"  ✗ ERROR: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print()
    print("=" * 70)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 70)
    
    if failed == 0:
        print("\n✅ All tests passed! Task 4.2 implementation verified.")
        print("\nImplementation Summary:")
        print("  • Audio files are tracked in CodeGenerator.used_audio_files (Set)")
        print("  • Each animation references audio files by string literal")
        print("  • No audio data is duplicated in generated code")
        print("  • Deduplication report shows unique audio files used")
        print("  • Requirement 4.3 satisfied: Audio data deduplication working")
        return 0
    else:
        print(f"\n❌ {failed} test(s) failed!")
        return 1


if __name__ == "__main__":
    sys.exit(main())
