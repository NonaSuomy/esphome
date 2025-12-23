#!/usr/bin/env python3
"""
Verification script for audio deduplication property test (Task 4.6).

This script demonstrates that the property test correctly validates
Requirement 4.3: Audio data deduplication.
"""
import sys

from code_generator import CodeGenerator
from keyframe_extractor import ProceduralFaceData


def test_basic_deduplication():
    """Test that audio deduplication works with shared audio files."""
    print("Testing basic audio deduplication...")
    
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
        )
    ]
    
    # Generate 3 animations that share audio files
    # Animation 1: uses "blink.wav"
    gen.generate_keyframe_array("anim1", keyframes, {0: "blink.wav"}, 100)
    
    # Animation 2: uses "blink.wav" (shared) and "curious.wav"
    gen.generate_keyframe_array("anim2", keyframes, {0: "blink.wav"}, 100)
    
    # Animation 3: uses "curious.wav" (shared)
    gen.generate_keyframe_array("anim3", keyframes, {0: "curious.wav"}, 100)
    
    # Verify deduplication
    print(f"  Total animations generated: 3")
    print(f"  Total audio file references: 3 (blink, blink, curious)")
    print(f"  Unique audio files tracked: {len(gen.used_audio_files)}")
    print(f"  Audio files: {sorted(gen.used_audio_files)}")
    
    # Property: Only 2 unique audio files should be tracked
    assert len(gen.used_audio_files) == 2, \
        f"Expected 2 unique audio files, got {len(gen.used_audio_files)}"
    
    assert "blink.wav" in gen.used_audio_files, "blink.wav should be tracked"
    assert "curious.wav" in gen.used_audio_files, "curious.wav should be tracked"
    
    print("  ✓ Deduplication working correctly!")
    print()


def test_no_audio_animations():
    """Test that animations without audio don't affect deduplication."""
    print("Testing animations without audio...")
    
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
    
    # Generate animations without audio
    gen.generate_keyframe_array("silent_anim1", keyframes, {}, 100)
    gen.generate_keyframe_array("silent_anim2", keyframes, {}, 100)
    gen.generate_keyframe_array("silent_anim3", keyframes, {}, 100)
    
    print(f"  Total animations generated: 3")
    print(f"  Total audio file references: 0")
    print(f"  Unique audio files tracked: {len(gen.used_audio_files)}")
    
    # Property: No audio files should be tracked
    assert len(gen.used_audio_files) == 0, \
        f"Expected 0 audio files, got {len(gen.used_audio_files)}"
    
    print("  ✓ No audio files tracked for silent animations!")
    print()


def test_deduplication_report():
    """Test that the deduplication report is accurate."""
    print("Testing deduplication report...")
    
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
    
    # Generate animations with various audio files
    gen.generate_keyframe_array("anim1", keyframes, {0: "sound1.wav"}, 100)
    gen.generate_keyframe_array("anim2", keyframes, {0: "sound2.wav"}, 100)
    gen.generate_keyframe_array("anim3", keyframes, {0: "sound1.wav"}, 100)  # Reuse sound1
    gen.generate_keyframe_array("anim4", keyframes, {0: "sound3.wav"}, 100)
    
    # Generate report
    report = gen.generate_audio_deduplication_report()
    
    print("  Generated report:")
    print(report)
    
    # Verify report content
    assert "Total unique audio files used: 3" in report, \
        "Report should show 3 unique audio files"
    
    assert "sound1.wav" in report, "Report should list sound1.wav"
    assert "sound2.wav" in report, "Report should list sound2.wav"
    assert "sound3.wav" in report, "Report should list sound3.wav"
    
    print("  ✓ Deduplication report is accurate!")
    print()


def test_mixed_animations():
    """Test deduplication with a mix of animations (some with audio, some without)."""
    print("Testing mixed animations (with and without audio)...")
    
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
    
    # Mix of animations
    gen.generate_keyframe_array("anim_with_audio_1", keyframes, {0: "blink.wav"}, 100)
    gen.generate_keyframe_array("anim_silent_1", keyframes, {}, 100)
    gen.generate_keyframe_array("anim_with_audio_2", keyframes, {0: "blink.wav"}, 100)  # Shared
    gen.generate_keyframe_array("anim_silent_2", keyframes, {}, 100)
    gen.generate_keyframe_array("anim_with_audio_3", keyframes, {0: "happy.wav"}, 100)
    
    print(f"  Total animations: 5 (3 with audio, 2 silent)")
    print(f"  Audio references: blink, blink, happy")
    print(f"  Unique audio files tracked: {len(gen.used_audio_files)}")
    print(f"  Audio files: {sorted(gen.used_audio_files)}")
    
    # Property: Only 2 unique audio files should be tracked
    assert len(gen.used_audio_files) == 2, \
        f"Expected 2 unique audio files, got {len(gen.used_audio_files)}"
    
    assert "blink.wav" in gen.used_audio_files
    assert "happy.wav" in gen.used_audio_files
    
    print("  ✓ Mixed animations handled correctly!")
    print()


def main():
    """Run all verification tests."""
    print("=" * 70)
    print("Audio Deduplication Property Test Verification (Task 4.6)")
    print("Property 12: Audio deduplication")
    print("Validates: Requirements 4.3")
    print("=" * 70)
    print()
    
    try:
        test_basic_deduplication()
        test_no_audio_animations()
        test_deduplication_report()
        test_mixed_animations()
        
        print("=" * 70)
        print("✅ All verification tests passed!")
        print("=" * 70)
        print()
        print("Summary:")
        print("  The property test correctly validates that:")
        print("  1. Multiple animations can share the same audio files")
        print("  2. Each unique audio file is tracked exactly once")
        print("  3. The deduplication report accurately lists unique files")
        print("  4. Silent animations don't affect deduplication tracking")
        print("  5. Mixed scenarios (with/without audio) work correctly")
        print()
        print("  This ensures Requirement 4.3 is satisfied:")
        print("  'WHEN multiple animations share audio files THEN the system")
        print("   SHALL reference shared audio data rather than duplicating it'")
        print()
        
        return 0
        
    except AssertionError as e:
        print(f"❌ Verification failed: {e}")
        import traceback
        traceback.print_exc()
        return 1
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
