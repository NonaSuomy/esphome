#!/usr/bin/env python3
"""
Verification script for the refactored JSON parsing implementation.
Tests the three main components: AnimationData, JSONParser, and KeyframeExtractor.
"""

import json
import tempfile
import os


def test_animation_data():
    """Test AnimationData dataclass."""
    print("Testing AnimationData...")
    
    # Import locally to avoid path issues
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from animation_data import AnimationData, KeyframeData, AudioEventData
    
    # Create test data
    kf1 = KeyframeData(trigger_time=0, keyframe_type="ProceduralFaceKeyFrame", data={"test": "data"})
    kf2 = KeyframeData(trigger_time=100, keyframe_type="RobotAudioKeyFrame", data={"audio": "test"})
    
    audio1 = AudioEventData(trigger_time=100, event_names=["Play__Test"])
    
    anim = AnimationData(
        name="test_anim",
        keyframes=[kf1, kf2],
        duration_ms=200,
        audio_events=[audio1]
    )
    
    # Test validation
    assert anim.validate(), "Animation validation failed"
    assert kf1.validate(), "Keyframe validation failed"
    assert audio1.validate(), "Audio event validation failed"
    
    # Test filtering methods
    face_kfs = anim.get_procedural_face_keyframes()
    assert len(face_kfs) == 1, f"Expected 1 face keyframe, got {len(face_kfs)}"
    
    audio_kfs = anim.get_audio_keyframes()
    assert len(audio_kfs) == 1, f"Expected 1 audio keyframe, got {len(audio_kfs)}"
    
    print("  ✓ AnimationData tests passed")
    return True


def test_json_parser():
    """Test JSONParser with a sample file."""
    print("Testing JSONParser...")
    
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from json_parser import JSONParser
    
    # Create a temporary test JSON file
    test_data = {
        "test_animation": [
            {
                "Name": "ProceduralFaceKeyFrame",
                "triggerTime_ms": 0,
                "faceScaleX": 1.0,
                "faceScaleY": 1.0,
                "faceAngle": 0.0,
                "faceCenterX": 0.0,
                "faceCenterY": 0.0,
                "leftEye": [0, 0, 0, 0, 0, 0.6, 0.6],
                "rightEye": [0, 0, 0, 0, 0, 0.6, 0.6]
            },
            {
                "Name": "RobotAudioKeyFrame",
                "triggerTime_ms": 50,
                "audioName": ["Play__Test_Sound"]
            }
        ]
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(test_data, f)
        temp_file = f.name
    
    try:
        parser = JSONParser()
        animation = parser.parse_animation_file(temp_file)
        
        assert animation is not None, "Parser returned None"
        assert animation.name == "test_animation", f"Wrong name: {animation.name}"
        assert len(animation.keyframes) == 2, f"Expected 2 keyframes, got {len(animation.keyframes)}"
        assert len(animation.audio_events) == 1, f"Expected 1 audio event, got {len(animation.audio_events)}"
        
        print("  ✓ JSONParser tests passed")
        return True
    finally:
        os.unlink(temp_file)


def test_keyframe_extractor():
    """Test KeyframeExtractor."""
    print("Testing KeyframeExtractor...")
    
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from keyframe_extractor import KeyframeExtractor
    from animation_data import KeyframeData, AnimationData
    
    # Create test keyframes
    kf1 = KeyframeData(
        trigger_time=0,
        keyframe_type="ProceduralFaceKeyFrame",
        data={
            "faceScaleX": 1.0,
            "faceScaleY": 1.0,
            "faceAngle": 0.0,
            "faceCenterX": 0.0,
            "faceCenterY": 0.0,
            "leftEye": [0, 0, 0, 0, 0, 0.6, 0.6],
            "rightEye": [0, 0, 0, 0, 0, 0.6, 0.6]
        }
    )
    
    kf2 = KeyframeData(
        trigger_time=100,
        keyframe_type="ProceduralFaceKeyFrame",
        data={
            "faceScaleX": 1.5,
            "faceScaleY": 1.5,
            "faceAngle": 10.0,
            "faceCenterX": 5.0,
            "faceCenterY": 5.0,
            "leftEye": [0, 0, 0, 0, 0, 0.3, 0.3],
            "rightEye": [0, 0, 0, 0, 0, 0.3, 0.3]
        }
    )
    
    anim = AnimationData(
        name="test",
        keyframes=[kf2, kf1],  # Intentionally out of order
        duration_ms=100
    )
    
    extractor = KeyframeExtractor()
    
    # Test sorting
    sorted_kfs = extractor.sort_by_trigger_time(anim.keyframes)
    assert sorted_kfs[0].trigger_time == 0, "Sorting failed"
    assert sorted_kfs[1].trigger_time == 100, "Sorting failed"
    
    # Test extraction
    face_data = extractor.extract_procedural_face(kf1)
    assert face_data is not None, "Failed to extract face data"
    assert face_data.scale_x == 1.0, f"Wrong scale_x: {face_data.scale_x}"
    assert face_data.left_lid_top == 0.6, f"Wrong lid value: {face_data.left_lid_top}"
    
    # Test extract all
    faces = extractor.extract_all_procedural_faces(anim)
    assert len(faces) == 2, f"Expected 2 faces, got {len(faces)}"
    assert faces[0].trigger_time == 0, "Faces not sorted"
    assert faces[0].duration == 100, f"Wrong duration: {faces[0].duration}"
    
    print("  ✓ KeyframeExtractor tests passed")
    return True


def main():
    """Run all tests."""
    print("=" * 60)
    print("Verifying JSON Parsing and Validation Implementation")
    print("=" * 60)
    
    tests = [
        test_animation_data,
        test_json_parser,
        test_keyframe_extractor
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
        except Exception as e:
            print(f"  ✗ Test failed: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print("=" * 60)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 60)
    
    return failed == 0


if __name__ == "__main__":
    import sys
    success = main()
    sys.exit(0 if success else 1)
