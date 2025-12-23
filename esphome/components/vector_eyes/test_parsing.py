"""
Simple test script to verify JSON parsing and keyframe extraction.
"""
import sys
import os

# Add the parent directory to the path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vector_eyes.json_parser import JSONParser
from vector_eyes.keyframe_extractor import KeyframeExtractor


def test_parse_single_file():
    """Test parsing a single animation file."""
    parser = JSONParser()
    extractor = KeyframeExtractor()
    
    # Test with the blink animation
    test_file = "esphome003/esphome/esphome/components/vector_eyes/animations_json/anim_keepalive_blink_01.json"
    
    print(f"Testing parsing of {test_file}...")
    
    try:
        animation = parser.parse_animation_file(test_file)
        
        if animation:
            print(f"✓ Successfully parsed animation: {animation.name}")
            print(f"  - Total keyframes: {len(animation.keyframes)}")
            print(f"  - Duration: {animation.duration_ms}ms")
            print(f"  - Audio events: {len(animation.audio_events)}")
            
            # Test keyframe extraction
            face_keyframes = animation.get_procedural_face_keyframes()
            audio_keyframes = animation.get_audio_keyframes()
            
            print(f"  - ProceduralFace keyframes: {len(face_keyframes)}")
            print(f"  - Audio keyframes: {len(audio_keyframes)}")
            
            # Test extracting procedural faces
            faces = extractor.extract_all_procedural_faces(animation)
            print(f"  - Extracted face data: {len(faces)}")
            
            if faces:
                print(f"  - First face: trigger={faces[0].trigger_time}ms, duration={faces[0].duration}ms")
            
            # Test sorting
            sorted_kf = extractor.sort_by_trigger_time(animation.keyframes)
            print(f"  - Keyframes sorted: {len(sorted_kf)}")
            
            # Test grouping by type
            by_type = extractor.extract_all_by_type(animation)
            print(f"  - Keyframe types found: {list(by_type.keys())}")
            
            print("\n✓ All tests passed!")
            return True
        else:
            print("✗ Failed to parse animation")
            return False
            
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_parse_single_file()
    sys.exit(0 if success else 1)
