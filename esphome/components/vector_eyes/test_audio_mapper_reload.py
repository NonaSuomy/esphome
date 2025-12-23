"""
Additional tests for AudioMapper reload functionality with error handling.
"""
import os
import tempfile
from audio_mapper import AudioMapper


def test_reload_with_syntax_error():
    """Test that reload handles JSON syntax errors gracefully."""
    # Create temporary config file with valid JSON
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        f.write('{"exact_matches": {"test": "result"}, "pattern_matches": {}}')
        temp_file = f.name
    
    try:
        mapper = AudioMapper(temp_file)
        
        # Verify initial mapping works
        result = mapper.map_event_to_wav("test")
        assert result == "result", f"Expected 'result', got {result}"
        
        # Corrupt the file with invalid JSON
        with open(temp_file, 'w') as f:
            f.write('{"exact_matches": {"test": "result", invalid json')
        
        # Reload should fail gracefully
        success = mapper.reload_mappings()
        assert not success, "Reload should fail with invalid JSON"
        
        # Old mappings should still work (preserved on failed reload)
        result = mapper.map_event_to_wav("test")
        assert result == "result", f"Expected 'result' (preserved mapping), got {result}"
        
        print("✓ Reload with syntax error test passed")
    finally:
        os.unlink(temp_file)


def test_reload_with_missing_file():
    """Test that reload handles missing file gracefully."""
    mapper = AudioMapper("nonexistent_file.json")
    
    # Should handle missing file gracefully
    result = mapper.map_event_to_wav("test")
    assert result is None, f"Expected None with missing file, got {result}"
    
    print("✓ Reload with missing file test passed")


def test_reload_preserves_on_success():
    """Test that successful reload updates mappings correctly."""
    # Create temporary config file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        f.write('{"exact_matches": {"event1": "sound1"}, "pattern_matches": {}}')
        temp_file = f.name
    
    try:
        mapper = AudioMapper(temp_file)
        
        # Verify initial mapping
        result = mapper.map_event_to_wav("event1")
        assert result == "sound1", f"Expected 'sound1', got {result}"
        
        # Update file with new mappings
        with open(temp_file, 'w') as f:
            f.write('{"exact_matches": {"event2": "sound2"}, "pattern_matches": {}}')
        
        # Reload should succeed
        success = mapper.reload_mappings()
        assert success, "Reload should succeed with valid JSON"
        
        # Old mapping should be gone
        result = mapper.map_event_to_wav("event1")
        assert result is None, f"Expected None for old mapping, got {result}"
        
        # New mapping should work
        result = mapper.map_event_to_wav("event2")
        assert result == "sound2", f"Expected 'sound2', got {result}"
        
        print("✓ Reload preserves on success test passed")
    finally:
        os.unlink(temp_file)


if __name__ == "__main__":
    print("Running AudioMapper reload tests...\n")
    
    test_reload_with_syntax_error()
    test_reload_with_missing_file()
    test_reload_preserves_on_success()
    
    print("\n✅ All reload tests passed!")
