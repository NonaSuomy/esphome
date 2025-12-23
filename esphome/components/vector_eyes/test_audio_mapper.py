"""
Tests for AudioMapper class.
"""
import os
import json
import tempfile
from pathlib import Path
from audio_mapper import AudioMapper


def test_exact_match():
    """Test exact match lookup."""
    mapper = AudioMapper()
    mapper.add_exact_mapping("Play__Robot_Vic_Sfx__Blink", "blink")
    
    result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Blink")
    assert result == "blink", f"Expected 'blink', got {result}"
    print("✓ Exact match test passed")


def test_pattern_match():
    """Test pattern/substring matching."""
    mapper = AudioMapper()
    mapper.add_pattern_mapping("Scrn_Happy", "happy")
    
    # Should match events containing "Scrn_Happy"
    result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Scrn_Happy_Short")
    assert result == "happy", f"Expected 'happy', got {result}"
    print("✓ Pattern match test passed")


def test_specificity_precedence():
    """Test that more specific patterns take precedence."""
    mapper = AudioMapper()
    # Add patterns in non-specific order
    mapper.add_pattern_mapping("Scrn_Happy", "happy")
    mapper.add_pattern_mapping("Scrn_Happy_Short", "happy_short")
    
    # Should match the more specific pattern
    result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Scrn_Happy_Short")
    assert result == "happy_short", f"Expected 'happy_short', got {result}"
    
    # Should match the less specific pattern
    result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Scrn_Happy_Long")
    assert result == "happy", f"Expected 'happy', got {result}"
    print("✓ Specificity precedence test passed")


def test_unmapped_event():
    """Test handling of unmapped events."""
    mapper = AudioMapper()
    mapper.add_exact_mapping("Play__Robot_Vic_Sfx__Blink", "blink")
    
    result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Unknown")
    assert result is None, f"Expected None for unmapped event, got {result}"
    print("✓ Unmapped event test passed")


def test_config_file_loading():
    """Test loading mappings from configuration file."""
    # Create temporary config file
    config = {
        "exact_matches": {
            "Play__Robot_Vic_Sfx__Blink": "blink",
            "Play__Robot_Vic_Sfx__Wake_Word_On": "zelda"
        },
        "pattern_matches": {
            "Scrn_Happy_Short": "happy_short",
            "Scrn_Happy": "happy"
        }
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        mapper = AudioMapper(temp_file)
        
        # Test exact match
        result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Blink")
        assert result == "blink", f"Expected 'blink', got {result}"
        
        # Test pattern match with specificity
        result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Scrn_Happy_Short")
        assert result == "happy_short", f"Expected 'happy_short', got {result}"
        
        # Test stats
        stats = mapper.get_mapping_stats()
        assert stats['exact_count'] == 2, f"Expected 2 exact mappings, got {stats['exact_count']}"
        assert stats['pattern_count'] == 2, f"Expected 2 pattern mappings, got {stats['pattern_count']}"
        
        print("✓ Config file loading test passed")
    finally:
        os.unlink(temp_file)


def test_reload_mappings():
    """Test hot-reload of mappings."""
    # Create temporary config file
    config = {
        "exact_matches": {
            "Play__Robot_Vic_Sfx__Blink": "blink"
        },
        "pattern_matches": {}
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        temp_file = f.name
    
    try:
        mapper = AudioMapper(temp_file)
        
        # Initial mapping
        result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Blink")
        assert result == "blink", f"Expected 'blink', got {result}"
        
        # Update config file
        config["exact_matches"]["Play__Robot_Vic_Sfx__Happy"] = "happy"
        with open(temp_file, 'w') as f:
            json.dump(config, f)
        
        # Reload
        success = mapper.reload_mappings()
        assert success, "Reload should succeed"
        
        # Test new mapping
        result = mapper.map_event_to_wav("Play__Robot_Vic_Sfx__Happy")
        assert result == "happy", f"Expected 'happy', got {result}"
        
        print("✓ Reload mappings test passed")
    finally:
        os.unlink(temp_file)


def test_find_best_match():
    """Test find_best_match helper method."""
    mapper = AudioMapper()
    
    patterns = ["Scrn_Happy", "Scrn_Happy_Short", "Happy"]
    
    # Should find the most specific match
    result = mapper.find_best_match("Play__Robot_Vic_Sfx__Scrn_Happy_Short", patterns)
    assert result == "Scrn_Happy_Short", f"Expected 'Scrn_Happy_Short', got {result}"
    
    # Should find less specific match when more specific doesn't match
    result = mapper.find_best_match("Play__Robot_Vic_Sfx__Scrn_Happy_Long", patterns)
    assert result == "Scrn_Happy", f"Expected 'Scrn_Happy', got {result}"
    
    print("✓ Find best match test passed")


if __name__ == "__main__":
    print("Running AudioMapper tests...\n")
    
    test_exact_match()
    test_pattern_match()
    test_specificity_precedence()
    test_unmapped_event()
    test_config_file_loading()
    test_reload_mappings()
    test_find_best_match()
    
    print("\n✅ All tests passed!")
