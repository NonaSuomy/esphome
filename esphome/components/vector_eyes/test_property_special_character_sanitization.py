"""
Property-based tests for special character sanitization.

Tests Property 23: Special character sanitization
Validates: Requirements 7.2
"""

import pytest
from hypothesis import given, strategies as st, assume
import re

try:
    from .code_generator import CodeGenerator
except ImportError:
    from code_generator import CodeGenerator


# Strategy for generating strings with various special characters
@st.composite
def audio_event_name_strategy(draw):
    """Generate realistic audio event names with special characters."""
    # Common patterns in Vector's audio event names
    prefixes = ["Play__Robot_Vic_Sfx__", "Play__Robot_Vic__", "Stop__", "Pause__"]
    base_names = ["Blink", "Happy", "Sad", "Curious", "Alert", "Sleep", "Wake"]
    suffixes = ["", "_Short", "_Long", "_01", "_02", "_Loop", "_End"]
    
    # Special characters that might appear
    special_chars = [" ", ":", "-", ".", "/", "(", ")", "[", "]", "&", "|", "!", "?", "@", "#", "$", "%"]
    
    # Build a name
    prefix = draw(st.sampled_from(prefixes))
    base = draw(st.sampled_from(base_names))
    suffix = draw(st.sampled_from(suffixes))
    
    # Optionally inject special characters
    if draw(st.booleans()):
        special = draw(st.sampled_from(special_chars))
        # Insert special character at random position
        name = prefix + base + special + suffix
    else:
        name = prefix + base + suffix
    
    return name


# Strategy for generating arbitrary strings with special characters
@st.composite
def string_with_special_chars_strategy(draw):
    """Generate strings with various special characters."""
    # Generate a string with printable ASCII characters
    base = draw(st.text(
        alphabet=st.characters(
            min_codepoint=32,  # Space
            max_codepoint=126,  # Tilde
        ),
        min_size=1,
        max_size=50
    ))
    return base


class TestSpecialCharacterSanitization:
    """
    Test suite for Property 23: Special character sanitization.
    
    **Feature: vector-animation-audio-sync, Property 23: Special character sanitization**
    
    For any audio event name containing special characters (spaces, colons, 
    underscores), the system should sanitize them to produce valid C++ identifiers.
    """
    
    def setup_method(self):
        """Set up test fixtures."""
        self.code_gen = CodeGenerator()
    
    @given(event_name=audio_event_name_strategy())
    def test_sanitized_output_is_valid_cpp_identifier(self, event_name):
        """
        Property: Sanitized output must be a valid C++ identifier.
        
        A valid C++ identifier:
        - Contains only alphanumeric characters and underscores
        - Does not start with a digit
        - Is not empty
        """
        sanitized = self.code_gen.sanitize_identifier(event_name)
        
        # Must not be empty
        assert len(sanitized) > 0, f"Sanitized identifier is empty for input: {event_name}"
        
        # Must contain only alphanumeric and underscores
        assert re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', sanitized), \
            f"Sanitized identifier '{sanitized}' is not a valid C++ identifier (from '{event_name}')"
        
        # Must not start with a digit
        assert not sanitized[0].isdigit(), \
            f"Sanitized identifier '{sanitized}' starts with a digit (from '{event_name}')"
    
    @given(event_name=string_with_special_chars_strategy())
    def test_all_special_characters_removed_or_replaced(self, event_name):
        """
        Property: All special characters must be removed or replaced.
        
        The sanitized output should contain no special characters that are
        invalid in C++ identifiers.
        """
        # Skip empty strings
        assume(len(event_name.strip()) > 0)
        
        sanitized = self.code_gen.sanitize_identifier(event_name)
        
        # Check that result contains only valid characters
        assert re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', sanitized), \
            f"Sanitized identifier '{sanitized}' contains invalid characters (from '{event_name}')"
    
    @given(event_name=st.text(min_size=1, max_size=100))
    def test_sanitization_is_deterministic(self, event_name):
        """
        Property: Sanitization must be deterministic.
        
        The same input should always produce the same output.
        """
        result1 = self.code_gen.sanitize_identifier(event_name)
        result2 = self.code_gen.sanitize_identifier(event_name)
        
        assert result1 == result2, \
            f"Sanitization is not deterministic for '{event_name}': got '{result1}' and '{result2}'"
    
    @given(event_name=st.text(alphabet=st.characters(min_codepoint=48, max_codepoint=57), min_size=1, max_size=10))
    def test_numeric_only_strings_get_prefix(self, event_name):
        """
        Property: Strings that are only digits must get a prefix.
        
        C++ identifiers cannot start with a digit, so numeric-only strings
        must be prefixed.
        """
        sanitized = self.code_gen.sanitize_identifier(event_name)
        
        # Must not start with a digit
        assert not sanitized[0].isdigit(), \
            f"Sanitized identifier '{sanitized}' starts with a digit (from '{event_name}')"
        
        # Must be a valid identifier
        assert re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', sanitized), \
            f"Sanitized identifier '{sanitized}' is not valid (from '{event_name}')"
    
    def test_common_audio_event_patterns(self):
        """
        Test common patterns found in Vector's audio event names.
        
        Note: Consecutive underscores are collapsed to single underscores for cleaner identifiers.
        """
        test_cases = [
            # Double underscores are collapsed to single underscores
            ("Play__Robot_Vic_Sfx__Blink", "PLAY_ROBOT_VIC_SFX_BLINK"),
            ("Play__Robot_Vic__Happy_Short", "PLAY_ROBOT_VIC_HAPPY_SHORT"),
            ("Stop__Audio_Event", "STOP_AUDIO_EVENT"),
            # Special characters are replaced
            ("Event-With-Dashes", "EVENT_NEGWITH_NEGDASHES"),
            ("Event With Spaces", "EVENT_WITH_SPACES"),
            ("Event:With:Colons", "EVENT_WITH_COLONS"),
            ("Event.With.Dots", "EVENT_DOTWITH_DOTDOTS"),
            ("Event/With/Slashes", "EVENT_SLASHWITH_SLASHSLASHES"),
            ("Event(With)Parens", "EVENT_LPARENWITH_RPARENPARENS"),
            ("Event[With]Brackets", "EVENT_LBRACKWITH_RBRACKBRACKETS"),
            ("Event{With}Braces", "EVENT_LBRACEWITH_RBRACEBRACES"),
            ("Event<With>Angles", "EVENT_LTWITH_GTANGLES"),
            ("Event&With&Ampersands", "EVENT_AMPWITH_AMPAMPERSANDS"),
            ("Event|With|Pipes", "EVENT_PIPEWITH_PIPEPIPES"),
            ("Event!With!Bangs", "EVENT_BANGWITH_BANGBANGS"),
            ("Event?With?Questions", "EVENT_QUESTWITH_QUESTQUESTIONS"),
            ("Event@With@Ats", "EVENT_ATWITH_ATATS"),
            ("Event#With#Hashes", "EVENT_HASHWITH_HASHHASHES"),
            ("Event$With$Dollars", "EVENT_DOLLARWITH_DOLLARDOLLARS"),
            ("Event%With%Percents", "EVENT_PERCENTWITH_PERCENTPERCENTS"),
            ("Event+With+Plus", "EVENT_PLUSWITH_PLUSPLUS"),
            ("Event=With=Equals", "EVENT_EQWITH_EQEQUALS"),
            ("Event*With*Stars", "EVENT_STARWITH_STARSTARS"),
            # Leading digits get underscore prefix
            ("123StartWithDigit", "_123STARTWITHDIGIT"),
            # Empty and underscore-only strings become UNNAMED
            ("", "UNNAMED"),
            ("___", "UNNAMED"),
            # Multiple consecutive underscores are collapsed
            ("Multiple___Underscores", "MULTIPLE_UNDERSCORES"),
        ]
        
        for input_name, expected_output in test_cases:
            sanitized = self.code_gen.sanitize_identifier(input_name)
            assert sanitized == expected_output, \
                f"Expected '{expected_output}' but got '{sanitized}' for input '{input_name}'"
    
    @given(event_name=st.text(min_size=1, max_size=100))
    def test_no_consecutive_underscores(self, event_name):
        """
        Property: Sanitized output should not have consecutive underscores.
        
        Multiple consecutive underscores should be collapsed to a single underscore.
        """
        sanitized = self.code_gen.sanitize_identifier(event_name)
        
        # Should not contain consecutive underscores (unless it's UNNAMED which is a special case)
        if sanitized != "UNNAMED":
            assert '__' not in sanitized, \
                f"Sanitized identifier '{sanitized}' contains consecutive underscores (from '{event_name}')"
    
    @given(event_name=st.text(min_size=1, max_size=100))
    def test_no_leading_or_trailing_underscores(self, event_name):
        """
        Property: Sanitized output should not have leading or trailing underscores.
        
        Unless the result is "UNNAMED", it should not start or end with underscores.
        """
        sanitized = self.code_gen.sanitize_identifier(event_name)
        
        if sanitized != "UNNAMED":
            # Should not start or end with underscore
            assert not sanitized.startswith('_') or sanitized[0] == '_' and sanitized[1].isdigit(), \
                f"Sanitized identifier '{sanitized}' starts with underscore (from '{event_name}')"
            assert not sanitized.endswith('_'), \
                f"Sanitized identifier '{sanitized}' ends with underscore (from '{event_name}')"


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v", "--tb=short"])
