#!/usr/bin/env python3
"""
Simple test for special character sanitization.
"""
from code_generator import CodeGenerator

print("Testing special character sanitization...")
print("=" * 60)

code_gen = CodeGenerator()

# Test cases from the property test
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

passed = 0
failed = 0

for input_name, expected_output in test_cases:
    sanitized = code_gen.sanitize_identifier(input_name)
    if sanitized == expected_output:
        print(f"✓ PASS: '{input_name}' -> '{sanitized}'")
        passed += 1
    else:
        print(f"✗ FAIL: '{input_name}'")
        print(f"  Expected: '{expected_output}'")
        print(f"  Got:      '{sanitized}'")
        failed += 1

print("\n" + "=" * 60)
print(f"RESULTS: {passed} passed, {failed} failed")
print("=" * 60)

if failed > 0:
    exit(1)
else:
    print("\n✓ All tests PASSED!")
    exit(0)
