#!/usr/bin/env python3
from code_generator import CodeGenerator

gen = CodeGenerator()

test_cases = [
    ("Play__Robot_Vic_Sfx__Blink", "PLAY_ROBOT_VIC_SFX_BLINK"),
    ("Play__Robot_Vic__Happy_Short", "PLAY_ROBOT_VIC_HAPPY_SHORT"),
    ("Stop__Audio_Event", "STOP_AUDIO_EVENT"),
    ("Event-With-Dashes", "EVENT_NEGWITH_NEGDASHES"),
    ("Event With Spaces", "EVENT_WITH_SPACES"),
    ("Event:With:Colons", "EVENT_WITH_COLONS"),
    ("Event.With.Dots", "EVENT_DOTWITH_DOTDOTS"),
    ("123StartWithDigit", "_123STARTWITHDIGIT"),
    ("", "UNNAMED"),
    ("___", "UNNAMED"),
    ("Multiple___Underscores", "MULTIPLE_UNDERSCORES"),
]

print("Testing sanitize_identifier:")
for input_name, expected in test_cases:
    result = gen.sanitize_identifier(input_name)
    status = "✓" if result == expected else "✗"
    print(f"{status} Input: '{input_name}'")
    print(f"  Expected: '{expected}'")
    print(f"  Got:      '{result}'")
    if result != expected:
        print(f"  MISMATCH!")
    print()
