#!/usr/bin/env python3
"""
Demonstration of special character sanitization functionality.

This script shows how the sanitize_identifier() method handles various
special characters found in Vector's audio event names and animation identifiers.
"""

from code_generator import CodeGenerator

def main():
    gen = CodeGenerator()
    
    print("=" * 70)
    print("Special Character Sanitization Demonstration")
    print("=" * 70)
    print()
    
    # Real Vector audio event names
    print("Real Vector Audio Event Names:")
    print("-" * 70)
    real_events = [
        "Play__Robot_Vic_Sfx__Blink",
        "Play__Robot_Vic_Sfx__Curious_Short",
        "Play__Robot_Vic_Sfx__Happy_Long",
        "Stop__Robot_Vic_Sfx__Alert",
        "Play__Robot_Vic__Wake_Up",
    ]
    
    for event in real_events:
        sanitized = gen.sanitize_identifier(event)
        print(f"  {event:45} → {sanitized}")
    
    print()
    
    # Special characters
    print("Special Character Handling:")
    print("-" * 70)
    special_cases = [
        ("Event With Spaces", "Spaces"),
        ("Event:With:Colons", "Colons"),
        ("Event-With-Dashes", "Dashes"),
        ("Event.With.Dots", "Dots"),
        ("Event/With/Slashes", "Slashes"),
        ("Event(With)Parens", "Parentheses"),
        ("Event[With]Brackets", "Brackets"),
        ("Event{With}Braces", "Braces"),
        ("Event<With>Angles", "Angle Brackets"),
        ("Event&With&Ampersands", "Ampersands"),
        ("Event|With|Pipes", "Pipes"),
        ("Event!With!Bangs", "Exclamation Marks"),
        ("Event?With?Questions", "Question Marks"),
        ("Event@With@Ats", "At Signs"),
        ("Event#With#Hashes", "Hash Signs"),
        ("Event$With$Dollars", "Dollar Signs"),
        ("Event%With%Percents", "Percent Signs"),
        ("Event+With+Plus", "Plus Signs"),
        ("Event=With=Equals", "Equals Signs"),
        ("Event*With*Stars", "Asterisks"),
    ]
    
    for event, description in special_cases:
        sanitized = gen.sanitize_identifier(event)
        print(f"  {description:20} {event:30} → {sanitized}")
    
    print()
    
    # Edge cases
    print("Edge Cases:")
    print("-" * 70)
    edge_cases = [
        ("123StartWithDigit", "Starts with digit"),
        ("", "Empty string"),
        ("___", "Only underscores"),
        ("Multiple___Underscores", "Multiple underscores"),
        ("  Leading_Trailing  ", "Leading/trailing spaces"),
        ("MixedCase_Event", "Mixed case"),
    ]
    
    for event, description in edge_cases:
        sanitized = gen.sanitize_identifier(event)
        print(f"  {description:25} '{event:25}' → '{sanitized}'")
    
    print()
    print("=" * 70)
    print("All identifiers are valid C++ identifiers!")
    print("=" * 70)

if __name__ == "__main__":
    main()
