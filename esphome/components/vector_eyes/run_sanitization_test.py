#!/usr/bin/env python3
"""
Test runner for special character sanitization property tests.
"""
import sys
import traceback

# Force unbuffered output
sys.stdout = open(sys.stdout.fileno(), 'w', buffering=1)
sys.stderr = open(sys.stderr.fileno(), 'w', buffering=1)

print("Starting special character sanitization property tests...", flush=True)
print("=" * 60, flush=True)

try:
    from test_property_special_character_sanitization import TestSpecialCharacterSanitization
    from hypothesis import given, settings
    
    test_suite = TestSpecialCharacterSanitization()
    test_suite.setup_method()
    
    tests_passed = 0
    tests_failed = 0
    
    # Test 1: Sanitized output is valid C++ identifier
    print("\n[TEST 1] Running: Sanitized output is valid C++ identifier")
    try:
        # Run the property test with a few examples
        from hypothesis import strategies as st
        from test_property_special_character_sanitization import audio_event_name_strategy
        
        # Test with some examples
        test_cases = [
            "Play__Robot_Vic_Sfx__Blink",
            "Play__Robot_Vic__Happy_Short",
            "Event With Spaces",
            "Event:With:Colons",
            "123StartWithDigit",
            "",
            "___",
        ]
        
        for event_name in test_cases:
            test_suite.test_sanitized_output_is_valid_cpp_identifier(event_name)
        
        print("✓ PASSED: Sanitized output is valid C++ identifier")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Sanitized output is valid C++ identifier")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 2: All special characters removed or replaced
    print("\n[TEST 2] Running: All special characters removed or replaced")
    try:
        test_cases = [
            "Event-With-Dashes",
            "Event.With.Dots",
            "Event/With/Slashes",
            "Event(With)Parens",
            "Event[With]Brackets",
        ]
        
        for event_name in test_cases:
            test_suite.test_all_special_characters_removed_or_replaced(event_name)
        
        print("✓ PASSED: All special characters removed or replaced")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: All special characters removed or replaced")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 3: Sanitization is deterministic
    print("\n[TEST 3] Running: Sanitization is deterministic")
    try:
        test_cases = [
            "Play__Robot_Vic_Sfx__Blink",
            "Event With Spaces",
            "123StartWithDigit",
            "Special!@#$%Characters",
        ]
        
        for event_name in test_cases:
            test_suite.test_sanitization_is_deterministic(event_name)
        
        print("✓ PASSED: Sanitization is deterministic")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Sanitization is deterministic")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 4: Numeric only strings get prefix
    print("\n[TEST 4] Running: Numeric only strings get prefix")
    try:
        test_cases = ["123", "456789", "0", "999"]
        
        for event_name in test_cases:
            test_suite.test_numeric_only_strings_get_prefix(event_name)
        
        print("✓ PASSED: Numeric only strings get prefix")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Numeric only strings get prefix")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 5: Common audio event patterns
    print("\n[TEST 5] Running: Common audio event patterns")
    try:
        test_suite.test_common_audio_event_patterns()
        print("✓ PASSED: Common audio event patterns")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Common audio event patterns")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 6: No consecutive underscores
    print("\n[TEST 6] Running: No consecutive underscores")
    try:
        test_cases = [
            "Multiple___Underscores",
            "Play__Robot__Vic",
            "Event____With____Many",
        ]
        
        for event_name in test_cases:
            test_suite.test_no_consecutive_underscores(event_name)
        
        print("✓ PASSED: No consecutive underscores")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: No consecutive underscores")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 7: No leading or trailing underscores
    print("\n[TEST 7] Running: No leading or trailing underscores")
    try:
        test_cases = [
            "_LeadingUnderscore",
            "TrailingUnderscore_",
            "_BothSides_",
            "NormalName",
        ]
        
        for event_name in test_cases:
            test_suite.test_no_leading_or_trailing_underscores(event_name)
        
        print("✓ PASSED: No leading or trailing underscores")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: No leading or trailing underscores")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    print("\n" + "=" * 60)
    print(f"RESULTS: {tests_passed} passed, {tests_failed} failed")
    print("=" * 60)
    
    if tests_failed == 0:
        print("\n✓ All property tests PASSED!")
        print("Property 23: Special character sanitization is validated.")
    
    sys.exit(0 if tests_failed == 0 else 1)

except Exception as e:
    print(f"FATAL ERROR: Failed to run tests")
    print(f"Error: {e}")
    traceback.print_exc()
    sys.exit(1)
