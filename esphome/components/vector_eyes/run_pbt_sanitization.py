#!/usr/bin/env python3
"""
Property-based test runner for special character sanitization.
"""
import sys
from hypothesis import given, settings, strategies as st
from code_generator import CodeGenerator
import re

print("Running property-based tests for special character sanitization...")
print("=" * 60)

code_gen = CodeGenerator()

# Test 1: Sanitized output is valid C++ identifier
@given(event_name=st.text(min_size=0, max_size=100))
@settings(max_examples=100)
def test_sanitized_output_is_valid_cpp_identifier(event_name):
    """
    Property: Sanitized output must be a valid C++ identifier.
    """
    sanitized = code_gen.sanitize_identifier(event_name)
    
    # Must not be empty
    assert len(sanitized) > 0, f"Sanitized identifier is empty for input: {event_name}"
    
    # Must contain only alphanumeric and underscores
    assert re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', sanitized), \
        f"Sanitized identifier '{sanitized}' is not a valid C++ identifier (from '{event_name}')"
    
    # Must not start with a digit
    assert not sanitized[0].isdigit(), \
        f"Sanitized identifier '{sanitized}' starts with a digit (from '{event_name}')"

# Test 2: Sanitization is deterministic
@given(event_name=st.text(min_size=0, max_size=100))
@settings(max_examples=100)
def test_sanitization_is_deterministic(event_name):
    """
    Property: Sanitization must be deterministic.
    """
    result1 = code_gen.sanitize_identifier(event_name)
    result2 = code_gen.sanitize_identifier(event_name)
    
    assert result1 == result2, \
        f"Sanitization is not deterministic for '{event_name}': got '{result1}' and '{result2}'"

# Test 3: No consecutive underscores
@given(event_name=st.text(min_size=1, max_size=100))
@settings(max_examples=100)
def test_no_consecutive_underscores(event_name):
    """
    Property: Sanitized output should not have consecutive underscores.
    """
    sanitized = code_gen.sanitize_identifier(event_name)
    
    # Should not contain consecutive underscores (unless it's UNNAMED which is a special case)
    if sanitized != "UNNAMED":
        assert '__' not in sanitized, \
            f"Sanitized identifier '{sanitized}' contains consecutive underscores (from '{event_name}')"

# Test 4: No leading or trailing underscores
@given(event_name=st.text(min_size=1, max_size=100))
@settings(max_examples=100)
def test_no_leading_or_trailing_underscores(event_name):
    """
    Property: Sanitized output should not have leading or trailing underscores.
    """
    sanitized = code_gen.sanitize_identifier(event_name)
    
    if sanitized != "UNNAMED":
        # Should not end with underscore
        assert not sanitized.endswith('_'), \
            f"Sanitized identifier '{sanitized}' ends with underscore (from '{event_name}')"
        
        # Should not start with underscore unless it's a number prefix
        if sanitized.startswith('_'):
            # If it starts with underscore, the next character must be a digit
            assert len(sanitized) > 1 and sanitized[1].isdigit(), \
                f"Sanitized identifier '{sanitized}' starts with underscore but not followed by digit (from '{event_name}')"

# Run all tests
tests_passed = 0
tests_failed = 0

print("\n[TEST 1] Running: Sanitized output is valid C++ identifier (100 examples)")
try:
    test_sanitized_output_is_valid_cpp_identifier()
    print("✓ PASSED: Sanitized output is valid C++ identifier")
    tests_passed += 1
except Exception as e:
    print(f"✗ FAILED: Sanitized output is valid C++ identifier")
    print(f"  Error: {e}")
    tests_failed += 1

print("\n[TEST 2] Running: Sanitization is deterministic (100 examples)")
try:
    test_sanitization_is_deterministic()
    print("✓ PASSED: Sanitization is deterministic")
    tests_passed += 1
except Exception as e:
    print(f"✗ FAILED: Sanitization is deterministic")
    print(f"  Error: {e}")
    tests_failed += 1

print("\n[TEST 3] Running: No consecutive underscores (100 examples)")
try:
    test_no_consecutive_underscores()
    print("✓ PASSED: No consecutive underscores")
    tests_passed += 1
except Exception as e:
    print(f"✗ FAILED: No consecutive underscores")
    print(f"  Error: {e}")
    tests_failed += 1

print("\n[TEST 4] Running: No leading or trailing underscores (100 examples)")
try:
    test_no_leading_or_trailing_underscores()
    print("✓ PASSED: No leading or trailing underscores")
    tests_passed += 1
except Exception as e:
    print(f"✗ FAILED: No leading or trailing underscores")
    print(f"  Error: {e}")
    tests_failed += 1

print("\n" + "=" * 60)
print(f"RESULTS: {tests_passed} passed, {tests_failed} failed")
print("=" * 60)

if tests_failed == 0:
    print("\n✓ All property-based tests PASSED!")
    print("Property 23: Special character sanitization is validated.")
    print("Validates: Requirements 7.2")
    sys.exit(0)
else:
    print("\n✗ Some tests FAILED!")
    sys.exit(1)
