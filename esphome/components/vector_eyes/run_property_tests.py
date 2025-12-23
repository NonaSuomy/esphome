#!/usr/bin/env python3
"""
Simple test runner for property-based tests.
"""
import sys
import traceback

# Force unbuffered output
sys.stdout = open(sys.stdout.fileno(), 'w', buffering=1)
sys.stderr = open(sys.stderr.fileno(), 'w', buffering=1)

print("Starting property-based tests...", flush=True)
print("=" * 60, flush=True)

try:
    from test_property_keyframe_extraction import (
        test_complete_keyframe_extraction,
        test_keyframe_ordering_preservation,
        test_json_structure_navigation,
        test_malformed_keyframe_handling
    )
    from test_property_audio_event_lookup import (
        test_audio_event_lookup,
        test_pattern_based_lookup,
        test_combined_exact_and_pattern_lookup
    )
    from test_property_unmapped_event_handling import (
        test_unmapped_event_handling,
        test_unmapped_pattern_handling
    )
    
    tests_passed = 0
    tests_failed = 0
    
    # Test 1: Complete keyframe extraction
    print("\n[TEST 1] Running: Complete keyframe extraction")
    try:
        test_complete_keyframe_extraction()
        print("✓ PASSED: Complete keyframe extraction")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Complete keyframe extraction")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 2: Keyframe ordering preservation
    print("\n[TEST 2] Running: Keyframe ordering preservation")
    try:
        test_keyframe_ordering_preservation()
        print("✓ PASSED: Keyframe ordering preservation")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Keyframe ordering preservation")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 3: JSON structure navigation
    print("\n[TEST 3] Running: JSON structure navigation")
    try:
        test_json_structure_navigation()
        print("✓ PASSED: JSON structure navigation")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: JSON structure navigation")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 4: Malformed keyframe handling
    print("\n[TEST 4] Running: Malformed keyframe handling")
    try:
        test_malformed_keyframe_handling()
        print("✓ PASSED: Malformed keyframe handling")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Malformed keyframe handling")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 5: Audio event lookup (exact matches)
    print("\n[TEST 5] Running: Audio event lookup (exact matches)")
    try:
        test_audio_event_lookup()
        print("✓ PASSED: Audio event lookup (exact matches)")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Audio event lookup (exact matches)")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 6: Audio event lookup (pattern matching)
    print("\n[TEST 6] Running: Audio event lookup (pattern matching)")
    try:
        test_pattern_based_lookup()
        print("✓ PASSED: Audio event lookup (pattern matching)")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Audio event lookup (pattern matching)")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 7: Audio event lookup (combined)
    print("\n[TEST 7] Running: Audio event lookup (combined)")
    try:
        test_combined_exact_and_pattern_lookup()
        print("✓ PASSED: Audio event lookup (combined)")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Audio event lookup (combined)")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 8: Unmapped event handling (exact matches)
    print("\n[TEST 8] Running: Unmapped event handling (exact matches)")
    try:
        test_unmapped_event_handling()
        print("✓ PASSED: Unmapped event handling (exact matches)")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Unmapped event handling (exact matches)")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    # Test 9: Unmapped event handling (patterns)
    print("\n[TEST 9] Running: Unmapped event handling (patterns)")
    try:
        test_unmapped_pattern_handling()
        print("✓ PASSED: Unmapped event handling (patterns)")
        tests_passed += 1
    except Exception as e:
        print(f"✗ FAILED: Unmapped event handling (patterns)")
        print(f"  Error: {e}")
        traceback.print_exc()
        tests_failed += 1
    
    print("\n" + "=" * 60)
    print(f"RESULTS: {tests_passed} passed, {tests_failed} failed")
    print("=" * 60)
    
    sys.exit(0 if tests_failed == 0 else 1)

except Exception as e:
    print(f"FATAL ERROR: Failed to run tests")
    print(f"Error: {e}")
    traceback.print_exc()
    sys.exit(1)
