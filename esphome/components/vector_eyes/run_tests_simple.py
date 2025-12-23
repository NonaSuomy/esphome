#!/usr/bin/env python3
import sys

# Write to file
with open('test_results.txt', 'w') as f:
    f.write("Starting tests...\n")
    f.flush()
    
    try:
        from test_property_keyframe_extraction import (
            test_complete_keyframe_extraction,
            test_keyframe_ordering_preservation
        )
        
        # Test 1
        f.write("\n[TEST 1] Complete keyframe extraction\n")
        f.flush()
        try:
            test_complete_keyframe_extraction()
            f.write("✓ PASSED\n")
        except Exception as e:
            f.write(f"✗ FAILED: {e}\n")
            import traceback
            traceback.print_exc(file=f)
        
        # Test 2
        f.write("\n[TEST 2] Keyframe ordering preservation\n")
        f.flush()
        try:
            test_keyframe_ordering_preservation()
            f.write("✓ PASSED\n")
        except Exception as e:
            f.write(f"✗ FAILED: {e}\n")
            import traceback
            traceback.print_exc(file=f)
        
        f.write("\nDone!\n")
    except Exception as e:
        f.write(f"FATAL: {e}\n")
        import traceback
        traceback.print_exc(file=f)

print("Results written to test_results.txt")
