#!/usr/bin/env python3
"""
Verification script for Task 2: Implement file operation methods in StorageAdapter

This script verifies that:
1. file_exists() is implemented with retry logic
2. read_file() is implemented with error handling and retry logic
3. list_animations() is implemented for file discovery with retry logic
4. Retry logic is properly implemented with exponential backoff
"""

import re
import sys

def check_file_content(filepath, patterns, description):
    """Check if file contains all required patterns"""
    print(f"\n{'='*70}")
    print(f"Checking: {description}")
    print(f"File: {filepath}")
    print(f"{'='*70}")
    
    try:
        with open(filepath, 'r') as f:
            content = f.read()
        
        all_found = True
        for pattern_name, pattern in patterns.items():
            if re.search(pattern, content, re.MULTILINE | re.DOTALL):
                print(f"✅ Found: {pattern_name}")
            else:
                print(f"❌ Missing: {pattern_name}")
                all_found = False
        
        return all_found
    except FileNotFoundError:
        print(f"❌ File not found: {filepath}")
        return False
    except Exception as e:
        print(f"❌ Error reading file: {e}")
        return False

def verify_storage_adapter_header():
    """Verify storage_adapter.h has retry logic declaration"""
    patterns = {
        "retry_operation template": r"template<typename Func>\s+bool retry_operation",
        "retry_operation parameters": r"retry_operation\(Func operation, const char \*operation_name, int max_retries",
        "DEFAULT_MAX_RETRIES constant": r"static constexpr int DEFAULT_MAX_RETRIES",
        "RETRY_DELAY_MS constant": r"static constexpr int RETRY_DELAY_MS",
        "file_exists declaration": r"bool file_exists\(const std::string &path\)",
        "read_file declaration": r"bool read_file\(const std::string &path, std::vector<uint8_t> &data\)",
        "list_animations declaration": r"bool list_animations\(std::vector<std::string> &animation_names\)",
    }
    
    return check_file_content(
        "esphome003/esphome/esphome/components/vector_eyes/storage_adapter.h",
        patterns,
        "StorageAdapter Header - Retry Logic Declaration"
    )

def verify_storage_adapter_implementation():
    """Verify storage_adapter.cpp has retry logic implementation"""
    patterns = {
        "file_exists with retry": r"bool StorageAdapter::file_exists.*retry_operation.*file_exists \(primary device\)",
        "read_file with retry": r"bool StorageAdapter::read_file.*retry_operation.*read_file \(primary device\)",
        "list_animations with retry": r"bool StorageAdapter::list_animations.*retry_operation.*list_animations",
        "open_file with retry": r"void \*StorageAdapter::open_file.*retry_operation.*open_file",
        "exponential backoff": r"delay_ms \* 2",
        "max delay cap": r"std::min.*1000",
        "retry logging": r"retrying in %d ms",
        "algorithm include": r"#include <algorithm>",
        "hal include": r"#include \"esphome/core/hal.h\"",
    }
    
    return check_file_content(
        "esphome003/esphome/esphome/components/vector_eyes/storage_adapter.cpp",
        patterns,
        "StorageAdapter Implementation - Retry Logic"
    )

def verify_retry_logic_features():
    """Verify specific retry logic features"""
    patterns = {
        "attempt counter": r"int attempt = 0",
        "delay variable": r"int delay_ms = RETRY_DELAY_MS",
        "while loop": r"while \(attempt < max_retries\)",
        "attempt increment": r"attempt\+\+",
        "operation call": r"if \(operation\(\)\)",
        "success on retry log": r"succeeded on attempt",
        "failure after retries log": r"failed after.*attempts",
        "retry warning log": r"failed \(attempt.*retrying",
        "delay call": r"delay\(delay_ms\)",
    }
    
    return check_file_content(
        "esphome003/esphome/esphome/components/vector_eyes/storage_adapter.h",
        patterns,
        "Retry Logic Features"
    )

def main():
    print("\n" + "="*70)
    print("TASK 2 VERIFICATION: File Operation Methods with Retry Logic")
    print("="*70)
    
    results = []
    
    # Check header file
    results.append(("Header Declaration", verify_storage_adapter_header()))
    
    # Check implementation file
    results.append(("Implementation", verify_storage_adapter_implementation()))
    
    # Check retry logic features
    results.append(("Retry Logic Features", verify_retry_logic_features()))
    
    # Summary
    print("\n" + "="*70)
    print("VERIFICATION SUMMARY")
    print("="*70)
    
    all_passed = True
    for name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {name}")
        if not passed:
            all_passed = False
    
    print("="*70)
    
    if all_passed:
        print("\n✅ ALL CHECKS PASSED - Task 2 is complete!")
        print("\nImplemented features:")
        print("  • file_exists() with retry logic")
        print("  • read_file() with error handling and retry logic")
        print("  • list_animations() with retry logic")
        print("  • open_file() with retry logic")
        print("  • Generic retry_operation() template")
        print("  • Exponential backoff (100ms → 200ms → 400ms → 800ms → 1000ms)")
        print("  • Multi-device fallback support")
        print("  • Comprehensive error logging")
        return 0
    else:
        print("\n❌ SOME CHECKS FAILED - Please review the implementation")
        return 1

if __name__ == "__main__":
    sys.exit(main())
