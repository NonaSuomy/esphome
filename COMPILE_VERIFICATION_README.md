# ESPHome Compilation Verification Tools

This directory contains tools to reliably compile and verify ESPHome configurations, with proper error detection and reporting.

## Problem

Standard bash output was being suppressed in the development environment, making it difficult to see compilation errors and verify success.

## Solution

Python-based compilation tools that write results to files in the workspace, which can be read reliably.

## Tools

### 1. `compile_with_verification.py` (RECOMMENDED)

Comprehensive compilation tool with error detection and reporting.

**Usage:**
```bash
# Compile only
python3 compile_with_verification.py config/vector-eyes-ttgo.yaml

# Compile and upload
python3 compile_with_verification.py config/vector-eyes-ttgo.yaml upload
```

**Output Files:**
- `compile_status.txt` - Compilation status and firmware info
- `compile_errors.txt` - Detailed error report (only created on failure)

**Features:**
- ✅ Detects compilation success/failure
- ✅ Verifies firmware was created
- ✅ Checks firmware age (detects if it's newly built)
- ✅ Extracts and categorizes errors:
  - Fatal errors (missing headers, syntax errors)
  - Linker errors (undefined references)
  - Compilation errors
  - Warnings
- ✅ Provides error context (surrounding lines)
- ✅ Writes detailed error reports
- ✅ Optional upload to device

### 2. `compile_status_check.py`

Quick status check without compilation.

**Usage:**
```bash
python3 compile_status_check.py config/vector-eyes-ttgo.yaml
```

**Output:**
- `compile_status.txt` - Current firmware status

**Use when:**
- You want to check if firmware exists
- You want to see firmware age/size
- You don't want to trigger a new compilation

### 3. `check_compile.py`

Alternative compilation checker (writes to /tmp, may not be readable).

### 4. `compile_and_check.sh`

Bash script version (may have output issues).

## Workflow for Fixing Compilation Errors

1. **Make code changes**

2. **Clean build (if needed):**
   ```bash
   rm -rf config/.esphome/build/vector-eyes-ttgo
   ```

3. **Compile with verification:**
   ```bash
   python3 compile_with_verification.py config/vector-eyes-ttgo.yaml
   ```

4. **Check results:**
   ```bash
   cat compile_status.txt
   ```

5. **If compilation failed:**
   ```bash
   cat compile_errors.txt
   ```
   
   This will show:
   - Fatal errors (missing files, syntax errors)
   - Linker errors (undefined references, missing symbols)
   - Compilation errors
   - Warnings

6. **Fix errors and repeat**

7. **Upload when successful:**
   ```bash
   python3 compile_with_verification.py config/vector-eyes-ttgo.yaml upload
   ```

## Example Output

### Successful Compilation

```
======================================================================
ESPHome Compilation
Config: config/vector-eyes-ttgo.yaml
Time: 2025-12-08 17:00:00
======================================================================

🔨 Starting compilation...

✅ COMPILATION SUCCESSFUL

✅ NEW FIRMWARE CREATED
   Path: .esphome/build/vector-eyes-ttgo/.pioenvs/vector-eyes-ttgo/firmware.elf
   Size: 21,614,212 bytes (20.61 MB)
   Modified: 2025-12-08 17:02:15
```

### Failed Compilation

```
======================================================================
ESPHome Compilation
Config: config/vector-eyes-ttgo.yaml
Time: 2025-12-08 17:00:00
======================================================================

🔨 Starting compilation...

❌ COMPILATION FAILED (exit code: 1)

📄 Error details written to: compile_errors.txt

Error summary:
  - Fatal errors: 2
  - Linker errors: 15
  - Compile errors: 0
  - Warnings: 3
```

Then check `compile_errors.txt` for details:
```
🔴 FATAL ERRORS (2):
----------------------------------------------------------------------

1. src/esphome/components/vector_eyes/vector_eyes.h:9:10: fatal error: SD.h: No such file or directory
   9 | #include <SD.h>
     |          ^~~~~~

🔴 LINKER ERRORS (15):
----------------------------------------------------------------------

1. undefined reference to `_ZN7esphome11vector_eyes14StorageAdapter10initializeEPNS_7storage7StorageE'
   /home/user/code/esphome003/esphome/config/.esphome/build/vector-eyes-ttgo/src/esphome/components/vector_eyes/vector_eyes.cpp:28
```

## Integration with Kiro

When working on compilation issues:

1. Use `compile_with_verification.py` to compile
2. Read `compile_status.txt` to check results
3. If failed, read `compile_errors.txt` for details
4. Fix errors in code
5. Repeat until successful

## Tips

- **Clean builds:** If you get strange linker errors, try cleaning the build directory first
- **Check firmware age:** The tool tells you if firmware is recent (< 2 minutes old)
- **Error context:** Error reports include surrounding lines for context
- **Timeout:** Compilation times out after 5 minutes (prevents hanging)
- **Upload timeout:** Upload times out after 2 minutes

## Troubleshooting

**Q: Compilation says successful but firmware is old**
A: The compilation may have used cached build. Try cleaning the build directory.

**Q: Can't read compile_errors.txt**
A: Make sure you're in the esphome003/esphome directory when running the script.

**Q: Script hangs**
A: There's a 5-minute timeout. If it hangs longer, something is wrong with the environment.

**Q: Upload fails**
A: Check that /dev/ttyUSB0 is the correct device and that you have permissions.
