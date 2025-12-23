#!/usr/bin/env python3
"""
Comprehensive ESPHome compilation with verification
Handles compilation, error detection, status reporting, and serial logging
"""

import subprocess
import os
import sys
import re
import time
import signal
from pathlib import Path
from datetime import datetime

class ESPHomeCompiler:
    def __init__(self, config_file, workspace_dir="."):
        self.config_file = config_file
        self.workspace_dir = workspace_dir
        self.build_name = Path(config_file).stem
        self.status_file = "compile_status.txt"
        self.error_file = "compile_errors.txt"
        self.serial_log_file = "serial_boot_log.txt"
        
    def write_status(self, message, append=False):
        """Write status message to file"""
        mode = 'a' if append else 'w'
        with open(self.status_file, mode) as f:
            f.write(message + "\n")
        print(message)
    
    def check_firmware_exists(self):
        """Check if firmware was built"""
        firmware_paths = [
            f".esphome/build/{self.build_name}/.pioenvs/{self.build_name}/firmware.elf",
            f"config/.esphome/build/{self.build_name}/.pioenvs/{self.build_name}/firmware.elf",
        ]
        
        for path in firmware_paths:
            if os.path.exists(path):
                size = os.path.getsize(path)
                mtime = datetime.fromtimestamp(os.path.getmtime(path))
                age_seconds = (datetime.now() - mtime).total_seconds()
                
                return {
                    'exists': True,
                    'path': path,
                    'size': size,
                    'modified': mtime,
                    'age_seconds': age_seconds,
                    'is_recent': age_seconds < 120  # Within 2 minutes
                }
        
        return {'exists': False}
    
    def extract_errors(self, output):
        """Extract errors from compilation output"""
        errors = {
            'fatal': [],
            'linker': [],
            'compile': [],
            'warnings': []
        }
        
        lines = output.split('\n')
        for i, line in enumerate(lines):
            # Fatal errors
            if 'fatal error:' in line.lower():
                # Get context (current line + next 2 lines)
                context = '\n'.join(lines[i:min(i+3, len(lines))])
                errors['fatal'].append(context)
            
            # Linker errors
            elif 'undefined reference' in line:
                # Get context
                context = '\n'.join(lines[max(0,i-1):min(i+2, len(lines))])
                errors['linker'].append(context)
            
            # Compilation errors
            elif re.search(r'\berror:', line, re.IGNORECASE) and 'fatal error' not in line.lower():
                context = '\n'.join(lines[max(0,i-1):min(i+2, len(lines))])
                errors['compile'].append(context)
            
            # Warnings
            elif re.search(r'\bwarning:', line, re.IGNORECASE):
                errors['warnings'].append(line.strip())
        
        return errors
    
    def write_error_report(self, errors, returncode):
        """Write detailed error report"""
        with open(self.error_file, 'w') as f:
            f.write("="*70 + "\n")
            f.write("ESPHome Compilation Error Report\n")
            f.write(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Config: {self.config_file}\n")
            f.write(f"Exit Code: {returncode}\n")
            f.write("="*70 + "\n\n")
            
            if errors['fatal']:
                f.write(f"🔴 FATAL ERRORS ({len(errors['fatal'])}):\n")
                f.write("-"*70 + "\n")
                for i, err in enumerate(errors['fatal'][:5], 1):
                    f.write(f"\n{i}. {err}\n")
                if len(errors['fatal']) > 5:
                    f.write(f"\n... and {len(errors['fatal'])-5} more fatal errors\n")
                f.write("\n")
            
            if errors['linker']:
                f.write(f"🔴 LINKER ERRORS ({len(errors['linker'])}):\n")
                f.write("-"*70 + "\n")
                for i, err in enumerate(errors['linker'][:10], 1):
                    f.write(f"\n{i}. {err}\n")
                if len(errors['linker']) > 10:
                    f.write(f"\n... and {len(errors['linker'])-10} more linker errors\n")
                f.write("\n")
            
            if errors['compile']:
                f.write(f"🔴 COMPILATION ERRORS ({len(errors['compile'])}):\n")
                f.write("-"*70 + "\n")
                for i, err in enumerate(errors['compile'][:10], 1):
                    f.write(f"\n{i}. {err}\n")
                if len(errors['compile']) > 10:
                    f.write(f"\n... and {len(errors['compile'])-10} more compilation errors\n")
                f.write("\n")
            
            if errors['warnings']:
                f.write(f"⚠️  WARNINGS ({len(errors['warnings'])}):\n")
                f.write("-"*70 + "\n")
                for i, warn in enumerate(errors['warnings'][:5], 1):
                    f.write(f"{i}. {warn}\n")
                if len(errors['warnings']) > 5:
                    f.write(f"... and {len(errors['warnings'])-5} more warnings\n")
    
    def compile(self):
        """Run compilation"""
        self.write_status("="*70)
        self.write_status(f"ESPHome Compilation")
        self.write_status(f"Config: {self.config_file}")
        self.write_status(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        self.write_status("="*70)
        
        # Check firmware before compilation
        before = self.check_firmware_exists()
        if before['exists']:
            self.write_status(f"\n📦 Existing firmware: {before['path']}", append=True)
            self.write_status(f"   Age: {before['age_seconds']/60:.1f} minutes", append=True)
        
        self.write_status("\n🔨 Starting compilation...", append=True)
        
        # Run compilation
        cmd = f"source ./venv/bin/activate && esphome compile {self.config_file}"
        
        try:
            result = subprocess.run(
                ["bash", "-c", cmd],
                capture_output=True,
                text=True,
                timeout=300,
                cwd=self.workspace_dir
            )
            
            # Check result
            if result.returncode == 0:
                self.write_status("\n✅ COMPILATION SUCCESSFUL", append=True)
                
                # Check firmware
                after = self.check_firmware_exists()
                if after['exists']:
                    if after['is_recent']:
                        self.write_status(f"\n✅ NEW FIRMWARE CREATED", append=True)
                    else:
                        self.write_status(f"\n⚠️  Firmware exists but may not be new", append=True)
                    
                    self.write_status(f"   Path: {after['path']}", append=True)
                    self.write_status(f"   Size: {after['size']:,} bytes ({after['size']/1024/1024:.2f} MB)", append=True)
                    self.write_status(f"   Modified: {after['modified'].strftime('%Y-%m-%d %H:%M:%S')}", append=True)
                    return True
                else:
                    self.write_status(f"\n❌ Firmware not found after compilation!", append=True)
                    return False
            else:
                self.write_status(f"\n❌ COMPILATION FAILED (exit code: {result.returncode})", append=True)
                
                # Extract and report errors
                combined_output = result.stdout + result.stderr
                errors = self.extract_errors(combined_output)
                
                # Write error report
                self.write_error_report(errors, result.returncode)
                
                self.write_status(f"\n📄 Error details written to: {self.error_file}", append=True)
                self.write_status(f"\nError summary:", append=True)
                self.write_status(f"  - Fatal errors: {len(errors['fatal'])}", append=True)
                self.write_status(f"  - Linker errors: {len(errors['linker'])}", append=True)
                self.write_status(f"  - Compile errors: {len(errors['compile'])}", append=True)
                self.write_status(f"  - Warnings: {len(errors['warnings'])}", append=True)
                
                return False
                
        except subprocess.TimeoutExpired:
            self.write_status("\n❌ COMPILATION TIMED OUT (5 minutes)", append=True)
            return False
        except Exception as e:
            self.write_status(f"\n❌ ERROR: {e}", append=True)
            return False
    
    def upload(self, device="/dev/ttyUSB0"):
        """Upload firmware to device"""
        self.write_status(f"\n📤 Uploading to {device}...", append=True)
        
        cmd = f"source ./venv/bin/activate && esphome upload {self.config_file} --device {device}"
        
        try:
            result = subprocess.run(
                ["bash", "-c", cmd],
                capture_output=True,
                text=True,
                timeout=120,
                cwd=self.workspace_dir
            )
            
            if result.returncode == 0:
                self.write_status("✅ UPLOAD SUCCESSFUL", append=True)
                return True
            else:
                self.write_status(f"❌ UPLOAD FAILED (exit code: {result.returncode})", append=True)
                return False
                
        except subprocess.TimeoutExpired:
            self.write_status("❌ UPLOAD TIMED OUT", append=True)
            return False
        except Exception as e:
            self.write_status(f"❌ UPLOAD ERROR: {e}", append=True)
            return False
    
    def capture_serial_logs(self, device="/dev/ttyUSB0", duration=15):
        """Capture serial logs after device boots"""
        self.write_status(f"\n📡 Capturing serial logs from {device} for {duration} seconds...", append=True)
        
        cmd = f"source ./venv/bin/activate && esphome logs {self.config_file} --device {device}"
        
        try:
            # Start the logs process
            process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=self.workspace_dir,
                preexec_fn=os.setsid  # Create new process group for clean termination
            )
            
            # Collect output for specified duration
            start_time = time.time()
            output_lines = []
            
            self.write_status(f"   Waiting for device to boot and collecting logs...", append=True)
            
            while time.time() - start_time < duration:
                line = process.stdout.readline()
                if line:
                    output_lines.append(line.rstrip())
                else:
                    # Check if process ended
                    if process.poll() is not None:
                        break
                    time.sleep(0.1)
            
            # Terminate the process gracefully
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=2)
            except:
                process.kill()
            
            # Write logs to file
            with open(self.serial_log_file, 'w') as f:
                f.write("="*70 + "\n")
                f.write(f"Serial Boot Log - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Device: {device}\n")
                f.write(f"Duration: {duration} seconds\n")
                f.write("="*70 + "\n\n")
                
                for line in output_lines:
                    f.write(line + "\n")
            
            # Analyze logs for errors
            errors_found = self.analyze_serial_logs(output_lines)
            
            self.write_status(f"✅ Serial logs captured: {self.serial_log_file}", append=True)
            self.write_status(f"   Lines captured: {len(output_lines)}", append=True)
            
            if errors_found:
                self.write_status(f"⚠️  Found {len(errors_found)} potential issues in logs", append=True)
                for issue in errors_found[:5]:
                    self.write_status(f"   - {issue}", append=True)
            else:
                self.write_status(f"✅ No obvious errors detected in boot logs", append=True)
            
            return True
            
        except Exception as e:
            self.write_status(f"❌ Serial log capture error: {e}", append=True)
            return False
    
    def analyze_serial_logs(self, log_lines):
        """Analyze serial logs for common issues"""
        issues = []
        
        # Patterns to look for
        error_patterns = [
            (r'\[E\]', 'Error log entry'),
            (r'invalid dev handle', 'Invalid device handle'),
            (r'Transmit failed', 'SPI transmission failure'),
            (r'not ready, cannot begin', 'Device not ready'),
            (r'No storage device.*found', 'Storage device not found'),
            (r'initialization failed', 'Initialization failure'),
            (r'mount.*failed', 'Mount failure'),
            (r'Preferred mount path not found', 'Mount path issue'),
            (r'fall back to', 'Fallback triggered'),
        ]
        
        for line in log_lines:
            for pattern, description in error_patterns:
                if re.search(pattern, line, re.IGNORECASE):
                    issues.append(f"{description}: {line[:80]}")
                    break
        
        return issues

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 compile_with_verification.py <config_file> [upload] [--log-duration SECONDS]")
        print("  upload: Upload firmware after compilation")
        print("  --log-duration: Seconds to capture serial logs after upload (default: 15)")
        sys.exit(1)
    
    config_file = sys.argv[1]
    do_upload = "upload" in sys.argv
    
    # Parse log duration
    log_duration = 15
    if "--log-duration" in sys.argv:
        try:
            idx = sys.argv.index("--log-duration")
            log_duration = int(sys.argv[idx + 1])
        except (IndexError, ValueError):
            print("Warning: Invalid --log-duration value, using default 15 seconds")
    
    compiler = ESPHomeCompiler(config_file)
    
    # Compile
    if not compiler.compile():
        print(f"\n❌ Compilation failed. Check {compiler.error_file} for details.")
        sys.exit(1)
    
    # Upload if requested
    if do_upload:
        if not compiler.upload():
            sys.exit(1)
        
        # Capture serial logs after upload
        print(f"\n⏳ Waiting 3 seconds for device to reset...")
        time.sleep(3)
        compiler.capture_serial_logs(duration=log_duration)
    
    print(f"\n✅ All operations completed successfully")
    print(f"📄 Status: {compiler.status_file}")
    if do_upload:
        print(f"📄 Serial logs: {compiler.serial_log_file}")
    sys.exit(0)

if __name__ == "__main__":
    main()
