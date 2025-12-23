#!/usr/bin/env python3
"""
ESPHome Compilation Checker
Compiles ESPHome config and provides detailed error reporting
"""

import subprocess
import sys
import os
import re
from pathlib import Path
from datetime import datetime

class CompileChecker:
    def __init__(self, config_file, venv_path="./venv/bin/activate"):
        self.config_file = config_file
        self.venv_path = venv_path
        self.log_file = f"/tmp/esphome_compile_{int(datetime.now().timestamp())}.log"
        self.build_name = Path(config_file).stem
        
    def run_command(self, cmd, description):
        """Run a command and capture output"""
        print(f"\n{'='*60}")
        print(f"{description}")
        print(f"{'='*60}")
        
        # Use bash to source venv and run command
        full_cmd = f"source {self.venv_path} && {cmd}"
        
        try:
            result = subprocess.run(
                ["bash", "-c", full_cmd],
                capture_output=True,
                text=True,
                timeout=300  # 5 minute timeout
            )
            
            # Write to log file
            with open(self.log_file, 'a') as f:
                f.write(f"\n{'='*60}\n")
                f.write(f"{description}\n")
                f.write(f"{'='*60}\n")
                f.write(result.stdout)
                f.write(result.stderr)
            
            return result
            
        except subprocess.TimeoutExpired:
            print("❌ Command timed out after 5 minutes")
            return None
        except Exception as e:
            print(f"❌ Error running command: {e}")
            return None
    
    def check_firmware_exists(self):
        """Check if firmware was built successfully"""
        possible_paths = [
            f"config/.esphome/build/{self.build_name}/.pioenvs/{self.build_name}/firmware.elf",
            f".esphome/build/{self.build_name}/.pioenvs/{self.build_name}/firmware.elf",
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                size = os.path.getsize(path)
                print(f"✅ Firmware found: {path}")
                print(f"   Size: {size:,} bytes")
                return True
        
        print(f"❌ Firmware not found. Checked:")
        for path in possible_paths:
            print(f"   - {path}")
        return False
    
    def extract_errors(self, output):
        """Extract and categorize errors from output"""
        errors = {
            'fatal': [],
            'linker': [],
            'compile': [],
            'other': []
        }
        
        lines = output.split('\n')
        for line in lines:
            if 'fatal error:' in line:
                errors['fatal'].append(line.strip())
            elif 'undefined reference' in line:
                errors['linker'].append(line.strip())
            elif re.search(r'error:', line) and 'fatal error:' not in line:
                errors['compile'].append(line.strip())
            elif 'Error' in line or 'ERROR' in line:
                errors['other'].append(line.strip())
        
        return errors
    
    def print_error_summary(self, result):
        """Print a summary of errors"""
        print("\n" + "="*60)
        print("ERROR SUMMARY")
        print("="*60)
        
        combined_output = result.stdout + result.stderr
        errors = self.extract_errors(combined_output)
        
        if errors['fatal']:
            print("\n🔴 Fatal Errors:")
            for err in errors['fatal'][:5]:
                print(f"  {err}")
        
        if errors['linker']:
            print("\n🔴 Linker Errors:")
            for err in errors['linker'][:10]:
                print(f"  {err}")
        
        if errors['compile']:
            print("\n🔴 Compilation Errors:")
            for err in errors['compile'][:10]:
                print(f"  {err}")
        
        if errors['other']:
            print("\n⚠️  Other Errors:")
            for err in errors['other'][:5]:
                print(f"  {err}")
        
        print(f"\n📄 Full log: {self.log_file}")
    
    def compile(self):
        """Compile the ESPHome config"""
        print(f"\n{'='*60}")
        print(f"ESPHome Compilation Checker")
        print(f"{'='*60}")
        print(f"Config: {self.config_file}")
        print(f"Log: {self.log_file}")
        
        # Check config exists
        if not os.path.exists(self.config_file):
            print(f"❌ Config file not found: {self.config_file}")
            return False
        
        # Run compilation
        result = self.run_command(
            f"esphome compile {self.config_file}",
            "Compiling ESPHome configuration"
        )
        
        if result is None:
            return False
        
        # Check result
        if result.returncode == 0:
            print("\n✅ COMPILATION SUCCESSFUL")
            return self.check_firmware_exists()
        else:
            print("\n❌ COMPILATION FAILED")
            self.print_error_summary(result)
            return False
    
    def upload(self, device="/dev/ttyUSB0"):
        """Upload firmware to device"""
        result = self.run_command(
            f"esphome upload {self.config_file} --device {device}",
            f"Uploading to {device}"
        )
        
        if result is None:
            return False
        
        if result.returncode == 0:
            print("\n✅ UPLOAD SUCCESSFUL")
            return True
        else:
            print("\n❌ UPLOAD FAILED")
            self.print_error_summary(result)
            return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 check_compile.py <config_file> [upload]")
        sys.exit(1)
    
    config_file = sys.argv[1]
    do_upload = len(sys.argv) > 2 and sys.argv[2] == "upload"
    
    checker = CompileChecker(config_file)
    
    # Compile
    if not checker.compile():
        sys.exit(1)
    
    # Upload if requested
    if do_upload:
        if not checker.upload():
            sys.exit(1)
    
    print("\n✅ All operations completed successfully")
    sys.exit(0)

if __name__ == "__main__":
    main()
