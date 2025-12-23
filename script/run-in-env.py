#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess
import sys


def find_and_activate_virtualenv():
    if "VIRTUAL_ENV" in os.environ:
        return

    try:
        # Get the top-level directory of the git repository
        my_path = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], text=True, close_fds=False
        ).strip()
    except subprocess.CalledProcessError:
        print(
            "Error: Not a git repository or unable to determine the top-level directory.",
            file=sys.stderr,
        )
        return

    # Check for virtual environments
    for venv in ["venv", ".venv", "."]:
        activate_path = (
            Path(my_path)
            / venv
            / ("Scripts" if os.name == "nt" else "bin")
            / "activate"
        )
        if activate_path.exists():
            # Activate the virtual environment by updating PATH
            venv_bin_dir = activate_path.parent
            os.environ["PATH"] = f"{venv_bin_dir}{os.pathsep}{os.environ['PATH']}"
            os.environ["VIRTUAL_ENV"] = str(venv_bin_dir.parent)
            print(f"Activated virtual environment: {venv_bin_dir.parent}")
            return

    print("No virtual environment found.", file=sys.stderr)


def run_command():
    # Execute the remaining arguments in the new environment
    if len(sys.argv) > 1:
        subprocess.run(sys.argv[1:], check=False, close_fds=False)
    else:
        print(
            "No command provided to run in the virtual environment.",
            file=sys.stderr,
        )


def cleanup_serial_locks():
    """Check for --device argument and kill processes using the port."""
    args = sys.argv[1:]
    for i, arg in enumerate(args):
        if arg == "--device" and i + 1 < len(args):
            device = args[i+1]
            # Only act on /dev/tty* devices to be safe
            if device.startswith("/dev/tty"):
                print(f"Checking for processes locking {device}...")
                # fuser -k sends SIGKILL to processes accessing the file
                # stdout/stderr suppressed to avoid noise if no process found
                subprocess.run(
                    ["fuser", "-k", device], 
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL
                )


if __name__ == "__main__":
    find_and_activate_virtualenv()
    cleanup_serial_locks()
    run_command()
