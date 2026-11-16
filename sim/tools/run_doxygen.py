"""This script runs Doxygen on the C sources of the simulator and writes the HTML documentation
to build/doxygen/html (start page: index.html).

It reads sim/Doxyfile and adds two settings that depend on this machine:
- `INPUT_FILTER`: this Python, running sim/tools/doxygen_filter.py, which turns the plain
  comments into Doxygen comments while Doxygen reads each file.
- `DOT_PATH`: the bin folder of the newest Graphviz in toolchain/, for the call graphs.
Doxygen itself is the newest toolchain/doxygen-*/doxygen.exe. Doxygen runs from the project
root, so the relative paths of sim/Doxyfile hold.

Usage: python -I sim/tools/run_doxygen.py
"""
import subprocess
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]  # This script is in sim/tools.
TOOLCHAIN = PROJECT / "toolchain"
FILTER = Path("sim") / "tools" / "doxygen_filter.py"  # Relative: Doxygen passes it to the shell.


def find_newest(pattern):
    """`find_newest` returns the last match of `pattern` under toolchain/, by name, or stops with
    a message."""
    matches = sorted(TOOLCHAIN.glob(pattern))
    if not matches:
        sys.exit(f"run_doxygen: nothing matches toolchain/{pattern}")
    return matches[-1]


doxygen = find_newest("doxygen-*/doxygen.exe")
dot_dir = find_newest("graphviz-*/*/bin")
config = (PROJECT / "sim" / "Doxyfile").read_text(encoding="utf-8")
config += f"\nINPUT_FILTER = \"{Path(sys.executable).as_posix()} -I {FILTER.as_posix()}\"\n"
config += f"DOT_PATH = \"{dot_dir.as_posix()}\"\n"  # Absolute: dot runs in the output folder.
(PROJECT / "build" / "doxygen").mkdir(parents=True, exist_ok=True)
subprocess.run([str(doxygen), "-"], input=config, text=True, cwd=PROJECT, check=True)
warnings = (PROJECT / "build" / "doxygen" / "warnings.txt").read_text(encoding="utf-8").strip()
warning_count = len(warnings.splitlines()) if warnings else 0
print(f"wrote build/doxygen/html/index.html ({warning_count} warning lines in warnings.txt)")
