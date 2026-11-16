"""This script checks that two builds of the simulator give the same output, to prove that a
refactor or a speed-up changed no behaviour. All runs go to one thread pool, under `nice -n 10`.

Runs: Each run pair runs both builds with the same arguments and compares their stdout (stderr
holds only timing lines, which vary). The pairs cover:
- the defaults and the replay baseline switches, on `QUICK_SEEDS` seeds (`--full`: `FULL_SEEDS`);
- each on/off switch flipped from its default, on `QUICK_FLIP_SEEDS` seeds (`--full`:
  `FULL_FLIP_SEEDS`);
- traces of seed 0 (defaults and baseline) and of ridge seed 65, compared byte for byte;
- two `--find` route searches.
The quick set takes about a fifth of the time of the full set.

Usage: python -I sim/tools/compare_builds.py OLD_EXE NEW_EXE [--full] [-- extra arguments]
  Extra arguments after `--` go to the NEW build only, for example `-- --fullsearch off` to show
  that a new switch, turned off, keeps the old output.
"""
import os
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]  # This script is in sim/tools.
NICE = "C:/Program Files/Git/usr/bin/nice.exe"  # Git for Windows has `nice`.
DEFAULT_ON = ["pursuit", "rejoinroom", "smooth", "wideturns", "yield", "reassign", "lanes",
              "crossing", "keepright", "turns", "speedfirst", "smoothsway", "replan",
              "linearsteer", "bendlook", "aheadslow", "endstop", "overtake", "centersearch",
              "lateyield", "formfirst", "keeproute", "parkreplan", "parkleeway", "endcap",
              "cutcorners", "shortreturn", "lateshift", "partialshift", "widegap", "parkspeed",
              "setupchain", "widenstop", "keepfollow", "waitahead", "reformmatch", "fullsearch",
              "setupthreads", "keepspeed", "parkahead", "crossfollow", "oldroute", "slotpoint"]
DEFAULT_OFF = ["turnslow", "leaderfile", "findinfantry", "setupslice"]
BASELINE = ["--leaderfile", "on", "--keeproute", "off", "--formfirst", "off",
            "--parkreplan", "off", "--endcap", "off", "--cutcorners", "off",
            "--shortreturn", "off", "--lateshift", "off", "--partialshift", "off",
            "--widegap", "off", "--parkspeed", "off"]
QUICK_SEEDS = 10       # The quick set runs the defaults and the baseline on this many seeds.
QUICK_FLIP_SEEDS = 2   # It runs each flipped switch on this many seeds.
FULL_SEEDS = 40        # The full set runs the defaults and the baseline on this many seeds.
FULL_FLIP_SEEDS = 10   # It runs each flipped switch on this many seeds.


def build_argument_sets(seed_count, flip_seed_count):
    """`build_argument_sets` returns (label, arguments, trace_name) for every run pair, without
    the exe."""
    argument_sets = []
    for seed in range(seed_count):
        argument_sets.append((f"defaults seed {seed}", ["--seed", str(seed)], None))
        argument_sets.append((f"baseline seed {seed}", ["--seed", str(seed), *BASELINE], None))
    flips = [(name, "off") for name in DEFAULT_ON] + [(name, "on") for name in DEFAULT_OFF]
    for name, value in flips:
        for seed in range(flip_seed_count):
            arguments = ["--seed", str(seed), f"--{name}", value]
            argument_sets.append((f"--{name} {value} seed {seed}", arguments, None))
    argument_sets.append(("trace defaults", [], "defaults"))
    argument_sets.append(("trace baseline", BASELINE, "baseline"))
    argument_sets.append(("trace ridge 65", ["--seed", "65", "ridge"], "ridge65"))
    argument_sets.append(("find FZ1", ["--find", "FZ1", "20"], None))
    argument_sets.append(("find MISSION10 infantry",
                          ["--find", "MISSION10", "10", "--findinfantry", "on"], None))
    return argument_sets


def run(exe, tag, arguments, trace_name, trace_dir):
    """`run` runs `exe` once and returns its stdout and, for a traced run, the bytes of the
    trace."""
    command = [NICE, "-n", "10", exe, *arguments]
    trace_path = None
    if trace_name:
        trace_path = Path(trace_dir) / f"{tag}_{trace_name}.json"
        command += ["--trace", str(trace_path)]
    result = subprocess.run(command, cwd=PROJECT, capture_output=True, text=True, check=True)
    trace_bytes = trace_path.read_bytes() if trace_path else b""
    return result.stdout, trace_bytes


# Read the command line.
arguments = sys.argv[1:]
extra_arguments = []
if "--" in arguments:
    split_index = arguments.index("--")
    extra_arguments = arguments[split_index + 1:]
    arguments = arguments[:split_index]
is_full = "--full" in arguments
old_exe, new_exe = [argument for argument in arguments if argument != "--full"]
seed_count = FULL_SEEDS if is_full else QUICK_SEEDS
flip_seed_count = FULL_FLIP_SEEDS if is_full else QUICK_FLIP_SEEDS

# Run every pair in one pool, and compare.
argument_sets = build_argument_sets(seed_count, flip_seed_count)
with tempfile.TemporaryDirectory() as trace_dir:
    jobs = []
    for _, run_arguments, trace_name in argument_sets:
        jobs.append((old_exe, "old", run_arguments, trace_name, trace_dir))
        jobs.append((new_exe, "new", run_arguments + extra_arguments, trace_name, trace_dir))
    with ThreadPoolExecutor(os.cpu_count() or 4) as pool:
        results = list(pool.map(lambda job: run(*job), jobs))
difference_count = 0
for set_index, (label, _, _) in enumerate(argument_sets):
    old_result = results[2 * set_index]
    new_result = results[2 * set_index + 1]
    if old_result != new_result:
        difference_count += 1
        print(f"DIFF {label}")
print(f"{len(argument_sets)} run pairs, {difference_count} differ")
