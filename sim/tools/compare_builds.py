"""This script checks that two builds of the simulator give the same output, to prove that a
refactor or a speed-up changed no behavior. All runs go to one thread pool, under `nice -n 10`.

Runs: Each run pair runs both builds with the same arguments and compares their stdout (stderr
holds only timing lines, which vary). The pairs cover:
- the defaults and the replay baseline switches, on `QUICK_SEEDS` seeds (`--full`: `FULL_SEEDS`);
- each on/off switch flipped from its default, on `QUICK_FLIP_SEEDS` seeds (`--full`:
  `FULL_FLIP_SEEDS`), with the longer tick of `FLIP_TICK_ARGUMENTS`, which needs about a
  seventh of the ticks of the game frame;
- traces of seed 0 (defaults and baseline) and of ridge seed 65, compared byte for byte;
- two `--find` route searches, on two maps.
The quick set takes about a fifth of the time of the full set.

Cache: the output of each run is saved in `CACHE_DIR` under the hash of the exe file and of its
arguments, so a later check against the same build reads it instead of running that build again.
A build gives the same output for the same arguments, as long as gc.exe and the maps stay the same.

Usage: python -I sim/tools/compare_builds.py OLD_EXE NEW_EXE [--full] [-- extra arguments]
  Extra arguments after `--` go to the NEW build only, for example `-- --fullsearch off` to show
  that a new switch, turned off, keeps the old output.
"""
import hashlib
import os
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]  # This script is in sim/tools.
NICE = "C:/Program Files/Git/usr/bin/nice.exe"  # Git for Windows has `nice`.
CACHE_DIR = PROJECT / "build" / "compare_cache"
HASH_LENGTH = 16  # Hex digits of a hash in a cache file name.
DEFAULT_ON = ["driving", "smooth", "widebends", "formation", "lanes",
              "crossing", "keepright", "turns", "speedfirst", "detour",
              "pass", "formfirst", "keeproute", "parkreroute",
              "parkrobust", "partialshift", "fullsearch",
              "setupthreads", "tightbends", "factiontuning"]
DEFAULT_OFF = ["setupaccumulate"]
# The baseline turns off the passing and parking rules that have a switch.
BASELINE = ["--keeproute", "off", "--formfirst", "off", "--parkreroute", "off",
            "--partialshift", "off"]
QUICK_SEEDS = 5        # The quick set runs the defaults and the baseline on this many seeds.
QUICK_FLIP_SEEDS = 1   # It runs each flipped switch on this many seeds.
FULL_SEEDS = 40        # The full set runs the defaults and the baseline on this many seeds.
FULL_FLIP_SEEDS = 10   # It runs each flipped switch on this many seeds.
FLIP_TICK_ARGUMENTS = ["--tick", "0.04"]  # The flipped switches run with this tick.


def build_argument_sets(seed_count, flip_seed_count):
    """`build_argument_sets` returns (label, arguments, trace_name) for every run pair, without
    the exe. The traced runs, the slowest, come first, so that no slow run is left alone at the
    end of the pool."""
    argument_sets = [
        ("trace defaults", [], "defaults"),
        ("trace baseline", BASELINE, "baseline"),
        ("trace ridge 65", ["--seed", "65", "ridge"], "ridge65"),
    ]
    for seed in range(seed_count):
        argument_sets.append((f"defaults seed {seed}", ["--seed", str(seed)], None))
        argument_sets.append((f"baseline seed {seed}", ["--seed", str(seed), *BASELINE], None))
    flips = [(name, "off") for name in DEFAULT_ON] + [(name, "on") for name in DEFAULT_OFF]
    for name, value in flips:
        for seed in range(flip_seed_count):
            arguments = ["--seed", str(seed), f"--{name}", value, *FLIP_TICK_ARGUMENTS]
            argument_sets.append((f"--{name} {value} seed {seed}", arguments, None))
    argument_sets.append(("find FZ1", ["--find", "FZ1", "20"], None))
    argument_sets.append(("find MISSION10", ["--find", "MISSION10", "10"], None))
    return argument_sets


def hash_file(path):
    """`hash_file` returns the first `HASH_LENGTH` hex digits of the SHA-256 of a file."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:HASH_LENGTH]


def write_atomically(path, data):
    """`write_atomically` writes `data` to `path` through a temporary file, so that a reader never
    sees a half-written file, even when two runs of one exe save the same result."""
    temporary_path = path.with_name(f"{path.name}.{os.getpid()}.{id(data)}.tmp")
    temporary_path.write_bytes(data)
    os.replace(temporary_path, path)


def run_uncached(exe, tag, arguments, trace_name, trace_dir):
    """`run_uncached` runs `exe` once and returns its stdout and, for a traced run, the bytes of
    the trace."""
    command = [NICE, "-n", "10", exe, *arguments]
    trace_path = None
    if trace_name:
        trace_path = Path(trace_dir) / f"{tag}_{trace_name}.json"
        command += ["--trace", str(trace_path)]
    result = subprocess.run(command, cwd=PROJECT, capture_output=True, check=True)
    trace_bytes = trace_path.read_bytes() if trace_path else b""
    return result.stdout, trace_bytes


def run(exe, exe_hash, tag, arguments, trace_name, trace_dir):
    """`run` returns the stdout and trace bytes of one run of `exe`, from the cache when this exe
    already ran with these arguments, else from a new run, which it then saves."""
    key_text = "\0".join([*arguments, f"trace={bool(trace_name)}"])
    arguments_hash = hashlib.sha256(key_text.encode()).hexdigest()[:HASH_LENGTH]
    stdout_path = CACHE_DIR / exe_hash / f"{arguments_hash}.out"
    trace_path = CACHE_DIR / exe_hash / f"{arguments_hash}.trace"
    if stdout_path.exists() and trace_path.exists():
        return stdout_path.read_bytes(), trace_path.read_bytes()

    stdout, trace_bytes = run_uncached(exe, tag, arguments, trace_name, trace_dir)
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    write_atomically(trace_path, trace_bytes)
    write_atomically(stdout_path, stdout)
    return stdout, trace_bytes


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
start_seconds = time.monotonic()
old_hash = hash_file(old_exe)
new_hash = hash_file(new_exe)
argument_sets = build_argument_sets(seed_count, flip_seed_count)
with tempfile.TemporaryDirectory() as trace_dir:
    jobs = []
    for _, run_arguments, trace_name in argument_sets:
        jobs.append((old_exe, old_hash, "old", run_arguments, trace_name, trace_dir))
        new_arguments = run_arguments + extra_arguments
        jobs.append((new_exe, new_hash, "new", new_arguments, trace_name, trace_dir))
    with ThreadPoolExecutor(os.cpu_count() or 4) as pool:
        results = list(pool.map(lambda job: run(*job), jobs))
difference_count = 0
for set_index, (label, _, _) in enumerate(argument_sets):
    old_result = results[2 * set_index]
    new_result = results[2 * set_index + 1]
    if old_result != new_result:
        difference_count += 1
        print(f"DIFF {label}")
elapsed_seconds = time.monotonic() - start_seconds
print(f"{len(argument_sets)} run pairs, {difference_count} differ ({elapsed_seconds:.0f} s)")
