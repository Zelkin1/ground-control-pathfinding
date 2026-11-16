"""This script compares simulator setups over the same seeds, like `average.py` for each setup,
but faster: it puts every run of every setup into one pool, so that all processor cores stay busy
until the last run, instead of waiting at the end of each setup.

A setup is a label, an optional simulator build, and switches, in the form
`LABEL=[PATH.exe,]ARGUMENT,ARGUMENT,...`. Arguments after `--` go to every setup.

Usage: python -I sim/tools/compare.py N SETUP... [-- ARGUMENT...]
  e.g. python -I sim/tools/compare.py 100 "off=--lateshift,off" "on=" -- --avoid coord

For each setup, it prints the totals line of each mode, as `average.py` does, and for each
scenario and mode the sums over its squads of the averaged arrival time, stops, slow time,
in-place turning, bumps with other squads and new-avoidance onsets.
"""
import os
import sys
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

# `python -I` leaves the folder of this script off the import path, so add it for `average.py`.
SCRIPT_FOLDER = Path(__file__).resolve().parent  # `average.py` is in this folder too.
sys.path.insert(0, str(SCRIPT_FOLDER))
import average  # noqa: E402

SETUP_SEPARATOR = "="  # The label of a setup ends at the first of these.
COMMON_MARKER = "--"  # Arguments after this go to every setup.
# The scenario lines sum these metrics.
SCENARIO_COLUMNS = ["arrive", "stops", "slow_s", "spin", "bmpx", "adjn", "start"]


def parse_setup(text):
    """`parse_setup` reads one setup in the form `LABEL=[PATH.exe,]ARGUMENT,...`.

    Result: It returns the label, the simulator path or None for the default build, and the list
      of arguments.
    """
    label, _, rest = text.partition(SETUP_SEPARATOR)
    parts = [part for part in rest.split(",") if part]
    sim_exe = None
    if parts and parts[0].endswith(".exe"):
        sim_exe = parts.pop(0)
    return label, sim_exe, parts


def print_setup(label, seed_results):
    """`print_setup` prints the totals of each mode and the sums of each scenario for one setup,
    from its runs `seed_results`, one per seed."""
    # Average every squad, and add it to the totals of its mode and the sums of its scenario.
    totals_by_mode = {}
    scenario_sums = defaultdict(lambda: defaultdict(float))
    for squad_key in seed_results[0]:
        means, _ = average.add_squad_averages(seed_results, totals_by_mode, squad_key)
        sums = scenario_sums[(squad_key.scenario, squad_key.mode)]
        for column in SCENARIO_COLUMNS:
            value = means[average.COLUMNS.index(column)]
            sums[column] += max(0.0, value)

    # Print the label, the totals line of each mode, and the sums of each scenario and mode.
    print(f"== {label}")
    for line in average.format_totals_lines(seed_results, totals_by_mode):
        print(line)
    for (scenario, mode), sums in scenario_sums.items():
        sum_texts = "  ".join(f"{column} {sums[column]:7.2f}" for column in SCENARIO_COLUMNS)
        print(f"    {scenario:<12} {mode:<5} {sum_texts}")


def main():
    # Read the seed count, the setups and the arguments for every setup.
    seed_count = int(sys.argv[1])
    arguments = sys.argv[2:]
    common_arguments = []
    if COMMON_MARKER in arguments:
        marker_index = arguments.index(COMMON_MARKER)
        common_arguments = arguments[marker_index + 1:]
        arguments = arguments[:marker_index]
    setups = [parse_setup(text) for text in arguments]

    # Submit every run of every setup to one pool of threads, and collect the results in setup
    # order. Each thread only waits for its `sim.exe` process, so threads are as fast as a pool
    # of processes here (4 setups x 100 seeds on 16 cores: 56-57 s for both, against 59 s with
    # one `average.py` run per setup).
    worker_count = os.cpu_count() or average.FALLBACK_WORKER_COUNT
    seeds = range(1, seed_count + 1)
    with ThreadPoolExecutor(worker_count) as pool:
        futures_by_setup = []
        for _label, sim_exe, setup_arguments in setups:
            sim_arguments = setup_arguments + common_arguments
            futures = []
            for seed in seeds:
                futures.append(pool.submit(average.run_seed, seed, sim_arguments, sim_exe))
            futures_by_setup.append(futures)
        results_by_setup = []
        for futures in futures_by_setup:
            results_by_setup.append([future.result() for future in futures])

    # Print each setup.
    for (label, _sim_exe, _arguments), seed_results in zip(setups, results_by_setup):
        print_setup(label, seed_results)


if __name__ == "__main__":
    main()
