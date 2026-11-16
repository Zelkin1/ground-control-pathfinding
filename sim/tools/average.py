"""This script runs the simulator for many seeds and prints the averages of its metrics.

Runs with many units that meet each other are chaotic. A start that moves by a centimeter can
change which unit waits for which, and it can change single results by 10-50%. Each `--seed`
value of `build/sim.exe` moves the start of every squad by up to 1.5 times the size of the squad
itself, onto free ground with a clear line from the original spot. It then moves every unit by up
to 1.5 m on each axis and turns it by up to 0.05 rad, never closer than 0.5 m to another unit.
This script runs seeds 1 to N, averages every metric of each squad over the seeds, and prints
the totals for each mode. A total is the sum of the per-squad averages over all squads of the
mode.

Usage: python -I sim/tools/average.py N [sim.exe arguments...]
  e.g. python -I sim/tools/average.py 1000 --avoid coord

The arrival time is averaged only over the runs in which the squad arrives, and the column
`never` counts the other runs. The seeds run in parallel, one for each processor core. A run of
all scenarios in one mode takes about 70 ms.
"""
import os
import subprocess
import sys
from collections import namedtuple
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

PROJECT_FOLDER = Path(__file__).resolve().parents[2]  # This script is in `sim/tools`.
SIM_EXE = PROJECT_FOLDER / "build" / "sim.exe"  # `sim/build.sh` builds the simulator here.
# sim.exe prints these metrics in each squad row, in this order.
COLUMNS = [
    "arrive", "stops", "slow_s", "stuck", "spin", "waits", "swerve", "adjust", "bmpo", "bmpx",
    "wall", "inobs", "sep", "sepmax", "slot", "slotx", "lost", "xing", "swaps", "adjn", "top",
    "start",
]
# The totals line of each mode shows the sums of these metrics.
TOTAL_COLUMNS = [
    "arrive", "stops", "slow_s", "spin", "bmpo", "bmpx", "wall", "inobs", "sepmax", "lost",
    "swaps", "adjn", "start",
]
ARRIVE_INDEX = COLUMNS.index("arrive")
FIRST_METRIC_FIELD_INDEX = 5  # The metrics start at this field of a squad row.
ROW_FIELD_COUNT = FIRST_METRIC_FIELD_INDEX + len(COLUMNS)
NEVER_ARRIVED = -1  # sim.exe prints this arrival time for a squad that never arrives.
FALLBACK_WORKER_COUNT = 4  # This many runs go in parallel when the core count is unknown.

# A `SquadKey` tells which squad a row is for. A `SquadRow` holds the rest of the row: the kind of
# unit and the metrics in `COLUMNS` order.
SquadKey = namedtuple("SquadKey", ["scenario", "mode", "squad_index"])
SquadRow = namedtuple("SquadRow", ["kind", "metrics"])


def run_seed(seed, sim_arguments, sim_exe=None):
    """`run_seed` runs the simulator with one seed and reads the squad rows that it prints.

    A squad row has five fields before the metrics in `COLUMNS`. It can end with words about the
    state of the squad, and `run_seed` ignores them.
    Inputs: `sim_exe` is the simulator to run, or None for `SIM_EXE`.
    Result: It returns a dict that maps the `SquadKey` of each row to its `SquadRow`.
    """
    # Run the simulator.
    command = [str(sim_exe or SIM_EXE), "--seed", str(seed), *sim_arguments]
    process_result = subprocess.run(command, capture_output=True, text=True, check=True)

    # Read the squad rows. Skip the header line and every line with fewer fields than a row.
    squad_rows = {}
    for line in process_result.stdout.splitlines()[1:]:
        fields = line.split()
        if len(fields) < ROW_FIELD_COUNT:
            continue
        scenario, mode, squad_index_text, kind, _unit_count = fields[:FIRST_METRIC_FIELD_INDEX]
        metric_fields = fields[FIRST_METRIC_FIELD_INDEX:ROW_FIELD_COUNT]
        metrics = [float(field) for field in metric_fields]
        squad_key = SquadKey(scenario, mode, int(squad_index_text))
        squad_rows[squad_key] = SquadRow(kind, metrics)
    return squad_rows


def average_squad(seed_results, squad_key):
    """`average_squad` averages the metrics of one squad over the seeds in which the squad appears.

    Result: It returns the averages in `COLUMNS` order and the number of runs in which the squad
      never arrived. The arrival time is averaged only over the runs in which the squad arrives.
    Edge case: The arrival time is `NEVER_ARRIVED` when the squad never arrives.
    """
    # Collect the metrics and arrival times of the squad from every seed that has the squad.
    metrics_per_seed = []
    for seed_result in seed_results:
        if squad_key in seed_result:
            metrics_per_seed.append(seed_result[squad_key].metrics)
    arrival_values = [metrics[ARRIVE_INDEX] for metrics in metrics_per_seed]
    arrival_seconds = [arrival_value for arrival_value in arrival_values if arrival_value >= 0]

    # Average every column over these runs.
    run_count = len(metrics_per_seed)
    means = []
    for column_index in range(len(COLUMNS)):
        column_sum = sum(metrics[column_index] for metrics in metrics_per_seed)
        means.append(column_sum / run_count)

    # Average the arrival time again, only over the runs in which the squad arrived.
    arrived_count = len(arrival_seconds)
    if arrival_seconds:
        means[ARRIVE_INDEX] = sum(arrival_seconds) / arrived_count
    else:
        means[ARRIVE_INDEX] = NEVER_ARRIVED
    never_arrived_count = run_count - arrived_count
    return means, never_arrived_count


def add_to_mode_totals(mode_totals, means, never_arrived_count):
    """`add_to_mode_totals` adds the averages of one squad to the totals of its mode.

    Edge case: A squad that never arrived adds nothing to the arrival total.
    """
    has_arrival_time = means[ARRIVE_INDEX] >= 0
    for column in TOTAL_COLUMNS:
        if column == "arrive" and not has_arrival_time:
            continue
        mode_totals[column] += means[COLUMNS.index(column)]
    mode_totals["never"] += never_arrived_count


def sum_mode_metric(seed_result, mode, column):
    """`sum_mode_metric` sums the metric `column` over all squads of `mode` in the run of one seed.

    Edge case: Negative values, such as `NEVER_ARRIVED`, count as 0.
    """
    # Collect the rows of the squads of the mode.
    mode_rows = []
    for squad_key, squad_row in seed_result.items():
        if squad_key.mode == mode:
            mode_rows.append(squad_row)

    # Sum the metric.
    column_index = COLUMNS.index(column)
    return sum(max(0.0, squad_row.metrics[column_index]) for squad_row in mode_rows)


def compute_standard_error(values):
    """`compute_standard_error` returns the standard error of the mean of `values`.

    The standard error estimates by how much the mean changes from one set of seeds to another.
    Edge case: It returns 0 for fewer than two values.
    """
    value_count = len(values)
    if value_count <= 1:
        return 0.0
    mean = sum(values) / value_count
    variance = sum((value - mean) ** 2 for value in values) / (value_count - 1)
    return (variance / value_count) ** 0.5


def format_mode_total(seed_results, mode, mode_totals, column):
    """`format_mode_total` makes the text for one metric in the totals line of `mode`.

    Result: It returns the name of the metric, its total, and the standard error of the totals of
      the seeds themselves for it.
    """
    seed_totals = [sum_mode_metric(seed_result, mode, column) for seed_result in seed_results]
    standard_error = compute_standard_error(seed_totals)
    return f"{column} {mode_totals[column]:.1f} +-{standard_error:.1f}"


def format_totals_lines(seed_results, totals_by_mode):
    """`format_totals_lines` makes one line per mode with its totals in `TOTAL_COLUMNS`, each with
    the standard error of the totals of the seeds themselves, and the count of runs that never
    arrived.

    Inputs: `totals_by_mode` maps each mode to its totals from `add_to_mode_totals`.
    """
    lines = []
    for mode, mode_totals in totals_by_mode.items():
        column_texts = []
        for column in TOTAL_COLUMNS:
            column_texts.append(format_mode_total(seed_results, mode, mode_totals, column))
        lines.append(f"  {mode:<5} " + "  ".join(column_texts) + f"  never {mode_totals['never']}")
    return lines


def add_squad_averages(seed_results, totals_by_mode, squad_key):
    """`add_squad_averages` averages one squad over the seeds and adds it to the totals of its
    mode in `totals_by_mode`, which it creates when needed.

    Result: It returns the averages of the squad in `COLUMNS` order and its count of runs that
      never arrived.
    """
    means, never_arrived_count = average_squad(seed_results, squad_key)
    empty_totals = {column: 0.0 for column in TOTAL_COLUMNS} | {"never": 0}
    mode_totals = totals_by_mode.setdefault(squad_key.mode, empty_totals)
    add_to_mode_totals(mode_totals, means, never_arrived_count)
    return means, never_arrived_count


def main():
    # Read the seed count. The other arguments go to the simulator.
    seed_count = int(sys.argv[1])
    sim_arguments = sys.argv[2:]

    # Run the seeds in parallel, one for each processor core.
    worker_count = os.cpu_count() or FALLBACK_WORKER_COUNT
    seeds = range(1, seed_count + 1)
    sim_arguments_per_seed = [sim_arguments] * seed_count
    with ThreadPoolExecutor(worker_count) as pool:
        seed_results = list(pool.map(run_seed, seeds, sim_arguments_per_seed))

    # Print the averages of each squad, in the row order of seed 1, and add them to the totals of
    # the mode of the squad.
    column_headers = " ".join(f"{column:>6}" for column in COLUMNS)
    print(f"{'scen':<8} {'mode':<5} q {'kind':<13} never " + column_headers)
    totals_by_mode = {}
    for squad_key, first_squad_row in seed_results[0].items():
        means, never_arrived_count = add_squad_averages(seed_results, totals_by_mode, squad_key)
        key_text = f"{squad_key.scenario:<8} {squad_key.mode:<5} {squad_key.squad_index}"
        mean_texts = " ".join(f"{value:6.2f}" for value in means)
        print(f"{key_text} {first_squad_row.kind:<13} {never_arrived_count:>5} " + mean_texts)

    # Print the totals of each mode, each with the standard error of the per-seed totals.
    totals_title = f"totals over {seed_count} seeds"
    print(f"\n{totals_title} (sums of the per-squad averages, +- standard error over the seeds):")
    for line in format_totals_lines(seed_results, totals_by_mode):
        print(line)


if __name__ == "__main__":
    main()
