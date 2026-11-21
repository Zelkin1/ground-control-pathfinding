#!/usr/bin/env bash
# This script builds the replay page OUT_DIR/replay.html from new simulator traces. The page
# shows each scenario at seed 0, except those of `LEFT_OUT_SCENARIOS`, in these variants:
#   - "original": the original game mode, shown as "Base game". Its runs add
#     `BASE_GAME_SWITCHES`, which turn off the changes that also act in the original game mode, so
#     that it stays the base game.
#   - "coord": formation following with the coordination between squads, shown as "Mod", with
#     all default switches.
#   - "new": the same as "coord" with `NEW_FEATURE_SWITCHES`, the new features that wait for
#     approval, shown as "New features". Once a feature is approved, it becomes a default and
#     leaves the list. With an empty list, this variant is left out.
# Each SEED SCENARIO pair adds one more tab with the run of that seed in that scenario, in the
# same variants, with the description of the scenario itself. The first pair is the worst
# run, and the others are runs that still need work. Each scenario of `LINE_SCENARIOS` adds a tab
# with its run at seed 0 in line formation (`--layout line`). make_replay.py, beside this script,
# merges the traces. The page shows the scenarios as written, with the Crayven units. Each faction of
# `EXTRA_FACTIONS` gets the same runs with `--faction`, merged into OUT_DIR/faction_NAME.json, which
# the page loads from beside itself when its faction tab is selected.
#
# Usage: bash simulator/replay/build_replay.sh OUT_DIR SEED SCENARIO [SEED SCENARIO]...
# Set PYTHON to the Python that runs make_replay.py. It is `python` by default. The script runs
# from the project root, two folders above this one.
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT_DIR=$1
shift
PYTHON=${PYTHON:-python}
SIM=build/sim.exe
NEW_FEATURE_SWITCHES=() # "new" adds these.
BASE_GAME_SWITCHES=(--fullsearch off) # "original" adds these, to show the base game.
LINE_SCENARIOS=(headon) # Each gets a tab in line formation.
EXTRA_FACTIONS=(order phoenix) # Each gets a data file with the same runs for its faction tab.
# The page and the data files leave out these scenarios, separated by commas: with them, the page
# would pass the 16 MB limit of one published file.
LEFT_OUT_SCENARIOS=closequarters,boarding,marineorders,parkedend
NEW_MODE=new # The "new" variant shows under this mode.

# `write_run_traces PREFIX SUFFIX ARGUMENT...` writes the traces of one run in every variant, with
# the simulator arguments ARGUMENT, to files that start with PREFIX. It lists them for the merge,
# each with its suffix, note and mode. An empty suffix adds nothing to the tab name, an empty note
# keeps the description of the scenario, and an empty mode keeps the mode that the trace names.
extra_arguments=()
write_run_traces() {
    local prefix=$1
    local suffix=$2
    shift 2
    "$SIM" "$@" --avoid original "${BASE_GAME_SWITCHES[@]}" --trace "${prefix}_original.json" \
        > /dev/null
    "$SIM" "$@" --avoid coord --trace "${prefix}_coord.json" > /dev/null
    extra_arguments+=(
        "${prefix}_original.json" "$suffix" "" ""
        "${prefix}_coord.json" "$suffix" "" ""
    )
    if [ "${#NEW_FEATURE_SWITCHES[@]}" -gt 0 ]; then
        "$SIM" "$@" --avoid coord "${NEW_FEATURE_SWITCHES[@]}" --trace "${prefix}_new.json" \
            > /dev/null
        extra_arguments+=("${prefix}_new.json" "$suffix" "" "$NEW_MODE")
    fi
}

PAIRS=("$@") # The SEED SCENARIO pairs of the command line.

# `write_faction_traces PREFIX ARGUMENT...` writes the traces of every run of the page with the
# simulator arguments ARGUMENT, to files that start with PREFIX, and lists them for the merge: seed
# 0, each chosen run, named by the index of its pair, and the runs in line formation. The first
# trace of seed 0 is the base of the merge, so it leaves the list of extra traces.
write_faction_traces() {
    local prefix=$1
    shift
    extra_arguments=()
    write_run_traces "${prefix}t" "" "$@"
    extra_arguments=("${extra_arguments[@]:4}")
    local pair_index=0
    for ((pair_start = 0; pair_start < ${#PAIRS[@]}; pair_start += 2)); do
        local seed=${PAIRS[pair_start]}
        local scenario=${PAIRS[pair_start + 1]}
        write_run_traces "${prefix}run${pair_index}" "seed $seed" "$@" --seed "$seed" "$scenario"
        pair_index=$((pair_index + 1))
    done
    for scenario in "${LINE_SCENARIOS[@]}"; do
        write_run_traces "${prefix}line_${scenario}" "line" "$@" --layout line "$scenario"
    done
}

# Merge the traces of the scenarios as written into the page, and those of each other faction
# into its data file.
write_faction_traces "$OUT_DIR/"
"$PYTHON" -I simulator/replay/make_replay.py --leave-out "$LEFT_OUT_SCENARIOS" \
    "$OUT_DIR/t_original.json" "$OUT_DIR/replay.html" "${extra_arguments[@]}"
for faction in "${EXTRA_FACTIONS[@]}"; do
    write_faction_traces "$OUT_DIR/${faction}_" --faction "$faction"
    "$PYTHON" -I simulator/replay/make_replay.py --leave-out "$LEFT_OUT_SCENARIOS" \
        "$OUT_DIR/${faction}_t_original.json" "$OUT_DIR/faction_${faction}.json" \
        "${extra_arguments[@]}"
done
