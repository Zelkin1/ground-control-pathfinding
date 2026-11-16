#!/usr/bin/env bash
# This script builds the replay page OUT_DIR/replay.html from new simulator traces. The page
# shows each scenario at seed 0 in these variants:
#   - "game": the base game.
#   - "coord": formation following with coordination and passing, shown as
#     "Form. + coord. + pass.", with all default switches.
#   - "new": the same with `NEW_FEATURE_SWITCHES`, the new features that wait for approval, shown
#     as "New features". Once a feature is approved, it becomes a default and leaves the list.
#     With an empty list, this variant is left out.
# Each SEED SCENARIO pair adds one more tab with the run of that seed in that scenario, in the
# same variants, with the description of the scenario itself. The first pair is the worst
# run, and the others are runs that still need work. sim/replay/make_replay.py merges the traces.
#
# Usage: bash sim/replay/build_replay.sh OUT_DIR SEED SCENARIO [SEED SCENARIO]...
# Set PYTHON to the Python that runs make_replay.py. It is `python` by default. The script runs
# from the project root, two folders above this one.
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT_DIR=$1
shift
PYTHON=${PYTHON:-python}
SIM=build/sim.exe
NEW_FEATURE_SWITCHES=() # "new" adds these to coord.
NEW_MODE=new # The "new" variant shows under this mode.

# Write the traces of seed 0, and list the extra ones for the merge. Each extra trace comes with
# its suffix, note and mode. An empty suffix adds nothing to the tab name, an empty note keeps
# the description of the scenario, and an empty mode keeps the mode that the trace names.
"$SIM" --avoid game --trace "$OUT_DIR/t_game.json" > /dev/null
"$SIM" --avoid coord --trace "$OUT_DIR/t_coord.json" > /dev/null
extra_arguments=(
    "$OUT_DIR/t_coord.json" "" "" ""
)
if [ "${#NEW_FEATURE_SWITCHES[@]}" -gt 0 ]; then
    "$SIM" --avoid coord "${NEW_FEATURE_SWITCHES[@]}" --trace "$OUT_DIR/t_new.json" > /dev/null
    extra_arguments+=("$OUT_DIR/t_new.json" "" "" "$NEW_MODE")
fi

# Write the traces of each chosen run, named by its place in the list of pairs.
pair_index=0
while [ "$#" -ge 2 ]; do
    seed=$1
    scenario=$2
    shift 2
    run_arguments=(--seed "$seed" "$scenario")
    trace_prefix="$OUT_DIR/run${pair_index}"
    suffix="seed $seed"
    "$SIM" "${run_arguments[@]}" --avoid game --trace "${trace_prefix}_game.json" > /dev/null
    "$SIM" "${run_arguments[@]}" --avoid coord --trace "${trace_prefix}_coord.json" > /dev/null
    extra_arguments+=(
        "${trace_prefix}_game.json" "$suffix" "" ""
        "${trace_prefix}_coord.json" "$suffix" "" ""
    )
    if [ "${#NEW_FEATURE_SWITCHES[@]}" -gt 0 ]; then
        "$SIM" "${run_arguments[@]}" --avoid coord "${NEW_FEATURE_SWITCHES[@]}" \
            --trace "${trace_prefix}_new.json" > /dev/null
        extra_arguments+=("${trace_prefix}_new.json" "$suffix" "" "$NEW_MODE")
    fi
    pair_index=$((pair_index + 1))
done

# Merge the traces into one page.
"$PYTHON" -I sim/replay/make_replay.py "$OUT_DIR/t_game.json" "$OUT_DIR/replay.html" \
    "${extra_arguments[@]}"
