#!/usr/bin/env bash
# This script builds build/sim.exe with the bundled llvm-mingw. The exe is linked at 0x10000
# without ASLR (address space layout randomization, which loads an exe at a random address), so
# that the real gc.exe image fits at 0x400000 inside the .bss of sim.exe (see
# simulator/src/game/game_image.c).
# `-msse2 -mfpmath=sse` makes the float math of the simulator itself use SSE, not the x87 unit.
# TODO: confirm why. Code hooked into the game needs SSE to keep full precision, because the game
# sets the x87 precision mode, but it is not known whether the simulator needs it too.
#
# Sources: shared/ holds the code that the game mod compiles too: sim.h, the switches, the
# clearance fields of the maps, the worker threads, the base game steps in shared/game/ and
# formation following with the coordination between squads in shared/formation/. simulator/src/
# holds the simulator alone: the main loop, the unit types, the scenarios, the metrics, the
# timing, and in simulator/src/game/ the rebuilt movement of the original game and the mapping of
# gc.exe.
# `INCLUDE_FLAGS` lets every file include each header by its plain name, such as "sim.h".
#
# Speed: each source file compiles to its own object file in build/obj, all files at the same
# time, and then the objects are linked. The object files take the base names of the sources,
# which are unique over all folders. A file compiles again only when it or a header is newer than
# its object. Delete build/obj after a change of `COMPILE_FLAGS`, because the script does not
# notice that.
set -euo pipefail
cd "$(dirname "$0")/.."
COMPILER=toolchain/llvm-mingw/bin/i686-w64-mingw32-clang
COMPILE_FLAGS=(-std=c11 -O2 -msse2 -mfpmath=sse -Wall -Wextra)
INCLUDE_FLAGS=(-Ishared -Ishared/game -Ishared/formation -Isimulator/src)
LINK_FLAGS=(-static -Wl,--image-base,0x10000 -Wl,--disable-dynamicbase)
SOURCES=(
    simulator/src/sim.c simulator/src/find.c simulator/src/units.c simulator/src/scenarios.c simulator/src/metrics.c
    simulator/src/timing.c simulator/src/game/movement.c simulator/src/game/physics.c simulator/src/game/squad.c
    simulator/src/game/game_image.c simulator/src/game/boarding.c
    shared/maps.c shared/workers.c shared/switches.c
    shared/game/game.c shared/game/mover_steps.c shared/game/reroute.c shared/game/moving_ahead.c
    shared/formation/avoidance.c shared/formation/route.c shared/formation/smoothing.c
    shared/formation/lanes.c shared/formation/formation.c shared/formation/steering.c
    shared/formation/coordination.c shared/formation/tuning.c shared/formation/stamping.c
    shared/formation/detour.c
)
OBJECT_DIR=build/obj
mkdir -p "$OBJECT_DIR"

# `is_object_current` succeeds when the object file exists and is newer than its source and
# than every header.
is_object_current() {
    local source=$1
    local object=$2
    [ -f "$object" ] && [ "$object" -nt "$source" ] || return 1
    for header in shared/*.h shared/game/*.h shared/formation/*.h; do
        [ "$object" -nt "$header" ] || return 1
    done
}

# Compile the files that changed, all at the same time, and wait for each one.
objects=()
process_ids=()
for source in "${SOURCES[@]}"; do
    base_name=${source##*/}
    object="$OBJECT_DIR/${base_name%.c}.o"
    objects+=("$object")
    if ! is_object_current "$source" "$object"; then
        "$COMPILER" "${COMPILE_FLAGS[@]}" "${INCLUDE_FLAGS[@]}" -c "$source" -o "$object" &
        process_ids+=($!)
    fi
done
is_failed=false
for process_id in "${process_ids[@]}"; do
    wait "$process_id" || is_failed=true
done
if [ "$is_failed" = true ]; then
    echo "build failed" >&2
    exit 1
fi

# Link the objects.
"$COMPILER" "${LINK_FLAGS[@]}" -o build/sim.exe "${objects[@]}"
echo "built build/sim.exe (${#process_ids[@]} of ${#SOURCES[@]} files compiled)"
