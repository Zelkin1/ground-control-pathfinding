#!/usr/bin/env bash
# This script builds the two DLLs of the game mod with the bundled llvm-mingw, and copies
# gcmod.ini next to them in build/mod/dist. The files there are the ones to put in the game folder:
# - wsock32.dll, the loader: a proxy of the system wsock32.dll (mod/loader/proxy.c) with the code
#   hooks. The game keeps it open, so it can only be replaced while the game is closed.
#   mod/tools/gen_proxy.py writes its export stubs into build/mod/generated once; delete that
#   folder to write them again.
# - gcmod_core.dll, the core: the bridge, the HUD text and the formation code. The loader runs
#   a copy of it and loads it again when the file changes (mod/loader/reload.c), so it can be replaced
#   while the game runs, as can gcmod.ini, which each load of the core reads.
#
# `-msse2 -mfpmath=sse` keeps the float math of the mod in SSE: the game sets the x87 unit to a
# lower precision, which would change the results of the shared simulator code. The game code
# keeps the stack aligned to 4 bytes only, so `-mstackrealign` aligns it in every function before
# SSE spills that need 16 bytes. `GCMOD` selects the frame step and limits of the mod in sim.h.
#
# Sources, one folder for each part of the game that the mod changes:
# - mod/pathfinding/: squad movement. The simulator compiles this folder too, except the bridge to
#   the units of the game (bridge.c) and the layout of group orders (group_order.c).
# - mod/interface/: screen sizes, interface scale, camera turn and click to deselect.
# - mod/gameplay/: backward moves, run-over kills and the hit slowdown.
# - mod/music/: the music on repeat.
# mod/loader/ holds the loader, and mod/core/ the start of the core, gcmod.ini reading, the heap,
# the HUD text and the readers of game objects.
set -euo pipefail
cd "$(dirname "$0")/.."
COMPILER=toolchain/llvm-mingw/bin/i686-w64-mingw32-clang
PYTHON=${PYTHON:-python}           # Python 3 for gen_proxy.py; set PYTHON to choose another one
GENERATED_DIR=build/mod/generated
DIST_DIR=build/mod/dist
PATHFINDING_DIR=mod/pathfinding
COMPILE_FLAGS=(-std=c11 -O2 -msse2 -mfpmath=sse -mstackrealign -Wall -Wextra -DGCMOD)
INCLUDE_FLAGS=(-Imod/core -Imod/loader -I"$PATHFINDING_DIR" -I"$PATHFINDING_DIR/game"
    -I"$PATHFINDING_DIR/formation" -I"$GENERATED_DIR")
LOADER_LINK_FLAGS=(-shared -static -s)
CORE_LINK_FLAGS=(-shared -static -s -lgdi32 -luser32)
LOADER_SOURCES=(mod/loader/proxy.c mod/loader/mod.c mod/loader/patch.c mod/loader/hooks.c
    mod/loader/reload.c)
CORE_SOURCES=(
    mod/core/core.c mod/core/heap.c mod/core/config.c "$PATHFINDING_DIR/bridge.c"
    mod/core/overlay.c "$PATHFINDING_DIR/group_order.c"
    mod/interface/ground_click.c mod/interface/middle_rotate.c mod/interface/resolution.c
    mod/interface/ui_scale.c mod/music/music_loop.c
    mod/gameplay/reverse_order.c mod/gameplay/crush.c mod/gameplay/hit_slowdown.c
    "$PATHFINDING_DIR/switches.c" "$PATHFINDING_DIR/maps.c" "$PATHFINDING_DIR/workers.c"
    "$PATHFINDING_DIR/game/game.c" "$PATHFINDING_DIR/game/mover_steps.c"
    "$PATHFINDING_DIR/game/reroute.c" "$PATHFINDING_DIR/game/moving_ahead.c"
    "$PATHFINDING_DIR/formation/avoidance.c" "$PATHFINDING_DIR/formation/route.c"
    "$PATHFINDING_DIR/formation/smoothing.c" "$PATHFINDING_DIR/formation/lanes.c"
    "$PATHFINDING_DIR/formation/formation.c" "$PATHFINDING_DIR/formation/steering.c"
    "$PATHFINDING_DIR/formation/coordination.c" "$PATHFINDING_DIR/formation/tuning.c"
    "$PATHFINDING_DIR/formation/stamping.c" "$PATHFINDING_DIR/formation/detour.c"
)

# Write the export stubs of the proxy when they are missing.
if [ ! -f "$GENERATED_DIR/proxy.def" ]; then
    "$PYTHON" -I mod/tools/gen_proxy.py wsock32
fi

# Compile and link each DLL in one step, then copy the configuration file.
mkdir -p "$DIST_DIR"
"$COMPILER" "${COMPILE_FLAGS[@]}" "${INCLUDE_FLAGS[@]}" "${LOADER_LINK_FLAGS[@]}" \
    -o "$DIST_DIR/wsock32.dll" "${LOADER_SOURCES[@]}" "$GENERATED_DIR/proxy_exports.S" \
    "$GENERATED_DIR/proxy.def"
"$COMPILER" "${COMPILE_FLAGS[@]}" "${INCLUDE_FLAGS[@]}" "${CORE_LINK_FLAGS[@]}" \
    -o "$DIST_DIR/gcmod_core.dll" "${CORE_SOURCES[@]}"
cp mod/gcmod.ini "$DIST_DIR/gcmod.ini"
echo "built $DIST_DIR/wsock32.dll and $DIST_DIR/gcmod_core.dll"
