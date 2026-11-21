#!/usr/bin/env bash
# This script builds the two DLLs of the game mod with the bundled llvm-mingw, and copies
# gcmod.ini next to them in build/mod/dist. The files there are the ones to put in the game folder:
# - wsock32.dll, the loader: a proxy of the system wsock32.dll (mod/src/proxy.c) with the code
#   hooks. The game keeps it open, so it can only be replaced while the game is closed.
#   mod/tools/gen_proxy.py writes its export stubs into build/mod/generated once; delete that
#   folder to write them again.
# - gcmod_core.dll, the core: the bridge, the HUD text and the formation code. The loader runs
#   a copy of it and loads it again when the file changes (mod/src/reload.c), so it can be replaced
#   while the game runs, as can gcmod.ini, which each load of the core reads.
#
# `-msse2 -mfpmath=sse` keeps the float math of the mod in SSE: the game sets the x87 unit to a
# lower precision, which would change the results of the shared simulator code. The game code
# keeps the stack aligned to 4 bytes only, so `-mstackrealign` aligns it in every function before
# SSE spills that need 16 bytes. `GCMOD` selects the frame step and limits of the mod in sim.h.
#
# Sources: the formation code, the base game steps, the clearance fields and the switches come from
# shared/, which the simulator compiles too.
set -euo pipefail
cd "$(dirname "$0")/.."
COMPILER=toolchain/llvm-mingw/bin/i686-w64-mingw32-clang
PYTHON=${PYTHON:-python}           # Python 3 for gen_proxy.py; set PYTHON to choose another one
GENERATED_DIR=build/mod/generated
DIST_DIR=build/mod/dist
COMPILE_FLAGS=(-std=c11 -O2 -msse2 -mfpmath=sse -mstackrealign -Wall -Wextra -DGCMOD)
INCLUDE_FLAGS=(-Imod/src -Ishared -Ishared/game -Ishared/formation -I"$GENERATED_DIR")
LOADER_LINK_FLAGS=(-shared -static -s)
CORE_LINK_FLAGS=(-shared -static -s -lgdi32 -luser32)
LOADER_SOURCES=(mod/src/proxy.c mod/src/mod.c mod/src/patch.c mod/src/hooks.c mod/src/reload.c)
CORE_SOURCES=(
    mod/src/core.c mod/src/heap.c mod/src/config.c mod/src/bridge.c mod/src/overlay.c
    mod/src/group_order.c mod/src/ground_click.c mod/src/middle_rotate.c mod/src/resolution.c
    mod/src/ui_scale.c shared/switches.c shared/maps.c
    shared/workers.c
    shared/game/game.c shared/game/mover_steps.c shared/game/reroute.c shared/game/moving_ahead.c
    shared/formation/avoidance.c shared/formation/route.c shared/formation/smoothing.c
    shared/formation/lanes.c shared/formation/formation.c shared/formation/steering.c
    shared/formation/coordination.c shared/formation/tuning.c shared/formation/stamping.c
    shared/formation/detour.c
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
