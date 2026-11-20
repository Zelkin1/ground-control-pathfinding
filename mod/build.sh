#!/usr/bin/env bash
# This script builds the two DLLs of the game mod with the bundled llvm-mingw, and copies
# gcmod.ini next to them in build/mod/dist. The files there are the ones to put in the game folder:
# - wsock32.dll, the loader: a proxy of the system wsock32.dll (src/proxy.c) with the code hooks.
#   The game keeps it open, so it can only be replaced while the game is closed.
#   tools/pathfix/gen_proxy.py writes its export stubs into build/mod/generated once; delete that
#   folder to write them again.
# - gcmod_core.dll, the core: the bridge, the status text and the formation code. The loader runs
#   a copy of it and loads it again when the file changes (src/reload.c), so it can be replaced
#   while the game runs, as can gcmod.ini, which each load of the core reads.
#
# `-msse2 -mfpmath=sse` keeps the float math of the mod in SSE: the game sets the x87 unit to a
# lower precision, which would change the results of the shared simulator code. The game code
# keeps the stack aligned to 4 bytes only, so `-mstackrealign` aligns it in every function before
# SSE spills that need 16 bytes. `GCMOD` selects the frame step and limits of the mod in sim.h.
#
# Sources: the formation code, the base game steps, the clearance fields and the switches come from
# src/ at the project root, which the simulator compiles too.
set -euo pipefail
cd "$(dirname "$0")/.."
COMPILER=toolchain/llvm-mingw/bin/i686-w64-mingw32-clang
PYTHON=${PYTHON:-python}           # Python 3 for gen_proxy.py; set PYTHON to choose another one
GENERATED_DIR=build/mod/generated
DIST_DIR=build/mod/dist
COMPILE_FLAGS=(-std=c11 -O2 -msse2 -mfpmath=sse -mstackrealign -Wall -Wextra -DGCMOD)
INCLUDE_FLAGS=(-Imod/src -Isrc -Isrc/game -Isrc/formation -I"$GENERATED_DIR")
LOADER_LINK_FLAGS=(-shared -static -s)
CORE_LINK_FLAGS=(-shared -static -s -lgdi32 -luser32)
LOADER_SOURCES=(mod/src/proxy.c mod/src/mod.c mod/src/patch.c mod/src/hooks.c mod/src/reload.c)
CORE_SOURCES=(
    mod/src/core.c mod/src/heap.c mod/src/config.c mod/src/bridge.c mod/src/overlay.c
    src/switches.c src/maps.c src/workers.c
    src/game/game.c src/game/mover_steps.c src/game/reroute.c
    src/formation/avoidance.c src/formation/route.c src/formation/smoothing.c
    src/formation/lanes.c src/formation/formation.c src/formation/steering.c
    src/formation/coordination.c src/formation/tuning.c src/formation/stamping.c
    src/formation/shift.c src/formation/passing.c src/formation/detour.c
)

# Write the export stubs of the proxy when they are missing.
if [ ! -f "$GENERATED_DIR/proxy.def" ]; then
    "$PYTHON" -I tools/pathfix/gen_proxy.py wsock32
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
