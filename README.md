# Ground Control movement mod and simulator

This project changes how squads move in Ground Control (2000, Steam "Ground Control Anthology",
`gc.exe` 1.0.1.1). In the base game, the units of a squad follow one route and often get in each
other's way. The mod makes each squad drive in formation along a smooth route, keeps units apart
with ORCA avoidance (optimal reciprocal collision avoidance), and coordinates squads that meet at
crossings, head-on, or while passing.

The project has two programs that share most of their code:

- The **simulator** (`build/sim.exe`) runs test scenarios on the real maps of the game outside the
  game. It calls the real path search and formation code of `gc.exe` and rebuilds the rest of the
  base game movement, so it can compare the base game with the mod over many runs.
- The **mod** (`wsock32.dll` and `gcmod_core.dll`) loads into the game and runs the same
  formation code on the units of the game.

## Layout

| Folder | What it holds |
|---|---|
| `src/` | C code that both the simulator and the mod compile: the shared header `sim.h`, the switches, the clearance fields of the maps, the worker threads, the base game steps in `src/game/`, and formation following with the coordination between squads in `src/formation/`. |
| `sim/` | The simulator alone: its main loop, scenarios, metrics and rebuilt base game movement in `sim/src/`, the replay page in `sim/replay/`, and the test scripts in `sim/tools/`. |
| `mod/` | The game mod: the loader and core DLL sources in `mod/src/`, the build script, and the settings file `gcmod.ini`. |
| `docs/` | Reference documents: the reverse-engineered game details (`PATHFINDING.md`), the switches (`SWITCHES.md`), the constants (`PARAMETERS.md`) and the changes that could affect gameplay (`GAMEPLAY_NOTES.md`). |
| `tools/` | Python tools to read the game archives, disassemble and decompile `gc.exe`, and test the mod DLLs. |
| `NOTES.md` | The research notes: findings, measurements, decisions and rejected features. |

`build/`, `toolchain/`, `tools/pylib/` and `re/gc_decompiled.c` are not in the repository. The
build scripts create `build/`, and the steps below create the others.

## Setup

1. Install the game from Steam. The simulator and the tools read `gc.exe` from the default Steam
   folder (`sim.exe --exe PATH` reads another copy).
2. Extract the maps for the simulator, which reads them from `build/sdf/Missions` (`--maps DIR`
   reads another folder):
   `python -I tools/sdf.py extract "<game folder>/Missions.sdf" build/sdf/Missions`.
3. Download llvm-mingw (release 20261006, `llvm-mingw-20261006-msvcrt-x86_64.zip`) from
   https://github.com/mstorsjo/llvm-mingw/releases and extract it to `toolchain/llvm-mingw`.
4. Use Python 3 for the tools. The disassembler needs capstone:
   `pip install --target tools/pylib capstone==5.0.9`.
5. Run the scripts from the project root in Git Bash.

## Simulator

```bash
bash sim/build.sh                          # builds build/sim.exe
build/sim.exe --avoid coord --seed 3 ridge # one scenario in the coordination mode
python -I sim/tools/compare.py 20 "off=" "on=--parkedbox,on" -- --avoid coord
python -I sim/tools/compare_builds.py OLD.exe build/sim.exe
```

`--avoid` picks the movement mode: `original` (the base game), `form` (formation following) or
`coord` (formation following and coordination). `docs/SWITCHES.md` lists the switches that turn
each feature on or off. `sim/replay/build_replay.sh` builds the replay page from simulator traces.

## Mod

```bash
bash mod/build.sh                          # builds build/mod/dist
```

Copy `wsock32.dll`, `gcmod_core.dll` and `gcmod.ini` from `build/mod/dist` into the game folder.
Replace `wsock32.dll` only while the game is closed; the core and `gcmod.ini` can be copied while
the game runs, and the mod loads them again. The mod writes `gcmod.log` next to `gc.exe`, and a
key (F9 by default) turns it on or off during a mission.
