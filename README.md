# Ground Control movement mod and simulator

This project changes how squads move in Ground Control Anthology v1.0.1.1. In the base game, the units of a squad follow one route and often get in each
other's way. The mod makes each squad drive in formation along a smooth route, keeps units apart
with ORCA avoidance (optimal reciprocal collision avoidance), and coordinates the squads: each
moving squad predicts the paths of the others, and it re-routes around them or slows down early
to avoid collisions.

The project has two programs that share most of their code:

- The **simulator** (`build/sim.exe`) runs test scenarios on the real maps of the game outside the
  game. It calls the real path search and formation code of `gc.exe` and rebuilds the rest of the
  base game movement, so it can compare the base game with the mod over many runs.
- The **mod** (`wsock32.dll` and `gcmod_core.dll`) loads into the game and runs the same
  formation code on the units of the game.

## Setup

1. Install the game from GOG or Steam, which have the same `gc.exe`. The simulator reads it from
   the default GOG folder (`sim.exe --exe PATH` reads another copy), and the tools read it from
   the default Steam folder.
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
bash simulator/build.sh                    # builds build/sim.exe
build/sim.exe --avoid coord --seed 3 ridge # one scenario in the coordination mode
python -I simulator/tools/compare.py 20 "off=" "on=--wholeparked,on" -- --avoid coord
python -I simulator/tools/compare_builds.py OLD.exe build/sim.exe
```

`--avoid` picks the movement mode: `original` (the base game), `form` (formation following) or
`coord` (formation following and coordination). `docs/SWITCHES.md` lists the switches that turn
each feature on or off. `simulator/replay/build_replay.sh` builds the replay page from simulator
traces.

## Mod

```bash
bash mod/build.sh                          # builds build/mod/dist
```

Copy `wsock32.dll`, `gcmod_core.dll` and `gcmod.ini` from `build/mod/dist` into the game folder.
Replace `wsock32.dll` only while the game is closed; the core and `gcmod.ini` can be copied while
the game runs, and the mod loads them again. The mod writes `gcmod.log` next to `gc.exe`, and a
key (F9 by default) turns it on or off during a mission. During a mission, text at the top right
of the screen shows the state of the mod and its keys; F10 shows or hides this text.

### Screen sizes and interface

- **Modern screen sizes** (`modern_resolution`): the video menu of the game offers sizes only up
  to 1024x768. The mod adds 1920x1080, 2560x1440 and 3840x2160, each one only when the display
  offers it and it is not larger than the Windows desktop. The game saves only its own sizes, so
  the mod writes the last modern size into the `resolution` line of `gcmod.ini` and starts the
  game in it.
- **Interface scale** (`ui_scale`): at a modern size, a number above 1 makes the menus, the
  in-game interface that many times larger.

### Mouse

- **Camera turn with the middle button** (`middlerotate`): while the middle mouse button is held
  in the 3D view, moving the mouse to the left or right turns the camera around, and moving it up
  or down tilts the camera. The cursor stays where the button went down, and the screen borders
  no longer turn the camera. `middlerotate_sensitivity` scales the turn (1 by default), and the
  rotation speed option of the game scales it too.
- **Click to deselect** (`clickdeselect`): a left click on ground with no unit under the cursor
  deselects every squad, so that the next orders go only to the squads selected after it. A click
  within 1 m of the body of a unit still selects its squad. With the game option
  `leftclickmovement` on, a left click on empty ground no longer moves the selected squads.

Each of these features has its line in `gcmod.ini`, where "off" keeps the behavior of the game.
