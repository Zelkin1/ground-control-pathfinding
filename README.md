# Ground Control: Second Drop

Ground Control: Second Drop is an unofficial remastering work on Ground Control Anthology v1.0.1.1.
For now, the work focuses on gameplay. It is a mod that loads into the game and changes these
parts of it:

- Movement: squads drive in formation along smooth routes, and moving squads avoid each other.
- Orders and combat: backward moves for vehicles, run-over kills of enemy infantry, and no
  slowdown of units that are hit by fire.
- Screen and interface: modern screen sizes, and a scale for the interface.
- Controls: a camera turn with the middle mouse button, and a left click on empty ground that
  deselects every squad.
- Music: the music of the mission on repeat.

The [Features](#features) section describes each part and its line in `gcmod.ini`, the
configuration file of the mod.

## Install

1. Download the zip file of the latest release from the Releases page of this repository.
2. Extract the zip file into the game folder, next to `gc.exe`. The zip file holds
   `wsock32.dll`, `gcmod_core.dll`, and `gcmod.ini`.
3. Start the game.

The mod has two code files. The loader, `wsock32.dll`, loads into the game when the game starts.
The loader then loads the core, `gcmod_core.dll`, which holds the features. The core reads
`gcmod.ini` each time that it loads, so after you edit `gcmod.ini`, start the game again. The mod
writes its log to `gcmod.log`, next to `gc.exe`.

During a mission, F9 turns the mod on or off, and the `toggle_key` line of `gcmod.ini` can set
another key. F10 shows or hides the status text at the top right of the screen. The status text
shows whether the mod is on, the frame rate, the local time, and the time of the last core load.

## Features

Each feature has a line in `gcmod.ini`. For a feature that turns on or off, `off` keeps the
behavior of the base game. `docs/SWITCHES.md` describes most of the on/off lines in more detail.

### Movement

In the base game, the units of a squad follow one route and often get in each other's way. The
mod moves each ground squad that gets a move order in formation along a smooth route, and each
unit drives on its own lane beside the route. The mod keeps the units apart with ORCA avoidance (optimal reciprocal
collision avoidance), which adjusts the velocity of each unit. In a passage that is too narrow for
the lanes, a squad of vehicles drives in a column, but a squad of infantry keeps its formation.

The `avoid` line selects the movement mode. `form` gives formation following with ORCA
avoidance, and `coord`, the default, adds the coordination between squads.

With `coord`, the mod predicts the paths of the other moving squads for each moving squad. It then
re-routes the squad around them, or it slows the squad down early to avoid collisions. Two squads
that meet head-on each move to their own side and pass close to each other. In both modes, a new
order gets a route around the squads that stand still (`avoidparked`).

### Orders and combat

- Backward moves (`reverse`): hold Alt while you give or queue a move order for a squad of
  vehicles, and the `reverse_key` line can set `ctrl` or `shift` instead. A unit whose rear does
  not point along its way first turns on the spot. Each unit then drives backward at its own
  pace and at a lower speed, along the route of the base game. The squad keeps no formation
  and does not coordinate with other squads. At the destination, the units face away from it.
  Squads with infantry and squads of the computer always move forward.
- Run-over kills (`crush`): a ground vehicle that drives fast enough kills each enemy foot unit
  that it drives over. The game counts the kill for the squad of the
  vehicle, as for a weapon kill. The ORCA avoidance of a vehicle leaves out enemy infantry, so the
  vehicle no longer steers around it. Own and allied infantry are safe, and so are the units that
  a mission makes invulnerable.
- No hit slowdown (`disablehitslow`, on by default): in the base game, a unit under fire drives
  slower and turns its turret slower for a few seconds. `disablehitslow = on` removes both
  slowdowns.

### Screen and controls

- Modern screen sizes (`modern_resolution`): the video menu of the game offers only small screen
  sizes. The mod adds modern sizes to the menu. It adds each size only if the display supports it
  and the size is not larger than the Windows desktop. The game saves
  only its own sizes, so the mod writes the last modern size to the `resolution` line of
  `gcmod.ini`. At the next start, the mod starts the game at that size.
- Interface scale (`ui_scale`): at a modern screen size, the mod makes the menus, the in-game
  interface, and the cursor larger by the factor of this line.
- Camera turn with the middle mouse button (`middlerotate`): while you hold the middle mouse
  button in the 3D view, mouse movement turns the camera. Movement to the left or right turns the
  camera as the left or right screen border does in the base game. Movement up or down tilts the
  camera as the top or bottom border does. The cursor stays where you pressed the button, and the
  screen borders no longer turn the camera. The `middlerotate_sensitivity` line multiplies the
  turn, and the rotation speed option of the game also scales it.
- Click to deselect (`clickdeselect`): a left click on ground with no unit under the cursor
  deselects every squad. Your next orders then go only to the squads that you select after the
  click. A click on or near a unit still selects the squad of that unit. If the
  option `leftclickmovement` of the game is on, the click does not move the selected squads.

### Music

While a mission loads, the game plays one of its music tracks, chosen at random. A few
seconds after the mission starts, the game fades that track into the ambient sound of the
mission, such as desert wind or jungle. With `music_loop = on` (off by default), the track keeps
playing with the ambient sound and starts again each time it ends, until the mission ends. The
music volume option of the game applies to it. A music track that the mission itself starts
still replaces it.

## Build from source

The project holds two programs that share most of their code, the mod and the simulator. The
simulator (`build/sim.exe`) runs test scenarios on the maps of the game, outside the game. It
calls the path search and formation code of `gc.exe` and rebuilds the rest of the movement of the
base game. It also runs the same formation following as the mod, so it can compare the base game
with the mod over many runs.

### Setup

1. Install the game from GOG or Steam. Both editions have the same `gc.exe`. The simulator reads
   it from the default GOG folder, `C:\Program Files\GOG Galaxy\Games\Ground Control`, and
   `--exe PATH` names another copy. `decompilation/tools/gcdis.py` and `mod/tools/loadtest.py`
   read the game from the default Steam folder.
2. Install Git for Windows. Run each command of this file in Git Bash, from the project root.
3. Install Python 3, and make sure that `python` starts it in Git Bash. If the environment
   variable `PYTHON` names a program, the build scripts use that program instead.
4. Install capstone for the disassembler `decompilation/tools/gcdis.py`:
   `pip install --target tools/pylib capstone==5.0.9`.
5. Download the llvm-mingw release 20261006, `llvm-mingw-20261006-msvcrt-x86_64.zip`, from
   https://github.com/mstorsjo/llvm-mingw/releases.
6. Extract it to `toolchain/llvm-mingw`, so that the compiler is
   `toolchain/llvm-mingw/bin/i686-w64-mingw32-clang.exe`.
7. Extract the maps of the game for the simulator:
   `python -I tools/sdf.py extract "<game folder>/Missions.sdf" build/sdf/Missions`. The
   simulator reads the maps from `build/sdf/Missions`, and `--maps DIR` names another folder.

### Build the mod

```bash
bash mod/build.sh   # writes wsock32.dll, gcmod_core.dll and gcmod.ini to build/mod/dist
```

Copy the three files from `build/mod/dist` into the game folder, next to `gc.exe`. If you changed
your own `gcmod.ini`, copy your changed lines into the new file. Replace `wsock32.dll` only while
the game is closed, because Windows keeps the file open while the game runs. You can replace
`gcmod_core.dll` while the game runs. The loader then loads the new core between two frames, and
the new core reads `gcmod.ini` again.

### Run the simulator

```bash
bash simulator/build.sh                      # writes build/sim.exe
build/sim.exe --avoid coord --seed 3 ridge   # runs the scenario ridge with seed 3
python -I simulator/tools/compare.py 20 "off=--wholeparked,off" "on=" -- --avoid coord
python -I simulator/tools/compare_builds.py OLD.exe build/sim.exe
bash simulator/replay/build_replay.sh build/replay 3 ridge
```

The `--avoid` option selects the movement mode: `original` (the base game), `form`, or `coord`.
`compare.py` runs each setup with the seeds 1 to 20 and prints the averages of the results. In
the example, one setup turns `--wholeparked` off and the other one keeps the defaults.
`compare_builds.py` makes sure that two builds give the same output for the same arguments.
`build_replay.sh` writes the replay page `build/replay/replay.html` from new simulator traces,
with one more tab for the run of the scenario ridge with seed 3. `docs/SWITCHES.md` lists the
on/off switches of the simulator and the mod.
