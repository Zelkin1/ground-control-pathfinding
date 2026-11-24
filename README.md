# Ground Control: Second Drop

**Ground Control: Second Drop** is an unofficial remaster mod for **Ground Control Anthology v1.0.1.1**, currently focused on gameplay improvements.

The mod adds formation movement and coordination between squads, reverse movement for vehicles, infantry crushing, modern display resolutions, interface scaling, and updated camera controls. It can also remove the movement and turret penalties caused by incoming fire and loop mission music.

## Installation

1. Download the latest release ZIP from this repository’s Releases page.
2. Extract these three files into your game folder, alongside `gc.exe`:
   - `wsock32.dll`
   - `gcmod_core.dll`
   - `gcmod.ini`
3. Launch the game.

The mod loads automatically at startup. It reads its settings from `gcmod.ini` and writes diagnostic information to `gcmod.log`, both in the game folder. Restart the game after editing the configuration.

### In-game shortcuts

| Shortcut | Action |
| --- | --- |
| **F9** | Enable or disable the mod during a mission. Change this key with `toggle_key` in `gcmod.ini`. |
| **F10** | Show or hide the status overlay in the upper-right corner. |
| **Alt + move order** | Order vehicles to reverse, including queued orders. Requires `reverse`; change the modifier with `reverse_key`. |
| **Hold middle mouse button** | Rotate and tilt the camera in the 3D view. Requires `middlerotate`. |

The status overlay shows whether the mod is enabled, the frame rate, the local time, and when the core was last loaded.

## Features and configuration

Configure the features below in `gcmod.ini`. For on/off settings, `off` preserves the base game’s behavior. See [docs/SWITCHES.md](docs/SWITCHES.md) for more detail on most feature switches.

### Formation movement and squad coordination

In the base game, squad members follow a shared route and often obstruct one another. The mod gives each unit its own lane along a smooth route, allowing ground squads to move in formation.

Units avoid one another using **Optimal Reciprocal Collision Avoidance (ORCA)**, which adjusts their velocities to prevent collisions. When a passage is too narrow for the formation, vehicle squads switch to a column. Infantry squads retain their formation.

The `avoid` setting selects the movement mode:

| Value | Behavior |
| --- | --- |
| `form` | Formation movement with ORCA avoidance. |
| `coord` **(default)** | Formation movement with ORCA avoidance and coordination between moving squads. |

In `coord` mode, squads predict one another’s paths and either reroute or slow down in advance to avoid collisions. Squads approaching head-on move to opposite sides to pass one another.

In both modes, new move orders plan routes around stationary squads through `avoidparked`.

### Reverse movement

**Setting:** `reverse`  
**Modifier:** `reverse_key` — Alt by default; also accepts `ctrl` or `shift`.

Hold the modifier while issuing or queuing a move order to make a vehicle squad drive backward. Vehicles first turn in place if needed to align their rear with the direction of travel, then reverse at reduced speed. At the destination, they face away from the direction of travel.

Reverse movement uses the base game’s routes. Each vehicle moves at its own pace, without formation movement or coordination between squads. Squads containing infantry and computer-controlled squads always move forward.

### Infantry crushing

**Setting:** `crush`

A ground vehicle moving fast enough can run over and kill enemy infantry. The kill is credited to the vehicle’s squad, just like a weapon kill.

Vehicle ORCA avoidance ignores enemy infantry, allowing vehicles to drive over them instead of steering around them. Friendly infantry, allied infantry, and units made invulnerable by a mission are protected.

### Remove hit slowdown

**Setting:** `disablehitslow` — **on by default**.

In the base game, incoming fire temporarily reduces a unit’s movement speed and turret rotation speed. Set `disablehitslow = on` to remove both penalties.

### Modern resolutions and interface scaling

**Settings:** `modern_resolution`, `resolution`, `ui_scale`

`modern_resolution` adds modern resolutions to the game’s video menu. Only resolutions supported by the display and no larger than the Windows desktop are offered.

Because the game can only save its original resolution options, the mod stores the last selected modern resolution in the `resolution` entry of `gcmod.ini` and restores it at the next launch.

At modern resolutions, `ui_scale` sets the scale factor for menus, the in-game interface, and the cursor.

### Middle-mouse camera control

**Settings:** `middlerotate`, `middlerotate_sensitivity`

Hold the middle mouse button in the 3D view and move the mouse to control the camera:

- Move left or right to rotate.
- Move up or down to tilt.

These movements correspond to the base game’s screen-edge camera controls. The cursor stays where you pressed the button, and screen-edge camera rotation is disabled.

`middlerotate_sensitivity` multiplies the camera movement. The game’s rotation speed setting also applies.

### Click to deselect

**Setting:** `clickdeselect`

Left-click empty ground to deselect all squads. Subsequent orders apply only to squads you select afterward. Clicking on or near a unit still selects its squad.

If the game’s `leftclickmovement` option is enabled, clicking empty ground deselects your squads without issuing a move order.

### Loop mission music

**Setting:** `music_loop` — **off by default**.

The game normally plays a random music track while a mission loads, then fades it out shortly after the mission begins, leaving only ambient sound.

Set `music_loop = on` to keep that track playing alongside the ambience and repeat it until the mission ends. The game’s music volume setting still applies. Music triggered by the mission itself replaces the looping track.

## Building from source

The project includes two programs that share most of their code:

- **The mod**, which runs inside the game.
- **The simulator** (`build/sim.exe`), which runs test scenarios on game maps outside the game.

The simulator calls the pathfinding and formation code from `gc.exe` and recreates the rest of the base game’s movement system. It also runs the mod’s formation movement code, allowing repeated comparisons between the original and modified behavior.

### Development setup

Run all commands below in **Git Bash**, from the project root.

1. **Install Ground Control from GOG or Steam.** Both editions use the same `gc.exe`.
   - The simulator looks for the game in `C:\Program Files\GOG Galaxy\Games\Ground Control` by default. Use `--exe PATH` to specify another executable.
   - `decompilation/tools/gcdis.py` and `mod/tools/loadtest.py` use the default Steam installation folder.
2. **Install Git for Windows** to provide Git Bash.
3. **Install Python 3** and make sure `python` works in Git Bash. To use a different Python executable, set the `PYTHON` environment variable; the build scripts use it when set.
4. **Install Capstone** for the disassembler, `decompilation/tools/gcdis.py`:

   ```bash
   pip install --target tools/pylib capstone==5.0.9
   ```

5. **Download llvm-mingw release 20261006** from the [llvm-mingw releases page](https://github.com/mstorsjo/llvm-mingw/releases). Use `llvm-mingw-20261006-msvcrt-x86_64.zip`.
6. **Extract the toolchain** to `toolchain/llvm-mingw`. The compiler should be at:

   ```text
   toolchain/llvm-mingw/bin/i686-w64-mingw32-clang.exe
   ```

7. **Extract the game maps** for the simulator, replacing `<game folder>` with your installation path:

   ```bash
   python -I tools/sdf.py extract "<game folder>/Missions.sdf" build/sdf/Missions
   ```

   The simulator reads maps from `build/sdf/Missions` by default. Use `--maps DIR` to specify another directory.

### Build and install the mod

```bash
bash mod/build.sh
```

The build writes `wsock32.dll`, `gcmod_core.dll`, and `gcmod.ini` to `build/mod/dist`. Copy all three files into your game folder, alongside `gc.exe`. If you have customized `gcmod.ini`, transfer your settings to the new configuration file.

### Reload the core during development

The mod has two DLLs: `wsock32.dll` is the loader, and `gcmod_core.dll` contains the features.

- **Loader updates require the game to be closed.** Windows keeps `wsock32.dll` open while the game is running.
- **Core updates can be applied while the game is running.** Replace `gcmod_core.dll`; the loader loads the new core between frames. The core rereads `gcmod.ini` each time it loads.

### Build and run the simulator

Build the simulator:

```bash
bash simulator/build.sh
```

Run the `ridge` scenario with coordination enabled and random seed 3:

```bash
build/sim.exe --avoid coord --seed 3 ridge
```

The simulator’s `--avoid` option accepts `original` for base-game movement, `form` for formation movement, or `coord` for formation movement with squad coordination.

### Compare simulations

Compare two configurations across seeds 1–20 and print their average results:

```bash
python -I simulator/tools/compare.py 20 "off=--wholeparked,off" "on=" -- --avoid coord
```

This example disables `--wholeparked` in the `off` configuration and retains the defaults in the `on` configuration.

Check that two simulator builds produce identical output for the same arguments:

```bash
python -I simulator/tools/compare_builds.py OLD.exe build/sim.exe
```

### Generate a replay page

```bash
bash simulator/replay/build_replay.sh build/replay 3 ridge
```

This generates new simulator traces and writes `build/replay/replay.html`, including an additional tab for the `ridge` scenario with seed 3.

See [docs/SWITCHES.md](docs/SWITCHES.md) for the simulator and mod’s on/off switches.
