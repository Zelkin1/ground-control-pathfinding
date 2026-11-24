# Ground Control 1 pathfinding and movement: reverse-engineering notes

Target: `gc.exe` 1.0.1.1, Steam "Ground Control Anthology"
sha256 `4f5db3501f7f1fd611c98fca00db66cfb690e04e4e810da8883575c3b83a2797`.
Built with MSVC 6, image base `0x400000`, **no relocations** (addresses are fixed), no RTTI for game classes. Methods use `thiscall` (the object pointer `this` is passed in `ecx`).
DirectX is loaded dynamically. Imports include WINMM (3 functions), BINKW32 and IJL11.

The installed `gc.exe` can differ from this hash by 4 bytes in the first 1 KB of the file when a resolution changer has been applied (`gc.resolution-changer-backup.exe` is then the original). Those bytes are most likely the size constants of the 800x600 branch of 0x401140 (see "Screen resolution"); the rest of the code is the same.

Sources of the names: the debug strings in the game itself (`python -I decompilation/tools/gcdis.py names`). Names in *italics* are guesses. `decompilation/tools/ghidra_decompile.py` writes a full Ghidra 12.2 decompile of all 8820 functions (pseudo-C, one `// ==== <address>` header per function). Claims below are read from that code or the disassembly unless marked *inferred*.

Units: distances in meters (m); one grid cell is 6.25 m; times in seconds. `dt` is the frame time (float at `0x82f5cc`, double at `[0x6a8094]+0x1a0/+0x1a8`).

## Overview: when, where and how units move

The server simulates and the client only interpolates (`SV_Session` / `CL_Session`). All movement logic runs on the host.

**When a squad moves.** A squad only starts moving because of one of these:
1. a player order from the network: move, stacked (queued) move, move with facing, attack, set formation (`SV_Session::Parse*`);
2. the next queued order, taken when the current move is finished;
3. its *stance* reacting to a visible enemy while idle or moving: `AGGRESSIVE` squads attack and chase, `CHICKEN` squads run to a random nearby point, `NORMAL` squads only shoot;
4. for computer-controlled squads (`AiSquad`): mission-script orders, idle orders (go to a point, patrol a list, pick a random point, wander, follow a squad), counter-attacks on whoever hit a protected unit in the last 2 s, and a reposition every second while their units cannot fire at the target.

**Where it goes.** The base game path search finds one route for the whole squad, from the front unit (the member nearest to the destination). The destination becomes the row center (the center of the first row) of the formation layout. Member *k* of the member list of the squad gets the final parking position *k* of that layout, where it stops at the destination (the order is reversed when the new facing points backwards). Each member drives its own short path to the route, then the shared waypoints, then straight to its final parking position.

**How each unit moves.** Every tick the `Mover` of each unit follows its waypoint list (skipping ahead when a later waypoint is in sight, re-pathing when the current one is hidden), computes a throttle and a steering value, adds the bias of the base game avoidance (AvoidUnits), turns at its turn rate, applies friction and a speed cap, integrates its position, and then resolves collisions (collision push out of other objects, blocking terrain slide along blocking cells, slope rescue off too-steep ground).

```
CLCMD_MOVE ─► SV_Session::ParseMove 0x4f3a60 ─► Squad::OrderMove 0x50ab30 (order state = 1)
                                                  │ next Squad::Update 0x504950
                                                  ▼
                     Squad::ProcessOrder* 0x50ac20 ─ StartSearchQuiet(front unit → dest), state 13
                                                  │ every tick
                                                  ▼
                     Squad::CalculatingMoveOrderLogic 0x50a6a0 ─ PathFinder::Update(req, 60)
                        │  forward search failed → StartSearch (backward); both failed → "Movement denied"
                        ▼  (done)
                     Squad::SetupCalculatedMove 0x50a7d0
                        │  GetPath → route;  formation setter 0x507ee0 (final parking positions at the last point)
                        │  for each member: Mover::SetMove 0x50e7e0 (route, facing, squad)
                        ▼                         └─ Mover::FindPath 0x50eb60 (budget 200) to the route
                     Squad state 1 (moving) 0x507070 ─ arrival test, straggler timer, next queued order
                     per unit, Mover::Tick 0x50e5b0:
                        Update 0x50eec0 → Steer 0x50ff40 + AvoidUnits 0x5101a0 → Turn → Friction → Integrate → *Physics* 0x510b20
```

## Orders and the squad

### Server order intake (`SV_Session`)

Each handler reads a squad id, looks the squad up (0x4f2ea0) and logs an error for an illegal id.

| handler | what it does | reply to the client |
|---|---|---|
| `ParseMove` 0x4f3a60 | `Squad::OrderMove(dest, facing 0,0, quiet 0)` 0x50ab30 | later: confirm with the list of final parking positions, or deny `0x4d` |
| `ParseStackMove` 0x4f3b20 | `Squad::QueueMove`* 0x505110: appends queued order type 6. Refused when the squad is empty, in hold mode, or the cell of the point blocks the terrain class of the first member | `0x4e` accepted, `0x4f` refused |
| `ParseFaceMove` 0x4f3ca0 / `ParseStackFaceMove` 0x4f3d60 | the same with a facing direction | |
| `ParseStop` 0x4f39c0 | `Squad::Stop(clearQueue 1, 1, 0)` 0x504bd0 | `0x4b` |
| `ParseSetMovement` 0x4f4b10 | `Squad::OrderSetMovementMode` 0x5068a0: `+0x54` = 0, 1 or 2; mode 2 also stops the squad | |
| `ParseSetFormation` 0x4f4c30 | *SetFormation* 0x5069e0: formation type `+0x91`; an idle or moving squad re-issues a direct move to its current destination with its current facing, so the new layout is applied at once | |
| `ParseAttack` 0x4f3ed0 and the stack/multi variants | attack orders (below) | |
| `Squad::OrderSetFireBehaviour` 0x5068f0 | `+0x64` = 0, 1 or 2; 1 and 2 clear all weapon targets | |

Movement modes (`+0x54`): **2 = hold position** (moves and stacked moves are refused; attacks only set weapon targets). 0 and 1 are *offensive* and *defensive* (*inferred* order from the UI strings): in mode 1, moving members whose weapon has a target get a 1 s wait (`mover+0x68 = 1.0`), so they pause to fire. Fire behaviors (`+0x64`), *inferred* from the UI: 0 fire at will, 1 return fire, 2 hold fire.

### Order processing: `Squad::ProcessOrder`* 0x50ac20

Called first in every `Squad::Update` 0x504950. The type of the pending order is `+0x1f0` (3 = nothing pending).
* **1, move:** refused (deny `0x4d`, or silently for a quiet order) when the squad is empty, cannot act (0x5063a0) or is in hold mode. Otherwise:
  * the squad state `+0x30` is saved in `+0x24c`; a non-quiet order clears the order queue; an active special-weapon state (6) is stopped;
  * the search start `+0x204/+0x208` is the position of **the front unit**, the member nearest to the destination;
  * the facing `+0x1fc/+0x200` defaults to destination − start;
  * the forward search `StartSearchQuiet(start, dest, terrain class and corner gap 2 × radius of that member, tolerance 0)` starts, and the squad state becomes 13 (calculating).
* **0:** attack an entity (0x5056d0), replies `0x50` (accepted) / `0x51` (refused).
* **2:** an order on a position through 0x505cc0, replies `0x54` / `0x55` (*inferred*: special equipment used on a point).

`Squad::OrderMove` 0x50ab30 (thiscall: destination pointer, facing x, facing y, quiet byte; `ret 0x10`) only stores the order: destination `+0x1f4/+0x1f8`, facing `+0x1fc/+0x200`, the *quiet* flag `+0x250` (queued and scripted orders: no deny message), order type 1. Its callers are `ParseMove` 0x4f3a60, `ParseFaceMove` 0x4f3ca0 and the queue 0x506aee (quiet).

### Route search over several ticks: `Squad::CalculatingMoveOrderLogic` 0x50a6a0 (state 13)

* Each tick it runs `PathFinder::Update(squad+0x20c, 60)`.
* Done → `SetupCalculatedMove`.
* Search over but not found, and it was the forward search (`StartSearchQuiet`) → the backward search (`StartSearch`) from the same start, but with the terrain class and radius of the *first* member, not of the front unit.
* Both failed → `Reset`; a quiet order stops the squad, a normal one sends `0x4d` ("Movement denied", `moveno*.wav`) and restores the saved state.

### `Squad::SetupCalculatedMove` 0x50a7d0

1. Normalizes the facing (zero when shorter than 1e-5) and stores it as the formation facing `+0x94/+0x98`.
2. `GetPath` into a local list.
3. Formation setter 0x507ee0 at the last point of the route. On failure the move is denied like a failed search.
4. For the mover of each member: `SetReachLast(radius)` (final parking position reached within the radius of the unit, at least 1 m), `SetReachMid(3 × radius)` (ignored, see "Waypoint reach"), *SetTarget(none)* 0x50e740, `+0x3c = +0x5c = 0`, then `Mover::SetMove(route, facing, squad)`.
5. Members with a weapon (`entity+0x5c`) get 0x512f80.
6. State 1 (moving), squad destination `+0x1c/+0x24` = order destination, straggler timer `+0x1c0 = −1`.
7. A non-quiet order sends the move confirmation with every final parking position (the client draws the markers).

### Synchronous squad move: *Squad::MoveDirect* 0x505200

Used by everything except the plain move of the player: queued orders, formation changes, the AI, scripts, attack moves and evasion (through 0x504df0, which also clears the queue on request and sets state 1).
* Same start rule (the front unit) and facing default.
* The forward search (`StartSearchQuiet` + `PathFinder::Update(req, 500)`) runs in one call; if that fails it logs "TRYING BACKWARDS!" and runs the backward search (`StartSearch` + `Update(500)`).
* Formation at the last point of the route unless a target entity is given; an optional maximum route length (in points) can refuse the move.
* Then the same per-member set-up as `SetupCalculatedMove`, with the given target entity and flags (`mover+0x5c`, `+0x3c`).

### Order queue (stacked moves) and squad states

* Queue: list at `+0x38` (head), `+0x3c` (iterator), `+0x40` (tail), `+0x44` (count). Items are 0x2c bytes: point `[0..1]`, facing `[2..3]`, order type `[5]`, entity/squad references.
* *PopQueuedOrder* 0x506a20 runs the next item: order type 0 move (direct), 1 move with facing, 2 attack entity (0x5056d0), 4 attack several, 5 0x50a160, 6 path-searched move (`OrderMove(point, facing, quiet 1)` + `ProcessOrder` at once).
* `Squad::Stop(clearQueue, b, keepHeading)` 0x504bd0: without clearQueue it first tries the next queued order. Otherwise it clears the queue, sets the squad destination to the centroid of the members and the formation facing to their average heading, gives each mover `SetReachLast(7.0)` and stops it where it stands (0x50e690), and sets state 0.
* Squad states `+0x30` handled in `Squad::Update` 0x504950: 0 idle 0x506e50, 1 moving 0x507070, 3 attacking 0x507620, 5 evading (set by 0x505ed0), 6 special weapon (0x5082f0 unsets it), 13 calculating 0x50a6a0; 4, 8, 9, 10, 11, 12 are other activities (0x507a10, 0x5086f0, 0x507a80, 0x509ea0, 0x50a270).

### Arrival of a squad: moving state 0x507070

* With queued orders waiting, the squad counts as arrived as soon as **every member is within 2 × N × r of the destination** (N members, r = radius of the first member). The next order then starts while units are still settling.
* Otherwise the squad waits until every mover reports stopped (`mover+0x54`).
* **Straggler timer:** when the squad has more than 2 members (1 for some squads, flag `+0x19`) and fewer than 2 are still moving, a 20 s timer starts (`+0x1c0 = 20.0`). When it runs out the squad counts as arrived even if a unit is still stuck.
* On arrival: a client squad (`+0xa4`) gets `0x5e` (pop waypoint marker), then `Stop(0,0,0)`, which starts the next queued order or goes idle.

### Final parking positions (formation layout)

* Formation setter 0x507ee0(row center, facing): calls `SquadFormation::CalculateFormationPoints(row center, facing, out, N, radius of the first member, its terrain class, type +0x91)`. On success it replaces the list of final parking positions `+0x28`; when the new facing points against the previous one (dot product < 0) it toggles `+0x60`; it stores the facing in `+0x58/+0x5c`.
* *SlotFor(entity)* 0x508030: the index of the final parking position is the index of the entity in the member vector. With `+0x60` set the final parking positions are counted from the end, so a squad that turns around keeps its members on the same sides instead of crossing over (*inferred* purpose). The game does not give each member the closest final parking position.
* `SquadFormation::CalculateFormationPoints(Vec2 row_center, Vec2 dir, list *out, int n, float radius, char terrain_class, char type)` **0x4e7be0**, **cdecl**; returns 1 when `out` holds exactly n positions. `n <= 0` just empties `out`.
  * Other callers: `Ingameinterface::CalculateSquadFormationPoints` 0x4b1020 (order preview), `SV_Session::CreateClientSquads` / `ReinforceClientSquads` (spawning) and the scripted 0x4f08f0.
  * Layout: row spacing `sp = 3.5·radius` (type 3: 3.4). Positions per row by type: 0 → 2n (one row), 1 and 3 → (int)(√n + 0.9) (box), 2 → 1 (column), other → 4.
  * The rows are perpendicular to `dir`. For each row of `w = min(remaining, cols)` positions, centered at P, the first position is at `P + perp·(w−1)·sp/2` with perp = (dir.y, −dir.x). Each next position steps by `−perp·sp`, and up to `cols` positions are tried per row. P is the center of the current row: it starts at `row_center` and moves by `−dir·sp` per row.
  * A position is kept only if `Raycast(position + 3·unit(position−P), row_center)` is clear (call @ 0x4e7edb). Dropped positions are not replaced in their row, so the formation stretches backwards.
  * It **fails** when a row center P is on a blocking cell (cell value > terrain class, 0x4e82e0), and after 13 rows. The setter then keeps the old formation, and the move order does not go through.

### Positions of squads in a group order of the player

* `Ingameinterface::CalculateSquadFormationPoints` **0x4b1020** (thiscall, the interface object): for the squads of the player with the selected flag (squad `+0x34`) in the squad list (first node at interface `+0x1a4`; node = squad pointer, next node at `+4`), it calls `CalculateFormationPoints(order point +0x10cb8/+0x10cbc, facing +0x10cc8/+0x10ccc, out = list at +0x10cd0, n = selected count, radius = largest squad width, terrain class = smallest of squad +4, type 5)`. Type 5 → 4 positions per row. The squad width is `0x4e7bb0(squad type, member count) ×` the radius of the first member. The list holds n positions (node = x, y floats, next node at `+8`); the count is at `+0x10cdc`.
* Callers: the order preview (0x4b1200 draws one marker per selected squad, the `i`-th position for the `i`-th selected squad) and the order builder 0x4b6f90, which copies the squad ids (squad `+2`, 0x4b16c0) and the positions into the order in the same order. So the `i`-th selected squad gets the `i`-th position, whatever its place: squads can cross. The server side of the order is *inferred* to pair them by index too (the crossing in the base game fits).
* Squad of the interface: member list pointer at `+0x2c` (count at list `+8`; the first node starts with the 16-bit id of the first member, 0x4c6930); client entities by id in the table at **0x6e6320** (600 pointers). `CL_Entity` keeps its dead-reckoned position at `+0x38` and its shown position at `+0x44`, each as x, height, y (`CL_Entity::Interpolate` 0x4bbe10; the x, height, y order is *inferred* from the (x, 0, y) vectors of 0x4b1200).

### Selection of squads by the player

* Mouse object at `[0x69b694]`: byte `+0x24` is the order button now and `+3` the same button in the last frame; `+0x25` and `+2` are the select button. Tests: 0x4c2b50 order button down, 0x4c2b80 order button up, 0x4c2bc0 order button held; 0x4c2af0 select button down, 0x4c2b20 select button up, 0x4c2bb0 select button held (each also true when its flag `+4` to `+7` is set). The select button opens selection boxes and the order button starts orders, so the select button is *inferred* to be the left button and the order button the right one.
* Mouse handler of the 3D view **0x4d09f0** (virtual thiscall method, the interface object at `+4`, one argument: the order that the interface builds; `ret 4`). Interface fields: `+0x9e4` = 16-bit id of the entity under the cursor (600 = none); selection box `+0x10d04` open, `+0x10d05` opened by the select button, `+0x10d08/+0x10d0c` start pixel, `+0x10d10` grown into a drag.
  * A press of the select button with no modifier key opens a box (0x4b6050(interface, 1)).
  * At the release of either button, a dragged box selects the squads inside it (0x4b6550). A box that is not dragged counts as a click: an entity of a player squad under the cursor selects that squad (0x4b6de0, which also handles double clicks), and an enemy gives an order on it when the option `leftclickattack` (32-bit console variable 0x677740) is on.
  * A click with no entity under the cursor: with the option `leftclickmovement` (32-bit console variable 0x67773c) on, the selected squads get a move order to the clicked point (0x4b6750, 0x4b6820, 0x4b1020, 0x4b6f90). Otherwise 0x4af660(interface, 30.0) (thiscall; the handler pushes 30.0 at 0x4d0b75) looks for the entity nearest the cursor, and a player squad found there gets selected; with nothing found, the selection stays as it is. So the base game never deselects on a ground click.
  * 0x4af660 measures in 3D, in meters: from the shown position of each client entity (`+0x44`, x, height, y) to the point of the ground under the cursor (0x6e7748, x, height, y), and keeps the nearest one within the radius. It then calls 0x4af810(interface, id) (thiscall), which writes the id to `+0x9e4`; with id 600, 0x4af810 clears `+0x9e4` and resets the hover state. The radius of 30 m is the same for every unit, while unit bodies have radii of 1 to 6.5 m (client entity `+4` → type `+0x88`).
* `Ingameinterface::SelectSquads(char number, bool selected)` **0x4aee70** (thiscall, `ret 8`): number −1 = every squad with members (0x4c6920). `CL_Squad::SetSelected` 0x4c6870 sets squad `+0x34` and the state of the squad icon (widget `+0x30`).
* Only squads with `+0x34` get orders: the order builder 0x4b6f90 and the keys take the squad ids from 0x4b16c0. With no squad selected, a move order has no squad and is not sent.

### Squad fields used above

| offset | meaning |
|---|---|
| `+0x0c` | current target entity (ref-counted) |
| `+0x14` | member vector: begin `[0]`, count `[2]`, end `[3]`, iterator `[4]` |
| `+0x18` | team, a byte from 0 to 9; the alliance test 0x4e6840 (thiscall, two team byte pointers) tells whether two teams are allies: a team is its own ally; in game modes 0 and 2 (global 0x67faa4) also {0, 2, 4}, {1, 3, 4} and {4, 5, 6}; in mode 1 only the same team or team 8; in other modes none. `Squad::LogAlliedAndEnemyStats` 0x5099a0 uses it |
| `+0x19` | flag set for computer-controlled squads (*inferred*) |
| `+0x1c/+0x20/+0x24` | current destination (x, y, z); after `Stop`, the centroid of the members |
| `+0x28` | list of final parking positions (formation layout) |
| `+0x30` | squad state |
| `+0x34` | stance: 0 `NORMAL`, 1 `AGGRESSIVE`, 2 `CHICKEN` (parsed at 0x519870; 0/1 *inferred* from behavior) |
| `+0x38..+0x44` | order queue |
| `+0x54` | movement mode (2 = hold) |
| `+0x58/+0x5c`, `+0x60` | last formation facing, flip of the order of the final parking positions |
| `+0x64` | fire behavior |
| `+0x80` | attack timer (25 s on a new attack) |
| `+0x91` | formation type |
| `+0x94/+0x98` | formation facing |
| `+0xa4`, `+0xa8` | owning client, squad id |
| `+0x1c0` | straggler timer |
| `+0x1f0..+0x208` | pending order: order type, destination, facing, search start |
| `+0x20c` | PathRequest |
| `+0x24c`, `+0x250` | saved state, quiet flag |

### Reactions to enemies (stance)

* Idle (0x506e50) and moving (0x507070) squads look for targets (0x507ce0, 0x507db0; *inferred*: visible enemies).
* `NORMAL`: only assigns weapon targets (0x505b40), depending on the fire behavior.
* `AGGRESSIVE`: attack order on the target (0x5056d0), which moves the squad to engage. In the attacking state it switches to a new visible target.
* `CHICKEN`: *Evade* 0x505ed0(target, 100): a random point around the first member, offset in relation to the target, at up to 100 m; direct move there, state 5.
* Attack order 0x5056d0: in hold mode it only sets weapon targets. Otherwise it stores the target, optionally clears the queue, and calls 0x505840, which moves the squad with `MoveDirect` and the target entity given to every mover (units then chase it, see "Chasing a target"). State 3, attack timer 25 s.

## AI squads (`AiSquad`)

Computer-controlled squads are driven by the mission scripts (*ScriptEngine*, triggers and actions around 0x500000–0x502500 and 0x515000) plus a small amount of local logic. All their moves use the synchronous `MoveDirect` (budget 500).

* *AiSquad::ExecuteOrder* 0x5037d0 (script orders, "Fallback order set..."): 0 move to a point; 1 attack a list of entities or squads (the first one now, the rest queued); 2 set the lists of units and squads to protect (`+0x274`, `+0x284`).
* *AiSquad::Update* 0x502ae0, then the normal `Squad::Update`:
  * removes dead entries from the protect lists;
  * **counter-attack:** for each protected unit hit within the last 2 s (`entity+0x38` = time of the last hit, `+0x34` = attacker), attack the attacker;
  * **idle order** `+0x268` while the squad is idle: 0 go to a point unless every member is within 70 m of it; 1 patrol a list of points in turn; 2 a random point from the list; 3 a random point within ±R m of a point; 4 follow a squad;
  * **reposition while attacking:** every 1 s, when the blocked-fire counters of the members (`entity+0xb7`, *inferred* meaning) add up to 21 or more, move to a random point near the first member (0–50 plus −5…+25 m along the line from the target, and up to ±75 m sideways) with a route of at most 5 points, then attack again; with a 9% chance a second sideways step follows.

## The path search (`PathFinder`)

### Global PathFinder object `@0x6eb470`

| offset | meaning |
|---|---|
| +0x00 (byte) | side of the last DDA step (0 = y side, else x side); used by the blocking terrain slide (*WallSlide*) |
| +0x04/+0x08 | last raycast hit point (x,y) |
| +0x0c | **passability grid**: 1024×1024 cells, 2 bits per cell, row stride 256 bytes (256 KB) |
| +0x4000c.. | scratch list used by Expand |
| `0x82f4ec` (global float) | 1/cell size; cell = **6.25 m** (`0x6130b8`) |

* `GetCell(x,y)` **0x4e83d0**: `(grid[y*256 + x/4] >> (6 - 2*(x&3))) & 3`
* A cell blocks a mover when `cell > moverClass`, so it is blocking terrain for that mover. The terrain class of the mover (`moverClass`) is a byte at `entity->+0x54->+0xb0`. Class 4 (*aircraft*) skips the search and flies straight.
* `Raycast(x0,y0,x1,y1,moverClass)` **0x4e8400**: an Amanatides–Woo DDA (grid line walk) using 16.16 fixed-point cell coordinates. Returns `-1` when the line is clear, otherwise the blocking cell index `(cy<<10)|cx`, and stores the hit point at +4/+8. The ray has zero width.
* *GridDDA variant* **0x4e868b**, *CellCoords* **0x4e82e0**, *CellPos* **0x4e8380**, *terrain height* **0x4e5e60**.
* `LoadGrid(name)` **0x4e8200** (`ret 4`): reads 256 KB and copies the rows in reverse order into +0x0c (the last row of the file becomes grid row 0). It is called once per mission by both the client (0x4a1640) and the server (0x4ef660). Nothing else writes the grid, so it is static per map. The file is `<mission>/<mission>.SLP` in `Missions.sdf` (name format `%s.slp` @ 0x675dbc); there is one per campaign, tutorial and multiplayer map.
* The inverse cell size (1 / 6.25) at **0x82f4ec** is set by the static initializer **0x4e8130**; the raycast, `CellAt` 0x4e82e0, Expand and the outline traces multiply map coordinates in meters by it.

### PathRequest (0x40 bytes; e.g. `Squad+0x20c`, or on the stack in Mover::FindPath)

| off | type | meaning |
|---|---|---|
| +0x00 | node* | open list head, sorted by f (insert **0x4eba70**, head **0x4db9d0**) |
| +0x04.. | list | closed list (**0x4ebad0** moves a node from open to closed) |
| +0x14/+0x18 | float | search **target** |
| +0x1c | byte | terrain class (`moverClass`) |
| +0x20 | float | corner gap (2 × radius `entity+0x24`) |
| +0x24 | float | goal tolerance² |
| +0x28..+0x34 | list | points already expanded toward (linear scan in Expand, O(n²)) |
| +0x38 | bool | done (found) |
| +0x39 | bool | backward search (set by StartSearch) |
| +0x3a | bool | still in progress (budget ran out this call) |
| +0x3c | int16 | total expansions; **hard cap 500** (`cmp word [edi+3c],1f4` @ 0x4e88ca / 0x4e88da) |

PathNode (0x1c bytes): `+0/+4` open-list links, `+8 g`, `+0xc f`, `+0x10 parent`, `+0x14 x`, `+0x18 y`.

* `Reset` **0x4ebb10**: frees the open and closed lists with operator delete (the node destructor 0x42ba60 is a no-op).
* `StartSearch(a1,a2,a3,a4,terrain_class,corner_gap,tol)` **0x4eb9a0** (`ret 1Ch`, sets `+0x39` = 1): **target = (a1,a2), root = (a3,a4)**.
* `StartSearchQuiet` **0x4eb8d0** (`+0x39` = 0): **root = (a1,a2), target = (a3,a4)**, i.e. the roles are swapped. It stores tol² and clears the expanded-point list.
  Callers pass (unit, destination) to both. So `StartSearch` starts the backward search (from the destination toward the unit), and `StartSearchQuiet` starts the forward search. The squad, `MoveDirect` and `Mover::FindPath` all try the forward search first, then the backward search.
* `GetPath(list*)` **0x4ebbb0**: starts at the open-list head.
  * backward search: `push_back` of head->parent … root (the head is the unit itself and is skipped);
  * forward search: `push_front` of head … (root excluded).

  Both give unit → destination order, ending at the destination. List nodes are 0x10 bytes (`x, y, next, prev`); `push_back` is 0x4b8d80 and `push_front` is 0x4ebc20.
* `operator new` 0x5e89be / `delete` 0x5e89e7 (game CRT heap).

### `PathFinder::Update(req, budget)` 0x4e8760 (`ret 8`)

* Class 4: one node is put at the target and the search is done.
* Otherwise, up to `budget` times: pop the best node, close it, clear the scratch list, `Expand(node, req, node.pos, target)`. Stop when the new head is at the target, within tolerance of it, or after 500 expansions in total.
* Result flags: found → done; budget used up → in progress; open list empty or 500 reached → failed.

### `PathFinder::Expand(node, req, from, to)` 0x4e8920 (recursive)

1. If `to` was already expanded toward, return.
2. `Raycast(from → to, class)`:
   * **clear:** a new node at `to` with `g = node.g + |to − from|` and `f = g + 1.5 · |to − target|` (`fmul [0x6116c4]`); parent = node. The heuristic weight 1.5 makes the search greedy (not guaranteed shortest).
   * **blocked**, and the hit point is within the tolerance of the target: a node at the hit point (it counts as arriving).
   * **blocked** otherwise: *TraceLeft* **0x4e91f0** walks the outline of the blocking terrain to its left end and returns a corner point **offset from the edge of the blocking cell by the corner gap** (2 × radius); if that point lies in another blocking cell, the trace continues from there. Then `Expand(node, req, from, corner)`. The same with *TraceRight* **0x4e9620**.
3. Points are de-duplicated against the scratch list of this expansion and the expanded list of the request.

So the graph is built lazily from the corners of blocking terrain, every edge is checked with a zero-width ray, and only the corners keep the corner gap.

## The unit (`Mover`, `entity+0x58`)

### Fields

* Entity at `+0x74`: position `+0xc/+0x10/+0x14` (x, height, y), heading angle `+0x1c`, radius `+0x24`, `+0x3e` (int16; called team here before, but it differs between the squads of one player: 6, 7 and 10 for three player squads in MISSION4; meaning TODO: confirm), type `+0x54`, mover `+0x58`, weapon `+0x5c`, active flag `+0x88` (*inferred*; Update and Steer are skipped while it is 0), counter `+0xb7`.
* Entity type (`entity+0x54`, the unit type, loaded from the ENT files, see "Unit data"): radius `+0x88`, acceleration `+0x94`, friction `+0x98`, top speed `+0x9c`, turn rate `+0xa4`, terrain class (`moverClass`) `+0xb0`, TurnOffset (close range) `+0xc0`, flag `+0xd9`, MoverType `+0x38` (foot 1, air 2, vehicle 3, hover 4), force `+0x70` (see "Armor profiles and the Mover top speed").
* Mover: owner type `+0x04`; velocity `+0x08/+0x0c`; pending velocity `+0x10/+0x14`; heading `+0x18/+0x1c` = (sin a, cos a); current waypoint `+0x20/+0x24/+0x28`; waypoint list `+0x2c` (head), `+0x30` (iterator), `+0x34` (tail), `+0x38` (count); flags `+0x3c`; top speed `+0x40`; acceleration `+0x48`; target entity `+0x50`; stopped `+0x54`; reach `+0x58`; flag `+0x5c`; final facing `+0x60/+0x64`; wait timer `+0x68`; heading saved for a wait `+0x6c/+0x70`; avoidance turn side `+0x78`; blocker collision object `+0x7c` and entity `+0x80`; reach values `+0x84` (last waypoint), `+0x8c` (other waypoints); phase counter `+0x90`; never re-orient `+0x94`; last waypoint flag `+0x95`; avoid counter `+0x98`; re-orient `+0x9c`; target position `+0xa0/+0xa4/+0xa8`; side-switch cooldown `+0xac`.
* Vtable **0x613498** (a subclass uses 0x6134ec with *Integrate* 0x513010): `[0]` *Tick* 0x50e5b0, `[1]` *Friction* 0x50fdf0, `[2]` *Integrate* 0x5109e0, `[3]` *Turn* 0x511560, `[4]` *SteerTo* 0x511400.

### `Mover::SetMove(list *route, Vec2 facing, Squad*)` 0x50e7e0 (`ret 10h`)

1. Clears the blocker, stores the final facing `+0x60/+0x64`, copies the route of the squad into `+0x2c`.
2. Takes the first point of the route as the current waypoint and, if more points remain, also skips to the second one.
3. `FindPath(current waypoint, tolerance = 1.0, or the reach of the last waypoint if nothing else remains, budget 200)` from the unit.
4. On failure: empty list, current waypoint = the position of the unit itself (the unit does not move).
5. On success, without a target entity: the last point of the route is replaced by the final parking position of this unit (0x508030). The way from the last shared waypoint to the final parking position is not searched.
6. Sets the last waypoint flag and the reach (20.0 for the last waypoint of air units (MoverType 2)), clears stopped, sets re-orient (`+0x9c`, unless `+0x94`), and resets the wait timer to −3.

So **members reuse the route of the squad**: each searches only from itself to the second point of the route. The same unit→waypoint search is often issued twice in one tick (16% of searches in a play log).

### `Mover::FindPath(Vec2 dest, float tol, int budget)` 0x50eb60 (`ret 10h`)

Runs a local PathRequest **synchronously**: the forward search (`StartSearchQuiet`) + `Update(budget)`, then the backward search (`StartSearch`) if that failed. The terrain class and the corner gap (2 × radius) come from the unit. The found path is **put in front of** the remaining waypoints, its first point becomes the current waypoint, and re-orient is set. Budgets passed in:
`0x50e969` 200 (SetMove) · `0x50f044` 32 · `0x50f052` 100 · `0x50f6ee` 40 · `0x50f7fa`/`0x50f894`/`0x50f95f`/`0x50f9f6`/`0x50fa3d` 20 · `0x50fd90` 50.

### *Tick* 0x50e5b0, once per unit per tick

1. Decrement the avoid counter `+0x98` (not below 0).
2. 0x50ee80: a pending velocity `+0x10` replaces the velocity.
3. If the entity is active: `Mover::Update`; then, unless stopped, *Steer* + *AvoidUnits* as the steering value; a stopped unit instead steers toward its final facing (it turns on the spot to face the formation direction). Then *Turn*.
4. *Friction*, then *Integrate*.

### `Mover::Update` 0x50eec0: following the waypoints

* A phase counter at `+0x90` makes each check run every 4th tick:
  * look-ahead: if a zero-width `Raycast` from the unit reaches the next waypoint, skip the current one (call **0x50f3d3**); the next phase does the same with the 2nd next (**0x50f287**, pops 2) and then the 3rd next (**0x50f0e9**, pops 3);
  * visibility: if the current waypoint is not visible, re-path to it with `FindPath` (budget 32 or 100).
* Arrival: when the distance to the current waypoint is within the reach `+0x58`, the next waypoint is taken only if it is visible (0x50f4e6); on the last waypoint the unit is marked stopped (`+0x54`).
* **Chasing a target** (`+0x50` set by an attack move): the position of the target is tracked in `+0xa0` and the final facing points at it.
  * When the weapon of the unit has this target and can fire, and the counter `entity+0xb7` is below 8 × a weapon-type value, the unit stops.
  * Otherwise, every 4th tick, it drives straight at the target when the line is clear, or calls `FindPath` toward it (several fallbacks with reach 10, 5 or 1).
  * When the target is gone, the unit stops where it is.

### *Steer* 0x50ff40: throttle and steering

* `+0x68 -= dt`. While the wait timer is positive the unit steers to the saved heading and **does not accelerate**, so *Friction* brings it to a stop.
* While re-orient `+0x9c` is set it does not accelerate either; the flag clears once the heading is within acos(0.95) ≈ 18° of the current waypoint.
* With `+0x94` set: full throttle.
* With the avoid counter at 0, the waypoint behind (heading · offset < −0.2) and farther than the close-range radius of the type `+0xc0`: half throttle.
* Otherwise the throttle is the **arrival factor** 0x511280, but **never below 0.6**:
  * before the last waypoint: 1;
  * toward the last waypoint, with d = distance to the final parking position and v = speed toward it: k = v² / (2 (d − 3)) / friction, the braking needed compared with friction alone; k < 0.5 → 1; else (1 − 2(k − 0.5)) · friction / acceleration, at least 0.
  * With the real unit data (see "Unit data"), 0.6 × acceleration is below friction for almost every unit, so the floor still decelerates: arrival braking works.
* Acceleration 0x511500: `v += heading · accel(+0x48) · dt · throttle`.
* Steering: 0 when close to the waypoint (within `+0xc0`) and it lies on one side of the velocity (*inferred* purpose: stop circling around a close point); otherwise *SteerTo*.
* *SteerTo(d)* 0x511400: with c = cos of the angle between heading and d: steering = ±(1 − c⁶) when d is ahead, full lock ±1 when it is behind.

### *AvoidUnits* 0x5101a0: the base game avoidance, unit against unit

Returns a bias that is added to the steering value; skipped while re-orienting.
* It queries the collision map (0x4fef30) along the heading, within a radius of `clamp(0.6·dist to waypoint, 1.5 r, 4 r)` (halved when heading away from the waypoint).
* Toward the last waypoint, an object that is not a unit, or a stopped unit, at the destination ends the move where the unit stands.
* Otherwise it picks the nearest blocker (the unit in the way) ahead, then decides:
  * a friendly blocker that is moving, not waiting, has avoid counter ≤ 2 and no target → **return 0**: keep driving into it;
  * if `+0x68 < −2` (cooldown over) and the mover of the blocker outranks ours on `+0x40` (top speed), or its type has flag `+0xd9` → **wait 2 s** (`mov [esi+68h], 2.0` @ 0x5108d7) with the heading saved;
  * else make an avoidance turn: choose a side (copying the choice of the blocker when it is avoiding us) and return **±4**, which is full lock after the clamp in *Turn*.
* The decision only uses current positions, not where the units are going.

### *Turn*, *Friction*, *Integrate*

* *Turn(s)* 0x511560: s is clamped to [−1, 1]; heading angle += turn rate (`type+0xa4`) · dt · s, wrapped at 2π; heading = (sin a, cos a).
* *Friction* 0x50fdf0: `v −= unit(v) · friction(type+0x98) · dt` per axis (a component that changes sign becomes 0); then the speed is capped at `+0x40`.
* *Integrate* 0x5109e0: `pos += v · dt`, clamped to the map (10 … 6390 m); the height comes from the terrain (0x4e5e60); then *Physics* 0x510b20.

### *Physics* 0x510b20: collisions and slopes

1. **Collision push:** if the collision box of the unit overlaps another object in the collision map (0x4fecf0; the query adds 200 to box fields `+0x10` and `+0x14` and takes them back after, meaning TODO: confirm), the unit is pushed out along the line between the centers to touching distance + 0.3 m (0x612004), and its velocity along that line is changed so that it moves apart: a component toward the other below −0.1 m/s (0x6132ac) is removed; otherwise its own speed, or 0.2 m/s when that is below 0.1 m/s (0x6116a0), is added away from the other. If it still overlaps, it goes back to its previous position. Only the unit of this mover moves; the other object is only read. Every unit runs this in its own *Tick*: *Tick* 0x50e5b0 calls *Friction* and *Integrate* for stopped and inactive units too, and *Integrate* always calls *Physics*. So a standing unit that another unit overlaps is pushed out in its own tick, at 0.2 m/s.
2. **Blocking terrain slide:** *WallSlide* 0x511170 raycasts from the old to the new position for the terrain class of the unit. When blocked, the velocity component across the cell side that was hit becomes 0 and the position is moved to 0.55 cells from that cell. This repeats up to three times per tick. A slide flips the preferred avoidance turn side `+0x78` (at most once per 2 s) and sets the avoid counter to 5 ticks.
3. **Slope rescue:** when the cell under the unit is above its terrain class, it logs `"%s IS STANDING IN SLOPE!!!!"` and moves the unit at once to the first cell of free terrain found ±x or ±y at 0.5, 1.0, 1.5 … cells (0x511000).

### Waypoint reach

The arrival check in `Mover::Update` (0x50f47c) compares the distance to the current waypoint with the reach `+0x58`.
* That value comes from `+0x84` for the last waypoint and from `+0x8c` otherwise.
* `Mover::SetReachLast` 0x50ee10 stores max(arg, 1); orders pass the radius of the unit, `Stop` passes 7.
* **`Mover::SetReachMid` 0x50ee40 stores 1.0 and ignores its argument**, although both callers (`SetupCalculatedMove` 0x50a8f0 and `MoveDirect` 0x505601) pass 3 × the radius of the unit.
* So a unit must come within 1 m of every waypoint before the last one. A vehicle that misses one circles it until the look-ahead lets it skip ahead.

### Damage and death of a unit

* Entity fields: hit points `+0x32` (int16), alive flag `+0x3c` (byte), squad id `+0x3e` (int16; 0x4ee960(server `[0x82f504]`, id) finds the squad), entity id `+0x4c`, armor `+0x90` (taken off each hit), invulnerable flag `+0xa8`, killer squad id `+0xb8` (int16), killer client `+0xbc` (the `+0xa4` field of the killer squad).
* *Damage* 0x4fa770 (thiscall: entity, projectile, float damage, bool) is the area damage of an exploding projectile; 0x4fa330 (thiscall: entity, projectile) is the damage of a direct hit. Both skip a unit with the invulnerable flag, and, while the byte at 0x6eb424 is set, a unit whose squad has the byte `+0x19` at 0 (meaning *unknown*). When the hit points drop to 0 or below, they write the killer fields and call the death function with the effect on.
* *Die* 0x4fa0f0 (thiscall: entity, char with effect) runs once per unit (alive flag): with the effect, 0x4f1df0 tells the session of the kill, which shows the death; without it, 0x4f02d0 removes the entity. Then 0x5048d0 removes the unit from its squad, and **a squad that has no member left deletes itself at once** (vtable slot 1). A unit carried in an APC is released. The script action KILL calls it with the effect on.

## Client side

`CL_Entity::Interpolate` **0x4bbe10**: dead-reckoning plus exponential error correction. The rate is the console variable `movement_smoothing` (float `@0x67af84`, default 4.0/s; alpha = rate·dt, clamped to 1). This is cosmetic only.

### Mouse and camera rotation

* Mouse update **0x418fc0** (thiscall, the mouse object `[0x69b694]`; called each frame through 0x41f670 from the main loop 0x4018a0). It reads the DirectInput device `[0x69b690]` (IDirectInputDevice7, data format DIMOUSESTATE2): the buffered events (vtable `+0x28`, GetDeviceData) bring the motion on x (`dwOfs` 0), y (4) and the wheel (8), and buttons 0 and 1 (`0xc`, `0xd`; swapped when `+0x3d` differs from `+1`). Events of button 2, the middle button, fall to the default case, so the game never reads the middle button. The full state (vtable `+0x24`, GetDeviceState, 0x14 bytes) is read only once, after the flag `+0` is set.
* The raw axis totals are at `+0x18`/`+0x1c`/`+0x20`. The cursor position in pixels `+0x28`/`+0x2c` grows by the motion times the mouse sensitivity (float `+0x34`), cut to whole pixels, and stays within `+8`…`+0x10` (x) and `+0xc`…`+0x14` (y); a copy goes to `[0x6a8094]+0x1a48`/`+0x1a4c`. 0x4c2aa0 (stdcall: x, y) moves the cursor through 0x419300. 0x419330 confines the Windows cursor to one pixel at the window center, and the game draws a cursor of its own.
* Border rotation **0x4b6a80** (thiscall, the interface object, no arguments; first instructions push −1, push imm32). Its only caller is the input handler of the interface 0x4afe50 (call at 0x4aff31), which runs it after the key and joystick camera 0x4b0060 and only while no selection box is open (`+0x10d04` = 0). Cursor x < 1 turns the yaw by −rate·dt and x ≥ screen width − 2 by +rate·dt; cursor y < 1 turns the pitch by −rate·dt and y ≥ screen height − 2 by +rate·dt. dt is the frame time (double `[0x6a8094]+0x1a8`).
* The camera is the member at interface `+0x1c4`. 0x4c23b0 (thiscall: camera, float angle; `ret 4`) adds to the yaw `+0x668` and keeps it in [0, 2π). 0x4c2330 (same form) adds to the pitch `+0x664`, keeps it in [0, 2π), and moves a pitch between the limits 0x612040 and 0x6120e8 to π/2 when it is below 0x611fb0, else to 3π/2.
* Rates: float 0x6e6138 (yaw) and 0x6e613c (pitch), both 2.0 rad/s by default and both set from one game option by 0x4ac240. The tutorial scripts lock each axis: byte 0x67d0c6 allows the yaw and 0x67d0c5 the pitch (`EnableYRotation`, `EnableXRotation` and their `Disable` forms; both 1 by default), and 0x67d0c4 allows camera movement (`EnableCameraMovement`).

## Archive format (.sdf)

Read by *Archive::Open* 0x410dc0, *Archive::Find* 0x4110c0, *ReadBlock* 0x410d60, *ReadFile* 0x410cb0. `tools/sdf.py` lists and extracts archives.

* **Header:** `u32 magic`: the low 24 bits are `"RYS"`, bits 24–29 are the version (7 in the Anthology files), the top byte selects the mode:

  | top byte | mode `+0x10` | meaning |
  |---|---|---|
  | 0x07 | 0x40 | plain files; the archive may be memory-mapped |
  | 0x87 | 0x80 | files are zlib-compressed (all Anthology archives) |
  | 0xc7 | 0x88 | files are encrypted and compressed |
  | 0x47 | 0x08 | files are encrypted |

  Version > 5: `u32 file size` follows (checked against the real size). The root directory starts right after (offset 8, or 4 for old versions).
* **Encryption** (*Decrypt* 0x410b80), used for every directory read and for file data in modes 0x08/0x88. A rolling XOR over 32-bit words, keyed by the length n of the block being read:
  `key = ~n ^ 0x6d3ae31b` (version 7; versions 5 and 6 use `0x7e183c5a`); for each word, with `rem` = bytes left before this word: `word ^= key; key = rotr(key, key & 31) ^ rem`. A trailing partial word is not encrypted. (So the first header words decrypt with `0x92c51ce0`, which is why they look XORed with `0x92c51ceX`.)
* **Directory:** the root is `u32 count`, `u32 block size`, then the block. A subdirectory, at the offset of its entry, is `u32 block size` and the block. Each block starts with `count` `u32` offsets (relative to the block) to its entries. Entry: `u32 size`, `u32 offset` (absolute in the file), `u8 flags` (bit 0 = directory; for a directory `size` is its entry count), then the name, NUL-terminated. Lookup splits the path on `/` or `\` and compares names case-sensitively (names are upper case).
* **File data:** `size` bytes at `offset`. Compressed files start with `u32 uncompressed size`, then a zlib stream (the game links zlib 1.1.3 and calls `uncompress`, 0x4129e0).
* Archives: `Data0.sdf` (game data, 396 files: `ENTFILES`, dialogs, localization, briefings), `DATA1.sdf` and `DATA2.sdf` (later data: more entity files and multiplayer missions), `Missions.sdf` (missions), `graphics.sdf`, `sounds.sdf`, `music.sdf`, `cutscenes.sdf`. Mods load with `gc.exe -mod NAME` from `data/mods/NAME/NAME0.sdf`.

## Unit data (ENTFILES)

Entity types are text files `ENTFILES/<NAME>.ENT` in `Data0.sdf`, with overrides and additions in `DATA1.sdf` and `DATA2.sdf`. Lines starting with `//` are comments; each key is on its own line, followed by its value lines. `EntityData::LoadMe` 0x4212c0 stores the values unscaled:

| key | entity type field | use |
|---|---|---|
| `Radius` | `+0x88` | collision radius; copied to the entity radius `+0x24` |
| `Acceleration` | `+0x94` | copied to Mover `+0x48` (and the base `+0x4c`) by the Mover init 0x50e450 |
| `Friction` | `+0x98` | *Friction*, arrival factor |
| `MaxSpeed` | `+0x9c` | copied to Mover `+0x40` (and the base `+0x44`) |
| `TurnSpeed` | `+0xa0`, `+0xa4` | rad/s |
| `SlopeBorder` | `+0xb0` | the terrain class (`moverClass`; values above 4 become 2) |
| `TurnOffset` | `+0xc0` | the close range of *Steer*; default 5 (EntityData ctor 0x420da0) |
| `MaxUnits` | | squad size |
| `MoverType` | | `foot`, `vehicle`, `hover`, `air` |

Values of the main units (speeds in m/s, accelerations in m/s²):

| unit | size | radius | accel | friction | top speed | turn | class | TurnOffset |
|---|---|---|---|---|---|---|---|---|
| CC_MARINE | 8 | 1.0 | 110 | 100 | 8 | 3.2 | 2 | 5 |
| CC_JAEGER | 4 | 1.3 | 150 | 100 | 13 | 2.2 | 2 | 5 |
| CC_LIGHTTANK | 4 | 3.5 | 87 | 80 | 28 | 1.8 | 1 | 10 |
| CC_MEDIUMTANK | 4 | 3.5 | 70 | 65 | 20 | 1.5 | 1 | 5 |
| CC_HEAVYTANK | 2 | 6.0 | 87 | 80 | 15 | 0.9 | 1 | 12 |
| CC_SCOUTVEHICLE | 3 | 3.29 | 75 | 65 | 32 | 2.2 | 1 | 15 |
| CC_APC | 1 | 3.8 | 95 | 65 | 20 | 2.2 | 1 | 5 |
| WC_MEDIUMTANK | 4 | 5.0 | 80 | 70 | 28 | 2.2 | 1 | 11 |
| WC_HEAVYTANK | 2 | 5.8 | 90 | 75 | 25 | 2.2 | 1 | 10 |
| PX_MEDIUMTANK | 4 | 5.6 | 70 | 68.5 | 23 | 1.1 | 1 | 5 |
| CC_ATTACKAERODYNE | 4 | 3.5 | 125 | 105 | 50 | 1.2 | 4 | 100 |

* Factions (`Force` key): `CRAYVEN` (prefix CC), `WRATH` (prefix WC, the Order of the New Dawn), `PHOENIX` (prefix PX, in DATA2.sdf). Order ground units are `hover` movers. Ground squad sizes go from 1 (APCs, artillery, trucks) over 2 (heavy tanks, power armor), 3 (scout vehicles), 4 (tanks), 5 (Plasmag) and 8 (Marines, Crusaders) to 12 (Faithful).

### Armor profiles and the Mover top speed

* MANAGEMENTDATA/ARMOR.TXT (loader 0x424c10, after "Can't load armours") holds entries `ID` (name), `STEALTHMODIFIER`, `SPEEDMODIFIER`, `ARMORMODIFIER`, stored as 16-byte entries from 0x69c048: name, stealth, speed, armor; *FindArmor* 0x424f00 looks an entry up by name. Each ground unit type has four profiles, `<Unit> Balanced`, `Defensive`, `Offensive` and `Speed`, with speed modifiers 1, 0.75, 0.75 and 1.2 (Defensive also has armor 1.2). MANAGEMENTDATA/ENTCONFIGS.TXT pairs each unit type with a weapon and one of these profiles for each squad configuration.
* 0x4f9fd0(entity, armor entry): base top speed of the Mover `+0x44` *= speed modifier, then the top speed `+0x40` = `+0x44`; it also sets the armor `+0x94`/`+0x32` and scales the stealth `+0x7c`. Acceleration, friction and turn rate stay as in the unit file.
* 0x4f9fd0 runs once per unit, from *Squad::AddMember* 0x504af0 (call at 0x504b7f) when the squad has an armor profile (`squad+0x8c`). So the base top speed `+0x44` = `MaxSpeed` × speed modifier, fixed for the mission.
* Every tick, the entity update 0x4fa1b0 (vtable slot 0x61328c; not in the Ghidra output) calls 0x4fb940 (at 0x4fa235) and then *Tick* of the Mover. 0x4fb940 resets the Mover top speed `+0x40` and acceleration `+0x48` to their bases `+0x44`/`+0x4c` and then applies two modifiers:
  * Active effects, list at entity `+0x98` (*AddEffect* 0x4fbe90, applied by 0x4fbb80, dropped when their end time passes). Only *Entity::UseSpecialEquipment* 0x4fc0b0 and 0x4fdd90 add effects, from `AttributeModifiers/<name>.atm` files (loader 0x4eb0c0; keys `MaxSpeed` flag 1 = top speed factor, `Acceleration` flag 2, `Health` 4, `Accuracy` 8, `ViewRange` 0x10, `Perception` 0x20, `Stealth` 0x40, `ProtectorShield` 0x80, `TurnSpeed` 0x100, `Timer` = duration, `OneShot`). The speed files: SPEED.ATM (special equipment `SE_SPEED`, case 0xe, for `wc_lighttank` and `wc_heavytank`, 3 charges, 30 s recharge): top speed × 1.5 and acceleration × 1.5 for 60 s; AFTERBURNER.ATM (`SE_AFTERBURNER`, Crayven aerodynes): × 2 for 20 s. The code also names `Muck1.atm` and `boosterstation.atm`, but no archive holds them, so those special equipments fail to load and change nothing.
  * The hit slowdown: while the timer at entity `+0x84` is positive (it drops by the time step each tick), the top speed of a ground unit (MoverType not air) is × 0.5 for Crayven, × 0.9 for the Order and × 0.75 for Phoenix (constants 0x611380, 0x6132b0, 0x6132b4, picked by the force of the entity type `+0x70`: CRAYVEN 0, WRATH 1, PHOENIX 2, names at 0x66c820). 0x50da90 adds 1 s to this timer, up to 4 s, for every entity in a 4 m box around a projectile when the projectile ends (called at the end of the three update functions in vtables 0x6136fc, 0x613724, 0x613750; that they are projectiles is *inferred* from their code: they move by speed × time step with a collision test and damage what they hit).
  * So the top speed of a unit stays fixed in a mission only while it uses no speed booster and no projectile ends near it.
* Entity type `+0x38` is the MoverType: foot 1 (`ebx` = 1 at 0x42139d), air 2, vehicle 3, hover 4 (parser at 0x421419). `+0x70` is the `Force`.

**Consequences for movement.** Friction is applied in full every frame, so at full throttle a unit gains only acceleration − friction (5–15 m/s² for most units); a medium tank needs 4 s to reach 20 m/s. Any throttle below friction/acceleration (0.67–0.93 for the units above) decelerates. So the 0.6 throttle floor of *Steer* still brakes firmly near the final parking position, and its half throttle when the waypoint is behind is a hard brake.

## Screen resolution

The game draws through DirectDraw 7 and Direct3D 7 with the D3DX utility library linked into gc.exe. All video settings sit in the settings block at `[0x6a8094]`; the renderer object is at `[0x6a8090]`.

* **Choices.** The video menu offers 640x480, 800x600 and 1024x768, as the menu index `+0x7c8` (0, 1, 2; 3 = 1600x1200). The device table at `+0x98c` (16 bytes per device: type, name, D3DX index, widest menu width) gives hardware devices 1024 as the widest width (0x402d90). The command line option `-1600` sets the width to 1600 and the index to 3 (0x402f50). `data/capabilities.txt` can lower the width per graphics card (`MAXRES`, only 640, 800 or 1024 allowed, 0x42c880); a card that is not in the file gets no limit there.
* **Size list of the menu.** The builder of the video page 0x47e380 takes its labels from `localization/options_video.loc` (0x420110 returns text number n; texts 2 to 4 are "640x480", "800x600" and "1024x768"). The sizes form one row of check boxes (constructor 0x462d80, vtable 0x6118f8), bound to the size index of the menu copy `[0x69c018]+8`. 0x463200 (thiscall: list, label, index; `ret 8`) adds one entry: it copies the label and widens the row by the check box, 15 pixels and the label. The builder calls it at 0x47ee28 (index 0), at 0x47ee5b (index 1, only for a device whose widest width is above 640) and at 0x47ee9e (index 2, only above 800), and nothing else calls it. Index 3 never gets an entry. The row draws (0x463050) the entry whose index equals `*[row+0x34]` as checked, and a click (0x462fb0) writes the index of the clicked entry there, so several rows bound to one index act as one choice. A page appends a widget with 0x467cc0 (thiscall: page, widget, flag) and shows each widget on a line of its own; 0x467e40 sets the line height (20 pixels for the video page) and the number of visible lines (page height / line height). When the menu opens, 0x420760 copies the device, size index, full screen and color fields of the settings block into the menu copy, and it copies the 0x6c bytes from `+0x7c0` back when the menu closes.
* **Index outside 0 to 2.** 0x403f20 knows only the widths of indexes 0 to 2; for another index it reads an uninitialized width and then limits it to the widest width of the device. So index 3 works only when that width is 1600, and a new index needs its own code.
* **Saved choice.** `data/gcvideo.dat` (encrypted) holds the device table, the device `+0xd00`, the width `+0xd08` and 32-bit color `+0xd0d`. WinMain 0x404210 reads it at the start (0x402f50) and writes it at the exit. After it creates the renderer, WinMain sets the size index from the width (640 to 0, 800 to 1, 1024 to 2, 1600 to 3). A player profile also saves the 0x6c bytes from `+0x7c0`, but loading a profile keeps the device, size index and full screen fields of the running game.
* **Start.** The application constructor 0x401140 picks the height from the width: 640x480, 800x600, 1024x768 or 1600x1200. Any other width ends the game with the fatal error "Unsupported resolution: %d" (0x42b8e0 shows a fatal error and exits). It then calls slot 2 of the renderer vtable.
* **Renderer.** The D3DX renderer (vtable 0x611588, constructor 0x431cd0) keeps its width at `+0x1c` and its height at `+0x20` (getters 0x4294b0, 0x430140). Slot 1 (0x431e30) releases the D3DX context. Slot 2 (0x431eb0, thiscall: width, height, flag, `ret 0xc`) stores the size and calls slot 3 (0x431ed0), which creates the context, full screen or windowed (`+0x10`); the flag 1 means a change after the start and skips the capability lookup. The software renderer (0x51bfe0) has its own vtable.
* **Context creation.** In full screen, slot 3 calls D3DXCreateContextEx (0x404e53: device, flag 1 = full screen, window, color bits 16 or 32, one back buffer, width `+0x1c`, height `+0x20`). If it fails in 32-bit color, slot 3 sets `+0xd0d` to 16-bit. Then a failed 1024x768 tries 800x600, and a failed 800x600 tries 640x480. Any other failed size shows the error text (0x4293b0, which returns), and slot 3 returns 0x80004005 without a Direct3D device (`+0x40` stays NULL). Slot 2 returns that result. The next frame then crashes at 0x44e24b, which reads the device. In one test, 2560x1440 failed this way with DDERR_INVALIDOBJECT, though DirectDraw listed that mode at 8, 16 and 32 bits, and 1920x1080 worked.
* **Windowed mode.** The game window is a WS_POPUP window, which has no border (WinMain 0x404210). The base renderer (0x4293e0) keeps the window handle at `+0xc`, the windowed flag at `+0x10` and the window position at `+0x14`/`+0x18`. With the flag set, slot 3 creates a windowed D3DX context in the color depth of the desktop and moves the window to that position, but keeps its size (SetWindowPos with SWP_NOSIZE). If the windowed context fails, slot 3 clears the flag and creates a full-screen context of the same size. A change of the full-screen field `+0x7cc` makes 0x403f20 build a new renderer, but only WinMain writes the applied field `+0xd0c` (0 at the start), so that change would repeat at every pass; no widget of the video page seems to set `+0x7cc`.
* **Change during the game.** The main loop of WinMain calls 0x403f20 once per pass while the renderer is active. It computes the size of the menu index, at most the widest width of the device, and compares it with `+0xd08` and the color depth `+0x818` (menu) with `+0xd0d`. After a change it calls slot 1, then slot 2 with the flag 1, stores the new renderer width in `+0xd08`, and sets the cursor area with 0x419330 (thiscall on the mouse object `[0x69b694]`: left, top, right, bottom). Another device or a change between window and full screen makes a new renderer object instead.
* **Intro movies.** The application object `[0x6997cc]` runs its states in 0x4018a0: states 0 to 2 open the Bink movies (sierra, massive, intro), state 7 plays one, with the movie object at `+0`, which is NULL without a movie.
* **Community resolution changer.** Its guides set the game menu to 800x600 before the changer is used. In 0x401140, the 800x600 branch is the only one that pushes both the width and the height as constants: `push 0x258` at 0x4011af and `push 0x320` at 0x4011b4. The low 2 bytes of each constant are at file offsets 0x3b0–0x3b1 and 0x3b5–0x3b6, inside the first 1 KB of the file. So the changer most likely changed those 4 bytes; 1920x1080 changes exactly 4 bytes there. The menu then still says 800x600, so 0x403f20 sees no change, and gcvideo.dat keeps 800. This is inferred from the code, since no changed copy was kept (TODO: confirm).
* **Picture.** WSGF rates the game as "pixel-based" for wide screens with a "proper" HUD, with stretched movies (TODO: confirm in game). The default projection of the D3DX context (`CD3duContext::_InitViewport` 0x409676) takes its aspect ratio from the width `+0x38` and height `+0x3c` of the context. Whether the 3D view of the game uses another projection with a fixed aspect ratio is not checked (TODO: confirm in game).

## Interface layout and 2D drawing

The game lays out its 2D interface in screen pixels and never scales it, so at a large screen size the interface keeps its pixel size. The findings below are static (read from the code, not seen at run time).

* **Menus.** Each menu page is a text file `dialogs/*.txt` in Data0.sdf (also per faction: `dialogs/crayven/`, `dialogs/order/`, `dialogs/phoenix/`). Its lines place pictures, buttons and texts at fixed pixel positions of one 640x480 layout, for example `PICTURE 63 33 Mngm_SinglePlayerGame.tga` or `BUTTON ... 48 75 Mngm_ContinueGame.tga ...`. The parsers 0x4574a0 and 0x458b00 add two floats of the settings block to each position: `+0x82c` (x) and `+0x830` (y). Slot 3 of the D3DX renderer sets them to the middle of the renderer size, width/2 − 320 and height/2 − 240 (0x432351 and 0x432585), so the 640x480 layout sits in the middle of any screen. The background of the main menu is a 3D scene (`ENTMODELS/MUI_STARTMENU`).
* **In-game interface.** It is placed from the renderer width and height getters 0x4294b0 (`+0x1c`) and 0x430140 (`+0x20`). They have about 100 call sites each, in about 60 functions, among them `IngameInterface::Init` 0x4a98b0, the cursor area 0x419330 and the picking of the camera view (0x4cfa50, 0x4d0270). So the interface parts stay at the screen edges, with fixed sizes in pixels.
* **2D items.** The interface code queues its 2D items in lists of the settings block, and slot 35 of the D3DX renderer (0x434630) draws them at the end of the frame:

  | List (count, items) | Item size | Drawn by |
  |---|---|---|
  | `+0x930`, `+0x938` (first layer) and `+0x944`, `+0x94c` (last layer) | 0x20 bytes: texture, x, y, width, height, source x, source y, color | slot 40 (0x434ab0): a quad of the size of its source, one texel per pixel |
  | `+0x958`, `+0x960` | 0x14 bytes: x0, y0, x1, y1, color | slot 41 (0x434c90): a line |
  | `+0x96c`, `+0x974` | 0xc bytes | slot 42 (0x434d20) |
  | `+0x980`, `+0x988` | 0xac bytes: 4 vertices of the format 0x2c4 at `+8`, vertex count at `+0xa8` | `IDirect3DDevice7::DrawPrimitive` in slot 35 |

  Slot 40 hands a texture whose byte `+0x20` is set to slot 36 (0x434870) instead, which copies the image with `IDirectDrawSurface7::BltFast` to the surface that the D3DX context returns from its vtable offset 0x20. The game ends with the fatal error "Blitting Bitmap failed" when that copy fails.
* **Screen positions on the device.** The device is `IDirect3DDevice7` at renderer `+0x40`; the D3DX context is at `+0x48`. Every `DrawPrimitive` call with a vertex format of screen positions (`D3DFVF_XYZRHW`) is in the renderer, 0x43338d to 0x434e19: format 0x2c4 (position and RHW, two colors, two texture coordinate sets; triangle fans) for quads, and 0x44 (position and RHW, one color) for lines and points. The game sets its viewports through the D3DX context (`CD3duContext::SetViewport`, device call at 0x409616), and the D3DX `Clear` (0x408a03) clears the rectangle of the stored viewport.
* **Lens flare test.** 0x433050 locks the Z buffer, clears one value at a screen point, draws a quad there and reads the value back, to find out whether the point is covered.
* **Mouse.** The game reads the mouse through DirectInput (`DIMouse`, 0x418ba0) and moves its own cursor by the reported movement, inside the area that 0x419330 sets (mouse object `+8` to `+0x14`: left, top, right, bottom). 0x419330 also centers the Windows cursor in the game window.
* **Interface scale of the mod.** `ui_scale` (mod/src/ui_scale.c) gives the game code a smaller "game screen" (the screen divided by the scale) in the renderer fields `+0x1c` and `+0x20`, the menu origin and the cursor area. It converts only what reaches Direct3D back to screen pixels: the screen positions of `DrawPrimitive` and `DrawIndexedPrimitive`, the viewports and clear rectangles that fit in the game screen, and the image copy of slot 36 (as a stretched `Blt`). Slot 3 (0x431ed0, thiscall with two arguments, `ret 8`) creates the D3DX context; after a lost device, 0x432b80 calls it with the size that the renderer holds, so the hook puts the screen size back for that call. In game, the Direct3D runtime replaced the `DrawPrimitive` entry of the device vtable with another function of its own after the first draw, so the mod puts its hooks back whenever an entry changes. The game logged no `SetViewport` call at all: it transforms its 3D world into screen positions itself and draws everything with format 0x2c4, so scaling those vertices also scales the 3D view.
* **Bink movies.** 0x4189c0 (thiscall on the movie) shows one frame: it copies the frame into the surface of the movie (`+8`) and then copies that surface with `IDirectDrawSurface7::Blt` to the target surface of the movie (`+4`, the render surface of the context when the movie opened). The rectangle is centered with the renderer getters and the movie size (`+0x14`, `+0x18`), or NULL (stretched over the whole target) when `[[0x69b69c]+0x48]+0xa0` is set. Under `ui_scale`, the mod converts that rectangle to screen pixels.

## Frame time

Movement runs once per rendered frame with the measured frame time: *Timing::CalculateTime* 0x428550 reads `timeGetTime` or `QueryPerformanceCounter`; 0x428670 stores the frame time (`[0x6a8094]+0x1a8`), the game time step (`+0x1a0`, 0 while paused) and the game clock (`+0x198`). A fixed-step mode (flag `0x69b6ac`) uses exactly 0.04 s. The search budgets (60 expansions for a squad) are per frame, so searches finish sooner at higher frame rates.

## Loading a mod DLL

Steam sets the compatibility layers `RUNASADMIN HIGHDPIAWARE` for gc.exe (`installscript.vdf`), so Windows loads the shim engine (`apphelp.dll`, `AcGenral.dll`, `AcLayers.dll`). This happens *before* the imports of gc.exe are resolved. `AcGenral.dll` imports **winmm.dll** and **version.dll** from System32, so proxies with those names are never used. The first PathFix release used winmm, and a crash dump showed that the system winmm was the one loaded.

The imports of the game itself are resolved after that, application directory first: `binkw32.dll`, `ijl11.dll`, `WSOCK32.DLL`, `COMCTL32`, etc. **wsock32.dll** is outside the shim dependency closure, so PathFix 0.1.1 shipped as a wsock32 proxy, and `mod/tools/loadtest.py` confirmed that it loaded. Later dynamic loads (`dinput.dll`, `dsound.dll`, `ddraw.dll`) would also work, but other game fixes commonly ship DLLs with those names.
