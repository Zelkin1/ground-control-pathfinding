# Ground Control 1 pathfinding and movement: reverse-engineering notes

Target: `gc.exe` 1.0.1.1, Steam "Ground Control Anthology"
sha256 `4f5db3501f7f1fd611c98fca00db66cfb690e04e4e810da8883575c3b83a2797`.
Built with MSVC 6, image base `0x400000`, **no relocations** (addresses are fixed), no RTTI for game classes. Methods use `thiscall` (the object pointer `this` is passed in `ecx`).
DirectX is loaded dynamically. Imports include WINMM (3 functions), BINKW32 and IJL11.

The installed `gc.exe` can differ from this hash by 4 bytes in the PE header when a resolution changer has been applied (`gc.resolution-changer-backup.exe` is then the original). The code is the same.

Sources of the names: the debug strings in the game itself (`python -I tools/re/gcdis.py names`). Names in *italics* are guesses. A full Ghidra 12.2 decompile of all 8820 functions is in `re/gc_decompiled.c` (pseudo-C, one `// ==== <address>` header per function). Claims below are read from that code or the disassembly unless marked *inferred*.

Units: world units ("m"); one grid cell is 6.25 units; times in seconds. `dt` is the frame time (float at `0x82f5cc`, double at `[0x6a8094]+0x1a0/+0x1a8`).

## Overview: when, where and how units move

The server simulates and the client only interpolates (`SV_Session` / `CL_Session`). All movement logic runs on the host.

**When a squad moves.** A squad only starts moving because of one of these:
1. a player order from the network: move, stacked (queued) move, move with facing, attack, set formation (`SV_Session::Parse*`);
2. the next queued order, taken when the current move is finished;
3. its *stance* reacting to a visible enemy while idle or moving: `AGGRESSIVE` squads attack and chase, `CHICKEN` squads run to a random nearby point, `NORMAL` squads only shoot;
4. for computer-controlled squads (`AiSquad`): mission-script orders, idle orders (go to a point, patrol a list, pick a random point, wander, follow a squad), counter-attacks on whoever hit a protected unit in the last 2 s, and a reposition every second while their units cannot fire at the target.

**Where it goes.** One route is searched for the whole squad, from the member nearest to the destination. The destination becomes the anchor of the formation. Member *k* of the member list of the squad gets formation slot *k* (the order is reversed when the new facing points backwards). Each member drives its own short path to the squad route, then the shared waypoints, then straight to its slot.

**How each unit moves.** Every tick the `Mover` of each unit follows its waypoint list (skipping ahead when a later waypoint is in sight, re-pathing when the current one is hidden), computes a throttle and a steering value, adds a unit-avoidance bias, turns at its turn rate, applies friction and a speed cap, integrates its position, and then resolves collisions (push-out from other objects, sliding along blocked cells, teleport off too-steep ground).

```
CLCMD_MOVE ─► SV_Session::ParseMove 0x4f3a60 ─► Squad::OrderMove 0x50ab30 (order state = 1)
                                                  │ next Squad::Update 0x504950
                                                  ▼
                     Squad::ProcessOrder* 0x50ac20 ─ StartSearchQuiet(nearest member → dest), state 13
                                                  │ every tick
                                                  ▼
                     Squad::CalculatingMoveOrderLogic 0x50a6a0 ─ PathFinder::Update(req, 60)
                        │  quiet search failed → StartSearch (active); both failed → "Movement denied"
                        ▼  (done)
                     Squad::SetupCalculatedMove 0x50a7d0
                        │  GetPath → squad route;  formation setter 0x507ee0 (slots at the last point)
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
| `ParseMove` 0x4f3a60 | `Squad::OrderMove(dest, facing 0,0, quiet 0)` 0x50ab30 | later: confirm with the slot list, or deny `0x4d` |
| `ParseStackMove` 0x4f3b20 | `Squad::QueueMove`* 0x505110: appends queued order type 6. Refused when the squad is empty, in hold mode, or the first member's class may not stand on the point's cell | `0x4e` accepted, `0x4f` refused |
| `ParseFaceMove` 0x4f3ca0 / `ParseStackFaceMove` 0x4f3d60 | the same with a facing direction | |
| `ParseStop` 0x4f39c0 | `Squad::Stop(clearQueue 1, 1, 0)` 0x504bd0 | `0x4b` |
| `ParseSetMovement` 0x4f4b10 | `Squad::OrderSetMovementMode` 0x5068a0: `+0x54` = 0, 1 or 2; mode 2 also stops the squad | |
| `ParseSetFormation` 0x4f4c30 | *SetFormation* 0x5069e0: formation type `+0x91`; an idle or moving squad re-issues a direct move to its current destination with its current facing, so the new layout is applied at once | |
| `ParseAttack` 0x4f3ed0 and the stack/multi variants | attack orders (below) | |
| `Squad::OrderSetFireBehaviour` 0x5068f0 | `+0x64` = 0, 1 or 2; 1 and 2 clear all weapon targets | |

Movement modes (`+0x54`): **2 = hold position** (moves and stacked moves are refused; attacks only set weapon targets). 0 and 1 are *offensive* and *defensive* (*inferred* order from the UI strings): in mode 1, moving members whose weapon has a target get a 1 s wait (`mover+0x68 = 1.0`), so they pause to fire. Fire behaviours (`+0x64`), *inferred* from the UI: 0 fire at will, 1 return fire, 2 hold fire.

### Order processing: `Squad::ProcessOrder`* 0x50ac20

Called first in every `Squad::Update` 0x504950. The pending order kind is `+0x1f0` (3 = nothing pending).
* **1, move:** refused (deny `0x4d`, or silently for a quiet order) when the squad is empty, cannot act (0x5063a0) or is in hold mode. Otherwise:
  * the squad state `+0x30` is saved in `+0x24c`; a non-quiet order clears the order queue; an active special-weapon state (6) is stopped;
  * the search start `+0x204/+0x208` is the position of **the member nearest to the destination**;
  * the facing `+0x1fc/+0x200` defaults to destination − start;
  * `StartSearchQuiet(start, dest, class and 2 × radius of that member, tolerance 0)`, and the squad state becomes 13 (calculating).
* **0:** attack an entity (0x5056d0), replies `0x50` (accepted) / `0x51` (refused).
* **2:** an order on a position through 0x505cc0, replies `0x54` / `0x55` (*inferred*: special equipment used on a point).

`Squad::OrderMove` 0x50ab30 only stores the order: destination `+0x1f4/+0x1f8`, facing `+0x1fc/+0x200`, the *quiet* flag `+0x250` (queued and scripted orders: no deny message), kind 1.

### Route search, time-sliced: `Squad::CalculatingMoveOrderLogic` 0x50a6a0 (state 13)

* Each tick it runs `PathFinder::Update(squad+0x20c, 60)`.
* Done → `SetupCalculatedMove`.
* Search over but not found, and it was the quiet (forward) search → `StartSearch` (active, backward) from the same start, but with the class and radius of the *first* member, not the nearest one.
* Both failed → `Reset`; a quiet order stops the squad, a normal one sends `0x4d` ("Movement denied", `moveno*.wav`) and restores the saved state.

### `Squad::SetupCalculatedMove` 0x50a7d0

1. Normalises the facing (zero when shorter than 1e-5) and stores it as the formation facing `+0x94/+0x98`.
2. `GetPath` into a local list.
3. Formation setter 0x507ee0 at the last point of the route. On failure the move is denied like a failed search.
4. For the mover of each member: `SetReachLast(radius)` (final slot reached within the radius of the unit, at least 1), `SetReachMid(3 × radius)` (ignored, see "Waypoint reach radius"), *SetTarget(none)* 0x50e740, `+0x3c = +0x5c = 0`, then `Mover::SetMove(route, facing, squad)`.
5. Members with a weapon (`entity+0x5c`) get 0x512f80.
6. State 1 (moving), squad destination `+0x1c/+0x24` = order destination, straggler timer `+0x1c0 = −1`.
7. A non-quiet order sends the move confirmation with every formation slot (the client draws the markers).

### Synchronous squad move: *Squad::MoveDirect* 0x505200

Used by everything except the plain move of the player: queued orders, formation changes, the AI, scripts, attack moves and evasion (through 0x504df0, which also clears the queue on request and sets state 1).
* Same start rule (member nearest to the destination) and facing default.
* `StartSearchQuiet` + `PathFinder::Update(req, 500)` in one call; if that fails it logs "TRYING BACKWARDS!" and runs `StartSearch` + `Update(500)`.
* Formation at the last point of the route unless a target entity is given; an optional maximum route length (in points) can refuse the move.
* Then the same per-member set-up as `SetupCalculatedMove`, with the given target entity and flags (`mover+0x5c`, `+0x3c`).

### Order queue (stacked moves) and squad states

* Queue: list at `+0x38` (head), `+0x3c` (iterator), `+0x40` (tail), `+0x44` (count). Items are 0x2c bytes: point `[0..1]`, facing `[2..3]`, kind `[5]`, entity/squad references.
* *PopQueuedOrder* 0x506a20 runs the next item: kind 0 move (direct), 1 move with facing, 2 attack entity (0x5056d0), 4 attack several, 5 0x50a160, 6 path-searched move (`OrderMove(point, facing, quiet 1)` + `ProcessOrder` at once).
* `Squad::Stop(clearQueue, b, keepHeading)` 0x504bd0: without clearQueue it first tries the next queued order. Otherwise it clears the queue, sets the squad destination to the centroid of the members and the formation facing to their average heading, gives each mover `SetReachLast(7.0)` and stops it where it stands (0x50e690), and sets state 0.
* Squad states `+0x30` handled in `Squad::Update` 0x504950: 0 idle 0x506e50, 1 moving 0x507070, 3 attacking 0x507620, 5 evading (set by 0x505ed0), 6 special weapon (0x5082f0 unsets it), 13 calculating 0x50a6a0; 4, 8, 9, 10, 11, 12 are other activities (0x507a10, 0x5086f0, 0x507a80, 0x509ea0, 0x50a270).

### Arrival of a squad: moving state 0x507070

* With queued orders waiting, the squad counts as arrived as soon as **every member is within 2 × N × r of the destination** (N members, r = radius of the first member). The next order then starts while units are still settling.
* Otherwise the squad waits until every mover reports stopped (`mover+0x54`).
* **Straggler timer:** when the squad has more than 2 members (1 for some squads, flag `+0x19`) and fewer than 2 are still moving, a 20 s timer starts (`+0x1c0 = 20.0`). When it runs out the squad counts as arrived even if a unit is still stuck.
* On arrival: a client squad (`+0xa4`) gets `0x5e` (pop waypoint marker), then `Stop(0,0,0)`, which starts the next queued order or goes idle.

### Formation slots

* Formation setter 0x507ee0(anchor, facing): calls `SquadFormation::CalculateFormationPoints(anchor, facing, out, N, first member's radius, its class, type +0x91)`. On success it replaces the slot list `+0x28`; when the new facing points against the previous one (dot product < 0) it toggles `+0x60`; it stores the facing in `+0x58/+0x5c`.
* *SlotFor(entity)* 0x508030: the slot index is the index of the entity in the member vector. With `+0x60` set the slots are counted from the end, so a squad that turns around keeps its members on the same sides instead of crossing over (*inferred* purpose). There is no nearest-slot matching.
* `SquadFormation::CalculateFormationPoints(Vec2 anchor, Vec2 dir, list *out, int n, float radius, char class, char type)` **0x4e7be0**, **cdecl**; returns 1 when `out` holds exactly n slots. `n <= 0` just empties `out`.
  * Other callers: `Ingameinterface::CalculateSquadFormationPoints` 0x4b1020 (order preview), `SV_Session::CreateClientSquads` / `ReinforceClientSquads` (spawning) and the scripted 0x4f08f0.
  * Layout: row spacing `sp = 3.5·radius` (type 3: 3.4). Columns per type: 0 → 2n (line), 1 and 3 → (int)(√n + 0.9) (box), 2 → 1 (column), other → 4.
  * The rows are perpendicular to `dir`. For each row of `w = min(remaining, cols)` slots, centred at P, the first slot is at `P + perp·(w−1)·sp/2` with perp = (dir.y, −dir.x). Each next slot steps by `−perp·sp`, and up to `cols` positions are tried per row. P starts at the anchor and moves by `−dir·sp` per row.
  * A slot is kept only if `Raycast(slot + 3·unit(slot−P), anchor)` is clear (call @ 0x4e7edb). Dropped slots are not replaced in their row, so the formation stretches backwards.
  * It **fails** when a row centre P is on a blocked cell (cell > class, 0x4e82e0), and after 13 rows. The setter then keeps the old formation, and the move order does not go through.

### Squad fields used above

| offset | meaning |
|---|---|
| `+0x0c` | current target entity (ref-counted) |
| `+0x14` | member vector: begin `[0]`, count `[2]`, end `[3]`, iterator `[4]` |
| `+0x19` | flag set for computer-controlled squads (*inferred*) |
| `+0x1c/+0x20/+0x24` | current destination (x, y, z); after `Stop`, the members' centroid |
| `+0x28` | formation slot list |
| `+0x30` | squad state |
| `+0x34` | stance: 0 `NORMAL`, 1 `AGGRESSIVE`, 2 `CHICKEN` (parsed at 0x519870; 0/1 *inferred* from behaviour) |
| `+0x38..+0x44` | order queue |
| `+0x54` | movement mode (2 = hold) |
| `+0x58/+0x5c`, `+0x60` | last formation facing, slot-order flip |
| `+0x64` | fire behaviour |
| `+0x80` | attack timer (25 s on a new attack) |
| `+0x91` | formation type |
| `+0x94/+0x98` | formation facing |
| `+0xa4`, `+0xa8` | owning client, squad id |
| `+0x1c0` | straggler timer |
| `+0x1f0..+0x208` | pending order: kind, destination, facing, search start |
| `+0x20c` | PathRequest |
| `+0x24c`, `+0x250` | saved state, quiet flag |

### Reactions to enemies (stance)

* Idle (0x506e50) and moving (0x507070) squads look for targets (0x507ce0, 0x507db0; *inferred*: visible enemies).
* `NORMAL`: only assigns weapon targets (0x505b40), depending on the fire behaviour.
* `AGGRESSIVE`: attack order on the target (0x5056d0), which moves the squad to engage. In the attacking state it switches to a new visible target.
* `CHICKEN`: *Evade* 0x505ed0(target, 100): a random point around the first member, offset in relation to the target, at up to 100 units; direct move there, state 5.
* Attack order 0x5056d0: in hold mode it only sets weapon targets. Otherwise it stores the target, optionally clears the queue, and calls 0x505840, which moves the squad with `MoveDirect` and the target entity given to every mover (units then chase it, see "Chasing a target"). State 3, attack timer 25 s.

## AI squads (`AiSquad`)

Computer-controlled squads are driven by the mission scripts (*ScriptEngine*, triggers and actions around 0x500000–0x502500 and 0x515000) plus a small amount of local logic. All their moves use the synchronous `MoveDirect` (budget 500).

* *AiSquad::ExecuteOrder* 0x5037d0 (script orders, "Fallback order set..."): 0 move to a point; 1 attack a list of entities or squads (the first one now, the rest queued); 2 set the lists of units and squads to protect (`+0x274`, `+0x284`).
* *AiSquad::Update* 0x502ae0, then the normal `Squad::Update`:
  * removes dead entries from the protect lists;
  * **counter-attack:** for each protected unit hit within the last 2 s (`entity+0x38` = time of the last hit, `+0x34` = attacker), attack the attacker;
  * **idle order** `+0x268` while the squad is idle: 0 go to a point unless every member is within 70 units of it; 1 patrol a list of points in turn; 2 a random point from the list; 3 a random point within ±R units of a point; 4 follow a squad;
  * **reposition while attacking:** every 1 s, when the blocked-fire counters of the members (`entity+0xb7`, *inferred* meaning) add up to 21 or more, move to a random point near the first member (0–50 plus −5…+25 units along the line from the target, and up to ±75 units sideways) with a route of at most 5 points, then attack again; with a 9% chance a second sideways step follows.

## The path search (`PathFinder`)

### Global PathFinder object `@0x6eb470`

| offset | meaning |
|---|---|
| +0x00 (byte) | side of the last DDA step (0 = y side, else x side); used by the wall slide |
| +0x04/+0x08 | last raycast hit point (x,y) |
| +0x0c | **passability grid**: 1024×1024 cells, 2 bits per cell, row stride 256 bytes (256 KB) |
| +0x4000c.. | scratch list used by Expand |
| `0x82f4ec` (global float) | 1/cell size; cell = **6.25 world units** (`0x6130b8`) |

* `GetCell(x,y)` **0x4e83d0**: `(grid[y*256 + x/4] >> (6 - 2*(x&3))) & 3`
* A cell blocks a mover when `cell > moverClass`. The mover class is a byte at `entity->+0x54->+0xb0`. Class 4 (*aircraft*) skips the search and flies straight.
* `Raycast(x0,y0,x1,y1,moverClass)` **0x4e8400**: an Amanatides–Woo DDA (grid line walk) using 16.16 fixed-point cell coordinates. Returns `-1` when the line is clear, otherwise the blocking cell index `(cy<<10)|cx`, and stores the hit point at +4/+8. The ray has zero width.
* *GridDDA variant* **0x4e868b**, *CellCoords* **0x4e82e0**, *CellPos* **0x4e8380**, *terrain height* **0x4e5e60**.
* `LoadGrid(name)` **0x4e8200** (`ret 4`): reads 256 KB and copies the rows in reverse order into +0x0c (the last row of the file becomes grid row 0). It is called once per mission by both the client (0x4a1640) and the server (0x4ef660). Nothing else writes the grid, so it is static per map. The file is `<mission>/<mission>.SLP` in `Missions.sdf` (name format `%s.slp` @ 0x675dbc); there is one per campaign, tutorial and multiplayer map.
* The inverse cell size (1 / 6.25) at **0x82f4ec** is set by the static initializer **0x4e8130**; the raycast, `CellAt` 0x4e82e0, Expand and the outline traces multiply world coordinates by it.

### PathRequest (0x40 bytes; e.g. `Squad+0x20c`, or on the stack in Mover::FindPath)

| off | type | meaning |
|---|---|---|
| +0x00 | node* | open list head, sorted by f (insert **0x4eba70**, head **0x4db9d0**) |
| +0x04.. | list | closed list (**0x4ebad0** moves a node from open to closed) |
| +0x14/+0x18 | float | search **target** |
| +0x1c | byte | mover class |
| +0x20 | float | clearance (2 × radius `entity+0x24`) |
| +0x24 | float | goal tolerance² |
| +0x28..+0x34 | list | points already expanded toward (linear scan in Expand, O(n²)) |
| +0x38 | bool | done (found) |
| +0x39 | bool | active (set by StartSearch) |
| +0x3a | bool | still in progress (budget ran out this call) |
| +0x3c | int16 | total expansions; **hard cap 500** (`cmp word [edi+3c],1f4` @ 0x4e88ca / 0x4e88da) |

PathNode (0x1c bytes): `+0/+4` open-list links, `+8 g`, `+0xc f`, `+0x10 parent`, `+0x14 x`, `+0x18 y`.

* `Reset` **0x4ebb10**: frees the open and closed lists with operator delete (the node destructor 0x42ba60 is a no-op).
* `StartSearch(a1,a2,a3,a4,class,clear,tol)` **0x4eb9a0** (`ret 1Ch`, sets active=1): **target = (a1,a2), root = (a3,a4)**.
* `StartSearchQuiet` **0x4eb8d0** (active=0): **root = (a1,a2), target = (a3,a4)**, i.e. the roles are swapped. It stores tol² and clears the expanded-point list.
  Callers pass (unit, destination) to both. So active is a backward search (from the destination toward the unit) and quiet is a forward one. The squad, `MoveDirect` and `Mover::FindPath` all try quiet first, then active.
* `GetPath(list*)` **0x4ebbb0**: starts at the open-list head.
  * active: `push_back` of head->parent … root (the head is the unit itself and is skipped);
  * quiet: `push_front` of head … (root excluded).

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
   * **blocked** otherwise: *TraceLeft* **0x4e91f0** walks the outline of the blocking obstacle to its left end and returns a corner point **offset from the edge of the blocking cell by the clearance** (2 × radius); if that point lies in another blocked cell, the trace continues from there. Then `Expand(node, req, from, corner)`. The same with *TraceRight* **0x4e9620**.
3. Points are de-duplicated against the scratch list of this expansion and the expanded list of the request.

So the graph is built lazily from obstacle corners, every edge is checked with a zero-width ray, and only the corners keep the clearance.

## The unit (`Mover`, `entity+0x58`)

### Fields

* Entity at `+0x74`: position `+0xc/+0x10/+0x14` (x, height, y), heading angle `+0x1c`, radius `+0x24`, team `+0x3e` (int16), type `+0x54`, mover `+0x58`, weapon `+0x5c`, active flag `+0x88` (*inferred*; Update and Steer are skipped while it is 0), counter `+0xb7`.
* Entity type (`entity+0x54`, loaded from the ENT files, see "Unit data"): radius `+0x88`, acceleration `+0x94`, friction `+0x98`, max speed `+0x9c`, turn rate `+0xa4`, mover class `+0xb0`, TurnOffset (close range) `+0xc0`, flag `+0xd9`, kind `+0x38` (2 = *inferred* aircraft).
* Mover: owner type `+0x04`; velocity `+0x08/+0x0c`; pending velocity `+0x10/+0x14`; heading `+0x18/+0x1c` = (sin a, cos a); current waypoint `+0x20/+0x24/+0x28`; waypoint list `+0x2c` (head), `+0x30` (iterator), `+0x34` (tail), `+0x38` (count); flags `+0x3c`; max speed `+0x40`; acceleration `+0x48`; target entity `+0x50`; stopped `+0x54`; reach radius `+0x58`; flag `+0x5c`; final facing `+0x60/+0x64`; wait timer `+0x68`; heading saved for a wait `+0x6c/+0x70`; pass side `+0x78`; blocker collision object `+0x7c` and entity `+0x80`; reach radii `+0x84` (last leg), `+0x8c` (intermediate); phase counter `+0x90`; never re-orient `+0x94`; last leg `+0x95`; avoid counter `+0x98`; re-orient `+0x9c`; target position `+0xa0/+0xa4/+0xa8`; side-switch cooldown `+0xac`.
* Vtable **0x613498** (a subclass uses 0x6134ec with *Integrate* 0x513010): `[0]` *Tick* 0x50e5b0, `[1]` *Friction* 0x50fdf0, `[2]` *Integrate* 0x5109e0, `[3]` *Turn* 0x511560, `[4]` *SteerTo* 0x511400.

### `Mover::SetMove(list *route, Vec2 facing, Squad*)` 0x50e7e0 (`ret 10h`)

1. Clears the blocker, stores the final facing `+0x60/+0x64`, copies the squad route into `+0x2c`.
2. Takes the first point of the route as the current waypoint and, if more points remain, also skips to the second one.
3. `FindPath(current waypoint, tolerance = 1.0, or the last-leg reach if nothing else remains, budget 200)` from the unit.
4. On failure: empty list, current waypoint = the position of the unit itself (the unit does not move).
5. On success, without a target entity: the last point of the route is replaced by the formation slot of this unit (0x508030). The leg from the last shared waypoint to the slot is not searched.
6. Sets the last-leg flag and reach radius (20.0 on the last leg for kind-2 units), clears stopped, sets re-orient (`+0x9c`, unless `+0x94`), and resets the wait timer to −3.

So **members reuse the squad route**: each searches only from itself to the second point of the route. The same unit→waypoint search is often issued twice in one tick (16% of searches in a play log).

### `Mover::FindPath(Vec2 dest, float tol, int budget)` 0x50eb60 (`ret 10h`)

Runs a local PathRequest **synchronously**: quiet search + `Update(budget)`, then the active search if that failed. Class and clearance (2 × radius) come from the unit. The found path is **put in front of** the remaining waypoints, its first point becomes the current waypoint, and re-orient is set. Budgets passed in:
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
* Arrival: when the distance to the current waypoint is within the reach radius `+0x58`, the next waypoint is taken only if it is visible (0x50f4e6); on the last waypoint the unit is marked stopped (`+0x54`).
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
  * not on the last leg: 1;
  * on the last leg, with d = distance to the slot and v = speed toward it: k = v² / (2 (d − 3)) / friction, the braking needed compared with friction alone; k < 0.5 → 1; else (1 − 2(k − 0.5)) · friction / acceleration, at least 0.
  * With the real unit data (see "Unit data"), 0.6 × acceleration is below friction for almost every unit, so the floor still decelerates: arrival braking works.
* Acceleration 0x511500: `v += heading · accel(+0x48) · dt · throttle`.
* Steering: 0 when close to the waypoint (within `+0xc0`) and it lies on one side of the velocity (*inferred* purpose: stop spinning around a close point); otherwise *SteerTo*.
* *SteerTo(d)* 0x511400: with c = cos of the angle between heading and d: steering = ±(1 − c⁶) when d is ahead, full lock ±1 when it is behind.

### *AvoidUnits* 0x5101a0: unit against unit

Returns a bias that is added to the steering value; skipped while re-orienting.
* It queries the collision map (0x4fef30) along the heading, within a radius of `clamp(0.6·dist to waypoint, 1.5 r, 4 r)` (halved when heading away from the waypoint).
* On the last leg, a non-unit obstacle or a stopped unit at the destination ends the move where the unit stands.
* Otherwise it picks the nearest blocker ahead, then decides:
  * a friendly blocker that is moving, not waiting, has avoid counter ≤ 2 and no target → **return 0**: keep driving into it;
  * if `+0x68 < −2` (cooldown over) and the mover of the blocker outranks ours on `+0x40` (max speed), or its type has flag `+0xd9` → **wait 2 s** (`mov [esi+68h], 2.0` @ 0x5108d7) with the heading saved;
  * else choose a side (copying the choice of the blocker when it is avoiding us) and return **±4**, which is full lock after the clamp in *Turn*.
* The decision only uses current positions, not where the units are going.

### *Turn*, *Friction*, *Integrate*

* *Turn(s)* 0x511560: s is clamped to [−1, 1]; heading angle += turn rate (`type+0xa4`) · dt · s, wrapped at 2π; heading = (sin a, cos a).
* *Friction* 0x50fdf0: `v −= unit(v) · friction(type+0x98) · dt` per axis (a component that changes sign becomes 0); then the speed is capped at `+0x40`.
* *Integrate* 0x5109e0: `pos += v · dt`, clamped to the map (10 … 6390 units); the height comes from the terrain (0x4e5e60); then *Physics* 0x510b20.

### *Physics* 0x510b20: collisions and slopes

1. **Other objects:** if the collision box of the unit overlaps another object in the collision map (0x4fecf0), the unit is pushed out along the line between the centres to touching distance + 0.3, and its velocity along that line is changed so that it moves apart. If it still overlaps, it goes back to its previous position.
2. **Blocked cells:** *WallSlide* 0x511170 raycasts from the old to the new position for the class of the unit. When blocked, the velocity component across the cell side that was hit becomes 0 and the position is moved to 0.55 cells from that cell. This repeats up to three times per tick. A slide flips the preferred pass side `+0x78` (at most once per 2 s) and sets the avoid counter to 5 ticks.
3. **Too-steep ground:** when the cell under the unit is above its class it logs `"%s IS STANDING IN SLOPE!!!!"` and teleports the unit to the first legal cell found ±x or ±y at 0.5, 1.0, 1.5 … cells (0x511000).

### Waypoint reach radius

The arrival check in `Mover::Update` (0x50f47c) compares the distance to the current waypoint with `+0x58`.
* That value comes from `+0x84` on the last leg and from `+0x8c` otherwise.
* `Mover::SetReachLast` 0x50ee10 stores max(arg, 1); orders pass the radius of the unit, `Stop` passes 7.
* **`Mover::SetReachMid` 0x50ee40 stores 1.0 and ignores its argument**, although both callers (`SetupCalculatedMove` 0x50a8f0 and `MoveDirect` 0x505601) pass 3 × the radius of the unit.
* So every intermediate waypoint must be passed within 1 world unit. A vehicle that misses one circles it until the look-ahead lets it skip ahead.

## Client side

`CL_Entity::Interpolate` **0x4bbe10**: dead-reckoning plus exponential error correction. The rate is the console variable `movement_smoothing` (float `@0x67af84`, default 4.0/s; alpha = rate·dt, clamped to 1). This is cosmetic only.

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
| `SlopeBorder` | `+0xb0` | the mover class (values above 4 become 2) |
| `TurnOffset` | `+0xc0` | *Steer*'s close range; default 5 (EntityData ctor 0x420da0) |
| `MaxUnits` | | squad size |
| `MoverType` | | `foot`, `vehicle`, `hover`, `air` |

Values of the main units (speeds in units/s, accelerations in units/s²):

| unit | size | radius | accel | friction | max speed | turn | class | TurnOffset |
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

**Consequences for movement.** Friction is applied in full every frame, so at full throttle a unit gains only acceleration − friction (5–15 units/s² for most units); a medium tank needs 4 s to reach 20 units/s. Any throttle below friction/acceleration (0.67–0.93 for the units above) decelerates. So *Steer*'s 0.6 throttle floor still brakes firmly near the slot, and its half throttle when the waypoint is behind is a hard brake.

## Frame time

Movement runs once per rendered frame with the measured frame time: *Timing::CalculateTime* 0x428550 reads `timeGetTime` or `QueryPerformanceCounter`; 0x428670 stores the frame time (`[0x6a8094]+0x1a8`), the game time step (`+0x1a0`, 0 while paused) and the game clock (`+0x198`). A fixed-step mode (flag `0x69b6ac`) uses exactly 0.04 s. The search budgets (60 expansions for a squad) are per frame, so searches finish sooner at higher frame rates.

## Loading a mod DLL

Steam sets the compatibility layers `RUNASADMIN HIGHDPIAWARE` for gc.exe (`installscript.vdf`), so Windows loads the shim engine (`apphelp.dll`, `AcGenral.dll`, `AcLayers.dll`). This happens *before* the imports of gc.exe are resolved. `AcGenral.dll` imports **winmm.dll** and **version.dll** from System32, so proxies with those names are never used. The first PathFix release used winmm, and a crash dump showed that the system winmm was the one loaded.

The imports of the game itself are resolved after that, application directory first: `binkw32.dll`, `ijl11.dll`, `WSOCK32.DLL`, `COMCTL32`, etc. **wsock32.dll** is outside the shim dependency closure, so PathFix 0.1.1 shipped as a wsock32 proxy, and `tools/pathfix/loadtest.py` confirmed that it loaded. Later dynamic loads (`dinput.dll`, `dsound.dll`, `ddraw.dll`) would also work, but other game fixes commonly ship DLLs with those names.
