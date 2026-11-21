# Simulator parameters

This file lists the constants of the simulator that have a physical meaning: distances, times, speeds, angles, rates and the shares of such values. Each table gives the name of the constant in the code, its value, its unit, and what it sets. The heading of each section names the file that defines its constants, and a row names another file when its constant lives there.

The file has four parts:

- Base game: The values of Ground Control itself. The simulator copies them from the code of gc.exe and from the unit files of the game, and it does not change them.
- Our changes: The values of the behavior that the simulator adds to the game. This part covers formation following, the coordination between squads, and the few points where the simulator changes the movement of the base game.
- Testing: The values that serve only the tests. They do not change how units move.
- Tuning status: Which values of "Our changes" were tuned by tests of several values, and which were not tuned yet.

In the first three parts, the sections follow the order in which a move happens.

## Terms

- Blocking terrain is the map cells that a unit cannot cross, such as cliffs, rocks, trees and buildings. Free terrain is the ground that it can cross. A passage is free terrain between blocking terrain.
- The clearance of a point is its distance to the closest blocking terrain.
- The simulator has three modes: the original game mode (`--avoid original`), the form mode (`--avoid form`) and the coord mode (`--avoid coord`). The form and coord modes are the formation modes.
- A route is the path that a squad follows to its destination. The base game path search (the path search in the code of gc.exe) gives its corners, the route corners. An expansion is one step of that search.
- The front unit of a squad is the member closest to the destination at the order. The top speed of a squad is the top speed of its slowest member.
- A control point is a point that shapes the curved route that replaces the route. The curve passes near the control points, not through them.
- A curve position is a point of the curved route, about every 4 m. The clearance tests run at the curve positions, and the curve positions become the points of the route that the squad drives.
- A clearance test checks the clearance at one curve position. It measures the width of the passage with two rays across the curve, and it checks that a circle around the curve position holds no blocking terrain. The wanted clearance is the clearance that the curve needs at a curve position. A cramped position is a curve position with less than the wanted clearance.
- A displacement moves the control points near a cramped position sideways, away from the blocking terrain. The displacements run in iterations, and each iteration goes over the whole curve once.
- The route smoothing set-up is the work that turns the route of a squad into the curved route and lays out its lanes and slots.
- The anchor is a point that moves along the route at the speed of the squad, and the formation is built around it. Each member has a slot, a point that moves with the anchor, and a final parking position, where the member stops at the destination. Members can swap slots, so that their ways to their slots stay short.
- A lane is the path of one slot beside the route. A lane cross-section holds the values of all lanes at one distance along the route. The lane clamp limits the sideways offset of a lane, so that the body of the widest member keeps clear of blocking terrain.
- Where the lanes fit next to each other, the squad drives in square formation. Where they do not fit, it drives in column formation: one member behind the other, down the middle of the passage. The column gap is the gap between two members in column formation.
- The reach of a unit is the distance at which it counts its current waypoint as reached.
- A parked squad is a squad that neither moves nor searches.
- A squad gives way when it slows down or waits so that another squad can go first.
- A detour is a way around blocking terrain that the base game path search finds for one unit that makes no progress. A re-route is a new path search of a moving squad around a squad that parks on its way.
- A share of friction is a deceleration given as a part of the friction of the unit.
- A tick is one step of the game, one frame long (about 5.6 ms), in which every unit moves once.

## Base game

Every value in this part is a value of the game. The simulator uses it as it is in all modes. Where the form and coord modes behave differently, they use the values of the part "Our changes".

### Map and time

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `GC_CELL_METERS` | 6.25 | m | Width of one square map cell, which is either free terrain or blocking terrain for each unit type (game/game.h). |
| `GC_GRID_SIZE` | 1024 | cells | Number of cells along each side of the square map, which makes the map 6400 m wide (game/game.h). |
| `GAME_FRAME_SECONDS` | 0.0056 | s | Length of one tick: the game moves every unit once per frame, with the frame time as its time step, and the mod logs show frames of 5.3 to 5.8 ms (sim.h). `--tick SECONDS` sets another step, such as the 0.04 s that the simulator used before. The game mod steps by the measured time of each frame. |
| `WORLD_MIN_METERS` | 10 | m | Smallest x or y of a unit: Integrate (0x5109e0) clamps each move to stay 10 m inside the map (game/physics.c). |
| `WORLD_MAX_METERS` | 6390 | m | Largest x or y of a unit, 10 m inside the far edge of the 6400 m map, kept by the same step (game/physics.c). |

### Unit types (units.c)

The values come from the unit files of the game (the .ENT files in Data0.sdf, with the newer values of DATA2.sdf). A unit file without `TurnOffset` has the close range 5 of the game. The scenarios use the Crayven unit types, and `--faction` replaces each one by the unit type of the same role in the Order of the New Dawn or in Phoenix.

| Unit type | Faction | Radius (m) | Top speed (m/s) | Acceleration (m/s²) | Friction (m/s²) | Turn rate (rad/s) | Close range (m) | Units per squad |
|---|---|---|---|---|---|---|---|---|
| `MEDIUM_TANK` (cc_mediumtank) | Crayven | 3.5 | 20 | 70 | 65 | 1.5 | 5 | 4 |
| `LIGHT_TANK` (cc_lighttank) | Crayven | 3.5 | 28 | 87 | 80 | 1.8 | 10 | 4 |
| `HEAVY_TANK` (cc_heavytank) | Crayven | 6.0 | 15 | 87 | 80 | 0.9 | 12 | 2 |
| `MARINE` (cc_marine) | Crayven | 1.0 | 8 | 110 | 100 | 3.2 | 5 | 8 |
| `COMMAND_APC` (cc_commandapc) | Crayven | 4.1 | 20 | 70 | 65 | 1.7 | 5 | 1 |
| `ORDER_MEDIUM_TANK` (wc_mediumtank) | Order | 5.0 | 28 | 80 | 70 | 2.2 | 11 | 4 |
| `ORDER_LIGHT_TANK` (wc_lighttank) | Order | 6.5 | 30 | 45 | 30 | 2.2 | 10 | 4 |
| `ORDER_HEAVY_TANK` (wc_heavytank) | Order | 5.8 | 25 | 90 | 75 | 2.2 | 10 | 2 |
| `ORDER_CRUSADER` (wc_crusader) | Order | 1.3 | 8 | 110 | 100 | 2.5 | 5 | 8 |
| `ORDER_COMMAND_APC` (wc_commandapc) | Order | 4.5 | 28 | 90 | 75 | 2.2 | 10 | 1 |
| `PHOENIX_MEDIUM_TANK` (px_mediumtank) | Phoenix | 5.6 | 23 | 70 | 68.5 | 1.1 | 5 | 4 |
| `PHOENIX_LIGHT_TANK` (px_lighttank) | Phoenix | 5.1 | 29 | 87 | 84 | 1.1 | 10 | 4 |
| `PHOENIX_HEAVY_TANK` (px_heavytank) | Phoenix | 6.1 | 19 | 87 | 86 | 1.1 | 12 | 2 |
| `PHOENIX_BASIC` (px_basic) | Phoenix | 1.0 | 8 | 110 | 100 | 3.2 | 5 | 8 |
| `PHOENIX_COMMAND_APC` (px_commandapc) | Phoenix | 4.1 | 23 | 70 | 67 | 1.7 | 5 | 1 |

Friction is the speed that a unit loses each second. The close range is the distance at which the steering of the game changes its behavior near a waypoint. The Order units and the Phoenix command APC are hover units in the game, and the simulator moves them as vehicles (TODO: confirm that the game moves them alike).

### Armor profiles (units.c)

Each ground unit type has four armor profiles in MANAGEMENTDATA/ARMOR.TXT. The game multiplies the top speed of a unit by the speed modifier of its profile (0x4f9fd0), but not its acceleration, friction or turn rate. `--profile` sets the profile of every unit.

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `PROFILE_SPEED_MODIFIER` (balanced) | 1 | factor of top speed | Top speed of a unit with the Balanced profile. |
| `PROFILE_SPEED_MODIFIER` (defensive) | 0.75 | factor of top speed | Top speed of a unit with the Defensive profile. |
| `PROFILE_SPEED_MODIFIER` (offensive) | 0.75 | factor of top speed | Top speed of a unit with the Offensive profile. |
| `PROFILE_SPEED_MODIFIER` (speed) | 1.2 | factor of top speed | Top speed of a unit with the Speed profile. |

### Speed booster (scenarios.c)

In every tick, the game sets the top speed and the acceleration of a unit back to their base values and then applies the active effects (0x4fb940). The speed booster of SPEED.ATM is one of these effects. `--boost SECONDS` gives it to every unit from that time on. A negative time means a boost that began before the order. For tests, `--boostcycle SECONDS` turns the boost on and off for that long by turns instead, and `--booststagger SECONDS` starts it that much later for each squad index.

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `BOOSTER_FACTOR` | 1.5 | factor | Top speed and acceleration of a boosted unit, as a factor of its base values. |
| `BOOSTER_SECONDS` | 60 | s | Time that the speed booster lasts. |

### Formation layouts (scenarios.c)

The formation layout of the game puts `n` final parking positions in rows across the facing (PATHFINDING.md, "Final parking positions"). `--layout` sets the formation type of every squad.

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `SQUARE_FORMATION_TYPE` | 1 | formation type | Square layout (`--layout square`): about √n positions per row. |
| `LINE_FORMATION_TYPE` | 0 | formation type | Line layout (`--layout line`): all positions in one row. |

### Units (game/movement.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `STOP_REACH_METERS` | 7 | m | Reach of a unit that stands or must stop, so it counts its waypoint as reached within 7 m (sim.h). |
| `MID_REACH_METERS` | 1 | m | Reach of every waypoint before the last one of a move, so units drive close to each corner (sim.h). |
| `WAIT_TIMER_AT_ORDER_SECONDS` | -3 | s | Start value of the wait timer of a unit after a new order, so waits start only once it counts up (sim.h). |
| `WAYPOINT_TOLERANCE_METERS` | 1 | m | The path search of a new move can end this close to its waypoint instead of exactly on it. |
| `REACH_EPSILON_METERS` | 0.1 | m | A reach this much larger than the reach of the last waypoint skips the new search toward it. |
| `ARRIVAL_STOP_DISTANCE_METERS` | 3 | m | Braking before a waypoint aims to bring the unit to a stop this short of the waypoint. |
| `BRAKING_START_FRACTION` | 0.5 | share of friction | Braking starts where the unit could stop by slowing with this share of its friction. |
| `WAYPOINT_BEHIND_METERS` | -0.2 | m | A waypoint that lies below this distance along the heading of the unit counts as behind it. |
| `BEHIND_THROTTLE` | 0.5 | share of acceleration | Throttle that the steering uses while it turns the unit toward a waypoint behind it. |
| `MIN_THROTTLE` | 0.6 | share of acceleration | Smallest throttle that the steering uses toward a waypoint ahead of the unit. |
| `REORIENT_DONE_COSINE` | 0.95 | cosine | A turn on the spot toward a new direction ends once the unit faces within about 18 degrees of it. |
| `QUERY_RADIUS_FACTOR` | 1.1 | factor of radius | The search for units in the way widens the radius of the unit by this factor. |
| `QUERY_RADIUS_MARGIN_FACTOR` | 1.25 | factor of radius | The base game avoidance (AvoidUnits) also widens the radius of the unit by this factor before that search. |
| `LAST_WAYPOINT_TURN_COSINE` | 0.7 | cosine | A unit that drives to its last waypoint first turns to face its final parking position when that lies more than about 45.6 degrees off its heading. |
| `PARKING_QUERY_SHORTFALL_METERS` | 1 | m | The check for units in the way of that turn ends this short of the final parking position. |
| `LOOK_DISTANCE_FACTOR` | 0.6 | share of distance | The corridor that a unit checks for units in its way spans this share of the distance to its waypoint. |
| `MAX_LOOK_RADII` | 4 | unit radii | Longest corridor that a unit checks for units in its way, in radii of the unit. |
| `MIN_LOOK_RADII` | 1.5 | unit radii | Shortest corridor that a unit checks, in radii of the unit, also when the corridor is halved. |
| `BEHIND_LOOK_FACTOR` | 0.5 | factor | The corridor that a unit checks for units in its way halves when the unit faces away from its waypoint. |
| `MAX_BLOCKER_TANGENT_SQUARED` | 100000 | m² | Squared distance beyond which no corridor reaches a unit in the way, about 316 m. |
| `NEW_BLOCKER_RADII` | 2.4 | unit radii | A new unit in the way at least this far away makes the unit pick a new side for its avoidance turn (a hard turn to one side, around the unit in the way). |
| `AVOID_TICKS` | 5 | ticks | Time that an avoidance turn or a blocking terrain slide marks a unit as avoiding, which is 28 ms at a frame of 5.6 ms (sim.h). |
| `DRIVING_ON_AVOID_TICKS` | 2 | ticks | A unit in the way that has avoided for at most this long still counts as driving on. |
| `WAIT_SECONDS` | 2 | s | Time that a unit waits behind a faster unit in its way before it makes an avoidance turn around it. |
| `AVOIDANCE_TURN_AFTER_WAIT_SECONDS` | -2 | s | A unit does not wait while its wait timer is at this value or more, so it makes an avoidance turn instead. |

### Collisions and blocking terrain (game/physics.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `PUSH_GAP_METERS` | 0.3 | m | Gap that a collision leaves between two bodies after it pushes them apart, so they no longer overlap. |
| `MIN_PUSHED_AWAY_SPEED` | -0.1 | m/s | Two units that move apart faster than this get no extra push speed from a collision. |
| `SLOW_PUSH_SPEED` | 0.1 | m/s | A pushed unit slower than this gets the speed `MIN_PUSH_SPEED` away from the other body. |
| `MIN_PUSH_SPEED` | 0.2 | m/s | Speed away from the other body that a collision gives a pushed unit slower than `SLOW_PUSH_SPEED`. |
| `SLIDE_OFFSET_CELLS` | 0.55 | cells | A blocking terrain slide (a slide along the side of a blocking cell) puts the unit this far from the center of the blocking cell, about 3.4 m. |
| `SIDE_COOLDOWN_SECONDS` | 2 | s | Blocking terrain slides flip the preferred avoidance turn side of a unit at most once in this time. |
| `SLOPE_RESCUE_STEP_CELLS` | 0.5 | cells | Step of the slope rescue, the search for free terrain when a unit ends up on a cell that blocks it, about 3.1 m. |

### Squads (game/squad.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `SQUAD_SEARCH_BUDGET` | 60 | expansions per tick | The path search of a squad runs this many expansions in each tick, so a long search takes several ticks. |
| `SQUAD_SEARCH_TOLERANCE_METERS` | 0 | m | The path search of a squad ends only exactly at its target, with no tolerance. |
| `MIN_LAST_REACH_METERS` | 1 | m | Smallest reach of the last waypoint of a member, which is otherwise the radius of the member. |
| `STRAGGLER_WAIT_SECONDS` | 20 | s | A squad counts as arrived this long after all its members but one stopped, even if that one still moves. |

## Our changes

Every value in this part belongs to the simulator. These values set formation following (`--avoid form`), the coordination between squads (`--avoid coord`), and the switches that turn each of their parts on or off.

### Changes to the base game movement

These values change the movement of the base game at a few points. Two of them fix faults of the game, and two change its behavior. The column "Fix or change" tells which.

| Constant | Value | Unit | Fix or change | What it sets |
|---|---|---|---|---|
| `SIGHT_TOUCH_MARGIN_METERS` | 0.5 | m | Change, formation modes only | A line of sight fails when it brings the unit this close to a unit of a parked squad (game/movement.c). |
| `SLOPE_RESCUE_MAX_CELLS` | 20 | cells | Fix, all modes | The slope rescue gives up at 125 m and leaves the unit where it is, where the base game searches on (game/physics.c). |
| `HIT_REACH_CELLS` | 1 | cells | Fix, formation modes only | The slide jump guard: a blocking terrain slide ignores a hit cell further than this from its ray, so units no longer jump far (game/physics.c). |
| `FULL_SEARCH_BUDGET` | 500 | expansions per tick | Change, all modes | With `--fullsearch on`, a squad search gets all its expansions in one tick, where the base game gives 60 (game/squad.c). |

The two fixes replace these behaviors of the base game:

- Slope rescue: When a unit stands on a cell that blocks it, the base game tries +x, -x, +y and -y at 0.5, 1.0, 1.5 cells and so on. The search has no other end, so it never ends when no cell of free terrain lies on those four lines.
- Blocking terrain slide: For a tiny move along a cell edge, the ray of the base game can return a blocking cell hundreds of meters away. The slide then moves the unit there in one tick.

The search change makes squads start sooner after an order. The base game ends every search after 500 expansions, so with `FULL_SEARCH_BUDGET` the search of a squad always ends in the tick where it starts. When the first search fails, the backup search of the game then runs in the same tick too. In the base game, a long search takes up to 3 ticks in the scenarios.

The sight change can change gameplay, because the game can also use a line of sight to decide whether a unit fires. GAMEPLAY_NOTES.md says what to check in the game before the change goes in.

One more change has no constant. In the formation modes, collision pushes never move the units of parked squads (game/physics.c), and in the base game they do. This change is different from the base game, and it is part of the collisions, not of the route planning.

### Parked squads (game/reroute.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `PARKED_STAMP_MARGIN_METERS` | 1 | m | Cells this close to the bodies of parked units count as cliffs in the searches of a moving squad (game/squad.c). |
| `PARKED_ROUTE_STEP_METERS` | 2 | m | When a squad parks, the rest of the route of each moving squad is checked at this step for it. |
| `PARKED_ROUTE_MARGIN_METERS` | 2 | m | A route counts as going past a parked unit when it comes within this gap of the body of the unit. |
| `TIGHT_CIRCLE_RADII` | 2 | unit radii | A unit circles tightly, and can count as settled, when it stays within this distance. |
| `PARK_AHEAD_SECONDS` | 16 | s | With `--parkreroute on`, a moving squad looks this long ahead on its route, at the speed of its anchor, for the final parking positions of a squad that parks there first. |
| `FREE_PARKING_MAX_RINGS` | 12 | rings | With `--freeparking on`, a final parking position that overlaps a parked unit moves to a free spot at most this many unit radii away (formation/stamping.c). It overlaps when its body comes within `PARKED_STAMP_MARGIN_METERS` of the parked body. |
| `FREE_PARKING_RING_POINTS` | 6 | spots | The free-spot search tries this many spots on the first ring around the position, and this many times the ring number on each later ring, one unit radius apart. |

With `--parkrobust on`, a squad whose re-route fails, or whose final parking positions no longer fit, keeps its old route and its final parking positions. In the base game, the move is then denied. This change has no constant.

The check of `PARK_AHEAD_SECONDS` measures the time of the moving squad until its route comes closest to the final parking positions of the other squad, not until its route first comes within reach of them. The time of the other squad also ends at its final parking positions, so both times end at the same point. This change has no constant.

### Route smoothing set-up (formation/route.c)

#### Route

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `corner_cut_margin_meters` (faction tuning) | 8 | m | A corner cut moves a corner only as far as a band stays clear of blocking terrain. The band reaches the radius of the widest member plus this to each side. |
| `CENTERED_ROUTE_MIN_GAIN_METERS` | 10 | m | The route of the centered game search (a base game path search from the center of the squad) replaces the route from the front unit only when it is this much shorter. |
| `REROUTE_MAX_LENGTH_FACTOR` | 1.25 | factor of length | While a squad re-routes, its route keeps its corners when the smoothing would make it longer by more than this factor (formation/smoothing.c). |
| `FORMATION_DIRECTION_SPAN_METERS` | 10 | m | The formation direction at a point is that of the route from this far before it to this far after it. |
| `MIN_CHORD_LENGTH_SQUARED` | 1 | m² | That span gives a direction only above this squared length, and otherwise the direction of the route segment counts. |
| `ROUTE_END_EPSILON_METERS` | 0.01 | m | An anchor this close to the end of the route counts as at the end, where the members stop (sim.h). |

#### Curved route (formation/smoothing.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `CONTROL_SPACING_METERS` | 16 | m | Largest distance between two control points along the route, which sets how finely the curve can bend. |
| `CURVE_POSITION_SPACING_METERS` | 4 | m | Distance between two curve positions, where the clearance tests run and which become the points of the route. |
| `MAX_FULL_ROUTE_METERS` | 8000 | m | Longest route that keeps both spacings above, and longer routes scale them up in proportion to length. |
| `FREE_DISTANCE_STEP_METERS` | 0.5 | m | Step of the rays that measure free distances, in the clearance tests and in the lane layout (formation/formation.h). |
| `CELL_SKIP_MARGIN_METERS` | 0.01 | m | A ray skips only the probes that lie at least this far inside a free cell, so the skip never changes a result (formation/route.c). |
| `LANE_MAX_DISTANCE_METERS` | 30 | m | Longest free distance that a ray measures, and terrain further away counts as free terrain (formation/formation.h). A wider formation measures farther: its lanes as far as its outer slots plus the widest radius plus `LANE_MARGIN_METERS`, and its clearance tests as far as half the passage width that its formation clearance needs. |
| `BODY_MARGIN_METERS` | 0.5 | m | The body clearance (the clearance for the body of the widest member) is its radius plus this, and every curve position needs it. |
| `LANE_MARGIN_METERS` | 1.5 | m | The clearance for the whole formation is half the formation width plus the radius of the widest member plus this (formation/formation.h). |
| `PASSAGE_MIDDLE_MARGIN_METERS` | 0.25 | m | In a narrow passage, the wanted clearance is half the passage width less this, so that the curve runs down the middle. |
| `AWAY_SEARCH_MARGIN_METERS` | 1 | m | A displacement looks for the closest blocking terrain up to this far past the wanted clearance of the cramped position. |
| `DISPLACEMENT_STEP_METERS` | 0.25 | m | Distance that one displacement moves the control point closest to the cramped position away from the blocking terrain. |
| `DISPLACEMENT_SPREAD_METERS` | 64 | m | Displacements reach this far each way, 4 control points, so ahead they leave out the last point of the 80 m fade. |
| `DISPLACEMENT_FADE_METERS` | 80 | m | Distance from the cramped position over which a displacement fades from full strength to zero, as a cosine bell. |

#### Widening of bends (formation/smoothing.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `BEND_SPEED_MARGIN` | 0.8 | factor | Full speed holds in bends whose radius is at least top speed / (this × turn rate) (formation/formation.h). |
| `WIDEN_STEP_FRACTION` | 0.5 | share of excess | Share of the curvature above the wanted limit that one widening iteration takes away at a bend. |
| `BEND_STENCIL_METERS` | 8 | m | The widening measures the sharpness of a bend from the curve this far before and after each curve position. |
| `TIGHTEN_STEP_FRACTION` | 0.5 | share of way | With `--tightbends`, share of the way to the midpoint of its two neighbors that a control point moves in one widening iteration. |
| `TIGHTEN_CURVATURE_FRACTION` | 0.9 | share of limit | The tightening keeps bends below this share of the curvature limit of the widening, so it does not undo the widening. |
| `TIGHTEN_MIN_MOVE_METERS` | 0.05 | m | Smallest control point move that counts as tightening, which keeps the widening iterations going. |
| `TIGHTEN_MIN_GAIN_METERS` | 0.5 | m | A widened curve with the same bend excess counts as better only when it is this much shorter. |

#### Lead-in turn at a new order (`--smoothreorder`)

A squad that gets a new order while it moves starts its route with a turn along a circle that leaves along its current motion. The widening and the tightening of bends do not move the control points of this turn.

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `REORDER_MIN_SPEED` | 2 | m/s | A squad whose members move slower than this on average counts as standing at a new order, so its route gets no lead-in turn and its anchor starts at speed 0 (formation/formation.c). |
| `LEAD_IN_RADIUS_FRACTION` | 0.7 | share of radius | Radius of the lead-in turn, as a share of the largest radius at which a squad member turns at its current speed (current speed / (`BEND_SPEED_MARGIN` × turn rate)). Below 1, the members slow down in the turn (formation/formation.c). |
| `LEAD_IN_STEP_RADIANS` | π/4 | rad | Largest turn between two lead-in points on the circle. |
| `LEAD_IN_MIN_SWEEP_RADIANS` | 0.2 | rad | A turn smaller than this gets no lead-in points. |

### Lanes (formation/lanes.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `LANE_STEP_METERS` | 2 | m | Distance between two lane cross-sections along the route, where the lanes and their free distances are laid out. |
| `MAX_FULL_LANE_METERS` | 8000 | m | Longest route that keeps `LANE_STEP_METERS`, and longer routes get a lane spacing that grows with their length. |
| `LANE_CLAMP_MARGIN_METERS` | 0.5 | m | The lane clamp keeps the body of the widest unit at least this far from blocking terrain. |
| `LANE_SLOPE` | 0.3 | m per m | The edges of the free range of the lanes move sideways by at most this much per meter of route. |
| `SAME_LANE_METERS` | 0.5 | m | Slots that are closer than this sideways share one lane, one behind the other. |
| `squeezed_gap_meters` (faction tuning) | 3 | m | Lanes squeezed in a narrow passage keep two radii of the widest unit plus this gap apart. |
| `FREE_WINDOW_METERS` | 4 | m | At each lane cross-section, the lanes keep clear of the narrowest free distance within this far before and after. |
| `AVERAGE_WINDOW_METERS` | 6 | m | The lanes are averaged over this far before and after each lane cross-section, but at most over `COLUMN_WINDOW_METERS`. |
| `COLUMN_WINDOW_METERS` | 8 | m | Column formation starts this far before a passage too narrow for the squad in square formation, and ends this far after it. |
| `BELL_REACH_METERS` | 40 | m | A sideways move of the lane offset (the sideways offset that all lanes of a cross-section share) fades in and out over this distance on each side, as a smooth bump. |
| `FOLD_SHRINK_FACTOR` | 0.8 | factor | Each pull at a fold, where an inner lane runs backward in a sharp bend, scales the sideways offsets of all lanes from the route by this. |
| `MIN_FORWARD_STEP_FRACTION` | 0.25 | share of lane spacing | The outer lanes must move forward by at least this share of the lane spacing from one lane cross-section to the next. |

### Slots and anchor (formation/formation.c)

#### Slots and swaps

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `START_BLEND_METERS` | 25 | m | The slots blend from where the members stand into the formation while the anchor drives this far from the route start. |
| `END_BLEND_METERS` | 25 | m | The slots blend into the shape of the final parking positions over this last distance of the route (formation/formation.h). |
| `column_gap_radii` (faction tuning) | 2.2 | unit radii | Column gap in radii of the widest member, before `COLUMN_GAP_EXTRA_METERS` is added. |
| `COLUMN_GAP_EXTRA_METERS` | 2 | m | Extra distance added to the column gap, so medium tanks keep 2.2 × 3.5 + 2 = 9.7 m. |
| `COLUMN_HEADWAY_SECONDS` | 0.3 | s | With lanes, the column gap also grows by this time at the speed of the anchor. |
| `MIN_FORMATION_REACH_METERS` | 2 | m | Smallest reach of a member in the formation modes, which is otherwise its radius. |
| `COORDINATES_BEHIND_METERS` | 80 | m | The position of a member along the route is sought up to this far before the anchor. |
| `COORDINATES_AHEAD_METERS` | 40 | m | The position of a member along the route is sought up to this far past the anchor. |
| `SWAP_INTERVAL_SECONDS` | 0.5 | s | The members try to swap slots this often, and they swap when that makes their ways shorter. |
| `SWAP_SIDE_WEIGHT` | 4 | factor | A sideways difference between a member and a slot counts this many times in the cost of a swap. |
| `SWAP_MIN_SAVING_FRACTION` | 0.1 | share of cost | A swap happens only when it saves at least this share of the cost. |
| `SWAP_MIN_SAVING_SQUARED` | 10 | m² | A swap must also save at least this much cost, as a sum of squared distances. |

#### Anchor speed

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `LAG_SLOWDOWN_FRACTION` | 0.1 | share of speed | Members that lag behind their slots slow the anchor by up to this share of its speed. |
| `LAG_SLOWDOWN_METERS` | 4 | m | Lag behind the slot at which that slowdown reaches its full share, after growing in step with the lag. |
| `REAR_MARGIN_METERS` | 10 | m | Checks along the route for bends and column formation reach this far past the front and the rear of the formation. |
| `COLUMN_LEAVE_AHEAD_METERS` | 20 | m | Column formation ends once the squad fits in square formation up to this far past its front. |
| `END_DECELERATION_FRACTION` | 0.5 | share of friction | The anchor brakes before the route end with this share of the lowest friction of the members. |
| `lag_wait_meters` (faction tuning) | 8 | m | When a member lags more than this, the anchor slows further to wait for it. |
| `LAG_WAIT_RANGE_METERS` | 20 | m | That waiting slowdown grows by one full share for each this many meters of lag. |
| `CRAWL_FRACTION` | 0.25 | share of speed | A waiting anchor keeps at least this share of its speed, so the squad keeps crawling forward. |
| `LAG_STOP_METERS` | 60 | m | The anchor stops completely only when a member lags more than this, and below it keeps crawling. |
| `ANCHOR_STILL_SPEED` | 0.2 | m/s | Below this speed, the anchor counts as nearly still, which starts the timer of `STUCK_WAIT_SECONDS`. |
| `STUCK_WAIT_SECONDS` | 10 | s | After this long nearly still, the anchor moves on, so that one stuck member cannot stop the squad forever. |
| `IGNORE_LAG_SECONDS` | 5 | s | After that wait, the anchor ignores members that lag behind for this long, and then waits for them again. |

### Member steering (formation/steering.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `FORM_SPEED_TIME_CONSTANT_SECONDS` | 0.1 | s | The throttle of a member closes a gap to its wanted speed in about this time. |
| `PROGRESS_BEHIND_METERS` | 10 | m | The search for the position of a member along the route starts this far before its last position. |
| `PROGRESS_AHEAD_METERS` | 30 | m | That search ends this far past the last position of the member, so the position never jumps further. |
| `REJOIN_BEHIND_METERS` | 12 | m | A member this far behind its slot drives back to its lane instead of to its slot. |
| `REJOIN_LOOK_METERS` | 10 | m | The point where a member rejoins its lane lies at most this far past its position along the route. |
| `REJOIN_STEP_METERS` | 2.5 | m | Rejoin points along the lane are tried this far apart, up to `REJOIN_LOOK_METERS` past the member. |
| `NO_LANE_REJOIN_METERS` | 60 | m | With `--lanes off`, rejoin points lie up to this far past the position of the member. |
| `pursuit_slot_meters` (faction tuning) | 4 | m | A member further than this from its slot pursues a point ahead on its lane. |
| `PURSUIT_MIN_METERS` | 6 | m | The pursued point lies at least this far past the position of the member along the route. |
| `PURSUIT_MAX_METERS` | 16 | m | The pursued point lies at most this far past that position, also for a member at top speed. |
| `pursuit_look_seconds` (faction tuning) | 0.8 | s | The pursued point lies this long ahead at the speed of the member, within the two limits above. |
| `PURSUIT_STEP_METERS` | 2 | m | Points for pursuit are tried this far apart along the lane, from the furthest one back toward the member. |
| `PURSUIT_MIN_AHEAD_METERS` | 2 | m | Pursuit tries points down to this distance past the position of the member. |
| `CATCH_UP_GAIN` | 0.8 | (m/s) per m | A member behind its slot drives this much faster than the anchor for each meter behind. |
| `AHEAD_SPEED_FLOOR_FRACTION` | 0.5 | share of anchor speed | A member ahead of its slot keeps at least this share of the speed of the anchor. |
| `AHEAD_DECELERATION_FRACTION` | 0.2 | share of friction | A member ahead of its slot slows with this share of its friction, so it falls back gently. |
| `COLUMN_AHEAD_SPEED_FLOOR_FRACTION` | 0.2 | share of anchor speed | In column formation, a member ahead of its slot keeps this smaller share instead. |
| `COLUMN_AHEAD_DECELERATION_FRACTION` | 0.5 | share of friction | In column formation, a member ahead of its slot slows with this larger share instead. |
| `SLOT_GAIN` | 0.8 | (m/s) per m | A member near its slot moves toward it at this speed for each meter away, on top of the motion of the anchor. |
| `ARRIVAL_BRAKING_FRACTION` | 0.7 | share of friction | Plans to stop at a target assume braking with this share of the friction, which leaves a margin. |
| `ARRIVAL_SLACK_METERS` | 0.5 | m | Plans to stop at a target aim this far short of it, so that a member does not overshoot its slot. |
| `PARKING_SPEED` | 2 | m/s | A member on its final parking position at the route end that drives slower than this stops there and turns to its facing. |
| `END_STOP_REACH_METERS` | 1 | m | On the end section of the route (`--driving on`), a member that drives straight at its final parking position counts as on it within this distance. |
| `END_BRAKING_FRACTION` | 0.3 | share of friction | On the end section of the route, a member brakes onto its final parking position with this share of the friction, for a soft stop. |
| `AVOIDANCE_MIN_CHANGE_SQUARED` | 0.25 | (m/s)² | Changes of the velocity by the ORCA avoidance below 0.5 m/s are ignored, and the member keeps its wanted velocity. |
| `WAIT_SPEED_FRACTION` | 0.25 | share of top speed | A member that the ORCA avoidance keeps below this share of its top speed, against its wish, counts as waiting. |
| `ORCA_WAIT_LIMIT_SECONDS` | 3 | s | A member that waits for the ORCA avoidance for longer than this drives on, without avoidance for a while. |
| `AVOIDANCE_OFF_SECONDS` | 2 | s | A member that drove on through the ORCA avoidance then drives without it for this long. |
| `BEND_LOOK_MIN_METERS` | 8 | m | Checks for bends ahead look at least this far past the position of the member. |
| `BEND_LOOK_SECONDS` | 0.8 | s | With `--driving off`, checks for bends look this long ahead at the speed of the member. |
| `far_bend_look_seconds` (faction tuning) | 1.2 | s | With `--driving on`, the checks for bends ahead look this long ahead instead. |
| `BEND_SLOT_FRACTION` | 0.5 | share of distance | The checks for bends also look this share as far past the slot of the member. |
| `BEND_MIN_DISTANCE_METERS` | 1 | m | The bend check gives no bend maximum speed when its lane point lies within this distance of the member. |
| `BEND_MIN_SINE` | 0.05 | sine | Points of a bend almost straight ahead, within about 3 degrees, give no bend maximum speed. |
| `BEND_FLOOR_FRACTION` | 0.2 | share of top speed | The bend maximum speed never falls below this share of the top speed, so units keep moving around the bend. |
| `heading_time_constant_seconds` (faction tuning) | 0.1 | s | The steering closes a gap to the wanted heading in about this time, whatever the turn rate of the unit. |
| `brake_look_seconds` (faction tuning) | 0.4 | s | The brake check for blocking terrain ahead looks this long ahead along the heading. |
| `BRAKE_LOOK_MARGIN_METERS` | 1 | m | That check looks at least the radius of the member plus this far ahead, also when the member is slow. |
| `STEER_MIN_SPEED` | 0.5 | m/s | Below this speed, a member steers straight toward its target instead of along its velocity. |
| `STEER_MIN_DISTANCE_METERS` | 0.5 | m | A target this close gives no steering when the member is also slower than `STEER_MIN_SPEED`. |
| `WAIT_AHEAD_MIN_COSINE` | 0.87 | cosine | A member past its slot that faces within 30 degrees of forward waits instead of turning (`--formation`). |

### ORCA avoidance and stepping aside (formation/avoidance.c)

The ORCA avoidance between units (optimal reciprocal collision avoidance) picks for each unit the velocity closest to the wanted one that stays clear of the other units for some time.

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `ORCA_HORIZON_SECONDS` | 2 | s | Units must stay clear of each other for this long at their new velocities, so they react this early. |
| `orca_margin_meters` (faction tuning) | 0.5 | m | Extra gap that the ORCA avoidance keeps between the bodies of two units, on top of their radii. |
| `MEMBER_AVOID_RANGE_METERS` | 2 | m | Members of the same squad avoid each other only at body gaps up to this plus `ORCA_MARGIN_METERS`. |
| `SHARED_EFFORT` | 0.5 | share of change | Each of two moving units makes this share of the change of velocity that keeps them apart. |
| `FULL_EFFORT` | 1 | share of change | A unit makes the whole change when the other unit cannot dodge, for example one that turns on the spot. |
| `SPEED_TIME_CONSTANT_SECONDS` | 0.3 | s | A stopped unit that steps aside, or returns to its final parking position, reaches its new speed in about this time. |
| `MIN_STEP_ASIDE_SPEED` | 1 | m/s | A stopped unit steps aside only when the ORCA avoidance asks it to move faster than this. |

### Stuck units and detours (formation/detour.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `PROGRESS_CHECK_SECONDS` | 2 | s | The progress check compares the positions of a unit this far apart in time. |
| `MIN_PROGRESS_METERS` | 1 | m | A unit that moved less than this in that time made no progress and counts as stuck. |
| `STUCK_SLOT_METERS` | 6 | m | A unit this close to its slot never counts as stuck, even when it hardly moves. |
| `DETOUR_AHEAD_METERS` | 20 | m | A detour for a stuck unit ends on its lane at most this far along the route past its progress. |
| `DETOUR_TOLERANCE_METERS` | 2 | m | The path search of a detour can end this close to the end point of the detour instead of on it. |
| `DETOUR_LIMIT_SECONDS` | 20 | s | A unit drops a detour that takes longer than this, and goes back to following its slot. |

### Coordination between squads (formation/coordination.c)

The rules for crossings, head-ons and passing, with their constants, were removed: the predictions of `--moveahead` (game/moving_ahead.c) replace them. The coordination keeps the give-up timer of the speed caps.

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `CAP_WAIT_SPEED` | 0.2 | m/s | A speed cap below this keeps the anchor nearly still, which starts the timer of `GIVE_UP_SECONDS`. |
| `GIVE_UP_SECONDS` | 15 | s | After this long nearly still, a squad stops giving way, so that two squads cannot block each other forever. |
| `IGNORE_SECONDS` | 5 | s | After it gives up, the squad ignores the other squads for this long, and then coordinates again. |

### Moving squads ahead (game/moving_ahead.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `STEP_SECONDS` | 0.25 | s | Time step of the predictions; a kept yield ends when its predicted wait falls below one step. |
| `MAX_PREDICTION_SECONDS` | 120 | s | The predictions end this far ahead. |
| `MIN_PREDICTED_SPEED` | 0.5 | m/s | A squad moves at least this fast in the predictions. |
| `MIN_WAIT_SECONDS` | 2 | s | A predicted wait this long or longer starts the trial searches of a re-route. |
| `MIN_YIELD_WAIT_SECONDS` | 0.5 | s | A predicted wait this long or longer starts a speed cap. |
| `MIN_GAIN_SECONDS` | 1 | s | A found route must arrive at least this much earlier. |
| `GAP_METERS` | 2 | m | Two formations closer than this count as meeting. |
| `TIME_MARGIN_SECONDS` | 0.5 | s | The formation of another squad counts this much earlier and later along its route. |
| `CORRIDOR_MARGIN_SECONDS` | 2 | s | The corridor of another squad starts this long before its first predicted wait and ends this long after its last one. |
| `CORRIDOR_MARGIN_METERS` | 1 | m | The corridor reaches this far past the formation of the other squad. |
| `STAMP_STEP_FRACTION` | 0.5 | disc radii | Spacing of the discs that the trial searches go around. |
| `SAME_WAY_COSINE` | 0.7 | cosine | Two squads less than 45 degrees apart at their meeting drive the same way. |
| `HEAD_ON_COSINE` | -0.7 | cosine | Two squads more than 135 degrees apart at their meeting meet head-on. |
| `DIRECTION_SPAN_METERS` | 1 | m | The direction at a route point comes from the route this far before and after it. |
| `DEVIATION_STEP_METERS` | 4 | m | Spacing of the points of a found route in the check of its distance from the current route. |
| `DEVIATION_MARGIN_CELLS` | 3 | cells | A found route may stray this far past the needed gap from the current route. |
| `PARKING_LOOK_SECONDS` | 16 | s | A squad re-routes around the final parking positions of another squad when they would block it within this time. |
| `WAIT_LOOK_SECONDS` | 10 | s | A squad adapts only to predicted waits that start within this time. |
| `EARLY_SIDE_STAMP_RADII` | 3 | disc radii | In the second head-on search of a squad whose route strayed to the wrong side, the discs start this far ahead of its front. |
| `LATE_YIELD_SPEED_FRACTION` | 0.5 | share of top speed | With `--lateyield`, at a crossing, the speed cap acts only once it is below this share of the top speed of the squad. |
| `YIELD_HOLD_SECONDS` | 2 | s | After a yield ends, the other squad still counts the squad as giving way for this long. |
| `CLOSE_PASS_MARGIN_METERS` | 4 | m | With `--closepass`, two squads that meet head-on move apart by the needed gap less their sideways offset at the meeting, plus this. |
| `CLOSE_PASS_CELLS` | 0.5 | cells | With `--closepass`, the search discs of a head-on re-route shrink by this. |
| `CLOSE_PASS_SET_UP_CELLS` | 0.5 | cells | With `--closepass`, the set-up discs of a head-on re-route shrink by this. |
| `CLOSE_PASS_MIN_RADIUS_CELLS` | 2 | cells | With `--closepass`, a search disc of a head-on re-route keeps at least this radius. |

### Work limits and capacities

These constants have no physical meaning. They limit the work of route planning, or they size its arrays. The three `SET_UP_WORK_*` constants apply only with `--setupaccumulate on`, which spreads the route smoothing set-up over several ticks (temporal accumulation). With that switch off, each route smoothing set-up runs in full in one tick. With `--setupthreads on`, the smoothings of the squads that set up in the same tick run at the same time on worker threads, one for each processor core but one.

| Constant | Value | File | What it sets |
|---|---|---|---|
| `SMOOTHING_ITERATIONS` | 60 | formation/smoothing.c | Largest number of displacement iterations over the whole curve before the curve is judged. |
| `WIDEN_ITERATIONS` | 40 | formation/smoothing.c | Largest number of iterations that widen sharp bends, before displacement iterations follow, up to 100 iterations in all. |
| `WIDEN_PATIENCE_ITERATIONS` | 30 | formation/smoothing.c | The widening stops this many iterations after its last better curve. |
| `BODY_PATIENCE_ITERATIONS` | 10 | formation/smoothing.c | With `--smoothstop`, a curve that still runs too close to blocking terrain for the body of a unit after this many displacement iterations ends the smoothing, and the route stays unchanged. |
| `SET_UP_WORK_PER_TICK` | 4000 | game/squad.c | With `--setupaccumulate on`, the work in clearance tests that all route smoothing set-ups of one tick can share. |
| `SET_UP_WORK_PER_ROUTE_CORNER` | 60 | game/squad.c | Work in clearance tests charged for the first part of a set-up, per route corner. |
| `SET_UP_WORK_PER_CROSS_SECTION` | 2 | game/squad.c | Work in clearance tests charged for the last part of a set-up, per lane cross-section, fitted to measured times. |
| `MAX_JOBS` | 32 | workers.c | The job list holds this many smoothings, more than a tick hands over, as a run has at most 8 squads. |
| `CLEARANCE_MEMO_START_BITS` | 14 | formation/smoothing.c | The clearance memo (the table that remembers clearance tests) starts with 2^14 (16,384) entries, enough for most smoothings. |
| `CLEARANCE_MEMO_MAX_LOAD` | 0.5 | formation/smoothing.c | The clearance memo doubles before it would be more than half full, so it holds the clearance tests of any smoothing. |
| `COS_CACHE_BITS` | 12 | formation/smoothing.c | The table that remembers the cosines of the displacement fades has 2^12 (4,096) entries. |
| `MAX_WAYPOINTS` | 4096 | sim.h | Points that a route, a path or a waypoint list can hold, so a curved route of 8 km at 4 m fits. |
| `LANE_MAX_CROSS_SECTIONS` | 4096 | formation/lanes.c | Lane cross-sections that each squad can hold, which covers 8 km of route at 2 m, and 2 more. |
| `ROUTE_POSITION_COUNT` | 128 | formation/formation.h | Route positions that one coordination check can hold, 768 m at 6 m, while the longest check reads 600 m. |

### Counts that follow from distances

The code computes each count of points from a distance and the spacing of the points that it counts, once per route. A count is the distance divided by the spacing, rounded to the nearest whole number, and it is at least 1. So the reach in meters stays the same when a spacing changes. On a route longer than 8 km, the spacings grow, and the counts get smaller to match.

| Distance | Spacing | Count at the values above |
|---|---|---|
| `BEND_STENCIL_METERS` | `CURVE_POSITION_SPACING_METERS` | 2 curve positions |
| `DISPLACEMENT_SPREAD_METERS` | `CONTROL_SPACING_METERS` | 4 control points |
| `DISPLACEMENT_FADE_METERS` | `CONTROL_SPACING_METERS` | 5 control points |
| `FREE_WINDOW_METERS` | `LANE_STEP_METERS` | 2 lane cross-sections |
| `AVERAGE_WINDOW_METERS` | `LANE_STEP_METERS` | 3 lane cross-sections |
| `COLUMN_WINDOW_METERS` | `LANE_STEP_METERS` | 4 lane cross-sections |
| `BELL_REACH_METERS` | `LANE_STEP_METERS` | 20 lane cross-sections |

## Testing

These constants serve only the tests: the length of a run, the test scenarios, the metrics that judge the movement, and the route finder that helps to pick scenarios. They do not change how units move.

### Runs (sim.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `MAX_RUN_SECONDS` | 240 | s | Longest simulated time of one run, after which the run ends even when squads still move. |
| `SETTLE_SECONDS` | 3 | s | Time that a run goes on after no squad is busy any more, so that the units come to rest. |

### Scenarios and seeds (scenarios.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `START_GAP_METERS` | 0.5 | m | Units that `--seed` moves keep at least this gap between their bodies at the start. |
| `SQUAD_JITTER_SPANS` | 1.5 | squad spans | `--seed` moves the start of each squad by up to this many times the size of the squad. |
| `UNIT_JITTER_METERS` | 1.5 | m | After the squad, `--seed` moves each unit by up to this much on each axis, onto free terrain. |
| `UNIT_JITTER_RADIANS` | 0.05 | rad | `--seed` also turns each unit by up to this angle, about 3 degrees, either way. |
| `RIDGE_GAP_METERS` | 50 | m | In `ridge`, the medium and the heavy squad start and drive next to each other, this far apart. |
| `FOLLOW_GAP_METERS` | 45 | m | In `corridor`, the three squads start this far apart, one behind the other on the same route. |
| `SPREAD_DESTINATION_METERS` | 40 | m | In `corridor`, the destinations of the rear squads lie this far to the side. |
| `WINDING_GAP_METERS` | 60 | m | In `winding`, the squads start this far apart, and their destinations lie this far apart as well. |
| `SIX_SQUADS_GAP_METERS` | 40 | m | In `sixsquads`, the squads stand this far apart in each row, the old one and the new one. |
| `SIX_SQUADS_ADVANCE_METERS` | 250 | m | In `sixsquads`, the middle of the new row lies this far ahead of the old one. |

### Metrics and trace (metrics.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `FAST_SPEED_FRACTION` | 0.5 | share of top speed | Above this speed a unit counts as driving, and a stop starts when a driving unit gets slow. |
| `SLOW_SPEED_FRACTION` | 0.15 | share of top speed | Below this share of its top speed a unit counts as slow, for the stops and the slow time. |
| `TOP_SPEED_FRACTION` | 0.95 | share of top speed | At this speed or more a unit counts as at top speed, for the time at top speed. |
| `STUCK_AFTER_SECONDS` | 3 | s | A unit that stays slow for longer than this counts as stuck, for the stuck time of the results. |
| `ARRIVING_RADII` | 3 | unit radii | Within this distance of its final parking position a unit counts as arriving, and its stops and slow time no longer count. |
| `TURN_ON_SPOT_RADIANS_PER_SECOND` | 0.4 | rad/s | A turn faster than this at low speed counts as a turn on the spot, for the time of turns on the spot in the results. |
| `TURN_ON_SPOT_SPEED_FRACTION` | 0.2 | share of top speed | For a turn on the spot, the unit must also be slower than this share of its top speed. |
| `TERRAIN_DEPTH_METERS` | 0.5 | m | A body that reaches deeper than this into blocking terrain counts as inside blocking terrain. |
| `TOUCH_MARGIN_METERS` | 0.35 | m | Bodies this close count as touching, a bit more than the 0.3 m gap that collisions leave between them. |
| `LOST_RADII` | 3 | unit radii | A unit that ends further than this from its final parking position counts as lost, for the lost count of the results. |
| `TRACE_GRID_MARGIN_METERS` | 60 | m | The map in the trace reaches this far past the area that the units of the run use. |
| `TRACE_LANE_STEP_METERS` | 4 | m | Step of the lane points that the trace stores, from which the replay draws the lanes. |

### Route finder (find.c)

These constants serve `sim.exe --find`, which measures random routes on a map to help pick scenarios.

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `MAP_SIZE_METERS` | 6400 | m | Side of the square map, `GC_GRID_SIZE` cells of 6.25 m each, which holds the random route ends. |
| `WORLD_MIN_METERS` | 10 | m | Smallest x or y of a route end, the same bound that Integrate keeps units in (see "Map and time"). |
| `WORLD_MAX_METERS` | 6390 | m | Largest x or y of a route end, with the same source as `WORLD_MIN_METERS` above. |
| `MIN_SPOT_CLEARANCE_METERS` | 25 | m | A random route end must have at least this much clearance to be used. |
| `MIN_ROUTE_METERS` | 250 | m | The two ends of a random route lie at least this far apart in a straight line. |
| `ROUTE_RANGE_METERS` | 450 | m | A random distance of up to this is added to that, so routes run 250 to 700 m in a straight line. |
| `TURN_DEGREES` | 30 | degrees | A bend of the path that the base game path search finds counts as a turn when it is sharper than this. |
| `MEASURE_STEP_METERS` | 2 | m | The width of the passage is measured this often along a route, to find its narrowest point. |
| `END_EXCLUSION_METERS` | 30 | m | The width of the passage is not measured within this distance of either end of the route. |
| `HALF_WIDTH_SEARCH_CELLS` | 4 | cells | The search for the half-width of the passage reaches this far from the route, 4 cells or 25 m. |
| `STRAIGHT_STEP_COST` | 3 | cost units | Cost of a step to a cell in the same row or column, in the search for the half-width. |
| `DIAGONAL_STEP_COST` | 4 | cost units | Cost of a diagonal step in that search, close to 3 times the square root of 2. |

## Tuning status

This part shows which constants of the part "Our changes" were tuned. A constant counts as tuned when test runs compared it at two or more values. Each scan changed one constant and kept the others, so no scan tested two constants together. The column "Runs" tells how the values were compared:

- Recent: 100 seeds of all 12 scenarios in the coord mode, from 2026-11-14 on.
- Older: an earlier version of the simulator with fewer scenarios. The best value of such a scan can have moved since.

### Tuned by scans

| Constant | Value | Other values tried | Runs |
|---|---|---|---|
| `CONTROL_SPACING_METERS` | 16 | 4, 8, 12, 24, 32 | Recent |
| `CURVE_POSITION_SPACING_METERS` | 4 | 0.5, 1, 2, 3, 5, 6, 8, 10, 12, 16, 20 | Recent |
| `FREE_DISTANCE_STEP_METERS` | 0.5 | 0.125, 0.25, 1, 2 | Recent |
| `DISPLACEMENT_SPREAD_METERS` | 64 | 16, 32, 48, 80 | Recent |
| `DISPLACEMENT_FADE_METERS` | 80 | 48, 64, 96, 112 | Recent |
| `BEND_STENCIL_METERS` | 8 | 4, 12, 16, 24 | Recent |
| `BEND_SPEED_MARGIN` | 0.8 | 0.9, 1.0 | Older, 9 scenarios |
| `LANE_STEP_METERS` | 2 | 0.5, 1, 4, 8 | Recent |
| `FREE_WINDOW_METERS` | 4 | 2, 8, 12 | Recent |
| `AVERAGE_WINDOW_METERS` | 6 | 2, 4, 10, 14 | Recent, and values above 8 act as 8 |
| `COLUMN_WINDOW_METERS` | 8 | 4, 12, 16, 24 | Recent |
| `BELL_REACH_METERS` | 40 | 20, 30, 60, 80 | Recent |
| `COLUMN_AHEAD_SPEED_FLOOR_FRACTION`, `COLUMN_AHEAD_DECELERATION_FRACTION` | 0.2, 0.5 | The pairs 0.5 and 0.2, 0 and 0.5, 0.2 and 1.0 | Older, 9 scenarios |
| `END_BRAKING_FRACTION` | 0.3 | 0.2, 0.5, 0.7 | Older, 5 seeds of 9 scenarios |
| `FAR_BEND_LOOK_SECONDS` | 1.2 | 1.0, 1.6 | Older, 9 scenarios |
| `HEADING_TIME_CONSTANT_SECONDS` | 0.1 | 0.07, 0.15, 0.25 | Older, 9 scenarios |
| `WAIT_AHEAD_MIN_COSINE` | 0.87 | 0 (90 degrees) | Recent |
| `PASS_SQUEEZE_METERS` | 3 | 6 | Older, corridor only |
| `FORMED_LANE_METERS` | 2 | 3, 4, with the same results | Older, 9 scenarios |
| `FORMED_FRACTION` | 0.9 | 0.5, 0.7, with the same results | Older, 9 scenarios |
| `SET_UP_WORK_PER_TICK` | 4000 | 2000, 6000, 8000, 12000 | Recent, start delay against tick time |
| `WIDEN_PATIENCE_ITERATIONS` | 30 | 10, 20 | Recent with 40 seeds, routes that change against work saved |
| `BODY_PATIENCE_ITERATIONS` | 10 | 4, 5, 6, 8, 15, 20 | Recent with 100 seeds, checked on the recorded iterations of each smoothing, routes that change against work saved |
| `PARK_AHEAD_SECONDS` | 16 | 1, 2, 3, 5, 8, 12, 24, 32 | Recent, with the follow rule for crossing routes, the old-route fallback and the parking point on |
| `WAIT_LOOK_SECONDS` | 10 | no limit, 4, 6, 16 | Recent, 17 scenarios, without the coordination rules |
| `DEVIATION_MARGIN_CELLS` | 3 | 6 | Recent, 17 scenarios, without the coordination rules |
| `EARLY_SIDE_STAMP_RADII` | 3 | 1, 5 | Head-on seeds 0 and 34 only |
| `CLOSE_PASS_MARGIN_METERS` | 4 | 0, 2 | Recent, 17 scenarios |
| `CLOSE_PASS_MIN_RADIUS_CELLS` | 2 | 1 | Head-on seeds 0, 34 and 85 only |

### Faction tuning

The rows marked "faction tuning" are fields of `FactionTuning` (formation/tuning.c), not `#define`s. Their values above are those from before the tuning, which the Phoenix units keep and which every faction uses with `--factiontuning off`. With `--factiontuning on`, the default, the Crayven and Order units use their own values. Parameter scans found them: each constant alone, then in combinations, over 300 seeds of every scenario in the coordination mode. NOTES.md ("Faction tuning") has the measured effects.

| Field | Off (all), Phoenix | Crayven | Order |
|---|---|---|---|
| `brake_look_seconds` | 0.4 | 0.4 | 1.6 |
| `far_bend_look_seconds` | 1.2 | 1.6 | 1.2 |
| `heading_time_constant_seconds` | 0.1 | 0.05 | 0.1 |
| `pursuit_slot_meters` | 4 | 4 | 8 |
| `pursuit_look_seconds` | 0.8 | 0.8 | 0.8 |
| `orca_margin_meters` | 0.5 | 1.0 | 0.5 |
| `squeezed_gap_meters` | 3 | 3 | 3 |
| `column_gap_radii` | 2.2 | 2.2 | 2.2 |
| `corner_cut_margin_meters` | 8 | 8 | 8 |
| `lag_wait_meters` | 8 | 16 | 8 |

### Not tuned yet

No test compared these constants at several values on their own. Some came with a switch, and their tests compared only the switch on and off. A few changed once as part of a larger fix, such as the crossing margins.

- Changes to the base game movement: `SIGHT_TOUCH_MARGIN_METERS`, `SLOPE_RESCUE_MAX_CELLS`, `HIT_REACH_CELLS`.
- Parked squads: `PARKED_STAMP_MARGIN_METERS`, `PARKED_ROUTE_STEP_METERS`, `PARKED_ROUTE_MARGIN_METERS`, `TIGHT_CIRCLE_RADII`.
- Route: `CORNER_CUT_MARGIN_METERS`, `CENTERED_ROUTE_MIN_GAIN_METERS`, `REROUTE_MAX_LENGTH_FACTOR`, `FORMATION_DIRECTION_SPAN_METERS`, `MIN_CHORD_LENGTH_SQUARED`, `ROUTE_END_EPSILON_METERS`.
- Curved route and widening of bends: `MAX_FULL_ROUTE_METERS`, `CELL_SKIP_MARGIN_METERS`, `LANE_MAX_DISTANCE_METERS`, `BODY_MARGIN_METERS`, `LANE_MARGIN_METERS`, `PASSAGE_MIDDLE_MARGIN_METERS`, `AWAY_SEARCH_MARGIN_METERS`, `DISPLACEMENT_STEP_METERS`, `WIDEN_STEP_FRACTION`.
- Lanes: `MAX_FULL_LANE_METERS`, `LANE_CLAMP_MARGIN_METERS`, `LANE_SLOPE`, `SAME_LANE_METERS`, `SQUEEZED_GAP_METERS`, `FOLD_SHRINK_FACTOR`, `MIN_FORWARD_STEP_FRACTION`.
- Slots and swaps: `START_BLEND_METERS`, `END_BLEND_METERS`, `COLUMN_GAP_RADII`, `COLUMN_GAP_EXTRA_METERS`, `COLUMN_HEADWAY_SECONDS`, `MIN_FORMATION_REACH_METERS`, `COORDINATES_BEHIND_METERS`, `COORDINATES_AHEAD_METERS`, `SWAP_INTERVAL_SECONDS`, `SWAP_SIDE_WEIGHT`, `SWAP_MIN_SAVING_FRACTION`, `SWAP_MIN_SAVING_SQUARED`.
- Anchor speed: `LAG_SLOWDOWN_FRACTION`, `LAG_SLOWDOWN_METERS`, `REAR_MARGIN_METERS`, `COLUMN_LEAVE_AHEAD_METERS`, `END_DECELERATION_FRACTION`, `LAG_WAIT_METERS`, `LAG_WAIT_RANGE_METERS`, `CRAWL_FRACTION`, `LAG_STOP_METERS`, `ANCHOR_STILL_SPEED`, `STUCK_WAIT_SECONDS`, `IGNORE_LAG_SECONDS`.
- Member steering: `FORM_SPEED_TIME_CONSTANT_SECONDS`, `PROGRESS_BEHIND_METERS`, `PROGRESS_AHEAD_METERS`, `REJOIN_BEHIND_METERS`, `REJOIN_LOOK_METERS`, `REJOIN_STEP_METERS`, `NO_LANE_REJOIN_METERS`, `PURSUIT_SLOT_METERS`, `PURSUIT_MIN_METERS`, `PURSUIT_MAX_METERS`, `PURSUIT_LOOK_SECONDS`, `PURSUIT_STEP_METERS`, `PURSUIT_MIN_AHEAD_METERS`, `CATCH_UP_GAIN`, `AHEAD_SPEED_FLOOR_FRACTION`, `AHEAD_DECELERATION_FRACTION`, `SLOT_GAIN`, `ARRIVAL_BRAKING_FRACTION`, `ARRIVAL_SLACK_METERS`, `PARKING_SPEED`, `END_STOP_REACH_METERS`, `AVOIDANCE_MIN_CHANGE_SQUARED`, `WAIT_SPEED_FRACTION`, `ORCA_WAIT_LIMIT_SECONDS`, `AVOIDANCE_OFF_SECONDS`, `BEND_LOOK_MIN_METERS`, `BEND_LOOK_SECONDS`, `BEND_SLOT_FRACTION`, `BEND_MIN_DISTANCE_METERS`, `BEND_MIN_SINE`, `BEND_FLOOR_FRACTION`, `BRAKE_LOOK_SECONDS`, `BRAKE_LOOK_MARGIN_METERS`, `STEER_MIN_SPEED`, `STEER_MIN_DISTANCE_METERS`.
- ORCA avoidance and stepping aside: `ORCA_HORIZON_SECONDS`, `ORCA_MARGIN_METERS`, `MEMBER_AVOID_RANGE_METERS`, `SHARED_EFFORT`, `FULL_EFFORT`, `SPEED_TIME_CONSTANT_SECONDS`, `MIN_STEP_ASIDE_SPEED`.
- Stuck units and detours: `PROGRESS_CHECK_SECONDS`, `MIN_PROGRESS_METERS`, `STUCK_SLOT_METERS`, `DETOUR_AHEAD_METERS`, `DETOUR_TOLERANCE_METERS`, `DETOUR_LIMIT_SECONDS`.
- Giving up: `CAP_WAIT_SPEED`, `GIVE_UP_SECONDS`, `IGNORE_SECONDS`.
- Moving squads ahead: `STEP_SECONDS`, `MAX_PREDICTION_SECONDS`, `MIN_PREDICTED_SPEED`, `MIN_WAIT_SECONDS`, `MIN_YIELD_WAIT_SECONDS`, `MIN_GAIN_SECONDS`, `GAP_METERS`, `TIME_MARGIN_SECONDS`, `CORRIDOR_MARGIN_SECONDS`, `CORRIDOR_MARGIN_METERS`, `STAMP_STEP_FRACTION`, `SAME_WAY_COSINE`, `HEAD_ON_COSINE`, `DIRECTION_SPAN_METERS`, `DEVIATION_STEP_METERS`, `PARKING_LOOK_SECONDS`, `LATE_YIELD_SPEED_FRACTION`, `YIELD_HOLD_SECONDS`, `CLOSE_PASS_CELLS`, `CLOSE_PASS_SET_UP_CELLS`.
- Work limits: `SMOOTHING_ITERATIONS`, `WIDEN_ITERATIONS`.

`FULL_SEARCH_BUDGET` needs no tuning, because it equals the limit of the game on the expansions of one search.

The other constants of "Work limits and capacities" are not tuned either. They size tables and arrays, or they hold work costs fitted to measured times. The counts in "Counts that follow from distances" follow from the distances above.
