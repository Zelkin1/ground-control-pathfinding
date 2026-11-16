# Simulator parameters

This file lists the constants of the simulator that have a physical meaning: distances, times, speeds, angles, rates and the shares of such values. Each table gives the name of the constant in the code, its value, its unit, and what it sets. The heading of each section names the file that defines its constants, and a row names another file when its constant lives there.

The file has four parts:

- Base game: The values of Ground Control itself. The simulator copies them from the code of gc.exe and from the unit files of the game, and it does not change them.
- Our changes: The values of the behavior that the simulator adds to the game. This part covers formation following, the coordination between squads, and the few places where the simulator changes the movement of the base game.
- Testing: The values that serve only the tests. They do not change how units move.
- Tuning status: Which values of "Our changes" were tuned by tests of several values, and which were not tuned yet.

In the first three parts, the sections follow the order in which a move happens.

## Terms

- A route is the line that a squad follows to its destination. The path search of the game gives its corners.
- A control point is a point that shapes the smooth curve that replaces the route. The curve passes near the control points, not through them.
- A curve sample is a point read off the smooth curve. The room tests run at the curve samples, and the samples become the route that the squad drives.
- A room test checks the free ground around one curve sample. It measures the width of the passage with two rays across the curve, and it checks that a circle around the sample holds no blocked cell.
- A push moves the control points near a sample that lacks room away from the obstacle.
- The anchor is a virtual leader that drives along the squad route. Each member has a place, a point at a fixed offset from the anchor, and a slot, its final point at the end of the route.
- A lane is the line beside the route that one member of the squad drives. A lane sample holds the lanes at one point of the route.
- The reach of a unit is the distance at which it counts its current waypoint as reached.
- A parked squad is a squad that neither moves nor searches.
- A squad gives way when it slows down or waits so that another squad can go first.
- A head-on shift moves the lanes of two squads that meet sideways, so that they pass each other.
- A pass is a move of a faster squad beside a slower squad that drives the same way.
- A share of friction is a deceleration given as a part of the friction of the unit.
- A tick is one step of the game, 0.04 s long, in which every unit moves once.

## Base game

Every value in this part is a value of the game. The simulator uses it as it is in all modes. Where the formation and coord modes behave differently, they use the values of the part "Our changes".

### Map and time

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `GC_CELL_METERS` | 6.25 | m | Width of one square map cell, which is either free or blocked for each kind of unit (game/game.h). |
| `GC_GRID_SIZE` | 1024 | cells | Number of cells along each side of the square map, which makes the map 6400 m wide (game/game.h). |
| `TICK_SECONDS` | 0.04 | s | Length of one tick, the fixed time step in which the game moves every unit once (sim.h). |
| `WORLD_MIN_METERS` | 10 | m | Smallest x or y of a unit: Integrate (0x5109e0) clamps each move to stay 10 m inside the map (game/physics.c). |
| `WORLD_MAX_METERS` | 6390 | m | Largest x or y of a unit, 10 m inside the far edge of the 6400 m map, kept by the same step (game/physics.c). |

### Unit types (scenarios.c)

The values come from the unit files of the game (the .ENT files in Data0.sdf).

| Unit type | Radius (m) | Top speed (m/s) | Acceleration (m/s²) | Friction (m/s²) | Turn rate (rad/s) | Close range (m) | Units per squad |
|---|---|---|---|---|---|---|---|
| `MEDIUM_TANK` | 3.5 | 20 | 70 | 65 | 1.5 | 5 | 4 |
| `LIGHT_TANK` | 3.5 | 28 | 87 | 80 | 1.8 | 10 | 4 |
| `HEAVY_TANK` | 6.0 | 15 | 87 | 80 | 0.9 | 12 | 2 |
| `MARINE` | 1.0 | 8 | 110 | 100 | 3.2 | 5 | 8 |

Friction is the speed that a unit loses each second. The close range is the distance at which the steering of the game changes its behavior near a waypoint.

### Units (game/movement.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `STOP_REACH_METERS` | 7 | m | Reach of a unit that stands or must stop, so it counts its waypoint as reached within 7 m (sim.h). |
| `MID_REACH_METERS` | 1 | m | Reach of every waypoint before the last one of a move, so units pass close to each corner (sim.h). |
| `WAIT_TIMER_AT_ORDER_SECONDS` | -3 | s | Start value of the wait timer of a unit after a new order, so waits start only once it counts up (sim.h). |
| `WAYPOINT_TOLERANCE_METERS` | 1 | m | The path search of a new move may end this close to its waypoint instead of exactly on it. |
| `REACH_EPSILON_METERS` | 0.1 | m | A reach this much larger than the reach of the last waypoint skips the new search toward it. |
| `ARRIVAL_STOP_DISTANCE_METERS` | 3 | m | Braking before a waypoint aims to bring the unit to a stop this short of the waypoint. |
| `BRAKING_START_FRACTION` | 0.5 | share of friction | Braking starts where the unit could stop by slowing with this share of its friction. |
| `WAYPOINT_BEHIND_METERS` | -0.2 | m | A waypoint that lies below this distance along the heading of the unit counts as behind it. |
| `BEHIND_THROTTLE` | 0.5 | share of acceleration | Throttle that the steering uses while it turns the unit toward a waypoint behind it. |
| `MIN_THROTTLE` | 0.6 | share of acceleration | Smallest throttle that the steering uses toward a waypoint ahead of the unit. |
| `REORIENT_DONE_COSINE` | 0.95 | cosine | A turn on the spot toward a new direction ends once the unit faces within about 18 degrees of it. |
| `QUERY_RADIUS_FACTOR` | 1.1 | factor of radius | The search for units in the way widens the radius of the unit by this factor. |
| `QUERY_RADIUS_MARGIN_FACTOR` | 1.25 | factor of radius | The avoidance of the game also widens the radius of the unit by this factor before that search. |
| `LAST_LEG_TURN_COSINE` | 0.7 | cosine | On its last leg, a unit first turns to face a slot that lies more than about 45.6 degrees off its heading. |
| `SLOT_QUERY_SHORTFALL_METERS` | 1 | m | The check for units in the way of that turn toward the slot ends this short of the slot. |
| `LOOK_DISTANCE_FACTOR` | 0.6 | share of distance | The corridor that a unit checks for units in its way spans this share of the distance to its waypoint. |
| `MAX_LOOK_RADII` | 4 | unit radii | Longest corridor that a unit checks for units in its way, in radii of the unit. |
| `MIN_LOOK_RADII` | 1.5 | unit radii | Shortest corridor that a unit checks, in radii of the unit, also when the corridor is halved. |
| `BEHIND_LOOK_FACTOR` | 0.5 | factor | The corridor that a unit checks for units in its way halves when the unit faces away from its waypoint. |
| `MAX_BLOCKER_TANGENT_SQUARED` | 100000 | m² | Squared distance beyond which no corridor reaches a unit in the way, about 316 m. |
| `NEW_BLOCKER_RADII` | 2.4 | unit radii | A new unit in the way at least this far away makes the unit pick a new swerve side. |
| `AVOID_TICKS` | 5 | ticks | Time that a swerve or a wall slide marks a unit as avoiding, which is 0.2 s (sim.h). |
| `DRIVING_ON_AVOID_TICKS` | 2 | ticks | A unit in the way that has avoided for at most this long still counts as driving on. |
| `WAIT_SECONDS` | 2 | s | Time that a unit waits behind a faster unit in its way before it swerves around it. |
| `SWERVE_AFTER_WAIT_SECONDS` | -2 | s | A unit does not wait while its wait timer is at this value or more, so it swerves instead. |

### Collisions and walls (game/physics.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `PUSH_GAP_METERS` | 0.3 | m | Gap that a collision leaves between two bodies after it pushes them apart, so they no longer overlap. |
| `MIN_PUSHED_AWAY_SPEED` | -0.1 | m/s | Two units that move apart faster than this get no extra push speed from a collision. |
| `SLOW_PUSH_SPEED` | 0.1 | m/s | A pushed unit slower than this gets the speed `MIN_PUSH_SPEED` away from the other body. |
| `MIN_PUSH_SPEED` | 0.2 | m/s | Speed away from the other body that a collision gives a pushed unit slower than `SLOW_PUSH_SPEED`. |
| `WALL_PUSH_CELLS` | 0.55 | cells | A slide along a wall puts the unit this far from the center of the blocked cell, about 3.4 m. |
| `SIDE_COOLDOWN_SECONDS` | 2 | s | Wall slides flip the preferred swerve side of a unit at most once in this time. |
| `TELEPORT_STEP_CELLS` | 0.5 | cells | Step of the search for a free cell when a unit ends up inside a blocked cell, about 3.1 m. |

### Squads (game/squad.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `SQUAD_SEARCH_BUDGET` | 60 | expansions per tick | The path search of a squad runs this many search steps in each tick, so a long search takes several ticks. |
| `SQUAD_SEARCH_TOLERANCE_METERS` | 0 | m | The path search of a squad ends only exactly at its target, with no tolerance. |
| `MIN_LAST_REACH_METERS` | 1 | m | Smallest reach of the last waypoint of a member, which is otherwise the radius of the member. |
| `STRAGGLER_WAIT_SECONDS` | 20 | s | A squad counts as arrived this long after all its members but one stopped, even if that one still moves. |

## Our changes

Every value in this part belongs to the simulator. These values set formation following (`--avoid form`), the coordination between squads (`--avoid coord`), and the switches that turn each of their parts on or off.

### Changes to the base game movement

These values change the movement of the base game in a few places. Two of them fix faults of the game, and two change its behavior. The column "Kind" tells which.

| Constant | Value | Unit | Kind | What it sets |
|---|---|---|---|---|
| `SIGHT_TOUCH_MARGIN_METERS` | 0.5 | m | Change, formation modes only | A line of sight fails when it brings the unit this close to a unit of a parked squad (game/movement.c). |
| `TELEPORT_MAX_CELLS` | 20 | cells | Fix, all modes | The slope rescue gives up at 125 m and leaves the unit in place, where the base game searches on (game/physics.c). |
| `HIT_REACH_CELLS` | 1 | cells | Fix, formation modes only | A wall slide ignores a hit cell further than this from its ray, so units no longer jump far (game/physics.c). |
| `FULL_SEARCH_BUDGET` | 500 | expansions per tick | Change, formation modes only | With `--fullsearch on`, a squad search gets all its steps in one tick, where the base game gives 60 (game/squad.c). |

The two fixes replace these behaviors of the base game:

- Slope rescue: When a unit stands on a cell that blocks it, the base game tries +x, -x, +y and -y at 0.5, 1.0, 1.5 cells and so on. The search has no other end, so it never ends when no legal cell lies on those four lines.
- Wall slide: For a tiny move along a cell edge, the ray of the base game can return a blocked cell hundreds of meters away. The wall slide then teleports the unit there in one tick.

The search change makes squads start sooner after an order. The base game ends every search after 500 steps, so with `FULL_SEARCH_BUDGET` the search of a squad always ends in the tick where it starts. When the first search fails, the backup search of the game then runs in the same tick too. In the base game, a long search takes up to 3 ticks in the scenarios.

The sight change can change gameplay, because the game can also use a line of sight to decide whether a unit fires. sim/GAMEPLAY_NOTES.md says what to check in the game before the change goes in.

One more change has no constant. In the formation modes, collisions never push the units of parked squads (game/physics.c), and in the base game they do. This change is different from the base game, and it is part of the collisions, not of the route planning.

### Parked squads (game/squad.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `PARKED_STAMP_MARGIN_METERS` | 1 | m | Cells this close to the bodies of parked units count as cliffs in the searches of a moving squad. |
| `PARKED_ROUTE_STEP_METERS` | 2 | m | When a squad parks, the rest of the route of each moving squad is checked at this step for it. |
| `PARKED_ROUTE_MARGIN_METERS` | 2 | m | A route counts as passing a parked unit when it comes within this gap of the body of the unit. |
| `TIGHT_CIRCLE_RADII` | 2 | unit radii | A unit circles tightly, and can count as settled, when it stays within this distance. |
| `PARKED_SPEED_FRACTION` | 0.1 | share of top speed | With `--parkspeed on`, units slower than this share also count as parked for new searches. |
| `PARK_AHEAD_SECONDS` | 16 | s | With `--parkahead on`, a moving squad looks this long at its anchor speed ahead on its route for the slots of a squad that parks there first. |

With `--oldroute on`, a squad whose new search around parked squads fails, or whose slots no longer fit, keeps its old route and slots. In the base game, the move is then denied. This change has no constant.

With `--slotpoint on`, the check of `PARK_AHEAD_SECONDS` measures the time of the moving squad until its route comes closest to the slots of the other squad, not until its route first comes within reach of them. The time of the other squad also ends at its slots, so both times end at the same place. This change has no constant.

### Route set-up and smoothing (formation/route.c)

#### Route set-up

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `CORNER_CUT_MARGIN_METERS` | 8 | m | A corner is cut only when a band this much wider on each side than the squad is free. |
| `CENTER_ROUTE_MIN_GAIN_METERS` | 10 | m | The route from the center of the squad replaces the route from its lead only when it is this much shorter. |
| `REPLAN_MAX_STRETCH` | 1.25 | factor of length | A replanned route keeps its corners when the smoothing would make it longer by more than this factor. |
| `FORMATION_DIRECTION_SPAN_METERS` | 10 | m | The direction of the formation at a point is that of the route from this far before it to this far after it. |
| `MIN_CHORD_LENGTH_SQUARED` | 1 | m² | That span gives a direction only above this squared length, and else the direction of the segment counts. |
| `ROUTE_END_EPSILON_METERS` | 0.01 | m | An anchor this close to the end of the route counts as at the end, where the members stop (sim.h). |

#### Smooth curve

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `CONTROL_SPACING_METERS` | 16 | m | Largest distance between two control points along the route, which sets how finely the curve can bend. |
| `CURVE_SAMPLE_SPACING_METERS` | 4 | m | Distance between two curve samples, where the room tests run and which become the route points. |
| `MAX_FULL_ROUTE_METERS` | 8000 | m | Longest route that keeps both spacings above, and longer routes scale them up in proportion to length. |
| `FREE_DISTANCE_STEP_METERS` | 0.5 | m | Step of the rays that measure free distances, in the room tests and in the lane layout. |
| `CELL_SKIP_MARGIN_METERS` | 0.01 | m | A ray skips only the probes that lie at least this far inside a free cell, so the skip never changes a result. |
| `LANE_MAX_DISTANCE_METERS` | 30 | m | Longest free distance that a ray measures, and ground further away counts as free (formation/formation.h). |
| `BODY_MARGIN_METERS` | 0.5 | m | The body room at a sample is the radius of the widest unit plus this, and every curve sample needs it. |
| `LANE_MARGIN_METERS` | 1.5 | m | The formation room is half the formation width plus the widest radius plus this (formation/formation.h). |
| `PASSAGE_MIDDLE_MARGIN_METERS` | 0.25 | m | In a narrow passage, the curve wants half the passage width less this, so that it runs down the middle. |
| `AWAY_SEARCH_MARGIN_METERS` | 1 | m | A push looks for the closest obstacle up to this far past the wanted room of the sample. |
| `PUSH_STEP_METERS` | 0.25 | m | Distance that one push moves the control point closest to the sample away from the obstacle. |
| `PUSH_SPREAD_METERS` | 64 | m | Pushes reach this far each way, 4 control points, so ahead they leave out the last point of the 80 m fade. |
| `PUSH_TAPER_METERS` | 80 | m | Distance from the sample over which a push fades from full strength to zero, as a cosine bell. |

#### Wide bends

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `BEND_SPEED_MARGIN` | 0.8 | factor | Full speed holds in bends whose radius is at least top speed / (this × turn rate) (formation/formation.h). |
| `WIDEN_STEP_FRACTION` | 0.5 | share of excess | Share of the curvature above the wanted limit that one widening step takes away at a bend. |
| `BEND_STENCIL_METERS` | 8 | m | The widening measures the sharpness of a bend from the curve this far before and after each sample. |

### Lanes (formation/lanes.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `LANE_STEP_METERS` | 2 | m | Distance between two lane samples along the route, where the lanes and their free room are laid out. |
| `MAX_FULL_LANE_METERS` | 8000 | m | Longest route that keeps `LANE_STEP_METERS`, and longer routes get a lane step that grows with their length. |
| `LANE_CLAMP_MARGIN_METERS` | 0.5 | m | Lanes are clamped so that the body of the widest unit stays at least this far from blocked ground. |
| `LANE_SLOPE` | 0.3 | m per m | The edges of the free range of the lanes move sideways by at most this much per meter of route. |
| `SAME_COLUMN_METERS` | 0.5 | m | Slots that are closer than this sideways share one lane, as one column of the formation. |
| `SQUEEZED_GAP_METERS` | 3 | m | Lanes squeezed in a narrow place keep two radii of the widest unit plus this gap apart. |
| `ROOM_WINDOW_METERS` | 4 | m | At each lane sample, the lanes keep clear of the narrowest free distance within this far before and after. |
| `AVERAGE_WINDOW_METERS` | 6 | m | The lanes are averaged over this far before and after each sample, but at most over `FILE_WINDOW_METERS`. |
| `FILE_WINDOW_METERS` | 8 | m | Single file starts this far before a place too narrow for the squad side by side, and ends this far after it. |
| `BELL_REACH_METERS` | 40 | m | A sideways shift of the lanes fades in and out over this distance on each side, as a smooth bump. |
| `FOLD_SHRINK_FACTOR` | 0.8 | factor | Each pull at a fold, where an inner lane runs backward in a sharp corner, scales the lane offsets by this. |
| `MIN_FORWARD_STEP_FRACTION` | 0.25 | share of lane step | The outer lanes must move forward by at least this share of the lane step from one sample to the next. |

### Places and anchor (formation/formation.c)

#### Places and matching

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `START_BLEND_METERS` | 25 | m | The places blend from where the members stand into the formation over this distance from the route start. |
| `END_BLEND_METERS` | 25 | m | Places blend into the shape of the slots over this last distance of the route (formation/formation.h). |
| `FILE_SPACING_RADII` | 2.2 | unit radii | Gap between members in single file, in radii of the widest member, before `FILE_SPACING_GAP_METERS` is added. |
| `FILE_SPACING_GAP_METERS` | 2 | m | Extra distance added to that gap in single file, so medium tanks keep 2.2 × 3.5 + 2 = 9.7 m. |
| `FILE_HEADWAY_SECONDS` | 0.3 | s | With lanes, the gap in single file also grows by this time at the speed of the anchor. |
| `MIN_FORMATION_REACH_METERS` | 2 | m | Smallest reach of a member in formation mode, which is otherwise its radius. |
| `COORDINATES_BEHIND_METERS` | 80 | m | The position of a member along the route is sought up to this far before the anchor. |
| `COORDINATES_AHEAD_METERS` | 40 | m | The position of a member along the route is sought up to this far past the anchor. |
| `REASSIGN_INTERVAL_SECONDS` | 0.5 | s | Members are matched to places again this often, so that they swap places when that is shorter. |
| `REASSIGN_SIDE_WEIGHT` | 4 | factor | A sideways difference between a member and a place counts this many times in the cost of a match. |
| `REASSIGN_MIN_SAVING_FRACTION` | 0.1 | share of cost | A new match replaces the current one only when it saves at least this share of the cost. |
| `REASSIGN_MIN_SAVING_SQUARED` | 10 | m² | A new match must also save at least this much cost, as a sum of squared distances. |

#### Anchor speed

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `LAG_SLOWDOWN_FRACTION` | 0.1 | share of speed | Members that lag behind their places slow the anchor by up to this share of its speed. |
| `LAG_SLOWDOWN_METERS` | 4 | m | Lag behind the place at which that slowdown reaches its full share, after growing in step with the lag. |
| `TURN_SPEED_FLOOR` | 0.55 | share of speed | With `--turnslow on`, the anchor keeps this share of its speed in turns of 90 degrees or more. |
| `TURN_SPEED_RANGE` | 0.45 | share of speed | Gentler turns add this share times the cosine of the turn to that floor, up to full speed when straight. |
| `REAR_MARGIN_METERS` | 10 | m | Checks along the route for turns and single file reach this far past the front and the rear of the formation. |
| `TURN_LOOK_AHEAD_METERS` | 25 | m | A turn is measured from the route direction at the rear of the file to the one this far past the anchor. |
| `FILE_LEAVE_AHEAD_METERS` | 20 | m | Single file ends once the squad fits side by side up to this far past its front. |
| `END_DECELERATION_FRACTION` | 0.5 | share of friction | The anchor brakes before the route end with this share of the lowest friction of the members. |
| `LAG_WAIT_METERS` | 8 | m | When a member lags more than this, the anchor slows further to wait for it. |
| `LAG_WAIT_RANGE_METERS` | 20 | m | That waiting slowdown grows by one full share for each this many meters of lag. |
| `CRAWL_FRACTION` | 0.25 | share of speed | A waiting anchor keeps at least this share of its speed, so the squad keeps crawling forward. |
| `LAG_STOP_METERS` | 60 | m | The anchor stops completely only when a member lags more than this, and below it keeps crawling. |
| `ANCHOR_STILL_SPEED` | 0.2 | m/s | Below this speed, the anchor counts as held still, which starts the timer of `STUCK_WAIT_SECONDS`. |
| `STUCK_WAIT_SECONDS` | 10 | s | After this long held still, the anchor moves on, so that one stuck member cannot hold the squad forever. |
| `IGNORE_LAG_SECONDS` | 5 | s | After that wait, the anchor ignores members that lag behind for this long, and then waits for them again. |

### Member steering (formation/steering.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `FORM_SPEED_TIME_CONSTANT_SECONDS` | 0.1 | s | The throttle of a member closes a gap to its wanted speed in about this time. |
| `PROGRESS_BEHIND_METERS` | 10 | m | The search for the position of a member along the route starts this far before its last position. |
| `PROGRESS_AHEAD_METERS` | 30 | m | That search ends this far past the last position of the member, so the position never jumps further. |
| `REJOIN_BEHIND_METERS` | 12 | m | A member this far behind its place drives back to its lane instead of to its place. |
| `REJOIN_LOOK_METERS` | 10 | m | The point where a member rejoins its lane lies at most this far past its position along the route. |
| `REJOIN_STEP_METERS` | 2.5 | m | Rejoin points along the lane are tried this far apart, up to `REJOIN_LOOK_METERS` past the member. |
| `NO_LANE_REJOIN_METERS` | 60 | m | With `--lanes off`, rejoin points lie up to this far past the position of the member. |
| `PURSUIT_PLACE_METERS` | 4 | m | A member further than this from its place pursues a point ahead on its lane. |
| `PURSUIT_MIN_METERS` | 6 | m | The pursued point lies at least this far past the position of the member along the route. |
| `PURSUIT_MAX_METERS` | 16 | m | The pursued point lies at most this far past that position, also for a member at top speed. |
| `PURSUIT_LOOK_SECONDS` | 0.8 | s | The pursued point lies this long ahead at the speed of the member, within the two limits above. |
| `PURSUIT_STEP_METERS` | 2 | m | Points for pursuit are tried this far apart along the lane, from the furthest one back toward the member. |
| `PURSUIT_MIN_AHEAD_METERS` | 2 | m | Pursuit tries points down to this distance past the position of the member. |
| `CATCH_UP_GAIN` | 0.8 | (m/s) per m | A member behind its place drives this much faster than the anchor for each meter behind. |
| `AHEAD_SPEED_FLOOR_FRACTION` | 0.5 | share of anchor speed | A member ahead of its place keeps at least this share of the speed of the anchor. |
| `AHEAD_DECELERATION_FRACTION` | 0.2 | share of friction | A member ahead of its place slows with this share of its friction, so it falls back gently. |
| `FILE_AHEAD_SPEED_FLOOR_FRACTION` | 0.2 | share of anchor speed | In single file, a member ahead of its place keeps this smaller share instead. |
| `FILE_AHEAD_DECELERATION_FRACTION` | 0.5 | share of friction | In single file, a member ahead of its place slows with this larger share instead. |
| `PLACE_GAIN` | 0.8 | (m/s) per m | A member near its place moves toward it at this speed for each meter away, on top of the anchor motion. |
| `ARRIVAL_BRAKING_FRACTION` | 0.7 | share of friction | Plans to stop at a target assume braking with this share of the friction, which leaves a margin. |
| `ARRIVAL_SLACK_METERS` | 0.5 | m | Plans to stop at a target aim this far short of it, so that a member does not overshoot its place. |
| `SLOT_SPEED` | 2 | m/s | A member on its slot at the route end that drives slower than this stops there and turns to its facing. |
| `END_STOP_REACH_METERS` | 1 | m | With `--endstop on`, a member that drives straight at its slot counts as on it within this distance. |
| `END_BRAKING_FRACTION` | 0.3 | share of friction | The last leg brakes onto the slot with this share of the friction, for a soft stop. |
| `AVOIDANCE_MIN_CHANGE_SQUARED` | 0.25 | (m/s)² | Changes of the velocity by the avoidance below 0.5 m/s are ignored, and the member keeps its wanted velocity. |
| `HELD_SPEED_FRACTION` | 0.25 | share of top speed | A member that the avoidance keeps below this share of its top speed, against its wish, counts as held still. |
| `HELD_LIMIT_SECONDS` | 3 | s | A member that the avoidance holds still for longer than this pushes on, without avoidance for a while. |
| `AVOIDANCE_OFF_SECONDS` | 2 | s | A member that pushed on through the avoidance then drives without avoidance for this long. |
| `BEND_LOOK_MIN_METERS` | 8 | m | Checks for bends ahead look at least this far past the position of the member. |
| `BEND_LOOK_SECONDS` | 0.8 | s | With `--bendlook off`, checks for bends look this long ahead at the speed of the member. |
| `FAR_BEND_LOOK_SECONDS` | 1.2 | s | With `--bendlook on`, the checks for bends ahead look this long ahead instead. |
| `BEND_PLACE_FRACTION` | 0.5 | share of distance | The checks for bends also look this share as far past the place of the member. |
| `BEND_MIN_DISTANCE_METERS` | 1 | m | The bend check gives no speed limit when its lane point lies within this distance of the member. |
| `BEND_MIN_SINE` | 0.05 | sine | Points of a bend almost straight ahead, within about 3 degrees, give no speed limit. |
| `BEND_FLOOR_FRACTION` | 0.2 | share of top speed | The speed limit of a bend never falls below this share of the top speed, so units keep moving around it. |
| `HEADING_TIME_CONSTANT_SECONDS` | 0.1 | s | The steering closes a gap to the wanted heading in about this time, whatever the turn rate of the unit. |
| `BRAKE_LOOK_SECONDS` | 0.4 | s | The check for blocked ground ahead looks this long ahead along the heading. |
| `BRAKE_LOOK_MARGIN_METERS` | 1 | m | That check looks at least the radius of the member plus this far ahead, also when the member is slow. |
| `STEER_MIN_SPEED` | 0.5 | m/s | Below this speed, a member steers straight toward its target instead of along its velocity. |
| `STEER_MIN_DISTANCE_METERS` | 0.5 | m | A target this close gives no steering when the member is also slower than `STEER_MIN_SPEED`. |
| `WAIT_AHEAD_MIN_COSINE` | 0.87 | cosine | A member past its place that faces within 30 degrees of forward waits instead of turning (`--waitahead`). |

### Avoidance and making room (formation/avoidance.c)

The avoidance between units uses ORCA, a method that picks for each unit the velocity closest to the wanted one that stays clear of the other units for some time.

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `ORCA_HORIZON_SECONDS` | 2 | s | Units must stay clear of each other for this long at their new velocities, so they react this early. |
| `ORCA_MARGIN_METERS` | 0.5 | m | Extra gap that the avoidance keeps between the bodies of two units, on top of their radii. |
| `MATE_AVOID_RANGE_METERS` | 2 | m | Members of the same squad avoid each other only at body gaps up to this plus `ORCA_MARGIN_METERS`. |
| `SHARED_EFFORT` | 0.5 | share of change | Each of two moving units makes this share of the change of velocity that keeps them apart. |
| `FULL_EFFORT` | 1 | share of change | A unit makes the whole change when the other unit cannot move aside, for example a turning one. |
| `SPEED_TIME_CONSTANT_SECONDS` | 0.3 | s | A parked unit that makes room, or returns to its spot, reaches its new speed in about this time. |
| `MIN_YIELD_SPEED` | 1 | m/s | A parked unit makes room only when the avoidance asks it to move faster than this. |
| `YIELD_SPEED_FRACTION` | 0.3 | share of top speed | Parked units that make room move aside at most at this share of their top speed. |
| `YIELD_MAX_METERS` | 3.5 | m | Parked units that make room move only to points this close to their spot, and then return. |
| `HOME_REACHED_METERS` | 1 | m | A parked unit that made room counts as back within this distance of its spot, and stops there. |

### Stuck units and detours (formation/replan.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `PROGRESS_CHECK_SECONDS` | 2 | s | The progress check compares the positions of a unit this far apart in time. |
| `MIN_PROGRESS_METERS` | 1 | m | A unit that moved less than this in that time made no progress and counts as stuck. |
| `STUCK_PLACE_METERS` | 6 | m | A unit this close to its place never counts as stuck, even when it hardly moves. |
| `DETOUR_AHEAD_METERS` | 20 | m | A detour for a stuck unit ends on its lane at most this far along the route past its progress. |
| `DETOUR_TOLERANCE_METERS` | 2 | m | The path search of a detour may end this close to the end point of the detour instead of on it. |
| `DETOUR_LIMIT_SECONDS` | 20 | s | A unit drops a detour that takes longer than this, and goes back to following its place. |

### Coordination between squads (formation/coordination.c)

#### Crossings and giving way

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `CROSSING_SAMPLE_METERS` | 6 | m | Step of the route points that coordination reads to find crossings, meetings and the room beside the route. |
| `CROSSING_HORIZON_METERS` | 250 | m | Coordination looks for crossings up to this far past the anchor of each squad. |
| `MIN_CROSSING_SINE` | 0.5 | sine | Routes that cross at less than 30 degrees do not count as a crossing, because their zones would grow too long. |
| `CROSSING_MARGIN_METERS` | 2 | m | Each crossing zone reaches this much further along each route, each way, as a safety margin. |
| `CROSSING_TIME_MARGIN_SECONDS` | 0.5 | s | A squad that gives way enters the crossing zone only this long after the other squad left it. |
| `LATE_YIELD_SPEED_FRACTION` | 0.5 | share of cruise speed | With `--lateyield on`, a squad that gives way late slows to this share of its cruise speed. |
| `MIN_PREDICTED_SPEED` | 0.5 | m/s | Estimates of when a squad reaches or leaves a zone use at least this speed, also for a squad that stands. |
| `HELD_SPEED` | 0.2 | m/s | A speed limit below this holds the anchor nearly still, which starts the timer of `GIVE_UP_SECONDS`. |
| `GIVE_UP_SECONDS` | 15 | s | After this long held, a squad stops giving way, so that two squads cannot block each other forever. |
| `IGNORE_SECONDS` | 5 | s | After it gives up, the squad ignores the other squads for this long, and then coordinates again. |
| `SAME_LINE_METERS` | 2 | m | Routes that run closer than this sideways keep to the right of each other (formation/formation.h). |

#### Head-on meetings

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `HEAD_ON_MAX_COSINE` | -0.7 | cosine | Routes that run within about 45 degrees of opposite directions count as head-on. |
| `HEAD_ON_MARGIN_METERS` | 4 | m | Gap that the bodies of two formations keep when they pass each other head-on (formation/formation.h). |
| `HEAD_ON_EXTRA_GAP_METERS` | 4 | m | With `--widegap on`, the head-on gap between the bodies grows by this much, to 8 m in all. |
| `MEETING_REACH_METERS` | 40 | m | The room to pass each other is measured this far before and after the meeting point. |
| `WAIT_ROOM_METERS` | 24 | m | A squad that gives way waits where it has room to the side over this length of route behind the spot. |

#### Lane shifts (formation/shift.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `SIDE_SHIFT_SLOPE` | 0.3 | m per m | The lanes shift sideways by at most this much per meter that the squad drives. |
| `SHIFT_REACH_METERS` | 80 | m | The full shift of a head-on meeting reaches this far before and after the meeting point. |
| `SHIFT_AFTER_METERS` | 10 | m | With `--shortreturn on`, the full shift ends this far past the meeting point. |
| `SHIFT_BEFORE_METERS` | 20 | m | With `--lateshift on`, the full shift starts this far before the meeting point. |
| `SWAY_MIN_SPEED_FRACTION` | 0.25 | share of top speed | A shift moves as if the squad drove at least this share of its top speed, also when it is slower. |
| `SWAY_SLOPE` | 0.15 | m per m | With `--smoothsway on`, a shift moves by at most this much per meter driven. |

### Passing and following (formation/overtaking.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `SAME_WAY_MIN_COSINE` | 0.85 | cosine | Routes within 32 degrees of parallel run the same way, but `--keepfollow` lets a following squad skip it. |
| `CROSS_SIDE_FRACTION` | 0.5 | share of side gap | With `--crossfollow on`, a squad follows instead of passing when its route goes from this share of the needed side gap on one side of the route ahead to as much on the other side. |
| `PASS_MIN_SPEED_GAIN` | 1 | m/s | A squad passes only a squad that is at least this much slower, and otherwise it follows it. |
| `PASS_LOOK_SECONDS` | 6 | s | A pass is planned once the follower would catch up with the leader within this time. |
| `PASS_BACK_METERS` | 120 | m | The checks for a pass read the routes from this far behind the rear of the squad. |
| `PASS_HORIZON_METERS` | 450 | m | The checks for a pass read the routes up to this far past the anchor of each squad. |
| `PASS_END_MARGIN_METERS` | 20 | m | The full shift of a pass reaches this much further than the squad that is passed. |
| `PASS_GAP_METERS` | 5 | m | A pass starts and ends with this gap between the bodies of the follower and the squad it passes. |
| `PASS_ROOM_STEP_METERS` | 2 | m | The room for a pass is measured at this step along the route, as for the lanes. |
| `PASS_SQUEEZE_METERS` | 3 | m | Bodies of passing squads may come this much closer than the pass gap in narrow places. |
| `FORMED_LANE_METERS` | 2 | m | A squad that is passed counts as formed when its members are this close to their lanes. |
| `FORMED_FRACTION` | 0.9 | share of shift | A squad that is passed counts as formed when its shift has moved this share of the way. |
| `PASS_ROOM_MARGIN_METERS` | 4 | m | The shifts of a pass stay this far inside the free room of the lanes, away from their clamps. |
| `FILE_PASS_ROOM_MARGIN_METERS` | 1 | m | Past a squad in single file, the shifts of a pass stay this far inside instead. |
| `FOLLOW_HEADWAY_SECONDS` | 1 | s | A squad that follows keeps this time gap, at its speed, behind the squad ahead. |
| `FOLLOW_MIN_GAP_METERS` | 10 | m | The gap of a squad that follows is at least this long, also when the squad ahead drives slowly. |
| `FOLLOW_GAIN` | 0.5 | (m/s) per m | The speed limit of a squad that follows changes by this much for each meter of gap error. |

### Work limits and capacities

These constants have no physical meaning. They limit the work of route planning, or they size its arrays. The four `SET_UP_*` constants apply only with `--setupslice on`. That switch is off by default, so each route set-up runs in full in one tick. With `--setupthreads on` (the default), the smoothings of the squads that set up in the same tick run at the same time on worker threads, one for each processor core but one.

| Constant | Value | File | What it sets |
|---|---|---|---|
| `SMOOTHING_ROUNDS` | 60 | formation/route.c | Largest number of push rounds over the whole curve before the curve is judged. |
| `WIDEN_ROUNDS` | 40 | formation/route.c | Largest number of rounds that widen sharp bends, before push rounds follow, up to 100 rounds in all. |
| `WIDEN_PATIENCE_PASSES` | 30 | formation/route.c | With `--widenstop on`, the widening stops this many rounds after its last better curve. |
| `SET_UP_WORK_PER_TICK` | 4000 | game/squad.c | With `--setupchain on`, the work in room tests that all route set-ups of one tick may share. |
| `SET_UP_WORK_PER_ROUTE_POINT` | 60 | game/squad.c | Work in room tests charged for the first part of a set-up, per route point. |
| `SET_UP_WORK_PER_LANE_SAMPLE` | 2 | game/squad.c | Work in room tests charged for the last part of a set-up, per lane sample, fitted to measured times. |
| `SET_UP_ROOM_TESTS_PER_TICK` | 2000 | game/squad.c | With `--setupchain off`, the room tests of one part of a smoothing in one tick. |
| `MAX_JOBS` | 32 | workers.c | The job list holds this many smoothings, more than a tick hands over, as a run has at most 8 squads. |
| `ROOM_MEMO_START_BITS` | 14 | formation/route.c | The table that remembers room tests starts with 2^14 (16,384) entries, enough for most smoothings. |
| `ROOM_MEMO_MAX_LOAD` | 0.5 | formation/route.c | The table doubles before it would be more than half full, so it holds the samples of any smoothing. |
| `COS_CACHE_BITS` | 12 | formation/route.c | The table that remembers the cosines of the push fades has 2^12 (4,096) entries. |
| `MAX_WAYPOINTS` | 4096 | sim.h | Points that a route, a path or a waypoint list can hold, so a smoothed route of 8 km at 4 m fits. |
| `LANE_MAX_SAMPLES` | 4096 | formation/lanes.c | Lane samples that each squad can hold, which covers 8 km of route at 2 m, and 2 more. |
| `ROUTE_SAMPLE_COUNT` | 128 | formation/formation.h | Route points that one coordination check can hold, 768 m at 6 m, while the longest check reads 600 m. |

### Counts that follow from distances

The code computes each count of points from a distance and the spacing of the points that it counts, once per route. A count is the distance divided by the spacing, rounded to the nearest whole number, and it is at least 1. So the reach in meters stays the same when a spacing changes. On a route longer than 8 km, the spacings grow, and the counts get smaller to match.

| Distance | Spacing | Count at the values above |
|---|---|---|
| `BEND_STENCIL_METERS` | `CURVE_SAMPLE_SPACING_METERS` | 2 curve samples |
| `PUSH_SPREAD_METERS` | `CONTROL_SPACING_METERS` | 4 control points |
| `PUSH_TAPER_METERS` | `CONTROL_SPACING_METERS` | 5 control points |
| `ROOM_WINDOW_METERS` | `LANE_STEP_METERS` | 2 lane samples |
| `AVERAGE_WINDOW_METERS` | `LANE_STEP_METERS` | 3 lane samples |
| `FILE_WINDOW_METERS` | `LANE_STEP_METERS` | 4 lane samples |
| `BELL_REACH_METERS` | `LANE_STEP_METERS` | 20 lane samples |
| `WAIT_ROOM_METERS` | `CROSSING_SAMPLE_METERS` | 4 route points |

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
| `START_CLEARANCE_METERS` | 0.5 | m | Units that `--seed` moves keep at least this gap between their bodies at the start. |
| `SQUAD_JITTER_SPANS` | 1.5 | squad spans | `--seed` moves the start of each squad by up to this many times the size of the squad. |
| `UNIT_JITTER_METERS` | 1.5 | m | After the squad, `--seed` moves each unit by up to this much on each axis, onto free ground. |
| `UNIT_JITTER_RADIANS` | 0.05 | rad | `--seed` also turns each unit by up to this angle, about 3 degrees, either way. |
| `SIDE_BY_SIDE_METERS` | 50 | m | In `ridge`, the medium and the heavy squad start and drive side by side this far apart. |
| `FOLLOW_GAP_METERS` | 45 | m | In `corridor`, the three squads start this far apart, one behind the other on the same route. |
| `SPREAD_DESTINATION_METERS` | 40 | m | In `corridor`, the destinations of the rear squads lie this far to the side. |
| `WINDING_GAP_METERS` | 60 | m | In `winding`, the squads start this far apart, and their destinations lie this far apart as well. |
| `LINE_GAP_METERS` | 40 | m | In `lineplain`, the squads stand this far apart in each line, the old one and the new one. |
| `LINE_ADVANCE_METERS` | 250 | m | In `lineplain`, the middle of the new line lies this far ahead of the old one. |

### Metrics and trace (metrics.c)

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `FAST_SPEED_FRACTION` | 0.5 | share of top speed | Above this speed a unit counts as driving, and a stop starts when a driving unit gets slow. |
| `SLOW_SPEED_FRACTION` | 0.15 | share of top speed | Below this share of its top speed a unit counts as slow, for the stops and the slow time. |
| `TOP_SPEED_FRACTION` | 0.95 | share of top speed | At this speed or more a unit counts as at top speed, for the time at top speed. |
| `STUCK_AFTER_SECONDS` | 3 | s | A unit that stays slow for longer than this counts as stuck, for the stuck time of the results. |
| `ARRIVING_RADII` | 3 | unit radii | Within this distance of its slot a unit counts as arriving, and its stops and slow time no longer count. |
| `SPIN_RADIANS_PER_SECOND` | 0.4 | rad/s | A turn faster than this at low speed counts as a turn on the spot, for the spin time of the results. |
| `SPIN_SPEED_FRACTION` | 0.2 | share of top speed | For a turn on the spot, the unit must also be slower than this share of its top speed. |
| `OBSTACLE_DEPTH_METERS` | 0.5 | m | A body that reaches deeper than this into blocked cells counts as inside an obstacle. |
| `TOUCH_MARGIN_METERS` | 0.35 | m | Bodies this close count as touching, a bit more than the 0.3 m gap that collisions leave between them. |
| `LOST_RADII` | 3 | unit radii | A unit that ends further than this from its slot counts as lost, for the lost count of the results. |
| `TRACE_GRID_MARGIN_METERS` | 60 | m | The map in the trace reaches this far past the area that the units of the run use. |
| `TRACE_LANE_STEP_METERS` | 4 | m | Step of the lane points that the trace stores, from which the replay draws the lanes. |

### Route finder (find.c)

These constants serve `sim.exe --find`, which measures random routes on a map to help pick scenarios.

| Constant | Value | Unit | What it sets |
|---|---|---|---|
| `MAP_SIZE_METERS` | 6400 | m | Side of the square map, `GC_GRID_SIZE` cells of 6.25 m each, which holds the random route ends. |
| `WORLD_MIN_METERS` | 10 | m | Smallest x or y of a route end, the same bound that Integrate keeps units in (see "Map and time"). |
| `WORLD_MAX_METERS` | 6390 | m | Largest x or y of a route end, with the same source as `WORLD_MIN_METERS` above. |
| `MIN_SPOT_CLEARANCE_METERS` | 25 | m | A random route end must lie at least this far from any blocked cell to be used. |
| `MIN_ROUTE_METERS` | 250 | m | The two ends of a random route lie at least this far apart in a straight line. |
| `ROUTE_RANGE_METERS` | 450 | m | A random distance of up to this is added to that, so routes run 250 to 700 m in a straight line. |
| `TURN_DEGREES` | 30 | degrees | A bend of the path that the search of the game finds counts as a turn when it is sharper than this. |
| `SAMPLE_STEP_METERS` | 2 | m | The width of the passage is measured this often along a route, to find its narrowest point. |
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
| `CURVE_SAMPLE_SPACING_METERS` | 4 | 0.5, 1, 2, 3, 5, 6, 8, 10, 12, 16, 20 | Recent |
| `FREE_DISTANCE_STEP_METERS` | 0.5 | 0.125, 0.25, 1, 2 | Recent |
| `PUSH_SPREAD_METERS` | 64 | 16, 32, 48, 80 | Recent |
| `PUSH_TAPER_METERS` | 80 | 48, 64, 96, 112 | Recent |
| `BEND_STENCIL_METERS` | 8 | 4, 12, 16, 24 | Recent |
| `BEND_SPEED_MARGIN` | 0.8 | 0.9, 1.0 | Older, 9 scenarios |
| `LANE_STEP_METERS` | 2 | 0.5, 1, 4, 8 | Recent |
| `ROOM_WINDOW_METERS` | 4 | 2, 8, 12 | Recent |
| `AVERAGE_WINDOW_METERS` | 6 | 2, 4, 10, 14 | Recent, and values above 8 act as 8 |
| `FILE_WINDOW_METERS` | 8 | 4, 12, 16, 24 | Recent |
| `BELL_REACH_METERS` | 40 | 20, 30, 60, 80 | Recent |
| `FILE_AHEAD_SPEED_FLOOR_FRACTION`, `FILE_AHEAD_DECELERATION_FRACTION` | 0.2, 0.5 | The pairs 0.5 and 0.2, 0 and 0.5, 0.2 and 1.0 | Older, 9 scenarios |
| `END_BRAKING_FRACTION` | 0.3 | 0.2, 0.5, 0.7 | Older, 5 seeds of 9 scenarios |
| `FAR_BEND_LOOK_SECONDS` | 1.2 | 1.0, 1.6 | Older, 9 scenarios |
| `HEADING_TIME_CONSTANT_SECONDS` | 0.1 | 0.07, 0.15, 0.25 | Older, 9 scenarios |
| `WAIT_AHEAD_MIN_COSINE` | 0.87 | 0 (90 degrees) | Recent |
| `CROSSING_SAMPLE_METERS` | 6 | 2, 3, 4, 9, 12 | Recent |
| `LATE_YIELD_SPEED_FRACTION` | 0.5 | 0.3, 0.7 | Older, 9 scenarios |
| `WAIT_ROOM_METERS` | 24 | 12, 18, 30, 36 | Recent |
| `SHIFT_BEFORE_METERS` | 20 | 10, 40, 80 | Older, 11 scenarios |
| `SHIFT_AFTER_METERS` | 10 | 0, 20, 80 | Older, 11 scenarios |
| `SWAY_SLOPE` | 0.15 | 0.2, 0.25, 0.3 | Older, 11 scenarios |
| `PASS_ROOM_MARGIN_METERS` | 4 | 2 | Older, corridor only |
| `FILE_PASS_ROOM_MARGIN_METERS` | 1 | 0, 2, 4 | Older, corridor only |
| `PASS_SQUEEZE_METERS` | 3 | 6 | Older, corridor only |
| `FORMED_LANE_METERS` | 2 | 3, 4, with the same results | Older, 9 scenarios |
| `FORMED_FRACTION` | 0.9 | 0.5, 0.7, with the same results | Older, 9 scenarios |
| `SET_UP_WORK_PER_TICK` | 4000 | 2000, 6000, 8000, 12000 | Recent, start delay against tick time |
| `WIDEN_PATIENCE_PASSES` | 30 | 10, 20 | Recent with 40 seeds, routes that change against work saved |
| `PARK_AHEAD_SECONDS` | 16 | 1, 2, 3, 5, 8, 12, 24, 32 | Recent, with `--crossfollow`, `--oldroute` and `--slotpoint` on |

### Not tuned yet

No test compared these constants at several values on their own. Some came with a switch, and their tests compared only the switch on and off. A few changed once as part of a larger fix, such as the crossing margins.

- Changes to the base game movement: `SIGHT_TOUCH_MARGIN_METERS`, `TELEPORT_MAX_CELLS`, `HIT_REACH_CELLS`.
- Parked squads: `PARKED_STAMP_MARGIN_METERS`, `PARKED_ROUTE_STEP_METERS`, `PARKED_ROUTE_MARGIN_METERS`, `TIGHT_CIRCLE_RADII`, `PARKED_SPEED_FRACTION`.
- Route set-up: `CORNER_CUT_MARGIN_METERS`, `CENTER_ROUTE_MIN_GAIN_METERS`, `REPLAN_MAX_STRETCH`, `FORMATION_DIRECTION_SPAN_METERS`, `MIN_CHORD_LENGTH_SQUARED`, `ROUTE_END_EPSILON_METERS`.
- Smooth curve and wide bends: `MAX_FULL_ROUTE_METERS`, `CELL_SKIP_MARGIN_METERS`, `LANE_MAX_DISTANCE_METERS`, `BODY_MARGIN_METERS`, `LANE_MARGIN_METERS`, `PASSAGE_MIDDLE_MARGIN_METERS`, `AWAY_SEARCH_MARGIN_METERS`, `PUSH_STEP_METERS`, `WIDEN_STEP_FRACTION`.
- Lanes: `MAX_FULL_LANE_METERS`, `LANE_CLAMP_MARGIN_METERS`, `LANE_SLOPE`, `SAME_COLUMN_METERS`, `SQUEEZED_GAP_METERS`, `FOLD_SHRINK_FACTOR`, `MIN_FORWARD_STEP_FRACTION`.
- Places and matching: `START_BLEND_METERS`, `END_BLEND_METERS`, `FILE_SPACING_RADII`, `FILE_SPACING_GAP_METERS`, `FILE_HEADWAY_SECONDS`, `MIN_FORMATION_REACH_METERS`, `COORDINATES_BEHIND_METERS`, `COORDINATES_AHEAD_METERS`, `REASSIGN_INTERVAL_SECONDS`, `REASSIGN_SIDE_WEIGHT`, `REASSIGN_MIN_SAVING_FRACTION`, `REASSIGN_MIN_SAVING_SQUARED`.
- Anchor speed: `LAG_SLOWDOWN_FRACTION`, `LAG_SLOWDOWN_METERS`, `TURN_SPEED_FLOOR`, `TURN_SPEED_RANGE`, `REAR_MARGIN_METERS`, `TURN_LOOK_AHEAD_METERS`, `FILE_LEAVE_AHEAD_METERS`, `END_DECELERATION_FRACTION`, `LAG_WAIT_METERS`, `LAG_WAIT_RANGE_METERS`, `CRAWL_FRACTION`, `LAG_STOP_METERS`, `ANCHOR_STILL_SPEED`, `STUCK_WAIT_SECONDS`, `IGNORE_LAG_SECONDS`.
- Member steering: `FORM_SPEED_TIME_CONSTANT_SECONDS`, `PROGRESS_BEHIND_METERS`, `PROGRESS_AHEAD_METERS`, `REJOIN_BEHIND_METERS`, `REJOIN_LOOK_METERS`, `REJOIN_STEP_METERS`, `NO_LANE_REJOIN_METERS`, `PURSUIT_PLACE_METERS`, `PURSUIT_MIN_METERS`, `PURSUIT_MAX_METERS`, `PURSUIT_LOOK_SECONDS`, `PURSUIT_STEP_METERS`, `PURSUIT_MIN_AHEAD_METERS`, `CATCH_UP_GAIN`, `AHEAD_SPEED_FLOOR_FRACTION`, `AHEAD_DECELERATION_FRACTION`, `PLACE_GAIN`, `ARRIVAL_BRAKING_FRACTION`, `ARRIVAL_SLACK_METERS`, `SLOT_SPEED`, `END_STOP_REACH_METERS`, `AVOIDANCE_MIN_CHANGE_SQUARED`, `HELD_SPEED_FRACTION`, `HELD_LIMIT_SECONDS`, `AVOIDANCE_OFF_SECONDS`, `BEND_LOOK_MIN_METERS`, `BEND_LOOK_SECONDS`, `BEND_PLACE_FRACTION`, `BEND_MIN_DISTANCE_METERS`, `BEND_MIN_SINE`, `BEND_FLOOR_FRACTION`, `BRAKE_LOOK_SECONDS`, `BRAKE_LOOK_MARGIN_METERS`, `STEER_MIN_SPEED`, `STEER_MIN_DISTANCE_METERS`.
- Avoidance and making room: `ORCA_HORIZON_SECONDS`, `ORCA_MARGIN_METERS`, `MATE_AVOID_RANGE_METERS`, `SHARED_EFFORT`, `FULL_EFFORT`, `SPEED_TIME_CONSTANT_SECONDS`, `MIN_YIELD_SPEED`, `YIELD_SPEED_FRACTION`, `YIELD_MAX_METERS`, `HOME_REACHED_METERS`.
- Stuck units and detours: `PROGRESS_CHECK_SECONDS`, `MIN_PROGRESS_METERS`, `STUCK_PLACE_METERS`, `DETOUR_AHEAD_METERS`, `DETOUR_TOLERANCE_METERS`, `DETOUR_LIMIT_SECONDS`.
- Crossings and giving way: `CROSSING_HORIZON_METERS`, `MIN_CROSSING_SINE`, `CROSSING_MARGIN_METERS`, `CROSSING_TIME_MARGIN_SECONDS`, `MIN_PREDICTED_SPEED`, `HELD_SPEED`, `GIVE_UP_SECONDS`, `IGNORE_SECONDS`, `SAME_LINE_METERS`.
- Head-on meetings and lane shifts: `HEAD_ON_MAX_COSINE`, `HEAD_ON_MARGIN_METERS`, `HEAD_ON_EXTRA_GAP_METERS`, `MEETING_REACH_METERS`, `SIDE_SHIFT_SLOPE`, `SHIFT_REACH_METERS`, `SWAY_MIN_SPEED_FRACTION`.
- Passing and following: `SAME_WAY_MIN_COSINE`, `CROSS_SIDE_FRACTION`, `PASS_MIN_SPEED_GAIN`, `PASS_LOOK_SECONDS`, `PASS_BACK_METERS`, `PASS_HORIZON_METERS`, `PASS_END_MARGIN_METERS`, `PASS_GAP_METERS`, `PASS_ROOM_STEP_METERS`, `FOLLOW_HEADWAY_SECONDS`, `FOLLOW_MIN_GAP_METERS`, `FOLLOW_GAIN`.
- Work limits: `SMOOTHING_ROUNDS`, `WIDEN_ROUNDS`.

`FULL_SEARCH_BUDGET` needs no tuning, because it equals the limit of the game on the steps of one search.

The other constants of "Work limits and capacities" are not tuned either. They size tables and arrays, or they hold work costs fitted to measured times. The counts in "Counts that follow from distances" follow from the distances above.
