# Switches

This file lists the features that the simulator adds to the game and that you can turn on or off on the command line, for example `sim.exe --formation off`.

## Route smoothing set-up

| Switch | What it does | Code |
|---|---|---|
| `--smooth` | The route becomes smooth curves that keep clear of blocking terrain. | `smooth_route` (smoothing.c) |
| `--widebends` | The smoothing also makes bends wider that are too sharp for the top speed of the squad. It has its own toggle, and it acts only while `--smooth` is on. | `widen_bends` (smoothing.c) |
| `--tightbends` | The widening also makes the route shorter: it pulls bends that are wider than needed toward their inner side, down to the radius at which every unit of the squad can still drive at full speed, and the route keeps its clearance from blocking terrain. It acts only while `--widebends` is on. | `tighten_bends` (smoothing.c) |
| `--smoothreorder` | A squad that gets a new order while it moves keeps its motion: its new route starts with a turn along lead-in points on a turning circle that leaves the start along the current motion, and its anchor starts at its current speed instead of 0. The circle is a bit tighter than the units can drive at their current speed, so they slow down a little in the turn, and the bend widening of the smoothing leaves it as it is. | `add_lead_in_points` (route.c), `get_reorder_speed`, `prepare_formation_route` (formation.c), `start_widening` (smoothing.c) |
| `--smoothstop` | The smoothing stops early when its curve still runs too close to blocking terrain for the body of a unit after 10 displacement iterations, and the squad keeps its unsmoothed route. Without it, such a smoothing runs all 60 iterations and nearly always ends the same way. | `run_displacement_iteration` (smoothing.c) |

## Navigation

The anchor of a squad is a point that moves along the route. Each unit has a slot in the formation around the anchor.

| Switch | What it does | Code |
|---|---|---|
| `--lanes` | Each unit drives along its own lane beside the route. Where the passage is narrow, the squad drives in column formation. | `set_up_lanes` (lanes.c) |
| `--formation` | The squad keeps its shape. The anchor waits for units that fall behind, units swap slots to shorten their ways, a unit ahead of its slot slows gently, and a unit just past its slot waits for it. | `wait_for_laggards`, `swap_slots` (formation.c), `slow_down_ahead`, `is_waiting_ahead` (steering.c) |
| `--driving` | Units drive more smoothly. They steer in proportion to how far they point the wrong way, look farther ahead for bends, aim only at rejoin points with clearance, follow their lanes, and drive straight onto their final parking positions at the end. | `steer_proportionally`, `get_bend_point_distance`, `is_rejoin_point_usable`, `find_pursuit_target`, `get_end_speed` (steering.c) |
| `--detour` | A unit that is stuck behind blocking terrain gets a detour. | `follow_detour` (detour.c) |
| `--wholeparked` | Off by default, under review. When the final formation of an order overlaps a parked squad, the whole formation moves to the nearest spot where it fits as laid out, so the squad parks in its regular formation. A parked squad counts as one area: the convex hull of its stopped units, widened by their radius plus a 1 m margin. The search tries shifts on rings 2 m apart, up to 50 m away, starting on the side away from the parked squad. The end of the route moves with the formation, so the lanes do not narrow beside the parked squad. Without a free spot, nothing moves, and `--freeparking` moves single positions as before. | `shift_parking_positions` (stamping.c) |
| `--parkedbox` | Off by default, under review. In the ORCA avoidance, each parked squad counts as one body, its parked box: the smallest rectangle around its stopped units, widened by their radius, so it has straight sides and round corners. A moving unit outside the box avoids the box instead of each parked unit, so it cannot steer into the gap between two parked units. It slows for the box only within half a second of reaching it, so a squad that parks beside the box drives straight to its spots. A unit already inside the rectangle, and a parked squad with only one stopped unit, keep the avoidance unit by unit. | `update_parked_boxes`, `add_box_lines` (avoidance.c) |
| `--freeparking` | A final parking position that overlaps the body of a stopped unit of a parked squad moves to the nearest free spot, clear of parked units, blocking terrain and the other final parking positions of the squad, so the unit parks there instead of pushing on toward an occupied place. In the mod, it acts on the final parking positions that the game laid out. | `free_parking_positions` (stamping.c) |
| `--factiontuning` | Each faction uses its own values for a few driving and formation constants, found by parameter scans: the Order units look farther ahead before braking and pursue a point on their lane only farther from their slot, and the Crayven units keep a wider gap in the ORCA avoidance, look farther ahead for bends, steer faster and wait for laggards later. The Phoenix units keep the values from before the tuning. Off, every faction uses those values. | `get_tuning` (tuning.c) |

## Coordination between squads

These switches act only in the coord mode.

| Switch | What it does | Code |
|---|---|---|
| `--friendcoord` | The coordination acts only between squads of one side: squads of different sides, such as the player and the computer, do not give way to each other, shift apart, pass each other or re-route around each other. The ORCA avoidance between their units stays. In the mod, the side tells whether a player or the computer orders the squad. In the simulator, every squad is on one side, so it changes nothing there. | `are_squads_coordinated` (sim.h) |
| `--ignoreboarders` | An APC does not avoid the units of its team that the game drives to board it, so it does not swerve away from them: the ORCA avoidance of the APC leaves them out, and the coordination does not act between the APC's squad and theirs. The route search at an order does not go around them either. The ORCA part acts in the form mode too. The mod finds these units from the target of their game mover; in the simulator, only the `boarding` scenario has them. | `is_boarding`, `are_squads_coordinated` (sim.h), `compute_orca_velocity` (avoidance.c), `stamp_parked` (stamping.c) |

### Crossings

| Switch | What it does | Code |
|---|---|---|
| `--crossing` | Where two routes cross, one squad gives way. It keeps full speed as long as it can, then slows firmly. | `give_way_at_crossing` (coordination.c) |
| `--speedfirst` | The faster squad goes first in narrow head-ons, and at crossings while `--leastdelay` is off. | `give_way_at_crossing`, `take_turns` (coordination.c) |
| `--leastdelay` | At a crossing, the squad that would wait less gives way: each wait runs from its own zone entry until the other squad has left its zone. This replaces `--speedfirst` at crossings, so a slow squad close to the crossing goes first instead of waiting for a fast squad that is still far away. | `get_give_way_delay`, `give_way_at_crossing` (coordination.c) |

### Head-ons

| Switch | What it does | Code |
|---|---|---|
| `--keepright` | Squads that meet head-on shift apart, just before the head-on meeting point and with a little extra gap, and shift back just after it. | `pass_head_on` (coordination.c) |
| `--partialshift` | When there is too little side room, each squad still shifts as far as it can. | `pass_head_on` (coordination.c) |
| `--turns` | When a head-on is too narrow to pass each other, one squad waits at a turnout. | `take_turns` (coordination.c) |
| `--speedreplan` | When the top speed of either squad of a head-on changes, for example with the speed booster, both squads move their shifts to the new head-on meeting point instead of keeping their first plan. Where a squad has already started to shift, its shift also keeps covering the old place. | `update_top_speed_change` (coordination.c), `keep_replanned_section` (shift.c) |
| `--timedshift` | The full shift of a head-on starts and ends as long before and after the head-on meeting point as at the base top speed, so a boosted squad shifts earlier along its route. At the base top speed nothing changes. | `get_head_on_target_shift` (coordination.c) |
| `--narrowpass` | A squad in line formation plans its head-on shift for square formation, or for column formation when the side room is too small for square formation. It keeps its line until 2 s before the front members of the two squads would meet, and goes back to its line as soon as the two squads have passed each other. | `plan_narrow_formations`, `narrow_before_meeting`, `have_passed` (coordination.c), `narrow_formation`, `widen_formation` (formation.c) |
| `--infantryshape` | Infantry squads (foot units, terrain class 2) never take square or column formation for a head-on: `--narrowpass` leaves them in their own formation, and only the other squad narrows. | `can_narrow` (formation.c) |

### Passing on one route

| Switch | What it does | Code |
|---|---|---|
| `--pass` | A faster rear squad passes a slower front squad on the same route, or follows it. | `pass_or_follow`, `try_pass` (passing.c) |
| `--keeproute` | The front squad keeps its route, and the rear squad goes around it. | `plan_pass` (passing.c) |
| `--formfirst` | The rear squad waits until the front squad has shifted aside. | `try_pass` (passing.c) |

### Parked squads

| Switch | What it does | Code |
|---|---|---|
| `--parkreroute` | Squads re-route around squads that park on their way, and they re-route early when another squad will park there first. | `reroute_around_parked`, `avoid_parking_ahead` (reroute.c) |
| `--parkrobust` | The re-routes get two safeguards. A squad counts as parked while its units still turn into their final parking positions, and a squad whose re-route fails keeps its old route. | `update_settled` (reroute.c), `fail_move` (squad.c) |

## Game behavior

These switches make the simulator do what the game does where it did otherwise. They act only in the simulator, since the game mod gets this behavior from the game itself.

| Switch | What it does | Code |
|---|---|---|
| `--parkedpush` | The units of a squad without an order are pushed out of overlaps in the formation modes too, as the game pushes every unit in its own frame (*Physics* 0x510b20). Off, the formation modes never pushed them. | `apply_physics` (physics.c) |
| `--squadfriends` | The base game avoidance treats only units of one squad as friends, as the game compares the value at entity +0x3e, which differs between squads. Off, every unit of one team counts as a friend. | `are_game_friends` (movement.c) |
| `--gamelayout` | The layout of the final parking positions of an order ignores parked squads, as the game lays out the final parking positions of the mod without the units, so they can overlap a parked squad. Off, the simulator stamped the parked units as cliffs for the layout. | `lay_out_parking_positions` (squad.c) |

## Path search and set-up

These switches change when and on which processor cores the path searches and the route smoothing set-ups run.

| Switch | What it does | Code |
|---|---|---|
| `--fullsearch` | Each path search finishes within one tick. It acts in every mode, also in the original game mode. | `update_search` (squad.c) |
| `--setupthreads` | Several squads smooth their routes at the same time on several processor cores. | workers.c, `finish_threaded_set_ups` (squad.c) |
| `--setupaccumulate` | The route smoothing set-up uses temporal accumulation: it spreads over several ticks, under a work budget for each tick. | `continue_accumulated_set_up` (squad.c) |
