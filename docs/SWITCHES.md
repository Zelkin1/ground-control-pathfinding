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
| `--infantrysquare` | Infantry squads never switch to column formation: in a passage too narrow for their lanes, they keep square formation, with lanes held off blocking terrain by the lane clamp. Vehicle squads still switch. It acts only with `--lanes` on. | `update_column_formation` (formation.c) |
| `--fastswap` | The squad checks every 0.25 s instead of every 0.5 s whether swapping slots would shorten the ways of its units. It acts only with `--formation` on. | `is_swap_due` (formation.c) |
| `--driving` | Units drive more smoothly. They steer in proportion to how far they point the wrong way, look farther ahead for bends, aim only at rejoin points with clearance, follow their lanes, and drive straight onto their final parking positions at the end. | `steer_proportionally`, `get_bend_point_distance`, `is_rejoin_point_usable`, `find_pursuit_target`, `get_end_speed` (steering.c) |
| `--detour` | A unit that is stuck behind blocking terrain gets a detour. | `follow_detour` (detour.c) |
| `--wholeparked` | When the final formation of an order overlaps a parked squad, the whole formation moves to the nearest spot where it fits as laid out, so the squad parks in its regular formation. A parked squad counts as one area: the convex hull of its stopped units, widened by their radius plus a 1 m margin. The search tries shifts on rings 2 m apart, up to 50 m away, starting on the side away from the parked squad. The end of the route moves with the formation, so the lanes do not narrow beside the parked squad. Without a free spot, nothing moves, and `--freeparking` moves single positions as before. | `shift_parking_positions` (stamping.c) |
| `--freeparking` | A final parking position that overlaps the body of a stopped unit of a parked squad moves to the nearest free spot, clear of parked units, blocking terrain and the other final parking positions of the squad, so the unit parks there instead of pushing on toward an occupied place. In the mod, it acts on the final parking positions that the game laid out. | `free_parking_positions` (stamping.c) || `--factiontuning` | Each faction uses its own values for a few driving and formation constants, found by parameter scans: the Order units look farther ahead before braking and pursue a point on their lane only farther from their slot, and the Crayven units keep a wider gap in the ORCA avoidance, look farther ahead for bends, steer faster and wait for laggards later. The Phoenix units keep the values from before the tuning. Off, every faction uses those values. | `get_tuning` (tuning.c) |

## Coordination between squads

These switches act only in the coord mode.

| Switch | What it does | Code |
|---|---|---|
| `--friendcoord` | The coordination acts only between squads of one side: squads of different sides, such as the player and the computer, do not slow down for each other or re-route around each other. The ORCA avoidance between their units stays. In the mod, the squads of the player and of the teams allied with it, such as the allies that the game shows in yellow, are one side, and all other squads the other side. In the simulator, every squad is on one side, so it changes nothing there. | `are_squads_coordinated` (sim.h) |
| `--ignoreboarders` | An APC does not avoid the units of its team that the game drives to board it, so it does not swerve away from them: the ORCA avoidance of the APC leaves them out, and the coordination does not act between the APC's squad and theirs. The route search at an order does not go around them either. The ORCA part acts in the form mode too. The mod finds these units from the target of their game mover; in the simulator, only the `boarding` scenario has them. | `is_boarding`, `are_squads_coordinated` (sim.h), `compute_orca_velocity` (avoidance.c), `stamp_parked` (stamping.c) |

### Parked and moving squads

| Switch | What it does | Code |
|---|---|---|
| `--parkreroute` | Squads re-route around squads that park on their way, and they re-route early when another squad will park there first. With `--moveahead`, the predictions of `--moveahead` find those early re-routes instead. | `reroute_around_parked`, `avoid_parking_ahead` (reroute.c) |
| `--parkrobust` | The re-routes get two safeguards. A squad counts as parked while its units still turn into their final parking positions, and a squad whose re-route fails keeps its old route. | `update_settled` (reroute.c), `fail_move` (squad.c) |
| `--moveahead` | In every tick, a moving squad predicts, with every squad driving along its route at its top speed, how long it would wait for each other moving squad that does not give way to it. It adapts only to waits that start within 10 s. After a predicted wait of 2 s or more, it runs trial searches of its route, once per other squad and order. When the two squads drive the same way (less than 45° apart at the meeting), it searches around the strip that the other squad covers from 2 s before its first predicted wait to 2 s after its last one. At a crossing (45° to 135° apart), it runs no search and only slows down, as described below. Head-on (more than 135° apart), both squads search first. Each one keeps the side on which the other squad already lies, and searches around discs on the other side of its own route where it would be during the meeting, so that its route moves by about half of the needed gap. A squad whose found route strays to the wrong side searches once more, with discs that start closer ahead of its front. When that side has no room, both try the other side. When the pair fails, the squad searches alone and moves its route by the whole needed gap. A squad re-routes only when the found route stays within the needed gap plus 3 grid cells of its current route, strays to its pass side when it meets the other squad head-on, arrives at least 1 s earlier, and its formation comes no closer than 2 m to the formation of any moving squad at any time until it arrives. A pair re-routes when both found routes pass these tests together: they must arrive at least 1 s earlier in sum, and meet neither each other nor a third moving squad. A third squad that either side could wait for, or that a route meets head-on, does not count, because the next trials handle it. A found route that meets a third moving squad gets one more search, also around the strip of that squad. The set-up of a head-on re-route stamps smaller discs, so that the route smoothing does not move the route out by another formation width. In the predictions, a squad that has arrived stands at its final parking positions. When they would block the route of a squad within 16 s, that squad re-routes around them at once, as the early re-route of `--parkreroute` does, which then stays off. Without a re-route, after a predicted wait of 0.5 s or more, the squad of the pair with the shorter predicted wait slows down early, so that it reaches the place of its first wait only when that wait is over. It keeps doing so until it would no longer wait for the other squad, and the other squad still counts it as slowing down for 2 s after that, so that the two squads do not swap that role while the predicted wait drops out for a few ticks. It does not slow down when the other squad would still meet it while it waits, as an oncoming squad on the same route would. It acts only with `--parkreroute` and `--parkrobust` on. | `avoid_moving_ahead`, `try_reroutes`, `try_route_around`, `try_pair_routes`, `is_route_close`, `avoid_met_squad` (moving_ahead.c), `stamp_parked` (stamping.c), `coordinate_squads` (coordination.c) |
| `--closepass` | Squads that meet head-on pass each other closer. Their routes move apart only by what the needed gap still lacks past the sideways offset that the squads already have from each other at the meeting, plus 4 m. The discs around which the route search and the set-up move the routes also shrink by half a grid cell each, because the search and the set-up block every cell that a disc touches. A search disc keeps a radius of at least two cells, so that the search does not go around it on the wrong side. It acts only with `--moveahead`. | `get_pass_shift`, `add_side_stamps` (moving_ahead.c) |
| `--lateyield` | At a crossing, the squad that slows down for `--moveahead` keeps its full speed while its speed cap would stay above half of its top speed. It then slows down later and more firmly, near the crossing. Behind a squad that drives the same way, it still slows down early. It acts only with `--moveahead`. | `avoid_moving_ahead` (moving_ahead.c) |

## Game behavior

These switches make the simulator do what the game does where it did otherwise. They act only in the simulator, since the game mod gets this behavior from the game itself.

| Switch | What it does | Code |
|---|---|---|
| `--parkedpush` | The units of a squad without an order are pushed out of overlaps in the formation modes too, as the game pushes every unit in its own frame (*Physics* 0x510b20). Off, the formation modes never pushed them. | `apply_physics` (physics.c) |
| `--squadfriends` | The base game avoidance treats only units of one squad as friends, as the game compares the value at entity +0x3e, which differs between squads. Off, every unit of one team counts as a friend. | `are_game_friends` (movement.c) |
| `--gamelayout` | The layout of the final parking positions of an order ignores parked squads, as the game lays out the final parking positions of the mod without the units, so they can overlap a parked squad. Off, the simulator stamped the parked units as cliffs for the layout. | `lay_out_parking_positions` (squad.c) |

## Player orders

These switches act only in the game mod, since the simulator has no player interface.

| Switch | What it does | Code |
|---|---|---|
| `--keeporder` | When the player gives one move order to several selected squads, the game lays out one position per squad, in rows of up to 4. With the switch, the front squads get the front row, and in each row the squads keep their order from side to side, so they do not cross each other on the way. Off, the game hands the positions out in the order in which the squads were selected. | `note_group_positions` (mod/pathfinding/group_order.c) |
| `--clickdeselect` | A left click on ground with no entity under the cursor deselects every squad, so that the next orders go only to the squads that the player selects after it. A click within the body of a unit, plus 1 m, still selects the squad of that unit, and the click moves no squad, even with the game option `leftclickmovement` on. Off, the click leaves the selection as it is, and with `leftclickmovement` on it gives the selected squads a move order to the clicked point. | `prepare_ground_click`, `finish_ground_click`, `hook_pick_nearest` (mod/interface/ground_click.c) |
| `--middlerotate` | While the middle mouse button is held in the 3D view, moving the mouse turns the camera as the screen borders do: to the left or right, it turns the camera around as the left or right border does, and up or down, it tilts the camera as the top or bottom border does. One pixel of cursor movement turns the camera as far as a border turns it in 0.0018 s, times the gcmod.ini setting `middlerotate_sensitivity` (1 by default), so the rotation speed option of the game scales it too. The cursor stays where the button went down. With the switch on, the screen borders no longer turn the camera. Off, the game ignores the middle button, and the borders turn the camera. | `hook_border_rotation` (mod/interface/middle_rotate.c) |

## Backward moves and run-over kills

`--reverse` acts in the game mod and, in the form and coord modes, in the simulator, whose `reverse` scenario gives backward move orders. The kills of `--crush` happen only in the game mod; its ORCA part is shared code, but the simulator has one team, so it changes nothing there.

| Switch | What it does | Code |
|---|---|---|
| `--reverse` | A move order of the player given with the reverse key held (gcmod.ini `reverse_key`, Alt by default) makes the squad drive backward, wherever its destination lies: a unit whose rear does not point along its way first turns on the spot until it does (within 18°). Every member must be a vehicle; a squad with infantry turns and moves forward as usual. Each unit drives backward on its own lane along the route of the base game path search, bends included, with its rear first, at its own pace and 30 % of its top speed, then backs onto its final parking position: the mod neither shortens nor smooths that route, and adds no lead-in turn. The squad keeps no formation: no unit waits for another, the slots never swap, and only the ORCA avoidance acts between the units. The coordination between squads does not act for it either, in both directions (no slowing down, waiting or re-routes for other squads, and no other squad slows down or re-routes for it), until it parks. Each unit takes the final parking position that matches its place in the squad, so the rear row stays at the rear, and at the destination the units keep facing away from it. A queued order (a chain of move orders) drives backward when the key was held as the player queued it. Orders of the computer always drive forward. Off, every order drives forward. | `decide_reverse`, `prepare_formation_route`, `advance_anchor` (formation.c), `steer_backward` (steering.c), `is_driving_backward`, `are_squads_coordinated`, `get_drive_direction`, `get_heading_target`, `limit_reverse_speed` (sim.h), `hook_order_move`, `hook_queue_move` (mod/gameplay/reverse_order.c) |
| `--crush` | A ground vehicle that drives at 3 m/s or faster kills each foot unit of a team that is not allied with its own when their bodies touch, with a body 0.7 m wider than its radius, in the game's unit physics (*Physics* 0x510b20, checked before it pushes them apart), as a weapon kill: the game's death function runs, with the vehicle's squad as the killer. Own and allied infantry are safe. The ORCA avoidance of a vehicle leaves out the infantry of the other side, so the vehicle drives straight at it; the infantry still avoids the vehicle. Units that the game keeps from taking damage, such as invulnerable units of a mission, never die this way. Off, vehicles avoid all infantry and never kill it by driving. | `hook_physics`, `note_crush_frame` (mod/gameplay/crush.c), `can_crush` (sim.h), `compute_orca_velocity` (avoidance.c) |

## Order routes in the mod

This switch acts only in the game mod. The simulator always searches the route of an order with the parked squads counted as blocking terrain.

| Switch | What it does | Code |
|---|---|---|
| `--avoidparked` | At a new order, the mod searches the route again from the front unit with the parked squads counted as blocking terrain, and uses it in place of the route of the game, which ignores parked units. Without a path around them, the squad keeps the route of the game. Off, squads drive through parked squads and rely on the avoidance alone. | `take_route_around_parked` (mod/pathfinding/bridge.c) |

## Path search and set-up

These switches change when and on which processor cores the path searches and the route smoothing set-ups run.

| Switch | What it does | Code |
|---|---|---|
| `--fullsearch` | Each path search finishes within one tick. It acts in every mode, also in the original game mode. | `update_search` (squad.c) |
| `--setupthreads` | Several squads smooth their routes at the same time on several processor cores. In the mod, only the set-ups of new orders use the other cores, and they end in the frame in which they start, so no squad starts later. The re-routes of the mod smooth their routes on the main thread. | `submit_threaded_smoothing`, `finish_threaded_set_up` (formation.c), workers.c, `finish_threaded_set_ups` (squad.c), `finish_frame_set_ups` (mod/pathfinding/bridge.c) |
| `--setupaccumulate` | The route smoothing set-up uses temporal accumulation: it spreads over several ticks, under a work budget for each tick, and the squad starts to move once its set-up ends. Only the simulator acts on it: the mod runs each set-up in one call. | `continue_accumulated_set_up` (squad.c) |

