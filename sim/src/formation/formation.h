/* formation.h holds the types, constants and functions that the files of formation following and
 * of the coordination between squads share.
 *
 * Files: avoidance.c, route.c, lanes.c, formation.c, steering.c, coordination.c, shift.c,
 * overtaking.c and replan.c define what this header declares. The top comment of sim.h says what
 * each of them covers, and it defines the terms that they use, such as the anchor, the place, the
 * lane, the clamp, the shift and the detour.
 *
 * Callers: Besides those files, sim.c, movement.c, squad.c and metrics.c include this header,
 * because they call into formation following. sim.h holds the types and the state of the whole
 * simulator, such as `Unit` and `Squad`. */
#ifndef FORMATION_H
#define FORMATION_H

#include "sim.h"

/* avoidance.c defines these functions of the new unit avoidance. */

/* These are the values of `avoided_units` in `compute_orca_velocity`. They say which other units
 * it avoids. A unit counts as moving when it is neither stopped nor re-orienting. */
enum {
    ORCA_AVOID_ALL = 0,          /* It avoids every other unit. */
    ORCA_AVOID_MOVING_MATES = 1, /* It avoids only the moving units of the same squad. */
    ORCA_AVOID_MOVING_UNITS = 2, /* It avoids only the moving units. */
};

/* `compute_orca_velocity` returns the velocity closest to `preferred` that keeps
 * `units[unit_index]` from touching the units that `avoided_units` picks within a short time, and
 * that is not faster than the top speed of the unit. The bodies keep a small gap, and units too
 * far away to touch within that time do not count. Two moving units share the change between
 * them. A unit that is stopped or turns on the spot cannot dodge, so the other unit makes the
 * whole change. Squad mates count only when they are about to touch. Blocked cells do not count,
 * so callers check the way along the new velocity.
 * Edge case: When no velocity keeps clear of all those units, it returns the velocity whose
 *   largest break of the ORCA limits, one limit for each avoided unit, is the smallest. */
V2 compute_orca_velocity(int unit_index, V2 preferred, int avoided_units);

/* `yield_mate` steers a stopped unit of a moving squad. The unit steps aside for moving squad
 * mates that come at it, and it avoids no other units.
 * Result: It returns the steering. When the unit steps aside, `yield_mate` also accelerates it.
 * Edge case: Once a unit that stepped aside is out of `reach` of `waypoint`, its slot, it no
 *   longer counts as stopped, so formation following drives it back from the next tick on. */
float yield_mate(int unit_index);

/* `yield_parked` steers a unit of a parked squad. The unit makes a little room for moving units
 * that come at it, then drives back to `home`, where it parked, and turns to its `facing`.
 * Result: It returns the steering. When the unit moves, `yield_parked` also accelerates it. */
float yield_parked(int unit_index);

/* The files of formation following and of coordination share these values. */
#define LENGTH_SQUARED_EPSILON 1e-6f   /* A vector has a direction above this squared length. */
#define LARGE_VALUE 1e9f               /* Minimum searches start here, above any real value. */
#define LANE_MAX_DISTANCE_METERS 30.0f /* Free distances are measured up to this far. */
#define LANE_MARGIN_METERS 1.5f        /* Lane plans keep bodies this far from blocked ground. */

/* route.c defines these functions of the squad route in formation following. */

/* `get_route_point` returns the point at `distance` along the squad route. It needs the
 * `route_distance` of the squad, which `set_up_formation` fills once the route is shortened and
 * smoothed.
 * Result: `direction` becomes the direction of the route segment there, with length 1.
 * Edge cases: Before the start, `get_route_point` returns the start point, and past the end, the
 *   end point. On a segment of almost zero length, it returns the end of the segment and takes
 *   `final_facing` as the direction. */
V2 get_route_point(const Squad *squad, float distance, V2 *direction);

/* `get_extended_route_point` works like `get_route_point`, but before the start and past the end
 * it continues the route in a straight line along the first or the last segment. It also needs
 * the `route_length` of the squad. */
V2 get_extended_route_point(const Squad *squad, float distance, V2 *direction);

/* `get_formation_direction` returns the forward direction of the formation at `distance` along
 * the route, with length 1. It turns gradually through corners, over a stretch of route before
 * and after `distance`.
 * Edge case: Where the two ends of that stretch lie very close together, for example on a very
 *   short route, it is the direction of the route segment at `distance`. */
V2 get_formation_direction(const Squad *squad, float distance);

/* `measure_free_distance` returns the distance from `point` to the first blocked cell for
 * `mover_class` along `direction`, or `max_distance` when there is none that close.
 * The result is accurate to a short fixed step. `direction` must have length 1, and
 * `max_distance` must be at most `LANE_MAX_DISTANCE_METERS`. A distance shorter than
 * `max_distance` is the same for every `max_distance`. */
float measure_free_distance(V2 point, V2 direction, int mover_class, float max_distance);

/* `move_route_start` moves the first point of the route to where the formation fits the members
 * best.
 * Inputs: The route must have at least two points. `slots` and `facing` are those of
 *   `set_up_formation`. `center` is the mean position of the members.
 * Edge cases: When the best fit is blocked or the straight line from it to the second point of
 *   the route is not clear, `move_route_start` tries `center`. When that fails too, the route
 *   keeps its start. */
void move_route_start(Squad *squad, const float *slots, V2 facing, V2 center, int mover_class);

/* `shorten_route` removes corners and back-and-forth steps from the squad route wherever a band
 * as wide as the widest member is clear. With `--cutcorners on`, it also cuts the corners that
 * are left, as far as a wider band stays clear.
 * Inputs: `max_radius` is the radius of the widest member, half the width of the band. */
void shorten_route(Squad *squad, float max_radius, int mover_class);

/* `take_shorter_center_route` replaces the squad route with a route that the search of the game
 * finds from `center` to the destination of the squad, when that route is shorter by at least a
 * fixed margin.
 * Inputs: The squad route must already have its start moved and be shortened. `center` is the
 *   mean position of the members, and `slots` and `facing` are those of `set_up_formation`.
 *   `max_radius` is the radius of the widest member, and `clearance` is the clearance of the
 *   search at corners.
 * Result: When `take_shorter_center_route` replaces the squad route, the new route also has its
 *   start moved and is shortened.
 * Edge case: When neither the quiet nor the active search finds a route, the squad route stays. */
void take_shorter_center_route(Squad *squad, const float *slots, V2 facing, V2 center,
                               float max_radius, int mover_class, float clearance);

/* `smooth_route` replaces the squad route with a smooth curve that has room for the formation.
 * Inputs: `max_radius` is the radius of the widest member, and `half_width` is half the width of
 *   the formation. `full_speed_radius` is the smallest bend radius that every member can drive at
 *   its top speed. With `--wideturns on`, bends sharper than that radius plus `half_width` are
 *   widened where the ground allows.
 * Result: The route becomes the samples of the curve and its end point.
 * Edge cases: When the curve lacks room for the body of the widest member at some sample, the
 *   route stays as it was. While the squad `is_replanning`, the route also stays as it was when
 *   the curve is longer than the route by more than a fixed factor. */
void smooth_route(Squad *squad, float max_radius, float half_width, float full_speed_radius,
                  int mover_class);

/* `begin_smoothing` and `run_smoothing` do the work of `smooth_route` in parts, so that a caller
 * can spread it over several ticks. Only one smoothing runs at a time on each thread: a new
 * `begin_smoothing` ends the last one of its thread, and `run_smoothing` goes on with the one of
 * its thread. Different threads can smooth the routes of different squads at the same time.
 * `begin_smoothing` takes the same inputs as `smooth_route` and changes nothing yet. Each
 * `run_smoothing` then does part of the work, and the squad route changes in the call that ends
 * it. Call both with the parked units stamped (`stamp_parked`), as for
 * `smooth_route`. While the stamped cells stay the same between the calls, the result is exactly
 * that of `smooth_route`; when they change, the later calls work on the new grid.
 * Inputs: `max_room_tests` limits the work of one call. It counts the tests of the room around a
 *   curve sample, which take nearly all of the time. It must be at least 1.
 * Result: `run_smoothing` returns true when the smoothing has ended, and false when work is
 *   left. */
void begin_smoothing(Squad *squad, float max_radius, float half_width, float full_speed_radius,
                     int mover_class);
bool run_smoothing(int max_room_tests);

/* `get_room_test_count` returns how many room tests all smoothings ran so far, the same count that
 * `max_room_tests` limits. The difference of two calls around a `run_smoothing` call gives the
 * work of that call. */
int get_room_test_count(void);

/* lanes.c defines these functions of the lanes in formation following. */

/* A `Lane` is one lane sample of a squad: the layout of all its lanes at one distance along the
 * squad route. `set_up_lanes` fills the samples at even steps along the route. Lane x is the
 * sideways offset of a lane from the route in meters, to the right of the direction of the
 * formation. The sideways slot offset of a member is the x of its `place_offset`. */
typedef struct {
    float scale;         /* The lane x of a slot is its sideways offset times this plus `shift`. */
    float shift;         /* All members share this sideways offset, in m. */
    float middle;        /* In single file, every lane x is this middle of the passage. */
    float min_x;         /* Lane x is clamped to at least this. */
    float max_x;         /* Lane x is clamped to at most this. */
    bool is_wide_enough; /* The squad fits side by side here. */
} Lane;

/* `allocate_lanes` allocates the lane samples of every squad on the heap. Call it once before
 * the first run. The samples stay allocated until the program ends. */
void allocate_lanes(void);

/* `set_up_lanes` lays out the lane samples of the squad, which `get_lane` and `get_lane_point`
 * read, and sets `lane_count`. It needs the `place_offset` of each member, and the
 * `route_distance` and `route_length` of the squad.
 * Inputs: `max_radius` is the radius of the widest member. */
void set_up_lanes(Squad *squad, float max_radius);

/* `get_lane` returns the lane values of the squad at `distance` along its route, interpolated
 * between the lane samples on either side of it. It takes `is_wide_enough` from the sample nearer
 * to `distance`. `set_up_formation` lays out the samples with lanes on.
 * Edge cases: Before the start of the route and past its last sample, it returns the first or the
 *   last sample. For a squad without lane samples (`lane_count` 0), it returns what the first
 *   sample of the squad holds: values from an earlier route of a squad at the same index, or
 *   values that were never set. */
Lane get_lane(const Squad *squad, float distance);

/* `do_lanes_fit` returns true when the squad fits side by side from `from_distance` to
 * `to_distance` along the route, checked at the spacing of the lane samples, and false
 * otherwise. */
bool do_lanes_fit(const Squad *squad, float from_distance, float to_distance);

/* `get_lane_point` returns the point of the lane of `unit` at `distance` along the squad route. In
 * single file, every lane runs down the middle of the passage. The lane also shifts sideways by
 * `get_side_shift`. Lanes are clamped to keep the body of the widest member clear of blocked
 * ground.
 * Edge cases: With `--lanes off`, it returns the route point itself. Before the start of the route
 *   and past its end, the route continues in a straight line. */
V2 get_lane_point(const Squad *squad, const Unit *unit, float distance);

/* formation.c defines these functions of formation following. */

#define END_BLEND_METERS 25.0f /* Places blend into the final shape this close to the route end. */
#define BEND_SPEED_MARGIN 0.8f /* Bend speeds are this part of `turn_rate` times the radius. */

/* `set_up_formation` prepares a squad for formation following when it gets its route. It moves
 * the start of the route to where the formation fits the members, and it shortens the route. With
 * `--centersearch on`, it takes a shorter route from the center of the members when there is one,
 * and with `--smooth on`, it smooths the route. It then measures `route_distance` and
 * `route_length`, matches the members to the slots so that they keep their arrangement, lays out
 * the lanes with `--lanes on`, and puts the anchor at the start of the route. Its searches and
 * ground tests go around parked units only while the caller stamps them with `stamp_parked`
 * around the call.
 * Inputs: `slots` holds the final slots of the formation as x, y pairs, one pair per member, and
 *   `facing` is the final facing of the formation.
 * Result: Each member gets its `slot`, `place_offset`, single-file rank and distance, and
 *   `start_offset`, and its movement state starts afresh. */
void set_up_formation(Squad *squad, const float *slots, V2 facing);

/* `begin_formation_set_up` and `finish_formation_set_up` do the work of `set_up_formation` in
 * parts, so that a caller can spread it over several ticks. Call `begin_formation_set_up` first,
 * with the same inputs. When it returns true, the smoothing of the route has begun, and the
 * caller runs `run_smoothing` until it returns true. Then call `finish_formation_set_up` with
 * the same `slots` and `facing`. Stamp the parked units around each call, as for
 * `set_up_formation`. The finish reads the positions of the members anew, at its own tick.
 * Result: `begin_formation_set_up` returns true when a smoothing has begun, and false with
 *   `--smooth off`. */
bool begin_formation_set_up(Squad *squad, const float *slots, V2 facing);
void finish_formation_set_up(Squad *squad, const float *slots, V2 facing);

/* `prepare_formation_route` and `begin_formation_smoothing` do the work of
 * `begin_formation_set_up` in its two parts, so that the second part can run on a worker thread.
 * Call `prepare_formation_route` on the main thread, with the parked units stamped: it calls code
 * of the game, which reads the grid of the game. `begin_formation_smoothing` uses only the cell
 * reads of the simulator, so it can run on any thread whose cell reads see the stamps (a grid
 * copy, game.h). It reads the members and the route of the squad, and it begins the smoothing of
 * the route on its own thread, where `run_smoothing` must then run. Use both only with
 * `--smooth on`, and finish with `finish_formation_set_up`, as above. */
void prepare_formation_route(Squad *squad, const float *slots, V2 facing);
void begin_formation_smoothing(Squad *squad, const float *slots, V2 facing);

/* `advance_anchor` moves the anchor of a moving squad by one tick, before the units of the squad
 * move. It first matches the members to the places again. With lanes on, it then switches single
 * file on or off, and single file also holds while `is_file_forced` is true. With
 * `--reformmatch on`, a squad that leaves single file reforms (`is_reforming`): its members are
 * matched to the places by the time to get back into formation until they have closed up. The
 * anchor drives
 * at the lowest top speed of the members. It drives slower while members lag behind their
 * places, through sharp turns with `--turnslow on`, and toward the end of the route, and never
 * faster than `coordination_speed_cap`. */
void advance_anchor(Squad *squad);

/* `get_file_distance` returns how far the place of a member is behind the anchor in single file.
 * With lanes on, it adds a gap to `file_distance` that grows with the speed of the anchor and
 * `file_rank`. */
float get_file_distance(const Squad *squad, const Unit *unit);

/* `get_place_distance` returns how far along the route the place of a member is. */
float get_place_distance(const Squad *squad, const Unit *unit);

/* `get_target_place` returns the place that a member drives to, and sets `forward` to the
 * direction of the formation there. The place is on the lane of the member, or in the fixed
 * formation shape without lanes, and it blends into the shape over the last `END_BLEND_METERS` of
 * the route. Over the first stretch of route that the anchor drives, it blends in from where the
 * member stood when the squad got its route.
 * Edge case: Without lanes, when blocked ground lies on the straight line from the anchor to the
 *   place in the shape, the place is the route point `file_distance` behind the anchor. */
V2 get_target_place(const Squad *squad, const Unit *unit, V2 *forward);

/* `project_onto_route` returns the distance along the route of the route point closest to
 * `point`, among the route points from `from_distance` to `to_distance` along it.
 * Edge case: When no route segment longer than 0 reaches into that range, it returns
 *   `from_distance`. */
float project_onto_route(const Squad *squad, V2 point, float from_distance, float to_distance);

/* steering.c defines this function of formation following. */

/* `steer_in_formation` steers `units[unit_index]` in formation mode: toward its place, along its
 * lane, back to its lane or the route, or along a detour from `follow_detour`. It also
 * accelerates the unit toward the speed it wants, and it marks the unit as stopped once it is on
 * its slot at the end of the route. It stores the point that the unit drives at in `waypoint`,
 * and the progress of the unit along the route in `route_progress`.
 * Result: It returns the steering, from -1 to 1, negative to the left. */
float steer_in_formation(int unit_index);

/* coordination.c defines these types, constants and functions of the coordination between
 * squads. */

#define HEAD_ON_MARGIN_METERS 4.0f /* Passing formations keep this gap between bodies. */
#define SAME_LINE_METERS 2.0f      /* Routes closer than this sideways keep to the right. */
#define ROUTE_SAMPLE_COUNT 128     /* A `RouteSamples` holds at most this many samples. */

/* A `FormationShape` gives the parts of the formation of a squad that coordination needs.
 * `measure_shape` fills it. */
typedef struct {
    float front_meters;  /* The bodies at their places reach this far past the anchor, in m. */
    float half_width;    /* Placed bodies reach at most this far to either side of the anchor. */
    float rear_progress; /* The rear of the bodies is this far along the route, in m. */
    float cruise_speed;  /* This is the lowest top speed of the members, in m/s. */
} FormationShape;

/* A `RouteSamples` holds a stretch of the route of a squad, which `sample_route_span` fills. It
 * has the points `points[0..count)`, their directions, with length 1, and their distances along
 * the route in meters. `is_shared[k]` marks the samples around the point where the squad meets
 * another one. */
typedef struct {
    V2 points[ROUTE_SAMPLE_COUNT];
    V2 directions[ROUTE_SAMPLE_COUNT];
    float distances[ROUTE_SAMPLE_COUNT];
    bool is_shared[ROUTE_SAMPLE_COUNT];
    int count;
} RouteSamples;

/* `measure_shape` returns the `FormationShape` of the squad. It measures the bodies of the
 * members at their places, and the rear at the progress of the members along the route. The
 * squad must have a route. */
FormationShape measure_shape(const Squad *squad);

/* `sample_route_span` samples the route of the squad from `start` to `end`, which are distances
 * along the route in meters. The first sample is at `start`, and the others follow at even steps
 * up to `end`, or until `samples` is full. It clears every `is_shared` mark. */
void sample_route_span(const Squad *squad, float start, float end, RouteSamples *samples);

/* `find_closest_sample_index` returns the index of the sample in `samples` closest to `point`, the
 * first one on a tie.
 * Edge case: It returns 0 when `samples` is empty. */
int find_closest_sample_index(const RouteSamples *samples, V2 point);

/* `measure_side_room` returns how far the lanes of the squad can shift toward `side`, +1 for right
 * and -1 for left, at `distance` along its route, before the outermost lane on that side reaches
 * its clamp. The result is in meters and never below 0. */
float measure_side_room(const Squad *squad, float distance, float side);

/* `coordinate_squads` sets the `coordination_speed_cap` of every squad for this tick, and moves
 * the `side_offset` of each moving squad. In `AVOID_COORD`, a squad gets a lower cap when it gives
 * way to another moving squad at a crossing, when it waits for an oncoming squad at a narrow
 * passage, or when it follows a squad on the same route (`pass_or_follow`). Every other
 * squad gets `LARGE_VALUE`, and so does a squad that gave up waiting. Squads that meet head-on,
 * or that pass each other, shift their lanes apart with `side_offset`. In `AVOID_FORM` every cap
 * stays `LARGE_VALUE`. Call it once per tick in both formation modes, before `update_squad`. */
void coordinate_squads(void);

/* shift.c defines this state and these functions of the shifts of the lanes around a meeting or a
 * pass. The wanted shift of a squad (`side_offset_target`) is the shift that `side_offset` moves
 * toward. The stretch of a shift is the part of the route on which the lanes shift by the full
 * `side_offset`, from `side_shift_start` to `side_shift_end`. A pair is two moving squads that
 * `coordinate_squads` checks against each other. */

/* `has_meeting[k]` is true when `pass_head_on` (coordination.c) found a head-on meeting of
 * `squads[k]` with another squad in this tick, whether the two shift to pass or take turns, or
 * when `set_pass_shift` set its shift for a pass. `coordinate_squads` clears it at the start of
 * each tick. */
extern bool has_meeting[MAX_SQUADS];

/* `is_pass_shift[k]` is true when `set_pass_shift` set the shift of `squads[k]` in this tick.
 * `coordinate_squads` clears it at the start of each tick. */
extern bool is_pass_shift[MAX_SQUADS];

/* `set_shift_target` sets the shift of the squad for a head-on meeting at `meeting_distance`
 * along its route. It sets `side_offset_target` to `target`, in meters and positive to the right,
 * and the stretch to a stretch of route around the meeting. `--lateshift on` and
 * `--shortreturn on` make that stretch shorter before and after the meeting. It changes nothing
 * of this when another pair already set a larger target in this tick. */
void set_shift_target(Squad *squad, float meeting_distance, float target);

/* `set_pass_shift` sets the shift of the squad for a pass in this tick. It sets
 * `side_offset_target` to `target`, in meters and positive to the right, and the stretch of the
 * full shift to the route from `start` to `end`, which are distances along the route of the squad.
 * It changes nothing of this when another pair already set a target at least as large in this
 * tick. With `--smoothsway on`, the shift of the pass then stays steady from tick to tick. */
void set_pass_shift(Squad *squad, float start, float end, float target);

/* `move_side_offset` moves `side_offset` of the squad one tick toward `side_offset_target`. With
 * `--smoothsway on`, the speed of the shift (`side_offset_rate`) builds up and dies down slowly
 * enough that the members can drive the sideways move of their lanes at top speed. Call it once
 * per tick for each moving squad. */
void move_side_offset(Squad *squad);

/* `get_side_shift` returns how far the lanes of the squad shift to the right at `distance` along
 * its route, in meters, to pass another squad. A negative result is a shift to the left. The
 * shift is `side_offset` from `side_shift_start` to `side_shift_end`, and it fades to 0 before
 * and after that stretch. */
float get_side_shift(const Squad *squad, float distance);

/* `keep_shift_steady` keeps the wanted shift of the squad and its stretch steady from tick to
 * tick, for `--smoothsway on`. Call it for each squad once every pair has set its shifts in this
 * tick.
 * Inputs: `last_target`, `last_start` and `last_end` are the wanted shift and stretch of the
 *   squad from the last tick.
 * Result: While the anchor of the squad is still before `last_end` and `last_target` is not 0,
 *   the squad keeps its last stretch. When no pair found a head-on meeting of the squad in this
 *   tick, it keeps its last wanted shift too. When a pair found a shift to the same side, it
 *   keeps the larger of the two shifts. A shift to the other side replaces the last one with its
 *   own stretch, and a squad that takes turns, which has a meeting but no shift, lets its shift
 *   go back to 0.
 *   A pass shift from `try_pass` (overtaking.c) to the same side keeps its own target instead.
 *   Its stretch only grows: it runs from the earlier of the two starts to the later of the two
 *   ends. */
void keep_shift_steady(Squad *squad, float last_target, float last_start, float last_end);

/* overtaking.c defines this function of the coordination between squads on one route. */

/* `pass_or_follow` checks whether two squads drive one behind the other on the same stretch of
 * route, as leader and follower. If they do, a faster follower passes where there is room, and
 * otherwise the follower keeps a gap behind the leader through its `coordination_speed_cap`.
 * Result: It returns true when the two squads drive on one stretch, so that the crossing rule
 *   leaves them out, and false otherwise. When the squads drive on one stretch, it also sets the
 *   shifts, caps and pass state (`passing_index`, `is_pass_kept`, `is_file_pass`,
 *   `is_pass_waiting` and `is_file_forced`) of the two squads for this tick, and it marks the
 *   leader in `is_following` of the follower. With `--keepfollow on`, a pair stays on one stretch
 *   whichever way the anchor of the leader moves, if the follower followed the leader in the
 *   last tick (`was_following`). */
bool pass_or_follow(Squad *first, Squad *second);

/* replan.c defines these functions of the detours. */

/* `reset_detour` drops the detour of the unit and starts its progress check afresh. Call it when
 * the squad of the unit gets its route. */
void reset_detour(Unit *unit);

/* `follow_detour` picks the point that a moving unit in formation mode drives at while it follows
 * a detour around obstacles. A unit gets a detour when it makes no progress for a while, it is
 * far from its place, and blocked ground lies on the straight line to `target`. The detour ends
 * when the unit reaches its last point, when the straight way to `target` is clear, or after a
 * time limit.
 * Inputs: `place` is the target place of the unit, and `place_distance` is how far along the route
 *   the place is. `target` is the point that formation following picked for the unit.
 * Result: It returns true while the unit follows a detour, and sets `target` to the detour point
 *   that the unit drives at. Otherwise it returns false and leaves `target` as it was. */
bool follow_detour(const Squad *squad, Unit *unit, V2 place, float place_distance, V2 *target);

#endif
