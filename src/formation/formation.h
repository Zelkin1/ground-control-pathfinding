// Formation following of the formation mode and the coordination mode (`--avoid form` and
// `--avoid coord`). At the order, the set-up of a squad prepares and smooths its route, assigns
// its slots and lays out its lanes. In each tick, the anchor of each moving squad advances along
// the route, and each squad member drives toward its slot on its lane. In the coordination mode,
// the squads also coordinate at crossings, head-on events and pass events.
//
// A route position is a distance along the route of a squad from the route start. A sideways
// position is positive to the right, as seen facing along the route.
#ifndef FORMATION_H
#define FORMATION_H

#include "sim.h"

#define LENGTH_SQUARED_EPSILON 1e-6f          // shortest squared length that has a direction
#define LARGE_VALUE 1e9f                      // larger than any distance, speed or cost
#define LANE_MAX_DISTANCE_METERS 30.0f        // longest free distance, farther terrain is free
#define LANE_MARGIN_METERS 1.5f               // margin of the clearance of a whole formation
#define ROUTE_EPSILON_METERS 1e-3f            // shortest route length that counts as nonzero
#define FREE_DISTANCE_STEP_METERS 0.5f        // step of the probes of a free distance

// Faction tuning (tuning.c): constants of formation following whose best values differ between
// the factions, because their unit types differ in size, braking and turn rate.
typedef struct {
    float brake_look_seconds;              // look time of the brake check
    float far_bend_look_seconds;           // bend look time, `--driving on`
    float heading_time_constant_seconds;   // time constant of proportional steering
    float pursuit_slot_meters;             // slot distance that starts a pursuit
    float pursuit_look_seconds;            // pursuit look time at the current speed
    float orca_margin_meters;              // extra gap between two bodies in the ORCA avoidance
    float squeezed_gap_meters;             // gap between squeezed lanes, on top of two radii
    float column_gap_radii;                // column gap in radii of the widest unit
    float corner_cut_margin_meters;        // band of a corner cut past the widest radius
    float lag_wait_meters;                 // larger lags make the anchor wait
} FactionTuning;

// `get_tuning` returns the constants for units of `unit_type`: those of its faction with
// `--factiontuning on`, else the values from before the tuning, alike for every faction.
const FactionTuning *get_tuning(const UnitType *unit_type);

// ORCA avoidance and step-aside moves (avoidance.c).

// Units that `compute_orca_velocity` keeps the unit away from.
enum {
    ORCA_AVOID_ALL = 0,                   // all units
    ORCA_AVOID_MOVING_MEMBERS = 1,        // moving squad members of the squad of the unit
};

// `compute_orca_velocity` returns the velocity closest to `preferred`, within the top speed of the
// unit `unit_index`, that keeps it away from the nearby units that `avoided_units` selects.
V2 compute_orca_velocity(int unit_index, V2 preferred, int avoided_units);

// `update_parked_boxes` fits the parked box of each squad that is not busy (`--parkedbox`), for
// `compute_orca_velocity`. It must run once per tick, before the units move.
void update_parked_boxes(void);

// TEMPORARY DEBUG: when set, `compute_orca_velocity` calls `orca_line_debug_hook` for each line
// that it builds, with the position of the other unit relative to the unit, the combined radius,
// how far the line moves the preferred velocity (0 or less when it does not), and the unit
// direction of that move. The mod sets it to trace cones of vehicles; the simulator leaves it NULL.
typedef void (*OrcaLineDebugHook)(int unit_index, int other_index, V2 relative_position,
                                  float combined_radius, float violation, V2 push_direction);
extern OrcaLineDebugHook orca_line_debug_hook;

// `step_aside_for_members` moves a stopped unit of a moving squad out of the way of the moving
// squad members of its squad. It sets the throttle and returns the steering input, and a unit
// that stepped aside drives back to where it stopped once it is out of the way.
float step_aside_for_members(int unit_index);

// Route geometry and route preparation (route.c).

// `get_route_point` returns the point of the route of a squad at the route position `distance`,
// and stores the direction of the route there in `direction`. Positions outside the route read
// the route start or the route end.
V2 get_route_point(const Squad *squad, float distance, V2 *direction);

// `get_extended_route_point` works as `get_route_point`, but the route continues straight along
// its first segment before its start and along its last segment past its end.
V2 get_extended_route_point(const Squad *squad, float distance, V2 *direction);

// `get_formation_direction` returns the formation direction of a squad at the route position
// `distance`, which turns smoothly at route corners.
V2 get_formation_direction(const Squad *squad, float distance);

// `measure_free_distance` returns the distance from `point` along the unit vector `direction` to
// blocking terrain of `terrain_class`, in steps of `FREE_DISTANCE_STEP_METERS`, or `max_distance`
// when the way is free that far.
float measure_free_distance(V2 point, V2 direction, int terrain_class, float max_distance);

// `measure_points_length` returns the length of the polyline through `points`.
float measure_points_length(const V2 *points, int point_count);

// `move_route_start` moves the start of the route of a squad from its front unit to where the
// shape of its final parking positions, laid along the first route segment, has its mean at
// `center`, or else to `center`. When neither point works, the route start stays.
void move_route_start(Squad *squad, const float *parking_positions, V2 facing, V2 center,
                      int terrain_class);

// `shorten_route` removes the route corners of a squad that a band of `max_radius` on each side
// of a straight line can skip, and then cuts the remaining corners.
void shorten_route(Squad *squad, float max_radius, int terrain_class);

// `take_shorter_centered_route` runs a centered game search from `center`, the mean position of
// the squad members, and replaces the route of the squad with its result when that is clearly
// shorter. `corner_gap` is the corner gap of the search.
void take_shorter_centered_route(Squad *squad, const float *parking_positions, V2 facing,
                                 V2 center, float max_radius, int terrain_class, float corner_gap);

// `add_lead_in_points` puts lead-in points after the route start of a squad: points on a turning
// circle of radius `turn_radius` that leaves the start along the unit vector `motion_direction`,
// up to where the circle turns toward the next route point. The curved route then leaves the start
// along the current motion. Without a turn whose band of `max_radius` on each side stays off
// blocking terrain, the route stays as it is.
void add_lead_in_points(Squad *squad, V2 motion_direction, float turn_radius, float max_radius,
                        int terrain_class);

// Route smoothing (smoothing.c). The smoothing state is per thread, so each worker thread can
// smooth one route at a time. At its end, the smoothing replaces the route of the squad with the
// curve positions of the curved route.

// `smooth_route` runs the whole route smoothing of a squad in one call. `max_radius` is the
// radius of the widest squad member and `half_width` is half the width of the formation.
void smooth_route(Squad *squad, float max_radius, float half_width, float full_speed_min_radius,
                  int terrain_class);

// `begin_smoothing` starts the route smoothing of a squad in this thread, with the inputs of
// `smooth_route`. Each call of `run_smoothing` then runs iterations until `max_clearance_tests`
// clearance tests have run, and returns true once the smoothing finished.
void begin_smoothing(Squad *squad, float max_radius, float half_width, float full_speed_min_radius,
                     int terrain_class);
bool run_smoothing(int max_clearance_tests);

// `get_clearance_test_count` returns the number of clearance tests that this thread ran so far,
// for the timing of the route smoothing.
int get_clearance_test_count(void);

// Lanes (lanes.c).

// Lane cross-section: the lanes of a squad at one route position. The lane of a slot with the
// sideways slot position `x` lies at `x * scale + offset`, limited by the lane clamp from `min_x`
// to `max_x`. In column formation, every lane lies at `middle`.
typedef struct {
    float scale;
    float offset;
    float middle;
    float min_x;
    float max_x;
    bool is_wide_enough;        // whether square formation fits here
} LaneCrossSection;

// `allocate_lanes` allocates the lane cross-sections of all squads, once at program start.
void allocate_lanes(void);

// `set_up_lanes` lays out the lane cross-sections of a squad over its whole route, keeping a
// squad member of `max_radius` clear of blocking terrain.
void set_up_lanes(Squad *squad, float max_radius);

// `get_lane_cross_section` returns the lane cross-section of a squad at the route position
// `distance`, interpolated between the stored ones.
LaneCrossSection get_lane_cross_section(const Squad *squad, float distance);

// `do_lanes_fit` tells whether square formation fits all along the route of a squad from
// `from_distance` to `to_distance`.
bool do_lanes_fit(const Squad *squad, float from_distance, float to_distance);

// `get_lane_point` returns the point of the lane of a squad member at the route position
// `distance`, with the lane shift. With `--lanes off`, it returns the route point.
V2 get_lane_point(const Squad *squad, const Unit *unit, float distance);

// Set-up, slots and anchor (formation.c).

#define END_BLEND_METERS 25.0f                // route end part where slots blend into parking
#define BEND_SPEED_MARGIN 0.8f                // share of the turn rate used at top speed

// `set_up_formation` runs the whole formation set-up of a squad in one call:
// `begin_formation_set_up`, then `run_smoothing` when that returns true, and then
// `finish_formation_set_up`. game/squad.c can also run these steps over several ticks or on
// worker threads.
void set_up_formation(Squad *squad, const float *parking_positions, V2 facing);

// `begin_formation_set_up` prepares the route of a squad and starts its route smoothing
// (`--smooth`). It returns true when a smoothing runs, which must finish before
// `finish_formation_set_up` assigns the slots and lays out the lanes.
bool begin_formation_set_up(Squad *squad, const float *parking_positions, V2 facing);
void finish_formation_set_up(Squad *squad, const float *parking_positions, V2 facing);

// `prepare_formation_route` moves the route start, shortens the route and tries the centered game
// search. `begin_formation_smoothing` starts the smoothing of the prepared route.
void prepare_formation_route(Squad *squad, const float *parking_positions, V2 facing);
void begin_formation_smoothing(Squad *squad, const float *parking_positions, V2 facing);

// Narrow formations of head-on events (`--narrowpass`). `can_narrow` tells whether the slots of a
// squad form a single row of three or more and the squad is not on the end blend of its route.
// `get_planned_slot` returns the slot position of a squad member that a head-on plan counts: its
// square slot with `is_narrow_planned`, on the route with `is_column_planned`. `narrow_formation`
// gives a squad its square slots, and its column formation when that is planned.
// `widen_formation` gives a squad its own slots back and clears the plans of the head-on.
bool can_narrow(const Squad *squad);
V2 get_planned_slot(const Squad *squad, const Unit *unit);
void narrow_formation(Squad *squad);
void widen_formation(Squad *squad);

// `advance_anchor` moves the anchor of a moving squad along its route for one tick. It also swaps
// slots and switches between square and column formation.
void advance_anchor(Squad *squad);

// `get_anchor_to_slot_distance` returns how far the slot of a squad member lies behind the anchor
// in column formation.
float get_anchor_to_slot_distance(const Squad *squad, const Unit *unit);

// `get_slot_distance` returns the route position of the slot of a squad member.
float get_slot_distance(const Squad *squad, const Unit *unit);

// `get_target_slot` returns the point that a squad member drives toward, its slot plus a start
// offset that fades out after the start, and stores the formation direction there in `forward`.
V2 get_target_slot(const Squad *squad, const Unit *unit, V2 *forward);

// `project_onto_route` returns the route position, from `from_distance` to `to_distance`, of the
// route point closest to `point`.
float project_onto_route(const Squad *squad, V2 point, float from_distance, float to_distance);

// Driving of the squad members (steering.c).

// `steer_in_formation` drives a squad member of a moving squad for one tick. It sets the throttle
// and returns the steering input.
float steer_in_formation(int unit_index);

// Coordination of the squads (coordination.c).

#define BODY_GAP_METERS 4.0f                  // gap between the bodies of passing formations
#define SAME_ROUTE_METERS 2.0f                // routes closer than this keep right
#define ROUTE_POSITION_COUNT 128              // most route positions of one coordination check

// Size of the formation of a squad for the coordination.
typedef struct {
    float front_meters;                // how far the bodies reach ahead of the anchor
    float half_width;                  // how far the bodies reach to the side of the route
    float rear_progress;               // route position of the rear end of the formation
    float squad_top_speed;
    float squad_base_top_speed;        // top speed of the squad without speed effects
    float own_half_width;              // half width in its own formation, without head-on plans
} FormationShape;

// Route positions of a squad, with the route point and the route direction at each.
typedef struct {
    V2 points[ROUTE_POSITION_COUNT];
    V2 directions[ROUTE_POSITION_COUNT];
    float distances[ROUTE_POSITION_COUNT];
    int count;
} RoutePositions;

// `measure_shape` measures the formation of a squad from the slots, the route progress and the
// unit types of its squad members.
FormationShape measure_shape(const Squad *squad);

// `fill_route_positions` fills `positions` with the route positions of a squad from `start` up to
// `end`, at a fixed step and at most `ROUTE_POSITION_COUNT` of them.
void fill_route_positions(const Squad *squad, float start, float end, RoutePositions *positions);

// `find_closest_position_index` returns the index of the route position closest to `point`, or 0
// when `positions` is empty.
int find_closest_position_index(const RoutePositions *positions, V2 point);

// `coordinate_squads` runs the coordination of all squads for one tick, before the anchors
// advance. It sets the speed cap and the shift of each squad.
void coordinate_squads(void);

// Lane shifts of head-on events and pass events (shift.c). The side of a shift is 1 for right
// and -1 for left, as seen facing along the route.

// Event that sets the shift of a squad, with a higher rank for a later value.
typedef enum {
    SHIFT_EVENT_NONE,
    SHIFT_EVENT_HEAD_ON,
    SHIFT_EVENT_PASS,
} ShiftEvent;

// Target shift of one squad in an event. The section is the part of the route with the full
// shift, and the magnitude is the size of the target shift.
typedef struct {
    Squad *squad;
    float side;
    float side_room;        // how far the lanes can move toward `side`
    float section_start;
    float section_end;
    float magnitude;
} TargetShift;

// `measure_side_room` returns how far all lanes of a squad can move toward `side` at the route
// position `distance` before the outermost lane reaches the lane clamp, without the current shift.
float measure_side_room(const Squad *squad, float distance, float side);

// `measure_min_side_room` returns the smallest side room of a squad toward `side` at the route
// positions `step` apart from `start` up to `end`, or zero without such a position.
float measure_min_side_room(const Squad *squad, float start, float end, float step, float side);

// `get_needed_gap` returns the sideways distance between the routes of two squads at which the
// bodies of their formations are `body_gap` apart.
float get_needed_gap(const FormationShape *first, const FormationShape *second, float body_gap);

// `choose_shift_side` returns the side of the shift of a squad, away from the other route at the
// sideways offset `other_route_offset`, or right when the routes are within `SAME_ROUTE_METERS`.
float choose_shift_side(float other_route_offset);

// `split_missing_gap_evenly` splits the missing gap into two equal shift magnitudes, or gives the
// rest to the other squad when one squad lacks the side room. The side rooms must cover the gap.
void split_missing_gap_evenly(float missing_gap, TargetShift *first, TargetShift *second);

// `split_missing_gap_in_order` gives the first squad as much of the missing gap as its side room
// allows, then the second squad as much of the rest. It returns the part that neither covers.
float split_missing_gap_in_order(float missing_gap, TargetShift *first, TargetShift *second);

// `clear_shift_target_and_event` clears the target shift and the event of a squad before the
// event checks of a tick.
void clear_shift_target_and_event(Squad *squad);

// `flag_shift_event` records `event` for a squad in the current tick, when it has a higher rank
// than the event that the squad already has.
void flag_shift_event(const Squad *squad, ShiftEvent event);

// `record_target_shifts` sets the target shifts and sections of both squads of an event, unless a
// squad already has a target shift of a larger magnitude, and flags the event for both.
void record_target_shifts(const TargetShift *first, const TargetShift *second, ShiftEvent event);

// `update_shift` adjusts the shift of a squad toward its target shift, once per tick, within the
// limits that let the squad members follow their lanes at top speed.
void update_shift(Squad *squad);

// `get_lane_shift` returns the sideways offset of the lanes of a squad at the route position
// `distance`: the full shift inside the section, with a smooth transition to zero outside it.
float get_lane_shift(const Squad *squad, float distance);

// `keep_shift_target_steady` combines the target shift and the section of a squad in this tick
// with those of the previous tick, so that they stay steady until the anchor leaves the section.
// With `is_replanning`, a head-on event takes the section of this tick after a top speed change
// (`--speedreplan`), and it keeps covering the previous one once the formation has reached it.
void keep_shift_target_steady(Squad *squad, float previous_target_shift,
                              float previous_section_start, float previous_section_end,
                              bool is_replanning);

// Pass events (passing.c).

// `pass_or_follow` makes the rear squad of two squads that drive the same way on one section of
// route pass the front squad or follow it. It returns false when the squads do not.
bool pass_or_follow(Squad *first, Squad *second);

// Detours of stuck units (detour.c).

// `reset_detour` clears the detour of a unit and restarts its progress check.
void reset_detour(Unit *unit);

// `follow_detour` starts a detour for a stuck unit, and on a detour replaces `target` with the
// next detour point. It returns false, and leaves `target` unchanged, without a detour. `slot` and
// `slot_distance` are the slot of the unit and its route position.
bool follow_detour(const Squad *squad, Unit *unit, V2 slot, float slot_distance, V2 *target);

#endif
