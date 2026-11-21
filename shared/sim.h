// Shared state and functions of the movement simulator: the vector helpers, the units and squads
// of the current run, the switches of the command line, and the functions of each source file.
//
// Map coordinates are in meters, with x to the east and y to the north. An angle counts clockwise
// from north, and a heading is the unit vector `(sin(angle), cos(angle))`. A frame offset gives a
// vector in the frame of a forward direction: x to its right and y along it.
#ifndef SIM_H
#define SIM_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "game.h"

// The time step of a tick. The game mod (mod/build.sh defines `GCMOD`) steps by the measured time
// of each game frame. The simulator steps by `GAME_FRAME_SECONDS` unless `--tick` sets another
// step, since the game moves its units once per frame, and some of its counters count frames.
extern float tick_seconds;
#define TICK_SECONDS tick_seconds
#define GAME_FRAME_SECONDS 0.0056f               // game frame time in the mod logs (5.3–5.8 ms)

// The game mod holds every unit and squad of a mission, so it replaces the small limits of the
// simulator. It uses no waypoint list of its own, because the game keeps one per mover.
#ifdef GCMOD
#define MAX_UNITS 512                            // most units of a mission that the mod tracks
#define MAX_SQUADS 128                           // most squads of a mission that the mod tracks
#define MAX_UNIT_WAYPOINTS 1                     // waypoints after the current one, per unit
#else
#define MAX_UNITS 48                             // most units of one run
#define MAX_SQUADS 8                             // most squads of one run
#define MAX_UNIT_WAYPOINTS MAX_WAYPOINTS         // waypoints after the current one, per unit
#endif
#define MAX_MEMBERS 16                           // most members of one squad (the game has 12)
#define MAX_WAYPOINTS 4096                       // most points of a route or a waypoint list
#define MAX_DETOUR_POINTS 16                     // most points of a detour
#define MAX_MOVE_AHEAD_STAMPS 128                // most stamp discs of a move-ahead re-route
#define PI 3.14159265f
#define ROUTE_END_EPSILON_METERS 0.01f           // margin before the route end that counts as it
#define NOT_STARTED_SECONDS -1.0f                // start time of a squad that did not start
#define STOP_REACH_METERS 7.0f                   // reach of the last waypoint after a stop
#define MID_REACH_METERS 1.0f                    // reach of the waypoints before the last one
#define WAIT_TIMER_AT_ORDER_SECONDS -3.0f        // wait timer of a unit after an order
#define NO_BLOCKER -1                            // blocker index without a blocker
#define NO_TARGET_UNIT -1                        // target unit index without a target unit

// Two-dimensional vector, a point or a direction on the map.
typedef struct {
    float x;
    float y;
} V2;


// `v2` builds a vector from its two components.
static inline V2 v2(float x, float y) {
    return (V2){
        .x = x,
        .y = y,
    };
}

// Sum, difference, scaling, dot product, 2D cross product, squared length and length.
static inline V2 add(V2 a, V2 b) { return v2(a.x + b.x, a.y + b.y); }
static inline V2 sub(V2 a, V2 b) { return v2(a.x - b.x, a.y - b.y); }
static inline V2 mul(V2 a, float factor) { return v2(a.x * factor, a.y * factor); }
static inline float dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
static inline float det2(V2 a, V2 b) { return a.x * b.y - a.y * b.x; }
static inline float length_squared(V2 a) { return dot(a, a); }
static inline float len(V2 a) { return sqrtf(length_squared(a)); }


// `get_right` returns the direction to the right of `forward`, as seen facing along it.
static inline V2 get_right(V2 forward) {
    return v2(forward.y, -forward.x);
}


// `get_direction` returns `vector` scaled to length 1, or `fallback` when its squared length is
// `min_length_squared` or less.
static inline V2 get_direction(V2 vector, float min_length_squared, V2 fallback) {
    if (length_squared(vector) > min_length_squared)
        return mul(vector, 1 / len(vector));
    return fallback;
}


// `get_frame_offset` returns the frame offset of a map vector for the unit vector `forward`.
static inline V2 get_frame_offset(V2 vector, V2 forward) {
    V2 right = get_right(forward);
    return v2(dot(vector, right), dot(vector, forward));
}


// `get_map_offset` returns the map vector of a frame offset for the unit vector `forward`, the
// reverse of `get_frame_offset`.
static inline V2 get_map_offset(V2 offset, V2 forward) {
    V2 sideways = mul(get_right(forward), offset.x);
    V2 along = mul(forward, offset.y);
    return add(sideways, along);
}


// `clamp_float` limits `value` to the range from `low` to `high`.
static inline float clamp_float(float value, float low, float high) {
    if (value < low)
        return low;
    return value > high ? high : value;
}


// `get_pair_point` returns the point `point_index` of an array of x, y pairs.
static inline V2 get_pair_point(const float *pairs, int point_index) {
    int x_index = 2 * point_index;
    return v2(pairs[x_index], pairs[x_index + 1]);
}


// `get_parking_offset` returns the frame offset of a final parking position from the route end
// `end`, for the final facing `facing`.
static inline V2 get_parking_offset(const float *parking_positions, int parking_index, V2 end,
                                    V2 facing) {
    V2 parking_from_end = sub(get_pair_point(parking_positions, parking_index), end);
    return get_frame_offset(parking_from_end, facing);
}


// `get_cell_coordinate` returns the cell coordinate of a map coordinate, rounded down, also for
// negative coordinates.
static inline int get_cell_coordinate(float world_meters) {
    float cells = world_meters / GC_CELL_METERS;
    int truncated = (int)cells;
    bool is_rounded_up = (float)truncated > cells;
    return is_rounded_up ? truncated - 1 : truncated;
}


// `make_cell_index` returns the cell index of the game for a cell, as `gc_cast_ray` returns it.
// `get_cell_x` and `get_cell_y` return the cell coordinates of a cell index.
static inline int make_cell_index(int cell_x, int cell_y) {
    return cell_y << GC_CELL_Y_SHIFT | cell_x;
}


static inline int get_cell_x(int cell_index) {
    return cell_index & GC_CELL_X_MASK;
}


static inline int get_cell_y(int cell_index) {
    return cell_index >> GC_CELL_Y_SHIFT;
}


// `get_closest_cell_point` returns the point of the cell `(cell_x, cell_y)` closest to `point`.
static inline V2 get_closest_cell_point(int cell_x, int cell_y, V2 point) {
    float closest_x = clamp_float(point.x, cell_x * GC_CELL_METERS, (cell_x + 1) * GC_CELL_METERS);
    float closest_y = clamp_float(point.y, cell_y * GC_CELL_METERS, (cell_y + 1) * GC_CELL_METERS);
    return v2(closest_x, closest_y);
}

// Linear congruential generator of the jitter of the seeds (scenarios.c), with the constants of
// the C standard example `rand`.
#define RANDOM_MULTIPLIER 1103515245u            // multiplier of the generator
#define RANDOM_INCREMENT 12345u                  // increment of the generator
#define RANDOM_SHIFT 8                           // low bits that a result drops
#define RANDOM_MASK 0xffff                       // bits of a result
#define RANDOM_RANGE 65536.0f                    // number of possible results


// `next_random` advances the generator state `state` and returns a number from 0 up to
// `RANDOM_RANGE - 1`.
static inline unsigned next_random(unsigned *state) {
    *state = *state * RANDOM_MULTIPLIER + RANDOM_INCREMENT;
    return (*state >> RANDOM_SHIFT) & RANDOM_MASK;
}

// Unit type, with the values of the unit files of the game (scenarios.c).
typedef struct {
    const char *name;
    float radius;
    float top_speed;
    float acceleration;
    float friction;
    float turn_rate;          // largest turn rate, in radians per second
    float close_range;        // distance of a close waypoint for the throttle
    int terrain_class;
    int units_per_squad;
    int faction;              // FACTION_CRAYVEN (0) unless set
} UnitType;

#define INFANTRY_TERRAIN_CLASS 2                 // `terrain_class` of foot units (ENT files)

// Unit of the current run. The first fields hold the state of the Mover of the game, the base
// game movement (docs/PATHFINDING.md), then the metrics, the formation following and the detours.
typedef struct {
    const UnitType *unit_type;
    int squad_index;
    int team;
    // Unit that the game drives this unit to, such as the APC that it boards or the unit that it
    // attacks, or `NO_TARGET_UNIT`. The mod reads it from the game; in the simulator, only the
    // squads of a scenario that board an APC have one (game/boarding.c).
    int target_unit_index;
    bool is_carried;             // inside the APC `target_unit_index` (simulator only)
    V2 position;
    V2 heading;
    V2 velocity;
    float base_top_speed;        // top speed of the unit type times the profile speed modifier
    float top_speed;             // base top speed with the speed effects of this tick
    float acceleration;          // acceleration of the unit type with the speed effects
    float angle;
    float last_turn_rate;        // signed turn rate of the last tick

    // Waypoint following and base game avoidance. The reach is the distance at which the unit
    // counts as at its current waypoint.
    V2 waypoint;                        // current base game waypoint
    V2 waypoints[MAX_UNIT_WAYPOINTS];   // waypoints after the current one, next first
    int waypoint_count;
    float reach;
    float reach_last;                   // reach of the last waypoint
    float reach_mid;                    // reach of the waypoints before the last one
    bool is_last_waypoint;
    bool is_stopped;
    bool is_reorienting;                // turns toward the waypoint before it drives
    int look_ahead_phase;               // which next waypoint the look-ahead tests
    int avoid_ticks;                    // ticks left of the current avoidance turn
    int avoidance_turn_side;
    int blocker_index;                  // index of the unit in the way, or `NO_BLOCKER`
    float wait_seconds;                 // wait timer, positive while the unit waits
    float side_cooldown_seconds;        // time until the avoidance turn side can flip
    V2 wait_heading;                    // heading that the unit keeps while it waits
    V2 facing;                          // final facing, or zero for none

    // Metrics (metrics.c).
    V2 start_position;
    V2 parking_position;                // final parking position
    bool was_fast;
    int stop_count;
    int wait_count;
    int own_bump_count;                 // bumps with units of its own squad
    int other_bump_count;               // bumps with units of other squads
    bool is_touching[MAX_UNITS];        // touch with each unit of a higher index
    float slow_seconds;
    float slow_run_seconds;             // time since the unit got slow
    float stuck_seconds;
    float travel_seconds;               // time heading to the final parking position
    float top_speed_seconds;
    float avoidance_turn_seconds;
    float turn_on_spot_seconds;
    float previous_angle;
    bool is_turning_on_spot;
    bool is_terrain_sliding;
    bool was_terrain_sliding;
    int terrain_slide_count;
    float in_terrain_seconds;
    bool is_avoidance_turning;
    bool is_in_contact;
    bool is_orca_avoiding;
    float orca_wait_seconds;            // time that the ORCA avoidance held the unit nearly still
    float avoidance_off_seconds;        // time left with the ORCA avoidance off
    float orca_avoidance_seconds;
    bool was_orca_avoiding;
    int avoidance_onset_count;

    // Formation following (formation/). A slot position is a frame offset from the anchor.
    bool has_stepped_aside;
    V2 slot_position;
    float anchor_to_slot_distance;
    int column_position;            // place in column formation, 0 at the front
    V2 start_offset;                // offset of the unit from its slot at the start
    float route_progress;           // route position of the unit

    // Detour and progress check (formation/detour.c).
    V2 detour[MAX_DETOUR_POINTS];
    int detour_count;
    int detour_index;        // next detour point
    float detour_seconds;
    V2 progress_check_position;
    float progress_check_seconds;
} Unit;

// States of a squad. The values are those of the game.
enum {
    SQUAD_IDLE = 0,
    SQUAD_MOVING = 1,
    SQUAD_ARRIVED = 2,
    SQUAD_DENIED = 3,        // the route search failed
    SQUAD_SEARCHING = 13,
};

// Squad of the current run. Its squad members are the `unit_count` units of `units` from
// `first_unit_index` on. A route position is a distance along `route` from its start.
typedef struct {
    const UnitType *unit_type;
    int squad_index;
    int first_unit_index;
    int unit_count;
    int formation_type;        // formation type of the game
    V2 destination;
    V2 facing;                 // facing of the order
    V2 start_position;         // position of the front unit at the order
    V2 last_facing;
    int state;

    // Route search, set-up and parking (game/squad.c and game/reroute.c). The set-up fields keep
    // the inputs of a set-up that runs over several ticks or on a worker thread.
    bool is_backward_search;
    bool is_rerouting;
    // Squads whose final parking positions the route of this squad avoids, by squad index.
    bool is_avoiding_parking[MAX_SQUADS];
    // Moving squads whose paths the route of this squad avoids (`--moveahead`), by squad index.
    bool is_avoiding_moving[MAX_SQUADS];
    // Whether the running re-route, from its search to the end of its set-up, goes around the
    // path of a moving squad, the predicted arrival time on the old route, which a trial route
    // must beat, and the discs that the search and the set-up stamp (`--moveahead`).
    bool is_move_ahead_reroute;
    float old_route_arrival_seconds;
    V2 move_ahead_stamp_centers[MAX_MOVE_AHEAD_STAMPS];
    int move_ahead_stamp_count;
    float move_ahead_stamp_radii[MAX_MOVE_AHEAD_STAMPS];              // during the route searches
    float move_ahead_set_up_stamp_radii[MAX_MOVE_AHEAD_STAMPS];       // during the set-up
    // Speed cap of the anchor that brings the squad to its predicted waits for the moving squads
    // only once those waits are over (`--moveahead`), from the last tick, and the squad that it
    // keeps adapting to until it would no longer wait for it, or `NO_SQUAD`.
    float yield_speed_cap;
    int yield_to_index;
    // Time of the last tick in which this squad yielded to each other squad, by squad index, or
    // `-LARGE_VALUE` when it has not yielded to it since its order. The other squad still counts
    // it as giving way for a short time after that (moving_ahead.c).
    float last_yield_seconds[MAX_SQUADS];
    bool is_settled;
    float start_time_seconds;
    int set_up_phase;
    float set_up_parking_positions[2 * MAX_MEMBERS];
    V2 set_up_facing;
    // Whether the final parking positions are in reverse order, so that the squad members keep
    // their sides after the facing turns around.
    bool is_flipped;
    GcRequest search;
    float straggler_seconds;        // time until the squad counts as arrived without stragglers
    float order_time_seconds;
    float arrive_time_seconds;

    // Metrics (metrics.c). The spread reference is the spread of the final formation.
    float spread_reference;
    float max_spread;
    float spread_sum;
    int spread_count;
    int crossing_count;        // pairs of squad members with crossing paths
    V2 route[MAX_WAYPOINTS];
    int route_count;
    int lead_in_point_count;        // lead-in points after the route start (`--smoothreorder`)

    // Route and anchor of formation following (formation/formation.c). `route_distance` holds the
    // route position of each route point.
    float route_distance[MAX_WAYPOINTS];
    float route_length;
    float anchor_distance;           // route position of the anchor
    float anchor_speed;
    float waited_seconds;            // time that the anchor waited nearly still for laggards
    float ignore_lag_seconds;        // time left in which the anchor ignores the lag
    float swap_seconds;              // time until the next slot swap
    int swap_count;
    bool has_final_swap;             // whether the slot swap of the end section is done
    V2 final_facing;
    float full_speed_min_radius;
    int cross_section_count;         // number of lane cross-sections
    bool is_in_column;
    bool is_reforming;

    // Coordination (formation/coordination.c).
    float speed_cap;
    float speed_cap_wait_seconds;          // time that the speed cap held the anchor nearly still
    float ignore_coordination_seconds;
} Squad;

#define NO_SQUAD -1                              // squad index without a squad


// `is_squad_busy` tells whether a squad moves or searches its route.
static inline bool is_squad_busy(const Squad *squad) {
    return squad->state == SQUAD_MOVING || squad->state == SQUAD_SEARCHING;
}


// `get_member_point` returns the final parking position of a squad member, or its position.
static inline V2 get_member_point(const Unit *member, bool is_at_parking_positions) {
    return is_at_parking_positions ? member->parking_position : member->position;
}


// `get_forward_share` returns the share of `direction`, with the length `direction_length`, that
// points along the heading of a unit, or zero for a direction behind it.
static inline float get_forward_share(const Unit *unit, V2 direction, float direction_length) {
    float alignment = dot(unit->heading, direction) / direction_length;
    return fmaxf(0, alignment);
}

// State of the current run (sim.c). `path_points` is a shared buffer for the paths of the base
// game path search, as x, y pairs, and `trace_file` is NULL without `--trace`.
extern Unit units[MAX_UNITS];
extern Squad squads[MAX_SQUADS];
extern int unit_count;
extern int squad_count;
extern float now_seconds;
extern float path_points[2 * MAX_WAYPOINTS];
extern FILE *trace_file;

// Movement mode of the run (`--avoid`): the original game mode, the formation mode or the
// coordination mode. `MODE_NAME` holds their names on the command line and in the output.
enum { AVOID_ORIGINAL, AVOID_FORM, AVOID_COORD };
extern int avoid_mode;
extern const char *MODE_NAME[];


// `is_formation_mode` tells whether the run uses formation following, in the formation mode or
// the coordination mode.
static inline bool is_formation_mode(void) {
    return avoid_mode != AVOID_ORIGINAL;
}

// Flags of the switches (switches.c, docs/SWITCHES.md), each on by default except
// `--setupaccumulate` and `--middlerotate`. `find_on_off_switch` returns the flag of the switch
// `name` (with its leading "--"), or NULL for any other name.
bool *find_on_off_switch(const char *name);
extern bool are_lanes_on;                   // `--lanes`
extern bool is_formation_keeping_on;        // `--formation`
extern bool is_smooth_on;                   // `--smooth`
extern bool is_wide_bend_on;                // `--widebends`
extern bool is_driving_on;                  // `--driving`
extern bool is_detour_on;                   // `--detour`
extern bool is_park_reroute_on;             // `--parkreroute`
extern bool is_park_robust_on;              // `--parkrobust`
extern bool is_set_up_accumulate_on;        // `--setupaccumulate`
extern bool is_full_search_on;              // `--fullsearch`
extern bool is_set_up_threads_on;           // `--setupthreads`
extern bool is_smoothing_stop_on;           // `--smoothstop`
extern bool is_tight_bend_on;               // `--tightbends`
extern bool is_faction_tuning_on;           // `--factiontuning`
extern bool is_smooth_reorder_on;           // `--smoothreorder`
extern bool is_friend_coordination_on;      // `--friendcoord`
extern bool are_boarders_ignored;           // `--ignoreboarders`
extern bool is_parked_push_on;              // `--parkedpush`
extern bool is_squad_friend_on;             // `--squadfriends`
extern bool is_game_layout_on;              // `--gamelayout`
extern bool is_free_parking_on;             // `--freeparking`
extern bool is_whole_parked_on;             // `--wholeparked`
extern bool is_move_ahead_on;               // `--moveahead`
extern bool is_late_yield_on;               // `--lateyield`
extern bool is_close_pass_on;               // `--closepass`
extern bool is_keep_order_on;               // `--keeporder` (mod only: group orders)
extern bool is_click_deselect_on;           // `--clickdeselect` (mod only: selection)
extern bool is_avoid_parked_on;             // `--avoidparked` (mod only: order routes)
extern bool is_middle_rotate_on;            // `--middlerotate` (mod only: camera)

// `is_boarding` tells whether `unit` drives to board the unit `carrier_index`, an APC: the game
// drives it to that unit, and both units are on one team, so it is no attack.
static inline bool is_boarding(const Unit *unit, int carrier_index) {
    if (unit->target_unit_index != carrier_index)
        return false;
    return unit->team == units[carrier_index].team;
}

// `is_squad_boarding` tells whether a member of `boarding` drives to board a member of `carrier`.
static inline bool is_squad_boarding(const Squad *boarding, const Squad *carrier) {
    int carrier_end_index = carrier->first_unit_index + carrier->unit_count;
    int end_index = boarding->first_unit_index + boarding->unit_count;
    for (int unit_index = boarding->first_unit_index; unit_index < end_index; unit_index++) {
        int target_index = units[unit_index].target_unit_index;
        bool is_carrier_member = target_index >= carrier->first_unit_index
                                 && target_index < carrier_end_index;
        if (is_carrier_member && is_boarding(&units[unit_index], target_index))
            return true;
    }
    return false;
}

// `are_squads_coordinated` tells whether the coordination between squads (head-on meetings,
// passing, crossings, re-routes around parked squads) acts between `first` and `second`. With
// `--ignoreboarders`, it does not act when one squad boards an APC of the other. With
// `--friendcoord`, it acts only between squads of one team; the ORCA avoidance between their units
// stays. A squad without members counts as a friend of every squad.
static inline bool are_squads_coordinated(const Squad *first, const Squad *second) {
    bool has_boarding = is_squad_boarding(first, second) || is_squad_boarding(second, first);
    if (are_boarders_ignored && has_boarding)
        return false;
    bool has_members = first->unit_count > 0 && second->unit_count > 0;
    if (!is_friend_coordination_on || !has_members)
        return true;
    int first_team = units[first->first_unit_index].team;
    int second_team = units[second->first_unit_index].team;
    return first_team == second_team;
}

// Base game movement (game/mover_steps.c, shared with the mod, and game/movement.c). A steering
// input goes from -1 to 1 of the turn rate, and a positive input turns clockwise.

// `steer_to` returns the steering input of the game toward `direction`.
float steer_to(const Unit *unit, V2 direction);

// `steer_to_facing` returns the steering input toward the final facing of a unit, or zero
// without one.
float steer_to_facing(const Unit *unit);

// `accelerate` speeds a unit up along its heading for one tick, with `throttle` times the
// acceleration of its unit type.
void accelerate(Unit *unit, float throttle);

// `accelerate_toward_speed` accelerates a unit so that its speed along its heading reaches
// `target_speed` in about `time_constant_seconds`, with the friction of the tick included.
void accelerate_toward_speed(Unit *unit, float target_speed, float time_constant_seconds);

// `get_corner_gap` returns the corner gap of a unit for the base game path search.
float get_corner_gap(const Unit *unit);

// `search_path` runs the base game path search of a unit to `destination` in one call, the
// forward search and then the backward search, each with `budget` expansions. It returns true
// when it found a path, which `search` then holds, and the caller frees `search` either way.
bool search_path(const Unit *unit, V2 destination, float tolerance, int budget,
                 GcRequest *search);

// `set_move` gives a unit the route of its squad as the game does. Its final parking position
// `parking_position` replaces the last route point, and `facing` becomes its final facing.
void set_move(Unit *unit, const V2 *route, int route_count, V2 facing, V2 parking_position);

// `stop_at` sends a unit straight to `point` and clears its other waypoints.
void stop_at(Unit *unit, V2 point);

// `tick_mover` moves a unit for one tick: waypoint following or formation following, then
// steering, turning, friction and the move with its collisions.
void tick_mover(int unit_index);

#define AVOID_TICKS 5                            // ticks of an avoidance turn

// Boarding of an APC (game/boarding.c, simulator only). A unit with a target unit drives straight
// at it and boards it within twice its radius. A carried unit rides inside it, out of the way of
// the other units.

// `is_chasing` tells whether a unit drives to its target unit and is not inside it yet.
bool is_chasing(const Unit *unit);

// `board_if_close` puts a chasing unit inside its target APC once it is close enough.
void board_if_close(Unit *unit);

// `aim_at_target` makes the target unit the waypoint of a chasing unit, in place of the waypoint
// following of the game.
void aim_at_target(Unit *unit);

// `ride_in_carrier` keeps a carried unit at the position of its APC, standing.
void ride_in_carrier(Unit *unit);

// Physics (game/physics.c).

// `turn` turns a unit for one tick with the steering input `steering`.
void turn(Unit *unit, float steering);

// `apply_friction` slows a unit by its friction for one tick and caps its speed at its top speed.
void apply_friction(Unit *unit);

// `integrate` moves the unit `unit_index` by its velocity for one tick and resolves its
// collisions with blocking terrain and other units.
void integrate(int unit_index);

// Stamping (formation/stamping.c). In the formation modes, the parked squads count as cliffs
// during the route search of a moving squad: `stamp_parked` sets their cells to `GC_CELL_CLIFF`
// in the grid of this thread, and `unstamp_parked` restores them. `stamp_parked_for_search` stamps
// them for a route search, which can stamp other discs of a move-ahead re-route than the set-up.
void stamp_parked(const Squad *moving_squad, V2 start, V2 end);
void stamp_parked_for_search(const Squad *moving_squad, V2 start, V2 end);
void unstamp_parked(void);

// `find_stopped_hull` writes into `hull` the convex hull of the positions of the stopped members
// of `parked` that no APC carries, counterclockwise, and returns its number of corners: at most
// `MAX_MEMBERS`, 0 without such members. `max_radius` gets the largest body radius among them.
int find_stopped_hull(const Squad *parked, V2 *hull, float *max_radius);

// `free_parking_positions` (`--freeparking`) moves each final parking position of `squad`, in
// `parking_positions` as x, y pairs, that overlaps the body of a stopped unit of a parked squad to
// the nearest spot clear of parked units, blocking terrain and its other final parking positions.
void free_parking_positions(const Squad *squad, float *parking_positions);

// `shift_parking_positions` (`--wholeparked`) moves all final parking positions of `squad`, in
// `parking_positions` as x, y pairs, by one shift when any of them comes within the body radius
// plus a margin of the convex hull of the stopped members of a parked squad: to the nearest spot
// where every body keeps that gap and stays clear of blocking terrain, so the squad keeps its
// formation. The last point of the route of `squad` moves with them, so the route must be set
// first; `destination` stays. Without such a spot within `WHOLE_PARKING_MAX_RINGS` rings, nothing
// moves.
void shift_parking_positions(Squad *squad, float *parking_positions);

// Smallest box of cells that holds every stamped cell of a thread. A worker thread that smooths a
// route on a grid copy takes over the stamp box of that copy with `set_stamp_box`.
typedef struct {
    bool has_cells;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
} StampBox;

StampBox get_stamp_box(void);
void set_stamp_box(StampBox box);

#define NO_STAMP_DISTANCE_CELLS 1000000          // stamp distance without stamped cells

// `get_stamp_distance_cells` returns the chessboard distance in cells from a cell to the box
// around the stamped cells of this thread, or `NO_STAMP_DISTANCE_CELLS` without stamps.
int get_stamp_distance_cells(int cell_x, int cell_y);

// `hash_stamped_cells` returns a hash of the stamped cells of this thread, which changes whenever
// the stamps change.
uint32_t hash_stamped_cells(void);

// Squads (game/squad.c).
// `order_move` orders a squad to `destination` and starts its route search.
void order_move(Squad *squad, V2 destination);

// `start_route_search` starts the forward search of a squad from its front unit to its
// destination.
void start_route_search(Squad *squad);

// `update_squad` updates a squad for one tick: its route search and set-up, or its move and its
// arrival.
void update_squad(Squad *squad);

// Re-routes around parked squads (game/reroute.c). `reroute_around_parked` makes the moving
// squads whose routes go past `parked` search their routes again. `avoid_parking_ahead` makes a
// squad re-route around the final parking positions of a squad that will park ahead of it.
// `update_settled` marks a squad at its route end as settled once its squad members stand.
void reroute_around_parked(const Squad *parked);
void avoid_parking_ahead(Squad *squad);
void update_settled(Squad *squad);

// Adaptation to the paths of moving squads (game/moving_ahead.c, `--moveahead`), after the anchor
// of a moving squad moved. `avoid_moving_ahead` makes the squad, and in a head-on meeting the
// other squad too, search its route again around the path of a moving squad that it would wait
// long for, when a trial search finds a better route, or else sets `yield_speed_cap`.
void avoid_moving_ahead(Squad *squad);

// `get_closest_segment_point` returns the point of the segment from `a` to `b` closest to `point`
// (formation/stamping.c).
V2 get_closest_segment_point(V2 point, V2 a, V2 b);

// `reset_set_up_work` refills the work budget of the set-ups of `--setupaccumulate` for a new
// tick, before the squads update.
void reset_set_up_work(void);

// `finish_threaded_set_ups` waits for the route smoothings of the worker threads
// (`--setupthreads`), and then finishes their set-ups, after the squads update.
void finish_threaded_set_ups(void);

// Maps (maps.c). `maps_dir` holds the extracted missions, and `map_name` the name of the loaded
// map.
#define MAP_NAME_SIZE 64                         // longest map name, with its terminator

extern const char *maps_dir;
extern char map_name[MAP_NAME_SIZE];

// `load_map` loads the map `name` into the path finder. The program ends when it cannot.
void load_map(const char *name);

// `select_map_clearance` makes `name` the current map, after its grid went into the path finder,
// and selects its clearance fields. The game mod calls it after the game loads a map.
void select_map_clearance(const char *name);

// `is_circle_clear` tells whether the circle of `radius` around `point` holds no blocking cell.
bool is_circle_clear(V2 point, float radius, int terrain_class);

// `is_line_clear` tells whether the zero-width ray of the game from `from` to `to` meets no
// blocking cell.
bool is_line_clear(V2 from, V2 to, int terrain_class);

// `is_way_clear` tells whether the line from a unit to `point` meets no cell that blocks it.
bool is_way_clear(const Unit *unit, V2 point);

// `get_clearance_meters` returns a distance from any point of a cell within which no cell blocks
// `terrain_class`, or zero when nothing is known.
float get_clearance_meters(int cell_x, int cell_y, int terrain_class);

// `find_spots` measures `try_count` random routes on the map `name` and prints them (`--find`).
void find_spots(const char *name, int try_count);

// Unit types, factions and armor profiles (units.c). The four Crayven unit types name the roles of
// the scenarios. `PROFILE_SPEED_MODIFIER` scales the top speed of the unit type for each profile.
enum { FACTION_CRAYVEN, FACTION_ORDER, FACTION_PHOENIX, FACTION_COUNT };
enum { PROFILE_BALANCED, PROFILE_DEFENSIVE, PROFILE_OFFENSIVE, PROFILE_SPEED, PROFILE_COUNT };

extern const char *FACTION_NAME[FACTION_COUNT];
extern const char *PROFILE_NAME[PROFILE_COUNT];
extern const float PROFILE_SPEED_MODIFIER[PROFILE_COUNT];

extern const UnitType MEDIUM_TANK;
extern const UnitType LIGHT_TANK;
extern const UnitType HEAVY_TANK;
extern const UnitType MARINE;
extern const UnitType COMMAND_APC;

// `get_faction_unit_type` returns the unit type of `faction` in the role of the Crayven unit type
// `unit_type`, or `unit_type` itself for any other unit type.
const UnitType *get_faction_unit_type(const UnitType *unit_type, int faction);

// Scenarios (scenarios.c), by scenario index. `SCENARIO_RUN_ORDER` lists the scenario indexes in
// the order of a run of all scenarios. A zero facing in `order_facings` means none. A squad gets
// its order in the first tick, or once the run reaches `order_seconds` when that is positive. A
// squad with a second order gets it, as a new move order, once the run reaches
// `second_order_seconds`.
#define SCENARIO_COUNT 17                        // number of scenarios

// Variants of the scenarios, which the command line sets before the runs: the faction of every
// squad, the most squad members of a squad (0 for the full squad size), the layout of every squad,
// square or line, the armor profile of every unit, and the speed booster: its start time, the
// length of its on and off times when it repeats, and the delay of its start per squad index. A
// negative start time means a boost that began before the order, so that it ends earlier.
enum { LAYOUT_SQUARE, LAYOUT_LINE, LAYOUT_COUNT };

#define NO_BOOST -1e9f                           // `boost_start_seconds` without a boost
#define NO_BOOST_CYCLE 0.0f                      // `boost_cycle_seconds` of a single boost

extern const char *LAYOUT_NAME[LAYOUT_COUNT];
extern int scenario_faction;
extern int max_member_count;
extern int scenario_layout;
extern int scenario_profile;
extern float boost_start_seconds;
extern float boost_cycle_seconds;
extern float boost_stagger_seconds;

// `update_speed_effects` sets the top speed and the acceleration of every unit for the current
// tick: its base values, times the speed booster while its squad has one (`--boost`,
// `--boostcycle`, `--booststagger`).
void update_speed_effects(void);

extern const char *SCENARIO_NAME[SCENARIO_COUNT];
extern const char *SCENARIO_DESCRIPTION[SCENARIO_COUNT];
extern const int SCENARIO_RUN_ORDER[SCENARIO_COUNT];

extern V2 order_destinations[MAX_SQUADS];
extern V2 order_facings[MAX_SQUADS];
extern bool has_order[MAX_SQUADS];
extern float order_seconds[MAX_SQUADS];
extern V2 second_order_destinations[MAX_SQUADS];
extern float second_order_seconds[MAX_SQUADS];
extern bool has_second_order[MAX_SQUADS];

// A move order that the squad `squad_index` gets, as a new order, once the run reaches `seconds`.
// A scenario lists any number of them up to `MAX_TIMED_ORDERS`, after the order of each squad.
typedef struct {
    int squad_index;
    float seconds;
    V2 destination;
    bool is_given;
} TimedOrder;

#define MAX_TIMED_ORDERS 32                      // most timed orders of a scenario
extern TimedOrder timed_orders[MAX_TIMED_ORDERS];
extern int timed_order_count;

// `set_up_scenario` loads the map of a scenario and adds its squads and orders, with the jitter
// of `seed`, or none for 0.
void set_up_scenario(int scenario_index, unsigned seed);

// Metrics and trace (metrics.c).

// `measure` adds the current tick to the metrics of the units and squads.
void measure(void);

// `print_result_header` prints the column names of the result rows.
void print_result_header(void);

// `begin_trace_run` starts the trace object of a run, `write_trace_frame` writes the frame of the
// current tick, `report_squad` writes the result row of a squad and its trace object, and
// `write_trace_grid` writes the grid around the area of the run.
void begin_trace_run(int scenario_index);
void write_trace_frame(bool is_first);
void report_squad(int scenario_index, int squad_index);
void write_trace_grid(void);

// `read_clock_seconds` returns the time of the performance counter (timing.c). Only differences
// of two readings have a meaning.
double read_clock_seconds(void);

// Worker threads (workers.c). `submit_job` adds a job for a worker thread, and `wait_for_jobs`
// runs the unclaimed jobs on the calling thread and waits until every submitted job finished.
typedef void WorkerJob(void *argument);

void submit_job(WorkerJob *job, void *argument);
void wait_for_jobs(void);

#endif
