/* sim.h holds the types, constants, state and functions that the parts of the offline traffic
 * simulator share.
 *
 * Files: each source file covers one part. The paths below are relative to sim/src. sim.h
 * declares the functions of each part, except that formation/formation.h declares those of the
 * files in formation/, and game/game.h declares those of game/game.c.
 * - sim.c reads the command line and runs the scenarios.
 * - maps.c loads the maps of the game and tests the grid for the other parts.
 * - find.c runs `--find`, which measures random routes of the search of the game on a map.
 * - scenarios.c holds the unit types and the test scenarios.
 * - metrics.c holds the metrics, the printed results and the JSON trace.
 * - timing.c reads the wall clock that times each tick.
 * - game/ holds the rebuilt movement of the game itself and the calls into gc.exe:
 *   - game/movement.c reconstructs the unit movement: Mover, Steer and AvoidUnits.
 *   - game/physics.c reconstructs the motion: Turn, Friction, Integrate and Physics.
 *   - game/squad.c reconstructs the squad movement: the orders, the search, the set-up of a move
 *     and the arrival. In formation mode it also makes the searches and the formation layouts go
 *     around parked squads.
 *   - game/game.c calls the real gc.exe functions: the path search, the raycast, the cell lookup
 *     and the formation layout.
 * - formation/ holds formation following and the coordination between squads:
 *   - formation/avoidance.c holds the new unit avoidance (ORCA) and the moves of units that make
 *     room.
 *   - formation/route.c holds the squad route of formation following: it moves the start of the
 *     route, shortens and smooths the route, and reads points and directions along it.
 *   - formation/lanes.c holds the lanes of formation following, one beside the route for each
 *     member.
 *   - formation/formation.c holds the rest of formation following: the places, the matching of
 *     members to places and the anchor.
 *   - formation/steering.c drives each member in formation following toward its place, along its
 *     lane or onto its slot.
 *   - formation/coordination.c makes moving squads in formation mode give way at crossings, keep
 *     to their right when they meet head-on, and take turns where they cannot pass.
 *   - formation/shift.c sets, moves and fades the shifts of the lanes around a meeting or a pass.
 *   - formation/overtaking.c lets a faster squad pass a slower one on the same route, or follow
 *     it.
 *   - formation/replan.c gives a detour to a unit in formation mode that makes no progress.
 *
 * Terms: these words have one meaning in every part.
 * - A tick is one step of the simulation, `TICK_SECONDS` long.
 * - World units are meters. Positions, offsets and distances are in world units, and speeds are
 *   in world units per second.
 * - A squad is a group of units of one kind that get their orders together. Its units are its
 *   members. The lead is the member closest to the destination at the order.
 * - The search of the game finds a path in one of two ways. The quiet search works forward from
 *   the start, and the active search works backward from the destination.
 * - The squad route is the path that the search of the game finds from the lead to the
 *   destination. In formation mode, `set_up_formation` moves its start and reshapes it.
 * - A slot is the final position of a member at the end of the route. The formation layout of the
 *   game lays out the slots.
 * - A unit is parked when it stopped and its squad neither moves nor searches.
 * - The Mover is the object of the game that moves one unit. Its waypoints are the points that the
 *   unit drives to, one after the other. A leg is the stretch to the current waypoint, and the
 *   last leg ends on the slot. A waypoint counts as reached within its reach, a distance in
 *   world units.
 * - Game mode is `AVOID_GAME`, and the formation modes are `AVOID_FORM` and `AVOID_COORD`. The
 *   comment of `avoid_mode` describes them.
 * - In formation following, the anchor is a virtual leader that moves along the squad route.
 *   Each member drives toward its place, a point that moves with the anchor. With lanes on, the
 *   place moves along the lane of the member, a line beside the route at the sideways offset of
 *   the slot.
 *   Where the lanes do not fit side by side, the squad drives in single file down the middle of
 *   the passage.
 * - The clamp of a lane limits its sideways offset from the route, so that the body of the widest
 *   member keeps clear of blocked ground.
 * - A shift moves all lanes of a squad sideways over a stretch of its route, where the squad
 *   meets or passes another squad. The shift is `side_offset` of the `Squad`, in meters, and it is
 *   positive to the right.
 * - When two squads drive one behind the other on the same stretch of route, the leader is the
 *   squad ahead and the follower is the squad behind. A faster follower can pass the leader.
 * - A detour is a way around obstacles that the search of the game finds from the position of a
 *   unit in formation mode. A unit can get one when its progress check finds that it makes no
 *   progress.
 * - The new avoidance is ORCA (optimal reciprocal collision avoidance) in avoidance.c. In the
 *   formation modes, it changes the velocities of the units so that they do not touch. */
#ifndef SIM_H
#define SIM_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "game.h"

#define TICK_SECONDS 0.04f /* A tick is this many seconds, the fixed step of the game. */

#define MAX_UNITS 48      /* A run has at most this many units. */
#define MAX_SQUADS 8      /* A run has at most this many squads. */
/* Waypoint lists, squad routes and found paths hold `MAX_WAYPOINTS` points. A smoothed squad
 * route of up to 8 km at its sample spacing of 4 m (route.c) fits, with room for samples that
 * the rounding of each curve piece adds. */
#define MAX_WAYPOINTS 4096
#define MAX_DETOUR_POINTS 16 /* The detour of a unit holds this many points. */
#define PI 3.14159265f    /* This is pi, rounded to float precision. */

#define ROUTE_END_EPSILON_METERS 0.01f /* An anchor this close to the route end is at the end. */
#define NOT_STARTED_SECONDS -1.0f      /* `start_time_seconds` is this until the move starts. */

/* Units start with these Mover values, and the code of the game sets them again. */
#define STOP_REACH_METERS 7.0f            /* Units that stand or must stop get this reach. */
#define MID_REACH_METERS 1.0f             /* Legs before the last one get this reach. */
#define WAIT_TIMER_AT_ORDER_SECONDS -3.0f /* A new order sets `wait_seconds` to this. */
#define NO_BLOCKER -1                     /* A unit without a blocker has this `blocker_index`. */

/* A `V2` is a 2D vector: a point or an offset in world units, or a velocity in world units per
 * second. */
typedef struct {
    float x;
    float y;
} V2;

/* ------------------------------------------------------------------------------------------------
 * `v2` returns the vector (`x`, `y`).
 * --------------------------------------------------------------------------------------------- */
static inline V2 v2(float x, float y) {
    return (V2){
        .x = x,
        .y = y,
    };
}

/* ------------------------------------------------------------------------------------------------
 * These helpers do 2D vector math. `dot` returns the dot product, and `det2` returns the cross
 * product, which is positive when `b` points to the left of `a`. `len` returns the length of a
 * vector, and `length_squared` returns the square of that length.
 * --------------------------------------------------------------------------------------------- */
static inline V2 add(V2 a, V2 b) { return v2(a.x + b.x, a.y + b.y); }
static inline V2 sub(V2 a, V2 b) { return v2(a.x - b.x, a.y - b.y); }
static inline V2 mul(V2 a, float factor) { return v2(a.x * factor, a.y * factor); }
static inline float dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
static inline float det2(V2 a, V2 b) { return a.x * b.y - a.y * b.x; }
static inline float length_squared(V2 a) { return dot(a, a); }
static inline float len(V2 a) { return sqrtf(length_squared(a)); }

/* ------------------------------------------------------------------------------------------------
 * `get_right` returns `forward` turned 90 degrees to the right.
 * --------------------------------------------------------------------------------------------- */
static inline V2 get_right(V2 forward) {
    return v2(forward.y, -forward.x);
}

/* ------------------------------------------------------------------------------------------------
 * `get_direction` returns `vector` scaled to length 1, or `fallback` when the squared length of
 * `vector` is `min_length_squared` or less.
 * --------------------------------------------------------------------------------------------- */
static inline V2 get_direction(V2 vector, float min_length_squared, V2 fallback) {
    if (length_squared(vector) > min_length_squared)
        return mul(vector, 1 / len(vector));
    return fallback;
}

/* ------------------------------------------------------------------------------------------------
 * `get_frame_offset` returns `vector` in the frame of `forward`, which must have length 1: x is
 * its part to the right of `forward`, and y is its part along `forward`.
 *
 *       ^ forward: y
 *       |    * vector
 *       |   /
 *       |  /
 *       | /
 *       o--------> right of forward: x
 * --------------------------------------------------------------------------------------------- */
static inline V2 get_frame_offset(V2 vector, V2 forward) {
    V2 right = get_right(forward);
    return v2(dot(vector, right), dot(vector, forward));
}

/* ------------------------------------------------------------------------------------------------
 * `get_map_offset` is the reverse of `get_frame_offset`. It returns the map vector whose part to
 * the right of `forward` is `offset.x` and whose part along `forward` is `offset.y`.
 * --------------------------------------------------------------------------------------------- */
static inline V2 get_map_offset(V2 offset, V2 forward) {
    V2 sideways = mul(get_right(forward), offset.x);
    V2 along = mul(forward, offset.y);
    return add(sideways, along);
}

/* ------------------------------------------------------------------------------------------------
 * `clamp_float` returns `value` limited to the range from `low` to `high`.
 * Edge case: When `low` is more than `high`, values below `low` give `low`, and other values give
 *   `high`.
 * --------------------------------------------------------------------------------------------- */
static inline float clamp_float(float value, float low, float high) {
    if (value < low)
        return low;
    return value > high ? high : value;
}

/* ------------------------------------------------------------------------------------------------
 * Some arrays hold points as x, y pairs: point k has its x at index `2 * k` and its y at index
 * `2 * k + 1`. `get_pair_point` returns point `point_index` of such an array.
 * --------------------------------------------------------------------------------------------- */
static inline V2 get_pair_point(const float *pairs, int point_index) {
    int x_index = 2 * point_index;
    return v2(pairs[x_index], pairs[x_index + 1]);
}

/* ------------------------------------------------------------------------------------------------
 * `get_slot_offset` returns slot `slot_index` of `slots`, which holds x, y pairs, as an offset
 * from `end`, the end of the route, with x to the right of `facing` and y along it. `facing` must
 * have length 1.
 * --------------------------------------------------------------------------------------------- */
static inline V2 get_slot_offset(const float *slots, int slot_index, V2 end, V2 facing) {
    V2 slot_from_end = sub(get_pair_point(slots, slot_index), end);
    return get_frame_offset(slot_from_end, facing);
}

/* ------------------------------------------------------------------------------------------------
 * `get_cell_coordinate` returns the column or row of the grid cell that holds the world
 * coordinate `world_meters`, as `gc_get_cell` takes it. It rounds down, so negative coordinates
 * give negative cells, which lie outside the grid.
 * It rounds down by hand, because floorf of MinGW is slow: the cast rounds toward zero, which is
 * one too high for a negative value with a fraction. That gives the floor of every float in the
 * range of `int`.
 * --------------------------------------------------------------------------------------------- */
static inline int get_cell_coordinate(float world_meters) {
    float cells = world_meters / GC_CELL_METERS;
    int truncated = (int)cells;
    bool is_rounded_up = (float)truncated > cells;
    return is_rounded_up ? truncated - 1 : truncated;
}

/* ------------------------------------------------------------------------------------------------
 * `make_cell_index` returns the index of the cell in column `cell_x` and row `cell_y`, in the
 * format of the game that game.h describes.
 * --------------------------------------------------------------------------------------------- */
static inline int make_cell_index(int cell_x, int cell_y) {
    return cell_y << GC_CELL_Y_SHIFT | cell_x;
}

/* ------------------------------------------------------------------------------------------------
 * `get_cell_x` returns the column of the cell at `cell_index`.
 * --------------------------------------------------------------------------------------------- */
static inline int get_cell_x(int cell_index) {
    return cell_index & GC_CELL_X_MASK;
}

/* ------------------------------------------------------------------------------------------------
 * `get_cell_y` returns the row of the cell at `cell_index`.
 * --------------------------------------------------------------------------------------------- */
static inline int get_cell_y(int cell_index) {
    return cell_index >> GC_CELL_Y_SHIFT;
}

/* ------------------------------------------------------------------------------------------------
 * `get_closest_cell_point` returns the point of cell (`cell_x`, `cell_y`) that is closest to
 * `point`.
 * --------------------------------------------------------------------------------------------- */
static inline V2 get_closest_cell_point(int cell_x, int cell_y, V2 point) {
    float closest_x = clamp_float(point.x, cell_x * GC_CELL_METERS, (cell_x + 1) * GC_CELL_METERS);
    float closest_y = clamp_float(point.y, cell_y * GC_CELL_METERS, (cell_y + 1) * GC_CELL_METERS);
    return v2(closest_x, closest_y);
}

/* `next_random` uses these values. They are those of the linear congruential generator of the
 * example `rand` in the C standard. */
#define RANDOM_MULTIPLIER 1103515245u /* Each step multiplies the state by this. */
#define RANDOM_INCREMENT 12345u       /* Each step then adds this. */
#define RANDOM_SHIFT 8                /* The result leaves out this many low bits of the state. */
#define RANDOM_MASK 0xffff            /* This mask keeps the 16 bits of the result. */
#define RANDOM_RANGE 65536.0f         /* `next_random` returns values below this. */

/* ------------------------------------------------------------------------------------------------
 * `next_random` advances `state` and returns the next pseudo-random number, from 0 to
 * `RANDOM_RANGE - 1`. The same start value of `state` always gives the same numbers.
 * --------------------------------------------------------------------------------------------- */
static inline unsigned next_random(unsigned *state) {
    *state = *state * RANDOM_MULTIPLIER + RANDOM_INCREMENT;
    return (*state >> RANDOM_SHIFT) & RANDOM_MASK;
}

/* A `Kind` is a unit type. Its values are the fields of the .ENT files of the game, and the
 * comment of each field names its .ENT field. Steer, the steering of the game, stops steering
 * toward a waypoint within the close range (`close_range`) that is not ahead of the motion of the
 * unit, while the unit neither re-orients nor avoids. */
typedef struct {
    const char *name;    /* The result rows and the trace show this name. */
    float radius;        /* Radius is the radius of the body of the unit, in m. */
    float max_speed;     /* MaxSpeed is the top speed, in m/s. */
    float acceleration;  /* Acceleration is the speed gain per second at full throttle. */
    float friction;      /* Friction is the speed loss per second. */
    float turn_rate;     /* TurnSpeed is the turn rate in radians per second. */
    float close_range;   /* TurnOffset is the close range of Steer, in m. */
    int mover_class;     /* SlopeBorder is the mover class. Cells above it block the unit. */
    int units_per_squad; /* MaxUnits is the number of units in a squad. */
} Kind;

/* A `Unit` holds one unit: its Mover state as the game keeps it, its metrics, and its state in
 * formation mode. */
typedef struct {
    const Kind *kind;
    int squad_index;             /* This is the index of the squad of the unit in `squads`. */
    int team;                    /* Units of different teams are enemies. */
    V2 position;
    V2 heading;                  /* The heading has length 1 and is (sin `angle`, cos `angle`). */
    V2 velocity;
    float angle;                 /* The angle is in radians, from 0 to 2 * PI. */
    float last_turn_rate;        /* It is the turn of the last tick, in rad/s. */

    /* These fields are the Mover state. */
    V2 waypoint;                 /* The unit drives to this waypoint now. */
    V2 waypoints[MAX_WAYPOINTS]; /* `waypoints[0..waypoint_count)` follow `waypoint`. */
    int waypoint_count;
    float reach;                 /* This is the reach of the current waypoint. */
    float reach_last;            /* `reach` takes this value on the last leg. */
    float reach_mid;             /* `reach` takes this value on the other legs. */
    bool is_last_leg;            /* It is true on the last leg of a move, ending on the slot. */
    bool is_stopped;             /* It is true once the unit reached the end of its move. */
    bool is_reorienting;         /* The unit turns toward its waypoint without accelerating. */
    int look_ahead_phase;        /* The unit looks ahead in phases 0 to 2 and re-paths in 3. */
    int avoid_ticks;             /* A swerve or wall slide sets it, and each tick takes 1 off. */
    int swerve_side;             /* AvoidUnits swerves to the left at 0 and to the right at 1. */
    int blocker_index;           /* This is the index of the unit that AvoidUnits avoids. */
    float wait_seconds;          /* The unit waits while this is above 0. It counts down. */
    float side_cooldown_seconds; /* Wall slides flip `swerve_side` only while this is below 0. */
    V2 wait_heading;             /* A waiting unit steers to this heading. */
    V2 facing;                   /* A standing unit turns to this direction, unless it is zero. */

    /* These fields hold the metrics and the state of the new avoidance. A bump is the tick at
     * which two bodies start to touch. */
    V2 start_position;           /* The unit stood here at the start of the run. */
    V2 slot;                     /* This is the slot of the unit, its end position of the move. */
    bool was_fast;               /* The unit drove fast after its last stop. */
    int stop_count;              /* This counts the stops after the unit drove fast. */
    int wait_count;              /* This counts the waits of AvoidUnits for a faster blocker. */
    int own_bump_count;          /* This counts bumps with units of the same squad. */
    int other_bump_count;        /* This counts bumps with units of other squads. */
    bool is_touching[MAX_UNITS]; /* Entry k is true if it touches unit k, for k above its index. */
    float slow_seconds;          /* The unit was slow for this long on its way to its slot. */
    float slow_run_seconds;      /* This is how long the unit is slow now without a break. */
    float stuck_seconds;         /* This is the slow time past the first seconds of slow runs. */
    float travel_seconds;        /* The unit drove toward its slot for this long. */
    float top_speed_seconds;     /* The unit drove that way at top speed for this long. */
    float swerve_seconds;        /* AvoidUnits made the unit swerve for this long. */
    float spin_seconds;          /* The unit turned on the spot for this long. */
    float previous_angle;        /* This is `angle` at the previous tick. */
    bool is_spinning;            /* The unit turns on the spot in this tick. */
    bool is_walled;              /* The unit slides along blocked cells in this tick. */
    bool was_walled;             /* The unit slid along blocked cells in the previous tick. */
    int wall_hit_count;          /* This counts the starts of such slides. */
    float in_obstacle_seconds;   /* The body was deep in blocked cells for this long. */
    bool is_swerving;            /* AvoidUnits makes the unit swerve in this tick. */
    bool is_in_contact;          /* The unit touches another unit in this tick. */
    bool is_adjusting;           /* The new avoidance changes the velocity in this tick. */
    float held_seconds;          /* The new avoidance held the unit nearly still for this long. */
    float avoidance_off_seconds; /* The new avoidance is off while this is above 0. */
    float adjust_seconds;        /* The new avoidance changed the velocity for this long. */
    bool was_adjusting;          /* The new avoidance changed the velocity in the previous tick. */
    int adjust_count;            /* This counts the starts of such changes. */

    /* These fields are the state of the unit in formation mode. */
    bool has_yielded;            /* The unit stepped aside from its slot for a squad mate. */
    V2 home;                     /* A parked unit parked here and dodges only close to it. */
    bool has_home;               /* It is true once `home` holds the parking spot. */
    V2 place_offset;             /* It is the place less the anchor, with x right and y forward. */
    float file_distance;         /* In single file, the place is this far behind a still anchor. */
    int file_rank;               /* This is the rank of the place in single file (0 is first). */
    V2 start_offset;             /* This is the position less the place at the move set-up. */
    float route_progress;        /* The unit is this far along the squad route. */

    /* These fields are the detour state of the unit. */
    V2 detour[MAX_DETOUR_POINTS];  /* `detour[0..detour_count)` are its points after its start. */
    int detour_count;
    int detour_index;              /* The unit drives at `detour[detour_index]` on its detour. */
    float detour_seconds;          /* The unit has followed its detour for this long. */
    V2 progress_check_position;    /* The unit stood here when the progress check started. */
    float progress_check_seconds;  /* The progress check has run for this long. */
} Unit;

/* These are the values of `Squad.state`, with the numbers of the game itself. */
enum {
    SQUAD_IDLE = 0,       /* The squad has no order. */
    SQUAD_MOVING = 1,     /* The squad drives its route. */
    SQUAD_ARRIVED = 2,    /* The squad arrived and stands. */
    SQUAD_DENIED = 3,     /* The squad found no route or no room for its formation. */
    SQUAD_SEARCHING = 13, /* The squad searches its route. */
};

/* A `Squad` holds one squad: its order, its state and its search, its metrics, and its state in
 * formation following. Its members are `units[first_unit_index]` to
 * `units[first_unit_index + unit_count - 1]`.
 * - The straggler timer lets a squad arrive while one member still moves.
 * - The spread of a group of points is the distance from their mean position to the point
 *   furthest from it. `measure` measures the spread of the members after each tick while the
 *   squad moves. */
typedef struct {
    const Kind *kind;        /* The units of the squad are of this kind. */
    int squad_index;         /* This is the index of the squad itself in `squads`. */
    int first_unit_index;    /* This is the index of the first member in `units`. */
    int unit_count;          /* The squad has this many members. */
    int formation_type;      /* This picks the shape of the formation layout of the game. */
    V2 destination;          /* The move order goes here. */
    V2 facing;               /* The formation ends facing this way. It can have any length. */
    V2 start_position;       /* The lead stood here at the order. The search starts here. */
    V2 last_facing;          /* This is the facing of the previous move, for `is_flipped`. */
    int state;               /* It holds one of the `SQUAD_` values. */
    bool is_active_search;   /* The quiet search failed, so the active search runs. */
    bool is_replanning;      /* It searches again around squads that parked on its route. */
    bool is_avoiding_slots[MAX_SQUADS]; /* With `--parkahead`, it avoids the slots of squad k. */
    bool is_settled;         /* It still moves, but counts as parked for those searches. */
    float start_time_seconds; /* The move of the last order started at this time. */
    int set_up_phase;        /* With `--setupslice on`, the route set-up is at this phase. */
    float set_up_slots[2 * MAX_UNITS]; /* These are the slots of that set-up, as x, y pairs. */
    V2 set_up_facing;        /* The formation of that set-up faces this way, with length 1. */
    bool is_flipped;         /* In game mode, members take the slots in reverse order. */
    GcRequest search;        /* This is the state of the search of the squad. */
    float straggler_seconds; /* This straggler timer runs while 0 or more. */
    float order_time_seconds;  /* The squad got its order at this time. */
    float arrive_time_seconds; /* The squad arrived at this time. */
    float spread_reference;  /* This is the spread of the slots. */
    float max_spread;        /* This is the largest spread beyond `spread_reference`. */
    float spread_sum;        /* This adds up the spreads beyond `spread_reference`. */
    int spread_sample_count; /* `spread_sum` adds up this many spreads. */
    int crossing_count;      /* This counts member pairs whose start-to-slot lines cross. */
    V2 route[MAX_WAYPOINTS]; /* `route[0..route_count)` is the squad route, from its start. */
    int route_count;

    /* These fields are the state of the squad in formation following. */
    float route_distance[MAX_WAYPOINTS]; /* Each route point is this far along the route. */
    float route_length;       /* The squad route is this long. */
    float anchor_distance;    /* The anchor is this far along the route. */
    float anchor_speed;       /* The anchor moves at this speed. */
    float waited_seconds;     /* The anchor is nearly still for this long now. */
    float ignore_lag_seconds; /* The anchor ignores laggards while this is above 0. */
    float reassign_seconds;   /* Members are matched to places again when this runs out. */
    int reassign_count;       /* This counts the changes of the match to places. */
    bool has_final_match;     /* The members got their final slots for the end leg. */
    V2 final_facing;          /* The formation faces this way at the end of the route. */
    float full_speed_radius;  /* Every member can drive bends of this radius at its top speed. */
    int lane_count;           /* The squad uses this many lane samples (`Lane`). */
    bool is_single_file;      /* The squad drives in single file now. */
    bool is_reforming;        /* It left single file, and its members still close up. */

    /* These fields are the state of the squad in the coordination between squads. */
    float coordination_speed_cap;      /* The anchor drives at most this fast in this tick. */
    float coordination_held_seconds;   /* The cap kept the anchor nearly still this long. */
    float ignore_coordination_seconds; /* The squad ignores the others while this is above 0. */
    float side_offset;                 /* Lanes shift this far right around a meeting or pass. */
    float side_offset_target;          /* `side_offset` moves toward this. */
    float side_offset_rate;            /* `side_offset` changes this fast, in m per s. */
    float side_shift_start;            /* The full shift starts this far along the route. */
    float side_shift_end;              /* The full shift ends this far along the route. */
    int yielding_to_index;             /* The squad waits for this squad, or `NO_SQUAD`. */
    bool is_yield_kept;                /* The squad still waits for it in this tick. */
    int passing_index;                 /* The squad passes this squad, or `NO_SQUAD`. */
    bool is_pass_kept;                 /* The squad still passes it in this tick. */
    bool is_file_pass;                 /* It passes that squad in single file. */
    bool is_pass_waiting;              /* It follows that squad until it has formed for it. */
    bool is_file_forced;               /* A passing squad needs this one in single file now. */
    bool is_following[MAX_SQUADS];     /* Entry k is true when the squad follows `squads[k]`. */
    bool was_following[MAX_SQUADS];    /* Entry k is true when it did so in the last tick. */
} Squad;

#define NO_SQUAD -1 /* `yielding_to_index` or `passing_index` holds this while there is none. */

/* ------------------------------------------------------------------------------------------------
 * `is_squad_busy` returns true while the squad moves or still searches its route. The units of
 * such a squad do not count as parked.
 * --------------------------------------------------------------------------------------------- */
static inline bool is_squad_busy(const Squad *squad) {
    return squad->state == SQUAD_MOVING || squad->state == SQUAD_SEARCHING;
}

/* ------------------------------------------------------------------------------------------------
 * `get_forward_share` returns the cosine of the angle between `direction` and the heading of the
 * unit, or 0 when they are more than 90 degrees apart. `direction_length` must be the length of
 * `direction`. Callers multiply a speed by it, so a unit drives slower the further it must turn,
 * and brakes to a stop while it must turn by more than 90 degrees.
 * --------------------------------------------------------------------------------------------- */
static inline float get_forward_share(const Unit *unit, V2 direction, float direction_length) {
    float alignment = dot(unit->heading, direction) / direction_length;
    return fmaxf(0, alignment);
}

/* sim.c defines this state, which all parts share. */
extern Unit units[MAX_UNITS];                /* The run has `units[0..unit_count)`. */
extern Squad squads[MAX_SQUADS];             /* The run has `squads[0..squad_count)`. */
extern int unit_count;                       /* The run has this many units. */
extern int squad_count;                      /* The run has this many squads. */
extern float now_seconds;                    /* This is the simulated time of the run. */
extern float path_points[2 * MAX_WAYPOINTS]; /* Routes from the search go here as x, y pairs. */
extern FILE *trace_file;                     /* `--trace` writes the JSON replay here. */

/* These are the values of `avoid_mode`. In `AVOID_GAME`, units move as in the game, with
 * AvoidUnits. In `AVOID_FORM`, squads use formation following with the new unit avoidance. In
 * `AVOID_COORD`, they also coordinate with each other. */
enum { AVOID_GAME, AVOID_FORM, AVOID_COORD };
extern int avoid_mode;
extern const char *MODE_NAME[]; /* `MODE_NAME[avoid_mode]` names the mode in the output. */

/* ------------------------------------------------------------------------------------------------
 * `is_formation_mode` returns true while squads use formation following, with or without the
 * coordination between squads.
 * --------------------------------------------------------------------------------------------- */
static inline bool is_formation_mode(void) {
    return avoid_mode != AVOID_GAME;
}

/* These switches are on by default, and the command-line switch in each comment turns one off.
 * `is_find_infantry_on`, `is_turn_slowdown_on` and `is_leader_file_on` are off by default
 * instead. On the command line, every value other than "off" turns a switch on. */
extern bool are_lanes_on;   /* With `--lanes off`, a place is the anchor plus the slot offset. */
extern bool is_reassign_on; /* With `--reassign off`, members keep their first places. */
extern bool is_yield_on;    /* With `--yield off`, parked units never move for others. */
extern bool is_smooth_on;   /* With `--smooth off`, the route keeps its sharp corners. */
extern bool is_wide_turn_on; /* With `--wideturns off`, smooth routes keep their sharper bends. */
extern bool is_pursuit_on;  /* With `--pursuit off`, units steer straight at their places. */
extern bool is_rejoin_room_on; /* With `--rejoinroom off`, rejoin points need no room or gap. */
extern bool is_crossing_yield_on; /* With `--crossing off`, squads ignore crossing routes. */
extern bool is_keep_right_on;     /* With `--keepright off`, head-on squads do not shift. */
extern bool is_turn_taking_on;    /* With `--turns off`, squads do not wait at narrow passages. */
extern bool is_speed_priority_on; /* With `--speedfirst off`, faster squads get no priority. */
extern bool is_smooth_sway_on;    /* With `--smoothsway off`, head-on shifts can swerve sharply. */
extern bool is_find_infantry_on;  /* With `--findinfantry on`, `--find` measures marine routes. */
extern bool is_replan_on;         /* With `--replan off`, stuck units get no detour. */
extern bool is_linear_steer_on;   /* With `--linearsteer off`, formation units use `steer_to`. */
extern bool is_bend_look_on;      /* With `--bendlook off`, bend checks look less far ahead. */
extern bool is_turn_slowdown_on;  /* With `--turnslow on`, anchors also slow down for turns. */
extern bool is_ahead_slowdown_on; /* With `--aheadslow off`, units far ahead stop abruptly. */
extern bool is_end_stop_on;       /* With `--endstop off`, units end by following their places. */
extern bool is_overtake_on;       /* With `--overtake off`, no squad passes or follows. */
extern bool is_center_search_on;  /* With `--centersearch off`, routes start at the lead only. */
extern bool is_late_yield_on;     /* With `--lateyield off`, squads give way early and gently. */
extern bool is_form_first_on;     /* With `--formfirst off`, passes start before leaders form. */
extern bool is_keep_route_on;     /* With `--keeproute off`, passed squads shift aside too. */
extern bool is_park_replan_on;    /* With `--parkreplan off`, routes ignore squads parked later. */
extern bool is_leader_file_on;    /* With `--leaderfile on`, passed squads can form single file. */
extern bool is_park_leeway_on;    /* With `--parkleeway off`, only arrived squads count parked. */
extern bool is_end_cap_on;        /* With `--endcap off`, avoidance can speed up end legs. */
extern bool is_corner_cut_on;     /* With `--cutcorners off`, routes keep the game corners. */
extern bool is_short_return_on;   /* With `--shortreturn off`, head-on shifts end later. */
extern bool is_late_shift_on;     /* With `--lateshift off`, head-on shifts start earlier. */
extern bool is_partial_shift_on;  /* With `--partialshift off`, too little room shifts nobody. */
extern bool is_wide_gap_on;       /* With `--widegap off`, head-on gaps are 4 m between bodies. */
extern bool is_park_speed_on;     /* With `--parkspeed off`, slow units never count as parked. */
extern bool is_set_up_slice_on;   /* With `--setupslice on`, route set-ups spread over ticks. */
extern bool is_set_up_chain_on;   /* With `--setupchain off`, set-ups do one part a tick. */
extern bool is_widen_stop_on;     /* With `--widenstop off`, widenings run all their passes. */
extern bool is_keep_follow_on;    /* With `--keepfollow off`, leaders that turn lose followers. */
extern bool is_wait_ahead_on;     /* With `--waitahead off`, units turn back to places behind. */
extern bool is_reform_match_on;   /* With `--reformmatch off`, file order picks the new places. */
extern bool is_full_search_on;    /* With `--fullsearch off`, squad searches take several ticks. */
extern bool is_set_up_threads_on; /* With `--setupthreads off`, one thread smooths all routes. */
extern bool is_keep_speed_on;     /* With `--keepspeed off`, routes around parked squads brake. */
extern bool is_park_ahead_on;     /* With `--parkahead off`, squads wait for slots to fill. */
extern bool is_cross_follow_on;   /* With `--crossfollow off`, crossing routes can pass. */
extern bool is_old_route_on;      /* With `--oldroute off`, failed replans deny the move. */
extern bool is_slot_point_on;     /* With `--slotpoint off`, park-ahead ends at first contact. */
/* movement.c defines these functions of the unit movement of the game itself. */

/* `steer_to` returns the steering that turns the unit toward `direction`, from -1 (full lock to
 * the left) to 1 (full lock to the right). The steering is smaller the better the unit faces
 * `direction`, and it is full lock when `direction` is more than 90 degrees off the heading.
 * Edge case: A zero `direction` gives NaN. */
float steer_to(const Unit *unit, V2 direction);

/* `steer_to_facing` returns the steering that turns the unit toward its `facing`, as the game
 * turns a stopped unit, or 0 when `facing` is zero. */
float steer_to_facing(const Unit *unit);

/* `accelerate` adds one tick of the acceleration of the unit along its heading, times `throttle`,
 * which goes from 0 to 1. Friction also slows the unit in every tick, so a throttle below
 * `friction / acceleration` slows a moving unit. */
void accelerate(Unit *unit, float throttle);

/* `accelerate_toward_speed` accelerates with the throttle that brings the speed along the
 * heading to `target_speed` within about `time_constant_seconds`. The throttle covers the
 * friction of the tick. A unit that is much too fast gets no throttle and brakes by friction
 * alone. */
void accelerate_toward_speed(Unit *unit, float target_speed, float time_constant_seconds);

/* `get_search_clearance` returns the clearance that the searches of the game keep at obstacle
 * corners for `unit`: twice its radius. */
float get_search_clearance(const Unit *unit);

/* `search_path` searches a path from the unit to `destination` with the search of the game, as
 * Mover::FindPath does. Each of its tries can use `budget` expansions (steps of the path search)
 * and can end within `tolerance` meters of `destination`. In formation mode the path goes around
 * parked squads.
 * Result: It returns true when a search found a path, and `search` then holds it for
 *   `gc_get_path`. The caller must free `search` with `gc_free_request`, whether a path was found
 *   or not. */
bool search_path(const Unit *unit, V2 destination, float tolerance, int budget,
                 GcRequest *search);

/* `set_move` gives the unit a move along the squad route `route[0..route_count)`, which holds at
 * most `MAX_WAYPOINTS` points, with the final `facing`. The route is the path of the squad,
 * without the position of the lead at its start. The unit searches its own path to the second
 * point of the route, or to its only point, and the move ends on `slot`.
 * Edge case: When that search fails, the unit gets no waypoints after the current one, and the
 *   current one becomes its own position, so it does not move. */
void set_move(Unit *unit, const V2 *route, int route_count, V2 facing, V2 slot);

/* `stop_at` makes `point` the current waypoint of the unit with no waypoints after it, and sets
 * `blocker_index` to `NO_BLOCKER`. It also clears `is_stopped` and `is_last_leg` and sets `reach`
 * to `reach_last`. The unit then drives to `point` and stops within `reach_last` of it. */
void stop_at(Unit *unit, V2 point);

/* `tick_mover` moves `units[unit_index]` for one tick, as Mover::Tick does in the game. In game
 * mode, and in formation mode while its squad is idle, the unit follows its waypoints, and a unit
 * that is not stopped steers as in the game. In formation mode, a unit that is not stopped and
 * whose squad is not idle steers by formation following. A stopped unit in formation mode makes
 * room for others while its squad moves, or while `is_yield_on` is true and its squad neither
 * moves nor searches. Otherwise a stopped unit turns to its `facing`. The unit then turns, slows
 * by friction, moves and resolves collisions. Call it once for each unit in every tick, after
 * `update_squad` ran for each squad. */
void tick_mover(int unit_index);

/* physics.c defines these functions of the motion of the game itself: Turn, Friction, Integrate
 * and Physics. `tick_mover` calls them in this order for each unit, after it found the steering of
 * the unit. */

#define AVOID_TICKS 5 /* Swerves (movement.c) and wall slides set `avoid_ticks` to this. */

/* `turn` turns the unit for one tick at its turn rate times `steering`, as Turn does in the game.
 * `steering` goes from -1 (full lock to the left) to 1 (full lock to the right), and `turn`
 * limits other values to that range. It sets `angle`, `heading` and `last_turn_rate`. */
void turn(Unit *unit, float steering);

/* `apply_friction` slows the unit by one tick of its friction, as Friction does in the game. It
 * then limits the speed of the unit to its top speed. */
void apply_friction(Unit *unit);

/* `integrate` moves `units[unit_index]` by one tick of its velocity and keeps it within the map,
 * as Integrate does in the game. It then resolves collisions as Physics does: with other units,
 * then with blocked cells, then with ground that is too steep. A slide along blocked cells sets
 * `is_walled` and `avoid_ticks`, and it can flip `swerve_side`. */
void integrate(int unit_index);

/* squad.c defines these functions of the squad movement of the game itself. */

/* In formation mode, `stamp_parked` stamps the parked units (stopped units of squads that
 * neither move nor search) of every squad other than `moving_squad`. Call it before the search or
 * the formation layout of `moving_squad` between `start` and `end`. It makes the cells close to
 * the bodies of those units cliffs (cells that block every unit) in the grid of the game, so that
 * the code of the game itself plans around them. While `moving_squad` `is_replanning`, it stamps
 * one disc around all members of each other squad that neither moves nor searches, or that
 * `is_settled`, instead. The cells that hold `start` and `end` stay as they are. In game mode it
 * stamps nothing.
 * Edge cases: Each call must be followed by `unstamp_parked` before the next one, because only
 *   the cells of the last call can get their old values back. Past a fixed number of changed
 *   cells, `stamp_parked` stamps no more cells. */
void stamp_parked(const Squad *moving_squad, V2 start, V2 end);

/* `unstamp_parked` gives the cells that `stamp_parked` stamped their old values back. Call it
 * right after the search or the layout. Until then, the physics and every other search also see
 * the stamped cells as cliffs. */
void unstamp_parked(void);

/* `get_stamp_distance_cells` returns how many cells cell (`cell_x`, `cell_y`) lies from the box
 * around the cells that the last `stamp_parked` stamped, counted along the axis that gives the
 * larger count: 0 inside the box and 1 next to it. While no cell is stamped, it returns
 * `NO_STAMP_DISTANCE_CELLS`. */
#define NO_STAMP_DISTANCE_CELLS 1000000 /* The stamp distance is this while nothing is stamped. */
int get_stamp_distance_cells(int cell_x, int cell_y);

/* `hash_stamped_cells` returns a hash of the cells that the last `stamp_parked` changed and of
 * their old values. Two stamps with the same hash changed the same cells, almost surely. */
uint32_t hash_stamped_cells(void);

/* `order_move` gives `squad` a move order to `destination`. It starts the quiet search of the
 * squad from its lead and sets the state to `SQUAD_SEARCHING`. In formation mode the search goes
 * around parked squads. It also sets `facing` from the lead to `destination` and
 * `order_time_seconds` to `now_seconds`. */
void order_move(Squad *squad, V2 destination);

/* `update_squad` runs one tick of `squad`. While the squad searches, its search goes on, and once
 * it finds a path, the members get their moves. While the squad moves, formation mode moves its
 * anchor, and once the squad counts as arrived, its members stop where they stand and the state
 * becomes `SQUAD_ARRIVED`. A squad whose quiet and active searches both fail, or whose formation
 * cannot be laid out at the end of the route, becomes `SQUAD_DENIED`. With `--parkreplan on` in
 * `AVOID_COORD`, the other moving squads whose routes pass a squad that arrives, or that becomes
 * `is_settled`, search their routes again. With `--oldroute on`, a squad whose new search or
 * layout fails then goes on along its old route instead of becoming `SQUAD_DENIED`. */
void update_squad(Squad *squad);

/* `reset_set_up_work` gives the sliced route set-ups (`--setupslice on`) the work budget of a new
 * tick, which they share with `--setupchain on`. Call it once at the start of every tick, before
 * the first `update_squad`. */
void reset_set_up_work(void);

/* `finish_threaded_set_ups` ends the route set-ups that `--setupthreads on` handed to worker
 * threads in this tick: it waits for their smoothings, finishes each set-up and starts each move,
 * in the order of the squads. `update_squad` calls it before a moving squad runs its tick. Call it
 * also after the last `update_squad` of each tick, before the units move. */
void finish_threaded_set_ups(void);

/* maps.c defines these functions and this state for the maps of the game. */

#define MAP_NAME_SIZE 64             /* `map_name` holds this many bytes. */

/* `maps_dir` is the folder with the files of Missions.sdf, which `--maps` sets. The default is
 * "build/sdf/Missions", relative to the working directory. */
extern const char *maps_dir;
extern char map_name[MAP_NAME_SIZE]; /* This is the name of the loaded map. */

/* `load_map` loads the grid of the mission `name` into the `PathFinder` of the game and sets
 * `map_name`. It looks for the .SLP file in the MISSIONS and MULTIPLAYER/MISSIONS folders of
 * `maps_dir`.
 * Edge case: When it finds no file, or the file is too short, the program prints an error and
 *   exits. */
void load_map(const char *name);

/* `is_circle_clear` returns true when the circle of `radius` around `point` overlaps no cell that
 * blocks `mover_class`, and false otherwise. `radius` is in meters. */
bool is_circle_clear(V2 point, float radius, int mover_class);

/* `is_line_clear` returns true when no cell on the straight line from `from` to `to` blocks
 * `mover_class`, and false otherwise. */
bool is_line_clear(V2 from, V2 to, int mover_class);

/* `is_way_clear` returns true when the straight line from the unit to `point` crosses no cell
 * that blocks the mover class of the unit, and false otherwise. */
bool is_way_clear(const Unit *unit, V2 point);

/* `get_clearance_meters` returns a distance in meters around cell (`cell_x`, `cell_y`) that holds
 * no cell blocking `mover_class`: every point closer than it to any point of the cell lies in a
 * cell that does not block the class, as both `gc_get_cell` and `gc_read_cell_at` read the cells.
 * It counts the cells that `stamp_parked` stamped, and cells outside the grid count as blocking.
 * It returns 0 next to a blocking cell, for a cell outside the grid, and for mover classes above
 * 2. `load_map` builds the field that it reads, once per map, because the grid of a map,
 * buildings included, stays the same for a whole run apart from the stamps. */
float get_clearance_meters(int cell_x, int cell_y, int mover_class);

/* find.c defines this function of `--find`. */

/* `find_spots` runs `--find`. It loads the map `name`, prints a header line, and makes
 * `sample_count` tries. Each try draws random route ends on open ground and searches the game
 * route between them, for a medium tank, or for a marine with `--findinfantry on`. For each route
 * found, it prints the ends, the straight distance, the length of the route divided by the
 * straight distance, the number of turns, and the half-width in world units of the narrowest
 * passage. Every run draws the same routes. */
void find_spots(const char *name, int sample_count);

/* scenarios.c defines these unit types, scenarios and orders. */

#define SCENARIO_COUNT 12 /* The simulator has this many test scenarios. */

/* These are the unit types. Their values come from the game, as `Kind` describes. */
extern const Kind MEDIUM_TANK;
extern const Kind LIGHT_TANK;
extern const Kind HEAVY_TANK;
extern const Kind MARINE; /* Marines are infantry, and they cross hills that block tanks. */

/* A `scenario_index` is an index into the three arrays below. */
extern const char *SCENARIO_NAME[SCENARIO_COUNT];        /* The command line uses these names. */
extern const char *SCENARIO_DESCRIPTION[SCENARIO_COUNT]; /* The trace shows these descriptions. */
extern const int SCENARIO_RUN_ORDER[SCENARIO_COUNT];     /* The scenarios run in this order. */

/* `set_up_scenario` sets the orders in these arrays, with one entry for each squad. */
extern V2 order_destinations[MAX_SQUADS]; /* Each squad with an order moves here. */
extern V2 order_facings[MAX_SQUADS];      /* A squad ends facing this way unless this is zero. */
extern bool has_order[MAX_SQUADS];        /* Squads without an order stay parked. */

/* `set_up_scenario` clears the previous run, loads the map of the scenario, places its squads as
 * parked squads on the formation layout of the game, and sets their orders in
 * `order_destinations`, `order_facings` and `has_order`. Call it before every run.
 * Inputs: A `seed` other than 0 moves the start of each squad and each unit a little. The moves
 *   depend only on `seed` and `scenario_index`, so they are the same in every mode.
 * Edge case: The line scenario, `lineplain`, keeps its starts for every `seed`. */
void set_up_scenario(int scenario_index, unsigned seed);

/* metrics.c defines these functions of the metrics, the results and the trace. */

/* `measure` updates the metrics of every unit and of every moving squad. Call it once after each
 * tick. */
void measure(void);

/* `print_result_header` prints the names of the columns of the result rows. Call it once before
 * the first row. */
void print_result_header(void);

/* `begin_trace_run`, `write_trace_frame`, `report_squad` and `write_trace_grid` write the JSON
 * replay into `trace_file`. Call `begin_trace_run`, `write_trace_frame` and `write_trace_grid`
 * only while `trace_file` is open. For each run, call them in this order:
 * 1. `begin_trace_run`, once, after `set_up_scenario`.
 * 2. `write_trace_frame`, for each frame to keep.
 * 3. `report_squad`, for each squad. While `trace_file` is open, the caller first writes
 *    `],"squads":[`.
 * 4. `write_trace_grid`, after the caller writes `]`.
 * Then the caller closes the object of the run with `}`.
 *
 * `begin_trace_run` starts the object of a run. It writes the names of the scenario, the mode and
 * the map, the description of the scenario, the cell size and the squad, radius and top speed of
 * each unit, and it opens the list of frames. */
void begin_trace_run(int scenario_index);

/* `write_trace_frame` writes one frame: the time, and the position, angle, waypoint and state
 * bits of each unit. `is_first` is true for the first frame of the run. */
void write_trace_frame(bool is_first);

/* `report_squad` writes the kind, route, slots and lanes of the squad into the trace while
 * `trace_file` is open. For a squad with an order, it also prints the result row and writes the
 * metrics into the trace. It closes the JSON object of the squad. */
void report_squad(int scenario_index, int squad_index);

/* `write_trace_grid` writes the cells of the map around the area that the units and routes used
 * in the run. */
void write_trace_grid(void);

/* timing.c defines this function of the wall clock. */

/* `read_clock_seconds` returns the time of a high-resolution wall clock, in seconds from a fixed
 * start. Only differences between two readings have a meaning. */
double read_clock_seconds(void);

/* workers.c defines these functions, which run jobs on worker threads beside the main thread. */

/* A `WorkerJob` is a function that a worker thread calls with the `argument` of `submit_job`. */
typedef void WorkerJob(void *argument);

/* `submit_job` hands `job` over to the worker threads, and a free one starts it at once. Call it
 * from the main thread only. The job may run on any thread, also on the main thread inside
 * `wait_for_jobs`, and it runs with the floating-point settings of the main thread. Until
 * `wait_for_jobs` returns, the job must not touch what the main thread changes, nor the other way
 * around. At most 31 jobs may wait at a time. The first call starts the worker threads, which
 * stop by themselves when the program ends, is interrupted (Ctrl+C, closed console) or crashes. */
void submit_job(WorkerJob *job, void *argument);

/* `wait_for_jobs` returns once every submitted job has ended. The main thread runs the jobs that
 * no worker took yet itself. */
void wait_for_jobs(void);

#endif
