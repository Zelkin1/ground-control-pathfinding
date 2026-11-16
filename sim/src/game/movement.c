/* This file reconstructs the unit movement of the game itself, the Mover code, from the
 * decompiled gc.exe.
 *
 * Modes: In game mode, where `avoid_mode` is `AVOID_GAME`, each unit follows its waypoints and
 * runs Steer, AvoidUnits, Turn, Friction, Integrate and Physics. physics.c holds the last four of
 * them. In the formation modes, where `avoid_mode` is `AVOID_FORM` or `AVOID_COORD`, steering.c
 * steers the moving units of every
 * squad that is not idle, and avoidance.c lets stopped units make room. The rest stays the same,
 * except for two changes. `stamp_parked` from squad.c and `is_in_sight` make routes go around
 * parked squads, and collisions never push the units of parked squads.
 *
 * Terms: The terms of sim.h apply (waypoint, leg, reach, parked). `waypoint` holds the current
 * waypoint of a unit, and `waypoints` holds the ones after it. The mover class of a unit picks the
 * grid cells that block it, as game.h describes.
 *
 * Game code: Each function that follows a game function names it and its address in gc.exe
 * 1.0.1.1, whose image always loads at 0x400000. Offsets such as +0x95 are byte offsets of fields
 * in the Mover object of the game. re/PATHFINDING.md describes these functions and fields. The
 * constants are values of the game itself unless their comments say otherwise.
 *
 * Callers: squad.c calls `set_move` to give the units of a squad their moves, and `stop_at` to
 * stop them.
 *
 * Mover fields: The Mover fields of `Unit` hold the state that these functions read and write.
 * `waypoint`, `waypoints`, `waypoint_count`, `is_last_leg`, `reach`, `is_stopped` and
 * `is_reorienting` describe the route that waypoint following drives, and `update_mover` and
 * `find_path` change them. `blocker_index`, `wait_seconds`, `wait_heading`, `wait_count`,
 * `avoid_ticks`, `swerve_side`, `swerve_seconds`, `is_swerving` and `side_cooldown_seconds` hold
 * the state of AvoidUnits, and `avoid_units` changes them. `angle`, `heading`, `velocity`,
 * `last_turn_rate` and `is_walled` hold the motion, and `turn`, `apply_friction` and
 * `apply_physics` in physics.c change them. `reach_last` and `reach_mid` keep the start values of
 * `reach`. */
#include <string.h>

#include "sim.h"
#include "formation.h"

/* This value belongs to the simulator, and only formation mode uses it. */
#define SIGHT_TOUCH_MARGIN_METERS 0.5f /* Sight fails when the line brings bodies this close. */

/* Mover::SetMove, Mover::FindPath and Mover::Update use these values. A search budget is a
 * number of expansions, the steps of the path search of the game. The game re-paths with a
 * budget of 32 for a unit that chases a target, which the simulator does not model. */
#define SET_MOVE_SEARCH_BUDGET 200     /* SetMove searches with this budget. */
#define REPATH_SEARCH_BUDGET 100       /* Re-paths to a hidden waypoint use this budget. */
#define WAYPOINT_TOLERANCE_METERS 1.0f /* SetMove can end its search this close to a waypoint. */
#define LOOK_AHEAD_PHASES 3            /* Update looks ahead in this many ticks out of four. */
#define REACH_EPSILON_METERS 0.1f      /* A reach this far past `reach_last` skips the re-path. */

/* Steer and its arrival factor use these values. Re-orienting means turning toward the current
 * waypoint without accelerating. */
#define ARRIVAL_STOP_DISTANCE_METERS 3.0f /* Braking aims to stop this short of the waypoint. */
#define BRAKING_START_FRACTION 0.5f       /* Braking starts at this part of the friction. */
#define WAYPOINT_BEHIND_METERS -0.2f      /* Waypoints below this along the heading are behind. */
#define BEHIND_THROTTLE 0.5f              /* Steer uses this throttle for a waypoint behind. */
#define MIN_THROTTLE 0.6f                 /* Steer never goes below this throttle otherwise. */
#define REORIENT_DONE_COSINE 0.95f        /* Re-orienting ends within about 18 degrees. */

/* AvoidUnits uses these values. It looks for other units in a corridor along the heading of the
 * unit, and the closest of them that counts is the blocker, which the unit avoids. A swerve is a
 * hard turn to one side, around the blocker. The half-width of the corridor is the radius of the
 * unit times both factors below, plus the radius of the other unit. AvoidUnits widens the
 * collision radius of the unit by `QUERY_RADIUS_MARGIN_FACTOR` before its query, and the query
 * (0x4fef30) widens it by `QUERY_RADIUS_FACTOR`. TODO: confirm which factor each of the two
 * functions uses. */
#define QUERY_RADIUS_FACTOR 1.1f         /* The query widens the radius of the unit by this. */
#define QUERY_RADIUS_MARGIN_FACTOR 1.25f /* AvoidUnits widens it by this. */
#define LAST_LEG_TURN_COSINE 0.7f        /* A slot over 45.6 degrees off the heading is faced. */
#define SLOT_QUERY_SHORTFALL_METERS 1.0f /* The check for that turn ends this short of the slot. */
#define LOOK_DISTANCE_FACTOR 0.6f        /* Corridors span this part of the waypoint distance. */
#define MAX_LOOK_RADII 4.0f              /* It spans at most this many radii of the unit. */
#define MIN_LOOK_RADII 1.5f              /* It spans at least this many radii, also when halved. */
#define BEHIND_LOOK_FACTOR 0.5f          /* It halves if the unit faces away from the waypoint. */
#define MAX_BLOCKER_TANGENT_SQUARED 100000.0f /* No corridor reaches a blocker this far away. */
#define NEW_BLOCKER_RADII 2.4f           /* A new blocker this many radii away gets a new side. */
#define DRIVING_ON_AVOID_TICKS 2        /* Blockers with `avoid_ticks` up to this can drive on. */
#define SWERVE_STEERING 4.0f             /* A swerve adds this steering, which Turn clamps to 1. */
#define WAIT_SECONDS 2.0f                /* A unit waits this long for a faster blocker. */
#define SWERVE_AFTER_WAIT_SECONDS -2.0f  /* Units never wait at `wait_seconds` of this or more. */
#define SWERVE_LEFT 0                    /* `swerve_side` holds this for a swerve to the left. */
#define SWERVE_RIGHT 1                   /* `swerve_side` holds this for a swerve to the right. */

/* ------------------------------------------------------------------------------------------------
 * `pop_waypoint` removes the next waypoint from the list of the unit and returns it. Callers make
 * sure that the list is not empty. `waypoints[0..waypoint_count)` holds the waypoints after the
 * current one, the next one first.
 * --------------------------------------------------------------------------------------------- */
static V2 pop_waypoint(Unit *unit) {
    V2 next_waypoint = unit->waypoints[0];
    size_t moved_bytes = (size_t)(unit->waypoint_count - 1) * sizeof(V2);
    memmove(&unit->waypoints[0], &unit->waypoints[1], moved_bytes);
    unit->waypoint_count--;
    return next_waypoint;
}

/* ------------------------------------------------------------------------------------------------
 * `update_leg` sets `is_last_leg` (Mover field +0x95) and `reach` (Mover field +0x58) after the
 * current waypoint changed. The unit is on its last leg when no waypoint follows the current
 * one. The current waypoint then counts as reached within `reach_last`, and otherwise within
 * `reach_mid`.
 * --------------------------------------------------------------------------------------------- */
static void update_leg(Unit *unit) {
    unit->is_last_leg = unit->waypoint_count == 0;
    unit->reach = unit->is_last_leg ? unit->reach_last : unit->reach_mid;
}

/* ------------------------------------------------------------------------------------------------
 * `is_in_sight` returns true when the unit sees `target`, as waypoint following checks it. The
 * raycast of the game must find no cell on the straight line that blocks the mover class of the
 * unit. In formation mode, the line must also keep the body of the unit more than
 * `SIGHT_TOUCH_MARGIN_METERS` from the body of every unit of another squad that neither moves
 * nor searches:
 *
 *     unit o-------------------+-------------------> `target`
 *                              |  the sight fails when this distance is less than both radii
 *                              |  plus `SIGHT_TOUCH_MARGIN_METERS`
 *                              O  unit of a parked squad
 *
 * A route that a parked squad blocks then counts as hidden, so waypoint following searches a way
 * around it.
 * --------------------------------------------------------------------------------------------- */
static bool is_in_sight(const Unit *unit, V2 target) {
    /* Cast the ray of the game. */
    V2 from = unit->position;
    int mover_class = unit->kind->mover_class;
    if (gc_cast_ray(from.x, from.y, target.x, target.y, mover_class) != GC_RAY_CLEAR)
        return false;
    if (!is_formation_mode())
        return true;

    /* Compare the distance from each unit of a parked squad to the closest point of the line with
     * the distance at which the bodies come too close. */
    V2 line = sub(target, from);
    float line_length_squared = length_squared(line);
    for (int other_index = 0; other_index < unit_count; other_index++) {
        const Unit *other = &units[other_index];
        bool is_same_squad = other->squad_index == unit->squad_index;
        if (is_same_squad || is_squad_busy(&squads[other->squad_index]))
            continue;
        float closest_fraction = 0;
        if (line_length_squared > 0) {
            float projection = dot(sub(other->position, from), line);
            closest_fraction = clamp_float(projection / line_length_squared, 0, 1);
        }
        V2 closest = add(from, mul(line, closest_fraction));
        float combined_radius = other->kind->radius + unit->kind->radius;
        float touch_distance = combined_radius + SIGHT_TOUCH_MARGIN_METERS;
        if (length_squared(sub(other->position, closest)) < touch_distance * touch_distance)
            return false;
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `get_search_clearance` takes twice the radius of the unit, which is the width of its body, as
 * the clearance value of the searches.
 * --------------------------------------------------------------------------------------------- */
float get_search_clearance(const Unit *unit) {
    return 2 * unit->kind->radius;
}

/* ------------------------------------------------------------------------------------------------
 * `search_path` follows Mover::FindPath (0x50eb60). It runs the quiet search first, which works
 * forward from the unit. When that finds no way, it runs the active search, which works backward
 * from `destination`.
 * --------------------------------------------------------------------------------------------- */
bool search_path(const Unit *unit, V2 destination, float tolerance, int budget,
                 GcRequest *search) {
    /* Prepare the request. In formation mode, `stamp_parked` then writes cliffs into the grid
     * under the parked units of other squads, so that the searches go around them. */
    gc_init_request(search);
    V2 from = unit->position;
    int mover_class = unit->kind->mover_class;
    float clearance = get_search_clearance(unit);
    stamp_parked(&squads[unit->squad_index], from, destination);

    /* Run the quiet search, and the active search when the quiet one finds no way. */
    gc_start_search(search, GC_SEARCH_QUIET, from.x, from.y, destination.x, destination.y,
                    mover_class, clearance, tolerance);
    bool is_found = gc_update_search(search, budget) == GC_SEARCH_FOUND;
    if (!is_found) {
        gc_start_search(search, GC_SEARCH_ACTIVE, from.x, from.y, destination.x, destination.y,
                        mover_class, clearance, tolerance);
        is_found = gc_update_search(search, budget) == GC_SEARCH_FOUND;
    }

    /* Give the stamped cells their old values back. */
    unstamp_parked();
    return is_found;
}

/* ------------------------------------------------------------------------------------------------
 * `find_path` follows Mover::FindPath (0x50eb60). It searches a path from the unit to
 * `destination` with `search_path`.
 * Result: It returns true when a search found a path. It then puts the path in front of the
 *   remaining waypoints, makes the first point of the path the current waypoint and starts
 *   re-orienting. When both searches fail, it returns false and leaves the waypoints as they
 *   are.
 * --------------------------------------------------------------------------------------------- */
static bool find_path(Unit *unit, V2 destination, float tolerance, int budget) {
    GcRequest search;
    bool is_found = search_path(unit, destination, tolerance, budget, &search);

    /* Put the path in front of the waypoint list, as far as the list has room, and drive to the
     * first point of the path. The path leaves out the position of the unit itself. */
    if (is_found) {
        int free_count = MAX_WAYPOINTS - unit->waypoint_count;
        int point_count = gc_get_path(&search, path_points, free_count);
        size_t kept_bytes = (size_t)unit->waypoint_count * sizeof(V2);
        memmove(&unit->waypoints[point_count], &unit->waypoints[0], kept_bytes);
        for (int point_index = 0; point_index < point_count; point_index++)
            unit->waypoints[point_index] = get_pair_point(path_points, point_index);
        unit->waypoint_count += point_count;
        unit->is_reorienting = true;
        unit->waypoint = pop_waypoint(unit);
        update_leg(unit);
    }
    gc_free_request(&search);
    return is_found;
}

/* ------------------------------------------------------------------------------------------------
 * `set_move` follows Mover::SetMove (0x50e7e0). The last leg goes to `slot` without a search, as
 * in the game, so the search only has to reach the second point of the route, or its only point.
 * `set_move` sets `wait_seconds` to `WAIT_TIMER_AT_ORDER_SECONDS` again, as the game does,
 * after the start value that scenarios.c gives. The values `reach_last` and `reach_mid` stay as
 * they were, and `update_leg` picks the one that `reach` takes.
 * --------------------------------------------------------------------------------------------- */
void set_move(Unit *unit, const V2 *route, int route_count, V2 facing, V2 slot) {
    /* Take the route, and skip its first point when more points follow. */
    unit->blocker_index = NO_BLOCKER;
    unit->facing = facing;
    memcpy(&unit->waypoints[0], &route[0], (size_t)route_count * sizeof(V2));
    unit->waypoint_count = route_count;
    unit->waypoint = pop_waypoint(unit);
    if (unit->waypoint_count > 0)
        unit->waypoint = pop_waypoint(unit);

    /* Search a way to that point. The search can end within `reach_last` of the last point of the
     * route, or within `WAYPOINT_TOLERANCE_METERS` of another point. Then put `slot` in place of
     * the last point of the route, which can be the current waypoint. */
    float tolerance = unit->waypoint_count == 0 ? unit->reach_last : WAYPOINT_TOLERANCE_METERS;
    if (!find_path(unit, unit->waypoint, tolerance, SET_MOVE_SEARCH_BUDGET)) {
        unit->waypoint_count = 0;
        unit->waypoint = unit->position;
    } else if (unit->waypoint_count == 0) {
        unit->waypoint = slot;
    } else {
        unit->waypoints[unit->waypoint_count - 1] = slot;
    }

    /* Start driving. */
    update_leg(unit);
    unit->is_stopped = false;
    unit->is_reorienting = true;
    unit->wait_seconds = WAIT_TIMER_AT_ORDER_SECONDS;
}

/* ------------------------------------------------------------------------------------------------
 * `stop_at` follows the Mover function at 0x50e690, which Squad::Stop (0x504bd0) calls. It sets
 * `reach` to `reach_last` and `is_last_leg` to false, so that `update_mover` stops the unit when
 * it is within `reach_last` of `point`, the only waypoint left.
 * --------------------------------------------------------------------------------------------- */
void stop_at(Unit *unit, V2 point) {
    unit->waypoint = point;
    unit->is_stopped = false;
    unit->reach = unit->reach_last;
    unit->is_last_leg = false;
    unit->blocker_index = NO_BLOCKER;
    unit->waypoint_count = 0;
}

/* ------------------------------------------------------------------------------------------------
 * `get_unreached_distance_squared` returns a squared distance just beyond `reach`. After the
 * current waypoint changed, Update uses it in place of the real distance, so that the arrival
 * check of the same tick does not count the new waypoint as reached.
 * --------------------------------------------------------------------------------------------- */
static float get_unreached_distance_squared(const Unit *unit) {
    return unit->reach * unit->reach + 1;
}

/* ------------------------------------------------------------------------------------------------
 * `look_ahead` makes `waypoints[ahead_index]` the current waypoint when that waypoint exists and
 * the unit sees it, and drops the waypoints before it. It then sets `*distance_squared` to the
 * value of `get_unreached_distance_squared`.
 * --------------------------------------------------------------------------------------------- */
static void look_ahead(Unit *unit, int ahead_index, float *distance_squared) {
    bool has_waypoint = unit->waypoint_count > ahead_index;
    if (has_waypoint && is_in_sight(unit, unit->waypoints[ahead_index])) {
        for (int i = 0; i < ahead_index; i++)
            pop_waypoint(unit);
        unit->waypoint = pop_waypoint(unit);
        update_leg(unit);
        *distance_squared = get_unreached_distance_squared(unit);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `repath_if_hidden` searches a new path when the unit does not see its current waypoint. When
 * more waypoints follow, it drops the hidden one and searches a path to the next one. On the
 * last waypoint it searches a path to that waypoint, but it skips the whole check when `reach`
 * is at least `REACH_EPSILON_METERS` above `reach_last`, as the game does. TODO: confirm why the
 * game skips the check then.
 * Result: After a search, found or not, it sets `*distance_squared` to the value of
 *   `get_unreached_distance_squared`.
 * --------------------------------------------------------------------------------------------- */
static void repath_if_hidden(Unit *unit, float *distance_squared) {
    /* Check whether the waypoint is hidden. */
    float skip_reach = unit->reach_last + REACH_EPSILON_METERS;
    bool is_skipped = unit->waypoint_count == 0 && !(skip_reach > unit->reach);
    if (is_skipped || is_in_sight(unit, unit->waypoint))
        return;

    /* Search the new path. */
    if (unit->waypoint_count > 0)
        unit->waypoint = pop_waypoint(unit);
    find_path(unit, unit->waypoint, 0, REPATH_SEARCH_BUDGET);
    *distance_squared = get_unreached_distance_squared(unit);
}

/* ------------------------------------------------------------------------------------------------
 * `update_mover` follows Mover::Update (0x50eec0) for a unit that chases no target. A unit that
 * is not stopped looks ahead in three ticks out of four, and in the fourth it re-paths when its
 * waypoint is hidden. Then every unit checks arrival: within `reach` of the last waypoint it
 * stops, and within `reach` of another waypoint it goes on to the next one once it sees it.
 * --------------------------------------------------------------------------------------------- */
static void update_mover(Unit *unit) {
    /* Measure the distance to the current waypoint. In phases 0 to 2 of the look-ahead, check
     * `waypoints[phase]`, and in phase 3 re-path when the current waypoint is hidden. */
    float distance_squared = length_squared(sub(unit->waypoint, unit->position));
    if (!unit->is_stopped) {
        int phase = unit->look_ahead_phase;
        unit->look_ahead_phase++;
        if (phase < LOOK_AHEAD_PHASES) {
            look_ahead(unit, phase, &distance_squared);
        } else {
            unit->look_ahead_phase = 0;
            repath_if_hidden(unit, &distance_squared);
        }
    }

    /* Check arrival. */
    if (distance_squared <= unit->reach * unit->reach) {
        if (unit->waypoint_count == 0) {
            unit->is_stopped = true;
        } else if (is_in_sight(unit, unit->waypoints[0])) {
            unit->waypoint = pop_waypoint(unit);
            update_leg(unit);
        }
    }
}

/* ------------------------------------------------------------------------------------------------
 * `steer_to` follows SteerTo (0x511400). While `direction` is at most 90 degrees off the heading,
 * the steering is 1 - c^6 toward its side, where c is the cosine of the angle between them.
 * Further off, it is full lock.
 * --------------------------------------------------------------------------------------------- */
float steer_to(const Unit *unit, V2 direction) {
    /* `alignment` is |direction| * c, where c is the cosine of the angle between `direction`
     * and the heading, because the heading has length 1. So `cos_squared` is c^2. */
    bool is_left = direction.x * unit->heading.y < direction.y * unit->heading.x;
    float side = is_left ? -1.0f : 1.0f;
    float alignment = dot(direction, unit->heading);
    if (alignment >= 0) {
        float cos_squared = alignment * alignment / length_squared(direction);
        return side * (1 - cos_squared * cos_squared * cos_squared);
    }
    return side;
}

/* ------------------------------------------------------------------------------------------------
 * `steer_to_facing` calls `steer_to` with `facing`, and skips the call when `facing` has length 0,
 * because `steer_to` cannot divide by the length of a zero direction.
 * --------------------------------------------------------------------------------------------- */
float steer_to_facing(const Unit *unit) {
    if (length_squared(unit->facing) > 0)
        return steer_to(unit, unit->facing);
    return 0;
}

/* ------------------------------------------------------------------------------------------------
 * `compute_arrival_factor` follows the arrival factor (0x511280), which lowers the throttle on
 * the last leg so that the unit stops at its waypoint, the slot.
 * Result: Before the last leg it returns 1, or 0 when the unit is stopped. On the last leg it
 *   returns 0 within `reach` of the waypoint. Further out it finds the deceleration that stops
 *   the unit `ARRIVAL_STOP_DISTANCE_METERS` before the waypoint. While that needs less than
 *   `BRAKING_START_FRACTION` of the friction, it returns 1. Above that it falls from
 *   `friction / acceleration`, the throttle that just cancels friction, to 0 at the full
 *   friction.
 * Edge case: Closer than `ARRIVAL_STOP_DISTANCE_METERS`, but out of `reach`, the needed
 *   deceleration comes out negative, so it returns 1.
 * --------------------------------------------------------------------------------------------- */
static float compute_arrival_factor(const Unit *unit) {
    /* Return the factor at once before the last leg and within `reach` of the waypoint. */
    if (!unit->is_last_leg)
        return unit->is_stopped ? 0.0f : 1.0f;
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float distance = len(to_waypoint);
    if (distance < unit->reach)
        return 0;

    /* Find the deceleration, as a part of the friction, from
     *     closing_speed^2 = 2 * deceleration * (distance - ARRIVAL_STOP_DISTANCE_METERS) */
    float closing_speed = dot(mul(to_waypoint, 1 / distance), unit->velocity);
    float stopping_distance = distance - ARRIVAL_STOP_DISTANCE_METERS;
    float deceleration = closing_speed * closing_speed / (2 * stopping_distance);
    float braking_fraction = deceleration / unit->kind->friction;
    if (braking_fraction < BRAKING_START_FRACTION)
        return 1;

    /* Lower the factor in a straight line. `excess` grows from 0 at `BRAKING_START_FRACTION` to
     * 1 at the full friction. */
    float excess = 2 * (braking_fraction - BRAKING_START_FRACTION);
    if (excess < 1)
        return (1 - excess) * unit->kind->friction / unit->kind->acceleration;
    return 0;
}

/* ------------------------------------------------------------------------------------------------
 * `accelerate` follows the acceleration step at 0x511500.
 * --------------------------------------------------------------------------------------------- */
void accelerate(Unit *unit, float throttle) {
    float speed_gain = unit->kind->acceleration * TICK_SECONDS * throttle;
    unit->velocity = add(unit->velocity, mul(unit->heading, speed_gain));
}

/* ------------------------------------------------------------------------------------------------
 * `accelerate_toward_speed` takes the speed change per second that closes the gap to
 * `target_speed` within `time_constant_seconds`, and adds the friction, which the acceleration of
 * the unit must also cancel. It divides that total by the acceleration to get the throttle, and
 * limits the throttle to 0..1 before it calls `accelerate`.
 * --------------------------------------------------------------------------------------------- */
void accelerate_toward_speed(Unit *unit, float target_speed, float time_constant_seconds) {
    float speed_along_heading = dot(unit->velocity, unit->heading);
    float speed_change_rate = (target_speed - speed_along_heading) / time_constant_seconds;
    float needed_acceleration = speed_change_rate + unit->kind->friction;
    float throttle = needed_acceleration / unit->kind->acceleration;
    accelerate(unit, clamp_float(throttle, 0, 1));
}

/* ------------------------------------------------------------------------------------------------
 * `compute_game_throttle` returns the throttle of Steer for a unit that does not re-orient. While
 * `avoid_ticks` is 0, and the waypoint is further than `close_range` and behind the unit (below
 * `WAYPOINT_BEHIND_METERS` along its heading), it is `BEHIND_THROTTLE`. Otherwise it is the
 * arrival factor, but at least `MIN_THROTTLE`.
 * --------------------------------------------------------------------------------------------- */
static float compute_game_throttle(const Unit *unit) {
    /* Check for a waypoint behind the unit. */
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float close_range_squared = unit->kind->close_range * unit->kind->close_range;
    bool is_avoiding = unit->avoid_ticks >= 1;
    bool is_behind = dot(unit->heading, to_waypoint) < WAYPOINT_BEHIND_METERS;
    bool is_far = close_range_squared < length_squared(to_waypoint);
    if (!is_avoiding && is_behind && is_far)
        return BEHIND_THROTTLE;

    /* Otherwise take the arrival factor. */
    float throttle = compute_arrival_factor(unit);
    return throttle <= MIN_THROTTLE ? MIN_THROTTLE : throttle;
}

/* ------------------------------------------------------------------------------------------------
 * `compute_game_steering` returns the steering of Steer toward the current waypoint. It returns 0
 * when the unit stands on the waypoint. It also returns 0 when the unit does not re-orient,
 * `avoid_ticks` is 0, the waypoint is within `close_range`, and the waypoint is not ahead of the
 * velocity of the unit (their dot product is 0 or less). TODO: confirm the purpose of that last
 * case. re/PATHFINDING.md infers that it keeps units from circling a close waypoint.
 * --------------------------------------------------------------------------------------------- */
static float compute_game_steering(const Unit *unit) {
    /* Go straight on past a close waypoint that is not ahead, while the unit neither re-orients
     * nor avoids. */
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float distance_squared = length_squared(to_waypoint);
    float close_range_squared = unit->kind->close_range * unit->kind->close_range;
    bool is_steering_freely = !unit->is_reorienting && unit->avoid_ticks < 1;
    bool is_close = distance_squared <= close_range_squared;
    bool is_not_ahead = dot(to_waypoint, unit->velocity) <= 0;
    if (is_steering_freely && is_close && is_not_ahead)
        return 0;

    /* Otherwise steer toward the waypoint. */
    if (distance_squared > 0)
        return steer_to(unit, to_waypoint);
    return 0;
}

/* ------------------------------------------------------------------------------------------------
 * `end_reorient` ends re-orienting once the heading is within about 18 degrees of the direction
 * to the current waypoint.
 * --------------------------------------------------------------------------------------------- */
static void end_reorient(Unit *unit) {
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float distance_squared = length_squared(to_waypoint);
    if (distance_squared > 0) {
        V2 direction = mul(to_waypoint, 1 / sqrtf(distance_squared));
        if (dot(unit->heading, direction) > REORIENT_DONE_COSINE)
            unit->is_reorienting = false;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `steer` follows Steer (0x50ff40). It counts `wait_seconds` down. While that is above 0, the
 * unit steers to `wait_heading` and does not accelerate, so friction stops it. Otherwise a unit
 * that does not re-orient accelerates with `compute_game_throttle`, and a unit that re-orients
 * checks whether that ends.
 * Result: It returns the steering of `compute_game_steering`, or the steering toward
 *   `wait_heading` while the unit waits.
 * --------------------------------------------------------------------------------------------- */
static float steer(Unit *unit) {
    /* Hold the wait heading while the unit waits. */
    unit->wait_seconds -= TICK_SECONDS;
    if (unit->wait_seconds > 0)
        return steer_to(unit, unit->wait_heading);

    /* Otherwise drive toward the waypoint. */
    if (!unit->is_reorienting)
        accelerate(unit, compute_game_throttle(unit));
    else
        end_reorient(unit);
    return compute_game_steering(unit);
}

/* ------------------------------------------------------------------------------------------------
 * `query_corridor` follows the collision-map query (0x4fef30) that AvoidUnits makes. It finds
 * the other units in a corridor that starts at the center of the unit and runs along
 * `direction`, which has length 1:
 *
 *            |<------------- `length` ------------->|
 *            +--------------------------------------+   -
 *            |                                      |   | `half_width`
 *     (unit) o------------- `direction` ----------->|   -
 *            |                                      |   | `half_width`
 *            +--------------------------------------+   -
 *
 * Another unit counts when its center lies ahead of the unit along `direction` and within
 * `half_width` of the line, and the measure of the game for the distance to its body
 * (`near_edge_distance`) is at most `length`. `half_width` is the widened radius of the unit
 * plus the radius of the other unit.
 * Result: It writes the indices of those units into `hit_indices`, in unit order, and returns
 *   their count.
 * Edge case: A zero `direction` finds no unit.
 * --------------------------------------------------------------------------------------------- */
static int query_corridor(int unit_index, V2 direction, float length, int *hit_indices) {
    const Unit *unit = &units[unit_index];
    float widened_radius = unit->kind->radius * QUERY_RADIUS_FACTOR * QUERY_RADIUS_MARGIN_FACTOR;
    int hit_count = 0;
    for (int other_index = 0; other_index < unit_count; other_index++) {
        if (other_index == unit_index)
            continue;

        /* Leave out units whose center is not ahead. */
        const Unit *other = &units[other_index];
        V2 to_other = sub(other->position, unit->position);
        float distance_ahead = dot(to_other, direction);
        if (distance_ahead <= 0)
            continue;

        /* Keep the units within the corridor. `lateral_squared` is the squared distance from the
         * center of the other unit to the center line of the corridor. */
        V2 closest = add(unit->position, mul(direction, distance_ahead));
        float lateral_squared = length_squared(sub(other->position, closest));
        float half_width = widened_radius + other->kind->radius;
        float other_radius_squared = other->kind->radius * other->kind->radius;
        float near_edge_distance = distance_ahead - sqrtf(other_radius_squared + lateral_squared);
        bool is_within_width = lateral_squared <= half_width * half_width;
        if (is_within_width && near_edge_distance <= length) {
            hit_indices[hit_count] = other_index;
            hit_count++;
        }
    }
    return hit_count;
}

/* ------------------------------------------------------------------------------------------------
 * `turn_on_last_leg` follows the first part of AvoidUnits. On the last leg, when the slot is
 * more than 45.6 degrees off the heading, the unit starts to re-orient and forgets its blocker,
 * unless an enemy unit is in the way. An enemy is in the way when it is in the corridor toward
 * the slot, which ends `SLOT_QUERY_SHORTFALL_METERS` short of the slot, the slot lies outside its
 * body, and it does not touch the unit. Friendly units are never in the way.
 * Inputs: `direction` (length 1, or zero on the slot) and `distance` lead from the unit to its
 *   current waypoint, which is the slot on the last leg.
 * Result: It returns true when the unit starts to re-orient.
 * --------------------------------------------------------------------------------------------- */
static bool turn_on_last_leg(int unit_index, V2 direction, float distance) {
    /* Leave out units that are not on the last leg, or that face the slot. */
    Unit *unit = &units[unit_index];
    if (!unit->is_last_leg || dot(direction, unit->heading) >= LAST_LEG_TURN_COSINE)
        return false;

    /* Look for an enemy in the way. `hit_indices[0..hit_count)` holds the units in the
     * corridor. */
    int hit_indices[MAX_UNITS];
    float query_length = distance - SLOT_QUERY_SHORTFALL_METERS;
    int hit_count = query_corridor(unit_index, direction, query_length, hit_indices);
    for (int hit_index = 0; hit_index < hit_count; hit_index++) {
        const Unit *other = &units[hit_indices[hit_index]];
        float combined_radius = unit->kind->radius + other->kind->radius;
        float other_radius_squared = other->kind->radius * other->kind->radius;
        float slot_distance_squared = length_squared(sub(other->position, unit->waypoint));
        float distance_squared = length_squared(sub(other->position, unit->position));
        bool is_enemy = other->team != unit->team;
        bool is_off_slot = slot_distance_squared - other_radius_squared > 0;
        bool is_apart = distance_squared - combined_radius * combined_radius > 0;
        if (is_enemy && is_off_slot && is_apart)
            return false;
    }

    /* Turn on the spot. */
    unit->is_reorienting = true;
    unit->blocker_index = NO_BLOCKER;
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `find_blocker` picks the blocker among the units in `hit_indices[0..hit_count)`. A tangent
 * runs from the center of the unit to the point T where it touches the body of another unit, at a
 * right angle to the radius of that unit:
 *
 *                         T
 *            tangent   .'   '.   radius of the other unit
 *                   .'        '.
 *          unit  o'--------------O  center of the other unit
 *                     distance
 *
 * So its squared length, `tangent_squared`, is the squared distance between the centers less the
 * squared radius of the other unit. The blocker is the unit with the shortest tangent among the
 * units whose body does not hold the center of the unit, and its `tangent_squared` must be below
 * `MAX_BLOCKER_TANGENT_SQUARED`. On the last leg, enemies whose body holds the slot are left out.
 * Result: It returns the index of the blocker, or `NO_BLOCKER` when no unit qualifies.
 * --------------------------------------------------------------------------------------------- */
static int find_blocker(const Unit *unit, const int *hit_indices, int hit_count) {
    int best_index = NO_BLOCKER;
    float best_tangent_squared = MAX_BLOCKER_TANGENT_SQUARED;
    for (int hit_index = 0; hit_index < hit_count; hit_index++) {
        /* Measure the tangent to the body of the other unit. */
        int other_index = hit_indices[hit_index];
        const Unit *other = &units[other_index];
        float other_radius_squared = other->kind->radius * other->kind->radius;
        float distance_squared = length_squared(sub(other->position, unit->position));
        float tangent_squared = distance_squared - other_radius_squared;

        /* Keep the shortest tangent among the units that count. */
        float slot_distance_squared = length_squared(sub(other->position, unit->waypoint));
        bool is_friend = other->team == unit->team;
        bool is_off_slot = slot_distance_squared - other_radius_squared > 0;
        bool is_relevant = !unit->is_last_leg || is_friend || is_off_slot;
        bool is_closer = tangent_squared < best_tangent_squared && tangent_squared >= 0;
        if (is_relevant && is_closer) {
            best_index = other_index;
            best_tangent_squared = tangent_squared;
        }
    }
    return best_index;
}

/* ------------------------------------------------------------------------------------------------
 * `end_wait` ends a wait early: a positive `wait_seconds` drops to 0.
 * --------------------------------------------------------------------------------------------- */
static void end_wait(Unit *unit) {
    if (unit->wait_seconds > 0)
        unit->wait_seconds = 0;
}

/* ------------------------------------------------------------------------------------------------
 * `get_look_distance` returns the length of the corridor in which AvoidUnits looks for
 * blockers.
 * Inputs: `direction` and `distance` lead from the unit to its current waypoint.
 * --------------------------------------------------------------------------------------------- */
static float get_look_distance(const Unit *unit, V2 direction, float distance) {
    /* Take a part of the distance to the waypoint, but at most `MAX_LOOK_RADII` radii. */
    float look_distance = distance * LOOK_DISTANCE_FACTOR;
    float max_look_distance = unit->kind->radius * MAX_LOOK_RADII;
    if (look_distance > max_look_distance)
        look_distance = max_look_distance;

    /* Shorten it when the unit faces away from the waypoint, but keep at least `MIN_LOOK_RADII`
     * radii. */
    if (dot(direction, unit->heading) < 0)
        look_distance *= BEHIND_LOOK_FACTOR;
    float min_look_distance = unit->kind->radius * MIN_LOOK_RADII;
    if (look_distance < min_look_distance)
        look_distance = min_look_distance;
    return look_distance;
}

/* ------------------------------------------------------------------------------------------------
 * `switch_blocker` makes the unit at `blocker_index` the blocker of the unit, and ends a wait
 * when the blocker changes.
 * Result: It returns true when the unit must pick its swerve side anew: when it had no blocker,
 *   or when the new blocker is at least `NEW_BLOCKER_RADII` times the radius of the unit plus
 *   the radii of both blockers away from the old one. Otherwise it returns false.
 * --------------------------------------------------------------------------------------------- */
static bool switch_blocker(Unit *unit, int blocker_index) {
    if (blocker_index == unit->blocker_index)
        return false;
    end_wait(unit);

    /* Pick a new side unless the new blocker is close to the old one. */
    bool is_new_side = true;
    if (unit->blocker_index >= 0) {
        const Unit *previous_blocker = &units[unit->blocker_index];
        const Unit *blocker = &units[blocker_index];
        float own_spacing = unit->kind->radius * NEW_BLOCKER_RADII;
        float spacing = own_spacing + previous_blocker->kind->radius + blocker->kind->radius;
        V2 between_blockers = sub(previous_blocker->position, blocker->position);
        float blocker_distance_squared = length_squared(between_blockers);
        is_new_side = spacing * spacing <= blocker_distance_squared;
    }
    unit->blocker_index = blocker_index;
    return is_new_side;
}

/* ------------------------------------------------------------------------------------------------
 * `swerve` makes the unit swerve around `blocker` and sets `avoid_ticks` to `AVOID_TICKS`. With
 * `is_new_side`, it first picks the side away from the blocker. `swerve_side` becomes
 * `SWERVE_LEFT` when the blocker is to the right of the heading or straight ahead, and
 * `SWERVE_RIGHT` otherwise. When the blocker has this unit as its own blocker, the unit takes the
 * `swerve_side` of the blocker instead.
 * Result: It returns `SWERVE_STEERING` toward the swerve side, negative to the left, or 0 while
 *   the unit still waits.
 * --------------------------------------------------------------------------------------------- */
static float swerve(int unit_index, const Unit *blocker, bool is_new_side) {
    /* Pick the side. */
    Unit *unit = &units[unit_index];
    if (is_new_side) {
        V2 to_blocker = sub(blocker->position, unit->position);
        bool is_blocker_right = to_blocker.y * unit->heading.x <= to_blocker.x * unit->heading.y;
        unit->swerve_side = is_blocker_right ? SWERVE_LEFT : SWERVE_RIGHT;
    }
    if (blocker->blocker_index == unit_index)
        unit->swerve_side = blocker->swerve_side;

    /* Swerve unless the unit still waits. */
    unit->avoid_ticks = AVOID_TICKS;
    if (unit->wait_seconds <= 0) {
        unit->swerve_seconds += TICK_SECONDS;
        unit->is_swerving = true;
        return unit->swerve_side == SWERVE_LEFT ? -SWERVE_STEERING : SWERVE_STEERING;
    }
    return 0;
}

/* ------------------------------------------------------------------------------------------------
 * `avoid_units` follows AvoidUnits (0x5101a0). It does nothing while the unit re-orients, and
 * on the last leg `turn_on_last_leg` can first make the unit turn on the spot. Otherwise
 * it looks for a blocker in the corridor along the heading, judged from the current positions
 * only. A blocker drives on when it is not stopped, does not wait, and has `avoid_ticks` of at
 * most `DRIVING_ON_AVOID_TICKS`. The unit then reacts in one of three ways:
 * - When the blocker is a friendly unit that drives on, the unit keeps going.
 * - When `wait_seconds` is at least `SWERVE_AFTER_WAIT_SECONDS` (the unit waits, or its last wait
 *   ended at most 2 s ago), or the blocker stopped, or the blocker is not faster, the unit
 *   swerves with `swerve`.
 * - Otherwise the unit waits `WAIT_SECONDS` for the blocker with its heading held.
 * Result: It returns the steering that `steer_unit` adds to the steering of Steer: 0, or
 *   `SWERVE_STEERING` toward the swerve side.
 * --------------------------------------------------------------------------------------------- */
static float avoid_units(int unit_index) {
    /* Count the side cooldown down. Do nothing while the unit re-orients, or when it starts to
     * turn on the spot. */
    Unit *unit = &units[unit_index];
    unit->side_cooldown_seconds -= TICK_SECONDS;
    if (unit->is_reorienting)
        return 0;
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float distance = len(to_waypoint);
    V2 direction = v2(0, 0);
    if (distance > 0)
        direction = mul(to_waypoint, 1 / distance);
    if (turn_on_last_leg(unit_index, direction, distance))
        return 0;

    /* Find the blocker. Without one, end a wait. `hit_indices[0..hit_count)` holds the units in
     * the corridor along the heading. */
    int hit_indices[MAX_UNITS];
    float look_distance = get_look_distance(unit, direction, distance);
    int hit_count = query_corridor(unit_index, unit->heading, look_distance, hit_indices);
    if (hit_count == 0) {
        unit->blocker_index = NO_BLOCKER;
        end_wait(unit);
        return 0;
    }
    int blocker_index = find_blocker(unit, hit_indices, hit_count);
    if (blocker_index == NO_BLOCKER) {
        end_wait(unit);
        return 0;
    }
    bool is_new_side = switch_blocker(unit, blocker_index);

    /* Keep going when the blocker is a friendly unit that drives on. */
    const Unit *blocker = &units[blocker_index];
    bool is_friend = blocker->team == unit->team;
    bool is_blocker_moving = !blocker->is_stopped && blocker->wait_seconds <= 0;
    bool is_driving_on = is_blocker_moving && blocker->avoid_ticks <= DRIVING_ON_AVOID_TICKS;
    if (is_friend && is_driving_on)
        return 0;

    /* Swerve when the unit waited recently, or the blocker stopped or is not faster. */
    bool has_waited_recently = unit->wait_seconds >= SWERVE_AFTER_WAIT_SECONDS;
    bool is_blocker_not_faster = blocker->kind->max_speed <= unit->kind->max_speed;
    if (has_waited_recently || blocker->is_stopped || is_blocker_not_faster)
        return swerve(unit_index, blocker, is_new_side);

    /* Otherwise wait for the faster blocker. */
    unit->wait_seconds = WAIT_SECONDS;
    unit->wait_heading = unit->heading;
    unit->wait_count++;
    return 0;
}

/* ------------------------------------------------------------------------------------------------
 * `steer_unit` returns the steering of the unit for this tick, as the steering part of
 * Mover::Tick (0x50e5b0) does, with formation mode added. The functions that it calls also set
 * the throttle.
 * --------------------------------------------------------------------------------------------- */
static float steer_unit(int unit_index, bool is_forming) {
    /* Steer a moving unit by formation following, or by Steer in the game plus AvoidUnits. */
    Unit *unit = &units[unit_index];
    if (!unit->is_stopped) {
        if (is_forming)
            return steer_in_formation(unit_index);
        float steering = steer(unit);
        return steering + avoid_units(unit_index);
    }

    /* In formation mode, let a stopped unit make room: for squad mates while its squad moves,
     * and, while `is_yield_on` is true, for any moving unit while its squad neither moves nor
     * searches. */
    int squad_state = squads[unit->squad_index].state;
    bool is_in_formation_mode = is_formation_mode();
    if (is_in_formation_mode && squad_state == SQUAD_MOVING)
        return yield_mate(unit_index);
    if (is_in_formation_mode && is_yield_on && squad_state != SQUAD_SEARCHING)
        return yield_parked(unit_index);

    /* Otherwise turn to the final facing. */
    return steer_to_facing(unit);
}

/* ------------------------------------------------------------------------------------------------
 * `tick_mover` follows Mover::Tick (0x50e5b0). It clears the flags of the tick, then decides
 * whether formation following steers the unit (`is_forming`: formation mode, and the squad is
 * not idle). Without it, `update_mover` follows the waypoints first. `steer_unit` then gives the
 * steering, and the unit turns, slows by friction, moves and resolves collisions, in the order
 * of the game.
 * --------------------------------------------------------------------------------------------- */
void tick_mover(int unit_index) {
    /* Clear the flags of this tick, and count `avoid_ticks` down, not below 0. */
    Unit *unit = &units[unit_index];
    unit->is_swerving = false;
    unit->is_adjusting = false;
    unit->avoid_ticks--;
    if (unit->avoid_ticks < 0)
        unit->avoid_ticks = 0;

    /* A unit of a squad that moves or searches forgets its `home`, so that `yield_parked`
     * records a new one once the squad parks again. */
    const Squad *squad = &squads[unit->squad_index];
    if (is_squad_busy(squad))
        unit->has_home = false;

    /* Follow the waypoints unless formation following steers the unit. Then steer, turn, slow
     * by friction and move. */
    bool is_forming = is_formation_mode() && squad->state != SQUAD_IDLE;
    if (!is_forming)
        update_mover(unit);
    float steering = steer_unit(unit_index, is_forming);
    turn(unit, steering);
    apply_friction(unit);
    integrate(unit_index);
}
