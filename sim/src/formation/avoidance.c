/* Unit avoidance for formation following. steering.c calls `compute_orca_velocity` every tick
 * for each moving unit. movement.c calls `yield_mate` and `yield_parked`, the two ways in which a
 * unit that stands still makes room for others.
 *
 * Fields of `Unit` that this file uses:
 *   `yield_mate` and `yield_parked` set `is_adjusting` and add to `adjust_seconds` (through
 *   `step_aside`, which the metrics count as avoidance). `yield_mate` sets `has_yielded` while the
 *   unit steps aside and clears it, with `is_stopped`, when the unit must drive back to its slot.
 *   `yield_parked` sets `home` and `has_home` once, at the first call, to remember where the unit
 *   parked. `held_seconds` and `avoidance_off_seconds` are not used here: steering.c sets them
 *   and decides when this avoidance is off.
 *
 * Method:
 *   Each unit has a velocity it wants, `preferred`. For each other unit
 * close enough to matter, some velocities make the two units touch within the next
 * `ORCA_HORIZON_SECONDS`. Each such unit forbids the velocities on one side of a straight line,
 * drawn in the plane of all possible velocities, so that the two units no longer touch within
 * that time if both keep to their allowed sides. The unit then takes the velocity closest to
 * `preferred` that is on the allowed side of every line and not faster than its top speed. This
 * method is ORCA (optimal reciprocal collision avoidance, from the RVO2 library). "Reciprocal"
 * means that two moving units share the change: each one makes half of it.
 *
 * Touch test:
 *   This is how two units can touch. `relative_position` is the position of the other unit minus
 * the position of this unit, `relative_velocity` is the velocity of this unit minus that of the
 * other unit, and `combined_radius` is the sum of their radii and `ORCA_MARGIN_METERS`. After a
 * time t, the center of the other unit is at `relative_position - t * relative_velocity` from the
 * center of this unit, so they touch when
 *     |relative_position - t * relative_velocity| < combined_radius
 * and, after dividing by t, when
 *     |relative_velocity - relative_position / t| < combined_radius / t
 * where |a| is the length of a. So the units touch at time t when `relative_velocity` lies in the
 * disc of radius `combined_radius / t` around `relative_position / t`. For all t up to a time
 * limit, `time_seconds`, these discs fill a cone. The cone starts at zero velocity (0 in the
 * sketch) and widens toward `relative_position`. The disc for t = `time_seconds`, the cut-off
 * disc, closes the cone at the side of 0, because a relative velocity between 0 and that disc
 * makes the units touch only after the time limit. The two straight edges of the cone are its
 * legs.
 *
 *                                  ..''  left leg
 *                             ..''
 *                        ..''           forbidden: the units touch
 *                   ..''  .--.          within `time_seconds`
 *              ..''     /      \
 *    0  ..''''         |  disc  |  - - - - - > toward `relative_position`
 *          ''..         \      /
 *              ''..       '--'
 *                   ''..
 *                        ''..
 *                             ''..  right leg
 *
 * Each line runs along the edge of this forbidden region, at the edge point closest to
 * `relative_velocity`. `det2`, the cross product that sim.h describes, tells the sides of a line
 * apart.
 *
 * Extra rules:
 *   ORCA here has these additions to the method of RVO2:
 * - A unit that is stopped, or turns on the spot without moving, cannot dodge. The other unit
 *   then makes the whole change.
 * - Squad mates count only when they are about to touch, because they follow nearly the same
 *   lines.
 * - Walls do not get lines, because the routes of the game run right along blocked cells, and
 *   lines for walls push units off those routes. Instead, the callers use a new velocity only
 *   when the straight line along it for the next second is clear for the mover class of the
 *   unit, which sets the cells that block the unit. The physics keeps units out of blocked
 *   cells.
 * - `stamp_parked` in squad.c makes routes go around parked squads, and collisions push only the
 *   moving unit. */
#include <string.h>

#include "sim.h"
#include "formation.h"

#define ORCA_HORIZON_SECONDS 2.0f        /* Units must stay apart this long at new velocities. */
#define ORCA_MARGIN_METERS 0.5f          /* Units keep this extra gap between their bodies. */
#define ORCA_MAX_LINES 64                /* Each other unit adds a line, so `MAX_UNITS` fit. */
#define MATE_AVOID_RANGE_METERS 2.0f     /* Mates count at body gaps up to this plus the margin. */
#define SHARED_EFFORT 0.5f               /* Each of two moving units makes half of the change. */
#define FULL_EFFORT 1.0f                 /* A unit makes all of it when the other cannot dodge. */
#define PARALLEL_EPSILON 1e-5f           /* Lines are parallel at this cross product or less. */
#define LENGTH_EPSILON 1e-6f             /* Vectors this short or shorter have no direction. */
#define SPEED_TIME_CONSTANT_SECONDS 0.3f /* A unit making room reaches its speed in about this. */
#define MIN_YIELD_SPEED 1.0f             /* Room is made only for avoidance faster than this. */
#define YIELD_SPEED_FRACTION 0.3f        /* Parked units dodge at up to this part of top speed. */
#define YIELD_MAX_METERS 3.5f            /* They dodge only to points this close to `home`. */
#define HOME_REACHED_METERS 1.0f         /* A parked unit this close to `home` is back. */

/* A `Line` is a straight line in the plane of velocities. The velocities to the left of the line
 * through `point` along `direction`, as seen when looking along `direction`, are allowed, and the
 * others are forbidden. `direction` has length 1. A velocity keeps the line when it lies on the
 * allowed side, and it breaks the line otherwise. */
typedef struct {
    V2 point;
    V2 direction;
} Line;

/* ------------------------------------------------------------------------------------------------
 * `get_line_violation` returns how far `velocity` lies on the forbidden side of `line`, measured
 * straight across the line. The value is negative on the allowed side.
 * --------------------------------------------------------------------------------------------- */
static float get_line_violation(const Line *line, V2 velocity) {
    V2 from_velocity = sub(line->point, velocity);
    return det2(line->direction, from_velocity);
}

/* Finding the velocity is a small linear program (a search for the best point that keeps a set of
 * straight-line limits), done as in RVO2. The three functions below follow its `linearProgram1`
 * to `linearProgram3`. The goal is the velocity closest to `preferred` that is on the allowed
 * side of every line and not faster than `max_speed`. With `is_direction_only` the goal is the
 * velocity that goes furthest in the direction of `preferred` instead, and then `preferred` must
 * have length 1.
 * `solve_linear_program_2` takes the lines one at a time. When the velocity so far keeps the next
 * line, nothing changes. When it breaks the line, the new velocity must lie on that line, and
 * `solve_linear_program_1` searches along it for the best point that keeps all earlier lines.
 * When no velocity keeps every line, `solve_linear_program_3` looks for the velocity whose worst
 * break of a line is as small as possible. */

/* ------------------------------------------------------------------------------------------------
 * `solve_linear_program_1` finds the best velocity on line `line_index` that keeps the lines
 * before it.
 *
 *               .-'''''-.
 *      - - - -x===========x- - - ->  the line, along `direction`
 *            |      0      |
 *             '-.._____..-'
 *
 *   The circle has radius `max_speed` around zero velocity (0). The line crosses it at the two
 *   points x, and only the stretch between them (===) is within `max_speed`.
 * Result: It returns true and puts into `result` the point of the line closest to `preferred`, or
 *   with `is_direction_only` the point furthest in the direction of `preferred`, among the points
 *   that are within `max_speed` of zero velocity and keep the lines before it.
 * Edge case: It returns false and leaves `result` as it was when no point qualifies.
 * --------------------------------------------------------------------------------------------- */
static bool solve_linear_program_1(const Line *lines, int line_index, float max_speed,
                                   V2 preferred, bool is_direction_only, V2 *result) {
    /* Find the stretch of the line within `max_speed` of zero velocity. Its points are
     * `point + along * direction`, and its ends, `min_along` and `max_along`, are the roots of
     *     |point + along * direction|^2 = max_speed^2
     *     along^2 + 2 * projection * along + |point|^2 - max_speed^2 = 0
     * A negative `discriminant` means that the whole line is faster than `max_speed`. */
    const Line *line = &lines[line_index];
    float projection = dot(line->point, line->direction);
    float point_length_squared = length_squared(line->point);
    float discriminant = projection * projection + max_speed * max_speed - point_length_squared;
    if (discriminant < 0)
        return false;
    float root = sqrtf(discriminant);
    float min_along = -projection - root;
    float max_along = -projection + root;

    /* Cut the stretch to the allowed side of each earlier line. At `along`, the violation of an
     * earlier line is `along * denominator - numerator`, so it allows `along <= crossing` when
     * `denominator` is positive, and `along >= crossing` when it is negative. A parallel earlier
     * line allows all of this line when `numerator` is not negative, and none of it otherwise. */
    for (int earlier_index = 0; earlier_index < line_index; earlier_index++) {
        const Line *earlier = &lines[earlier_index];
        float denominator = det2(line->direction, earlier->direction);
        float numerator = det2(earlier->direction, sub(line->point, earlier->point));
        if (fabsf(denominator) <= PARALLEL_EPSILON) {
            if (numerator < 0)
                return false;
            continue;
        }
        float crossing = numerator / denominator;
        if (denominator >= 0) {
            if (crossing < max_along)
                max_along = crossing;
        } else if (crossing > min_along) {
            min_along = crossing;
        }
        if (min_along > max_along)
            return false;
    }

    /* Pick the best point of the stretch. */
    float along;
    if (is_direction_only) {
        float alignment = dot(preferred, line->direction);
        along = alignment > 0 ? max_along : min_along;
    } else {
        float preferred_along = dot(line->direction, sub(preferred, line->point));
        along = clamp_float(preferred_along, min_along, max_along);
    }
    *result = add(line->point, mul(line->direction, along));
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `solve_linear_program_2` finds the best velocity for `lines[0..line_count)` and takes the lines
 * one at a time.
 * Result: It returns `line_count` and puts the best velocity for all lines into `result`.
 * Edge case: When no velocity keeps line `line_index` and the lines before it,
 *   `solve_linear_program_2` returns `line_index` and leaves the best velocity for the lines
 *   before it in `result`.
 * --------------------------------------------------------------------------------------------- */
static int solve_linear_program_2(const Line *lines, int line_count, float max_speed, V2 preferred,
                                  bool is_direction_only, V2 *result) {
    /* Start from `preferred`, cut to `max_speed`, or with `is_direction_only`, from the fastest
     * velocity in its direction. */
    if (is_direction_only)
        *result = mul(preferred, max_speed);
    else if (length_squared(preferred) > max_speed * max_speed)
        *result = mul(preferred, max_speed / len(preferred));
    else
        *result = preferred;

    /* Move onto each line that the velocity so far breaks. */
    for (int line_index = 0; line_index < line_count; line_index++) {
        if (get_line_violation(&lines[line_index], *result) <= 0)
            continue;
        V2 previous = *result;
        bool is_solved = solve_linear_program_1(lines, line_index, max_speed, preferred,
                                                is_direction_only, result);
        if (!is_solved) {
            *result = previous;
            return line_index;
        }
    }
    return line_count;
}

/* ------------------------------------------------------------------------------------------------
 * `solve_linear_program_3` finds the velocity whose worst break of any line is as small as
 * possible. Callers use it when `solve_linear_program_2` fails at `first_failed_index`.
 * Inputs: The first `hard_count` lines are hard lines, which must not be broken at all, and the
 *   others are soft lines. RVO2 uses hard lines for walls, and the caller here passes 0. `result`
 *   must hold the best velocity for the lines before `first_failed_index`.
 * Result: `result` becomes the velocity with the smallest worst break.
 * --------------------------------------------------------------------------------------------- */
static void solve_linear_program_3(const Line *lines, int line_count, int hard_count,
                                   int first_failed_index, float max_speed, V2 *result) {
    /* `projected` holds the limits of each search: first copies of the hard lines, then one line
     * for each earlier soft line. */
    static Line projected[ORCA_MAX_LINES];
    float worst_violation = 0;
    for (int line_index = first_failed_index; line_index < line_count; line_index++) {
        /* Skip a line that the velocity breaks no more than the worst break so far. */
        const Line *line = &lines[line_index];
        if (get_line_violation(line, *result) <= worst_violation)
            continue;

        /* For each earlier soft line `other`, add the line of velocities that break `other` and
         * this line by the same amount. Its allowed side holds the velocities that break `other`
         * less than this line. It runs through the crossing of the two lines, or for parallel
         * lines that face opposite ways, through the point halfway between them. Its direction is
         * halfway between `other->direction` and the reverse of `line->direction`. Parallel lines
         * that face the same way give no limit. */
        int projected_count = hard_count;
        memcpy(projected, lines, (size_t)hard_count * sizeof(Line));
        for (int other_index = hard_count; other_index < line_index; other_index++) {
            const Line *other = &lines[other_index];
            float determinant = det2(line->direction, other->direction);
            V2 point;
            if (fabsf(determinant) <= PARALLEL_EPSILON) {
                if (dot(line->direction, other->direction) > 0)
                    continue;
                point = mul(add(line->point, other->point), 0.5f);
            } else {
                V2 between = sub(line->point, other->point);
                float along = det2(other->direction, between) / determinant;
                point = add(line->point, mul(line->direction, along));
            }
            V2 direction_change = sub(other->direction, line->direction);
            V2 direction = mul(direction_change, 1 / len(direction_change));
            projected[projected_count] = (Line){
                .point = point,
                .direction = direction,
            };
            projected_count++;
        }

        /* Within those limits, pick the velocity that goes furthest toward the allowed side of
         * this line. Only rounding errors can make that search fail, and then the velocity stays
         * as it was. */
        V2 previous = *result;
        V2 outward = v2(-line->direction.y, line->direction.x);
        bool is_direction_only = true;
        int failed_index = solve_linear_program_2(projected, projected_count, max_speed, outward,
                                                  is_direction_only, result);
        if (failed_index < projected_count)
            *result = previous;
        worst_violation = get_line_violation(line, *result);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `get_from_cutoff_center` returns the relative velocity as seen from the center of the cut-off
 * disc for `time_seconds`.
 * --------------------------------------------------------------------------------------------- */
static V2 get_from_cutoff_center(V2 relative_position, V2 relative_velocity, float time_seconds) {
    V2 cutoff_center = mul(relative_position, 1 / time_seconds);
    return sub(relative_velocity, cutoff_center);
}

/* ------------------------------------------------------------------------------------------------
 * `get_cutoff_correction` finds the point on the edge of the cut-off disc that is closest to the
 * relative velocity.
 * Inputs: `from_center` is the relative velocity minus the center of the disc.
 * Results: `direction` is the direction of the edge at that point, with length 1 and the disc on
 *   its right, so that the allowed side, on its left, is outside the disc. `correction` is the
 *   change that moves the relative velocity onto that point.
 * Edge case: When `from_center` is too short to have a direction, the relative velocity is at the
 *   center and all edge points are equally close. `get_cutoff_correction` then takes the one in
 *   direction (1, 0) from the center.
 * --------------------------------------------------------------------------------------------- */
static void get_cutoff_correction(V2 from_center, float cutoff_radius, V2 *direction,
                                  V2 *correction) {
    float from_center_length = len(from_center);
    V2 from_center_unit = v2(1, 0);
    if (from_center_length > LENGTH_EPSILON)
        from_center_unit = mul(from_center, 1 / from_center_length);
    *direction = v2(from_center_unit.y, -from_center_unit.x);
    *correction = mul(from_center_unit, cutoff_radius - from_center_length);
}

/* ------------------------------------------------------------------------------------------------
 * `get_cone_leg_direction` returns the direction of one of the legs of the cone, with length 1.
 *
 *                            _.-''T
 *                       _.-''      '.
 *                  _.-''            '.  combined_radius
 *       leg   _.-''                  '.
 *        _.-''                        '.
 *       0-------------------------------o  relative_position
 *
 *   The sketch shows the left leg, from 0 through T, and `relative_position` (o).
 * Result: A leg touches the circle of radius `combined_radius` around `relative_position` at T,
 *   at a right angle to the radius there. So `leg_length`, the distance from 0 to T, follows
 *   from `leg_length^2 + combined_radius^2 = |relative_position|^2`. The direction of the leg is
 *   `relative_position` turned by the angle a, with
 *   sin(a) = combined_radius / |relative_position| and cos(a) = leg_length / |relative_position|,
 *   then divided by |relative_position| for length 1. The sine and cosine bring one division by
 *   |relative_position| and the length another, so the code divides by `distance_squared`.
 *   The left leg is turned to the left as seen from 0 looking toward `relative_position`, and
 *   points away from 0. The right leg points toward 0, so that the velocities outside the cone
 *   lie to the left of both legs.
 * --------------------------------------------------------------------------------------------- */
static V2 get_cone_leg_direction(V2 relative_position, float combined_radius, bool is_left_leg) {
    float distance_squared = length_squared(relative_position);
    float leg_length = sqrtf(distance_squared - combined_radius * combined_radius);
    float side = is_left_leg ? 1.0f : -1.0f;
    float x = relative_position.x;
    float y = relative_position.y;
    float leg_x = x * leg_length - side * y * combined_radius;
    float leg_y = side * x * combined_radius + y * leg_length;
    return mul(v2(leg_x, leg_y), side / distance_squared);
}

/* ------------------------------------------------------------------------------------------------
 * `make_orca_line` makes the line that one other unit puts on the velocity of this unit.
 * Inputs: `effort` is the part of the change that this unit makes, and `velocity` is the velocity
 *   of this unit itself.
 * Result: The line runs along the edge of the forbidden region at the edge point closest to
 *   `relative_velocity`. It goes through `velocity` plus `effort` times the change that moves
 *   `relative_velocity` onto that edge point.
 * Edge case: When the units already overlap, the edge is that of the cut-off disc for a time limit
 *   of one tick, so that they move apart by the next tick.
 * --------------------------------------------------------------------------------------------- */
static Line make_orca_line(V2 relative_position, V2 relative_velocity, float combined_radius,
                           float effort, V2 velocity) {
    /* Find the cut-off disc. */
    float combined_radius_squared = combined_radius * combined_radius;
    bool is_apart = length_squared(relative_position) > combined_radius_squared;
    float time_seconds = is_apart ? ORCA_HORIZON_SECONDS : TICK_SECONDS;
    V2 from_center = get_from_cutoff_center(relative_position, relative_velocity, time_seconds);
    float cutoff_radius = combined_radius / time_seconds;

    /* Find the edge closest to `relative_velocity`. While the units are apart, it is on the
     * cut-off disc when the angle between `from_center` and the direction from
     * `relative_position` back to 0 is less than 90 degrees minus the half-angle of the cone.
     * The sine of that half-angle is `combined_radius / |relative_position|`, so the test is
     *     -projection > combined_radius * |from_center|
     * Squared, to avoid a square root, it becomes: `projection` is negative and its square is
     * more than `cutoff_limit_squared`. Otherwise the edge is the leg on the same side of
     * `relative_position` as `from_center`, and the change onto that leg drops the part of
     * `relative_velocity` that points across the leg. */
    float projection = dot(from_center, relative_position);
    float cutoff_limit_squared = combined_radius_squared * length_squared(from_center);
    bool is_on_cutoff_disc = projection < 0 && projection * projection > cutoff_limit_squared;
    V2 direction;
    V2 correction;
    if (!is_apart || is_on_cutoff_disc) {
        get_cutoff_correction(from_center, cutoff_radius, &direction, &correction);
    } else {
        bool is_left_leg = det2(relative_position, from_center) > 0;
        direction = get_cone_leg_direction(relative_position, combined_radius, is_left_leg);
        float along = dot(relative_velocity, direction);
        correction = sub(mul(direction, along), relative_velocity);
    }

    /* Add the part of the change that this unit makes to its velocity. */
    V2 own_correction = mul(correction, effort);
    return (Line){
        .point = add(velocity, own_correction),
        .direction = direction,
    };
}

/* ------------------------------------------------------------------------------------------------
 * `compute_orca_velocity` makes one line for each other unit that can touch this unit within
 * `ORCA_HORIZON_SECONDS`, and then solves the linear program of these lines. The bodies keep
 * `ORCA_MARGIN_METERS` apart, because the margin is part of `combined_radius`. A unit that cannot
 * dodge counts as standing still, and this unit makes all of the change (`FULL_EFFORT`) for it.
 * Two moving units each make half (`SHARED_EFFORT`). Lines have the order of the units, and
 * only `ORCA_MAX_LINES` lines are made. It tries `solve_linear_program_2` first, and when that
 * fails it hands the rest to `solve_linear_program_3`, with no hard lines because walls get none.
 * --------------------------------------------------------------------------------------------- */
V2 compute_orca_velocity(int unit_index, V2 preferred, int avoided_units) {
    /* `lines[0..line_count)` holds one line for each avoided unit, in unit order. */
    static Line lines[ORCA_MAX_LINES];
    const Unit *unit = &units[unit_index];
    float speed = len(unit->velocity);
    int line_count = 0;
    for (int other_index = 0; other_index < unit_count; other_index++) {
        if (line_count == ORCA_MAX_LINES)
            break;
        if (other_index == unit_index)
            continue;

        /* Leave out units that are too far away to touch this one within `ORCA_HORIZON_SECONDS`
         * at the current speeds of both. */
        const Unit *other = &units[other_index];
        V2 relative_position = sub(other->position, unit->position);
        float combined_radius = unit->kind->radius + other->kind->radius + ORCA_MARGIN_METERS;
        float other_speed = len(other->velocity);
        bool is_mate = other->squad_index == unit->squad_index;
        float range = combined_radius + (speed + other_speed) * ORCA_HORIZON_SECONDS;
        if (is_mate)
            range = combined_radius + MATE_AVOID_RANGE_METERS;
        if (length_squared(relative_position) > range * range)
            continue;

        /* Leave out the units that `avoided_units` excludes. */
        bool is_still = other->is_stopped || other->is_reorienting;
        bool is_wrong_squad = avoided_units == ORCA_AVOID_MOVING_MATES && !is_mate;
        if (avoided_units != ORCA_AVOID_ALL && (is_still || is_wrong_squad))
            continue;

        /* Add the line that `other` puts on this unit. */
        V2 other_velocity = other->velocity;
        if (is_still)
            other_velocity = v2(0, 0);
        float effort = is_still ? FULL_EFFORT : SHARED_EFFORT;
        V2 relative_velocity = sub(unit->velocity, other_velocity);
        lines[line_count] = make_orca_line(relative_position, relative_velocity, combined_radius,
                                           effort, unit->velocity);
        line_count++;
    }

    /* Find the best velocity for all lines. */
    V2 result;
    float max_speed = unit->kind->max_speed;
    bool is_direction_only = false;
    int first_failed_index = solve_linear_program_2(lines, line_count, max_speed, preferred,
                                                    is_direction_only, &result);
    if (first_failed_index < line_count)
        solve_linear_program_3(lines, line_count, 0, first_failed_index, max_speed, &result);
    return result;
}

/* ------------------------------------------------------------------------------------------------
 * `step_aside` makes room along `new_velocity`, whose length is `new_speed`. The unit drives at
 * the part of `new_speed` along its heading and steers toward `new_velocity`. `step_aside` also
 * sets `is_adjusting` and adds the tick to `adjust_seconds`, which the metrics count as avoidance.
 * Result: It returns the steering.
 * --------------------------------------------------------------------------------------------- */
static float step_aside(Unit *unit, V2 new_velocity, float new_speed) {
    unit->is_adjusting = true;
    unit->adjust_seconds += TICK_SECONDS;
    float target_speed = new_speed * get_forward_share(unit, new_velocity, new_speed);
    accelerate_toward_speed(unit, target_speed, SPEED_TIME_CONSTANT_SECONDS);
    return steer_to(unit, new_velocity);
}

/* ------------------------------------------------------------------------------------------------
 * `yield_mate` asks `compute_orca_velocity` for the velocity that avoids the moving mates, with
 * zero as `preferred` because the unit wants to stand still. It steps aside only when that
 * velocity is faster than `MIN_YIELD_SPEED` and `is_way_clear` finds the way to the point that
 * the velocity reaches in 1 s free. Setting `is_stopped` to false hands the unit back to formation
 * following, which drives it to its slot, once it left the reach of its slot.
 * --------------------------------------------------------------------------------------------- */
float yield_mate(int unit_index) {
    Unit *unit = &units[unit_index];

    /* Step aside when `new_velocity` is faster than `MIN_YIELD_SPEED` and the way to where it
     * leads in 1 s is clear. */
    V2 new_velocity = compute_orca_velocity(unit_index, v2(0, 0), ORCA_AVOID_MOVING_MATES);
    V2 ahead = add(unit->position, new_velocity);
    float new_speed = len(new_velocity);
    if (new_speed > MIN_YIELD_SPEED && is_way_clear(unit, ahead)) {
        unit->has_yielded = true;
        return step_aside(unit, new_velocity, new_speed);
    }

    /* Otherwise, hand the unit back to formation following when it stepped aside
     * (`has_yielded`) and is now further than `reach` from its slot (`waypoint`). Then turn to
     * `facing`. */
    float slot_distance_squared = length_squared(sub(unit->waypoint, unit->position));
    if (unit->has_yielded && slot_distance_squared > unit->reach * unit->reach) {
        unit->has_yielded = false;
        unit->is_stopped = false;
    }
    return steer_to_facing(unit);
}

/* ------------------------------------------------------------------------------------------------
 * `yield_parked` remembers the position where the unit parked as `home` at its first call. It
 * dodges at up to `YIELD_SPEED_FRACTION` of its top speed, and only toward points within
 * `YIELD_MAX_METERS` of `home`, so that it mostly holds its place. When it does not dodge, it
 * drives back toward `home`.
 * --------------------------------------------------------------------------------------------- */
float yield_parked(int unit_index) {
    /* Remember where the unit parked. */
    Unit *unit = &units[unit_index];
    if (!unit->has_home) {
        unit->home = unit->position;
        unit->has_home = true;
    }

    /* Dodge when `new_velocity` is faster than `MIN_YIELD_SPEED`, and after the cut to
     * `speed_cap`, the point it leads to in 1 s is within `YIELD_MAX_METERS` of `home` and the
     * way there is clear. */
    float speed_cap = YIELD_SPEED_FRACTION * unit->kind->max_speed;
    V2 new_velocity = compute_orca_velocity(unit_index, v2(0, 0), ORCA_AVOID_MOVING_UNITS);
    V2 back = sub(unit->home, unit->position);
    float new_speed = len(new_velocity);
    float home_distance = len(back);
    if (new_speed > MIN_YIELD_SPEED) {
        if (new_speed > speed_cap) {
            new_velocity = mul(new_velocity, speed_cap / new_speed);
            new_speed = speed_cap;
        }
        V2 ahead = add(unit->position, new_velocity);
        bool is_near_home = len(sub(ahead, unit->home)) <= YIELD_MAX_METERS;
        if (is_near_home && is_way_clear(unit, ahead))
            return step_aside(unit, new_velocity, new_speed);
    }

    /* While the unit is further than `HOME_REACHED_METERS` from `home`, drive back at up to
     * `speed_cap`. `braking_speed` is the speed from which braking at half of `friction` stops the
     * unit at `home`: `braking_speed^2 = 2 * (friction / 2) * home_distance`.
     * TODO: confirm why the unit brakes at half of the friction, as a margin or for a gentle
     * stop. */
    if (home_distance > HOME_REACHED_METERS) {
        float braking_speed = sqrtf(unit->kind->friction * home_distance);
        float return_speed = fminf(speed_cap, braking_speed);
        float target_speed = return_speed * get_forward_share(unit, back, home_distance);
        accelerate_toward_speed(unit, target_speed, SPEED_TIME_CONSTANT_SECONDS);
        return steer_to(unit, back);
    }

    /* At `home`, turn to `facing`. */
    return steer_to_facing(unit);
}
