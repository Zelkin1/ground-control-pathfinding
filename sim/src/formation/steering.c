/* steering.c drives each member of a squad in formation following, the "form" and "coord" modes
 * of `--avoid`. movement.c calls `steer_in_formation` once per tick for each moving unit of such
 * a squad. formation.c gives each member its target place, and this file picks the point that the
 * member drives at, its speed and its steering.
 *
 * Terms: sim.h defines the anchor, the place, the lane and the slot.
 * - The route progress of a member (`route_progress`) is how far along the squad route it is. It
 *   is the distance along the route of the route point closest to the member.
 *
 * Driving:
 * - A member far behind its place, or without a straight way to it, rejoins its lane or the
 *   route. A member further than `PURSUIT_PLACE_METERS` from its place pursues a point on its own
 *   lane, and the place sets only its speed. A member close to its place drives at it. With
 *   `--waitahead on`, a member just past its place waits for it instead of turning around.
 * - Over the last `END_BLEND_METERS` of the route, a member drives straight at its slot and stops
 *   on it, then turns to the final facing.
 * - The new avoidance (avoidance.c) changes the wanted velocity, the bend limit slows the member
 *   before bends of its lane, and the brake check stops it before blocked ground.
 * - The member steers in proportion to its heading gap (the angle between its heading and the
 *   direction that it wants), or with `steer_to`, the steering of the game.
 *
 * Fields of `Unit` that this file sets:
 *   `steer_in_formation` stores the point that the member drives at in `waypoint`, and it updates
 *   `route_progress` from the position of the member each tick. With lanes, `route_progress` can
 *   go back by up to `PROGRESS_BEHIND_METERS`, and without lanes it never goes back.
 *   `apply_avoidance` sets `is_adjusting`, `adjust_seconds`, `held_seconds` and
 *   `avoidance_off_seconds`, and `steer_in_formation` sets `is_stopped` once the member stands on
 *   its slot. */
#include "sim.h"
#include "formation.h"

#define FORM_SPEED_TIME_CONSTANT_SECONDS 0.1f /* The throttle closes speed gaps in about this. */
#define PROGRESS_BEHIND_METERS 10.0f       /* Searches start this far before the old progress. */
#define PROGRESS_AHEAD_METERS 30.0f        /* Searches end this far past the old progress. */
#define REJOIN_BEHIND_METERS 12.0f         /* A unit this far behind its place rejoins its lane. */
#define REJOIN_LOOK_METERS 10.0f           /* Rejoin points lie this far past progress at most. */
#define REJOIN_STEP_METERS 2.5f            /* Rejoin points are tried this far apart. */
#define NO_LANE_REJOIN_METERS 60.0f        /* Rejoins without lanes look this far past progress. */
#define PURSUIT_PLACE_METERS 4.0f          /* Units further than this from their place pursue. */
#define PURSUIT_MIN_METERS 6.0f            /* Pursuit looks at least this far past progress. */
#define PURSUIT_MAX_METERS 16.0f           /* Pursuit looks at most this far past progress. */
#define PURSUIT_LOOK_SECONDS 0.8f          /* Pursuit looks this long ahead at unit speed. */
#define PURSUIT_STEP_METERS 2.0f           /* Pursuit points are tried this far apart. */
#define PURSUIT_MIN_AHEAD_METERS 2.0f      /* Pursuit tries points down to this past progress. */
#define CATCH_UP_GAIN 0.8f                 /* Pursuers add this speed per m behind their place. */
#define AHEAD_SPEED_FLOOR_FRACTION 0.5f    /* Pursuers ahead keep this part of anchor speed. */
#define AHEAD_DECELERATION_FRACTION 0.2f   /* Pursuers ahead slow by this part of the friction. */
#define FILE_AHEAD_SPEED_FLOOR_FRACTION 0.2f  /* In single file, they keep this part instead. */
#define FILE_AHEAD_DECELERATION_FRACTION 0.5f /* In single file, they slow by this part instead. */
#define PLACE_GAIN 0.8f                    /* Units move to their place at this speed per m. */
#define ARRIVAL_BRAKING_FRACTION 0.7f      /* Braking plans use this part of the friction. */
#define ARRIVAL_SLACK_METERS 0.5f          /* Braking plans stop this far short of the target. */
#define SLOT_SPEED 2.0f                    /* A unit on its slot slower than this stops there. */
#define END_STOP_REACH_METERS 1.0f         /* With `--endstop on`, the slot is this close then. */
#define END_BRAKING_FRACTION 0.3f          /* End legs brake onto slots with this friction part. */
#define AVOIDANCE_MIN_CHANGE_SQUARED 0.25f /* Avoidance changes under 0.5 m/s are ignored. */
#define HELD_SPEED_FRACTION 0.25f          /* A unit is held still under this part of top speed. */
#define HELD_LIMIT_SECONDS 3.0f            /* Units held still this long by avoidance push on. */
#define AVOIDANCE_OFF_SECONDS 2.0f         /* Units then drive without avoidance for this long. */
#define BEND_LOOK_MIN_METERS 8.0f          /* Bend checks look at least this far past progress. */
#define BEND_LOOK_SECONDS 0.8f             /* Bend checks look this many seconds of speed ahead. */
#define BEND_PLACE_FRACTION 0.5f           /* Bend checks look this part as far past the place. */
#define BEND_MIN_DISTANCE_METERS 1.0f      /* Bend points this close give no speed limit. */
#define BEND_MIN_SINE 0.05f                /* Bend points this nearly straight on give no limit. */
#define FAR_BEND_LOOK_SECONDS 1.2f         /* With `--bendlook on`, bend checks look this long. */
#define HEADING_TIME_CONSTANT_SECONDS 0.1f /* Proportional steering closes gaps in about this. */
#define BEND_FLOOR_FRACTION 0.2f           /* Bend limits stay above this part of top speed. */
#define BRAKE_LOOK_SECONDS 0.4f            /* Brake checks look this long along the heading. */
#define BRAKE_LOOK_MARGIN_METERS 1.0f      /* The brake check looks a radius plus this at least. */
#define STEER_MIN_SPEED 0.5f               /* Slower velocities steer toward the target instead. */
#define STEER_MIN_DISTANCE_METERS 0.5f     /* Targets this close give no steering. */
#define WAIT_AHEAD_MIN_COSINE 0.87f        /* Units wait ahead within 30 degrees of forward. */

/* ------------------------------------------------------------------------------------------------
 * `is_rejoin_point_usable` returns true when `point` can serve as a rejoin point (a point that
 * the member drives at to get back to its lane or the route). With `--rejoinroom on`, the point
 * must lie further than `reach` (the distance within which a waypoint counts as reached) from the
 * member, and a circle with the radius of the member around it must be clear. With
 * `--rejoinroom off`, every point can serve.
 * --------------------------------------------------------------------------------------------- */
static bool is_rejoin_point_usable(const Unit *unit, V2 point, float reach_squared) {
    if (!is_rejoin_room_on)
        return true;
    if (length_squared(sub(point, unit->position)) <= reach_squared)
        return false;
    return is_circle_clear(point, unit->kind->radius, unit->kind->mover_class);
}

/* ------------------------------------------------------------------------------------------------
 * `find_rejoin_target` picks the point that a member drives at, with lanes, when it is far behind
 * its place or has no straight way to it.
 * Inputs: `place` is the target place of the member, and `place_distance` is how far along the
 *   route it is.
 * Result: It returns the furthest point on the lane of the member that the member can reach
 *   in a straight line, up to `REJOIN_LOOK_METERS` past its route progress but not past its
 *   place. Points are tried every `REJOIN_STEP_METERS`. With no such point, it does the same
 *   along the squad route itself, which is clear for the widest member, so that the member never
 *   drives into an obstacle.
 * Edge cases: With `--rejoinroom on`, the points must lie further than the `reach` of the member
 *   from it and have room for its body. Without these tests a member can stand still for good. On
 *   the inside of a sharp corner, a lane is clamped to one spot over several meters, so all its
 *   points there can lie right under the member. And a corner of a route that stays unsmoothed
 *   can touch blocked ground, so that the brake check holds a member that drives at it.
 *   With no such point in sight, `find_rejoin_target` returns `place` when the member can reach
 *   it in a straight line, or else the lane point at the progress when the member can reach
 *   that. In every other case, it returns the route point at the progress of the member.
 * --------------------------------------------------------------------------------------------- */
static V2 find_rejoin_target(const Squad *squad, const Unit *unit, V2 place,
                             float place_distance) {
    /* Try the lane points first and the route points second, each from the furthest one back
     * toward the progress of the member. */
    float place_ahead = place_distance - unit->route_progress;
    float start_ahead = clamp_float(place_ahead, 0, REJOIN_LOOK_METERS);
    float reach_squared = unit->reach * unit->reach;
    for (int pass_index = 0; pass_index <= 1; pass_index++) {
        bool is_on_route = pass_index == 1;
        float ahead = start_ahead;
        while (ahead >= 0) {
            float distance = unit->route_progress + ahead;
            V2 point;
            if (is_on_route) {
                V2 unused_direction;
                point = get_extended_route_point(squad, distance, &unused_direction);
            } else {
                point = get_lane_point(squad, unit, distance);
            }
            bool is_usable = is_rejoin_point_usable(unit, point, reach_squared);
            if (is_usable && is_way_clear(unit, point))
                return point;
            ahead -= REJOIN_STEP_METERS;
        }
    }

    /* With no such point in sight, try the place, then the lane point at the progress. */
    if (is_rejoin_room_on) {
        if (is_way_clear(unit, place))
            return place;
        V2 lane_point = get_lane_point(squad, unit, unit->route_progress);
        if (is_way_clear(unit, lane_point))
            return lane_point;
    }
    V2 unused_direction;
    return get_extended_route_point(squad, unit->route_progress, &unused_direction);
}

/* ------------------------------------------------------------------------------------------------
 * `find_pursuit_target` looks for a point on the lane of the member, past its route progress,
 * that it can reach in a straight line (`is_way_clear`). Steering at that point makes the heading
 * of the member follow the bends of the lane instead of cutting across them toward its place.
 * Result: It returns true and sets `target` to the furthest such point, from `pursuit_distance`
 *   past the progress down to `PURSUIT_MIN_AHEAD_METERS`, in steps of `PURSUIT_STEP_METERS`.
 *   `pursuit_distance` is `PURSUIT_LOOK_SECONDS` at the speed of the member, within
 *   `PURSUIT_MIN_METERS` and `PURSUIT_MAX_METERS`.
 * Edge case: It returns false and leaves `target` as it was when no such point exists.
 * --------------------------------------------------------------------------------------------- */
static bool find_pursuit_target(const Squad *squad, const Unit *unit, V2 *target) {
    /* Find how far ahead to look. */
    float look_distance = PURSUIT_LOOK_SECONDS * len(unit->velocity);
    float pursuit_distance = clamp_float(look_distance, PURSUIT_MIN_METERS, PURSUIT_MAX_METERS);

    /* Try the lane points from there back toward the member. */
    float ahead = pursuit_distance;
    while (ahead >= PURSUIT_MIN_AHEAD_METERS) {
        V2 point = get_lane_point(squad, unit, unit->route_progress + ahead);
        if (is_way_clear(unit, point)) {
            *target = point;
            return true;
        }
        ahead -= PURSUIT_STEP_METERS;
    }
    return false;
}

/* ------------------------------------------------------------------------------------------------
 * `slow_down_ahead` returns the speed of a pursuing member that is ahead of its place, from
 * `catch_up_speed`, the speed of the anchor less `CATCH_UP_GAIN` per m ahead. A member more than
 * `anchor_speed / CATCH_UP_GAIN` ahead gets 0 from that, so it brakes with its whole friction
 * and stands until its place comes by.
 * Result: With `--aheadslow on`, the speed is at least a floor part of the speed of the anchor,
 *   so the member only lets its place catch up, and it falls from its current speed by at most a
 *   part of the friction of the member per second. Side by side, these parts are
 *   `AHEAD_SPEED_FLOOR_FRACTION` and `AHEAD_DECELERATION_FRACTION`. In single file they are
 *   `FILE_AHEAD_SPEED_FLOOR_FRACTION` and `FILE_AHEAD_DECELERATION_FRACTION`: when the squad
 *   forms a file, the places of the rear members jump back by whole file gaps, and members that
 *   drop back too slowly run into the mates that merge in front of them. The bend limit, the
 *   brake check and the avoidance still brake as hard as they need. With `--aheadslow off`, the
 *   speed is `catch_up_speed`.
 * Edge case: The member still stops when the anchor stands, as when its squad gives way.
 * --------------------------------------------------------------------------------------------- */
static float slow_down_ahead(const Squad *squad, const Unit *unit, float catch_up_speed) {
    if (!is_ahead_slowdown_on)
        return catch_up_speed;

    /* Pick the parts for side by side or for single file. */
    float floor_fraction = AHEAD_SPEED_FLOOR_FRACTION;
    float deceleration_fraction = AHEAD_DECELERATION_FRACTION;
    if (squad->is_single_file) {
        floor_fraction = FILE_AHEAD_SPEED_FLOOR_FRACTION;
        deceleration_fraction = FILE_AHEAD_DECELERATION_FRACTION;
    }

    /* Keep at least the floor speed, and at least the current speed less one tick of
     * `deceleration`. */
    float floor_speed = floor_fraction * squad->anchor_speed;
    float deceleration = deceleration_fraction * unit->kind->friction;
    float gentle_speed = len(unit->velocity) - deceleration * TICK_SECONDS;
    float slowest_speed = fmaxf(floor_speed, gentle_speed);
    return fmaxf(catch_up_speed, slowest_speed);
}

/* ------------------------------------------------------------------------------------------------
 * `find_rejoin_target_without_lanes` picks the point that a member drives at, without lanes, when
 * it has no straight way to its place. It returns the furthest route point that the member can
 * reach in a straight line, past its route progress and up to `NO_LANE_REJOIN_METERS` past it,
 * but not past the anchor. Points are tried every `REJOIN_STEP_METERS`.
 * Edge case: With no such point, it returns the route point at the progress of the member.
 * --------------------------------------------------------------------------------------------- */
static V2 find_rejoin_target_without_lanes(const Squad *squad, const Unit *unit) {
    float best_distance = unit->route_progress;
    float farthest_distance = unit->route_progress + NO_LANE_REJOIN_METERS;
    float distance = fminf(squad->anchor_distance, farthest_distance);
    while (distance > unit->route_progress) {
        V2 point = get_lane_point(squad, unit, distance);
        if (is_way_clear(unit, point)) {
            best_distance = distance;
            break;
        }
        distance -= REJOIN_STEP_METERS;
    }
    return get_lane_point(squad, unit, best_distance);
}

/* ------------------------------------------------------------------------------------------------
 * `get_velocity_toward` returns the velocity of `speed` along `to_point`, a vector whose length
 * is `distance`, or zero when `distance` is 0.
 * --------------------------------------------------------------------------------------------- */
static V2 get_velocity_toward(V2 to_point, float distance, float speed) {
    float factor = 0;
    if (distance > 0)
        factor = speed / distance;
    return mul(to_point, factor);
}

/* ------------------------------------------------------------------------------------------------
 * `apply_avoidance` returns the velocity that a unit takes after the new avoidance. It
 * also sets `is_adjusting` and adds the tick to `adjust_seconds` when it uses the velocity of the
 * avoidance, which the metrics count as avoidance.
 * Inputs: `preferred` is the velocity that the unit wants.
 * Result: It returns the velocity from `compute_orca_velocity` when avoidance is on for the unit,
 *   that velocity differs from `preferred` by at least 0.5 m/s, and the straight line along it
 *   for the next second is clear. Otherwise it returns `preferred`.
 * Edge case: Avoidance can hold a unit nearly still, slower than `HELD_SPEED_FRACTION` of its top
 *   speed while the speed of `preferred` is above that. After more than `HELD_LIMIT_SECONDS` of
 *   this, the unit drives without avoidance for `AVOIDANCE_OFF_SECONDS`.
 * --------------------------------------------------------------------------------------------- */
static V2 apply_avoidance(int unit_index, V2 preferred) {
    /* Decide whether to use the velocity of the avoidance. */
    Unit *unit = &units[unit_index];
    V2 new_velocity = compute_orca_velocity(unit_index, preferred, ORCA_AVOID_ALL);
    V2 ahead = add(unit->position, new_velocity);
    unit->avoidance_off_seconds -= TICK_SECONDS;
    bool is_avoidance_on = unit->avoidance_off_seconds <= 0;
    bool is_changed = length_squared(sub(new_velocity, preferred)) >= AVOIDANCE_MIN_CHANGE_SQUARED;
    if (!is_avoidance_on || !is_changed || !is_way_clear(unit, ahead))
        return preferred;
    unit->is_adjusting = true;
    unit->adjust_seconds += TICK_SECONDS;

    /* Switch avoidance off for a while when it holds the unit nearly still for too long. */
    float held_speed = HELD_SPEED_FRACTION * unit->kind->max_speed;
    bool is_held = len(unit->velocity) < held_speed && len(preferred) > held_speed;
    if (is_held)
        unit->held_seconds += TICK_SECONDS;
    else
        unit->held_seconds = 0;
    if (unit->held_seconds > HELD_LIMIT_SECONDS) {
        unit->held_seconds = 0;
        unit->avoidance_off_seconds = AVOIDANCE_OFF_SECONDS;
    }
    return new_velocity;
}

/* ------------------------------------------------------------------------------------------------
 * `limit_speed_for_bend` slows a member before bends of its own lane.
 *
 *                    P
 *                   / :
 *          chord   /  :  arc
 *                 /   :
 *                /   .'
 *               /_.-'
 *              U--------->  heading
 *
 *   U is the member and P is a lane point ahead. The chord is the straight line from U to P, and
 *   the arc starts along the heading and ends at P.
 * Result: It returns `target_speed`, cut to the speed at which the member can drive the arc. The
 *   radius of the arc follows from `radius = bend_distance / (2 * sine)`. `bend_distance` is the
 *   length of the chord, and `sine` is the sine of the angle at U between the heading and the
 *   chord. The member can drive the arc at `turn_rate * radius`. `limit_speed_for_bend` allows
 *   `BEND_SPEED_MARGIN` of that, but it never cuts the speed below `BEND_FLOOR_FRACTION` of the
 *   top speed.
 *   With `--bendlook on`, P is `FAR_BEND_LOOK_SECONDS` at the speed of the member past its route
 *   progress, or at least `BEND_LOOK_MIN_METERS`. A unit a little beside its lane then sees P at
 *   a small angle. For a sideways gap `gap`, the radius of the arc is about
 *   `bend_distance^2 / (2 * gap)`, so a short look makes the unit brake for its own gap and not
 *   for a bend.
 *   With `--bendlook off`, P is `BEND_LOOK_SECONDS` at the speed of the member past its progress,
 *   or at least `BEND_LOOK_MIN_METERS`. But P lies at most `BEND_PLACE_FRACTION` of that look past
 *   the place of the member, or past its progress when that is further along.
 * Edge case: The speed stays as it is when the point is within `BEND_MIN_DISTANCE_METERS`, or
 *   when `sine` is `BEND_MIN_SINE` or less, so that the point lies almost straight ahead.
 * --------------------------------------------------------------------------------------------- */
static float limit_speed_for_bend(const Squad *squad, const Unit *unit, float target_speed) {
    /* Find the lane point P and its angle off the heading. */
    float speed = len(unit->velocity);
    float bend_point_distance;
    if (is_bend_look_on) {
        float bend_look_ahead = fmaxf(BEND_LOOK_MIN_METERS, FAR_BEND_LOOK_SECONDS * speed);
        bend_point_distance = unit->route_progress + bend_look_ahead;
    } else {
        float bend_look_ahead = fmaxf(BEND_LOOK_MIN_METERS, BEND_LOOK_SECONDS * speed);
        float place_progress = fmaxf(unit->route_progress, get_place_distance(squad, unit));
        float own_look_distance = unit->route_progress + bend_look_ahead;
        float place_look_distance = place_progress + bend_look_ahead * BEND_PLACE_FRACTION;
        bend_point_distance = fminf(own_look_distance, place_look_distance);
    }
    V2 bend_point = get_lane_point(squad, unit, bend_point_distance);
    V2 to_bend_point = sub(bend_point, unit->position);
    float bend_distance = len(to_bend_point);
    if (bend_distance <= BEND_MIN_DISTANCE_METERS)
        return target_speed;
    float sine = fabsf(det2(unit->heading, to_bend_point)) / bend_distance;
    if (sine <= BEND_MIN_SINE)
        return target_speed;

    /* Find the speed for the arc. */
    float arc_speed = unit->kind->turn_rate * bend_distance / (2 * sine);
    float bend_speed = arc_speed * BEND_SPEED_MARGIN;
    float floor_speed = BEND_FLOOR_FRACTION * unit->kind->max_speed;
    float allowed_speed = fmaxf(bend_speed, floor_speed);
    return fminf(target_speed, allowed_speed);
}

/* ------------------------------------------------------------------------------------------------
 * `steer_proportionally` returns the steering that turns the unit toward `direction` with
 * `--linearsteer on`: the angle between the heading and `direction`, divided by the angle that the
 * unit turns in `HEADING_TIME_CONSTANT_SECONDS`, within full lock. So the unit closes a heading
 * gap in about that time, whatever its turn rate. The own steering of the game, `steer_to`,
 * grows with about 3 times the square of a small angle. To hold a bend of 40 m radius at 20 m/s,
 * a medium tank then trails the wanted direction by about 20 degrees, drifts toward the outside
 * of the bend, and loses speed because it does not face where it goes.
 * Edge case: A zero `direction` gives no steering.
 * --------------------------------------------------------------------------------------------- */
static float steer_proportionally(const Unit *unit, V2 direction) {
    if (length_squared(direction) == 0)
        return 0;
    float angle = atan2f(det2(unit->heading, direction), dot(unit->heading, direction));
    float turn_per_time_constant = unit->kind->turn_rate * HEADING_TIME_CONSTANT_SECONDS;
    return clamp_float(-angle / turn_per_time_constant, -1, 1);
}

/* ------------------------------------------------------------------------------------------------
 * `steer_toward` returns the steering toward `direction`: `steer_proportionally` with
 * `--linearsteer on`, or `steer_to` of the game with `--linearsteer off`.
 * --------------------------------------------------------------------------------------------- */
static float steer_toward(const Unit *unit, V2 direction) {
    if (is_linear_steer_on)
        return steer_proportionally(unit, direction);
    return steer_to(unit, direction);
}

/* ------------------------------------------------------------------------------------------------
 * `steer_in_formation` works in steps. It updates `route_progress` and picks the target, which is
 * the place, a rejoin point or a pursuit point. It lets the end leg override that target. When the
 * anchor has reached the end of the route (within `ROUTE_END_EPSILON_METERS`, the distance that
 * counts as at the end, shared with squad.c) and the unit stands on its target, it stops. Else it
 * finds the wanted velocity, lets `apply_avoidance` change it, and cuts the speed for bends and
 * blocked ground. It sets the throttle through `accelerate_toward_speed` and returns the steering.
 * It stores the final target in `waypoint`.
 * --------------------------------------------------------------------------------------------- */
float steer_in_formation(int unit_index) {
    Unit *unit = &units[unit_index];
    const Squad *squad = &squads[unit->squad_index];
    float speed = len(unit->velocity);
    float remaining_distance = squad->route_length - squad->anchor_distance;

    /* Pick where to drive: the target place, or a point on the lane or the route when the unit
     * rejoins or pursues. With lanes, the route progress of the unit can go back by up to
     * `PROGRESS_BEHIND_METERS`. Without lanes, it never goes back, because going back makes the
     * unit oscillate where the route doubles back. */
    V2 forward;
    V2 place = get_target_place(squad, unit, &forward);
    float place_distance = get_place_distance(squad, unit);
    V2 target = place;
    bool is_rejoining;
    bool is_pursuing = false;
    float pursuit_speed = 0;
    if (are_lanes_on) {
        float from_progress = fmaxf(0, unit->route_progress - PROGRESS_BEHIND_METERS);
        float to_progress = unit->route_progress + PROGRESS_AHEAD_METERS;
        float progress = project_onto_route(squad, unit->position, from_progress, to_progress);
        unit->route_progress = progress;
        float behind_place = place_distance - unit->route_progress;
        is_rejoining = behind_place > REJOIN_BEHIND_METERS || !is_way_clear(unit, target);
        bool is_before_end_blend = remaining_distance > END_BLEND_METERS;
        bool is_far_from_place = len(sub(target, unit->position)) > PURSUIT_PLACE_METERS;
        if (is_rejoining) {
            target = find_rejoin_target(squad, unit, place, place_distance);
        } else if (is_pursuit_on && is_before_end_blend && is_far_from_place) {
            /* The place sets only the speed: the speed of the anchor, plus `CATCH_UP_GAIN` per m
             * that the unit is behind its place, or less when it is ahead. */
            is_pursuing = find_pursuit_target(squad, unit, &target);
            float catch_up_speed = squad->anchor_speed + CATCH_UP_GAIN * behind_place;
            if (behind_place < 0)
                catch_up_speed = slow_down_ahead(squad, unit, catch_up_speed);
            pursuit_speed = clamp_float(catch_up_speed, 0, unit->kind->max_speed);
        }
    } else {
        float from_progress = unit->route_progress;
        float to_progress = unit->route_progress + PROGRESS_AHEAD_METERS;
        float projected = project_onto_route(squad, unit->position, from_progress, to_progress);
        unit->route_progress = fmaxf(unit->route_progress, projected);
        is_rejoining = !is_way_clear(unit, target);
        if (is_rejoining)
            target = find_rejoin_target_without_lanes(squad, unit);
    }

    /* With `--replan on`, drive along a detour around obstacles while the unit follows one, as if
     * it rejoined. */
    bool is_detouring = false;
    if (is_replan_on)
        is_detouring = follow_detour(squad, unit, place, place_distance, &target);
    if (is_detouring) {
        is_rejoining = true;
        is_pursuing = false;
    }

    /* With `--endstop on`, the unit is on its end leg while the anchor is within the last
     * `END_BLEND_METERS` of the route and the unit follows no detour. On it, the unit drives
     * straight at its slot when the way there is clear, and else straight at the target, as when
     * it rejoins. In both cases it leaves out the motion of the anchor, which can push the unit
     * past a place that lies behind it. */
    bool is_end_leg = is_end_stop_on && remaining_distance <= END_BLEND_METERS && !is_detouring;
    bool is_ending = is_end_leg && is_way_clear(unit, unit->slot);
    if (is_ending) {
        target = unit->slot;
        is_rejoining = false;
        is_pursuing = false;
    } else if (is_end_leg) {
        is_rejoining = true;
        is_pursuing = false;
    }
    unit->waypoint = target;
    V2 to_target = sub(target, unit->position);
    float target_distance = len(to_target);

    /* Once the anchor is within `ROUTE_END_EPSILON_METERS` of the end of the route, stop the unit
     * and turn it to `facing` when it follows no detour, is within `slot_reach` of its target and
     * is slower than `SLOT_SPEED`. `slot_reach` is `END_STOP_REACH_METERS` while the unit drives
     * straight at its slot, and `reach` otherwise. */
    float end_distance = squad->route_length - ROUTE_END_EPSILON_METERS;
    bool is_anchor_at_end = squad->anchor_distance >= end_distance;
    float slot_reach = is_ending ? END_STOP_REACH_METERS : unit->reach;
    bool is_on_slot = !is_detouring && target_distance < slot_reach && speed < SLOT_SPEED;
    if (is_anchor_at_end && is_on_slot) {
        unit->is_stopped = true;
        return steer_to(unit, unit->facing);
    }

    /* Find the wanted velocity (`preferred`) in one of three ways:
     * - A unit on its end leg that sees its slot drives straight at it at `end_speed`, which is
     *   at most `slot_braking_speed`, so that it stops there and not where the anchor stops. This
     *   plan brakes with the gentler `END_BRAKING_FRACTION` of `friction`, so that squad mates
     *   slow down together and the avoidance does not keep a unit going for a mate close behind.
     *   It stops the unit on the slot itself, from where the unit will be one tick later. A
     *   marine at top speed covers a third of a meter per tick, so without that lead it brakes
     *   too late and overshoots.
     * - A rejoining unit drives straight at its target at `speed_cap`.
     * - Any other unit takes the motion of the anchor plus `PLACE_GAIN` per m toward the target.
     * Then cap the velocity at `speed_cap`: the top speed, or the speed of the anchor plus
     * `braking_speed` when that is lower. Braking from `braking_speed` with
     * `ARRIVAL_BRAKING_FRACTION` of `friction` stops the unit `ARRIVAL_SLACK_METERS` short of the
     * target: `braking_speed^2 = 2 * deceleration * distance`. Throttle 0 brakes with the whole
     * `friction`, so the plan keeps a margin. Last, a pursuing unit drives straight at its target
     * at `pursuit_speed` instead, without the cap. */
    float friction = unit->kind->friction;
    float braking_distance = fmaxf(0, target_distance - ARRIVAL_SLACK_METERS);
    float braking_speed = sqrtf(2 * ARRIVAL_BRAKING_FRACTION * friction * braking_distance);
    float speed_cap = fminf(unit->kind->max_speed, braking_speed + squad->anchor_speed);
    V2 preferred;
    float end_speed = unit->kind->max_speed;
    if (is_ending) {
        float slot_braking_distance = fmaxf(0, target_distance - speed * TICK_SECONDS);
        float slot_braking_squared = 2 * END_BRAKING_FRACTION * friction * slot_braking_distance;
        float slot_braking_speed = sqrtf(slot_braking_squared);
        end_speed = fminf(unit->kind->max_speed, slot_braking_speed);
        preferred = get_velocity_toward(to_target, target_distance, end_speed);
    } else if (is_rejoining) {
        preferred = get_velocity_toward(to_target, target_distance, speed_cap);
    } else {
        V2 anchor_motion = mul(forward, squad->anchor_speed);
        V2 correction = mul(to_target, PLACE_GAIN);
        preferred = add(anchor_motion, correction);
    }
    float preferred_speed = len(preferred);
    if (preferred_speed > speed_cap)
        preferred = mul(preferred, speed_cap / preferred_speed);
    if (is_pursuing)
        preferred = get_velocity_toward(to_target, target_distance, pursuit_speed);

    /* With `--waitahead on`, a unit that drives at its place stands still and keeps its heading
     * until the place comes by, when three things hold: it faces the way that the place moves,
     * within `WAIT_AHEAD_MIN_COSINE`, the place lies behind it, and it wants to go forward slower
     * than `STEER_MIN_SPEED`. That happens when the anchor slows down hard and leaves the unit a
     * few meters past its place. A unit that stands at a larger angle to the way of its place,
     * as in a tangle at the start, still turns, because standing still there can block its
     * squad. Without the wait, the unit steers along a velocity that points back, or below
     * `STEER_MIN_SPEED` straight at the place, so it turns around on the spot, and it must turn
     * back once the anchor drives on. The last `END_BLEND_METERS` of the route are left out,
     * because there the unit must reach its slot. */
    bool is_before_end_blend = remaining_distance > END_BLEND_METERS;
    bool is_driving_at_place = is_before_end_blend && !is_rejoining && !is_pursuing;
    bool is_facing_forward = dot(unit->heading, forward) >= WAIT_AHEAD_MIN_COSINE;
    bool is_place_behind = dot(to_target, unit->heading) < 0;
    bool is_slow_forward = dot(preferred, unit->heading) < STEER_MIN_SPEED;
    bool is_turning_back = is_facing_forward && is_place_behind && is_slow_forward;
    bool is_waiting_ahead = is_wait_ahead_on && is_driving_at_place && is_turning_back;
    if (is_waiting_ahead)
        preferred = v2(0, 0);

    /* Avoid other units. Then drive at the part of the new speed along the heading. */
    V2 new_velocity = apply_avoidance(unit_index, preferred);
    float new_speed = len(new_velocity);
    float target_speed = 0;
    if (new_speed > 0)
        target_speed = new_speed * get_forward_share(unit, new_velocity, new_speed);

    /* With `--endcap on`, the avoidance can turn a unit on its end leg but not speed it up past
     * `end_speed`, from which it can still brake onto its slot. Else squad mates that come up fast
     * behind push it on past the slot, and then block its way back. */
    if (is_end_cap_on && is_ending)
        target_speed = fminf(target_speed, end_speed);

    /* With lanes, drive slower before bends of the lane. Also brake while blocked ground lies
     * within the next `BRAKE_LOOK_SECONDS` along the heading, or within the radius of the unit
     * plus `BRAKE_LOOK_MARGIN_METERS` when that is further. */
    if (are_lanes_on) {
        target_speed = limit_speed_for_bend(squad, unit, target_speed);
        float min_look_distance = unit->kind->radius + BRAKE_LOOK_MARGIN_METERS;
        float look_distance = fmaxf(speed * BRAKE_LOOK_SECONDS, min_look_distance);
        V2 look_point = add(unit->position, mul(unit->heading, look_distance));
        if (!is_way_clear(unit, look_point))
            target_speed = 0;
    }

    /* Set the throttle. Its short time constant makes units brake onto their places. */
    accelerate_toward_speed(unit, target_speed, FORM_SPEED_TIME_CONSTANT_SECONDS);

    /* Steer along the new velocity when it is faster than `STEER_MIN_SPEED`. Else steer toward
     * the target when it is further than `STEER_MIN_DISTANCE_METERS` and the unit does not wait
     * ahead of its place, and else not at all. */
    if (new_speed > STEER_MIN_SPEED)
        return steer_toward(unit, new_velocity);
    if (is_waiting_ahead)
        return 0;
    if (target_distance > STEER_MIN_DISTANCE_METERS)
        return steer_toward(unit, to_target);
    return 0;
}
