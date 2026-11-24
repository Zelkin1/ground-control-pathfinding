// Steering of one squad member in formation following. Each tick, `steer_in_formation` picks a
// target for the squad member, a preferred velocity toward it, and then a target speed and a
// steering input. The target is one of these points:
// - the slot, for a squad member close to its slot;
// - a pursuit point on its own lane ahead, for a squad member a few meters off its slot
//   (`--driving`), where the slot sets only the speed;
// - a rejoin point on its lane or on the route, for a squad member far behind its slot or without
//   a clear line to it;
// - a detour point (detour.c), or its final parking position on the end section of the route.
//
// The ORCA avoidance (avoidance.c) then adjusts the preferred velocity. The bend maximum speed
// limits the speed before bends, and the brake check stops a squad member whose next moment along
// its heading would hit blocking terrain. The route progress of a squad member is the route
// position of its projection onto the route.
#include "sim.h"
#include "formation.h"

#define FORM_SPEED_TIME_CONSTANT_SECONDS 0.1f          // speed time constant of the throttle
#define PROGRESS_BEHIND_METERS 10.0f                   // route progress search, reach back
#define PROGRESS_AHEAD_METERS 30.0f                    // route progress search, reach ahead
#define REJOIN_BEHIND_METERS 12.0f                     // lag behind the slot that starts a rejoin
#define REJOIN_LOOK_METERS 10.0f                       // farthest rejoin point past the progress
#define REJOIN_STEP_METERS 2.5f                        // spacing of the tried rejoin points
#define NO_LANE_REJOIN_METERS 60.0f                    // farthest rejoin point without lanes
#define PURSUIT_MIN_METERS 6.0f                        // nearest pursuit point past the progress
#define PURSUIT_MAX_METERS 16.0f                       // farthest pursuit point past the progress
#define PURSUIT_STEP_METERS 2.0f                       // spacing of the tried pursuit points
#define PURSUIT_MIN_AHEAD_METERS 2.0f                  // nearest tried pursuit point
#define CATCH_UP_GAIN 0.8f                             // extra speed per meter behind the slot
#define AHEAD_SPEED_FLOOR_FRACTION 0.5f                // lowest anchor speed share when ahead
#define AHEAD_DECELERATION_FRACTION 0.2f               // friction share for slowing when ahead
#define COLUMN_AHEAD_SPEED_FLOOR_FRACTION 0.2f         // the same in column formation
#define COLUMN_AHEAD_DECELERATION_FRACTION 0.5f        // the same in column formation
#define SLOT_GAIN 0.8f                                 // speed toward the slot per meter away
#define ARRIVAL_BRAKING_FRACTION 0.7f                  // friction share of planned braking
#define ARRIVAL_SLACK_METERS 0.5f                      // planned stop this far short of a target
#define PARKING_SPEED 2.0f                             // speed below which a squad member parks
#define END_STOP_REACH_METERS 1.0f                     // reach of the final parking position
#define END_BRAKING_FRACTION 0.3f                      // friction share of braking at the end
#define AVOIDANCE_MIN_CHANGE_SQUARED 0.25f             // smallest squared change kept from ORCA
#define WAIT_SPEED_FRACTION 0.25f                      // top speed share for waiting for ORCA
#define ORCA_WAIT_LIMIT_SECONDS 3.0f                   // longest wait for the ORCA avoidance
#define AVOIDANCE_OFF_SECONDS 2.0f                     // avoidance pause after that wait
#define BEND_LOOK_MIN_METERS 8.0f                      // shortest look ahead for bends
#define BEND_LOOK_SECONDS 0.8f                         // bend look time, `--driving off`
#define BEND_SLOT_FRACTION 0.5f                        // share of the bend look past the slot
#define BEND_MIN_DISTANCE_METERS 1.0f                  // nearer bend points give no limit
#define BEND_MIN_SINE 0.05f                            // smaller bend sines give no limit
#define BEND_FLOOR_FRACTION 0.2f                       // lowest bend speed, share of top speed
#define BRAKE_LOOK_MARGIN_METERS 1.0f                  // shortest brake look past the radius
#define STEER_MIN_SPEED 0.5f                           // slowest speed to steer along velocity
#define STEER_MIN_DISTANCE_METERS 0.5f                 // nearer targets give no steering
#define WAIT_AHEAD_MIN_COSINE 0.87f                    // facing limit for waiting, 30 degrees
#define REVERSE_ALIGN_COSINE 0.95f                     // rear aligned with the way, 18 degrees


// Target of one squad member in one tick, and how the squad member drives toward it.
typedef struct {
    V2 point;
    bool is_rejoining;        // straight at `point`, at a speed that can stop there
    bool is_pursuing;         // straight at `point`, at `pursuit_speed`
    float pursuit_speed;
} DriveTarget;


// `is_rejoin_point_usable` tells whether a squad member can rejoin at `point`. With `--driving`,
// the point must lie beyond the reach of the squad member, with room for its body.
static bool is_rejoin_point_usable(const Unit *unit, V2 point, float reach_squared) {
    // A point within reach would count as reached at once, and a rejoin there would not move the
    // squad member.
    if (!is_driving_on)
        return true;
    if (length_squared(sub(point, unit->position)) <= reach_squared)
        return false;
    return is_circle_clear(point, unit->unit_type->radius, unit->unit_type->terrain_class);
}


// `find_rejoin_target` finds the farthest usable point in sight on the lane of a squad member, up
// to `REJOIN_LOOK_METERS` past its route progress, and then on the route. Without such a point, it
// falls back to the slot, the lane point at the route progress, or the route point there.
static V2 find_rejoin_target(const Squad *squad, const Unit *unit, V2 slot,
                             float slot_distance) {
    // The first try reads the lane, the second the route, each from the farthest point back.
    float slot_ahead = slot_distance - unit->route_progress;
    float start = clamp_float(slot_ahead, 0, REJOIN_LOOK_METERS);
    float reach_squared = unit->reach * unit->reach;
    for (int try_index = 0; try_index <= 1; try_index++) {
        bool is_on_route = try_index == 1;
        float ahead = start;
        while (ahead >= 0) {
            float d = unit->route_progress + ahead;        // route position of the point
            V2 point;
            if (is_on_route) {
                V2 unused;
                point = get_extended_route_point(squad, d, &unused);
            } else {
                point = get_lane_point(squad, unit, d);
            }
            bool is_usable = is_rejoin_point_usable(unit, point, reach_squared);
            if (is_usable && is_way_clear(unit, point))
                return point;
            ahead -= REJOIN_STEP_METERS;
        }
    }

    // Without `--driving`, the fallback is the route point at the route progress.
    if (is_driving_on) {
        if (is_way_clear(unit, slot))
            return slot;
        V2 lane_point = get_lane_point(squad, unit, unit->route_progress);
        if (is_way_clear(unit, lane_point))
            return lane_point;
    }
    V2 unused;
    return get_extended_route_point(squad, unit->route_progress, &unused);
}


// `find_pursuit_target` finds the farthest point in sight on the lane of a squad member, between
// `PURSUIT_MIN_AHEAD_METERS` and the pursuit distance past its route progress. It returns false
// when no such point is in sight.
static bool find_pursuit_target(const Squad *squad, const Unit *unit, V2 *target) {
    // The pursuit distance grows with the speed, between `PURSUIT_MIN_METERS` and
    // `PURSUIT_MAX_METERS`.
    float look_seconds = get_tuning(unit->unit_type)->pursuit_look_seconds;
    float look = look_seconds * len(unit->velocity);
    float ahead = clamp_float(look, PURSUIT_MIN_METERS, PURSUIT_MAX_METERS);
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


// `slow_down_ahead` raises the catch-up speed of a squad member ahead of its slot, so that it
// falls back gently instead of stopping (`--formation`). The result is at least a share of the
// anchor speed, and at least the current speed minus a gentle deceleration over one tick.
static float slow_down_ahead(const Squad *squad, const Unit *unit, float catch_up_speed) {
    // In column formation, a squad member ahead falls back faster and to a lower speed. A slower
    // drop back there ran squad members into others that merged in front.
    if (!is_formation_keeping_on)
        return catch_up_speed;
    float floor_fraction = AHEAD_SPEED_FLOOR_FRACTION;
    float deceleration_fraction = AHEAD_DECELERATION_FRACTION;
    if (squad->is_in_column) {
        floor_fraction = COLUMN_AHEAD_SPEED_FLOOR_FRACTION;
        deceleration_fraction = COLUMN_AHEAD_DECELERATION_FRACTION;
    }

    // The gentle deceleration is a share of the friction of the unit type.
    float v_floor = floor_fraction * squad->anchor_speed;
    float decel = deceleration_fraction * unit->unit_type->friction;        // deceleration
    float v_gentle = len(unit->velocity) - decel * TICK_SECONDS;
    float v_min = fmaxf(v_floor, v_gentle);
    return fmaxf(catch_up_speed, v_min);
}


// `find_rejoin_target_without_lanes` finds the rejoin point with `--lanes off`: the farthest point
// in sight on the lane of a squad member, from the anchor, but at most `NO_LANE_REJOIN_METERS`
// past the route progress, back to the route progress.
static V2 find_rejoin_target_without_lanes(const Squad *squad, const Unit *unit) {
    float best = unit->route_progress;                     // route position of the result
    float d_max = unit->route_progress + NO_LANE_REJOIN_METERS;
    float d = fminf(squad->anchor_distance, d_max);        // route position of a try
    while (d > unit->route_progress) {
        V2 point = get_lane_point(squad, unit, d);
        if (is_way_clear(unit, point)) {
            best = d;
            break;
        }
        d -= REJOIN_STEP_METERS;
    }
    return get_lane_point(squad, unit, best);
}


// `get_velocity_toward` returns the velocity of magnitude `speed` along `to_point`, a vector of
// length `distance`. A zero distance gives a zero velocity.
static V2 get_velocity_toward(V2 to_point, float distance, float speed) {
    float factor = 0;
    if (distance > 0)
        factor = speed / distance;
    return mul(to_point, factor);
}


// `apply_avoidance` returns the velocity of a squad member after the ORCA avoidance. The squad
// member keeps `preferred` when the change is small, when the avoidance pauses, or when the line
// of one second along the new velocity meets blocking terrain.
static V2 apply_avoidance(int unit_index, V2 preferred) {
    // `ahead` is the position after one second at the new velocity.
    Unit *unit = &units[unit_index];
    V2 v_new = compute_orca_velocity(unit_index, preferred, ORCA_AVOID_ALL);
    V2 ahead = add(unit->position, v_new);
    unit->avoidance_off_seconds -= TICK_SECONDS;
    bool is_avoidance_on = unit->avoidance_off_seconds <= 0;
    bool is_changed = length_squared(sub(v_new, preferred)) >= AVOIDANCE_MIN_CHANGE_SQUARED;
    if (!is_avoidance_on || !is_changed || !is_way_clear(unit, ahead))
        return preferred;
    unit->is_orca_avoiding = true;
    unit->orca_avoidance_seconds += TICK_SECONDS;

    // A squad member that the avoidance keeps slow against a fast preferred velocity waits. After
    // `ORCA_WAIT_LIMIT_SECONDS`, the avoidance pauses for `AVOIDANCE_OFF_SECONDS`.
    float v_wait = WAIT_SPEED_FRACTION * unit->top_speed;
    bool is_waiting = len(unit->velocity) < v_wait && len(preferred) > v_wait;
    if (is_waiting)
        unit->orca_wait_seconds += TICK_SECONDS;
    else
        unit->orca_wait_seconds = 0;
    if (unit->orca_wait_seconds > ORCA_WAIT_LIMIT_SECONDS) {
        unit->orca_wait_seconds = 0;
        unit->avoidance_off_seconds = AVOIDANCE_OFF_SECONDS;
    }
    return v_new;
}


// `get_bend_point_distance` returns the route position of the lane point that the bend check of a
// squad member reads. With `--driving`, the point lies farther ahead, so that short chords to the
// own lane do not read as bends.
static float get_bend_point_distance(const Squad *squad, const Unit *unit, float speed) {
    // Without `--driving`, the point is the nearer of a point ahead of the squad member and a
    // point ahead of its slot.
    if (is_driving_on) {
        float look_seconds = get_tuning(unit->unit_type)->far_bend_look_seconds;
        float look = fmaxf(BEND_LOOK_MIN_METERS, look_seconds * speed);
        return unit->route_progress + look;
    }
    float look = fmaxf(BEND_LOOK_MIN_METERS, BEND_LOOK_SECONDS * speed);
    float slot_progress = fmaxf(unit->route_progress, get_slot_distance(squad, unit));
    float d_own = unit->route_progress + look;
    float d_slot = slot_progress + look * BEND_SLOT_FRACTION;
    return fminf(d_own, d_slot);
}


// `limit_speed_for_bend` limits `target_speed` to the bend maximum speed of a squad member: the
// speed at which its turn rate keeps it on the arc that leaves along its heading and reaches its
// lane point ahead, times `BEND_SPEED_MARGIN`.
//
//            bend point
//                *
//          chord /  .
//               /     .  arc of radius `r`
//              /       .
//    unit --> +- - - - - -> heading
//
static float limit_speed_for_bend(const Squad *squad, const Unit *unit, float target_speed) {
    // A bend point very close or almost straight ahead gives no limit.
    float speed = len(unit->velocity);
    float d = get_bend_point_distance(squad, unit, speed);
    V2 bend_point = get_lane_point(squad, unit, d);
    V2 chord = sub(bend_point, unit->position);
    float c = len(chord);        // chord length
    if (c <= BEND_MIN_DISTANCE_METERS)
        return target_speed;
    float sine = fabsf(det2(unit->heading, chord)) / c;
    if (sine <= BEND_MIN_SINE)
        return target_speed;

    // A chord of length `c` at the angle `a` to the tangent of an arc has the arc radius
    // `r = c / (2 * sin(a))`. The turn rate `w` drives that arc at the speed `w * r`.
    float v_arc = unit->unit_type->turn_rate * c / (2 * sine);
    float v_bend = v_arc * BEND_SPEED_MARGIN;
    float v_floor = BEND_FLOOR_FRACTION * unit->top_speed;
    float v_max = fmaxf(v_bend, v_floor);
    return fminf(target_speed, v_max);
}


// `steer_proportionally` returns a steering input in proportion to the angle from the heading of a
// unit to `direction`, which closes that angle in about the heading time constant of the faction
// tuning for every turn rate (`--driving`).
static float steer_proportionally(const Unit *unit, V2 direction) {
    // `angle` is positive to the left of the heading, and `turn` is the turn per time constant. A
    // positive steering input turns the unit clockwise, to the right, as seen from above.
    if (length_squared(direction) == 0)
        return 0;
    float angle = atan2f(det2(unit->heading, direction), dot(unit->heading, direction));
    float time_constant = get_tuning(unit->unit_type)->heading_time_constant_seconds;
    float turn = unit->unit_type->turn_rate * time_constant;
    return clamp_float(-angle / turn, -1, 1);
}


// `steer_toward` returns the steering input that makes a unit move along `direction`, with its
// rear first while it drives backward: proportional with `--driving`, and the steering of the base
// game otherwise.
static float steer_toward(const Unit *unit, V2 direction) {
    V2 heading_target = get_heading_target(unit, direction);
    if (is_driving_on)
        return steer_proportionally(unit, heading_target);
    return steer_to(unit, heading_target);
}


// `get_pursuit_speed` returns the speed of a pursuing squad member: the anchor speed, plus
// `CATCH_UP_GAIN` times the lag `behind_slot`, within zero and the top speed of the unit. A squad
// member ahead of its slot slows down gently (`slow_down_ahead`).
static float get_pursuit_speed(const Squad *squad, const Unit *unit, float behind_slot) {
    float v = squad->anchor_speed + CATCH_UP_GAIN * behind_slot;        // catch-up speed
    if (behind_slot < 0)
        v = slow_down_ahead(squad, unit, v);
    return clamp_float(v, 0, unit->top_speed);
}


// `get_end_speed` returns the speed at which a squad member can still stop on its final parking
// position, braking with `END_BRAKING_FRACTION` of its friction from its position after one tick.
static float get_end_speed(const Unit *unit, float parking_distance, float speed) {
    // A body that brakes at the deceleration `b` from `sqrt(2 * b * d)` stops within `d`.
    float friction = unit->unit_type->friction;
    float d = fmaxf(0, parking_distance - speed * TICK_SECONDS);        // braking distance
    float v_squared = 2 * END_BRAKING_FRACTION * friction * d;
    float v = sqrtf(v_squared);
    return fminf(unit->top_speed, v);
}


// `is_waiting_ahead` tells whether a squad member just past its slot waits instead of turning back
// to it (`--formation`): its drive direction points forward, its slot lies behind it, and
// `preferred` has almost no forward speed.
static bool is_waiting_ahead(const Unit *unit, V2 forward, V2 to_target, V2 preferred,
                             bool is_driving_at_slot) {
    if (!is_formation_keeping_on || !is_driving_at_slot)
        return false;
    V2 drive = get_drive_direction(unit);
    bool is_facing_forward = dot(drive, forward) >= WAIT_AHEAD_MIN_COSINE;
    bool is_slot_behind = dot(to_target, drive) < 0;
    bool is_slow_forward = dot(preferred, drive) < STEER_MIN_SPEED;
    return is_facing_forward && is_slot_behind && is_slow_forward;
}


// `update_route_progress` projects a squad member onto the route of its squad, near its last
// route progress. With lanes, the route progress can move back by up to `PROGRESS_BEHIND_METERS`,
// and without lanes, it never moves back.
static void update_route_progress(const Squad *squad, Unit *unit) {
    if (are_lanes_on) {
        float from = fmaxf(0, unit->route_progress - PROGRESS_BEHIND_METERS);
        float to = unit->route_progress + PROGRESS_AHEAD_METERS;
        unit->route_progress = project_onto_route(squad, unit->position, from, to);
        return;
    }
    float from = unit->route_progress;
    float to = unit->route_progress + PROGRESS_AHEAD_METERS;
    float projected = project_onto_route(squad, unit->position, from, to);
    unit->route_progress = fmaxf(unit->route_progress, projected);
}


// `choose_drive_target` picks the target of a squad member from its slot: a rejoin point when the
// squad member is far behind its slot or the slot is out of sight, a pursuit point when it is a
// few meters off its slot (`--driving`), and the slot otherwise.
static DriveTarget choose_drive_target(const Squad *squad, const Unit *unit, V2 slot,
                                       float slot_distance, float remaining) {
    // Without lanes, only the line of sight to the slot decides about a rejoin.
    DriveTarget drive = {
        .point = slot,
        .is_rejoining = false,
        .is_pursuing = false,
        .pursuit_speed = 0,
    };
    if (!are_lanes_on) {
        drive.is_rejoining = !is_way_clear(unit, slot);
        if (drive.is_rejoining)
            drive.point = find_rejoin_target_without_lanes(squad, unit);
        return drive;
    }

    // Pursuit stops on the end blend, where the slot moves onto the final parking position.
    float behind_slot = slot_distance - unit->route_progress;
    drive.is_rejoining = behind_slot > REJOIN_BEHIND_METERS || !is_way_clear(unit, slot);
    bool is_before_end_blend = remaining > END_BLEND_METERS;
    float pursuit_slot_meters = get_tuning(unit->unit_type)->pursuit_slot_meters;
    bool is_far_from_slot = len(sub(slot, unit->position)) > pursuit_slot_meters;
    if (drive.is_rejoining) {
        drive.point = find_rejoin_target(squad, unit, slot, slot_distance);
    } else if (is_driving_on && is_before_end_blend && is_far_from_slot) {
        drive.is_pursuing = find_pursuit_target(squad, unit, &drive.point);
        drive.pursuit_speed = get_pursuit_speed(squad, unit, behind_slot);
    }
    return drive;
}


// `apply_end_section` aims a squad member at its final parking position on the end section of the
// route (`--driving`), when that position is in sight. It returns true in that case, and the
// squad member then approaches its final parking position at the end speed.
static bool apply_end_section(const Unit *unit, float remaining, bool is_detouring,
                              DriveTarget *drive) {
    // A squad member on the end section without its final parking position in sight drives
    // straight at its target, as for a rejoin.
    bool is_near_end = remaining <= END_BLEND_METERS;
    bool is_end_section = is_driving_on && is_near_end && !is_detouring;
    bool is_approaching = is_end_section && is_way_clear(unit, unit->parking_position);
    if (is_approaching) {
        drive->point = unit->parking_position;
        drive->is_rejoining = false;
        drive->is_pursuing = false;
    } else if (is_end_section) {
        drive->is_rejoining = true;
        drive->is_pursuing = false;
    }
    return is_approaching;
}


// `get_preferred_velocity` returns the velocity of a squad member toward its target before the
// ORCA avoidance. Near its slot, the squad member moves with the anchor and closes the gap to the
// slot at `SLOT_GAIN`, and its speed always allows a stop at the target.
static V2 get_preferred_velocity(const Squad *squad, const Unit *unit, const DriveTarget *drive,
                                 V2 forward, V2 to_target, float target_distance,
                                 bool is_approaching, float end_speed) {
    // The speed that allows a stop at the target, braking with `ARRIVAL_BRAKING_FRACTION` of the
    // friction, comes on top of the anchor speed.
    float friction = unit->unit_type->friction;
    float d_brake = fmaxf(0, target_distance - ARRIVAL_SLACK_METERS);        // braking distance
    float v_brake = sqrtf(2 * ARRIVAL_BRAKING_FRACTION * friction * d_brake);
    float v_max = fminf(unit->top_speed, v_brake + squad->anchor_speed);

    // A pursuit sets its own speed, also above `v_max`.
    V2 v;
    if (is_approaching) {
        v = get_velocity_toward(to_target, target_distance, end_speed);
    } else if (drive->is_rejoining) {
        v = get_velocity_toward(to_target, target_distance, v_max);
    } else {
        V2 anchor_motion = mul(forward, squad->anchor_speed);
        V2 correction = mul(to_target, SLOT_GAIN);
        v = add(anchor_motion, correction);
    }
    float s = len(v);        // preferred speed
    if (s > v_max)
        v = mul(v, v_max / s);
    if (drive->is_pursuing)
        v = get_velocity_toward(to_target, target_distance, drive->pursuit_speed);
    return v;
}


// `is_heading_blocked` runs the brake check of a squad member: it tells whether the point one
// brake look time (faction tuning) ahead along its drive direction, at least its radius plus a
// margin away, is out of sight.
static bool is_heading_blocked(const Unit *unit, float speed) {
    float d_min = unit->unit_type->radius + BRAKE_LOOK_MARGIN_METERS;
    float look_seconds = get_tuning(unit->unit_type)->brake_look_seconds;
    float d = fmaxf(speed * look_seconds, d_min);        // look distance
    V2 look_point = add(unit->position, mul(get_drive_direction(unit), d));
    return !is_way_clear(unit, look_point);
}


// `get_target_speed` returns the target speed of a squad member from its velocity after the ORCA
// avoidance: the part along its heading, capped by the end speed on the final approach, and with
// lanes, by the bend maximum speed and the brake check.
static float get_target_speed(const Squad *squad, const Unit *unit, V2 v_new, float s_new,
                              float speed, bool is_approaching, float end_speed) {
    // The cap by the end speed keeps the ORCA avoidance from pushing a braking squad member past
    // its final parking position.
    float target_speed = 0;
    if (s_new > 0)
        target_speed = s_new * get_forward_share(unit, v_new, s_new);
    if (is_approaching)
        target_speed = fminf(target_speed, end_speed);
    if (are_lanes_on) {
        target_speed = limit_speed_for_bend(squad, unit, target_speed);
        if (is_heading_blocked(unit, speed))
            target_speed = 0;
    }
    return target_speed;
}


// `find_backward_target` returns the point that a squad member driving backward drives at: the
// farthest point in sight on its lane ahead, or else a rejoin point on its lane or on the route.
static V2 find_backward_target(const Squad *squad, const Unit *unit) {
    V2 target;
    if (find_pursuit_target(squad, unit, &target))
        return target;
    V2 lane_point = get_lane_point(squad, unit, unit->route_progress);
    return find_rejoin_target(squad, unit, lane_point, unit->route_progress + REJOIN_LOOK_METERS);
}


// `steer_backward` drives a squad member that drives backward (`--reverse`) for one tick, at its
// own pace and without formation keeping: it turns on the spot until its rear points along its
// way, follows its own lane along the route, then backs onto its final parking position, where it
// stops. It sets the throttle and returns the steering.
static float steer_backward(Unit *unit, const Squad *squad, int unit_index) {
    // The end approach starts on the last part of the route that the unit itself has left, once
    // its final parking position is in sight; the anchor plays no part.
    float speed = len(unit->velocity);
    update_route_progress(squad, unit);
    float own_remaining = squad->route_length - unit->route_progress;
    bool is_near_end = own_remaining <= END_BLEND_METERS;
    bool is_approaching = is_near_end && is_way_clear(unit, unit->parking_position);
    V2 target = is_approaching ? unit->parking_position : find_backward_target(squad, unit);
    unit->waypoint = target;
    V2 to_target = sub(target, unit->position);
    float d_target = len(to_target);        // distance to the target
    bool is_parked = is_approaching && d_target < END_STOP_REACH_METERS && speed < PARKING_SPEED;
    if (is_parked) {
        unit->is_stopped = true;
        return steer_to(unit, unit->facing);
    }

    // At the start, the unit turns on the spot until its rear points at its target.
    if (unit->is_aligning_to_reverse && d_target > 0) {
        float alignment = dot(get_drive_direction(unit), to_target) / d_target;
        unit->is_aligning_to_reverse = alignment < REVERSE_ALIGN_COSINE;
    }
    if (unit->is_aligning_to_reverse) {
        accelerate_toward_speed(unit, 0, FORM_SPEED_TIME_CONSTANT_SECONDS);
        return steer_toward(unit, to_target);
    }

    // The unit drives at its own top speed, and on the approach at the speed that lets it stop on
    // its final parking position.
    float end_speed = unit->top_speed;
    if (is_approaching)
        end_speed = get_end_speed(unit, d_target, speed);
    V2 preferred = get_velocity_toward(to_target, d_target, end_speed);
    V2 v_new = apply_avoidance(unit_index, preferred);
    float s_new = len(v_new);
    float target_speed = get_target_speed(squad, unit, v_new, s_new, speed, is_approaching,
                                          end_speed);
    accelerate_toward_speed(unit, target_speed, FORM_SPEED_TIME_CONSTANT_SECONDS);
    if (s_new > STEER_MIN_SPEED)
        return steer_toward(unit, v_new);
    if (d_target > STEER_MIN_DISTANCE_METERS)
        return steer_toward(unit, to_target);
    return 0;
}


// `steer_in_formation` drives a squad member of a moving squad for one tick: it sets the throttle
// and returns the steering input. A squad member that stops on its final parking position at the
// route end turns to its facing. A squad member that drives backward keeps no formation.
float steer_in_formation(int unit_index) {
    // `remaining` is the part of the route past the anchor.
    Unit *unit = &units[unit_index];
    const Squad *squad = &squads[unit->squad_index];
    if (unit->is_reversing)
        return steer_backward(unit, squad, unit_index);
    float speed = len(unit->velocity);
    float remaining = squad->route_length - squad->anchor_distance;
    V2 forward;
    V2 slot = get_target_slot(squad, unit, &forward);
    float slot_distance = get_slot_distance(squad, unit);
    update_route_progress(squad, unit);
    DriveTarget drive = choose_drive_target(squad, unit, slot, slot_distance, remaining);

    // A detour replaces the target, and the squad member drives straight at it.
    bool is_detouring = false;
    if (is_detour_on)
        is_detouring = follow_detour(squad, unit, slot, slot_distance, &drive.point);
    if (is_detouring) {
        drive.is_rejoining = true;
        drive.is_pursuing = false;
    }
    bool is_approaching = apply_end_section(unit, remaining, is_detouring, &drive);
    unit->waypoint = drive.point;
    V2 to_target = sub(drive.point, unit->position);
    float d_target = len(to_target);        // distance to the target

    // At the route end, a slow squad member at its target stops there. On the final approach, the
    // target counts as reached within `END_STOP_REACH_METERS`.
    float end_distance = squad->route_length - ROUTE_END_EPSILON_METERS;
    bool is_anchor_at_end = squad->anchor_distance >= end_distance;
    float parking_reach = is_approaching ? END_STOP_REACH_METERS : unit->reach;
    bool is_slow_at_target = d_target < parking_reach && speed < PARKING_SPEED;
    bool is_on_parking_position = !is_detouring && is_slow_at_target;
    if (is_anchor_at_end && is_on_parking_position) {
        unit->is_stopped = true;
        return steer_to(unit, unit->facing);
    }

    // A squad member just past its slot waits instead of turning back to it.
    float end_speed = unit->top_speed;
    if (is_approaching)
        end_speed = get_end_speed(unit, d_target, speed);
    V2 preferred = get_preferred_velocity(squad, unit, &drive, forward, to_target, d_target,
                                          is_approaching, end_speed);
    bool is_before_end_blend = remaining > END_BLEND_METERS;
    bool is_driving_at_slot = is_before_end_blend && !drive.is_rejoining && !drive.is_pursuing;
    bool is_waiting = is_waiting_ahead(unit, forward, to_target, preferred, is_driving_at_slot);
    if (is_waiting)
        preferred = v2(0, 0);

    // The throttle follows the target speed, and the steering follows the new velocity. A slow
    // squad member steers at its target instead, unless it waits.
    V2 v_new = apply_avoidance(unit_index, preferred);
    float s_new = len(v_new);
    float target_speed = get_target_speed(squad, unit, v_new, s_new, speed, is_approaching,
                                          end_speed);
    accelerate_toward_speed(unit, target_speed, FORM_SPEED_TIME_CONSTANT_SECONDS);
    if (s_new > STEER_MIN_SPEED)
        return steer_toward(unit, v_new);
    if (is_waiting)
        return 0;
    if (d_target > STEER_MIN_DISTANCE_METERS)
        return steer_toward(unit, to_target);
    return 0;
}
