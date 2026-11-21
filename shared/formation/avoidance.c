// ORCA avoidance between units in formation following (optimal reciprocal collision avoidance, the
// method of the RVO2 library). For each nearby unit, `make_orca_line` builds a half-plane of
// velocities that keep the two units apart for a horizon time (`ORCA_HORIZON_SECONDS`). The solver
// then picks the permitted velocity closest to the preferred one. When no velocity satisfies every
// half-plane, it picks the velocity with the smallest largest violation.
//
// Two moving units share the change of velocity that keeps them apart. A unit that stands still
// or turns on the spot cannot dodge, so the other unit makes the whole change. Blocking terrain is
// not part of the avoidance, because constraints from it would fight base game routes that run
// along blocking terrain. A stopped squad member of a moving squad steps aside for the other
// squad members (`step_aside_for_members`).
//
// Velocities are in meters per second, in the map frame: x to the right and y up, as seen from
// above.
#include "sim.h"
#include "formation.h"

#define ORCA_HORIZON_SECONDS 2.0f               // time for which two units stay apart
#define ORCA_MAX_LINES 64                       // most lines, one per nearby unit
#define MEMBER_AVOID_RANGE_METERS 2.0f          // largest gap for avoidance in one squad
#define SHARED_EFFORT 0.5f                      // share of the change for each moving unit
#define FULL_EFFORT 1.0f                        // share of the change when the other is still
#define PARALLEL_EPSILON 1e-5f                  // largest determinant of parallel directions
#define LENGTH_EPSILON 1e-6f                    // a shorter vector has no direction
#define OVERLAP_METERS 0.01f                    // closer units count as on one point
#define SPEED_TIME_CONSTANT_SECONDS 0.3f        // speed time constant when stepping aside
#define MIN_STEP_ASIDE_SPEED 1.0f               // slowest avoidance speed for a step aside


// Boundary of a half-plane of permitted velocities: the line through `point` along `direction`, a
// unit vector. The permitted velocities lie on the left of the line, as seen along `direction`.
typedef struct {
    V2 point;
    V2 direction;
} Line;


// `get_line_violation` returns the distance by which `velocity` lies on the forbidden side of
// `line`. The result is zero or negative for a permitted velocity.
static float get_line_violation(const Line *line, V2 velocity) {
    V2 from_velocity = sub(line->point, velocity);
    return det2(line->direction, from_velocity);
}


// `find_best_on_line` finds the velocity on the line `line_index` within the speed limit that
// satisfies all earlier lines: the one closest to `preferred`, or with `is_direction_only`, the
// one farthest along `preferred`. It returns false when no velocity on the line satisfies them.
static bool find_best_on_line(const Line *lines, int line_index, float top_speed, V2 preferred,
                              bool is_direction_only, V2 *result) {
    // A point of the line is `point + t * direction`. The speed limit keeps `t` between the
    // crossings with the speed circle, the roots of a quadratic with discriminant `disc`.
    const Line *line = &lines[line_index];
    float proj = dot(line->point, line->direction);
    float p_squared = length_squared(line->point);
    float disc = proj * proj + top_speed * top_speed - p_squared;
    if (disc < 0)
        return false;
    float root = sqrtf(disc);
    float t_min = -proj - root;
    float t_max = -proj + root;

    // Each earlier line limits `t` at its crossing with this line. A parallel earlier line permits
    // either the whole line or none of it.
    for (int earlier_index = 0; earlier_index < line_index; earlier_index++) {
        const Line *earlier = &lines[earlier_index];
        float den = det2(line->direction, earlier->direction);
        float num = det2(earlier->direction, sub(line->point, earlier->point));
        if (fabsf(den) <= PARALLEL_EPSILON) {
            if (num < 0)
                return false;
            continue;
        }
        float t_cross = num / den;        // `t` at the crossing with the earlier line
        if (den >= 0) {
            if (t_cross < t_max)
                t_max = t_cross;
        } else if (t_cross > t_min) {
            t_min = t_cross;
        }
        if (t_min > t_max)
            return false;
    }

    // In direction mode, only the sign of the alignment matters, because the result is one end of
    // the range of `t`.
    float t;
    if (is_direction_only) {
        float alignment = dot(preferred, line->direction);
        t = alignment > 0 ? t_max : t_min;
    } else {
        float t_preferred = dot(line->direction, sub(preferred, line->point));
        t = clamp_float(t_preferred, t_min, t_max);
    }
    *result = add(line->point, mul(line->direction, t));
    return true;
}


// `find_permitted_velocity` finds the velocity within the speed limit that satisfies all lines and
// is closest to `preferred`, or with `is_direction_only`, farthest along `preferred`. It returns
// the index of the first line that it cannot satisfy, or `line_count` when it satisfies all lines.
static int find_permitted_velocity(const Line *lines, int line_count, float top_speed,
                                   V2 preferred, bool is_direction_only, V2 *result) {
    // In direction mode, `preferred` must be a unit vector, and the start value is the speed limit
    // along it.
    if (is_direction_only)
        *result = mul(preferred, top_speed);
    else if (length_squared(preferred) > top_speed * top_speed)
        *result = mul(preferred, top_speed / len(preferred));
    else
        *result = preferred;

    // A violated line moves the result onto that line, at the best point that the earlier lines
    // permit. When there is no such point, the result keeps its last value.
    for (int line_index = 0; line_index < line_count; line_index++) {
        if (get_line_violation(&lines[line_index], *result) <= 0)
            continue;
        V2 previous = *result;
        bool is_solved = find_best_on_line(lines, line_index, top_speed, preferred,
                                           is_direction_only, result);
        if (!is_solved) {
            *result = previous;
            return line_index;
        }
    }
    return line_count;
}


// `find_least_violating_velocity` finds the velocity within the speed limit with the smallest
// largest violation, for lines that no velocity satisfies together. `result` holds the result of
// `find_permitted_velocity` on input, which satisfies the lines before `first_failed_index`.
static void find_least_violating_velocity(const Line *lines, int line_count,
                                          int first_failed_index, float top_speed, V2 *result) {
    static Line bisectors[ORCA_MAX_LINES];
    float worst = 0;        // largest violation so far
    for (int line_index = first_failed_index; line_index < line_count; line_index++) {
        // Only a line with a larger violation than `worst` changes the result.
        const Line *line = &lines[line_index];
        if (get_line_violation(line, *result) <= worst)
            continue;

        // Each bisector bounds the velocities at which an earlier line has no larger violation
        // than `line`. Two parallel lines that point the same way have no bisector.
        int count = 0;        // number of bisectors
        for (int other_index = 0; other_index < line_index; other_index++) {
            const Line *other = &lines[other_index];
            float det = det2(line->direction, other->direction);
            V2 point;
            if (fabsf(det) <= PARALLEL_EPSILON) {
                if (dot(line->direction, other->direction) > 0)
                    continue;
                point = mul(add(line->point, other->point), 0.5f);
            } else {
                V2 between = sub(line->point, other->point);
                float t = det2(other->direction, between) / det;
                point = add(line->point, mul(line->direction, t));
            }
            V2 turn = sub(other->direction, line->direction);        // change between directions
            V2 dir = mul(turn, 1 / len(turn));
            bisectors[count] = (Line){
                .point = point,
                .direction = dir,
            };
            count++;
        }

        // The search runs along the normal of `line`, toward its permitted side. The previous
        // result satisfies every bisector, so a failure could only come from rounding.
        V2 previous = *result;
        V2 normal = v2(-line->direction.y, line->direction.x);
        bool is_direction_only = true;
        int failed_index = find_permitted_velocity(bisectors, count, top_speed, normal,
                                                   is_direction_only, result);
        if (failed_index < count)
            *result = previous;
        worst = get_line_violation(line, *result);
    }
}


// `get_from_cutoff_center` returns `w`, the vector from the center of the cutoff circle to the
// relative velocity. The cutoff circle has its center at `relative_position / time_seconds`.
static V2 get_from_cutoff_center(V2 relative_position, V2 relative_velocity, float time_seconds) {
    V2 cutoff_center = mul(relative_position, 1 / time_seconds);
    return sub(relative_velocity, cutoff_center);
}


// `get_cutoff_correction` computes the smallest change of the relative velocity that reaches the
// cutoff circle, and the direction of the circle edge there. `from_center` is `w`.
static void get_cutoff_correction(V2 from_center, float cutoff_radius, V2 *direction,
                                  V2 *correction) {
    // A relative velocity at the center has no closest direction, so `w_unit` then points along x.
    float w_length = len(from_center);
    V2 w_unit = v2(1, 0);
    if (w_length > LENGTH_EPSILON)
        w_unit = mul(from_center, 1 / w_length);
    *direction = v2(w_unit.y, -w_unit.x);
    *correction = mul(w_unit, cutoff_radius - w_length);
}


// `get_cone_leg_direction` returns the unit direction of one leg of the velocity obstacle, from
// zero toward the tangent point on the circle of radius `combined_radius` around
// `relative_position`, or for the right leg, the opposite direction.
static V2 get_cone_leg_direction(V2 relative_position, float combined_radius, bool is_left_leg) {
    float d_squared = length_squared(relative_position);
    float leg = sqrtf(d_squared - combined_radius * combined_radius);        // leg length
    float side = is_left_leg ? 1.0f : -1.0f;
    float x = relative_position.x;
    float y = relative_position.y;
    float leg_x = x * leg - side * y * combined_radius;
    float leg_y = side * x * combined_radius + y * leg;
    return mul(v2(leg_x, leg_y), side / d_squared);
}


// `make_orca_line` builds the line of permitted velocities that keeps a unit away from one other
// unit. It finds `u`, the smallest change of the relative velocity that reaches the edge of the
// velocity obstacle, and the unit takes the share `effort` of that change.
//
//                                    _.-'   left leg
//                               _.-'
//                    .--.  _.-'
//    0              ( +  )               velocity obstacle
//                    '--'  `-._
//                               `-._
//                                    `-._   right leg
//
//    +: center of the cutoff circle, `relative_position / tau`, radius `combined_radius / tau`
//
// The velocity obstacle holds the relative velocities that would bring the bodies into contact
// within the time `tau`. The left leg lies on the left as seen from zero toward the circle.
//
static Line make_orca_line(V2 relative_position, V2 relative_velocity, float combined_radius,
                           float effort, V2 velocity, float horizon_seconds) {
    // Bodies in contact use one tick as `tau`, so that the result separates them in the next tick.
    float r_squared = combined_radius * combined_radius;
    bool is_apart = length_squared(relative_position) > r_squared;
    float tau = is_apart ? horizon_seconds : TICK_SECONDS;
    V2 w = get_from_cutoff_center(relative_position, relative_velocity, tau);
    float r_cut = combined_radius / tau;        // radius of the cutoff circle

    // The cutoff circle is the closest edge when `w` points back toward zero, inside the angle of
    // the cone. Otherwise, the closest edge is one of the two legs.
    float proj = dot(w, relative_position);
    float limit_squared = r_squared * length_squared(w);
    bool is_on_cutoff_disc = proj < 0 && proj * proj > limit_squared;
    V2 dir;
    V2 u;
    if (!is_apart || is_on_cutoff_disc) {
        get_cutoff_correction(w, r_cut, &dir, &u);
    } else {
        bool is_left_leg = det2(relative_position, w) > 0;
        dir = get_cone_leg_direction(relative_position, combined_radius, is_left_leg);
        float along = dot(relative_velocity, dir);
        u = sub(mul(dir, along), relative_velocity);
    }

    // The line passes through the velocity of the unit, moved by its share of `u`.
    V2 own = mul(u, effort);
    return (Line){
        .point = add(velocity, own),
        .direction = dir,
    };
}


// `separate_overlap` returns `relative_position`, the position of the unit `other_index` relative
// to the unit `unit_index`. For two units on one point, it returns instead a short offset along x
// whose sign depends on the order of the unit indexes, so that the two units move apart.
static V2 separate_overlap(V2 relative_position, int unit_index, int other_index) {
    if (length_squared(relative_position) >= OVERLAP_METERS * OVERLAP_METERS)
        return relative_position;

    // With the same position and velocity, both units of the pair would get the push along x of
    // `get_cutoff_correction`, and they would slide together without ever separating.
    float side = unit_index < other_index ? 1.0f : -1.0f;
    return v2(side * OVERLAP_METERS, 0);
}


// `compute_orca_velocity` computes the velocity of a unit closest to `preferred`, within its top
// speed, that keeps it away from the nearby units. `avoided_units` selects all units, or only the
// moving squad members of the squad of the unit.
V2 compute_orca_velocity(int unit_index, V2 preferred, int avoided_units) {
    // Units past the first `ORCA_MAX_LINES` lines get no line.
    static Line lines[ORCA_MAX_LINES];
    const Unit *unit = &units[unit_index];
    float speed = len(unit->velocity);
    float margin = get_tuning(unit->unit_type)->orca_margin_meters;        // extra body gap
    float horizon = ORCA_HORIZON_SECONDS;                                  // horizon time
    int count = 0;        // number of lines
    for (int other_index = 0; other_index < unit_count; other_index++) {
        if (count == ORCA_MAX_LINES)
            break;
        if (other_index == unit_index || units[other_index].is_carried)
            continue;

        // With `--ignoreboarders`, the units that drive to board the unit, an APC, get no line:
        // swerving away from them would keep them from reaching it.
        if (are_boarders_ignored && is_boarding(&units[other_index], unit_index))
            continue;

        // Only units within the distance that the two units could close in the horizon time get a
        // line. Squad members of one squad avoid each other only near contact.
        const Unit *other = &units[other_index];
        V2 offset = sub(other->position, unit->position);
        V2 p = separate_overlap(offset, unit_index, other_index);   // position of `other`, relative
        float r_sum = unit->unit_type->radius + other->unit_type->radius;
        float r = r_sum + margin;                           // combined radius
        float v_other = len(other->velocity);               // speed of `other`
        bool is_same_squad = other->squad_index == unit->squad_index;
        float range = r + (speed + v_other) * horizon;
        if (is_same_squad)
            range = r + MEMBER_AVOID_RANGE_METERS;
        if (length_squared(p) > range * range)
            continue;

        // With `ORCA_AVOID_MOVING_MEMBERS`, units that stand still or turn on the spot, and units
        // of other squads, get no line.
        bool is_still = other->is_stopped || other->is_reorienting;
        bool is_wrong_squad = avoided_units == ORCA_AVOID_MOVING_MEMBERS && !is_same_squad;
        if (avoided_units != ORCA_AVOID_ALL && (is_still || is_wrong_squad))
            continue;

        // A still unit counts as standing, and the unit makes the whole change.
        V2 velocity = other->velocity;
        if (is_still)
            velocity = v2(0, 0);
        float effort = is_still ? FULL_EFFORT : SHARED_EFFORT;
        V2 v_rel = sub(unit->velocity, velocity);        // relative velocity
        lines[count] = make_orca_line(p, v_rel, r, effort, unit->velocity, horizon);
        count++;
    }

    // When no velocity satisfies all lines, `find_least_violating_velocity` gives the best result.
    V2 result;
    float top_speed = unit->top_speed;
    bool is_direction_only = false;
    int failed_index = find_permitted_velocity(lines, count, top_speed, preferred,
                                               is_direction_only, &result);
    if (failed_index < count)
        find_least_violating_velocity(lines, count, failed_index, top_speed, &result);
    return result;
}


// `step_aside` drives a stopped unit toward `new_velocity` from the ORCA avoidance and returns the
// steering toward it. The target speed counts only the part of `new_velocity` along the heading.
static float step_aside(Unit *unit, V2 new_velocity, float new_speed) {
    unit->is_orca_avoiding = true;
    unit->orca_avoidance_seconds += TICK_SECONDS;
    float target_speed = new_speed * get_forward_share(unit, new_velocity, new_speed);
    accelerate_toward_speed(unit, target_speed, SPEED_TIME_CONSTANT_SECONDS);
    return steer_to(unit, new_velocity);
}


// `step_aside_for_members` moves a stopped unit of a moving squad out of the way of the moving
// squad members of its squad, and returns its steering. A unit that stepped aside drives back to
// its final parking position once it is out of reach of it.
float step_aside_for_members(int unit_index) {
    // With a preferred velocity of zero, any velocity from the avoidance is a move out of the way.
    // The unit steps aside only when that move is fast and its path for one second is clear.
    Unit *unit = &units[unit_index];
    V2 v = compute_orca_velocity(unit_index, v2(0, 0), ORCA_AVOID_MOVING_MEMBERS);
    V2 ahead = add(unit->position, v);        // position after one second at `v`
    float speed = len(v);
    if (speed > MIN_STEP_ASIDE_SPEED && is_way_clear(unit, ahead)) {
        unit->has_stepped_aside = true;
        return step_aside(unit, v, speed);
    }

    // `waypoint` holds the point where the unit stopped. Clearing `is_stopped` makes formation
    // following drive the unit back there.
    float d_squared = length_squared(sub(unit->waypoint, unit->position));
    if (unit->has_stepped_aside && d_squared > unit->reach * unit->reach) {
        unit->has_stepped_aside = false;
        unit->is_stopped = false;
    }
    return steer_to_facing(unit);
}
