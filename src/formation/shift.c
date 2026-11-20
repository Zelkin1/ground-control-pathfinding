// Lane shifts of the coordination mode (`--avoid coord`). A shift moves all lanes of a squad
// sideways over a part of its route, so that two squads can go past each other in a head-on event
// or a pass event.
//
// Each squad stores its shift in these fields:
// - `shift`: the current sideways offset of all lanes of the squad inside the section, in meters,
//   positive to the right of the formation direction as seen facing along it.
// - `shift_target`: the target shift, toward which `update_shift` adjusts `shift`.
// - `shift_rate`: the rate of change of `shift` over time, in meters per second.
// - `shift_section_start` and `shift_section_end`: the two route positions that delimit the
//   section, the part of the route with the full shift. Outside the section, a transition on each
//   side brings the lane offset down to zero.
// A route position is a distance along the route from the route start.
//
// Head-on events (coordination.c) and pass events (passing.c) plan their target shifts with the
// same functions of this file. The event code computes the missing gap, the part of the needed gap
// (see `get_needed_gap`) that the current sideways distance between the two routes does not give.
// It then fills one `TargetShift` per squad with a side, a section and a side room, splits the
// missing gap into two shift magnitudes, and calls `record_target_shifts`.
//
// In each tick, `coordinate_squads` saves the target shift and the section of every squad, and
// calls `clear_shift_target_and_event`. After the event detection, it calls
// `keep_shift_target_steady` with the saved values and then `update_shift`. `get_lane_point`
// (lanes.c) adds the result of `get_lane_shift` to the sideways position of every lane.
//
// Each squad member follows one lane. After a sharp sideways bend of its lane, a squad member that
// turns slowly could stay on the lane only by braking. For that reason, the change of the shift
// over time and the change of the lane offset along the route both have limits. These limits
// depend on `full_speed_min_radius`, the largest minimum turning radius at top speed among the
// squad members.
#include "sim.h"
#include "formation.h"

#define NO_ROUTE_MAX_SHIFT_SLOPE 0.3f             // shift limit per meter at top speed, no route
#define MIN_REFERENCE_SPEED_FRACTION 0.25f        // reference speed floor, fraction of top speed
#define MAX_SHIFT_SLOPE 0.15f                     // lane offset change limit per meter of travel

// Event of the highest rank that `flag_shift_event` recorded for each squad in the current tick,
// indexed by squad index. The rank follows the order of the `ShiftEvent` values.
static ShiftEvent shift_events[MAX_SQUADS];


// `measure_side_room` computes the side room of a squad toward `side` (1 for right, -1 for left)
// at the route position `distance`: how far all lanes can move sideways until the outermost lane
// on that side reaches the lane clamp. It ignores the current shift, so the result limits a shift.
float measure_side_room(const Squad *squad, float distance, float side) {
    // The smallest and largest planned sideways slot positions of the squad members give the two
    // outermost lanes, those of the formation that a head-on plans (`--narrowpass`).
    float x_min = LARGE_VALUE;         // smallest sideways slot position
    float x_max = -LARGE_VALUE;        // largest sideways slot position
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        float x = get_planned_slot(squad, &units[unit_index]).x;
        x_min = fminf(x_min, x);
        x_max = fmaxf(x_max, x);
    }

    // In column formation, all squad members share one lane at the middle of the cross-section.
    LaneCrossSection cs = get_lane_cross_section(squad, distance);
    float right = x_max * cs.scale + cs.offset;        // sideways position of the right lane
    float left = x_min * cs.scale + cs.offset;         // sideways position of the left lane
    if (squad->is_in_column || squad->is_column_planned) {
        right = cs.middle;
        left = cs.middle;
    }

    // A lane that lies beyond the lane clamp would give a negative side room.
    float room = cs.max_x - right;
    if (side < 0)
        room = left - cs.min_x;
    return fmaxf(0, room);
}


// `measure_min_side_room` computes the smallest side room of a squad toward `side` at route
// positions `step` apart, from `start` (at the route start at the earliest) up to `end`. It
// returns zero when no route position lies in that range.
float measure_min_side_room(const Squad *squad, float start, float end, float step, float side) {
    float room_min = LARGE_VALUE;                               // smallest side room
    for (float d = fmaxf(0, start); d <= end; d += step)        // route position
        room_min = fminf(room_min, measure_side_room(squad, d, side));
    if (room_min == LARGE_VALUE)
        return 0;
    return room_min;
}


// `get_needed_gap` computes the needed gap of two squads: the sideways distance between their
// routes at which the bodies of the two formations are `body_gap` apart.
float get_needed_gap(const FormationShape *first, const FormationShape *second, float body_gap) {
    return first->half_width + second->half_width + body_gap;
}


// `choose_shift_side` returns the side of the shift of a squad, 1 for right and -1 for left, as
// seen facing along its route. `other_route_offset` is the sideways offset of the other route.
// The side is away from that route, or right when that route is within `SAME_ROUTE_METERS`.
float choose_shift_side(float other_route_offset) {
    if (other_route_offset > SAME_ROUTE_METERS)
        return -1;
    return 1;
}


// `split_missing_gap_evenly` gives each of two squads half of the missing gap as shift magnitude.
// A squad with less side room than half takes its whole side room, and the other squad the rest.
// The caller makes sure that the two side rooms together cover the missing gap.
void split_missing_gap_evenly(float missing_gap, TargetShift *first, TargetShift *second) {
    float half = missing_gap / 2;
    float limit = fmaxf(half, missing_gap - second->side_room);        // first magnitude limit
    first->magnitude = fminf(first->side_room, limit);
    second->magnitude = missing_gap - first->magnitude;
}


// `split_missing_gap_in_order` gives the first squad as much of the missing gap as its side room
// allows, and the second squad as much of the rest as its own side room allows. It returns the
// missing gap minus the two shift magnitudes.
float split_missing_gap_in_order(float missing_gap, TargetShift *first, TargetShift *second) {
    first->magnitude = clamp_float(missing_gap, 0, first->side_room);
    float rest = missing_gap - first->magnitude;
    second->magnitude = clamp_float(rest, 0, second->side_room);
    return missing_gap - first->magnitude - second->magnitude;
}


// `clear_shift_target_and_event` sets the target shift of a squad to zero and its event to none,
// before the event detection of a tick. The section keeps its value for `get_lane_shift`.
void clear_shift_target_and_event(Squad *squad) {
    squad->shift_target = 0;
    shift_events[squad->squad_index] = SHIFT_EVENT_NONE;
}


// `flag_shift_event` records `event` for a squad in the current tick, unless the squad already
// has an event of the same or a higher rank.
void flag_shift_event(const Squad *squad, ShiftEvent event) {
    int squad_index = squad->squad_index;
    if (event > shift_events[squad_index])
        shift_events[squad_index] = event;
}


// `set_shift_target_of_larger_magnitude` sets the target shift and the section of a squad, unless
// its current target shift has the same or a larger magnitude. After the event detection, each
// squad thus has the target shift of the largest magnitude among its events, with its section.
static void set_shift_target_of_larger_magnitude(Squad *squad, float section_start,
                                                 float section_end, float target_shift) {
    if (fabsf(target_shift) <= fabsf(squad->shift_target))
        return;
    squad->shift_target = target_shift;
    squad->shift_section_start = section_start;
    squad->shift_section_end = section_end;
}


// `record_target_shifts` sets the target shift (side times magnitude) and the section of both
// squads of an event, and flags the event for both squads.
void record_target_shifts(const TargetShift *first, const TargetShift *second, ShiftEvent event) {
    const TargetShift *targets[] = {first, second};
    for (int target_index = 0; target_index < 2; target_index++) {
        const TargetShift *t = targets[target_index];
        float shift = t->side * t->magnitude;        // signed target shift
        set_shift_target_of_larger_magnitude(t->squad, t->section_start, t->section_end, shift);
        flag_shift_event(t->squad, event);
    }
}


// `update_shift` adjusts the shift of a squad toward its target shift, once per tick. It limits
// the shift rate and its rate of change, so that a squad member that follows its lane on a
// straight part of the route turns on a radius of about `full_speed_min_radius` or more.
void update_shift(Squad *squad) {
    float delta = squad->shift_target - squad->shift;          // remaining shift
    float v_top = measure_shape(squad).squad_top_speed;        // top speed of the squad

    // `full_speed_min_radius` is zero until `finish_formation_set_up` (formation.c) sets it for a
    // route. Without a route, the shift rate limit `r_max` is a fixed slope times the top speed.
    if (squad->full_speed_min_radius <= 0) {
        float r_max = NO_ROUTE_MAX_SHIFT_SLOPE * v_top;
        float ds_max = r_max * TICK_SECONDS;        // shift change limit per tick
        squad->shift += clamp_float(delta, -ds_max, ds_max);
        return;
    }

    // The reference speed `v` is the anchor speed with a floor, which keeps `a` above zero. With
    // `a` at zero, the shift of a squad that stands still could never change.
    float v_min = MIN_REFERENCE_SPEED_FRACTION * v_top;
    float v = fmaxf(squad->anchor_speed, v_min);

    // On a straight route, a squad member on its lane has a sideways acceleration equal to the
    // rate of change of the shift rate. At the speed `v`, the limit `a` is a turn of radius `R`.
    float R = squad->full_speed_min_radius;
    float a = v * v / R;                    // shift rate change limit per second
    float d = fabsf(delta);                 // magnitude of the remaining shift
    float r1 = MAX_SHIFT_SLOPE * v;         // shift rate limit from `MAX_SHIFT_SLOPE`
    float r2 = sqrtf(2 * a * d);            // limit that can still reach zero within `d`
    float r_max = fminf(r1, r2);            // shift rate limit
    float dr_max = a * TICK_SECONDS;        // shift rate change limit per tick

    // The shift rate changes once per tick, so the step of one tick could carry the shift past the
    // target shift. The shift then takes the target value, and the shift rate is set to zero.
    float r_target = copysignf(r_max, delta);           // target shift rate
    float dr = r_target - squad->shift_rate;            // shift rate change
    squad->shift_rate += clamp_float(dr, -dr_max, dr_max);
    float ds = squad->shift_rate * TICK_SECONDS;        // shift change in this tick
    bool is_toward_target = ds * delta > 0;
    if (is_toward_target && fabsf(ds) >= d) {
        squad->shift = squad->shift_target;
        squad->shift_rate = 0;
        return;
    }
    squad->shift += ds;
}


// `get_transition_length` returns the length of a half cosine wave of the height `height` that
// keeps both a mean slope of at most `MAX_SHIFT_SLOPE` and an end curvature of at most `1 / R`.
static float get_transition_length(float height, float R) {
    // On a straight route, a half cosine of height `A` over the length `L` has the curvature
    // `A / 2 * (PI / L)^2` at its ends. `L2` makes that curvature `1 / R`.
    float A = height;
    float L1 = A / MAX_SHIFT_SLOPE;          // length for a mean slope of `MAX_SHIFT_SLOPE`
    float L2 = PI * sqrtf(A * R / 2);        // length for an end curvature of `1 / R`
    return fmaxf(L1, L2);
}


// `get_lane_shift` computes the sideways offset of the lanes of a squad at the route position
// `distance`: the full shift inside the section, and half a cosine wave from the full shift down
// to zero over the transition on each side, so that the lanes have no kink at either end.
//
//   offset                .----------------.
//                      .'                    '.
//   0  _____________.'                          '._____________  route position
//                   |<-L->|                |<-L->|
//                         ^ section start  ^ section end
//
float get_lane_shift(const Squad *squad, float distance) {
    // With a zero shift, `L` would be zero, and `x / L` would give NaN.
    if (squad->shift == 0)
        return 0;
    float A = fabsf(squad->shift);                                           // transition height
    float L = get_transition_length(A, squad->full_speed_min_radius);        // transition length

    // The fraction of the full shift falls from 1 at the section to 0 at the outer end of the
    // transition.
    float x = 0;                                // distance outside the section
    if (distance < squad->shift_section_start)
        x = squad->shift_section_start - distance;
    if (distance > squad->shift_section_end)
        x = distance - squad->shift_section_end;
    float u = clamp_float(x / L, 0, 1);         // position in the transition, 0 to 1
    float f = 0.5f * (1 + cosf(PI * u));        // fraction of the full shift
    return squad->shift * f;
}


// `keep_replanned_section` keeps the new section of a squad after a top speed change moved the
// head-on meeting point (`--speedreplan`). Once the formation front has reached the transition
// before the previous section, the new section also covers the previous one. Otherwise, the lanes
// under the squad members could move back toward the other squad.
static void keep_replanned_section(Squad *squad, float previous_target_shift,
                                   float previous_section_start, float previous_section_end) {
    float A = fabsf(previous_target_shift);                                  // transition height
    float L = get_transition_length(A, squad->full_speed_min_radius);        // transition length
    float front = squad->anchor_distance + measure_shape(squad).front_meters;
    if (front < previous_section_start - L)
        return;
    squad->shift_section_start = fminf(squad->shift_section_start, previous_section_start);
    squad->shift_section_end = fmaxf(squad->shift_section_end, previous_section_end);
}


// `keep_shift_target_steady` combines the target shift and the section of a squad in this tick
// with those of the previous tick, so that they stay steady until the anchor passes the previous
// section end. Without it, they would jump between ticks, and the lanes would bend sharply. A
// head-on event that `is_replanning` after a top speed change takes its new section instead.
void keep_shift_target_steady(Squad *squad, float previous_target_shift,
                              float previous_section_start, float previous_section_end,
                              bool is_replanning) {
    // The new values stay when there was no previous target shift, when the anchor is past the
    // previous section end, or when an event gives a target shift of zero or on the other side.
    bool is_anchor_before_previous_end = squad->anchor_distance < previous_section_end;
    if (previous_target_shift == 0 || !is_anchor_before_previous_end)
        return;
    float t_new = squad->shift_target;        // new target shift
    ShiftEvent event = shift_events[squad->squad_index];
    bool is_in_event = event != SHIFT_EVENT_NONE;
    bool is_same_side = t_new * previous_target_shift > 0;
    if (is_in_event && !is_same_side)
        return;

    // A pass event keeps its new target shift, because a larger previous one could move the squad
    // deep into hills when the side room decreases. Only the section grows to cover the old one.
    if (event == SHIFT_EVENT_PASS) {
        squad->shift_section_start = fminf(squad->shift_section_start, previous_section_start);
        squad->shift_section_end = fmaxf(squad->shift_section_end, previous_section_end);
        return;
    }

    // Otherwise, the previous section stays with the target shift of the larger magnitude, also in
    // ticks in which the event detection finds no event for the squad.
    if (fabsf(previous_target_shift) > fabsf(t_new))
        squad->shift_target = previous_target_shift;
    if (is_replanning) {
        keep_replanned_section(squad, previous_target_shift, previous_section_start,
                               previous_section_end);
        return;
    }
    squad->shift_section_start = previous_section_start;
    squad->shift_section_end = previous_section_end;
}
