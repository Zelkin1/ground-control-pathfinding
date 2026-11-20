// Coordination between squads in the coordination mode (`--avoid coord`). Each tick,
// `coordinate_squads` checks every pair of moving squads for these events, in this order:
// - a head-on event (`pass_head_on`, `--keepright`): the lanes of both squads shift apart near the
//   head-on meeting point, or, with too little side room, one squad waits at a turnout
//   (`take_turns`, `--turns`);
// - a pass event or a follow (`pass_or_follow` in passing.c, `--pass`);
// - a crossing (`give_way_at_crossing`, `--crossing`): one squad gives way with a speed cap.
// A pair with a head-on event skips the other checks, because the routes of oncoming squads in
// winding valleys cross at bends, and a pair that drives the same way skips the crossing check.
//
// The checks read the routes at route positions, `CROSSING_POSITION_STEP_METERS` apart. The time
// estimates assume that each squad drives at its top speed, the top speed of its slowest squad
// member. `coordinate_squads` runs in every formation mode, but only the coordination mode checks
// the pairs.
#include "sim.h"
#include "formation.h"

#define CROSSING_POSITION_STEP_METERS 6.0f                 // spacing of the route positions
#define CROSSING_HORIZON_METERS 250.0f                     // reach past the anchor
#define MIN_CROSSING_SINE 0.5f                             // smallest crossing angle, 30 degrees
#define CROSSING_MARGIN_METERS 2.0f                        // crossing zone margin along each route
#define CROSSING_TIME_MARGIN_SECONDS 0.5f                  // gap between zone exit and entry
#define LATE_GIVE_WAY_SPEED_FRACTION 0.5f                  // cap share below which a squad slows
#define MIN_PREDICTED_SPEED 0.5f                           // slowest speed in time estimates
#define CAP_WAIT_SPEED 0.2f                                // slower caps count as waiting
#define GIVE_UP_SECONDS 15.0f                              // longest wait before giving up
#define IGNORE_SECONDS 5.0f                                // coordination pause after giving up
#define HEAD_ON_MAX_COSINE -0.7f                           // head-on direction limit, 45 degrees
#define MEETING_POINT_REACH_METERS 40.0f                   // side room reach at the meeting point
#define HEAD_ON_EXTRA_GAP_METERS 4.0f                      // extra body gap in a head-on event
#define SECTION_START_TO_MEETING_POINT_METERS 20.0f        // section start before meeting point
#define MEETING_POINT_TO_SECTION_END_METERS 10.0f          // section end past the meeting point
#define TURNOUT_LENGTH_METERS 24.0f                        // route length that a turnout needs
#define NARROW_LEAD_SECONDS 2.0f                           // narrowing time before the contact
#define SPEED_CHANGE_FRACTION 0.01f                        // top speed change that re-plans


// A crossing of the routes of two squads: its route position on each route, and the sine of the
// angle between the two routes there.
typedef struct {
    float first_distance;
    float second_distance;
    float sine;
} Crossing;


// When a squad enters and leaves its crossing zone at its top speed, and the distance from its
// front end to the zone entry.
typedef struct {
    float entry_seconds;
    float exit_seconds;
    float entry_distance;
} CrossingZoneTimes;


// One squad of a head-on event: its shape, its route positions, and the index of the head-on
// meeting point among them.
typedef struct {
    Squad *squad;
    const FormationShape *shape;
    const RoutePositions *positions;
    int meeting_point_index;
} HeadOnSide;


// `is_coordinated` tells whether a squad takes part in the coordination: only moving squads do.
static bool is_coordinated(const Squad *squad) {
    return squad->state == SQUAD_MOVING;
}


// `measure_shape` measures the formation of a squad for the coordination: how far the bodies reach
// ahead of the anchor and to the side of the route, the route position of the rear end of the
// formation, and the top speed of the squad with and without speed effects. When a head-on plans
// square or column formation (`--narrowpass`), the shape is that of the planned formation.
FormationShape measure_shape(const Squad *squad) {
    FormationShape shape = {
        .front_meters = 0,
        .half_width = 0,
        .rear_progress = LARGE_VALUE,
        .squad_top_speed = LARGE_VALUE,
        .squad_base_top_speed = LARGE_VALUE,
        .own_half_width = 0,
    };
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        float r = unit->unit_type->radius;
        V2 slot = get_planned_slot(squad, unit);
        float front = slot.y + r;                                      // body front, anchor frame
        float side = fabsf(slot.x) + r;                                // body side, off the route
        float own_side = fabsf(unit->wide_slot_position.x) + r;        // same, own formation
        float rear = unit->route_progress - r;                         // body rear, route position
        shape.front_meters = fmaxf(shape.front_meters, front);
        shape.half_width = fmaxf(shape.half_width, side);
        shape.own_half_width = fmaxf(shape.own_half_width, own_side);
        shape.rear_progress = fminf(shape.rear_progress, rear);
        shape.squad_top_speed = fminf(shape.squad_top_speed, unit->top_speed);
        shape.squad_base_top_speed = fminf(shape.squad_base_top_speed, unit->base_top_speed);
    }
    return shape;
}


// `is_fraction` tells whether `value` lies between 0 and 1.
static bool is_fraction(float value) {
    return value >= 0 && value <= 1;
}


// `get_crossing_search_start` returns the route position where the crossing search of the squad
// of `shape` starts: behind the rear end of its formation by the longest crossing zone reach, so
// that a crossing that the formation still covers counts. The reach counts half widths of at least
// `LANE_MAX_DISTANCE_METERS`, or the larger half width of the two squads.
static float get_crossing_search_start(const FormationShape *shape,
                                       const FormationShape *other_shape) {
    float wider_half_width = fmaxf(shape->half_width, other_shape->half_width);
    float half_width = fmaxf(LANE_MAX_DISTANCE_METERS, wider_half_width);
    float max_reach = 2 * half_width / MIN_CROSSING_SINE;
    float zone_reach = max_reach + CROSSING_MARGIN_METERS;
    return fmaxf(0, shape->rear_progress - zone_reach);
}


// `find_crossing` finds the first crossing along the route of `first` with the route of `second`,
// at an angle of at least 30 degrees, between the search start and `CROSSING_HORIZON_METERS`
// past the anchor on each route. It returns false when the routes do not cross there.
static bool find_crossing(const Squad *first, const FormationShape *first_shape,
                          const Squad *second, const FormationShape *second_shape,
                          Crossing *crossing) {
    // The route positions of `second` end exactly at the end of its search range.
    enum { MAX_POSITIONS = 128 };
    V2 points[MAX_POSITIONS];
    float distances[MAX_POSITIONS];
    float start = get_crossing_search_start(second_shape, first_shape);
    float horizon = second->anchor_distance + CROSSING_HORIZON_METERS;
    float end = fminf(second->route_length, horizon);
    int count = 0;          // number of route positions of `second`
    float d = start;        // route position on the route of `second`
    while (count < MAX_POSITIONS) {
        float d_clamped = fminf(d, end);
        V2 unused;
        points[count] = get_route_point(second, d_clamped, &unused);
        distances[count] = d_clamped;
        count++;
        if (d_clamped >= end)
            break;
        d += CROSSING_POSITION_STEP_METERS;
    }

    // The route of `first` runs in pieces of `CROSSING_POSITION_STEP_METERS`, each tested against
    // every segment between two route positions of `second`.
    float first_start = get_crossing_search_start(first_shape, second_shape);
    float first_horizon = first->anchor_distance + CROSSING_HORIZON_METERS;
    float first_end = fminf(first->route_length, first_horizon);
    V2 unused;
    V2 piece_start = get_route_point(first, first_start, &unused);
    for (float piece_d = first_start; piece_d < first_end;
         piece_d += CROSSING_POSITION_STEP_METERS) {
        float piece_end_d = fminf(piece_d + CROSSING_POSITION_STEP_METERS, first_end);
        V2 piece_end = get_route_point(first, piece_end_d, &unused);
        V2 piece = sub(piece_end, piece_start);
        float piece_length = len(piece);
        for (int position_index = 0; position_index + 1 < count; position_index++) {
            // `det` is the product of the two lengths and the sine of the angle between them.
            int next_index = position_index + 1;
            V2 other_start = points[position_index];
            V2 other = sub(points[next_index], other_start);
            float det = det2(piece, other);
            float det_abs = fabsf(det);
            float lengths = piece_length * len(other);        // product of the two lengths
            if (lengths <= 0 || det_abs < MIN_CROSSING_SINE * lengths)
                continue;

            // `t` and `u` are the fractions of the piece and of the other segment at the crossing.
            V2 to_other = sub(other_start, piece_start);
            float t = det2(to_other, other) / det;
            float u = det2(to_other, piece) / det;
            if (!is_fraction(t) || !is_fraction(u))
                continue;
            float other_start_d = distances[position_index];
            float other_length = distances[next_index] - other_start_d;
            float piece_span = piece_end_d - piece_d;
            crossing->first_distance = piece_d + t * piece_span;
            crossing->second_distance = other_start_d + u * other_length;
            crossing->sine = det_abs / lengths;
            return true;
        }
        piece_start = piece_end;
    }
    return false;
}


// `get_crossing_zone_times` estimates when a squad enters and leaves its crossing zone, which
// reaches `crossing_zone_reach` before and after the crossing along its route. The front end of
// the formation enters the zone, and its rear end leaves it.
static CrossingZoneTimes get_crossing_zone_times(const Squad *squad, const FormationShape *shape,
                                                 float crossing_distance,
                                                 float crossing_zone_reach) {
    float v = fmaxf(MIN_PREDICTED_SPEED, shape->squad_top_speed);        // predicted speed
    float front = squad->anchor_distance + shape->front_meters;
    float entry = crossing_distance - crossing_zone_reach - front;
    float exit = crossing_distance + crossing_zone_reach - shape->rear_progress;
    return (CrossingZoneTimes){
        .entry_seconds = entry / v,
        .exit_seconds = exit / v,
        .entry_distance = entry,
    };
}


// `can_give_way_later` tells whether a squad can still drive on in spite of `speed_cap`, because
// the cap is above `LATE_GIVE_WAY_SPEED_FRACTION` of the top speed of the squad. The squad then
// keeps full speed and slows later and harder, with the same entry time.
static bool can_give_way_later(float speed_cap, float squad_top_speed) {
    float late_speed = LATE_GIVE_WAY_SPEED_FRACTION * squad_top_speed;
    return speed_cap > late_speed;
}


// `get_give_way_delay` returns how long the squad of `giving_times` would wait at its crossing
// zone if it gave way: from its own zone entry until the other squad has left its zone, plus the
// margin, or 0 when it would enter only after that anyway.
static float get_give_way_delay(const CrossingZoneTimes *giving_times,
                                const CrossingZoneTimes *other_times) {
    float clear_seconds = other_times->exit_seconds + CROSSING_TIME_MARGIN_SECONDS;
    return fmaxf(0, clear_seconds - giving_times->entry_seconds);
}


// `give_way_at_crossing` makes one of two squads give way where their routes cross
// (`--crossing`). The speed cap brings the front end of that squad to its crossing zone when the
// rear end of the other squad has left its zone, plus `CROSSING_TIME_MARGIN_SECONDS`.
static void give_way_at_crossing(Squad *first, Squad *second) {
    // Along the first route, a body of half width `w1` overlaps the strip of half width `w2`
    // around the second route within `(w2 + w1 * cos) / sin` of the crossing.
    FormationShape shape_1 = measure_shape(first);
    FormationShape shape_2 = measure_shape(second);
    Crossing crossing;
    if (!find_crossing(first, &shape_1, second, &shape_2, &crossing))
        return;
    float cosine = sqrtf(1 - crossing.sine * crossing.sine);
    float slant_1 = shape_1.half_width * cosine;
    float slant_2 = shape_2.half_width * cosine;
    float reach_1 = (shape_2.half_width + slant_1) / crossing.sine;
    float reach_2 = (shape_1.half_width + slant_2) / crossing.sine;
    float zone_1 = reach_1 + CROSSING_MARGIN_METERS;
    float zone_2 = reach_2 + CROSSING_MARGIN_METERS;
    CrossingZoneTimes times_1 = get_crossing_zone_times(first, &shape_1, crossing.first_distance,
                                                        zone_1);
    CrossingZoneTimes times_2 = get_crossing_zone_times(second, &shape_2,
                                                        crossing.second_distance, zone_2);

    // No squad gives way when one has already left its zone, or when one enters its zone only
    // after the other has left its zone, plus the margin.
    bool has_any_left = times_1.exit_seconds <= 0 || times_2.exit_seconds <= 0;
    if (has_any_left)
        return;
    float margin = CROSSING_TIME_MARGIN_SECONDS;
    bool is_first_clear = times_1.entry_seconds > times_2.exit_seconds + margin;
    bool is_second_clear = times_2.entry_seconds > times_1.exit_seconds + margin;
    if (is_first_clear || is_second_clear)
        return;

    // The squad that would enter later gives way. With `--leastdelay`, the squad whose wait would
    // be shorter gives way. Otherwise, with `--speedfirst`, the slower squad gives way, unless its
    // front end is already in its zone; a slow squad close to its zone could then wait for a fast
    // squad that is still far away.
    bool is_second_giving_way = times_2.entry_seconds >= times_1.entry_seconds;
    bool has_slower_squad = shape_1.squad_top_speed != shape_2.squad_top_speed;
    if (is_least_delay_on) {
        float delay_1 = get_give_way_delay(&times_1, &times_2);
        float delay_2 = get_give_way_delay(&times_2, &times_1);
        is_second_giving_way = delay_2 <= delay_1;
    } else if (is_speed_priority_on && has_slower_squad) {
        bool is_second_slower = shape_2.squad_top_speed < shape_1.squad_top_speed;
        CrossingZoneTimes slower_times = is_second_slower ? times_2 : times_1;
        if (slower_times.entry_distance > 0)
            is_second_giving_way = is_second_slower;
    }

    // A squad whose front end is already in its zone keeps driving.
    Squad *giving = is_second_giving_way ? second : first;
    CrossingZoneTimes giving_times = is_second_giving_way ? times_2 : times_1;
    CrossingZoneTimes other_times = is_second_giving_way ? times_1 : times_2;
    if (giving_times.entry_distance <= 0)
        return;
    float wait = other_times.exit_seconds + margin;        // time until the zone entry
    float cap = giving_times.entry_distance / wait;

    // A late give way keeps full speed while the cap is high.
    const FormationShape *giving_shape = &shape_1;
    if (is_second_giving_way)
        giving_shape = &shape_2;
    if (can_give_way_later(cap, giving_shape->squad_top_speed))
        return;
    giving->speed_cap = fminf(giving->speed_cap, cap);
}


// `fill_route_positions` fills `positions` with the route positions of a squad from `start` up to
// `end`, `CROSSING_POSITION_STEP_METERS` apart and at most `ROUTE_POSITION_COUNT` of them, with
// the route point and the route direction at each.
void fill_route_positions(const Squad *squad, float start, float end, RoutePositions *positions) {
    int max_count = ROUTE_POSITION_COUNT;
    positions->count = 0;
    for (float d = start; d <= end; d += CROSSING_POSITION_STEP_METERS) {
        if (positions->count == max_count)
            break;
        int position_index = positions->count;
        V2 *direction = &positions->directions[position_index];
        positions->points[position_index] = get_route_point(squad, d, direction);
        positions->distances[position_index] = d;
        positions->count++;
    }
}


// `fill_route_positions_ahead` fills the route positions of a squad for the head-on check, from
// the rear end of the formation up to `CROSSING_HORIZON_METERS` past the anchor.
static void fill_route_positions_ahead(const Squad *squad, const FormationShape *shape,
                                       RoutePositions *positions) {
    float start = fmaxf(0, shape->rear_progress);
    float horizon = squad->anchor_distance + CROSSING_HORIZON_METERS;
    float end = fminf(squad->route_length, horizon);
    fill_route_positions(squad, start, end, positions);
}


// `has_turnout_side_room` tells whether a squad has at least `needed_side_room` toward `side` at
// the route positions of a turnout that ends at `last_index`. The turnout reaches back by
// `TURNOUT_LENGTH_METERS`, so that the whole formation fits.
static bool has_turnout_side_room(const Squad *squad, const RoutePositions *positions,
                                  int last_index, float side, float needed_side_room) {
    int n = (int)roundf(TURNOUT_LENGTH_METERS / CROSSING_POSITION_STEP_METERS);        // positions
    int first_index = last_index - n;
    if (first_index < 0)
        first_index = 0;
    for (int position_index = first_index; position_index <= last_index; position_index++) {
        float d = positions->distances[position_index];
        if (measure_side_room(squad, d, side) < needed_side_room)
            return false;
    }
    return true;
}


// `find_turnout` finds the turnout of a squad closest to the head-on meeting point, between the
// front end of the formation and the meeting point. It returns the distance from the front end to
// the turnout end, or -1 when no turnout lies ahead of the front end.
static float find_turnout(const Squad *squad, const FormationShape *shape,
                          const RoutePositions *positions, int meeting_point_index, float side,
                          float needed_side_room, int *turnout_index) {
    float front = squad->anchor_distance + shape->front_meters;        // route position, front end
    for (int position_index = meeting_point_index; position_index >= 0; position_index--) {
        float d = positions->distances[position_index];
        if (d <= front)
            return -1;
        if (has_turnout_side_room(squad, positions, position_index, side, needed_side_room)) {
            *turnout_index = position_index;
            return d - front;
        }
    }
    return -1;
}


// `find_closest_position_index` returns the index of the route position closest to `point`, or 0
// when `positions` is empty.
int find_closest_position_index(const RoutePositions *positions, V2 point) {
    int best_index = 0;
    float best_squared = LARGE_VALUE;        // smallest squared distance
    for (int position_index = 0; position_index < positions->count; position_index++) {
        V2 to_position = sub(positions->points[position_index], point);
        float d_squared = length_squared(to_position);
        if (d_squared < best_squared) {
            best_squared = d_squared;
            best_index = position_index;
        }
    }
    return best_index;
}


// `take_turns` makes one squad of a head-on event that is too narrow wait at a turnout, until the
// rear end of the other squad has gone past it (`--turns`). The turnout needs the whole missing
// gap as side room toward `side`.
static void take_turns(const HeadOnSide *first, const HeadOnSide *second, float side,
                       float missing_gap) {
    int turnout_1 = 0;        // turnout index of `first`
    int turnout_2 = 0;        // turnout index of `second`
    float d_1 = find_turnout(first->squad, first->shape, first->positions,
                             first->meeting_point_index, side, missing_gap, &turnout_1);
    float d_2 = find_turnout(second->squad, second->shape, second->positions,
                             second->meeting_point_index, side, missing_gap, &turnout_2);
    if (d_1 < 0 && d_2 < 0)
        return;

    // The squad with the farther turnout waits, or with `--speedfirst`, the slower squad when both
    // can wait. A squad that gave way to the other one keeps waiting while it has a turnout.
    bool can_first_wait = d_1 >= 0;
    bool can_second_wait = d_2 >= 0;
    bool is_first_waiting = d_1 >= d_2;
    float v_1 = first->shape->squad_top_speed;
    float v_2 = second->shape->squad_top_speed;
    bool can_both_wait = can_first_wait && can_second_wait;
    if (is_speed_priority_on && can_both_wait && v_1 != v_2)
        is_first_waiting = v_1 < v_2;
    bool was_first_waiting = first->squad->giving_way_to_index == second->squad->squad_index;
    bool was_second_waiting = second->squad->giving_way_to_index == first->squad->squad_index;
    if (was_first_waiting && can_first_wait)
        is_first_waiting = true;
    if (was_second_waiting && can_second_wait)
        is_first_waiting = false;
    const HeadOnSide *waiting = is_first_waiting ? first : second;
    const HeadOnSide *other = is_first_waiting ? second : first;
    float d_turnout = is_first_waiting ? d_1 : d_2;
    int turnout_index = is_first_waiting ? turnout_1 : turnout_2;

    // The other squad passes the turnout at its route position closest to the turnout point. The
    // cap brings the waiting squad there when the rear end of the other squad has gone past.
    V2 turnout_point = waiting->positions->points[turnout_index];
    int other_index = find_closest_position_index(other->positions, turnout_point);
    float d_other = other->positions->distances[other_index];
    float v_other = fmaxf(MIN_PREDICTED_SPEED, other->shape->squad_top_speed);
    float rear_reach = d_other + CROSSING_MARGIN_METERS - other->shape->rear_progress;
    float t_past = fmaxf(CROSSING_TIME_MARGIN_SECONDS, rear_reach / v_other);        // pass time
    float cap = d_turnout / t_past;
    Squad *waiting_squad = waiting->squad;
    waiting_squad->speed_cap = fminf(waiting_squad->speed_cap, cap);
    waiting_squad->giving_way_to_index = other->squad->squad_index;
    waiting_squad->is_give_way_kept = true;
}


// `get_arrival_seconds` returns the time until the anchor of a squad reaches the route position
// `distance` at the top speed of the squad.
static float get_arrival_seconds(const Squad *squad, const FormationShape *shape, float distance) {
    float v = fmaxf(MIN_PREDICTED_SPEED, shape->squad_top_speed);        // predicted speed
    return (distance - squad->anchor_distance) / v;
}


// `is_waiting_for` tells whether `squad` gives way to `other` in the current tick.
static bool is_waiting_for(const Squad *squad, const Squad *other) {
    bool is_giving_way_to_other = squad->giving_way_to_index == other->squad_index;
    return squad->is_give_way_kept && is_giving_way_to_other;
}


// `measure_meeting_point_side_room` measures the smallest side room of a squad toward `side` over
// its route positions within `MEETING_POINT_REACH_METERS` of the head-on meeting point.
static float measure_meeting_point_side_room(const HeadOnSide *head_on_side, float side) {
    const RoutePositions *positions = head_on_side->positions;
    float d_meet = positions->distances[head_on_side->meeting_point_index];
    float start = d_meet;
    float end = d_meet;
    for (int position_index = 0; position_index < positions->count; position_index++) {
        float d = positions->distances[position_index];
        if (fabsf(d - d_meet) > MEETING_POINT_REACH_METERS)
            continue;
        start = fminf(start, d);
        end = fmaxf(end, d);
    }
    float step = CROSSING_POSITION_STEP_METERS;
    return measure_min_side_room(head_on_side->squad, start, end, step, side);
}


// `get_head_on_target_shift` returns the target shift of one squad of a head-on event before the
// split of the missing gap: its section around the head-on meeting point and its side room there.
// With `--timedshift`, the section distances grow with the speed effects, so that the section
// starts and ends as long before and after the meeting as at the base top speed.
static TargetShift get_head_on_target_shift(const HeadOnSide *head_on_side, float side) {
    const RoutePositions *positions = head_on_side->positions;
    float d_meet = positions->distances[head_on_side->meeting_point_index];
    const FormationShape *shape = head_on_side->shape;
    float scale = 1;        // section distance factor
    if (is_timed_shift_on)
        scale = shape->squad_top_speed / shape->squad_base_top_speed;
    float before = SECTION_START_TO_MEETING_POINT_METERS * scale;
    float after = MEETING_POINT_TO_SECTION_END_METERS * scale;
    return (TargetShift){
        .squad = head_on_side->squad,
        .side = side,
        .side_room = measure_meeting_point_side_room(head_on_side, side),
        .section_start = d_meet - before,
        .section_end = d_meet + after,
        .magnitude = 0,
    };
}


// `plan_narrow_formations` plans the line formations of a head-on event in a narrow formation
// before the shifts are planned (`--narrowpass`): square formation, or column formation when the
// side rooms in square formation miss the gap. `shape_1` and `shape_2` are the shapes of `side_1`
// and `side_2`, which it measures again. It returns the needed gap of the planned formations.
static float plan_narrow_formations(const HeadOnSide *side_1, const HeadOnSide *side_2,
                                    FormationShape *shape_1, FormationShape *shape_2, float side,
                                    float lateral, float body_gap) {
    Squad *first = side_1->squad;
    Squad *second = side_2->squad;
    first->is_narrow_planned = first->is_narrow_planned || can_narrow(first);
    second->is_narrow_planned = second->is_narrow_planned || can_narrow(second);
    *shape_1 = measure_shape(first);
    *shape_2 = measure_shape(second);
    float needed_gap = get_needed_gap(shape_1, shape_2, body_gap);
    if (!first->is_narrow_planned && !second->is_narrow_planned)
        return needed_gap;

    // The side rooms count the lanes of the planned formation. A squad keeps planning column
    // formation until the head-on event ends (`update_narrowing`). Otherwise, it could switch
    // back and forth.
    float room_1 = measure_meeting_point_side_room(side_1, side);
    float room_2 = measure_meeting_point_side_room(side_2, side);
    if (room_1 + room_2 >= needed_gap - fabsf(lateral))
        return needed_gap;
    first->is_column_planned = first->is_narrow_planned;
    second->is_column_planned = second->is_narrow_planned;
    *shape_1 = measure_shape(first);
    *shape_2 = measure_shape(second);
    return get_needed_gap(shape_1, shape_2, body_gap);
}


// `find_end_member` returns the squad member of a squad that lies farthest ahead along its route
// with `is_front`, or farthest back otherwise.
static const Unit *find_end_member(const Squad *squad, bool is_front) {
    const Unit *end_member = &units[squad->first_unit_index];
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        bool is_ahead = unit->route_progress > end_member->route_progress;
        bool is_behind = unit->route_progress < end_member->route_progress;
        if (is_front ? is_ahead : is_behind)
            end_member = unit;
    }
    return end_member;
}


// `measure_time_to_contact` returns the time until the front members of two squads of a head-on
// event meet along the route of `first`, at their current anchor speeds.
static float measure_time_to_contact(const Squad *first, const Squad *second) {
    const Unit *front_1 = find_end_member(first, true);
    const Unit *front_2 = find_end_member(second, true);
    // `d_1` and `d_2` are the route positions of the two front members on the route of `first`.
    float d_1 = front_1->route_progress;
    float to = first->anchor_distance + CROSSING_HORIZON_METERS;
    float d_2 = project_onto_route(first, front_2->position, d_1, to);
    float radii = front_1->unit_type->radius + front_2->unit_type->radius;
    float closing_speed = first->anchor_speed + second->anchor_speed;
    return (d_2 - d_1 - radii) / fmaxf(MIN_PREDICTED_SPEED, closing_speed);
}


// `have_passed` tells whether the formations of two squads of a head-on event have passed each
// other: the rear member of `second`, projected onto the route of `first`, lies behind the rear
// member of `first`. From then on, the formations no longer overlap along the route.
static bool have_passed(const Squad *first, const Squad *second) {
    float rear = find_end_member(first, false)->route_progress;        // rear member position
    float from = fmaxf(0, rear - MEETING_POINT_REACH_METERS);
    float to = first->anchor_distance + CROSSING_HORIZON_METERS;
    V2 other_rear = find_end_member(second, false)->position;
    return project_onto_route(first, other_rear, from, to) < rear;
}


// `narrow_before_meeting` puts a squad of a head-on event into its planned narrow formation once
// its front member is `NARROW_LEAD_SECONDS` from the front member of `other` (`--narrowpass`). The
// squad keeps its own formation until then, and a narrowed squad follows a later column plan.
static void narrow_before_meeting(Squad *squad, const Squad *other) {
    if (!squad->is_narrow_planned)
        return;
    bool is_close = measure_time_to_contact(squad, other) <= NARROW_LEAD_SECONDS;
    if (is_close || squad->is_narrowed)
        narrow_formation(squad);
}


// `pass_head_on` detects a head-on event between two squads and plans their target shifts
// (`--keepright`). It returns true for a head-on event, also when the squads do not shift.
static bool pass_head_on(Squad *first, Squad *second) {
    // Squads in a head-on event keep `HEAD_ON_EXTRA_GAP_METERS` more between their bodies.
    FormationShape shape_1 = measure_shape(first);
    FormationShape shape_2 = measure_shape(second);
    static RoutePositions positions_1;
    static RoutePositions positions_2;
    fill_route_positions_ahead(first, &shape_1, &positions_1);
    fill_route_positions_ahead(second, &shape_2, &positions_2);
    float body_gap = BODY_GAP_METERS + HEAD_ON_EXTRA_GAP_METERS;
    float needed_gap = get_needed_gap(&shape_1, &shape_2, body_gap);
    float own_gap = shape_1.own_half_width + shape_2.own_half_width + body_gap;

    // The head-on meeting point is the pair of route positions with opposite directions, closer
    // than the needed gap of the own formations, where the anchors would arrive at the most
    // similar times. A planned narrow formation would otherwise end the event that planned it.
    float best_gap = LARGE_VALUE;        // smallest difference of the arrival times
    float lateral = 0;                   // sideways offset of the second route there
    int meet_1 = -1;                     // meeting point index of `first`
    int meet_2 = -1;                     // meeting point index of `second`
    for (int first_index = 0; first_index < positions_1.count; first_index++) {
        V2 point = positions_1.points[first_index];
        V2 direction = positions_1.directions[first_index];
        float d_1 = positions_1.distances[first_index];
        float t_1 = get_arrival_seconds(first, &shape_1, d_1);
        for (int second_index = 0; second_index < positions_2.count; second_index++) {
            V2 other_direction = positions_2.directions[second_index];
            if (dot(direction, other_direction) > HEAD_ON_MAX_COSINE)
                continue;
            V2 to_other = sub(positions_2.points[second_index], point);
            if (len(to_other) >= own_gap)
                continue;
            float d_2 = positions_2.distances[second_index];
            float t_2 = get_arrival_seconds(second, &shape_2, d_2);
            float time_gap = fabsf(t_1 - t_2);
            if (time_gap >= best_gap)
                continue;
            best_gap = time_gap;
            lateral = dot(to_other, get_right(direction));
            meet_1 = first_index;
            meet_2 = second_index;
        }
    }
    if (meet_1 < 0)
        return false;

    // `lateral` is positive to the right of the first route, as seen facing along it. Both squads
    // shift to the same side of their own routes, which moves them apart.
    flag_shift_event(first, SHIFT_EVENT_HEAD_ON);
    flag_shift_event(second, SHIFT_EVENT_HEAD_ON);
    float side = choose_shift_side(lateral);
    HeadOnSide side_1 = {
        .squad = first,
        .shape = &shape_1,
        .positions = &positions_1,
        .meeting_point_index = meet_1,
    };
    HeadOnSide side_2 = {
        .squad = second,
        .shape = &shape_2,
        .positions = &positions_2,
        .meeting_point_index = meet_2,
    };
    // With `--narrowpass`, line formations plan a narrow formation for the head-on, take it just
    // before the meeting, and take their own formation again as soon as they have passed.
    bool is_narrowing = is_narrow_pass_on && !have_passed(first, second);
    if (is_narrow_pass_on && !is_narrowing) {
        widen_formation(first);
        widen_formation(second);
    }
    if (is_narrowing) {
        needed_gap = plan_narrow_formations(&side_1, &side_2, &shape_1, &shape_2, side, lateral,
                                            body_gap);
        narrow_before_meeting(first, second);
        narrow_before_meeting(second, first);
    }
    float missing_gap = needed_gap - fabsf(lateral);
    TargetShift target_1 = get_head_on_target_shift(&side_1, side);
    TargetShift target_2 = get_head_on_target_shift(&side_2, side);
    float room_sum = target_1.side_room + target_2.side_room;
    if (room_sum >= missing_gap) {
        split_missing_gap_evenly(missing_gap, &target_1, &target_2);
        record_target_shifts(&target_1, &target_2, SHIFT_EVENT_HEAD_ON);
        return true;
    }

    // With too little side room, one squad can wait at a turnout. With `--partialshift`, and when
    // neither squad waits, each squad shifts as far as its side room allows.
    if (is_turn_taking_on)
        take_turns(&side_1, &side_2, side, missing_gap);
    bool is_any_waiting = is_waiting_for(first, second) || is_waiting_for(second, first);
    if (is_partial_shift_on && !is_any_waiting) {
        split_missing_gap_in_order(missing_gap, &target_1, &target_2);
        record_target_shifts(&target_1, &target_2, SHIFT_EVENT_HEAD_ON);
    }
    return true;
}


// `update_give_up` measures how long the speed cap of a squad keeps its anchor nearly still. After
// `GIVE_UP_SECONDS`, the squad ignores the other squads for `IGNORE_SECONDS`, so that two squads
// cannot block each other forever.
static void update_give_up(Squad *squad) {
    if (squad->speed_cap < CAP_WAIT_SPEED)
        squad->speed_cap_wait_seconds += TICK_SECONDS;
    else
        squad->speed_cap_wait_seconds = 0;
    if (squad->speed_cap_wait_seconds > GIVE_UP_SECONDS) {
        squad->speed_cap_wait_seconds = 0;
        squad->ignore_coordination_seconds = IGNORE_SECONDS;
    }
}


// `update_narrowing` brings a squad back to its own formation as soon as it has no head-on event
// any more, which happens once the two formations have passed each other, or once it stops moving
// or reaches the end blend of its route (`--narrowpass`).
static void update_narrowing(Squad *squad, bool is_in_head_on) {
    bool is_on_end_blend = squad->route_length - squad->anchor_distance <= END_BLEND_METERS;
    if (!is_in_head_on || !is_coordinated(squad) || is_on_end_blend)
        widen_formation(squad);
}


// `update_top_speed_change` keeps the top speed of a squad for the next tick, and tells whether it
// changed by more than `SPEED_CHANGE_FRACTION` since the last tick, as when a speed effect starts
// or ends.
static bool update_top_speed_change(Squad *squad) {
    float v_top = measure_shape(squad).squad_top_speed;
    float v_last = squad->previous_top_speed;        // top speed in the last tick
    squad->previous_top_speed = v_top;
    return v_last > 0 && fabsf(v_top - v_last) > SPEED_CHANGE_FRACTION * v_last;
}


// `coordinate_squads` runs the coordination of all squads for one tick: it resets the speed caps
// and the target shifts, checks each pair of moving squads in the coordination mode, and then
// updates the shift of each moving squad.
void coordinate_squads(void) {
    // The target shifts and sections of the last tick go to `keep_shift_target_steady`. The pass
    // and give-way states last only while a check keeps them in this tick.
    float previous_targets[MAX_SQUADS];
    float previous_starts[MAX_SQUADS];
    float previous_ends[MAX_SQUADS];
    bool has_speed_changed[MAX_SQUADS];
    bool is_replanning[MAX_SQUADS];
    bool is_in_head_on[MAX_SQUADS];
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        Squad *squad = &squads[squad_index];
        has_speed_changed[squad_index] = update_top_speed_change(squad);
        is_replanning[squad_index] = false;
        is_in_head_on[squad_index] = false;
        squad->speed_cap = LARGE_VALUE;
        previous_targets[squad_index] = squad->shift_target;
        previous_starts[squad_index] = squad->shift_section_start;
        previous_ends[squad_index] = squad->shift_section_end;
        clear_shift_target_and_event(squad);
        squad->ignore_coordination_seconds -= TICK_SECONDS;
        squad->is_give_way_kept = false;
        squad->is_pass_kept = false;
        for (int other_index = 0; other_index < squad_count; other_index++) {
            squad->was_following[other_index] = squad->is_following[other_index];
            squad->is_following[other_index] = false;
        }
    }

    // A pair with a head-on event skips the other checks, and a pair that drives the same way
    // skips the crossing check. With `--speedreplan`, a top speed change of either squad of a
    // head-on event moves the head-on meeting point, so both squads take their new sections.
    bool is_coordinating = avoid_mode == AVOID_COORD;
    for (int first_index = 0; first_index < squad_count && is_coordinating; first_index++) {
        Squad *first = &squads[first_index];
        if (!is_coordinated(first))
            continue;
        for (int second_index = first_index + 1; second_index < squad_count; second_index++) {
            Squad *second = &squads[second_index];
            if (!is_coordinated(second) || !are_squads_coordinated(first, second))
                continue;
            bool is_head_on = false;
            if (is_keep_right_on)
                is_head_on = pass_head_on(first, second);
            is_in_head_on[first_index] = is_in_head_on[first_index] || is_head_on;
            is_in_head_on[second_index] = is_in_head_on[second_index] || is_head_on;
            bool is_pair_speed_changed = has_speed_changed[first_index];
            is_pair_speed_changed = is_pair_speed_changed || has_speed_changed[second_index];
            if (is_speed_replan_on && is_head_on && is_pair_speed_changed) {
                is_replanning[first_index] = true;
                is_replanning[second_index] = true;
            }
            bool is_one_way = false;
            if (is_pass_on && !is_head_on)
                is_one_way = pass_or_follow(first, second);
            if (is_crossing_give_way_on && !is_head_on && !is_one_way)
                give_way_at_crossing(first, second);
        }
    }

    // A squad that ignores the coordination drives without a speed cap.
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        Squad *squad = &squads[squad_index];
        keep_shift_target_steady(squad, previous_targets[squad_index],
                                 previous_starts[squad_index], previous_ends[squad_index],
                                 is_replanning[squad_index]);
        if (!squad->is_give_way_kept)
            squad->giving_way_to_index = NO_SQUAD;
        if (!squad->is_pass_kept)
            squad->passing_index = NO_SQUAD;
        update_give_up(squad);
        if (squad->ignore_coordination_seconds > 0)
            squad->speed_cap = LARGE_VALUE;
        if (is_coordinated(squad))
            update_shift(squad);
        if (is_narrow_pass_on)
            update_narrowing(squad, is_in_head_on[squad_index]);
    }
}
