// Pass events and following in the coordination mode (`--avoid coord`). When two squads drive the
// same way on one section of route, the squad behind, the rear squad, either passes the squad
// ahead, the front squad, or follows it. With `--pass` on, `coordinate_squads` (coordination.c)
// calls `pass_or_follow` for each pair of coordinated squads that are not in a head-on event.
//
// In a pass event, the lanes of the rear squad shift to one side and, unless `--keeproute` is on,
// the lanes of the front squad shift to the other side (see shift.c). Only the rear squad gets a
// speed cap. A rear squad that does not pass gets a follow cap, a speed cap that keeps it about
// one headway behind the front squad. The headway is the gap along the route between the bodies.
//
// A pass lasts over several ticks. `passing_index` of the rear squad holds the index of the front
// squad that it passes, and `is_pass_waiting` is true while the rear squad waits behind for the
// front squad to shift (`--formfirst`). A pass counts as started once the rear squad no longer
// waits. Unless this file sets `is_pass_kept` again in a tick, `coordinate_squads` clears
// `passing_index`.
//
// Most route positions in this file are on the route of the rear squad, with the anchor of the
// front squad projected onto that route. A constant offset converts route positions between the
// two routes, which assumes that the routes run side by side.
#include "sim.h"
#include "formation.h"

#define SAME_WAY_MIN_COSINE 0.85f                // same-way direction limit, about 32 degrees
#define PASS_MIN_SPEED_GAIN 1.0f                 // smallest top speed difference for a pass
#define PASS_LOOK_SECONDS 6.0f                   // longest catch-up time for a pass plan

#define PASS_BACK_METERS 120.0f                  // route positions start behind the rear end
#define PASS_HORIZON_METERS 450.0f               // route positions end past the anchor

#define PASS_END_MARGIN_METERS 20.0f             // section margin beyond each end of the pass
#define PASS_GAP_METERS 5.0f                     // body gap at the contact and at the pass end
#define PASS_SIDE_ROOM_STEP_METERS 2.0f          // spacing of the side room measurements
#define PASS_SQUEEZE_METERS 3.0f                 // largest pass plan shortfall for a new pass
#define PASS_SIDE_ROOM_MARGIN_METERS 4.0f        // side room kept free next to the lane clamps

#define FORMED_LANE_METERS 2.0f                  // formed: largest distance to the lane point
#define FORMED_FRACTION 0.9f                     // formed: smallest fraction of planned shift

#define FOLLOW_HEADWAY_SECONDS 1.0f              // target headway at the front squad speed
#define FOLLOW_MIN_GAP_METERS 10.0f              // smallest target headway
#define FOLLOW_GAIN 0.5f                         // follow cap change per meter of headway error

#define CROSS_SIDE_FRACTION 0.5f                 // needed gap fraction on each side to cross


// `fill_route_positions_around` fills the route positions of a squad for the pass checks, around
// its formation and inside its route.
static void fill_route_positions_around(const Squad *squad, const FormationShape *shape,
                                        RoutePositions *positions) {
    float start = fmaxf(0, shape->rear_progress - PASS_BACK_METERS);
    float horizon = squad->anchor_distance + PASS_HORIZON_METERS;
    float end = fminf(squad->route_length, horizon);
    fill_route_positions(squad, start, end, positions);
}


// `locate_on_route` projects `point` onto the route of `positions`, at the closest route position.
// It returns false when the point lies `max_offset` or more to the side, or when `direction` does
// not run the same way as the route there, unless `is_direction_ignored` is true.
static bool locate_on_route(const RoutePositions *positions, V2 point, V2 direction,
                            float max_offset, bool is_direction_ignored, float *route_distance,
                            float *sideways_offset) {
    // Without route positions, `find_closest_position_index` would return an index with no data.
    if (positions->count == 0)
        return false;

    // The sideways offset is positive to the right of the route, as seen facing along it.
    int position_index = find_closest_position_index(positions, point);
    V2 dir = positions->directions[position_index];        // route direction at that position
    V2 to_point = sub(point, positions->points[position_index]);
    float offset = dot(to_point, get_right(dir));

    // The projection along the route direction places the point between the route positions.
    bool is_beside = fabsf(offset) < max_offset;
    float cosine = dot(direction, dir);
    bool is_same_way = is_direction_ignored || cosine >= SAME_WAY_MIN_COSINE;
    if (!is_beside || !is_same_way)
        return false;
    *route_distance = positions->distances[position_index] + dot(to_point, dir);
    *sideways_offset = offset;
    return true;
}


// Two squads that drive the same way on one section of route, as rear squad and front squad.
// `front_anchor_distance` and `front_anchor_offset` locate the anchor of the front squad on the
// route of the rear squad.
typedef struct {
    Squad *rear_squad;
    Squad *front_squad;
    const FormationShape *rear_squad_shape;
    const FormationShape *front_squad_shape;
    const RoutePositions *rear_squad_positions;
    const RoutePositions *front_squad_positions;
    float front_anchor_distance;
    float front_anchor_offset;
} OrderedPair;


// Sections of the two target shifts of a pass, each on the route of its own squad.
typedef struct {
    float rear_squad_start;
    float rear_squad_end;
    float front_squad_start;
    float front_squad_end;
} PassSections;


// Planned target shifts of a pass, and the pass plan shortfall: the part of the missing gap that
// the side rooms of the two squads do not cover.
typedef struct {
    TargetShift rear_squad_shift;
    TargetShift front_squad_shift;
    float shortfall;
} PassPlan;


// `get_front_squad_rear` returns the route position of the rear end of the front squad.
static float get_front_squad_rear(const OrderedPair *pair) {
    const Squad *front = pair->front_squad;
    const FormationShape *shape = pair->front_squad_shape;
    float back = front->anchor_distance - shape->rear_progress;        // body behind the anchor
    return pair->front_anchor_distance - back;
}


// `get_rear_squad_front` returns the route position of the front end of the rear squad.
static float get_rear_squad_front(const OrderedPair *pair) {
    return pair->rear_squad->anchor_distance + pair->rear_squad_shape->front_meters;
}


// `get_closing_speed` returns how much faster the rear squad can drive than the front squad,
// from the top speeds of the two squads.
static float get_closing_speed(const OrderedPair *pair) {
    float v_rear = pair->rear_squad_shape->squad_top_speed;
    return v_rear - pair->front_squad_shape->squad_top_speed;
}


// `is_passing_front_squad` tells whether the rear squad has a pass of the front squad from an
// earlier tick, started or waiting.
static bool is_passing_front_squad(const OrderedPair *pair) {
    return pair->rear_squad->passing_index == pair->front_squad->squad_index;
}


// `order_pair` finds which of two squads is the rear squad: the anchor of the other squad lies
// ahead on its route, less than `needed_gap` to the side, where both routes run the same way.
// It fills `pair` and returns true when one order fits, testing `first` as rear squad first.
static bool order_pair(Squad *first, const FormationShape *first_shape,
                       const RoutePositions *first_positions, Squad *second,
                       const FormationShape *second_shape, const RoutePositions *second_positions,
                       float needed_gap, OrderedPair *pair) {
    for (int order_index = 0; order_index < 2; order_index++) {
        // In the first order, `first` is the rear squad.
        bool is_first_rear = order_index == 0;
        Squad *rear = is_first_rear ? first : second;
        Squad *front = is_first_rear ? second : first;
        const RoutePositions *rear_pos = second_positions;        // route positions of `rear`
        if (is_first_rear)
            rear_pos = first_positions;

        // A rear squad that followed the front squad in the last tick skips the direction test.
        // Without it, a sharp route segment could make a follow pair read as a crossing.
        V2 dir;                  // route direction at the front squad anchor
        V2 anchor = get_route_point(front, front->anchor_distance, &dir);
        float d = 0;             // route position of `anchor` on the rear route
        float offset = 0;        // sideways offset of `anchor` from the rear route
        bool was_following = rear->was_following[front->squad_index];
        bool is_on_route = locate_on_route(rear_pos, anchor, dir, needed_gap, was_following, &d,
                                           &offset);
        bool is_front_squad_ahead = d > rear->anchor_distance;
        if (!is_on_route || !is_front_squad_ahead)
            continue;

        // The shapes and the route positions follow the order of the squads.
        *pair = (OrderedPair){
            .rear_squad = rear,
            .front_squad = front,
            .rear_squad_shape = is_first_rear ? first_shape : second_shape,
            .front_squad_shape = is_first_rear ? second_shape : first_shape,
            .rear_squad_positions = rear_pos,
            .front_squad_positions = is_first_rear ? second_positions : first_positions,
            .front_anchor_distance = d,
            .front_anchor_offset = offset,
        };
        return true;
    }
    return false;
}


// `get_pass_sections` computes the sections of a pass, as if both squads drove at their top
// speeds. The contact is when the gap between the bodies would drop to `PASS_GAP_METERS`, and the
// pass end is when the rear squad is that gap ahead of the front squad.
//
//   now       RRRR            FFFF
//   contact                 RRRR  FFFF
//   pass end                            FFFF  RRRR
//              |<--------------------------------->|  rear squad section
//                               |<---------->|        front squad section
//   ---------------------------------------------------->  route of the rear squad
//   R: body of the rear squad, F: body of the front squad
//
static PassSections get_pass_sections(const OrderedPair *pair) {
    // Route positions of the body ends now, on the route of the rear squad. The prefixes `r_` and
    // `f_` stand for the rear squad and the front squad.
    const FormationShape *r_shape = pair->rear_squad_shape;
    const FormationShape *f_shape = pair->front_squad_shape;
    float v_close = get_closing_speed(pair);
    float v_front = f_shape->squad_top_speed;          // top speed of the front squad
    float f_rear = get_front_squad_rear(pair);         // rear end of the front squad
    float f_front = pair->front_anchor_distance + f_shape->front_meters;
    float r_front = get_rear_squad_front(pair);        // front end of the rear squad

    // When the gap is already smaller than `PASS_GAP_METERS`, the contact is now.
    float d_contact = f_rear - PASS_GAP_METERS - r_front;        // distance left to contact
    float t_contact = d_contact / v_close;                       // time until the contact
    float contact_travel = v_front * fmaxf(0, t_contact);        // front squad travel
    float contact_rear = f_rear + contact_travel;                // f_rear at the contact
    float contact_front = contact_rear - PASS_GAP_METERS;        // r_front at the contact

    // The rear end of the rear squad gains on `target`, which moves with the front squad.
    float target = f_front + PASS_GAP_METERS;            // target of the rear squad rear end
    float gain = target - r_shape->rear_progress;        // distance to gain
    float t_pass = fmaxf(0, gain) / v_close;             // time until the pass end
    float end_travel = v_front * t_pass;                 // front squad travel
    float end_front = f_front + end_travel;              // f_front at the pass end
    float end_rear = target + end_travel;                // rear squad rear end at the pass end

    // The rear squad section starts at the front end of the rear squad, or at `contact_front` when
    // the bodies are already closer than `PASS_GAP_METERS`. No section ends past the route end.
    float r_length = r_front - r_shape->rear_progress;        // body length of the rear squad
    float r_start = fminf(r_front, contact_front) - PASS_END_MARGIN_METERS;
    float r_end = end_rear + r_length + PASS_END_MARGIN_METERS;
    r_end = fminf(pair->rear_squad->route_length, r_end);

    // `route_offset` converts route positions from the route of the rear squad to the route of the
    // front squad.
    float route_offset = pair->front_squad->anchor_distance - pair->front_anchor_distance;
    float f_start = contact_rear + route_offset - PASS_END_MARGIN_METERS;
    float f_end = end_front + route_offset + PASS_END_MARGIN_METERS;
    f_end = fminf(pair->front_squad->route_length, f_end);
    return (PassSections){
        .rear_squad_start = r_start,
        .rear_squad_end = r_end,
        .front_squad_start = f_start,
        .front_squad_end = f_end,
    };
}


// `plan_pass` plans the target shifts of a pass: the side, the section and the side room of each
// squad. `split_missing_gap_in_order` then gives the front squad its share of the missing gap
// first, and the rear squad the rest.
static void plan_pass(const OrderedPair *pair, PassPlan *plan) {
    // The front squad keeps right unless it is clearly left of the rear squad route, and the rear
    // squad passes on the other side. `missing` includes the offset of the front squad.
    float offset = pair->front_anchor_offset;
    float front_side = choose_shift_side(-offset);
    float rear_side = -front_side;
    float needed = get_needed_gap(pair->rear_squad_shape, pair->front_squad_shape,
                                  BODY_GAP_METERS);
    float missing = needed + rear_side * offset;        // missing gap

    // With `--keeproute`, the front squad has no side room, so the rear squad takes the whole
    // missing gap.
    Squad *rear = pair->rear_squad;
    Squad *front = pair->front_squad;
    PassSections sections = get_pass_sections(pair);
    float step = PASS_SIDE_ROOM_STEP_METERS;
    float rear_room = measure_min_side_room(rear, sections.rear_squad_start,
                                            sections.rear_squad_end, step, rear_side);
    float front_room = 0;
    if (!is_keep_route_on) {
        front_room = measure_min_side_room(front, sections.front_squad_start,
                                           sections.front_squad_end, step, front_side);
    }

    // The side rooms leave `PASS_SIDE_ROOM_MARGIN_METERS` free next to the lane clamps.
    plan->rear_squad_shift = (TargetShift){
        .squad = rear,
        .side = rear_side,
        .side_room = fmaxf(0, rear_room - PASS_SIDE_ROOM_MARGIN_METERS),
        .section_start = sections.rear_squad_start,
        .section_end = sections.rear_squad_end,
        .magnitude = 0,
    };
    plan->front_squad_shift = (TargetShift){
        .squad = front,
        .side = front_side,
        .side_room = fmaxf(0, front_room - PASS_SIDE_ROOM_MARGIN_METERS),
        .section_start = sections.front_squad_start,
        .section_end = sections.front_squad_end,
        .magnitude = 0,
    };
    plan->shortfall = split_missing_gap_in_order(missing, &plan->front_squad_shift,
                                                 &plan->rear_squad_shift);
}


// `has_squad_formed` tells whether a squad has shifted for a pass: its shift has reached
// `FORMED_FRACTION` of `planned_shift_magnitude`, and every squad member is near its lane point.
static bool has_squad_formed(const Squad *squad, float planned_shift_magnitude) {
    // With `--keeproute`, the planned shift magnitude is zero, and only the lane test applies.
    float min_shift = FORMED_FRACTION * planned_shift_magnitude;
    if (fabsf(squad->shift) < min_shift)
        return false;

    // The lane point of a squad member is the point of its lane at its route progress.
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        V2 lane_point = get_lane_point(squad, unit, unit->route_progress);
        float d = len(sub(unit->position, lane_point));        // distance to the lane point
        if (d > FORMED_LANE_METERS)
            return false;
    }
    return true;
}


// `cap_rear_squad_speed` lowers the speed cap of the rear squad to the follow cap: the speed of
// the front squad, plus `FOLLOW_GAIN` times the difference between the headway and
// `target_headway`. A lower speed cap of the rear squad stays.
static void cap_rear_squad_speed(const OrderedPair *pair, float target_headway) {
    Squad *rear = pair->rear_squad;
    float headway = get_front_squad_rear(pair) - get_rear_squad_front(pair);
    float error = headway - target_headway;        // headway error
    float v_follow = pair->front_squad->anchor_speed + FOLLOW_GAIN * error;
    float cap = fmaxf(0, v_follow);                // follow cap
    rear->speed_cap = fminf(rear->speed_cap, cap);
}


// `try_pass` plans a new pass of the front squad by the rear squad, or goes on with a pass from an
// earlier tick. It returns false when no pass applies, and the caller then gives the rear squad a
// follow cap.
static bool try_pass(const OrderedPair *pair, float target_headway) {
    // No pass is planned for a front squad that is not clearly slower, or that the rear squad
    // would catch up with only after `PASS_LOOK_SECONDS`.
    Squad *rear = pair->rear_squad;
    Squad *front = pair->front_squad;
    float v_close = get_closing_speed(pair);
    if (v_close < PASS_MIN_SPEED_GAIN)
        return false;
    float f_rear = get_front_squad_rear(pair);          // rear end of the front squad
    float r_front = get_rear_squad_front(pair);         // front end of the rear squad
    float d = f_rear - target_headway - r_front;        // distance to the target headway
    float t_catch = d / v_close;                        // catch-up time
    if (t_catch > PASS_LOOK_SECONDS)
        return false;

    // A started pass goes on whatever its shortfall. Without enough side room, a new pass does not
    // start, and a waiting pass keeps the rear squad behind the front squad.
    bool is_same_pass = is_passing_front_squad(pair);
    bool is_waiting_pass = is_same_pass && rear->is_pass_waiting;
    bool is_started_pass = is_same_pass && !rear->is_pass_waiting;
    PassPlan plan;
    plan_pass(pair, &plan);
    bool is_pass_possible = is_started_pass || plan.shortfall <= PASS_SQUEEZE_METERS;
    if (!is_pass_possible) {
        if (!is_waiting_pass)
            return false;
        rear->is_pass_kept = true;
        cap_rear_squad_speed(pair, target_headway);
        return true;
    }

    // The target shifts apply in every tick of the pass, also while the rear squad waits behind.
    record_target_shifts(&plan.rear_squad_shift, &plan.front_squad_shift, SHIFT_EVENT_PASS);
    rear->passing_index = front->squad_index;
    rear->is_pass_kept = true;

    // With `--formfirst`, a new pass waits behind until the front squad has formed, unless the
    // bodies are already closer than `PASS_GAP_METERS`.
    if (!is_started_pass) {
        bool is_behind = f_rear - r_front >= PASS_GAP_METERS;
        bool is_front_formed = has_squad_formed(front, plan.front_squad_shift.magnitude);
        bool is_waiting = is_form_first_on && is_behind && !is_front_formed;
        rear->is_pass_waiting = is_waiting;
        if (is_waiting)
            cap_rear_squad_speed(pair, target_headway);
    }
    return true;
}


// `does_route_cross_front_squad` tells whether the route of the rear squad, ahead of its anchor,
// goes from one side of the route of the front squad to the other. The shifts of a pass assume
// side-by-side routes, and a pass across the route of the front squad could run into that squad.
static bool does_route_cross_front_squad(const OrderedPair *pair, float needed_gap) {
    // A rear squad anchor close to the route of the front squad has no side to cross from.
    const Squad *rear = pair->rear_squad;
    const RoutePositions *rear_pos = pair->rear_squad_positions;          // rear squad route
    const RoutePositions *front_pos = pair->front_squad_positions;        // front squad route
    float min_offset = CROSS_SIDE_FRACTION * needed_gap;                  // offset for one side
    V2 anchor_dir;
    V2 anchor = get_route_point(rear, rear->anchor_distance, &anchor_dir);
    float unused;
    float start;                                                          // offset of `anchor`
    bool is_located = locate_on_route(front_pos, anchor, anchor_dir, LARGE_VALUE, true, &unused,
                                      &start);
    if (!is_located || fabsf(start) < min_offset)
        return false;

    // `locate_on_route` cannot fail in this loop: it ignores the direction, has no offset limit,
    // and the route positions of the front squad are not empty.
    for (int position_index = 0; position_index < rear_pos->count; position_index++) {
        if (rear_pos->distances[position_index] <= rear->anchor_distance)
            continue;
        V2 point = rear_pos->points[position_index];
        V2 dir = rear_pos->directions[position_index];
        float offset;
        locate_on_route(front_pos, point, dir, LARGE_VALUE, true, &unused, &offset);
        bool is_other_side = offset * start < 0;
        if (is_other_side && fabsf(offset) >= min_offset)
            return true;
    }
    return false;
}


// `pass_or_follow` makes the rear squad of two squads on one section of route pass the front
// squad or follow it. It returns false when the two squads do not drive the same way on one
// section of route, and true otherwise.
bool pass_or_follow(Squad *first, Squad *second) {
    // In `order_pair`, the needed gap `needed` is also the limit on the sideways offset of the
    // front squad anchor from the route of the rear squad.
    FormationShape first_shape = measure_shape(first);
    FormationShape second_shape = measure_shape(second);
    static RoutePositions first_pos;         // route positions of `first`
    static RoutePositions second_pos;        // route positions of `second`
    fill_route_positions_around(first, &first_shape, &first_pos);
    fill_route_positions_around(second, &second_shape, &second_pos);
    float needed = get_needed_gap(&first_shape, &second_shape, BODY_GAP_METERS);
    OrderedPair pair;
    bool is_same_way_pair = order_pair(first, &first_shape, &first_pos, second, &second_shape,
                                       &second_pos, needed, &pair);
    if (!is_same_way_pair)
        return false;

    // `is_following` becomes `was_following` in the next tick, for `order_pair`. The target
    // headway grows with the speed of the front squad.
    Squad *rear = pair.rear_squad;
    Squad *front = pair.front_squad;
    rear->is_following[front->squad_index] = true;
    float headway = FOLLOW_HEADWAY_SECONDS * front->anchor_speed;        // headway at that speed
    float target = fmaxf(FOLLOW_MIN_GAP_METERS, headway);                // target headway

    // A new pass needs a route that does not cross the route of the front squad, but a started
    // pass goes on. Without a pass, the rear squad follows.
    bool is_started_pass = is_passing_front_squad(&pair) && !rear->is_pass_waiting;
    bool can_try_pass = true;
    if (!is_started_pass)
        can_try_pass = !does_route_cross_front_squad(&pair, needed);
    if (can_try_pass && try_pass(&pair, target))
        return true;
    cap_rear_squad_speed(&pair, target);
    return true;
}
