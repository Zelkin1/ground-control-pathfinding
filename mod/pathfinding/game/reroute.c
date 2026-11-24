// Re-routes of moving squads around parked squads in the coordination mode (`--parkreroute`). When
// a squad arrives or settles, every moving squad whose remaining route goes past its squad members
// searches its route again, around a parking disc: a disc over the whole parked squad, stamped as
// cliffs (squad.c). A moving squad also re-routes early around the final parking positions of
// another moving squad that would get there first (park ahead), unless `--moveahead` covers that.
//
// With `--parkrobust`, a squad at its route end counts as settled, like a parked squad, once each
// squad member stands, turns on the spot, or circles tightly.
#include "sim.h"
#include "formation.h"

#define PARKED_ROUTE_STEP_METERS 2.0f          // step of the route checks for parked squads
#define PARKED_ROUTE_MARGIN_METERS 2.0f        // body gap at which a route goes past a unit
#define TIGHT_CIRCLE_RADII 2.0f                // largest tight circle, in unit radii
#define PARK_AHEAD_SECONDS 16.0f               // park-ahead look time at the anchor speed
#define NO_GO_PAST_DISTANCE (-1.0f)            // the route does not go past the squad


// `find_go_past_distance` returns the first route position of `moving`, from `from` to `to` at
// steps of `PARKED_ROUTE_STEP_METERS`, where a formation of half width `half_width` would come
// within `PARKED_ROUTE_MARGIN_METERS` of the body of a squad member of `parked`, or
// `NO_GO_PAST_DISTANCE`. `is_at_parking_positions` reads final parking positions instead.
static float find_go_past_distance(const Squad *moving, float half_width, const Squad *parked,
                                   bool is_at_parking_positions, float from, float to) {
    float d = from;        // route position
    int end_index = parked->first_unit_index + parked->unit_count;
    while (d <= to) {
        V2 unused;
        V2 route_point = get_route_point(moving, d, &unused);
        for (int unit_index = parked->first_unit_index; unit_index < end_index; unit_index++) {
            const Unit *member = &units[unit_index];
            V2 member_point = get_member_point(member, is_at_parking_positions);
            float d_touch = half_width + member->unit_type->radius;
            float gap = d_touch + PARKED_ROUTE_MARGIN_METERS;
            if (length_squared(sub(member_point, route_point)) < gap * gap)
                return d;
        }
        d += PARKED_ROUTE_STEP_METERS;
    }
    return NO_GO_PAST_DISTANCE;
}


// `find_parking_point_distance` returns the route position of the parking point of `parker` on
// the route of `moving`: from `go_past_distance` on, the route point where the distance to the
// nearest final parking position of `parker` stops shrinking.
static float find_parking_point_distance(const Squad *moving, const Squad *parker,
                                         float go_past_distance) {
    float best = go_past_distance;           // route position of the parking point
    float best_squared = LARGE_VALUE;        // its squared distance
    int end_index = parker->first_unit_index + parker->unit_count;
    for (float d = go_past_distance; d <= moving->route_length; d += PARKED_ROUTE_STEP_METERS) {
        V2 unused;
        V2 route_point = get_route_point(moving, d, &unused);
        float gap_squared = LARGE_VALUE;
        for (int unit_index = parker->first_unit_index; unit_index < end_index; unit_index++) {
            V2 parking_position = units[unit_index].parking_position;
            float parking_squared = length_squared(sub(parking_position, route_point));
            gap_squared = fminf(gap_squared, parking_squared);
        }

        if (gap_squared >= best_squared)
            break;
        best = d;
        best_squared = gap_squared;
    }
    return best;
}


// `does_route_go_past_squad` tells whether the route of `moving`, from the rear end of its
// formation to the route end, goes past a squad member of `parked`.
static bool does_route_go_past_squad(const Squad *moving, const Squad *parked) {
    FormationShape shape = measure_shape(moving);
    float from = fmaxf(0, shape.rear_progress);
    float to = moving->route_length;
    float d = find_go_past_distance(moving, shape.half_width, parked, false, from, to);
    return d != NO_GO_PAST_DISTANCE;
}


// `reroute_around_parked` makes every moving squad that coordinates with `parked` and whose route
// goes past it search its route again, around the parking disc of `parked`.
void reroute_around_parked(const Squad *parked) {
    if (!is_park_reroute_on || avoid_mode != AVOID_COORD)
        return;
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        Squad *other = &squads[squad_index];
        bool is_moving_other = other != parked && other->state == SQUAD_MOVING;
        bool is_coordinated_other = is_moving_other && are_squads_coordinated(other, parked);
        if (!is_coordinated_other || !does_route_go_past_squad(other, parked))
            continue;
        other->is_rerouting = true;
        other->is_move_ahead_reroute = false;
        start_route_search(other);
    }
}


// `avoid_parking_ahead` makes a moving squad re-route around the final parking positions of
// another moving squad that coordinates with it and that its route goes past within
// `PARK_AHEAD_SECONDS` at its anchor speed, when that squad would reach them first. A squad
// re-routes at most once per tick for this. With `--moveahead`, the predictions of moving_ahead.c
// cover the final parking positions, so it stays off.
void avoid_parking_ahead(Squad *squad) {
    bool is_move_ahead_used = is_move_ahead_on && is_park_robust_on;
    bool is_used = is_park_reroute_on && avoid_mode == AVOID_COORD && !is_move_ahead_used;
    if (!is_used)
        return;

    // A squad avoids the final parking positions of each other squad once per order.
    FormationShape shape = measure_shape(squad);
    float front = squad->anchor_distance + shape.front_meters;
    float look = PARK_AHEAD_SECONDS * squad->anchor_speed;        // look distance
    float look_end = fminf(front + look, squad->route_length);
    for (int other_index = 0; other_index < squad_count; other_index++) {
        const Squad *other = &squads[other_index];
        bool is_parking_other = other != squad && other->state == SQUAD_MOVING;
        bool is_coordinated_other = is_parking_other && are_squads_coordinated(squad, other);
        bool is_new_other = is_coordinated_other && !other->is_settled;
        if (!is_new_other || squad->is_avoiding_parking[other_index])
            continue;
        float d_past = find_go_past_distance(squad, shape.half_width, other, true, front,
                                             look_end);
        if (d_past == NO_GO_PAST_DISTANCE)
            continue;

        // Both times use the top speeds and end at the final parking positions of the other
        // squad: its remaining route, and the way of the front end of this squad to the parking
        // point.
        float other_top_speed = measure_shape(other).squad_top_speed;
        float other_remaining = other->route_length - other->anchor_distance;
        float other_seconds = other_remaining / other_top_speed;
        float d_parking = find_parking_point_distance(squad, other, d_past);
        float own_seconds = (d_parking - front) / shape.squad_top_speed;
        if (other_seconds >= own_seconds)
            continue;
        squad->is_avoiding_parking[other_index] = true;
        squad->is_rerouting = true;
        squad->is_move_ahead_reroute = false;
        start_route_search(squad);
        return;
    }
}


// `is_circling_tightly` tells whether a unit drives a circle smaller than `TIGHT_CIRCLE_RADII`
// times its radius: a unit at the speed `v` and the turn rate `w` drives a circle of radius
// `v / w`.
static bool is_circling_tightly(const Unit *unit) {
    float radius = unit->unit_type->radius;
    float v_circle = TIGHT_CIRCLE_RADII * radius * fabsf(unit->last_turn_rate);
    return len(unit->velocity) < v_circle;
}


// `update_settled` marks a squad at its route end as settled once each squad member stands, turns
// on the spot, or circles tightly (`--parkrobust`). A squad that settles makes the moving squads
// re-route around it.
void update_settled(Squad *squad) {
    bool is_used = is_park_robust_on && is_park_reroute_on && avoid_mode == AVOID_COORD;
    float end_distance = squad->route_length - ROUTE_END_EPSILON_METERS;
    bool is_at_route_end = squad->anchor_distance >= end_distance;
    if (!is_used || !is_at_route_end) {
        squad->is_settled = false;
        return;
    }

    // The re-route runs only in the tick in which the squad settles.
    bool are_members_parked = true;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *member = &units[unit_index];
        bool is_turning = member->is_turning_on_spot || is_circling_tightly(member);
        if (!member->is_stopped && !is_turning)
            are_members_parked = false;
    }
    bool has_just_settled = are_members_parked && !squad->is_settled;
    squad->is_settled = are_members_parked;
    if (has_just_settled)
        reroute_around_parked(squad);
}
