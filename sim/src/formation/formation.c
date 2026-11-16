/* formation.c moves squads by formation following, the "form" mode of `--avoid`. squad.c calls
 * `set_up_formation` when a squad gets its route and `advance_anchor` once per tick for each
 * moving squad. This file sets up the formation, moves the anchor and gives each member its
 * place. steering.c drives each member: movement.c calls `steer_in_formation` for each moving
 * unit of such a squad. The units avoid each other with `compute_orca_velocity` from
 * avoidance.c. route.c shortens and smooths the route, and lanes.c lays out the lane of each
 * member.
 * This mode uses no waypoint lists, re-paths or AvoidUnits. Only a unit that makes no progress
 * gets a new way, a detour, from replan.c.
 *
 * Method:
 *   A virtual leader, the anchor, moves along the squad route at the
 * top speed of the slowest member. It slows down while members fall behind their places, and with
 * `--turnslow on` also through sharp turns, so units that get ahead wait for the others. Each
 * member has a place, a point that moves with the anchor, and drives toward it. A place keeps the
 * slot offset of the member from the anchor, `place_offset`, with x to the right of the direction
 * of the formation and y along it. The slots are the final positions that the formation layout of
 * the game gives around the destination.
 *
 *                 ^ forward: the direction of the formation
 *                 |
 *        A . . . .@. . . . B        @    the anchor, on the squad route
 *        :        |        :        A B  places at `place_offset` from the anchor
 *        :        |        :        :    lanes, along which the places move
 *        :        C        :        C    a place with no sideways offset
 *        :        |        :
 *                 | the squad route
 *
 * Parts of the method:
 * - At the order, the route is moved to start where the formation fits the members. Then it is
 *   shortened and smoothed into a curve with room for the whole formation. Members are matched to
 *   slots so that they keep their arrangement.
 * - Each member drives along its own lane, a line beside the route at the sideways offset of its
 *   slot, fitted into the free ground. Lanes never cross. Where the squad does not fit side by
 *   side, it drives in single file down the middle of the passage.
 * - Over the first `START_BLEND_METERS` of the anchor, places blend in from where the members
 *   stood. Over the last `END_BLEND_METERS` of the route, they blend into the fixed formation
 *   shape, which ends on the slots with the final facing.
 * - Every `REASSIGN_INTERVAL_SECONDS`, members are matched to places again, so that a unit that
 *   got ahead takes a front place.
 * - The route progress of a unit, `route_progress`, is the distance along the route of the route
 *   point closest to it. A unit far behind its place, or without a straight way to it, follows
 *   its lane from there. A unit further than `PURSUIT_PLACE_METERS` from its place steers along
 *   its lane, and the place sets only its speed. Units slow down before bends of their lanes and
 *   brake onto their slots. steering.c holds these parts.
 * With `--lanes off`, a place is the anchor plus the slot offset, turned to the direction of the
 * formation. A unit whose place is cut off by blocked ground falls in behind the anchor in single
 * file, or rejoins the route.
 *
 * Members:
 *   The members of a squad are `units[first_unit_index]` to
 *   `units[first_unit_index + unit_count - 1]`.
 *
 * Fields of `Unit` that this file uses:
 *   `set_up_formation` sets `slot` and `place_offset` (the slot as an offset from the end of the
 *   route, in route coordinates), `file_rank` (the number of places ahead in single file) and
 *   `file_distance` (the gap behind the anchor in single file: `file_rank` times the file
 *   spacing). It also sets `start_offset` (the member position minus its place at the order, which
 *   `get_target_place` fades out) and resets the movement state of each member. `reassign_places`
 *   moves `slot`, `place_offset`, `file_rank` and `file_distance` together from one member to
 *   another, because they describe one place. */
#include <limits.h>
#include <string.h>

#include "sim.h"
#include "formation.h"

#define LARGE_SQUARED_VALUE 1e30f /* Minimum searches of squared values start here. */

/* This value shapes the blend of the places at the start of the route. */
#define START_BLEND_METERS 25.0f /* Places blend in from the start over this. */

/* These values set up the places and match members to them. */
#define MAX_MATCHED_MEMBERS 8             /* Squads up to this size try every match to slots. */
#define FILE_SPACING_RADII 2.2f           /* A file gap is this many radii of the widest member. */
#define FILE_SPACING_GAP_METERS 2.0f      /* A file gap is this much more than those radii. */
#define FILE_HEADWAY_SECONDS 0.3f         /* File gaps grow by this time at anchor speed. */
#define MIN_FORMATION_REACH_METERS 2.0f   /* `reach` of a unit is its radius, but at least this. */
#define COORDINATES_BEHIND_METERS 80.0f   /* Coordinates are sought this far before the anchor. */
#define COORDINATES_AHEAD_METERS 40.0f    /* Coordinates are sought this far past the anchor. */
#define REASSIGN_INTERVAL_SECONDS 0.5f    /* Members are matched to places again this often. */
#define REASSIGN_SIDE_WEIGHT 4.0f         /* A sideways difference counts this many times. */
#define REASSIGN_MIN_SAVING_FRACTION 0.1f /* New matches must save this part of the cost. */
#define REASSIGN_MIN_SAVING_SQUARED 10.0f /* New matches must save this many m^2 at least. */

/* These values set the speed of the anchor. */
#define LAG_SLOWDOWN_FRACTION 0.1f     /* Lag slows the anchor by up to this part of its speed. */
#define LAG_SLOWDOWN_METERS 4.0f       /* The lag slowdown is full at this lag. */
#define TURN_SPEED_FLOOR 0.55f         /* Turns of 90 degrees or more keep this part of speed. */
#define TURN_SPEED_RANGE 0.45f         /* Gentler turns add this part times turn cosine. */
#define REAR_MARGIN_METERS 10.0f       /* Route checks reach this far past the formation ends. */
#define TURN_LOOK_AHEAD_METERS 25.0f   /* The turn is measured to this far past the anchor. */
#define FILE_LEAVE_AHEAD_METERS 20.0f  /* Single file ends once this far past the front fits. */
#define END_DECELERATION_FRACTION 0.5f /* End braking uses this part of the lowest friction. */
#define LAG_WAIT_METERS 8.0f           /* Past this lag, the anchor slows further for laggards. */
#define LAG_WAIT_RANGE_METERS 20.0f    /* The wait slowdown grows by 1 per this many m of lag. */
#define CRAWL_FRACTION 0.25f           /* Waiting anchors keep at least this part of the speed. */
#define LAG_STOP_METERS 60.0f          /* The anchor stops only for a lag past this. */
#define ANCHOR_STILL_SPEED 0.2f        /* Below this speed, the anchor counts as held still. */
#define STUCK_WAIT_SECONDS 10.0f       /* After this long held still, the anchor moves on. */
#define IGNORE_LAG_SECONDS 5.0f        /* The anchor then ignores laggards for this long. */

/* ------------------------------------------------------------------------------------------------
 * `blend` returns the mix of `from` and `to` that has `weight` of `to`: `from` at 0 and `to` at 1.
 * --------------------------------------------------------------------------------------------- */
static V2 blend(V2 from, V2 to, float weight) {
    V2 from_part = mul(from, 1 - weight);
    V2 to_part = mul(to, weight);
    return add(from_part, to_part);
}

/* ------------------------------------------------------------------------------------------------
 * `get_end_weight` returns how far `distance` lies into the last `END_BLEND_METERS` of the route:
 * 0 where they start and 1 at the end of the route. The value is negative before them.
 * --------------------------------------------------------------------------------------------- */
static float get_end_weight(const Squad *squad, float distance) {
    return 1 - (squad->route_length - distance) / END_BLEND_METERS;
}

/* ------------------------------------------------------------------------------------------------
 * `get_formation_frame` returns the route point at `distance` along the route, and sets
 * `forward` to the direction of the formation there. Over the last `END_BLEND_METERS` of the
 * route, `forward` turns gradually to `final_facing`, which it reaches at the end, unless
 * `final_facing` is zero.
 * --------------------------------------------------------------------------------------------- */
static V2 get_formation_frame(const Squad *squad, float distance, V2 *forward) {
    V2 point = get_route_point(squad, distance, forward);
    *forward = get_formation_direction(squad, distance);
    float end_weight = get_end_weight(squad, distance);
    if (end_weight > 0 && length_squared(squad->final_facing) > 0) {
        float facing_weight = end_weight > 1 ? 1 : end_weight;
        V2 blended = blend(*forward, squad->final_facing, facing_weight);
        *forward = get_direction(blended, LENGTH_SQUARED_EPSILON, *forward);
    }
    return point;
}

/* ------------------------------------------------------------------------------------------------
 * With lanes, `get_file_distance` adds `FILE_HEADWAY_SECONDS` of the speed of the anchor for each
 * rank to `file_distance`. So the gap to the unit ahead in the file grows with the speed, and
 * units following each other have time to brake. Without lanes, the gap stays fixed.
 * --------------------------------------------------------------------------------------------- */
float get_file_distance(const Squad *squad, const Unit *unit) {
    float headway = 0;
    if (are_lanes_on)
        headway = unit->file_rank * FILE_HEADWAY_SECONDS * squad->anchor_speed;
    return unit->file_distance + headway;
}

/* ------------------------------------------------------------------------------------------------
 * `get_place_distance` takes the distance behind the anchor from `get_file_distance` in single
 * file, and the forward offset `place_offset.y` ahead of the anchor otherwise.
 * --------------------------------------------------------------------------------------------- */
float get_place_distance(const Squad *squad, const Unit *unit) {
    if (squad->is_single_file)
        return squad->anchor_distance - get_file_distance(squad, unit);
    return squad->anchor_distance + unit->place_offset.y;
}

/* ------------------------------------------------------------------------------------------------
 * `get_place` returns the place of a member and sets `forward` to the direction of the formation
 * there.
 * Result: The place in the fixed formation shape is `place_offset` from the anchor, turned to the
 *   direction of the formation. With lanes, the place is on the lane of the member at
 *   `get_place_distance` instead, which in single file is in the middle of the passage. Over the
 *   last `END_BLEND_METERS` of the route, it blends into the place in the shape, which ends on
 *   the slot.
 * Edge case: Without lanes, the place is the one in the shape. When blocked ground lies on the
 *   straight line from the anchor to it, the place is the route point `file_distance` behind the
 *   anchor along the route instead.
 * --------------------------------------------------------------------------------------------- */
static V2 get_place(const Squad *squad, const Unit *unit, V2 *forward) {
    /* Find the place in the fixed formation shape around the anchor. */
    V2 anchor_point = get_formation_frame(squad, squad->anchor_distance, forward);
    V2 shape_offset = get_map_offset(unit->place_offset, *forward);
    V2 shape_place = add(anchor_point, shape_offset);
    if (!are_lanes_on) {
        if (!is_line_clear(anchor_point, shape_place, unit->kind->mover_class)) {
            V2 unused_direction;
            float file_place_distance = squad->anchor_distance - unit->file_distance;
            shape_place = get_route_point(squad, file_place_distance, &unused_direction);
        }
        return shape_place;
    }

    /* Find the place on the lane of the member, and blend it into the shape near the end. */
    float distance = get_place_distance(squad, unit);
    V2 lane_forward = get_formation_direction(squad, distance);
    V2 lane_place = get_lane_point(squad, unit, distance);
    float end_weight = clamp_float(get_end_weight(squad, squad->anchor_distance), 0, 1);
    V2 blended = blend(lane_forward, *forward, end_weight);
    *forward = get_direction(blended, LENGTH_SQUARED_EPSILON, lane_forward);
    return blend(lane_place, shape_place, end_weight);
}

/* ------------------------------------------------------------------------------------------------
 * `get_target_place` adds `start_offset` to the place from `get_place`. At the order, the target
 * place is where the member stands, because `start_offset` is the position of the member less its
 * place then. The offset fades out linearly over the first `START_BLEND_METERS` of the distance
 * that the anchor drives, so units merge in instead of swerving.
 * --------------------------------------------------------------------------------------------- */
V2 get_target_place(const Squad *squad, const Unit *unit, V2 *forward) {
    V2 place = get_place(squad, unit, forward);
    float start_weight = fmaxf(0, 1 - squad->anchor_distance / START_BLEND_METERS);
    return add(place, mul(unit->start_offset, start_weight));
}

/* ------------------------------------------------------------------------------------------------
 * `project_onto_route` checks each route segment in the range. On a segment, the closest point to
 * `point` is the foot of the perpendicular from `point`, kept on the segment and in the range:
 *
 *                      point
 *                        o
 *                        |
 *      o-----------------+------------o
 *   segment_start     closest
 * --------------------------------------------------------------------------------------------- */
float project_onto_route(const Squad *squad, V2 point, float from_distance, float to_distance) {
    float best_distance_squared = LARGE_SQUARED_VALUE;
    float best_route_distance = from_distance;
    for (int end_index = 1; end_index < squad->route_count; end_index++) {
        /* Skip segments outside the range. */
        float segment_start_distance = squad->route_distance[end_index - 1];
        float segment_end_distance = squad->route_distance[end_index];
        float segment_length = segment_end_distance - segment_start_distance;
        bool is_outside_range =
            segment_end_distance < from_distance || segment_start_distance > to_distance;
        if (is_outside_range || segment_length <= 0)
            continue;

        /* Find the point closest to `point` on the segment, kept in the range. */
        V2 segment_start = squad->route[end_index - 1];
        V2 segment = sub(squad->route[end_index], segment_start);
        V2 from_start = sub(point, segment_start);
        float fraction = dot(from_start, segment) / (segment_length * segment_length);
        float from_fraction = (from_distance - segment_start_distance) / segment_length;
        float to_fraction = (to_distance - segment_start_distance) / segment_length;
        fraction = fmaxf(fmaxf(fraction, from_fraction), 0);
        fraction = fminf(fminf(fraction, to_fraction), 1);
        V2 closest = add(segment_start, mul(segment, fraction));
        float distance_squared = length_squared(sub(point, closest));
        if (distance_squared < best_distance_squared) {
            best_distance_squared = distance_squared;
            best_route_distance = segment_start_distance + fraction * segment_length;
        }
    }
    return best_route_distance;
}

/* ------------------------------------------------------------------------------------------------
 * `get_route_coordinates` returns `point` in route coordinates, which use the same axes as
 * `place_offset`: x is the offset to the right of the route, and y the distance along it. The
 * route point closest to `point` is searched from `COORDINATES_BEHIND_METERS` before the anchor
 * to `COORDINATES_AHEAD_METERS` past it.
 *
 *              ^ forward
 *              |
 *    point o---+     x = `sideways`, here negative, to the left
 *              |     y = `distance` + `along`
 *              | along
 *              o route point at `distance`
 * --------------------------------------------------------------------------------------------- */
static V2 get_route_coordinates(const Squad *squad, V2 point) {
    /* Find the closest route point near the anchor. */
    float from_distance = fmaxf(0, squad->anchor_distance - COORDINATES_BEHIND_METERS);
    float to_distance = squad->anchor_distance + COORDINATES_AHEAD_METERS;
    float distance = project_onto_route(squad, point, from_distance, to_distance);

    /* Split the offset from it into its parts across and along the direction of the formation. */
    V2 unused_direction;
    V2 forward = get_formation_direction(squad, distance);
    V2 route_point = get_extended_route_point(squad, distance, &unused_direction);
    V2 from_route = sub(point, route_point);
    float along = dot(from_route, forward);
    float sideways = dot(from_route, get_right(forward));
    return v2(sideways, distance + along);
}

/* ------------------------------------------------------------------------------------------------
 * `swap_ints` swaps the values of two ints.
 * --------------------------------------------------------------------------------------------- */
static void swap_ints(int *first, int *second) {
    int kept = *first;
    *first = *second;
    *second = kept;
}

/* ------------------------------------------------------------------------------------------------
 * `advance_permutation` changes `permutation`, an ordering of `count` different numbers, to the
 * next one in lexicographic order, the order in which a dictionary lists words.
 * Result: It returns true, or false when `permutation` is the last one, in falling order. Then it
 *   leaves `permutation` as it is.
 * --------------------------------------------------------------------------------------------- */
static bool advance_permutation(int *permutation, int count) {
    /* Find the last position whose number is smaller than the next one. */
    int pivot_index = count - 2;
    while (pivot_index >= 0 && permutation[pivot_index] > permutation[pivot_index + 1])
        pivot_index--;
    if (pivot_index < 0)
        return false;

    /* Swap it with the last number that is larger, then reverse the numbers after it. */
    int larger_index = count - 1;
    while (permutation[larger_index] < permutation[pivot_index])
        larger_index--;
    swap_ints(&permutation[pivot_index], &permutation[larger_index]);
    int left_index = pivot_index + 1;
    int right_index = count - 1;
    while (left_index < right_index) {
        swap_ints(&permutation[left_index], &permutation[right_index]);
        left_index++;
        right_index--;
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `find_best_match` finds the match of members to places with the smallest total cost, and tries
 * every permutation of the places for it.
 * Inputs: `costs[member_index][place_index]` is the cost of the member taking the place.
 *   `best_cost` must hold the cost to beat.
 * Result: `best_match[member_index]` is the place of each member in the best match, and
 *   `best_cost` its total cost. When no match costs less than `best_cost`, each member keeps its
 *   own place in `best_match`, and `best_cost` stays.
 * Edge case: For more than `MAX_MATCHED_MEMBERS` members, `find_best_match` tries only the match
 *   that keeps each member on its own place.
 * --------------------------------------------------------------------------------------------- */
static void find_best_match(float costs[][MAX_UNITS], int member_count, float *best_cost,
                            int *best_match) {
    int permutation[MAX_UNITS];
    for (int member_index = 0; member_index < member_count; member_index++) {
        permutation[member_index] = member_index;
        best_match[member_index] = member_index;
    }
    bool is_matchable = member_count <= MAX_MATCHED_MEMBERS;
    do {
        /* Add up the costs, and stop early once the sum cannot win. */
        float cost = 0;
        int member_index = 0;
        while (member_index < member_count && cost < *best_cost) {
            cost += costs[member_index][permutation[member_index]];
            member_index++;
        }
        if (cost < *best_cost) {
            *best_cost = cost;
            memcpy(best_match, permutation, sizeof(int) * member_count);
        }
    } while (is_matchable && advance_permutation(permutation, member_count));
}

/* ------------------------------------------------------------------------------------------------
 * `set_up_formation` runs the three parts of the set-up in one go: `begin_formation_set_up`, the
 * whole smoothing, and `finish_formation_set_up`. The route searches and ground tests of these
 * parts see the parked units as cliffs only because `stamp_parked` runs around the call of the
 * caller.
 * --------------------------------------------------------------------------------------------- */
void set_up_formation(Squad *squad, const float *slots, V2 facing) {
    bool is_smoothing = begin_formation_set_up(squad, slots, facing);
    if (is_smoothing)
        run_smoothing(INT_MAX);
    finish_formation_set_up(squad, slots, facing);
}

/* ------------------------------------------------------------------------------------------------
 * `measure_members` finds the radius of the widest member, the mean position of the members, and
 * the smallest bend radius that every member can drive at its top speed: `limit_speed_for_bend`
 * allows `BEND_SPEED_MARGIN * turn_rate * radius`.
 * --------------------------------------------------------------------------------------------- */
static void measure_members(const Squad *squad, float *max_radius, float *full_speed_radius,
                            V2 *center) {
    int member_count = squad->unit_count;
    const Unit *members = &units[squad->first_unit_index];
    float mean_weight = 1.0f / member_count;
    *max_radius = 0;
    *full_speed_radius = 0;
    *center = v2(0, 0);
    for (int member_index = 0; member_index < member_count; member_index++) {
        const Unit *member = &members[member_index];
        *max_radius = fmaxf(*max_radius, member->kind->radius);
        float bend_rate = BEND_SPEED_MARGIN * member->kind->turn_rate;
        *full_speed_radius = fmaxf(*full_speed_radius, member->kind->max_speed / bend_rate);
        *center = add(*center, mul(member->position, mean_weight));
    }
}

/* ------------------------------------------------------------------------------------------------
 * `prepare_formation_route` starts the route where the formation fits the members, and shortens
 * it. It takes the route from the center of the members instead when that one is shorter.
 * --------------------------------------------------------------------------------------------- */
void prepare_formation_route(Squad *squad, const float *slots, V2 facing) {
    Unit *members = &units[squad->first_unit_index];
    int mover_class = members[0].kind->mover_class;
    float max_radius;
    float full_speed_radius;
    V2 center;
    measure_members(squad, &max_radius, &full_speed_radius, &center);
    if (squad->route_count > 1)
        move_route_start(squad, slots, facing, center, mover_class);
    shorten_route(squad, max_radius, mover_class);
    if (is_center_search_on) {
        float clearance = get_search_clearance(&members[0]);
        take_shorter_center_route(squad, slots, facing, center, max_radius, mover_class,
                                  clearance);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `begin_formation_smoothing` starts the smoothing with half the width of the formation, the
 * largest sideways spread of the slots from their middle.
 * --------------------------------------------------------------------------------------------- */
void begin_formation_smoothing(Squad *squad, const float *slots, V2 facing) {
    int member_count = squad->unit_count;
    int mover_class = units[squad->first_unit_index].kind->mover_class;
    float max_radius;
    float full_speed_radius;
    V2 center;
    measure_members(squad, &max_radius, &full_speed_radius, &center);
    V2 end = squad->route[squad->route_count - 1];
    float min_side = LARGE_VALUE;
    float max_side = -LARGE_VALUE;
    for (int slot_index = 0; slot_index < member_count; slot_index++) {
        float slot_side = get_slot_offset(slots, slot_index, end, facing).x;
        min_side = fminf(min_side, slot_side);
        max_side = fmaxf(max_side, slot_side);
    }
    float half_width = (max_side - min_side) / 2;
    begin_smoothing(squad, max_radius, half_width, full_speed_radius, mover_class);
}

/* ------------------------------------------------------------------------------------------------
 * `begin_formation_set_up` runs `prepare_formation_route`, and then `begin_formation_smoothing`
 * with `--smooth on`.
 * --------------------------------------------------------------------------------------------- */
bool begin_formation_set_up(Squad *squad, const float *slots, V2 facing) {
    prepare_formation_route(squad, slots, facing);
    if (!is_smooth_on)
        return false;
    begin_formation_smoothing(squad, slots, facing);
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `finish_formation_set_up` measures the route and resets the state of the anchor. To match
 * members to slots, it compares the shape of the members at the start of the route with the
 * shape of the slots at its end, each taken as offsets from its own mean in route coordinates,
 * so that the members keep their arrangement. The cost of a match is the sum of the squared
 * differences, and `find_best_match` finds the cheapest match. Each member then takes `slot`,
 * `place_offset`, `file_rank` and `file_distance`, and a reset movement state. Last, lanes are
 * laid out and `start_offset` is found, so that each place starts where its member stands.
 * --------------------------------------------------------------------------------------------- */
void finish_formation_set_up(Squad *squad, const float *slots, V2 facing) {
    int member_count = squad->unit_count;
    Unit *members = &units[squad->first_unit_index];
    float mean_weight = 1.0f / member_count;
    float max_radius;
    float full_speed_radius;
    V2 center;
    measure_members(squad, &max_radius, &full_speed_radius, &center);

    /* Measure the route, and put the anchor at its start. */
    squad->route_distance[0] = 0;
    for (int point_index = 1; point_index < squad->route_count; point_index++) {
        V2 leg = sub(squad->route[point_index], squad->route[point_index - 1]);
        squad->route_distance[point_index] = squad->route_distance[point_index - 1] + len(leg);
    }
    squad->route_length = squad->route_distance[squad->route_count - 1];
    squad->anchor_distance = 0;

    /* A squad that searches again around a parked squad (`is_replanning`) is on its way. With
     * `--keepspeed on`, its anchor keeps its speed: the members steer by that speed in the tick of
     * the new route, before `advance_anchor` sets it again, and with speed 0 they would brake. */
    bool is_speed_kept = is_keep_speed_on && squad->is_replanning;
    if (!is_speed_kept)
        squad->anchor_speed = 0;
    squad->waited_seconds = 0;
    squad->ignore_lag_seconds = 0;
    squad->reassign_seconds = 0;
    squad->reassign_count = 0;
    squad->has_final_match = false;
    squad->final_facing = facing;
    squad->full_speed_radius = full_speed_radius;

    /* Find the offsets of the slots from the end of the route, with x to the right of `facing`,
     * and the offsets of the members from the start of the route, with x to the right of its
     * first segment. Entry k of `slot_offsets` belongs to slot k, and entry k of
     * `member_offsets` to member k. */
    V2 end = squad->route[squad->route_count - 1];
    V2 start_direction = facing;
    if (squad->route_count > 1)
        start_direction = sub(squad->route[1], squad->route[0]);
    start_direction = get_direction(start_direction, 0, facing);
    V2 slot_offsets[MAX_UNITS];
    V2 member_offsets[MAX_UNITS];
    V2 mean_slot_offset = v2(0, 0);
    V2 mean_member_offset = v2(0, 0);
    for (int member_index = 0; member_index < member_count; member_index++) {
        slot_offsets[member_index] = get_slot_offset(slots, member_index, end, facing);
        mean_slot_offset = add(mean_slot_offset, slot_offsets[member_index]);
        V2 member_from_start = sub(members[member_index].position, squad->route[0]);
        member_offsets[member_index] = get_frame_offset(member_from_start, start_direction);
        mean_member_offset = add(mean_member_offset, member_offsets[member_index]);
    }
    mean_slot_offset = mul(mean_slot_offset, mean_weight);
    mean_member_offset = mul(mean_member_offset, mean_weight);

    /* Match members to slots so that the members keep their arrangement. The cost compares the
     * shapes, the offsets less their means, by the squared differences. */
    float costs[MAX_UNITS][MAX_UNITS];
    for (int member_index = 0; member_index < member_count; member_index++) {
        V2 member_shape = sub(member_offsets[member_index], mean_member_offset);
        for (int slot_index = 0; slot_index < member_count; slot_index++) {
            V2 slot_shape = sub(slot_offsets[slot_index], mean_slot_offset);
            costs[member_index][slot_index] = length_squared(sub(member_shape, slot_shape));
        }
    }
    int best_match[MAX_UNITS];
    float best_cost = LARGE_SQUARED_VALUE;
    find_best_match(costs, member_count, &best_cost, best_match);

    /* Give each member its slot and place, and reset its movement state. Its single-file rank is
     * the number of places ahead of it: those further forward, and those level with it that
     * belong to earlier members. */
    float file_spacing = FILE_SPACING_RADII * max_radius + FILE_SPACING_GAP_METERS;
    for (int member_index = 0; member_index < member_count; member_index++) {
        Unit *member = &members[member_index];
        int slot_index = best_match[member_index];
        member->slot = get_pair_point(slots, slot_index);
        member->place_offset = slot_offsets[slot_index];

        /* Count the places ahead of this one. */
        float place_forward = member->place_offset.y;
        int rank = 0;
        for (int other_index = 0; other_index < member_count; other_index++) {
            float other_forward = slot_offsets[best_match[other_index]].y;
            bool is_ahead = other_forward > place_forward;
            bool is_level = other_forward == place_forward;
            bool is_level_and_before = is_level && other_index < member_index;
            if (is_ahead || is_level_and_before)
                rank++;
        }
        member->file_distance = rank * file_spacing;
        member->file_rank = rank;

        /* Reset the movement state. */
        member->route_progress = 0;
        member->is_stopped = false;
        member->waypoint_count = 0;
        reset_detour(member);
        member->is_reorienting = false;
        member->reach = member->kind->radius;
        if (member->reach < MIN_FORMATION_REACH_METERS)
            member->reach = MIN_FORMATION_REACH_METERS;
        member->facing = facing;
    }

    /* Lay out the lanes. Then find the start offsets, which make each place start where its
     * member stands. */
    squad->is_single_file = false;
    squad->is_reforming = false;
    if (are_lanes_on)
        set_up_lanes(squad, max_radius);
    for (int member_index = 0; member_index < member_count; member_index++) {
        Unit *member = &members[member_index];
        V2 forward;
        V2 place = get_place(squad, member, &forward);
        member->start_offset = sub(member->position, place);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `compute_place_cost` returns the cost of a member at route coordinates `member` taking the
 * place at route coordinates `place`: the squared forward difference plus `REASSIGN_SIDE_WEIGHT`
 * times the squared sideways difference.
 * --------------------------------------------------------------------------------------------- */
static float compute_place_cost(V2 member, V2 place) {
    float forward_difference = member.y - place.y;
    float side_difference = member.x - place.x;
    float forward_squared = forward_difference * forward_difference;
    float weighted_side_squared = REASSIGN_SIDE_WEIGHT * side_difference * side_difference;
    return forward_squared + weighted_side_squared;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_match_coordinates` finds the route coordinates of each member and of the place that
 * it holds now. `member_coordinates[k]` and `place_coordinates[k]` belong to member k of the
 * squad.
 * --------------------------------------------------------------------------------------------- */
static void measure_match_coordinates(const Squad *squad, V2 *member_coordinates,
                                      V2 *place_coordinates) {
    const Unit *members = &units[squad->first_unit_index];
    for (int member_index = 0; member_index < squad->unit_count; member_index++) {
        const Unit *member = &members[member_index];
        V2 forward;
        V2 place = get_place(squad, member, &forward);
        member_coordinates[member_index] = get_route_coordinates(squad, member->position);
        place_coordinates[member_index] = get_route_coordinates(squad, place);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `hand_over_places` gives each member k the place that member `best_match[k]` holds now. A place
 * carries its `place_offset`, its single-file rank and distance, and its slot, and it counts as
 * one change in `reassign_count`.
 * --------------------------------------------------------------------------------------------- */
static void hand_over_places(Squad *squad, const int *best_match) {
    typedef struct {
        V2 place_offset;
        V2 slot;
        float file_distance;
        int file_rank;
    } Place;
    Place places[MAX_UNITS];
    Unit *members = &units[squad->first_unit_index];
    int member_count = squad->unit_count;
    for (int member_index = 0; member_index < member_count; member_index++) {
        const Unit *member = &members[member_index];
        places[member_index] = (Place){
            .place_offset = member->place_offset,
            .slot = member->slot,
            .file_distance = member->file_distance,
            .file_rank = member->file_rank,
        };
    }
    for (int member_index = 0; member_index < member_count; member_index++) {
        Unit *member = &members[member_index];
        int place_index = best_match[member_index];
        member->place_offset = places[place_index].place_offset;
        member->slot = places[place_index].slot;
        member->file_distance = places[place_index].file_distance;
        member->file_rank = places[place_index].file_rank;
    }
    squad->reassign_count++;
}

/* ------------------------------------------------------------------------------------------------
 * `compute_catch_up_cost` returns how far a member at route coordinates `member` must catch up to
 * reach the place at route coordinates `place`. When the place lies ahead of the member, that is
 * the straight distance to it. When the place lies level with the member or behind it, only the
 * sideways difference counts, because a tank brakes almost at once but gains speed slowly, so
 * dropping back costs hardly any time.
 * --------------------------------------------------------------------------------------------- */
static float compute_catch_up_cost(V2 member, V2 place) {
    float forward_difference = place.y - member.y;
    float side_difference = place.x - member.x;
    if (forward_difference <= 0)
        return fabsf(side_difference);
    return hypotf(forward_difference, side_difference);
}

/* ------------------------------------------------------------------------------------------------
 * `find_reform_match` tries every permutation of the places, and it finds the match whose largest
 * cost is smallest. The squad waits for its slowest member, so that cost sets the time that the
 * squad needs to get back into its formation. Among matches with the same largest cost, it takes
 * the one with the smallest sum of squared costs, which keeps the other ways short.
 * Inputs: `costs[member_index][place_index]` is the cost of the member taking the place. The squad
 *   has at most `MAX_MATCHED_MEMBERS` members.
 * Result: `best_match[member_index]` is the place of each member in the best match, and
 *   `best_max_cost` is its largest cost.
 * --------------------------------------------------------------------------------------------- */
static void find_reform_match(float costs[][MAX_UNITS], int member_count, int *best_match,
                              float *best_max_cost) {
    int permutation[MAX_UNITS];
    for (int member_index = 0; member_index < member_count; member_index++)
        permutation[member_index] = member_index;
    float best_sum_squared = LARGE_VALUE;
    *best_max_cost = LARGE_VALUE;
    do {
        /* Find the largest cost of the match and the sum of its squared costs. */
        float max_cost = 0;
        float sum_squared = 0;
        for (int member_index = 0; member_index < member_count; member_index++) {
            float cost = costs[member_index][permutation[member_index]];
            max_cost = fmaxf(max_cost, cost);
            sum_squared += cost * cost;
        }

        /* Keep it when its largest cost is smaller, or the same with a smaller sum. */
        bool is_better = max_cost < *best_max_cost;
        bool is_tie = max_cost == *best_max_cost;
        if (is_better || (is_tie && sum_squared < best_sum_squared)) {
            *best_max_cost = max_cost;
            best_sum_squared = sum_squared;
            memcpy(best_match, permutation, sizeof(int) * member_count);
        }
    } while (advance_permutation(permutation, member_count));
}

/* ------------------------------------------------------------------------------------------------
 * `rematch_for_reform` matches the members to the places with `--reformmatch on`, so that the
 * squad gets back into its formation soonest after it leaves single file. It takes the costs from
 * `compute_catch_up_cost` and the match from `find_reform_match`.
 * Inputs: With `is_forced`, at the tick when the squad leaves single file, the best match always
 *   wins. Later, it wins only when its largest cost is at least `REASSIGN_MIN_SAVING_FRACTION`
 *   smaller than the largest cost of the current match, so that places do not flicker.
 * --------------------------------------------------------------------------------------------- */
static void rematch_for_reform(Squad *squad, bool is_forced) {
    /* Find the catch-up cost of each member taking each place. */
    int member_count = squad->unit_count;
    V2 member_coordinates[MAX_UNITS];
    V2 place_coordinates[MAX_UNITS];
    measure_match_coordinates(squad, member_coordinates, place_coordinates);
    float costs[MAX_UNITS][MAX_UNITS];
    for (int member_index = 0; member_index < member_count; member_index++) {
        V2 member_at = member_coordinates[member_index];
        for (int place_index = 0; place_index < member_count; place_index++) {
            V2 place_at = place_coordinates[place_index];
            costs[member_index][place_index] = compute_catch_up_cost(member_at, place_at);
        }
    }

    /* Find the best match, and hand the places over when it wins and changes anything. */
    float current_max_cost = 0;
    for (int member_index = 0; member_index < member_count; member_index++)
        current_max_cost = fmaxf(current_max_cost, costs[member_index][member_index]);
    int best_match[MAX_UNITS];
    float best_max_cost = 0;
    find_reform_match(costs, member_count, best_match, &best_max_cost);
    float min_saving = REASSIGN_MIN_SAVING_FRACTION * current_max_cost;
    bool is_saving_enough = best_max_cost <= current_max_cost - min_saving;
    bool is_change = false;
    for (int member_index = 0; member_index < member_count; member_index++)
        is_change = is_change || best_match[member_index] != member_index;
    if (is_change && (is_forced || is_saving_enough))
        hand_over_places(squad, best_match);
}

/* ------------------------------------------------------------------------------------------------
 * `reassign_places` matches the members to the places again, every `REASSIGN_INTERVAL_SECONDS`
 * once the anchor is past `START_BLEND_METERS`. It compares their positions in route coordinates
 * and takes the match with the smallest sum of `compute_place_cost`. So a unit that got ahead
 * takes a front place, and a unit that ended up on the other side keeps a place on that side
 * instead of crossing back. Where both cannot hold, keeping its side wins, because catching up or
 * dropping back is only a change of speed. The match changes only when it saves at least
 * `REASSIGN_MIN_SAVING_FRACTION` of the cost and at least `REASSIGN_MIN_SAVING_SQUARED`, so that
 * places do not flicker. A place carries its `place_offset`, its single-file rank and distance,
 * and its slot.
 * With `--endstop on`, the members are matched to the final slots once, at the first tick of the
 * last `END_BLEND_METERS` of the route, by the squared straight distance from each member to each
 * slot, so that the ways of the members to their slots do not cross. That match then holds to the
 * end, because a member that gets a new slot on its end leg stops and turns around.
 * While the squad reforms after single file (`is_reforming`), `rematch_for_reform` matches them
 * instead, by the time that the squad needs to get back into its formation.
 * Edge case: Nothing changes with `--reassign off`, or in squads of one member or of more than
 *   `MAX_MATCHED_MEMBERS`.
 * --------------------------------------------------------------------------------------------- */
static void reassign_places(Squad *squad) {
    /* Wait for the next match. The final match does not wait. */
    int member_count = squad->unit_count;
    float remaining_distance = squad->route_length - squad->anchor_distance;
    bool is_end_leg = is_end_stop_on && remaining_distance <= END_BLEND_METERS;
    squad->reassign_seconds -= TICK_SECONDS;
    bool is_too_early = squad->anchor_distance < START_BLEND_METERS || squad->reassign_seconds > 0;
    if (is_end_leg)
        is_too_early = squad->has_final_match;
    bool is_matchable = member_count >= 2 && member_count <= MAX_MATCHED_MEMBERS;
    if (!is_reassign_on || !is_matchable || is_too_early)
        return;
    squad->reassign_seconds = REASSIGN_INTERVAL_SECONDS;

    /* While the squad reforms after single file with `--reformmatch on`, match by the time to get
     * back into formation instead. */
    if (squad->is_reforming && !is_end_leg) {
        rematch_for_reform(squad, false);
        return;
    }

    /* Find the cost of each member taking each place: on the end leg from the squared distance to
     * the slot of the place, before it from their route coordinates. */
    Unit *members = &units[squad->first_unit_index];
    float costs[MAX_UNITS][MAX_UNITS];
    if (is_end_leg) {
        for (int member_index = 0; member_index < member_count; member_index++) {
            V2 member_at = members[member_index].position;
            for (int place_index = 0; place_index < member_count; place_index++) {
                V2 to_slot = sub(members[place_index].slot, member_at);
                costs[member_index][place_index] = length_squared(to_slot);
            }
        }
    } else {
        V2 member_coordinates[MAX_UNITS];
        V2 place_coordinates[MAX_UNITS];
        measure_match_coordinates(squad, member_coordinates, place_coordinates);
        for (int member_index = 0; member_index < member_count; member_index++) {
            V2 member_at = member_coordinates[member_index];
            for (int place_index = 0; place_index < member_count; place_index++) {
                V2 place_at = place_coordinates[place_index];
                costs[member_index][place_index] = compute_place_cost(member_at, place_at);
            }
        }
    }

    /* Find the best match, and keep the current one unless the best one saves enough. The final
     * match takes the best one whatever it saves. */
    float current_cost = 0;
    for (int member_index = 0; member_index < member_count; member_index++)
        current_cost += costs[member_index][member_index];
    float best_cost = current_cost;
    int best_match[MAX_UNITS];
    find_best_match(costs, member_count, &best_cost, best_match);
    float fraction_saving = REASSIGN_MIN_SAVING_FRACTION * current_cost;
    float min_saving = fmaxf(REASSIGN_MIN_SAVING_SQUARED, fraction_saving);
    if (is_end_leg)
        squad->has_final_match = true;
    else if (best_cost > current_cost - min_saving)
        return;
    hand_over_places(squad, best_match);
}

/* ------------------------------------------------------------------------------------------------
 * `measure_lag` returns how far the member furthest behind its target place is behind it, along
 * the forward direction of the place, or 0 when no member is behind its place.
 * --------------------------------------------------------------------------------------------- */
static float measure_lag(const Squad *squad) {
    float lag = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        V2 forward;
        V2 target = get_target_place(squad, unit, &forward);
        V2 to_target = sub(target, unit->position);
        lag = fmaxf(lag, dot(to_target, forward));
    }
    return lag;
}

/* ------------------------------------------------------------------------------------------------
 * `advance_anchor` finds the speed of the anchor as the lowest top speed of the members, times a
 * factor for the lag of the members, times a factor for the turn when `--turnslow on`, and at
 * most the stopping speed at the end of the route. A long lag slows it further, and a long wait
 * makes it ignore the lag for a while. It applies `coordination_speed_cap` last, so that giving
 * way to another squad does not count as waiting. The decision for single file runs before the
 * speed, because the turn is measured from the rear of the single file.
 * --------------------------------------------------------------------------------------------- */
void advance_anchor(Squad *squad) {
    reassign_places(squad);

    /* Find the lowest top speed and the lowest friction among the members, and how far the places
     * reach from the anchor: `file_depth` back in single file, and the smallest and largest
     * forward offsets side by side. */
    float min_max_speed = LARGE_VALUE;
    float min_friction = LARGE_VALUE;
    float file_depth = 0;
    float min_forward_offset = 0;
    float max_forward_offset = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        min_max_speed = fminf(min_max_speed, unit->kind->max_speed);
        min_friction = fminf(min_friction, unit->kind->friction);
        file_depth = fmaxf(file_depth, get_file_distance(squad, unit));
        min_forward_offset = fminf(min_forward_offset, unit->place_offset.y);
        max_forward_offset = fmaxf(max_forward_offset, unit->place_offset.y);
    }

    /* Drive in single file while the lanes do not fit side by side somewhere from
     * `REAR_MARGIN_METERS` before the rear place to that far past the front place. Leave single
     * file only once they fit from that far before the rear of the single file to
     * `FILE_LEAVE_AHEAD_METERS` past the front place. */
    float anchor_distance = squad->anchor_distance;
    float file_rear = anchor_distance - file_depth - REAR_MARGIN_METERS;
    bool was_single_file = squad->is_single_file;
    if (are_lanes_on) {
        float front = anchor_distance + max_forward_offset;
        if (!squad->is_single_file) {
            float rear = anchor_distance + min_forward_offset - REAR_MARGIN_METERS;
            squad->is_single_file = !do_lanes_fit(squad, rear, front + REAR_MARGIN_METERS);
        } else {
            float file_front = front + FILE_LEAVE_AHEAD_METERS;
            squad->is_single_file = !do_lanes_fit(squad, file_rear, file_front);
        }
        if (squad->is_file_forced)
            squad->is_single_file = true;
    }

    /* With `--reformmatch on`, a squad that leaves single file starts to reform, and its members
     * are matched to the places at once by the time to get back into formation. The regular
     * matches of `reassign_places` ran before the switch, on the places of the single file. */
    bool has_left_file = was_single_file && !squad->is_single_file;
    if (is_reform_match_on && has_left_file) {
        squad->is_reforming = true;
        int member_count = squad->unit_count;
        bool is_matchable = member_count >= 2 && member_count <= MAX_MATCHED_MEMBERS;
        float distance_to_end = squad->route_length - anchor_distance;
        bool is_end_leg = is_end_stop_on && distance_to_end <= END_BLEND_METERS;
        if (is_reassign_on && is_matchable && !is_end_leg)
            rematch_for_reform(squad, true);
    }

    /* Measure the turn between the direction of the route at `file_rear` and its direction
     * `TURN_LOOK_AHEAD_METERS` past the anchor, and the lag of the members.
     * TODO: confirm why the turn is measured from the rear of the single file also while the
     * squad drives side by side. */
    V2 rear_direction;
    V2 ahead_direction;
    get_route_point(squad, file_rear, &rear_direction);
    get_route_point(squad, anchor_distance + TURN_LOOK_AHEAD_METERS, &ahead_direction);
    float lag = measure_lag(squad);
    if (squad->is_reforming && (lag <= LAG_WAIT_METERS || squad->is_single_file))
        squad->is_reforming = false;

    /* Set the speed: the lowest top speed while everyone keeps up, and up to
     * `LAG_SLOWDOWN_FRACTION` less as the lag grows to `LAG_SLOWDOWN_METERS`, so that members can
     * catch up. With `--turnslow on`, also drive slower through sharp turns, until the whole
     * formation is around them. With `--turnslow off`, the members slow down for bends by their
     * own bend limits, and the anchor slows for turns only through the lag. Brake to a stop at
     * the end of the route with `END_DECELERATION_FRACTION` of the lowest friction:
     * `stopping_speed^2 = 2 * deceleration * remaining_distance`. */
    float lag_fraction = fminf(1, lag / LAG_SLOWDOWN_METERS);
    float lag_factor = 1 - LAG_SLOWDOWN_FRACTION * lag_fraction;
    float turn_factor = 1;
    if (is_turn_slowdown_on) {
        float turn_cosine = fmaxf(0, dot(rear_direction, ahead_direction));
        turn_factor = TURN_SPEED_FLOOR + TURN_SPEED_RANGE * turn_cosine;
    }
    float speed = min_max_speed * lag_factor * turn_factor;
    float remaining_distance = squad->route_length - anchor_distance;
    float twice_deceleration = 2 * END_DECELERATION_FRACTION * min_friction;
    float stopping_speed = sqrtf(twice_deceleration * remaining_distance);
    speed = fminf(speed, stopping_speed);

    /* Wait for laggards. Past a lag of `LAG_WAIT_METERS`, slow down further, but not below
     * `CRAWL_FRACTION` of the speed, and stop only past `LAG_STOP_METERS`. When the anchor is held
     * still (slower than `ANCHOR_STILL_SPEED`) for longer than `STUCK_WAIT_SECONDS`, a member is
     * likely stuck. Then the anchor moves on and ignores the lag for `IGNORE_LAG_SECONDS`. */
    squad->ignore_lag_seconds -= TICK_SECONDS;
    if (squad->ignore_lag_seconds <= 0 && lag > LAG_WAIT_METERS) {
        float wait_factor = 0;
        if (lag <= LAG_STOP_METERS) {
            float slowdown = 1 - (lag - LAG_WAIT_METERS) / LAG_WAIT_RANGE_METERS;
            wait_factor = fmaxf(CRAWL_FRACTION, slowdown);
        }
        speed *= wait_factor;
    }
    if (speed < ANCHOR_STILL_SPEED)
        squad->waited_seconds += TICK_SECONDS;
    else
        squad->waited_seconds = 0;
    if (squad->waited_seconds > STUCK_WAIT_SECONDS) {
        squad->ignore_lag_seconds = IGNORE_LAG_SECONDS;
        squad->waited_seconds = 0;
    }

    /* Keep to the cap from coordination.c, and move the anchor. The cap comes after the wait
     * timer, so that giving way to another squad does not count as waiting for laggards. */
    speed = fminf(speed, squad->coordination_speed_cap);
    squad->anchor_speed = speed;
    squad->anchor_distance = fminf(squad->route_length, anchor_distance + speed * TICK_SECONDS);
}
