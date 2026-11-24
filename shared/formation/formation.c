// Slots and the anchor of formation following. The anchor is a point that moves along the route
// at the speed of the squad, and each squad member has a slot, a point of the formation that moves
// with the anchor. `slot_position` places the slot relative to the anchor: `x` to the right of the
// formation direction and `y` forward along it. In column formation, the slots lie on the route
// behind the anchor, one column gap apart.
//
// The set-up of a squad (`set_up_formation`, or its three steps, which game/squad.c can spread
// over ticks, and which the simulator and the mod can run with the smoothing on a worker thread)
// prepares and smooths the route, assigns the final parking positions to the squad members, and
// lays out the lanes. Each tick, `advance_anchor` swaps slots when that
// shortens the ways of the squad members (`--formation`), switches between square and column
// formation, and moves the anchor at the top speed of the squad, slowed for lagging squad members.
//
// The slots blend from where the squad members stand into the formation over the first
// `START_BLEND_METERS` of anchor travel, and into the shape of the final parking positions over
// the last `END_BLEND_METERS` of the route, the end blend.
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "sim.h"
#include "formation.h"

#define LARGE_SQUARED_VALUE 1e30f              // start value of searches for a smallest square
#define START_BLEND_METERS 25.0f               // anchor travel over which start offsets fade
#define MAX_SWAP_MEMBERS 8                     // most squad members for a permutation search
#define COLUMN_GAP_EXTRA_METERS 2.0f           // column gap on top of those radii
#define COLUMN_HEADWAY_SECONDS 0.3f            // column gap growth per anchor speed, with lanes
#define MIN_FORMATION_REACH_METERS 2.0f        // smallest reach in formation following
#define COORDINATES_BEHIND_METERS 80.0f        // route coordinates: search before the anchor
#define COORDINATES_AHEAD_METERS 40.0f         // route coordinates: search past the anchor
#define SWAP_INTERVAL_SECONDS 0.5f             // time between two slot swap checks
#define FAST_SWAP_INTERVAL_SECONDS 0.25f       // that time with `--fastswap`
#define SWAP_SIDE_WEIGHT 4.0f                  // weight of sideways differences in swap costs
#define SWAP_MIN_SAVING_FRACTION 0.1f          // smallest saving of a swap, share of the cost
#define SWAP_MIN_SAVING_SQUARED 10.0f          // smallest saving of a swap, in square meters

#define LAG_SLOWDOWN_FRACTION 0.1f             // largest anchor slowdown share from lag
#define LAG_SLOWDOWN_METERS 4.0f               // lag at which that slowdown is full
#define REAR_MARGIN_METERS 10.0f               // margin of the column checks past both ends
#define COLUMN_LEAVE_AHEAD_METERS 20.0f        // lanes fit this far ahead to leave a column
#define END_DECELERATION_FRACTION 0.5f         // friction share of anchor braking at the end
#define LAG_WAIT_RANGE_METERS 20.0f            // lag per full share of the waiting slowdown
#define CRAWL_FRACTION 0.25f                   // lowest speed share of a waiting anchor
#define LAG_STOP_METERS 60.0f                  // larger lags stop the anchor
#define ANCHOR_STILL_SPEED 0.2f                // slower anchors count as still
#define STUCK_WAIT_SECONDS 10.0f               // longest wait for squad members that lag
#define IGNORE_LAG_SECONDS 5.0f                // time without waiting after that
#define REORDER_MIN_SPEED 2.0f                 // slower squads count as standing at a new order
#define LEAD_IN_RADIUS_FRACTION 0.7f        // lead-in radius, share of the current-speed radius


// `blend` returns the mix of `from` and `to` with the share `weight` of `to`.
static V2 blend(V2 from, V2 to, float weight) {
    V2 from_part = mul(from, 1 - weight);
    V2 to_part = mul(to, weight);
    return add(from_part, to_part);
}


// `get_end_weight` returns how far the route position `distance` lies into the end blend: 0 at its
// start and 1 at the route end. Before the end blend, the result is negative.
static float get_end_weight(const Squad *squad, float distance) {
    return 1 - (squad->route_length - distance) / END_BLEND_METERS;
}


// `get_formation_frame` returns the route point at `distance`, and in `forward`, the formation
// direction there. On the end blend, `forward` turns toward the final facing of the squad.
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


// `get_anchor_to_slot_distance` returns how far the slot of a squad member lies behind the anchor
// in column formation. With lanes, the column gap grows with the anchor speed.
float get_anchor_to_slot_distance(const Squad *squad, const Unit *unit) {
    float headway = 0;
    if (are_lanes_on)
        headway = unit->column_position * COLUMN_HEADWAY_SECONDS * squad->anchor_speed;
    return unit->anchor_to_slot_distance + headway;
}


// `get_slot_distance` returns the route position of the slot of a squad member.
float get_slot_distance(const Squad *squad, const Unit *unit) {
    if (squad->is_in_column)
        return squad->anchor_distance - get_anchor_to_slot_distance(squad, unit);
    return squad->anchor_distance + unit->slot_position.y;
}


// `get_slot` returns the slot of a squad member, and in `forward`, the formation direction at the
// slot. With lanes, the slot is its lane point at the slot route position, which blends into the
// shape of the final parking positions around the anchor on the end blend.
static V2 get_slot(const Squad *squad, const Unit *unit, V2 *forward) {
    // Without lanes, a slot behind blocking terrain, as seen from the anchor, moves to the route
    // behind the anchor, as in column formation.
    V2 anchor_point = get_formation_frame(squad, squad->anchor_distance, forward);
    V2 shape_offset = get_map_offset(unit->slot_position, *forward);
    V2 shape_slot = add(anchor_point, shape_offset);
    if (!are_lanes_on) {
        if (!is_line_clear(anchor_point, shape_slot, unit->unit_type->terrain_class)) {
            V2 unused;
            float d_column = squad->anchor_distance - unit->anchor_to_slot_distance;
            shape_slot = get_route_point(squad, d_column, &unused);
        }
        return shape_slot;
    }

    // The end weight at the anchor sets the blend for every slot of the squad.
    float d = get_slot_distance(squad, unit);        // route position of the slot
    V2 lane_forward = get_formation_direction(squad, d);
    V2 lane_slot = get_lane_point(squad, unit, d);
    float end_weight = clamp_float(get_end_weight(squad, squad->anchor_distance), 0, 1);
    V2 blended = blend(lane_forward, *forward, end_weight);
    *forward = get_direction(blended, LENGTH_SQUARED_EPSILON, lane_forward);
    return blend(lane_slot, shape_slot, end_weight);
}


// `get_target_slot` returns the point that a squad member drives toward: its slot, plus its start
// offset, which fades out over the first `START_BLEND_METERS` of anchor travel.
V2 get_target_slot(const Squad *squad, const Unit *unit, V2 *forward) {
    V2 slot = get_slot(squad, unit, forward);
    float start_weight = fmaxf(0, 1 - squad->anchor_distance / START_BLEND_METERS);
    return add(slot, mul(unit->start_offset, start_weight));
}


// `project_onto_route` returns the route position of the point of the route closest to `point`,
// among the route positions from `from_distance` to `to_distance`.
float project_onto_route(const Squad *squad, V2 point, float from_distance, float to_distance) {
    float best_squared = LARGE_SQUARED_VALUE;        // smallest squared distance
    float best = from_distance;                      // route position of the closest point
    for (int end_index = 1; end_index < squad->route_count; end_index++) {
        // Only route segments that overlap the range count.
        float d_start = squad->route_distance[end_index - 1];        // segment start
        float d_end = squad->route_distance[end_index];              // segment end
        float length = d_end - d_start;
        bool is_outside_range = d_end < from_distance || d_start > to_distance;
        if (is_outside_range || length <= 0)
            continue;

        // The fraction of the projection along the segment stays inside the range and the segment.
        V2 segment_start = squad->route[end_index - 1];
        V2 segment = sub(squad->route[end_index], segment_start);
        V2 from_start = sub(point, segment_start);
        float fraction = dot(from_start, segment) / (length * length);
        float from_fraction = (from_distance - d_start) / length;
        float to_fraction = (to_distance - d_start) / length;
        fraction = fmaxf(fmaxf(fraction, from_fraction), 0);
        fraction = fminf(fminf(fraction, to_fraction), 1);
        V2 closest = add(segment_start, mul(segment, fraction));
        float d_squared = length_squared(sub(point, closest));
        if (d_squared < best_squared) {
            best_squared = d_squared;
            best = d_start + fraction * length;
        }
    }
    return best;
}


// `get_route_coordinates` returns the route coordinates of `point` near the anchor: `x` is the
// sideways offset from the route, to the right of the formation direction, and `y` is the route
// position plus the offset along the formation direction.
static V2 get_route_coordinates(const Squad *squad, V2 point) {
    float from = fmaxf(0, squad->anchor_distance - COORDINATES_BEHIND_METERS);
    float to = squad->anchor_distance + COORDINATES_AHEAD_METERS;
    float d = project_onto_route(squad, point, from, to);        // route position of the point

    V2 unused;
    V2 forward = get_formation_direction(squad, d);
    V2 route_point = get_extended_route_point(squad, d, &unused);
    V2 from_route = sub(point, route_point);
    float along = dot(from_route, forward);
    float sideways = dot(from_route, get_right(forward));
    return v2(sideways, d + along);
}


// `swap_ints` swaps the values of two integers.
static void swap_ints(int *first, int *second) {
    int kept = *first;
    *first = *second;
    *second = kept;
}


// `advance_permutation` turns `permutation` into the next permutation in lexicographic order. It
// returns false after the last one, the descending order.
static bool advance_permutation(int *permutation, int count) {
    // The pivot is the last entry that is smaller than the entry after it.
    int pivot_index = count - 2;
    while (pivot_index >= 0 && permutation[pivot_index] > permutation[pivot_index + 1])
        pivot_index--;
    if (pivot_index < 0)
        return false;

    // The pivot swaps with the last larger entry, and the entries after the pivot reverse.
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


// `find_best_permutation` finds the permutation with the smallest sum of `costs[member][slot]`
// that is smaller than `best_cost` on input. A squad of more than `MAX_SWAP_MEMBERS` squad members
// tests only the identity, because the number of permutations grows too fast.
static void find_best_permutation(float costs[][MAX_MEMBERS], int member_count, float *best_cost,
                                  int *best_permutation) {
    int permutation[MAX_MEMBERS];
    for (int member_index = 0; member_index < member_count; member_index++) {
        permutation[member_index] = member_index;
        best_permutation[member_index] = member_index;
    }

    // A sum stops as soon as it reaches the best cost so far.
    bool is_swappable = member_count <= MAX_SWAP_MEMBERS;
    do {
        float cost = 0;
        int member_index = 0;
        while (member_index < member_count && cost < *best_cost) {
            cost += costs[member_index][permutation[member_index]];
            member_index++;
        }
        if (cost < *best_cost) {
            *best_cost = cost;
            memcpy(best_permutation, permutation, sizeof(int) * member_count);
        }
    } while (is_swappable && advance_permutation(permutation, member_count));
}


// `set_up_formation` runs the whole set-up of a squad in one call: the route preparation, the
// route smoothing, and the assignment of slots and lanes.
void set_up_formation(Squad *squad, const float *parking_positions, V2 facing) {
    bool is_smoothing = begin_formation_set_up(squad, parking_positions, facing);
    if (is_smoothing)
        run_smoothing(INT_MAX);
    finish_formation_set_up(squad, parking_positions, facing);
}


// `get_full_speed_min_radius` returns the radius at which a unit turns at its current top speed
// with `BEND_SPEED_MARGIN` of its turn rate.
static float get_full_speed_min_radius(const Unit *unit) {
    // A unit at the speed `v` and the turn rate `w` drives a circle of radius `v / w`.
    float w = BEND_SPEED_MARGIN * unit->unit_type->turn_rate;        // usable turn rate
    return unit->top_speed / w;
}


// `measure_members` measures the squad members of a squad: the largest radius, the full-speed
// minimum radius, and the mean position. The full-speed minimum radius of the squad is the
// largest one among its squad members.
static void measure_members(const Squad *squad, float *max_radius, float *full_speed_min_radius,
                            V2 *center) {
    int member_count = squad->unit_count;
    const Unit *members = &units[squad->first_unit_index];
    float mean_weight = 1.0f / member_count;
    *max_radius = 0;
    *full_speed_min_radius = 0;
    *center = v2(0, 0);
    for (int member_index = 0; member_index < member_count; member_index++) {
        const Unit *member = &members[member_index];
        *max_radius = fmaxf(*max_radius, member->unit_type->radius);
        float r = get_full_speed_min_radius(member);
        *full_speed_min_radius = fmaxf(*full_speed_min_radius, r);
        *center = add(*center, mul(member->position, mean_weight));
    }
}


// `measure_motion` returns the mean velocity of the squad members of a squad and, in `turn_radius`,
// the largest radius at which a squad member turns at its current speed with `BEND_SPEED_MARGIN` of
// its turn rate.
static V2 measure_motion(const Squad *squad, float *turn_radius) {
    const Unit *members = &units[squad->first_unit_index];
    float mean_weight = 1.0f / squad->unit_count;
    V2 mean_velocity = v2(0, 0);
    *turn_radius = 0;
    for (int member_index = 0; member_index < squad->unit_count; member_index++) {
        const Unit *member = &members[member_index];
        mean_velocity = add(mean_velocity, mul(member->velocity, mean_weight));
        float w = BEND_SPEED_MARGIN * member->unit_type->turn_rate;        // usable turn rate
        *turn_radius = fmaxf(*turn_radius, len(member->velocity) / w);
    }
    return mean_velocity;
}


// `get_reorder_speed` returns the speed of a squad that gets a new order while it moves
// (`--smoothreorder`): the speed of the mean velocity of its squad members, at most the top speed
// of the squad. Below `REORDER_MIN_SPEED`, at a re-route, and without the switch, it returns 0.
static float get_reorder_speed(const Squad *squad) {
    if (!is_smooth_reorder_on || squad->is_rerouting)
        return 0;
    float turn_radius;
    float speed = len(measure_motion(squad, &turn_radius));
    if (speed < REORDER_MIN_SPEED)
        return 0;
    float squad_top_speed = LARGE_VALUE;
    const Unit *members = &units[squad->first_unit_index];
    for (int member_index = 0; member_index < squad->unit_count; member_index++)
        squad_top_speed = fminf(squad_top_speed, members[member_index].top_speed);
    return fminf(speed, squad_top_speed);
}


// `decide_reverse` tells whether a squad drives backward on its new route (`--reverse`): the order
// asks for it and every member is a vehicle, wherever the destination lies. A member that faces
// its way turns on the spot first (steering.c `steer_backward`).
static bool decide_reverse(const Squad *squad) {
    if (!is_reverse_on || !squad->is_reverse_order)
        return false;
    const Unit *members = &units[squad->first_unit_index];
    for (int member_index = 0; member_index < squad->unit_count; member_index++) {
        if (is_infantry_type(members[member_index].unit_type))
            return false;
    }
    return true;
}


// `prepare_formation_route` prepares the route of a squad for formation following: it moves the
// route start toward the squad members, shortens the route, and takes the route of a centered game
// search when that route is clearly shorter (route.c). With `--smoothreorder`, the route of a squad
// that moves starts with a turn at its current speed, along lead-in points. A squad that drives
// backward keeps the route of the base game path search, with only its start moved.
static void prepare_formation_route(Squad *squad, const float *parking_positions, V2 facing) {
    Unit *members = &units[squad->first_unit_index];
    int terrain_class = members[0].unit_type->terrain_class;
    float max_radius;
    float full_speed_min_radius;
    V2 center;
    measure_members(squad, &max_radius, &full_speed_min_radius, &center);
    squad->is_reversing = decide_reverse(squad);

    // A route with a single point has no start to move.
    if (squad->route_count > 1)
        move_route_start(squad, parking_positions, facing, center, terrain_class);
    squad->lead_in_point_count = 0;
    if (squad->is_reversing)
        return;
    shorten_route(squad, max_radius, terrain_class);
    float corner_gap = get_corner_gap(&members[0]);
    take_shorter_centered_route(squad, parking_positions, facing, center, max_radius,
                                terrain_class, corner_gap);

    // The lead-in points come last, since the shortening would remove them. Below the
    // current-speed radius, the squad members slow down in the turn (steering.c).
    if (get_reorder_speed(squad) > 0) {
        float turn_radius;
        V2 motion = measure_motion(squad, &turn_radius);
        V2 motion_direction = mul(motion, 1 / len(motion));
        float lead_in_radius = LEAD_IN_RADIUS_FRACTION * turn_radius;
        add_lead_in_points(squad, motion_direction, lead_in_radius, max_radius, terrain_class);
    }
}


// `begin_formation_smoothing` starts the route smoothing of a squad (smoothing.c), with half the
// sideways spread of the final parking positions as the half width of the formation.
static void begin_formation_smoothing(Squad *squad, const float *parking_positions, V2 facing) {
    int member_count = squad->unit_count;
    int terrain_class = units[squad->first_unit_index].unit_type->terrain_class;
    float max_radius;
    float full_speed_min_radius;
    V2 center;
    measure_members(squad, &max_radius, &full_speed_min_radius, &center);

    // The sideways spread is measured in the frame of the final facing at the route end.
    V2 end = squad->route[squad->route_count - 1];
    float x_min = LARGE_VALUE;
    float x_max = -LARGE_VALUE;
    for (int parking_index = 0; parking_index < member_count; parking_index++) {
        float x = get_parking_offset(parking_positions, parking_index, end, facing).x;
        x_min = fminf(x_min, x);
        x_max = fmaxf(x_max, x);
    }
    float half_width = (x_max - x_min) / 2;
    begin_smoothing(squad, max_radius, half_width, full_speed_min_radius, terrain_class);
}


// `begin_formation_set_up` prepares the route of a squad and starts its smoothing (`--smooth`).
// It returns true when a smoothing runs, which must end before `finish_formation_set_up`. The
// route of a squad that drives backward is never smoothed.
bool begin_formation_set_up(Squad *squad, const float *parking_positions, V2 facing) {
    prepare_formation_route(squad, parking_positions, facing);
    if (!is_smooth_on || squad->is_reversing)
        return false;
    begin_formation_smoothing(squad, parking_positions, facing);
    return true;
}


// `smooth_in_thread` runs the whole route smoothing of a threaded set-up on a worker thread. The
// thread reads the grid copy of the set-up, and the stamp box of its stamps, for the time of the
// job. The route of a squad that drives backward stays as it is.
static void smooth_in_thread(void *argument) {
    ThreadedSetUp *set_up = argument;
    Squad *squad = set_up->squad;
    if (squad->is_reversing)
        return;
    StampBox own_stamp_box = get_stamp_box();
    gc_use_grid(set_up->grid);
    set_stamp_box(set_up->stamp_box);
    begin_formation_smoothing(squad, squad->set_up_parking_positions, squad->set_up_facing);
    run_smoothing(INT_MAX);
    set_stamp_box(own_stamp_box);
    gc_use_grid(NULL);
}


// `submit_threaded_smoothing` runs the first part of a threaded set-up on this thread, and submits
// the job of its route smoothing.
void submit_threaded_smoothing(ThreadedSetUp *set_up, Squad *squad) {
    // The grid copy holds the stamps of this squad, so the worker thread sees them.
    if (!set_up->grid)
        set_up->grid = malloc(GC_GRID_BYTES);
    stamp_parked(squad, squad->start_position, squad->destination);
    prepare_formation_route(squad, squad->set_up_parking_positions, squad->set_up_facing);
    gc_copy_grid(set_up->grid);
    set_up->stamp_box = get_stamp_box();
    unstamp_parked();

    set_up->squad = squad;
    submit_job(smooth_in_thread, set_up);
}


// `finish_threaded_set_up` ends a threaded set-up after its smoothing, with the parked squads
// stamped as cliffs again, as for the first part.
void finish_threaded_set_up(const ThreadedSetUp *set_up) {
    Squad *squad = set_up->squad;
    stamp_parked(squad, squad->start_position, squad->destination);
    finish_formation_set_up(squad, squad->set_up_parking_positions, squad->set_up_facing);
    unstamp_parked();
}


// `measure_route_distances` computes the route position of each route point, and the route length.
static void measure_route_distances(Squad *squad) {
    squad->route_distance[0] = 0;
    for (int point_index = 1; point_index < squad->route_count; point_index++) {
        V2 segment = sub(squad->route[point_index], squad->route[point_index - 1]);
        squad->route_distance[point_index] = squad->route_distance[point_index - 1] + len(segment);
    }
    squad->route_length = squad->route_distance[squad->route_count - 1];
}


// `reset_squad_motion` puts the anchor of a squad at the route start and clears the timers of the
// lag wait and the slot swaps. A re-routing squad keeps its anchor speed, and with
// `--smoothreorder`, a squad that moves at a new order starts its anchor at its current speed.
static void reset_squad_motion(Squad *squad, V2 facing, float full_speed_min_radius) {
    squad->anchor_distance = 0;
    if (!squad->is_rerouting)
        squad->anchor_speed = get_reorder_speed(squad);
    squad->waited_seconds = 0;
    squad->ignore_lag_seconds = 0;
    squad->swap_seconds = 0;
    squad->swap_count = 0;
    squad->has_final_swap = false;
    squad->final_facing = facing;
    squad->full_speed_min_radius = full_speed_min_radius;
}


// `match_parking_positions` finds which squad member takes which final parking position: the
// permutation whose shapes match best. Both shapes are centered on their means, so the match keeps
// the arrangement of the squad members, not their distances to the destination.
static void match_parking_positions(const Squad *squad, const float *parking_positions, V2 facing,
                                    V2 *parking_offsets, int *best_permutation) {
    // The parking offsets are in the frame of the final facing at the route end, and the member
    // offsets in the frame of the first route segment at the route start.
    int member_count = squad->unit_count;
    const Unit *members = &units[squad->first_unit_index];
    float mean_weight = 1.0f / member_count;
    V2 end = squad->route[squad->route_count - 1];
    V2 start_direction = facing;
    if (squad->route_count > 1)
        start_direction = sub(squad->route[1], squad->route[0]);
    start_direction = get_direction(start_direction, 0, facing);
    V2 member_offsets[MAX_MEMBERS];
    V2 parking_mean = v2(0, 0);
    V2 member_mean = v2(0, 0);
    for (int member_index = 0; member_index < member_count; member_index++) {
        parking_offsets[member_index] = get_parking_offset(parking_positions, member_index, end,
                                                           facing);
        parking_mean = add(parking_mean, parking_offsets[member_index]);
        V2 from_start = sub(members[member_index].position, squad->route[0]);
        member_offsets[member_index] = get_frame_offset(from_start, start_direction);
        member_mean = add(member_mean, member_offsets[member_index]);
    }
    parking_mean = mul(parking_mean, mean_weight);
    member_mean = mul(member_mean, mean_weight);

    // `costs` holds the squared distance between the centered offsets, indexed by squad member
    // and then by final parking position.
    float costs[MAX_MEMBERS][MAX_MEMBERS];
    for (int member_index = 0; member_index < member_count; member_index++) {
        V2 member_shape = sub(member_offsets[member_index], member_mean);
        for (int parking_index = 0; parking_index < member_count; parking_index++) {
            V2 parking_shape = sub(parking_offsets[parking_index], parking_mean);
            costs[member_index][parking_index] = length_squared(sub(member_shape, parking_shape));
        }
    }
    float best_cost = LARGE_SQUARED_VALUE;
    find_best_permutation(costs, member_count, &best_cost, best_permutation);
}


// `count_slots_ahead` returns the column position of a squad member: the number of slots that lie
// ahead of its slot, and of slots level with it, those of squad members before it.
static int count_slots_ahead(const V2 *parking_offsets, const int *permutation, int member_index,
                             int member_count) {
    float y = parking_offsets[permutation[member_index]].y;        // forward slot position
    int count = 0;
    for (int other_index = 0; other_index < member_count; other_index++) {
        float y_other = parking_offsets[permutation[other_index]].y;
        bool is_ahead = y_other > y;
        bool is_level = y_other == y;
        bool is_level_and_before = is_level && other_index < member_index;
        if (is_ahead || is_level_and_before)
            count++;
    }
    return count;
}


// `assign_slots` gives each squad member its final parking position and its slot position, from
// `permutation`, and resets its state for formation following. The slot position is the offset of
// the final parking position from the route end, in the frame of the final facing.
static void assign_slots(Squad *squad, const float *parking_positions, const V2 *parking_offsets,
                         const int *permutation, float max_radius, V2 facing) {
    // In column formation, the slots follow each other by one column gap, front slot first.
    Unit *members = &units[squad->first_unit_index];
    int member_count = squad->unit_count;
    float column_gap_radii = get_tuning(squad->unit_type)->column_gap_radii;
    float column_gap = column_gap_radii * max_radius + COLUMN_GAP_EXTRA_METERS;
    for (int member_index = 0; member_index < member_count; member_index++) {
        Unit *member = &members[member_index];
        int parking_index = permutation[member_index];
        member->parking_position = get_pair_point(parking_positions, parking_index);
        member->slot_position = parking_offsets[parking_index];
        int column_position = count_slots_ahead(parking_offsets, permutation, member_index,
                                                member_count);
        member->anchor_to_slot_distance = column_position * column_gap;
        member->column_position = column_position;

        // The reach is the radius of the unit, but at least `MIN_FORMATION_REACH_METERS`.
        member->route_progress = 0;
        member->is_stopped = false;
        member->waypoint_count = 0;
        reset_detour(member);
        member->is_reorienting = false;
        member->reach = member->unit_type->radius;
        if (member->reach < MIN_FORMATION_REACH_METERS)
            member->reach = MIN_FORMATION_REACH_METERS;
        member->facing = facing;
    }
}


// `set_start_offsets` stores, for each squad member, its offset from its slot at the route start,
// which `get_target_slot` fades out over the start blend.
static void set_start_offsets(const Squad *squad) {
    Unit *members = &units[squad->first_unit_index];
    for (int member_index = 0; member_index < squad->unit_count; member_index++) {
        Unit *member = &members[member_index];
        V2 forward;
        V2 slot = get_slot(squad, member, &forward);
        member->start_offset = sub(member->position, slot);
    }
}


// `apply_reverse` gives the members of a squad the decision of `decide_reverse`. Members that
// drive backward first align their rear with their way, and they end as they drove, facing away
// from the destination, so their final facing is the opposite of the order facing; the final
// parking positions keep the layout of the order.
static void apply_reverse(const Squad *squad, V2 facing) {
    Unit *members = &units[squad->first_unit_index];
    for (int member_index = 0; member_index < squad->unit_count; member_index++) {
        Unit *member = &members[member_index];
        member->is_reversing = squad->is_reversing;
        member->is_aligning_to_reverse = squad->is_reversing;
        if (squad->is_reversing)
            member->facing = mul(facing, -1);
    }
}


// `finish_formation_set_up` ends the set-up of a squad after the route smoothing: it measures the
// route, assigns the final parking positions and the slots, and lays out the lanes (`--lanes`).
// It also decides whether the squad drives backward (`--reverse`).
void finish_formation_set_up(Squad *squad, const float *parking_positions, V2 facing) {
    // The squad starts in square formation.
    float max_radius;
    float full_speed_min_radius;
    V2 center;
    measure_members(squad, &max_radius, &full_speed_min_radius, &center);
    measure_route_distances(squad);
    reset_squad_motion(squad, facing, full_speed_min_radius);
    V2 parking_offsets[MAX_MEMBERS];
    int permutation[MAX_MEMBERS];
    match_parking_positions(squad, parking_positions, facing, parking_offsets, permutation);
    assign_slots(squad, parking_positions, parking_offsets, permutation, max_radius, facing);
    apply_reverse(squad, facing);
    squad->is_in_column = false;
    squad->is_reforming = false;
    if (are_lanes_on)
        set_up_lanes(squad, max_radius);
    set_start_offsets(squad);
}


// `compute_slot_cost` returns the cost of a squad member at the route coordinates `member` for the
// slot at `slot`: the squared forward difference plus `SWAP_SIDE_WEIGHT` times the squared
// sideways difference. The weight keeps most squad members on their side of the formation.
static float compute_slot_cost(V2 member, V2 slot) {
    float dy = member.y - slot.y;        // forward difference
    float dx = member.x - slot.x;        // sideways difference
    float dy_squared = dy * dy;
    float weighted_dx_squared = SWAP_SIDE_WEIGHT * dx * dx;
    return dy_squared + weighted_dx_squared;
}


// `measure_swap_coordinates` computes the route coordinates of each squad member of a squad and of
// its slot, in the order of the squad members.
static void measure_swap_coordinates(const Squad *squad, V2 *member_coordinates,
                                     V2 *slot_coordinates) {
    const Unit *members = &units[squad->first_unit_index];
    for (int member_index = 0; member_index < squad->unit_count; member_index++) {
        const Unit *member = &members[member_index];
        V2 forward;
        V2 slot = get_slot(squad, member, &forward);
        member_coordinates[member_index] = get_route_coordinates(squad, member->position);
        slot_coordinates[member_index] = get_route_coordinates(squad, slot);
    }
}


// `hand_over_slots` gives each squad member of a squad the slot of the squad member
// `best_permutation[member]`: its slot position, its final parking position, and its column place.
static void hand_over_slots(Squad *squad, const int *best_permutation) {
    // The slots are copied first, because the hand-over overwrites them.
    typedef struct {
        V2 slot_position;
        V2 parking_position;
        float anchor_to_slot_distance;
        int column_position;
    } Slot;
    Slot slots[MAX_MEMBERS];
    Unit *members = &units[squad->first_unit_index];
    int member_count = squad->unit_count;
    for (int member_index = 0; member_index < member_count; member_index++) {
        const Unit *member = &members[member_index];
        slots[member_index] = (Slot){
            .slot_position = member->slot_position,
            .parking_position = member->parking_position,
            .anchor_to_slot_distance = member->anchor_to_slot_distance,
            .column_position = member->column_position,
        };
    }

    // Each hand-over counts as one slot swap of the squad.
    for (int member_index = 0; member_index < member_count; member_index++) {
        Unit *member = &members[member_index];
        int slot_index = best_permutation[member_index];
        member->slot_position = slots[slot_index].slot_position;
        member->parking_position = slots[slot_index].parking_position;
        member->anchor_to_slot_distance = slots[slot_index].anchor_to_slot_distance;
        member->column_position = slots[slot_index].column_position;
    }
    squad->swap_count++;
}


// `compute_catch_up_cost` returns the way that a squad member at the route coordinates `member`
// must catch up to reach the slot at `slot`. A slot that is not ahead costs only the sideways way.
static float compute_catch_up_cost(V2 member, V2 slot) {
    float dy = slot.y - member.y;        // forward way to the slot
    float dx = slot.x - member.x;        // sideways way to the slot
    if (dy <= 0)
        return fabsf(dx);
    return hypotf(dy, dx);
}


// `find_reforming_permutation` finds the permutation with the smallest largest cost, and among
// equal largest costs, the smallest sum of squared costs. It tests every permutation.
static void find_reforming_permutation(float costs[][MAX_MEMBERS], int member_count,
                                       int *best_permutation, float *best_max_cost) {
    int permutation[MAX_MEMBERS];
    for (int member_index = 0; member_index < member_count; member_index++)
        permutation[member_index] = member_index;
    float best_sum_squared = LARGE_VALUE;
    *best_max_cost = LARGE_VALUE;
    do {
        // The largest cost is the catch-up of the slowest squad member, which the squad waits for.
        float max_cost = 0;
        float sum_squared = 0;
        for (int member_index = 0; member_index < member_count; member_index++) {
            float cost = costs[member_index][permutation[member_index]];
            max_cost = fmaxf(max_cost, cost);
            sum_squared += cost * cost;
        }

        // A tie in the largest cost goes to the smaller sum of squared costs.
        bool is_better = max_cost < *best_max_cost;
        bool is_tie = max_cost == *best_max_cost;
        if (is_better || (is_tie && sum_squared < best_sum_squared)) {
            *best_max_cost = max_cost;
            best_sum_squared = sum_squared;
            memcpy(best_permutation, permutation, sizeof(int) * member_count);
        }
    } while (advance_permutation(permutation, member_count));
}


// `swap_slots_for_reforming` swaps slots of a squad that reforms after column formation, so that
// the largest catch-up way of a squad member is smallest. Without `is_forced`, the swap must save
// `SWAP_MIN_SAVING_FRACTION` of the current largest way.
static void swap_slots_for_reforming(Squad *squad, bool is_forced) {
    // `costs` holds the catch-up way, indexed by squad member and then by slot.
    int member_count = squad->unit_count;
    V2 member_coordinates[MAX_MEMBERS];
    V2 slot_coordinates[MAX_MEMBERS];
    measure_swap_coordinates(squad, member_coordinates, slot_coordinates);
    float costs[MAX_MEMBERS][MAX_MEMBERS];
    for (int member_index = 0; member_index < member_count; member_index++) {
        V2 member_at = member_coordinates[member_index];
        for (int slot_index = 0; slot_index < member_count; slot_index++) {
            V2 slot_at = slot_coordinates[slot_index];
            costs[member_index][slot_index] = compute_catch_up_cost(member_at, slot_at);
        }
    }

    // The current slots are the identity permutation.
    float current_max = 0;        // largest catch-up way with the current slots
    for (int member_index = 0; member_index < member_count; member_index++)
        current_max = fmaxf(current_max, costs[member_index][member_index]);
    int best_permutation[MAX_MEMBERS];
    float best_max = 0;           // largest catch-up way with the best slots
    find_reforming_permutation(costs, member_count, best_permutation, &best_max);
    float min_saving = SWAP_MIN_SAVING_FRACTION * current_max;
    bool is_saving_enough = best_max <= current_max - min_saving;
    bool is_change = false;
    for (int member_index = 0; member_index < member_count; member_index++)
        is_change = is_change || best_permutation[member_index] != member_index;
    if (is_change && (is_forced || is_saving_enough))
        hand_over_slots(squad, best_permutation);
}


// `swap_slots_by_distance` swaps slots of a squad when that clearly lowers the sum of the slot
// costs (`compute_slot_cost`). On the end section of the route, it makes one final swap by the
// squared distances to the final parking positions, and the slots then stay.
static void swap_slots_by_distance(Squad *squad, bool is_end_section) {
    // `costs` is indexed by squad member and then by slot.
    int member_count = squad->unit_count;
    Unit *members = &units[squad->first_unit_index];
    float costs[MAX_MEMBERS][MAX_MEMBERS];
    if (is_end_section) {
        for (int member_index = 0; member_index < member_count; member_index++) {
            V2 member_at = members[member_index].position;
            for (int slot_index = 0; slot_index < member_count; slot_index++) {
                V2 to_parking = sub(members[slot_index].parking_position, member_at);
                costs[member_index][slot_index] = length_squared(to_parking);
            }
        }
    } else {
        V2 member_coordinates[MAX_MEMBERS];
        V2 slot_coordinates[MAX_MEMBERS];
        measure_swap_coordinates(squad, member_coordinates, slot_coordinates);
        for (int member_index = 0; member_index < member_count; member_index++) {
            V2 member_at = member_coordinates[member_index];
            for (int slot_index = 0; slot_index < member_count; slot_index++) {
                V2 slot_at = slot_coordinates[slot_index];
                costs[member_index][slot_index] = compute_slot_cost(member_at, slot_at);
            }
        }
    }

    // Away from the end section, a swap must save `SWAP_MIN_SAVING_FRACTION` of the current cost,
    // and at least `SWAP_MIN_SAVING_SQUARED`.
    float current_cost = 0;
    for (int member_index = 0; member_index < member_count; member_index++)
        current_cost += costs[member_index][member_index];
    float best_cost = current_cost;
    int best_permutation[MAX_MEMBERS];
    find_best_permutation(costs, member_count, &best_cost, best_permutation);
    float fraction_saving = SWAP_MIN_SAVING_FRACTION * current_cost;
    float min_saving = fmaxf(SWAP_MIN_SAVING_SQUARED, fraction_saving);
    if (is_end_section)
        squad->has_final_swap = true;
    else if (best_cost > current_cost - min_saving)
        return;
    hand_over_slots(squad, best_permutation);
}


// `is_swap_due` tells whether a squad checks its slots for a swap in this tick: every
// `SWAP_INTERVAL_SECONDS` after the start blend (`FAST_SWAP_INTERVAL_SECONDS` with
// `--fastswap`), and on the end section only once.
static bool is_swap_due(Squad *squad, bool is_end_section) {
    squad->swap_seconds -= TICK_SECONDS;
    bool is_too_early = squad->anchor_distance < START_BLEND_METERS || squad->swap_seconds > 0;
    if (is_end_section)
        is_too_early = squad->has_final_swap;
    if (is_too_early)
        return false;
    squad->swap_seconds = is_fast_swap_on ? FAST_SWAP_INTERVAL_SECONDS : SWAP_INTERVAL_SECONDS;
    return true;
}


// `swap_slots` swaps slots of a squad when that shortens the ways of its squad members to their
// slots (`--formation`). `has_left_column` forces a check when the squad leaves column formation,
// except on the end section of the route (`--driving`).
static void swap_slots(Squad *squad, bool has_left_column) {
    // A squad of more than `MAX_SWAP_MEMBERS` squad members never swaps.
    int member_count = squad->unit_count;
    bool is_swappable = member_count >= 2 && member_count <= MAX_SWAP_MEMBERS;
    if (!is_formation_keeping_on || !is_swappable)
        return;

    // A reforming squad swaps for the smallest largest catch-up way.
    float remaining = squad->route_length - squad->anchor_distance;        // route past the anchor
    bool is_end_section = is_driving_on && remaining <= END_BLEND_METERS;
    bool is_due = !is_end_section;
    if (!has_left_column)
        is_due = is_swap_due(squad, is_end_section);
    if (!is_due)
        return;
    if (squad->is_reforming && !is_end_section)
        swap_slots_for_reforming(squad, has_left_column);
    else
        swap_slots_by_distance(squad, is_end_section);
}


// `measure_lag` returns the lag of a squad: the largest distance by which a squad member lies
// behind its target slot, along the formation direction there.
static float measure_lag(const Squad *squad) {
    float lag = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        V2 forward;
        V2 target = get_target_slot(squad, unit, &forward);
        V2 to_target = sub(target, unit->position);
        lag = fmaxf(lag, dot(to_target, forward));
    }
    return lag;
}


// `get_lag_factor` returns the share of the top speed that the anchor keeps for `lag`: it falls in
// step with the lag down to `1 - LAG_SLOWDOWN_FRACTION` at `LAG_SLOWDOWN_METERS`.
static float get_lag_factor(float lag) {
    float lag_fraction = fminf(1, lag / LAG_SLOWDOWN_METERS);
    return 1 - LAG_SLOWDOWN_FRACTION * lag_fraction;
}


// `wait_for_laggards` slows the anchor speed `speed` of a squad for a lag above the lag wait
// distance of the faction tuning, down to a crawl, and stops it above `LAG_STOP_METERS`. After
// `STUCK_WAIT_SECONDS` nearly still, the anchor ignores the lag for `IGNORE_LAG_SECONDS`, so one
// stuck squad member cannot stop it.
static float wait_for_laggards(Squad *squad, float lag, float speed) {
    // The waiting factor falls by one for each `LAG_WAIT_RANGE_METERS` of lag past the start.
    float lag_wait_meters = get_tuning(squad->unit_type)->lag_wait_meters;
    squad->ignore_lag_seconds -= TICK_SECONDS;
    if (squad->ignore_lag_seconds <= 0 && lag > lag_wait_meters) {
        float factor = 0;        // share of the speed that stays
        if (lag <= LAG_STOP_METERS) {
            float slowdown = 1 - (lag - lag_wait_meters) / LAG_WAIT_RANGE_METERS;
            factor = fmaxf(CRAWL_FRACTION, slowdown);
        }
        speed *= factor;
    }

    // The timer of the wait counts only the ticks in which the anchor is nearly still.
    if (speed < ANCHOR_STILL_SPEED)
        squad->waited_seconds += TICK_SECONDS;
    else
        squad->waited_seconds = 0;
    if (squad->waited_seconds > STUCK_WAIT_SECONDS) {
        squad->ignore_lag_seconds = IGNORE_LAG_SECONDS;
        squad->waited_seconds = 0;
    }
    return speed;
}


// `update_column_formation` switches a squad between square and column formation (`--lanes`).
// It enters column formation when the lanes do not fit around the formation, and leaves it when
// they fit from behind the column up to `COLUMN_LEAVE_AHEAD_METERS` past the front. With
// `--infantrysquare`, infantry squads always keep square formation.
static void update_column_formation(Squad *squad, float column_depth, float y_min, float y_max) {
    // `REAR_MARGIN_METERS` extends both checks past the front and the rear of the formation.
    float anchor = squad->anchor_distance;
    float column_rear = anchor - column_depth - REAR_MARGIN_METERS;
    bool is_square_kept = is_infantry_square_on && is_infantry_type(squad->unit_type);
    if (!are_lanes_on || is_square_kept)
        return;
    float front = anchor + y_max;
    if (!squad->is_in_column) {
        float rear = anchor + y_min - REAR_MARGIN_METERS;
        squad->is_in_column = !do_lanes_fit(squad, rear, front + REAR_MARGIN_METERS);
    } else {
        float column_front = front + COLUMN_LEAVE_AHEAD_METERS;
        squad->is_in_column = !do_lanes_fit(squad, column_rear, column_front);
    }
}


// `advance_anchor` moves the anchor of a squad for one tick, after the slot swaps and the switch
// between square and column formation. The anchor drives at the top speed of the squad, slowed
// for lag (`--formation`), for the stop at the route end, and by the speed cap. A squad that
// drives backward keeps no formation: its slots never swap and its anchor never waits for lag.
void advance_anchor(Squad *squad) {
    // `column_depth` is how far the last slot of the column lies behind the anchor.
    bool is_keeping_formation = !squad->is_reversing;
    if (is_keeping_formation)
        swap_slots(squad, false);
    float v_top = LARGE_VALUE;           // top speed of the squad
    float friction = LARGE_VALUE;        // lowest friction of the squad members
    float column_depth = 0;
    float y_min = 0;                     // smallest forward slot position
    float y_max = 0;                     // largest forward slot position
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        v_top = fminf(v_top, unit->top_speed);
        friction = fminf(friction, unit->unit_type->friction);
        column_depth = fmaxf(column_depth, get_anchor_to_slot_distance(squad, unit));
        y_min = fminf(y_min, unit->slot_position.y);
        y_max = fmaxf(y_max, unit->slot_position.y);
    }

    // A squad that leaves column formation reforms, and its slots swap at once.
    float anchor = squad->anchor_distance;
    bool was_in_column = squad->is_in_column;
    update_column_formation(squad, column_depth, y_min, y_max);
    bool has_left_column = was_in_column && !squad->is_in_column;
    if (has_left_column && is_keeping_formation) {
        squad->is_reforming = true;
        swap_slots(squad, true);
    }

    // Reforming ends when the lag is small again or the squad is back in column formation.
    float lag = measure_lag(squad);
    bool is_lag_small = lag <= get_tuning(squad->unit_type)->lag_wait_meters;
    if (squad->is_reforming && (is_lag_small || squad->is_in_column))
        squad->is_reforming = false;
    float waited_lag = 0;
    if (is_formation_keeping_on && is_keeping_formation)
        waited_lag = lag;

    // A body that brakes at the deceleration `b` stops within `d` from `sqrt(2 * b * d)`.
    float speed = v_top * get_lag_factor(waited_lag);
    float remaining = squad->route_length - anchor;
    float twice_deceleration = 2 * END_DECELERATION_FRACTION * friction;
    float v_stop = sqrtf(twice_deceleration * remaining);
    speed = fminf(speed, v_stop);
    speed = wait_for_laggards(squad, waited_lag, speed);
    speed = fminf(speed, squad->speed_cap);
    squad->anchor_speed = speed;
    squad->anchor_distance = fminf(squad->route_length, anchor + speed * TICK_SECONDS);
}
