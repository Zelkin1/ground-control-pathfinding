/* overtaking.c lets a faster squad pass a slower one on the same route, or follow it. It is part
 * of the coordination between squads in the "coord" mode of `--avoid`: `coordinate_squads` in
 * coordination.c calls `pass_or_follow` for every pair of moving squads that do not meet
 * head-on, with `--overtake on`. It uses the formation shapes, the route samples and the side
 * room that coordination.c defines, and the pass shifts of shift.c.
 *
 * Terms: The top comment of sim.h defines the anchor, the lane, the clamp, the shift, the leader
 * and the follower. In short, the shift of a squad moves all its lanes sideways over a stretch
 * of its route, and the clamp is the limit that keeps the bodies clear of blocked ground. The
 * room of a squad toward a side is how far its lanes can shift that way before the outermost
 * lane on that side reaches its clamp. coordination.c defines the room. The rear and the front
 * of a squad are how far its bodies reach before and past its anchor along the route.
 *
 * Passing: When the follower is faster, the two squads shift their lanes apart over the whole
 * pass, the leader usually to its right, and the follower passes:
 *
 *     before:   [follower] -->   [leader] -->
 *     passing:                    [leader] -->        (shifted right)
 *               [follower] ------------------->       (shifted left)
 *
 * With `--keeproute on`, the leader keeps its route, and only the follower shifts around it. With
 * `--leaderfile on`, which the baseline mode of the replay uses, the leader drives in single file
 * on its side where two formations do not fit side by side. With `--formfirst on`, the follower
 * follows until the leader has shifted, and only then passes.
 *
 * Following: Without room for a pass, or when the follower is not faster, the follower keeps a
 * gap behind the rear of the leader. Without this rule, the two squads drive on the same lanes and
 * their units weave through each other. With `--keepfollow on`, a pair stays leader and follower
 * while the leader drives around a sharp corner of its route. */
#include "sim.h"
#include "formation.h"

#define SAME_WAY_MIN_COSINE 0.85f         /* Routes this close to parallel run the same way. */
#define PASS_MIN_SPEED_GAIN 1.0f          /* Squads pass only squads this much slower, in m/s. */
#define PASS_LOOK_SECONDS 6.0f            /* A pass is planned this long before it would start. */
#define PASS_BACK_METERS 120.0f           /* Pass checks sample routes this far behind the rear. */
#define PASS_HORIZON_METERS 450.0f        /* Pass checks sample routes this far past the anchor. */
#define PASS_END_MARGIN_METERS 20.0f      /* The full pass shift reaches this far further. */
#define PASS_GAP_METERS 5.0f              /* Passes start and end with this gap between bodies. */
#define PASS_ROOM_STEP_METERS 2.0f        /* Pass room is measured this often, as lanes are. */
#define PASS_SQUEEZE_METERS 3.0f          /* Passing bodies may come this much closer still. */
#define FORMED_LANE_METERS 2.0f           /* Members of a formed leader are this close to lanes. */
#define FORMED_FRACTION 0.9f              /* Its shift has moved this part of the way. */
#define PASS_ROOM_MARGIN_METERS 4.0f      /* Pass shifts stay this far inside the lane room. */
#define FILE_PASS_ROOM_MARGIN_METERS 1.0f /* Past a single file, they stay this far inside. */
#define FOLLOW_HEADWAY_SECONDS 1.0f       /* Followers keep this time gap behind their leader. */
#define FOLLOW_MIN_GAP_METERS 10.0f       /* Their gap is at least this long. */
#define FOLLOW_GAIN 0.5f                  /* The cap of a follower moves this much per m. */
#define CROSS_SIDE_FRACTION 0.5f          /* With `--crossfollow`, sides need this gap share. */

/* ------------------------------------------------------------------------------------------------
 * `sample_route_around` samples the route of the squad from `PASS_BACK_METERS` before its rear
 * to `PASS_HORIZON_METERS` past its anchor. Both are distances along the route, and the samples
 * stay within the route.
 * --------------------------------------------------------------------------------------------- */
static void sample_route_around(const Squad *squad, const FormationShape *shape,
                                RouteSamples *samples) {
    float start = fmaxf(0, shape->rear_progress - PASS_BACK_METERS);
    float horizon = squad->anchor_distance + PASS_HORIZON_METERS;
    float end = fminf(squad->route_length, horizon);
    sample_route_span(squad, start, end, samples);
}

/* ------------------------------------------------------------------------------------------------
 * `locate_on_route` finds where `point`, which moves along `direction`, lies on a sampled route:
 *
 *                     `distance`
 *   o-----------------o----+-------------o----> route
 *      closest sample ^    |
 *                          | `lateral`, to the right of the route
 *                          * point, which moves along `direction`
 *
 * Result: It returns true, and sets `distance` to how far along the route the point lies and
 *   `lateral` to how far it lies to the right of the route, when the closest sample is less than
 *   `max_gap` to the side of the point and runs the same way, within `SAME_WAY_MIN_COSINE`. With
 *   `is_any_direction`, the direction of the sample does not matter. Otherwise it returns false.
 * --------------------------------------------------------------------------------------------- */
static bool locate_on_route(const RouteSamples *samples, V2 point, V2 direction, float max_gap,
                            bool is_any_direction, float *distance, float *lateral) {
    /* Find the closest sample, and how far the point lies to its right. */
    if (samples->count == 0)
        return false;
    int sample_index = find_closest_sample_index(samples, point);
    V2 sample_direction = samples->directions[sample_index];
    V2 to_point = sub(point, samples->points[sample_index]);
    float point_lateral = dot(to_point, get_right(sample_direction));

    /* Accept only a point beside the route that moves the same way. */
    bool is_beside = fabsf(point_lateral) < max_gap;
    float direction_cosine = dot(direction, sample_direction);
    bool is_same_way = is_any_direction || direction_cosine >= SAME_WAY_MIN_COSINE;
    if (!is_beside || !is_same_way)
        return false;
    *distance = samples->distances[sample_index] + dot(to_point, sample_direction);
    *lateral = point_lateral;
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_stretch_room` returns the smallest room (`measure_side_room`) of the squad toward
 * `side` along its route from `start` to `end`. It measures the room every
 * `PASS_ROOM_STEP_METERS`, and it begins at the start of the route when `start` lies before it.
 * It returns 0 when the stretch is empty.
 * --------------------------------------------------------------------------------------------- */
static float measure_stretch_room(const Squad *squad, float start, float end, float side) {
    float room = LARGE_VALUE;
    for (float distance = fmaxf(0, start); distance <= end; distance += PASS_ROOM_STEP_METERS)
        room = fminf(room, measure_side_room(squad, distance, side));
    if (room == LARGE_VALUE)
        return 0;
    return room;
}

/* An `OrderedPair` names which of two squads on one stretch of route drives behind the other. */
typedef struct {
    Squad *follower;
    Squad *leader;
    const FormationShape *follower_shape;
    const FormationShape *leader_shape;
    const RouteSamples *follower_samples;
    const RouteSamples *leader_samples;
    float leader_anchor;  /* The anchor of the leader is this far along the follower route. */
    float leader_lateral; /* The route of the leader lies this far right of the follower route. */
} OrderedPair;

/* ------------------------------------------------------------------------------------------------
 * `measure_back` returns how far the rear of the squad reaches before its anchor along the
 * route, as the squad drives now.
 * --------------------------------------------------------------------------------------------- */
static float measure_back(const Squad *squad, const FormationShape *shape) {
    return squad->anchor_distance - shape->rear_progress;
}

/* ------------------------------------------------------------------------------------------------
 * `get_leader_rear` returns how far along the follower route the rear of the leader is.
 * --------------------------------------------------------------------------------------------- */
static float get_leader_rear(const OrderedPair *pair) {
    float leader_back = measure_back(pair->leader, pair->leader_shape);
    return pair->leader_anchor - leader_back;
}

/* ------------------------------------------------------------------------------------------------
 * `get_follower_front` returns how far along its route the front of the follower is.
 * --------------------------------------------------------------------------------------------- */
static float get_follower_front(const OrderedPair *pair) {
    return pair->follower->anchor_distance + pair->follower_shape->front_meters;
}

/* ------------------------------------------------------------------------------------------------
 * `order_pair` finds out whether one squad drives behind the other on the same stretch of route.
 * That holds when the anchor of one squad lies on the route of the other, nearer to the route
 * than `needed_gap` sideways, moves the same way, and lies further along that route than the
 * anchor of the other squad.
 * With `--keepfollow on`, a squad that followed the other one in the last tick still follows it
 * when the anchor of the leader moves another way. A leader with a sharp corner on its route
 * points away from a smooth route of the follower for a few seconds. Without this hold, the
 * crossing rule would take over the pair for that time, and it could stop the follower until the
 * whole leader is far past the corner.
 * Result: It returns true and fills `pair`, or false when the squads are not on one stretch.
 * --------------------------------------------------------------------------------------------- */
static bool order_pair(Squad *first, const FormationShape *first_shape,
                       const RouteSamples *first_samples, Squad *second,
                       const FormationShape *second_shape, const RouteSamples *second_samples,
                       float needed_gap, OrderedPair *pair) {
    for (int order_index = 0; order_index < 2; order_index++) {
        /* Try the first squad as the follower, then the second one. */
        bool is_first_following = order_index == 0;
        Squad *follower = is_first_following ? first : second;
        Squad *leader = is_first_following ? second : first;
        const RouteSamples *follower_samples = is_first_following ? first_samples : second_samples;
        V2 leader_direction;
        V2 leader_point = get_route_point(leader, leader->anchor_distance, &leader_direction);
        float leader_anchor = 0;
        float leader_lateral = 0;
        bool was_following = follower->was_following[leader->squad_index];
        bool is_any_direction = is_keep_follow_on && was_following;
        bool is_on_route = locate_on_route(follower_samples, leader_point, leader_direction,
                                           needed_gap, is_any_direction, &leader_anchor,
                                           &leader_lateral);
        bool is_leader_ahead = leader_anchor > follower->anchor_distance;
        if (!is_on_route || !is_leader_ahead)
            continue;

        /* Fill the pair. */
        pair->follower = follower;
        pair->leader = leader;
        pair->follower_shape = is_first_following ? first_shape : second_shape;
        pair->leader_shape = is_first_following ? second_shape : first_shape;
        pair->follower_samples = follower_samples;
        pair->leader_samples = is_first_following ? second_samples : first_samples;
        pair->leader_anchor = leader_anchor;
        pair->leader_lateral = leader_lateral;
        return true;
    }
    return false;
}

/* A `PassPlan` is one way for the follower of an `OrderedPair` to pass the leader. It holds each
 * stretch of the pass of each squad along its own route, the shift of each one (positive to the
 * right), and how much of the missing sideways gap their room leaves over (`shortfall`). */
typedef struct {
    float follower_start;
    float follower_end;
    float leader_start;
    float leader_end;
    float follower_shift;
    float leader_shift;
    float shortfall;
} PassPlan;

/* ------------------------------------------------------------------------------------------------
 * `measure_file_stretch_room` returns the room toward `side` that the lanes of the squad would
 * have in single file along its route from `start` to `end`. It begins at the start of the route
 * when `start` lies before it. The room is the smallest distance from the middle of the passage
 * to the clamp on that side, measured every `PASS_ROOM_STEP_METERS`. It is 0 when the stretch is
 * empty.
 * --------------------------------------------------------------------------------------------- */
static float measure_file_stretch_room(const Squad *squad, float start, float end, float side) {
    float room = LARGE_VALUE;
    for (float distance = fmaxf(0, start); distance <= end; distance += PASS_ROOM_STEP_METERS) {
        Lane lane = get_lane(squad, distance);
        float file_room = lane.middle - lane.min_x;
        if (side > 0)
            file_room = lane.max_x - lane.middle;
        room = fminf(room, fmaxf(0, file_room));
    }
    if (room == LARGE_VALUE)
        return 0;
    return room;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_file_back` returns how far the rear of the squad would reach before its anchor along
 * the route in single file. That is the farthest that the body of a member would reach there (its
 * file distance plus its radius), or the reach of `measure_back` when that is longer. It takes the
 * file distance of each member from `get_file_distance` to find how long the squad becomes.
 * --------------------------------------------------------------------------------------------- */
static float measure_file_back(const Squad *squad, const FormationShape *shape) {
    float back = measure_back(squad, shape);
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        float file_back = get_file_distance(squad, unit) + unit->kind->radius;
        back = fmaxf(back, file_back);
    }
    return back;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_max_radius` returns the radius of the widest member of the squad.
 * --------------------------------------------------------------------------------------------- */
static float measure_max_radius(const Squad *squad) {
    float max_radius = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++)
        max_radius = fmaxf(max_radius, units[unit_index].kind->radius);
    return max_radius;
}

/* ------------------------------------------------------------------------------------------------
 * `plan_pass` plans a pass of the follower of `pair` past a leader whose rear reaches
 * `leader_back` before its anchor along the route, and whose bodies reach `leader_half_width`
 * to each side.
 * Stretches: Each squad shifts over a stretch of its own route. Contact is when the front of the
 *   follower comes within `PASS_GAP_METERS` (gap) of the rear of the leader, and the pass ends
 *   when the rear of the follower is that far past the front of the leader. Both times assume
 *   the cruise speeds:
 *
 *     now:      [follower]>         [leader]>
 *     contact:           [follower]> gap [leader]>
 *     pass end:                                       [leader]> gap [follower]>
 *                         |<------------ stretch of the follower ----------->|
 *                                        |<--leader stretch-->|
 *
 *   The stretch of the follower runs from its front now to its front at the pass end. The
 *   stretch of the leader runs from its rear at contact to its front at the pass end. When the
 *   front of the follower is within the gap already, the stretch of the follower starts at the
 *   gap behind the rear of the leader instead, and the stretch of the leader at its rear now.
 *   Each stretch gets `PASS_END_MARGIN_METERS` more at each end, and it ends at the end of its
 *   route at the latest.
 * Shifts: The follower passes on the left of the leader, which keeps right, unless the anchor of
 *   the leader lies more than `SAME_LINE_METERS` to the left of the follower route. The missing
 *   gap is how much farther the leader route must lie toward the side of the leader, so that the
 *   bodies keep `HEAD_ON_MARGIN_METERS` apart. The leader takes as much of the missing gap as
 *   its room allows, so that the faster follower keeps its line, and the follower takes the
 *   rest, as far as its room allows. With `--keeproute on`, the leader keeps its route instead,
 *   and the follower takes the whole missing gap.
 * Rooms: Each room stays `PASS_ROOM_MARGIN_METERS` inside the clamps, whose kinks would make units
 *   brake. With `is_leader_file`, the room of the leader is the room of its lanes in single
 *   file, and the rooms stay only `FILE_PASS_ROOM_MARGIN_METERS` inside. A pass past a single
 *   file is meant for passages too narrow for two formations, where the larger margin would
 *   leave no room.
 * --------------------------------------------------------------------------------------------- */
static void plan_pass(const OrderedPair *pair, float leader_back, float leader_half_width,
                      bool is_leader_file, PassPlan *plan) {
    /* Find contact. At contact, the rear of the leader is at `contact_rear` and the front of the
     * follower at `contact_front`, along the follower route. */
    const Squad *follower = pair->follower;
    const Squad *leader = pair->leader;
    const FormationShape *follower_shape = pair->follower_shape;
    const FormationShape *leader_shape = pair->leader_shape;
    float speed_gain = follower_shape->cruise_speed - leader_shape->cruise_speed;
    float leader_rear = pair->leader_anchor - leader_back;
    float leader_front = pair->leader_anchor + leader_shape->front_meters;
    float follower_front = get_follower_front(pair);
    float contact_seconds = (leader_rear - PASS_GAP_METERS - follower_front) / speed_gain;
    float contact_travel = leader_shape->cruise_speed * fmaxf(0, contact_seconds);
    float contact_rear = leader_rear + contact_travel;
    float contact_front = contact_rear - PASS_GAP_METERS;

    /* Find the pass end. There, the front of the leader is at `end_front` and the rear of the
     * follower at `end_rear`, along the follower route. */
    float gap_past_front = leader_front + PASS_GAP_METERS;
    float distance_to_gain = gap_past_front - follower_shape->rear_progress;
    float pass_seconds = fmaxf(0, distance_to_gain) / speed_gain;
    float leader_travel = leader_shape->cruise_speed * pass_seconds;
    float end_front = leader_front + leader_travel;
    float end_rear = gap_past_front + leader_travel;

    /* Set both stretches. `route_offset` turns distances along the follower route into
     * distances along the leader route. */
    float follower_length = follower_front - follower_shape->rear_progress;
    float follower_pass_end = end_rear + follower_length + PASS_END_MARGIN_METERS;
    float route_offset = leader->anchor_distance - pair->leader_anchor;
    float leader_pass_end = end_front + route_offset + PASS_END_MARGIN_METERS;
    plan->follower_start = fminf(follower_front, contact_front) - PASS_END_MARGIN_METERS;
    plan->follower_end = fminf(follower->route_length, follower_pass_end);
    plan->leader_start = contact_rear + route_offset - PASS_END_MARGIN_METERS;
    plan->leader_end = fminf(leader->route_length, leader_pass_end);

    /* Pick the sides, and find the missing gap. */
    float lateral = pair->leader_lateral;
    float follower_side = lateral < -SAME_LINE_METERS ? 1.0f : -1.0f;
    float leader_side = -follower_side;
    float half_widths = follower_shape->half_width + leader_half_width;
    float needed_gap = half_widths + HEAD_ON_MARGIN_METERS;
    float missing_gap = needed_gap + follower_side * lateral;

    /* Measure the room of each squad over its stretch, and keep the margin inside the clamps. */
    float follower_room = measure_stretch_room(follower, plan->follower_start, plan->follower_end,
                                               follower_side);
    float leader_room = 0;
    if (is_leader_file) {
        leader_room = measure_file_stretch_room(leader, plan->leader_start, plan->leader_end,
                                                leader_side);
    } else if (!is_keep_route_on) {
        leader_room = measure_stretch_room(leader, plan->leader_start, plan->leader_end,
                                           leader_side);
    }
    float room_margin = is_leader_file ? FILE_PASS_ROOM_MARGIN_METERS : PASS_ROOM_MARGIN_METERS;
    follower_room = fmaxf(0, follower_room - room_margin);
    leader_room = fmaxf(0, leader_room - room_margin);

    /* Split the missing gap. */
    float leader_shift = clamp_float(missing_gap, 0, leader_room);
    float follower_shift = clamp_float(missing_gap - leader_shift, 0, follower_room);
    plan->leader_shift = leader_side * leader_shift;
    plan->follower_shift = follower_side * follower_shift;
    plan->shortfall = missing_gap - leader_shift - follower_shift;
}

/* ------------------------------------------------------------------------------------------------
 * `plan_pass_kind` plans the pass past the leader of `pair` as it drives now. With `is_file`, it
 * plans the pass past the single file of the leader instead. The rear of that single file
 * reaches as far as `measure_file_back` says, and its bodies reach the radius of the widest
 * member to each side.
 * --------------------------------------------------------------------------------------------- */
static void plan_pass_kind(const OrderedPair *pair, bool is_file, PassPlan *plan) {
    const Squad *leader = pair->leader;
    const FormationShape *leader_shape = pair->leader_shape;
    if (is_file) {
        float file_back = measure_file_back(leader, leader_shape);
        float file_half_width = measure_max_radius(leader);
        plan_pass(pair, file_back, file_half_width, true, plan);
    } else {
        float leader_back = measure_back(leader, leader_shape);
        plan_pass(pair, leader_back, leader_shape->half_width, false, plan);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `plan_new_pass` plans a new pass past the formation of the leader. With `--leaderfile on`, it
 * plans a pass past the single file of the leader instead when the first plan leaves more than
 * `PASS_SQUEEZE_METERS` of the missing gap over.
 * Result: It returns true when the plan that it ends with leaves at most that much over, and sets
 *   `is_file` to whether that plan is past the single file.
 * --------------------------------------------------------------------------------------------- */
static bool plan_new_pass(const OrderedPair *pair, bool *is_file, PassPlan *plan) {
    *is_file = false;
    plan_pass_kind(pair, false, plan);
    bool has_room = plan->shortfall <= PASS_SQUEEZE_METERS;
    if (has_room || !is_leader_file_on)
        return has_room;
    *is_file = true;
    plan_pass_kind(pair, true, plan);
    return plan->shortfall <= PASS_SQUEEZE_METERS;
}

/* ------------------------------------------------------------------------------------------------
 * `has_squad_formed` returns true when the squad has changed its formation for a pass with the
 * shift `planned_shift`. That holds when its shift has moved `FORMED_FRACTION` of the way, and
 * every member is within `FORMED_LANE_METERS` of its lane point at its own progress.
 * --------------------------------------------------------------------------------------------- */
static bool has_squad_formed(const Squad *squad, float planned_shift) {
    /* Check that the shift has moved far enough. */
    float needed_shift = FORMED_FRACTION * fabsf(planned_shift);
    if (fabsf(squad->side_offset) < needed_shift)
        return false;

    /* Check that every member drives on its lane. */
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        V2 lane_point = get_lane_point(squad, unit, unit->route_progress);
        float lane_error = len(sub(unit->position, lane_point));
        if (lane_error > FORMED_LANE_METERS)
            return false;
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `cap_follower` makes the follower of `pair` keep `headway_meters` behind the rear of the leader:
 *
 *   [follower]>        gap        [leader]>
 *             |<----------------->|
 *           front                rear
 *
 *   The cap of the follower is the anchor speed of the leader plus `FOLLOW_GAIN` per m that the
 *   gap is longer than `headway_meters`, or minus that per m it is shorter, and at least 0.
 * --------------------------------------------------------------------------------------------- */
static void cap_follower(const OrderedPair *pair, float headway_meters) {
    Squad *leader = pair->leader;
    float leader_rear = get_leader_rear(pair);
    Squad *follower = pair->follower;
    float follower_front = get_follower_front(pair);
    float gap = leader_rear - follower_front;
    float follow_speed = leader->anchor_speed + FOLLOW_GAIN * (gap - headway_meters);
    float speed_cap = fmaxf(0, follow_speed);
    follower->coordination_speed_cap = fminf(follower->coordination_speed_cap, speed_cap);
}

/* ------------------------------------------------------------------------------------------------
 * `try_pass` lets the follower of `pair` pass its leader when it is faster and the pass has room.
 * Result: It returns true when the two squads shift to pass or a waiting pass holds. It returns
 *   false when the follower is not faster by `PASS_MIN_SPEED_GAIN`, when the pass is not near
 *   yet, or when the plan has no room and the pass does not wait already.
 * Timing: The pass is planned when the front of the follower would reach the gap
 *   `headway_meters` behind the rear of the leader within `PASS_LOOK_SECONDS` at the difference
 *   of their cruise speeds, or has reached it already.
 * Room: `try_pass` takes the plan of `plan_new_pass` when that leaves at most
 *   `PASS_SQUEEZE_METERS` of the missing gap over. Bodies then come at most that much closer
 *   than `HEAD_ON_MARGIN_METERS` at the narrowest spot. Both squads shift over the stretches of
 *   the plan. A pass that started in an earlier tick goes on in the same way even where the room
 *   ends. Short narrow spots then only bring the passing formations closer, and dropping back
 *   from beside the leader would lose more.
 * Waiting: With `--formfirst on`, a new pass first waits while the front of the follower is
 *   still `PASS_GAP_METERS` or more behind the rear of the leader. The leader shifts for it, but
 *   the follower keeps `headway_meters` behind the rear of the leader until `has_squad_formed`
 *   says that the leader is done. A waiting pass is planned anew in every tick. In a tick
 *   without room, it holds: the follower follows, and with `--smoothsway on` both squads keep
 *   their shifts. A pass that ended in such a tick would make the leader shift back and forth as
 *   narrow spots enter and leave the stretch ahead. Only a pass that no longer waits counts as
 *   started.
 * --------------------------------------------------------------------------------------------- */
static bool try_pass(const OrderedPair *pair, float headway_meters) {
    /* Check the speeds and how soon the follower catches up. */
    Squad *follower = pair->follower;
    Squad *leader = pair->leader;
    const FormationShape *follower_shape = pair->follower_shape;
    const FormationShape *leader_shape = pair->leader_shape;
    float speed_gain = follower_shape->cruise_speed - leader_shape->cruise_speed;
    if (speed_gain < PASS_MIN_SPEED_GAIN)
        return false;
    float leader_rear = get_leader_rear(pair);
    float follower_front = get_follower_front(pair);
    float catch_up_seconds = (leader_rear - headway_meters - follower_front) / speed_gain;
    if (catch_up_seconds > PASS_LOOK_SECONDS)
        return false;

    /* Plan the pass. A started pass goes on, and a new or waiting one needs room. A waiting pass
     * without room holds, and a leader in single file for it stays in single file. With
     * `--smoothsway on`, the shifts of both squads then stay, because `keep_shift_steady` keeps
     * the last shift of a squad that no pair shifted in this tick. */
    bool is_same_pass = follower->passing_index == leader->squad_index;
    bool is_waiting_pass = is_same_pass && follower->is_pass_waiting;
    bool is_continuing = is_same_pass && !follower->is_pass_waiting;
    bool is_file = false;
    bool has_room = true;
    PassPlan plan;
    if (is_continuing) {
        is_file = follower->is_file_pass;
        plan_pass_kind(pair, is_file, &plan);
    } else {
        has_room = plan_new_pass(pair, &is_file, &plan);
    }
    if (!has_room) {
        if (!is_waiting_pass)
            return false;
        follower->is_pass_kept = true;
        if (follower->is_file_pass)
            leader->is_file_forced = true;
        cap_follower(pair, headway_meters);
        return true;
    }

    /* Shift both squads, and mark the pass. A leader that the plan puts in single file forms it
     * through `is_file_forced`. */
    set_pass_shift(follower, plan.follower_start, plan.follower_end, plan.follower_shift);
    set_pass_shift(leader, plan.leader_start, plan.leader_end, plan.leader_shift);
    follower->passing_index = leader->squad_index;
    follower->is_pass_kept = true;
    follower->is_file_pass = is_file;
    if (is_file)
        leader->is_file_forced = true;

    /* With `--formfirst on`, let a new pass wait until the leader has formed for it. */
    if (!is_continuing) {
        bool is_behind = leader_rear - follower_front >= PASS_GAP_METERS;
        bool is_formed = has_squad_formed(leader, plan.leader_shift);
        bool is_waiting = is_form_first_on && is_behind && !is_formed;
        follower->is_pass_waiting = is_waiting;
        if (is_waiting)
            cap_follower(pair, headway_meters);
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `does_route_cross_leader` returns true when the route of the follower crosses over the route of
 * the leader ahead of the follower. That holds when, at the anchor of the follower, its route lies
 * at least `CROSS_SIDE_FRACTION` of `needed_gap` (the sideways gap that two formations need) to
 * one side of the route of the leader, and a later sample of its route lies at least as far to
 * the other side:
 *
 *                     follower route  ...o
 *     leader route   o----------------/--------->   (the other side, later)
 *                                 ../
 *          follower anchor   o../                   (one side, now)
 *
 * Two squads that drive along the same path, as in one valley, stay closer to each other's route
 * than that, so the follower can still pass there.
 * --------------------------------------------------------------------------------------------- */
static bool does_route_cross_leader(const OrderedPair *pair, float needed_gap) {
    /* Find the side of the leader route where the follower drives now. */
    const Squad *follower = pair->follower;
    const RouteSamples *follower_samples = pair->follower_samples;
    const RouteSamples *leader_samples = pair->leader_samples;
    float side_gap = CROSS_SIDE_FRACTION * needed_gap;
    V2 anchor_direction;
    V2 anchor_point = get_route_point(follower, follower->anchor_distance, &anchor_direction);
    float unused_distance;
    float start_lateral;
    bool is_located = locate_on_route(leader_samples, anchor_point, anchor_direction, LARGE_VALUE,
                                      true, &unused_distance, &start_lateral);
    if (!is_located || fabsf(start_lateral) < side_gap)
        return false;

    /* Look for a later sample of the follower route as far to the other side. */
    for (int sample_index = 0; sample_index < follower_samples->count; sample_index++) {
        if (follower_samples->distances[sample_index] <= follower->anchor_distance)
            continue;
        V2 point = follower_samples->points[sample_index];
        V2 direction = follower_samples->directions[sample_index];
        float lateral;
        locate_on_route(leader_samples, point, direction, LARGE_VALUE, true, &unused_distance,
                        &lateral);
        bool is_other_side = lateral * start_lateral < 0;
        if (is_other_side && fabsf(lateral) >= side_gap)
            return true;
    }
    return false;
}

/* ------------------------------------------------------------------------------------------------
 * `pass_or_follow` samples both routes around the squads and finds which squad follows
 * (`order_pair`). It lets `try_pass` pass first, and when that finds no room, it follows through
 * `cap_follower`. Both keep `headway_meters` behind the rear of the leader. That gap is the
 * distance that the anchor of the leader drives in `FOLLOW_HEADWAY_SECONDS`, but at least
 * `FOLLOW_MIN_GAP_METERS`.
 * With `--crossfollow on`, a follower whose route crosses over the route of the leader ahead
 * (`does_route_cross_leader`) starts no pass and follows. A pass beside a leader that the follower
 * must then cut across would drive it into the formation of the leader. Behind the leader, it
 * crosses once the leader is past. A pass that has started already goes on.
 * --------------------------------------------------------------------------------------------- */
bool pass_or_follow(Squad *first, Squad *second) {
    /* Sample both routes around the squads, and find which squad follows. */
    FormationShape first_shape = measure_shape(first);
    FormationShape second_shape = measure_shape(second);
    static RouteSamples first_samples;
    static RouteSamples second_samples;
    sample_route_around(first, &first_shape, &first_samples);
    sample_route_around(second, &second_shape, &second_samples);
    float needed_gap = first_shape.half_width + second_shape.half_width + HEAD_ON_MARGIN_METERS;
    OrderedPair pair;
    bool is_one_stretch = order_pair(first, &first_shape, &first_samples, second, &second_shape,
                                     &second_samples, needed_gap, &pair);
    if (!is_one_stretch)
        return false;

    /* Mark the follower, and pass, or else follow. */
    Squad *leader = pair.leader;
    pair.follower->is_following[leader->squad_index] = true;
    float headway_at_speed = FOLLOW_HEADWAY_SECONDS * leader->anchor_speed;
    float headway_meters = fmaxf(FOLLOW_MIN_GAP_METERS, headway_at_speed);
    bool is_same_pass = pair.follower->passing_index == leader->squad_index;
    bool is_started_pass = is_same_pass && !pair.follower->is_pass_waiting;
    bool may_pass = true;
    if (is_cross_follow_on && !is_started_pass)
        may_pass = !does_route_cross_leader(&pair, needed_gap);
    if (may_pass && try_pass(&pair, headway_meters))
        return true;
    cap_follower(&pair, headway_meters);
    return true;
}
