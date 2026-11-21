// Adaptation of moving squads to the paths of other moving squads in the coordination mode
// (`--moveahead`). Each tick, a squad predicts how long it would wait on its route for each other
// moving squad that does not give way to it. After a long predicted wait, it runs a trial search of
// its route, once per other squad and order, with stamps around the path of that squad. It
// re-routes only when the found route arrives clearly earlier and its predicted formation meets no
// predicted formation of a moving squad at any time. Without a re-route, the squad of the pair with
// the shorter useful wait slows down early, so that it reaches its first wait only once the waits
// are over.
//
// When two squads meet head-on, both squads search together. Each one keeps the side on which the
// other squad already lies, and searches around discs on the other side of its own route over the
// meeting section, so that it leaves its route by about half of the needed gap. At a crossing, no
// squad searches: one keeps its straight route and slows down. When two squads drive the same way,
// one squad searches around the corridor of the other squad. The stamps are discs that the trial
// stores in the squad, so that the search and the set-up of the re-route stamp the same discs
// (stamping.c). A trial route that strays from the current route by much more than the needed gap
// is rejected, so that a squad keeps its way around blocking terrain and stays near the squads
// that it travels with. A trial route that meets a third moving squad gets one more search, around
// the corridor of that squad.
//
//      corridor of the other squad:        discs on the own left, for a pass on the own right:
//         ( ( ( ( ( ( ( ( ( (              ( ( ( ( ( ( ( ( (     discs on the own left
//      ---o-o-o-o-o-o-o-o-o-o--- route     -----------------------  own route
//                                                new route passes here, on the own right
//
// The predictions move each squad along its route at its top speed, the top speed of its slowest
// squad member. A formation counts as a capsule: a segment along the route, between the route
// points one half width inside its rear and front ends, with the half width as its radius.
//
//      rear end                       front end
//         (  o-----------------------o  )       o: segment ends, half width inside
//
// A squad moves until it arrives. Then its squad members stand at their final parking positions
// for the rest of the prediction. A squad whose route they would block soon re-routes around the
// parking disc of that squad, without a trial. This covers the park ahead of `--parkreroute`
// (reroute.c), which then stays off. The members count one by one: as one disc, the final
// formation of a squad that parks next to the route end of another squad would block that end.
#include "sim.h"
#include "formation.h"

#define STEP_SECONDS 0.25f                // time step of the predictions
#define MAX_PREDICTION_SECONDS 120.0f     // longest prediction
#define MIN_PREDICTED_SPEED 0.5f          // slowest speed in the predictions
#define MIN_WAIT_SECONDS 2.0f             // shortest predicted wait that starts a re-route
#define MIN_YIELD_WAIT_SECONDS 0.5f       // shortest predicted wait that starts a speed cap
#define MIN_GAIN_SECONDS 1.0f             // shortest arrival gain of a new route
#define GAP_METERS 2.0f                   // body gap at which two formations count as meeting
#define TIME_MARGIN_SECONDS 0.5f          // lead and lag of the formation of another squad
#define CORRIDOR_MARGIN_SECONDS 2.0f      // corridor time before the first and after the last wait
#define CORRIDOR_MARGIN_METERS 1.0f       // corridor width past the formation of the other squad
#define STAMP_STEP_FRACTION 0.5f          // spacing of the stamp discs, in disc radii
#define TRIAL_SEARCH_BUDGET 500           // expansions of each trial search, as `--fullsearch`
#define TRIAL_SEARCH_TOLERANCE_METERS 0.0f        // the trial search ends on the destination
#define SAME_WAY_COSINE 0.7f              // limit of driving the same way, 45 degrees apart
#define HEAD_ON_COSINE -0.7f              // head-on direction limit, 135 degrees apart
#define DIRECTION_SPAN_METERS 1.0f        // route span before and after a point for its direction
#define DEVIATION_STEP_METERS 4.0f        // spacing of the trial route points in the deviation check
#define DEVIATION_MARGIN_CELLS 3.0f       // deviation past the needed gap, in grid cells
#define MAX_TRIAL_SEARCHES 2              // searches of a trial, each around one more met squad
#define PARKING_LOOK_SECONDS 16.0f        // look time for parked members, as the park ahead
#define WAIT_LOOK_SECONDS 10.0f           // latest start of a wait for a moving formation
#define EARLY_SIDE_STAMP_RADII 3.0f       // start of early side discs, in disc radii ahead
#define LATE_YIELD_SPEED_FRACTION 0.5f    // yield cap share of top speed that starts the slowdown
#define YIELD_HOLD_SECONDS 2.0f           // time a yield still counts as giving way after it ends
#define CLOSE_PASS_CELLS 0.5f             // search disc shrink of `--closepass`, in grid cells
#define CLOSE_PASS_SET_UP_CELLS 0.5f      // set-up disc shrink of `--closepass`, in grid cells
#define CLOSE_PASS_MIN_RADIUS_CELLS 2.0f  // smallest search disc radius of `--closepass`, in cells
#define CLOSE_PASS_MARGIN_METERS 4.0f     // shift kept past the lateral offset, with `--closepass`
#define CORRIDOR_STAMPS (MAX_MOVE_AHEAD_STAMPS / MAX_TRIAL_SEARCHES)        // discs per corridor


// A route as a polyline: its points and the route position of each point.
typedef struct {
    const V2 *points;
    const float *distances;
    int count;
} Polyline;


// The predicted motion of a squad along its route at the speed `speed`, from the current tick
// on: the route positions of the rear and front ends of its formation now, and how far it drives
// until it arrives.
typedef struct {
    Polyline route;
    float rear_start;
    float front_start;
    float remaining_meters;          // travel until the anchor reaches the route end
    float half_width;
    float speed;
    float lead_meters;               // reach added ahead of the front end and behind the rear end
    const Squad *parker;             // squad whose members park at the route end
} PredictedMotion;


// A segment from `start` to `end` with the radius `radius` around it.
typedef struct {
    V2 start;
    V2 end;
    float radius;
} Capsule;


// The predicted wait of a squad for the moving formation of another squad, the times from now at
// which its first and last waits start, its travel until the first wait, how long the first wait
// lasts without a break, whether the other squad would still meet it while it waits, as an
// oncoming squad on the same route would, whether the two squads meet head-on or crossing at the
// first wait, and how far the front end of the other formation then lies to the right of the
// front end of its own, as seen facing along its own route. Apart from the waits, whether and
// from when on the parked members of the other squad block it for good.
typedef struct {
    float wait_seconds;
    float first_wait_seconds;
    float last_wait_seconds;
    float first_wait_travel;
    float first_wait_length_seconds;
    bool is_met_while_waiting;
    bool is_head_on;
    bool is_crossing;
    float lateral_meters;
    bool is_blocked_by_parking;
    float blocked_seconds;
} WaitPrediction;


// A trial route from the front unit of a squad, and the route position of each of its points.
typedef struct {
    V2 points[MAX_WAYPOINTS];
    float distances[MAX_WAYPOINTS];
    int count;
} TrialRoute;

// Trial routes of one squad, or of the two squads of a pair search.
static TrialRoute trial_routes[2];

// Whether the last trial route of each squad that `is_route_close` checked strays farthest to the
// side other than its pass side, by squad index.
static bool is_route_on_wrong_side[MAX_SQUADS];


// Outcome of the checks of the routes of one trial search.
typedef enum {
    TRIAL_REJECTED,
    TRIAL_TAKEN,
    TRIAL_SEARCHED_AGAIN,        // a route met a moving squad, whose corridor is now stamped too
} TrialResult;


// `get_polyline_point` returns the point of `route` at the route position `distance`, clamped to
// the route.
static V2 get_polyline_point(const Polyline *route, float distance) {
    if (route->count < 2)
        return route->points[0];

    // The segment is the first one that ends at or past `distance`.
    int low_index = 1;
    int high_index = route->count - 1;
    while (low_index < high_index) {
        int middle_index = (low_index + high_index) / 2;
        if (route->distances[middle_index] < distance)
            low_index = middle_index + 1;
        else
            high_index = middle_index;
    }
    V2 from = route->points[low_index - 1];
    V2 segment = sub(route->points[low_index], from);
    float d_start = route->distances[low_index - 1];        // route position of the segment
    float length = route->distances[low_index] - d_start;
    if (length <= 0)
        return from;
    float fraction = clamp_float((distance - d_start) / length, 0, 1);
    return add(from, mul(segment, fraction));
}


// `get_formation_capsule` returns the capsule of the formation of `motion` after `travel` meters
// along its route.
static Capsule get_formation_capsule(const PredictedMotion *motion, float travel) {
    // A formation shorter than its width shrinks to a disc at its middle.
    float rear = motion->rear_start + travel - motion->lead_meters;
    float front = motion->front_start + travel + motion->lead_meters;
    float r = motion->half_width;        // capsule radius
    float segment_rear = rear + r;
    float segment_front = front - r;
    if (segment_rear > segment_front) {
        segment_rear = (rear + front) / 2;
        segment_front = segment_rear;
    }
    return (Capsule){
        .start = get_polyline_point(&motion->route, segment_rear),
        .end = get_polyline_point(&motion->route, segment_front),
        .radius = r,
    };
}


// `get_front_direction` returns the route direction under the front end of the formation of
// `motion` after `travel` meters, or a zero vector on a route without length there.
static V2 get_front_direction(const PredictedMotion *motion, float travel) {
    float front = motion->front_start + travel;
    V2 behind = get_polyline_point(&motion->route, front - DIRECTION_SPAN_METERS);
    V2 ahead = get_polyline_point(&motion->route, front + DIRECTION_SPAN_METERS);
    return get_direction(sub(ahead, behind), LENGTH_SQUARED_EPSILON, v2(0, 0));
}


// `predict_capsule` stores in `capsule` the capsule of the formation of `motion` after `seconds`.
// It returns false once the squad has arrived.
static bool predict_capsule(const PredictedMotion *motion, float seconds, Capsule *capsule) {
    float travel = motion->speed * seconds;
    if (travel > motion->remaining_meters)
        return false;
    *capsule = get_formation_capsule(motion, travel);
    return true;
}


// `do_segments_cross` tells whether two segments cross each other.
static bool do_segments_cross(V2 a_start, V2 a_end, V2 b_start, V2 b_end) {
    V2 a = sub(a_end, a_start);
    V2 b = sub(b_end, b_start);
    float b_start_side = det2(a, sub(b_start, a_start));
    float b_end_side = det2(a, sub(b_end, a_start));
    float a_start_side = det2(b, sub(a_start, b_start));
    float a_end_side = det2(b, sub(a_end, b_start));
    return b_start_side * b_end_side < 0 && a_start_side * a_end_side < 0;
}


// `measure_point_segment_distance` returns the distance from `point` to the segment from `a` to
// `b`.
static float measure_point_segment_distance(V2 point, V2 a, V2 b) {
    return len(sub(point, get_closest_segment_point(point, a, b)));
}


// `do_capsules_meet` tells whether the bodies of two capsules come closer than `GAP_METERS`. Two
// segments that do not cross are closest at an end of one of them.
static bool do_capsules_meet(const Capsule *first, const Capsule *second) {
    if (do_segments_cross(first->start, first->end, second->start, second->end))
        return true;
    float d = measure_point_segment_distance(first->start, second->start, second->end);
    d = fminf(d, measure_point_segment_distance(first->end, second->start, second->end));
    d = fminf(d, measure_point_segment_distance(second->start, first->start, first->end));
    d = fminf(d, measure_point_segment_distance(second->end, first->start, first->end));
    return d < first->radius + second->radius + GAP_METERS;
}


// `does_capsule_meet_parking` tells whether the body of `capsule` comes closer than `GAP_METERS`
// to the body of a squad member of `parker` at its final parking position.
static bool does_capsule_meet_parking(const Capsule *capsule, const Squad *parker) {
    int end_index = parker->first_unit_index + parker->unit_count;
    for (int unit_index = parker->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *member = &units[unit_index];
        float d = measure_point_segment_distance(member->parking_position, capsule->start,
                                                 capsule->end);
        if (d < capsule->radius + member->unit_type->radius + GAP_METERS)
            return true;
    }
    return false;
}


// `do_motions_meet` tells whether the formations of two predicted motions meet at any time step
// before one of the squads arrives. The final parking positions are left out: next to the route
// ends of the other squads, as in a line of squads, most routes would meet them.
static bool do_motions_meet(const PredictedMotion *first, const PredictedMotion *second) {
    for (float t = 0; t <= MAX_PREDICTION_SECONDS; t += STEP_SECONDS) {
        Capsule first_capsule;
        Capsule second_capsule;
        if (!predict_capsule(first, t, &first_capsule))
            return false;
        if (!predict_capsule(second, t, &second_capsule))
            return false;
        if (do_capsules_meet(&first_capsule, &second_capsule))
            return true;
    }
    return false;
}


// `predict_squad_motion` predicts the motion of a moving squad along its own route. The lead and
// lag of `TIME_MARGIN_SECONDS` cover the speed changes that the prediction leaves out.
static PredictedMotion predict_squad_motion(const Squad *squad) {
    FormationShape shape = measure_shape(squad);
    float v = fmaxf(MIN_PREDICTED_SPEED, shape.squad_top_speed);        // predicted speed
    PredictedMotion motion = {
        .route = {
            .points = squad->route,
            .distances = squad->route_distance,
            .count = squad->route_count,
        },
        .rear_start = shape.rear_progress,
        .front_start = squad->anchor_distance + shape.front_meters,
        .remaining_meters = squad->route_length - squad->anchor_distance,
        .half_width = shape.half_width,
        .speed = v,
        .lead_meters = v * TIME_MARGIN_SECONDS,
        .parker = squad,
    };
    return motion;
}


// `predict_own_motion` predicts the motion of `squad` along its own route, without lead and lag,
// for the wait that it predicts for itself.
static PredictedMotion predict_own_motion(const Squad *squad) {
    PredictedMotion motion = predict_squad_motion(squad);
    motion.lead_meters = 0;
    return motion;
}


// `predict_route_motion` predicts the motion of `squad` along the trial route `route`, which
// starts at its front unit. As after the set-up of a new route, the anchor starts again at the
// route start, and the formation keeps its current reach before and behind the anchor.
static PredictedMotion predict_route_motion(const Squad *squad, const TrialRoute *route) {
    PredictedMotion own = predict_own_motion(squad);
    own.route = (Polyline){
        .points = route->points,
        .distances = route->distances,
        .count = route->count,
    };
    own.rear_start -= squad->anchor_distance;
    own.front_start -= squad->anchor_distance;
    own.remaining_meters = route->distances[route->count - 1];
    return own;
}


// `describe_meeting` stores in `prediction` how the formations of `own`, after `travel` meters, and
// of `other`, after `seconds`, meet: head-on, crossing or the same way, and how far the front end
// of `other` lies to the right of the front end of `own`.
static void describe_meeting(const PredictedMotion *own, const PredictedMotion *other,
                             float travel, float seconds, WaitPrediction *prediction) {
    float other_travel = other->speed * seconds;
    V2 own_direction = get_front_direction(own, travel);
    V2 other_direction = get_front_direction(other, other_travel);
    float cosine = dot(own_direction, other_direction);        // of the angle between them
    prediction->is_head_on = cosine < HEAD_ON_COSINE;
    prediction->is_crossing = !prediction->is_head_on && cosine < SAME_WAY_COSINE;
    V2 own_front = get_polyline_point(&own->route, own->front_start + travel);
    V2 other_front = get_polyline_point(&other->route, other->front_start + other_travel);
    prediction->lateral_meters = dot(sub(other_front, own_front), get_right(own_direction));
}


// `predict_wait` predicts how long the formation of `own` would wait for the formation of `other`
// until it arrives: in each time step, it drives on at its speed unless it would then meet the
// formation of `other`, and waits otherwise. After the arrival of `other`, a meeting with its
// parked members ends the prediction, as `own` would wait for good.
static WaitPrediction predict_wait(const PredictedMotion *own, const PredictedMotion *other) {
    WaitPrediction prediction = {
        .wait_seconds = 0,
        .first_wait_seconds = 0,
        .last_wait_seconds = 0,
        .first_wait_travel = 0,
        .first_wait_length_seconds = 0,
        .is_met_while_waiting = false,
        .is_head_on = false,
        .is_crossing = false,
        .lateral_meters = 0,
        .is_blocked_by_parking = false,
        .blocked_seconds = 0,
    };
    float travel = 0;                    // travel of `own`
    bool is_first_wait_over = false;     // whether `own` drove on after its first wait
    for (float t = STEP_SECONDS; t <= MAX_PREDICTION_SECONDS; t += STEP_SECONDS) {
        if (travel >= own->remaining_meters)
            break;
        float next_travel = travel + own->speed * STEP_SECONDS;
        Capsule own_capsule = get_formation_capsule(own, next_travel);
        Capsule other_capsule;
        bool is_other_moving = predict_capsule(other, t, &other_capsule);
        bool is_blocked = !is_other_moving
                          && does_capsule_meet_parking(&own_capsule, other->parker);
        bool is_met = is_other_moving && do_capsules_meet(&own_capsule, &other_capsule);
        if (is_blocked) {
            prediction.blocked_seconds = t;
            prediction.is_blocked_by_parking = true;
            break;
        }
        if (!is_met) {
            travel = next_travel;
            is_first_wait_over = prediction.wait_seconds > 0;
            continue;
        }
        if (prediction.wait_seconds == 0) {
            prediction.first_wait_seconds = t;
            prediction.first_wait_travel = travel;
            describe_meeting(own, other, travel, t, &prediction);
        }
        if (!is_first_wait_over)
            prediction.first_wait_length_seconds += STEP_SECONDS;
        prediction.last_wait_seconds = t;
        prediction.wait_seconds += STEP_SECONDS;

        // A wait helps only when the waiting formation stays clear of the other formation.
        Capsule waiting_capsule = get_formation_capsule(own, travel);
        if (do_capsules_meet(&waiting_capsule, &other_capsule))
            prediction.is_met_while_waiting = true;
    }
    return prediction;
}


// `can_wait_usefully` tells whether the squad of `prediction` would wait for the moving formation
// of the other squad and stay clear of it while it waits.
static bool can_wait_usefully(const WaitPrediction *prediction) {
    return prediction->wait_seconds > 0 && !prediction->is_met_while_waiting;
}


// `get_yield_wait_seconds` returns the predicted wait of `prediction` when the squad can wait
// usefully, or `LARGE_VALUE` otherwise.
static float get_yield_wait_seconds(const WaitPrediction *prediction) {
    if (!can_wait_usefully(prediction))
        return LARGE_VALUE;
    return prediction->wait_seconds;
}


// `is_giving_way_to` tells whether `squad` gives way to `other` in this tick: it adapts its speed
// to it, or did so less than `YIELD_HOLD_SECONDS` ago, and does not ignore the coordination.
//
// The yield of a squad ends when its predicted wait drops out, which can last only a few ticks.
// Without the hold, the other squad could take the yield in those ticks, the roles could swap back
// and forth, and both squads would slow down.
static bool is_giving_way_to(const Squad *squad, const Squad *other) {
    float yield_age_seconds = now_seconds - squad->last_yield_seconds[other->squad_index];
    bool is_yielding = squad->yield_to_index == other->squad_index;
    bool has_yielded_lately = yield_age_seconds < YIELD_HOLD_SECONDS;
    bool is_ignoring = squad->ignore_coordination_seconds > 0;
    return !is_ignoring && (is_yielding || has_yielded_lately);
}


// `find_front_member` returns the squad member of `squad` closest to its destination, where its
// route searches start.
static const Unit *find_front_member(const Squad *squad) {
    const Unit *front = &units[squad->first_unit_index];
    float best_squared = length_squared(sub(front->position, squad->destination));
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *member = &units[unit_index];
        float d_squared = length_squared(sub(member->position, squad->destination));
        if (d_squared < best_squared) {
            front = member;
            best_squared = d_squared;
        }
    }
    return front;
}


// `get_corridor_range` returns in `from_distance` and `to_distance` the route positions of the
// formation of `other`, with the predicted motion `other_motion`, from `CORRIDOR_MARGIN_SECONDS`
// before the first wait of `prediction` until as long after its last wait.
static void get_corridor_range(const Squad *other, const PredictedMotion *other_motion,
                               const WaitPrediction *prediction, float *from_distance,
                               float *to_distance) {
    float start_seconds = fmaxf(0, prediction->first_wait_seconds - CORRIDOR_MARGIN_SECONDS);
    float end_seconds = prediction->last_wait_seconds + CORRIDOR_MARGIN_SECONDS;
    float rear = other_motion->rear_start + other_motion->speed * start_seconds;
    float front = other_motion->front_start + other_motion->speed * end_seconds;
    *from_distance = clamp_float(rear, 0, other->route_length);
    *to_distance = clamp_float(front, *from_distance, other->route_length);
}


// `add_stamps` adds to the stamp discs of `squad` discs along the route of `other`, from
// `from_distance` to `to_distance`, each moved by `side_offset` to the right of that route as seen
// facing along it. The discs get the radius `radius` in the route searches and `set_up_radius` in
// the set-up. Each call adds at most `CORRIDOR_STAMPS` discs, so the spacing grows on long spans.
static void add_stamps(Squad *squad, const Squad *other, float from_distance, float to_distance,
                       float side_offset, float radius, float set_up_radius) {
    float span = to_distance - from_distance;
    float min_step = span / (CORRIDOR_STAMPS - 1);
    float step = fmaxf(STAMP_STEP_FRACTION * radius, min_step);
    int end_count = squad->move_ahead_stamp_count + CORRIDOR_STAMPS;        // count limit
    float d = from_distance;        // route position of a disc
    while (squad->move_ahead_stamp_count < end_count) {
        V2 direction;
        V2 point = get_route_point(other, fminf(d, to_distance), &direction);
        V2 offset = mul(get_right(direction), side_offset);
        int disc_index = squad->move_ahead_stamp_count;
        squad->move_ahead_stamp_centers[disc_index] = add(point, offset);
        squad->move_ahead_stamp_radii[disc_index] = radius;
        squad->move_ahead_set_up_stamp_radii[disc_index] = set_up_radius;
        squad->move_ahead_stamp_count++;
        if (d >= to_distance)
            break;
        d += step;
    }
}


// `add_corridor_stamps` adds to the stamp discs of `squad` the corridor of the moving squad
// `other`, with the predicted motion `other_motion`, for the wait of `prediction`: discs along its
// route, `CORRIDOR_MARGIN_METERS` wider than its formation.
static void add_corridor_stamps(Squad *squad, const Squad *other,
                                const PredictedMotion *other_motion,
                                const WaitPrediction *prediction) {
    float from_distance;
    float to_distance;
    get_corridor_range(other, other_motion, prediction, &from_distance, &to_distance);
    float radius = other_motion->half_width + CORRIDOR_MARGIN_METERS;
    add_stamps(squad, other, from_distance, to_distance, 0, radius, radius);
}


// `search_trial_route` runs the route search of a re-route of `squad` at once, with the stamps of
// that re-route, and stores the found route, from the front unit, in `route`. It returns false
// when the search finds no path.
static bool search_trial_route(const Squad *squad, TrialRoute *route) {
    const Unit *front = find_front_member(squad);
    GcRequest search;
    bool is_found = search_path(front, squad->destination, TRIAL_SEARCH_TOLERANCE_METERS,
                                TRIAL_SEARCH_BUDGET, &search);
    int point_count = 0;        // number of path points
    if (is_found)
        point_count = gc_get_path(&search, path_points, MAX_WAYPOINTS - 1);
    gc_free_request(&search);
    if (point_count == 0)
        return false;

    route->points[0] = front->position;
    route->distances[0] = 0;
    for (int point_index = 1; point_index <= point_count; point_index++) {
        route->points[point_index] = get_pair_point(path_points, point_index - 1);
        V2 previous = route->points[point_index - 1];
        float piece = len(sub(route->points[point_index], previous));        // segment length
        route->distances[point_index] = route->distances[point_index - 1] + piece;
    }
    route->count = point_count + 1;
    return true;
}


// `can_either_wait` tells whether the formation of `own` or that of `other_motion` could wait for
// the other one without being met while it waits.
static bool can_either_wait(const PredictedMotion *own, const PredictedMotion *other_motion) {
    WaitPrediction own_prediction = predict_wait(own, other_motion);
    PredictedMotion other_own = *other_motion;
    other_own.lead_meters = 0;
    PredictedMotion own_with_lead = *own;
    own_with_lead.lead_meters = own->speed * TIME_MARGIN_SECONDS;
    WaitPrediction other_prediction = predict_wait(&other_own, &own_with_lead);
    return !own_prediction.is_met_while_waiting || !other_prediction.is_met_while_waiting;
}


// `avoid_met_squad` finds the first moving squad, other than `squad` and the squad `skipped_index`,
// whose formation meets the formation of `own`, and returns its index, or `NO_SQUAD`. With
// `is_waiting_allowed`, a squad counts only when neither formation could wait for the other and
// the two do not meet head-on. With `is_searched_again`, it adds the corridor of the found squad to
// the stamps of `squad` for the next trial search. Settled squads count as parked, so the search
// already went around them.
static int avoid_met_squad(Squad *squad, const PredictedMotion *own, int skipped_index,
                           bool is_waiting_allowed, bool is_searched_again) {
    for (int other_index = 0; other_index < squad_count; other_index++) {
        const Squad *other = &squads[other_index];
        bool is_moving_other = other != squad && other->state == SQUAD_MOVING;
        bool is_checked_other = is_moving_other && !other->is_settled;
        is_checked_other = is_checked_other && other_index != skipped_index;
        if (!is_checked_other || !are_squads_coordinated(squad, other))
            continue;
        PredictedMotion other_motion = predict_squad_motion(other);
        if (!do_motions_meet(own, &other_motion))
            continue;
        if (is_waiting_allowed && can_either_wait(own, &other_motion))
            continue;

        // A head-on meeting with a third squad is left to the trials of the next ticks, where a
        // pair trial with that squad can move both routes, each to its own side. Without that, a
        // squad that drives head-on into two squads could re-route only alone.
        WaitPrediction prediction = predict_wait(own, &other_motion);
        if (is_waiting_allowed && prediction.is_head_on)
            continue;
        if (is_searched_again)
            add_corridor_stamps(squad, other, &other_motion, &prediction);
        return other_index;
    }
    return NO_SQUAD;
}


// `measure_polyline_offset` returns the distance from `point` to the polyline `route`, positive
// when `point` lies to the right of its closest segment, as seen facing along `route`.
static float measure_polyline_offset(V2 point, const Polyline *route) {
    float d_min = LARGE_VALUE;        // smallest distance to a segment
    float side = 1;                   // 1 to the right of that segment, -1 to its left
    for (int point_index = 1; point_index < route->count; point_index++) {
        V2 a = route->points[point_index - 1];
        V2 b = route->points[point_index];
        float d = measure_point_segment_distance(point, a, b);
        if (d >= d_min)
            continue;
        d_min = d;
        side = det2(sub(b, a), sub(point, a)) > 0 ? -1 : 1;
    }
    return side * d_min;
}


// `is_route_close` tells whether the trial route `route` of `squad` stays within the needed gap
// of `own` and `other_motion`, plus `DEVIATION_MARGIN_CELLS`, of the current route of `squad`. The
// margin covers the stamped cells that reach past the discs and the corner gap of the search.
// With a `pass_side` of 1 or -1, the route must also stray farthest to the right or to the left.
static bool is_route_close(const Squad *squad, const TrialRoute *route,
                           const PredictedMotion *own, const PredictedMotion *other_motion,
                           float pass_side) {
    Polyline current = {
        .points = squad->route,
        .distances = squad->route_distance,
        .count = squad->route_count,
    };
    Polyline trial = {
        .points = route->points,
        .distances = route->distances,
        .count = route->count,
    };
    float needed_gap = own->half_width + other_motion->half_width + GAP_METERS;
    float max_deviation = needed_gap + DEVIATION_MARGIN_CELLS * GC_CELL_METERS;
    float length = route->distances[route->count - 1];
    float farthest = 0;        // offset of the trial route point farthest from the current route
    for (float d = 0; d <= length; d += DEVIATION_STEP_METERS) {
        V2 point = get_polyline_point(&trial, d);
        float offset = measure_polyline_offset(point, &current);
        if (fabsf(offset) > fabsf(farthest))
            farthest = offset;
    }
    bool is_wrong_side = farthest * pass_side < 0;
    is_route_on_wrong_side[squad->squad_index] = is_wrong_side;
    return fabsf(farthest) <= max_deviation && !is_wrong_side;
}


// `get_arrival_gain_seconds` returns how much earlier the formation of `own` on a trial route of
// `squad` arrives than its predicted arrival on its old route.
static float get_arrival_gain_seconds(const Squad *squad, const PredictedMotion *own) {
    float new_arrival_seconds = now_seconds + own->remaining_meters / own->speed;
    return squad->old_route_arrival_seconds - new_arrival_seconds;
}


// `is_route_earlier` tells whether the formation of `own` on a trial route of `squad` arrives
// `MIN_GAIN_SECONDS` before its predicted arrival on its old route.
static bool is_route_earlier(const Squad *squad, const PredictedMotion *own) {
    float new_arrival_seconds = now_seconds + own->remaining_meters / own->speed;
    float latest_seconds = squad->old_route_arrival_seconds - MIN_GAIN_SECONDS;
    return new_arrival_seconds <= latest_seconds;
}


// `begin_trial` marks `squad` as re-routing around moving squads for a trial search, with
// `wait_seconds` of predicted wait on its old route, and clears its stamp discs. It returns
// whether it was re-routing before.
static bool begin_trial(Squad *squad, const PredictedMotion *own, float wait_seconds) {
    float free_seconds = own->remaining_meters / own->speed;        // arrival without waiting
    squad->old_route_arrival_seconds = now_seconds + free_seconds + wait_seconds;
    bool was_rerouting = squad->is_rerouting;
    squad->is_rerouting = true;
    squad->is_move_ahead_reroute = true;
    squad->move_ahead_stamp_count = 0;
    return was_rerouting;
}


// `end_rejected_trial` leaves `squad` as it was before a rejected trial.
static void end_rejected_trial(Squad *squad, bool was_rerouting) {
    squad->is_rerouting = was_rerouting;
    squad->is_move_ahead_reroute = false;
}


// `check_single_route` runs a trial search of `squad` and checks the found route: it must stay
// close to the current route, on `pass_side` when it is not 0, arrive earlier, and meet no moving
// squad. With `is_searched_again`, a route that meets a moving squad gets another search around
// the corridor of that squad.
static TrialResult check_single_route(Squad *squad, const PredictedMotion *own,
                                      const PredictedMotion *other_motion, float pass_side,
                                      bool is_searched_again) {
    TrialRoute *route = &trial_routes[0];
    bool is_usable = search_trial_route(squad, route);
    is_usable = is_usable && is_route_close(squad, route, own, other_motion, pass_side);
    if (!is_usable)
        return TRIAL_REJECTED;
    PredictedMotion route_motion = predict_route_motion(squad, route);
    if (!is_route_earlier(squad, &route_motion))
        return TRIAL_REJECTED;
    if (avoid_met_squad(squad, &route_motion, NO_SQUAD, false, is_searched_again) == NO_SQUAD)
        return TRIAL_TAKEN;
    return is_searched_again ? TRIAL_SEARCHED_AGAIN : TRIAL_REJECTED;
}


// `check_pair_routes` runs the trial searches of the pair `squad` and `other`, with the
// predicted motions `own` and `other_own` on their current routes, and checks the found routes:
// each must stay close to its current route and on `pass_side`, together they must arrive
// earlier, and they must meet neither each other nor another moving squad. With
// `is_searched_again`, a route that meets a third squad gets another search around the corridor
// of that squad.
static TrialResult check_pair_routes(Squad *squad, Squad *other, const PredictedMotion *own,
                                     const PredictedMotion *other_own, float pass_side,
                                     bool is_searched_again) {
    TrialRoute *route = &trial_routes[0];
    TrialRoute *other_route = &trial_routes[1];
    bool is_usable = search_trial_route(squad, route);
    is_usable = is_usable && search_trial_route(other, other_route);
    is_usable = is_usable && is_route_close(squad, route, own, other_own, pass_side);
    is_usable = is_usable && is_route_close(other, other_route, other_own, own, pass_side);
    if (!is_usable)
        return TRIAL_REJECTED;

    // The wait of the other squad can be shorter than `MIN_WAIT_SECONDS`, so only the sum of the
    // gains has to reach `MIN_GAIN_SECONDS`.
    PredictedMotion route_motion = predict_route_motion(squad, route);
    PredictedMotion other_route_motion = predict_route_motion(other, other_route);
    float gain = get_arrival_gain_seconds(squad, &route_motion);
    gain += get_arrival_gain_seconds(other, &other_route_motion);
    if (gain < MIN_GAIN_SECONDS || do_motions_meet(&route_motion, &other_route_motion))
        return TRIAL_REJECTED;

    // A third squad that one side could wait for counts as passed, as the yield of the next ticks
    // handles it. Without that, a squad that crosses the paths of both would often leave the
    // meeting to a single re-route. Both routes are checked, so that each one that meets a third
    // squad gets its corridor.
    bool is_waiting_allowed = true;        // whether a waitable meeting counts as passed
    int met_index = avoid_met_squad(squad, &route_motion, other->squad_index, is_waiting_allowed,
                                    is_searched_again);
    int other_met_index = avoid_met_squad(other, &other_route_motion, squad->squad_index,
                                          is_waiting_allowed, is_searched_again);
    if (met_index == NO_SQUAD && other_met_index == NO_SQUAD)
        return TRIAL_TAKEN;
    return is_searched_again ? TRIAL_SEARCHED_AGAIN : TRIAL_REJECTED;
}


// `run_trial_searches` runs the trial searches of `squad`, alone when `pair_other` is `NULL` and
// otherwise with the pair squad `pair_other`, until the trial is taken or rejected, at
// most `MAX_TRIAL_SEARCHES` times. The routes must stray to `pass_side` unless it is 0. It
// returns true when the trial is taken.
static bool run_trial_searches(Squad *squad, Squad *pair_other, const PredictedMotion *own,
                               const PredictedMotion *other_motion, float pass_side) {
    TrialResult result = TRIAL_SEARCHED_AGAIN;
    for (int search_index = 0; result == TRIAL_SEARCHED_AGAIN; search_index++) {
        bool is_again = search_index + 1 < MAX_TRIAL_SEARCHES;        // is searched again
        if (pair_other)
            result = check_pair_routes(squad, pair_other, own, other_motion, pass_side, is_again);
        else
            result = check_single_route(squad, own, other_motion, pass_side, is_again);
    }
    return result == TRIAL_TAKEN;
}


// `get_pass_side` returns the side on which a squad passes the other squad of a head-on meeting
// of `prediction`: 1 for its right, -1 for its left. It keeps the side on which the other squad
// already lies. Seen from either squad, the other one lies on the same side, so both agree.
static float get_pass_side(const WaitPrediction *prediction) {
    return prediction->lateral_meters > 0 ? -1 : 1;
}


// `get_pass_shift` returns by how much two squads that meet head-on must move apart in sum, for
// the needed gap `needed_gap` between their routes. With `--closepass`, it leaves out the lateral
// offset `lateral_meters` that the other squad already has from the own route at the meeting,
// less `CLOSE_PASS_MARGIN_METERS`. Without it, squads that already drive side by side would move
// apart by the whole needed gap. Without the margin, more squads met in close quarters.
static float get_pass_shift(float needed_gap, float lateral_meters) {
    if (!is_close_pass_on)
        return needed_gap;
    float shift = needed_gap - fabsf(lateral_meters) + CLOSE_PASS_MARGIN_METERS;
    return clamp_float(shift, 0, needed_gap);
}


// `add_side_stamps` adds to the stamp discs of `squad`, with the predicted motion `motion`, the
// discs that move its route by `shift` to `pass_side` over the section where its formation would
// be during the waits of `prediction`. The discs lie `shift` to the other side of its own route,
// and the search keeps about the corner gap from their cells, so their search radius is twice
// the shift less the corner gap. The smoothing of the set-up keeps the formation
// `LANE_MARGIN_METERS` clear of the discs, so their set-up radius is twice the shift less the
// half width and that margin. With the search radius, the smoothing would move the route out by
// about another formation width. With `is_early_start`, the discs start
// `EARLY_SIDE_STAMP_RADII` disc radii ahead of the front end of the formation.
static void add_side_stamps(Squad *squad, const PredictedMotion *motion,
                            const WaitPrediction *prediction, float pass_side, float shift,
                            bool is_early_start) {
    float center_offset = shift;        // distance of the disc centers from the own route
    float radius = 2 * shift - get_corner_gap(find_front_member(squad));
    float set_up_radius = 2 * shift - (motion->half_width + LANE_MARGIN_METERS);

    // The search and the set-up treat every grid cell that a disc touches as blocked, so the
    // route could pass up to a whole cell farther out than the disc edge. With `--closepass`,
    // the discs shrink by `CLOSE_PASS_CELLS` and `CLOSE_PASS_SET_UP_CELLS` cells to make up for
    // part of that. A search disc keeps a radius of at least one cell, with its inner edge in
    // place: for a small shift, it would vanish, and the search could stray to the wrong side.
    if (is_close_pass_on) {
        float shrink = CLOSE_PASS_CELLS * GC_CELL_METERS;
        float set_up_shrink = CLOSE_PASS_SET_UP_CELLS * GC_CELL_METERS;
        float edge = radius - center_offset - shrink;        // inner disc edge past the route
        float set_up_edge = set_up_radius - center_offset - set_up_shrink;
        radius = fmaxf(center_offset + edge, CLOSE_PASS_MIN_RADIUS_CELLS * GC_CELL_METERS);
        center_offset = radius - edge;
        set_up_radius = center_offset + set_up_edge;
    }
    float from_distance;
    float to_distance;
    get_corridor_range(squad, motion, prediction, &from_distance, &to_distance);

    // The discs start a disc radius ahead of the front end of the formation. Over the squad
    // members, they would make the search leave them on the nearest side, which can be the
    // wrong one.
    from_distance = fmaxf(from_distance, motion->front_start + radius);
    if (is_early_start)
        from_distance = motion->front_start + EARLY_SIDE_STAMP_RADII * radius;
    to_distance = fmaxf(to_distance, from_distance);
    add_stamps(squad, squad, from_distance, to_distance, -pass_side * center_offset, radius,
               set_up_radius);
}


// `try_route_around` runs the trial search of a re-route of `squad` around the squad `other`, for
// which it would wait as `prediction` says, and starts the re-route when the trial route passes
// the checks. Head-on, the route moves by the whole needed gap to the side on which `squad`
// passes; otherwise, it goes around the corridor of `other`. It returns true when the re-route
// started.
static bool try_route_around(Squad *squad, const Squad *other, const PredictedMotion *own,
                             const PredictedMotion *other_motion,
                             const WaitPrediction *prediction) {
    bool was_rerouting = begin_trial(squad, own, prediction->wait_seconds);
    float pass_side = 0;        // side that the route must stray to, 0 for either side
    if (prediction->is_head_on) {
        pass_side = get_pass_side(prediction);
        float needed_gap = own->half_width + other_motion->half_width + GAP_METERS;
        float shift = get_pass_shift(needed_gap, prediction->lateral_meters);
        PredictedMotion squad_motion = predict_squad_motion(squad);
        add_side_stamps(squad, &squad_motion, prediction, pass_side, shift, false);
    } else {
        add_corridor_stamps(squad, other, other_motion, prediction);
    }
    bool is_taken = run_trial_searches(squad, NULL, own, other_motion, pass_side);
    if (!is_taken) {
        end_rejected_trial(squad, was_rerouting);
        return false;
    }
    start_route_search(squad);
    return true;
}


// `try_pair_routes` runs the trial searches of a head-on meeting of `squad` and `other`, each
// moving its route by half of the needed gap to the side on which it passes, and starts both
// re-routes when both trial routes pass the checks, against each other and against the other
// moving squads. It returns true when the re-routes started.
static bool try_pair_routes(Squad *squad, Squad *other, const PredictedMotion *own,
                            const PredictedMotion *other_motion,
                            const WaitPrediction *prediction) {
    // Both squads must see the meeting.
    PredictedMotion other_own = predict_own_motion(other);
    PredictedMotion squad_motion = predict_squad_motion(squad);
    WaitPrediction other_prediction = predict_wait(&other_own, &squad_motion);
    if (other_prediction.wait_seconds <= 0)
        return false;

    // The squads pass on the side on which they already are, or on the other side when the
    // terrain leaves no room there.
    // Each squad moves by half of the shift, which leaves out the smaller of the lateral offsets
    // that the two predictions see.
    float needed_gap = own->half_width + other_own.half_width + GAP_METERS;
    float lateral = fminf(fabsf(prediction->lateral_meters),
                          fabsf(other_prediction.lateral_meters));
    float half_gap = get_pass_shift(needed_gap, lateral) / 2;
    float first_side = get_pass_side(prediction);
    bool was_rerouting = squad->is_rerouting;
    bool was_other_rerouting = other->is_rerouting;
    bool is_taken = false;
    for (int side_index = 0; side_index < 2 && !is_taken; side_index++) {
        float pass_side = side_index == 0 ? first_side : -first_side;
        is_route_on_wrong_side[squad->squad_index] = false;
        is_route_on_wrong_side[other->squad_index] = false;
        bool is_early_start = false;              // whether the discs of `squad` start early
        bool is_other_early_start = false;        // whether the discs of `other` start early
        for (int attempt_index = 0; attempt_index < 2 && !is_taken; attempt_index++) {
            begin_trial(squad, own, prediction->wait_seconds);
            begin_trial(other, &other_own, other_prediction.wait_seconds);
            add_side_stamps(squad, &squad_motion, prediction, pass_side, half_gap,
                            is_early_start);
            add_side_stamps(other, other_motion, &other_prediction, pass_side, half_gap,
                            is_other_early_start);
            is_taken = run_trial_searches(squad, other, own, &other_own, pass_side);

            // The base game search is greedy. When the discs start far ahead of the front unit,
            // it can pass them on the far side. A squad whose route strays to the wrong side
            // searches once more, with discs that start closer ahead of its front.
            is_early_start = is_route_on_wrong_side[squad->squad_index];
            is_other_early_start = is_route_on_wrong_side[other->squad_index];
            if (!is_early_start && !is_other_early_start)
                break;
        }
    }
    if (!is_taken) {
        end_rejected_trial(squad, was_rerouting);
        end_rejected_trial(other, was_other_rerouting);
        return false;
    }
    squad->is_avoiding_moving[other->squad_index] = true;
    other->is_avoiding_moving[squad->squad_index] = true;
    start_route_search(squad);
    start_route_search(other);
    return true;
}


// `reroute_around_parking` starts a re-route of `squad` around the parking disc of the moving squad
// `other`, whose parked members would block its route (stamping.c).
static void reroute_around_parking(Squad *squad, const Squad *other) {
    squad->is_avoiding_parking[other->squad_index] = true;
    squad->is_rerouting = true;
    squad->is_move_ahead_reroute = false;
    start_route_search(squad);
}


// `is_yielding_to` tells whether `squad`, with the predicted wait `own_prediction` for `other`, is
// the squad of the pair that adapts: the one with the shorter useful wait (`get_yield_wait_seconds`),
// as the other squad would wait for it, or the one with the higher squad index for equal waits.
static bool is_yielding_to(const Squad *squad, const Squad *other,
                           const WaitPrediction *own_prediction) {
    PredictedMotion other_own = predict_own_motion(other);
    PredictedMotion squad_motion = predict_squad_motion(squad);
    WaitPrediction other_prediction = predict_wait(&other_own, &squad_motion);
    float own_wait = get_yield_wait_seconds(own_prediction);
    float other_wait = get_yield_wait_seconds(&other_prediction);
    if (own_wait != other_wait)
        return own_wait < other_wait;
    return squad->squad_index > other->squad_index;
}


// `try_reroutes` runs the trial searches of `squad` for the squad `other`, once per pair and order:
// head-on, both squads together and then `squad` alone; the same way, `squad` alone. At a
// crossing, it runs none: a squad that keeps its straight route and slows down is preferred over
// bends that lead away from the destination. It returns true when a re-route started.
static bool try_reroutes(Squad *squad, Squad *other, const PredictedMotion *own,
                         const PredictedMotion *other_motion,
                         const WaitPrediction *prediction) {
    if (prediction->is_crossing)
        return false;
    bool can_pair = prediction->is_head_on;
    can_pair = can_pair && !other->is_avoiding_moving[squad->squad_index];
    if (can_pair && try_pair_routes(squad, other, own, other_motion, prediction))
        return true;
    squad->is_avoiding_moving[other->squad_index] = true;
    return try_route_around(squad, other, own, other_motion, prediction);
}


// `avoid_moving_ahead` lets a moving squad adapt to each other moving squad for which it would
// wait and that does not adapt to it. After a wait of at least `MIN_WAIT_SECONDS`, it re-routes
// when a trial search finds a better route, once per other squad and order. Otherwise, after a wait
// of at least `MIN_YIELD_WAIT_SECONDS`, the squad of the pair that adapts (`is_yielding_to`) gets a
// speed cap that brings it to its first predicted wait only once that wait is over.
//
// A squad keeps adapting to that squad until it would no longer wait for it. Otherwise, its wait
// would shrink as it slows down, the decision could flip, and neither squad would adapt. It runs at
// most one trial per tick, and the coordination applies the cap in the next tick (coordination.c).
void avoid_moving_ahead(Squad *squad) {
    squad->yield_speed_cap = LARGE_VALUE;
    bool is_used = is_move_ahead_on && is_park_reroute_on && is_park_robust_on;
    if (!is_used || avoid_mode != AVOID_COORD || squad->state != SQUAD_MOVING) {
        squad->yield_to_index = NO_SQUAD;
        return;
    }

    PredictedMotion own = predict_own_motion(squad);
    bool has_tried = false;        // whether a trial search ran in this tick
    bool is_yield_kept = false;    // whether the squad still adapts to `yield_to_index`
    for (int other_index = 0; other_index < squad_count; other_index++) {
        Squad *other = &squads[other_index];
        bool is_moving_other = other != squad && other->state == SQUAD_MOVING;
        if (!is_moving_other || !are_squads_coordinated(squad, other))
            continue;
        if (is_giving_way_to(other, squad))
            continue;
        PredictedMotion other_motion = predict_squad_motion(other);
        WaitPrediction prediction = predict_wait(&own, &other_motion);

        // As the park ahead did, a squad re-routes around the final parking positions of each
        // other squad once per order, as soon as they would block it within
        // `PARKING_LOOK_SECONDS`, and without a trial.
        bool is_parking_near = prediction.is_blocked_by_parking
                               && prediction.blocked_seconds <= PARKING_LOOK_SECONDS;
        if (is_parking_near && !squad->is_avoiding_parking[other_index]) {
            reroute_around_parking(squad, other);
            return;
        }

        bool was_yielding = squad->yield_to_index == other_index;
        float min_wait = was_yielding ? STEP_SECONDS : MIN_YIELD_WAIT_SECONDS;
        if (prediction.wait_seconds < min_wait)
            continue;
        // A squad adapts only to waits that start within `WAIT_LOOK_SECONDS`, so that it does not
        // adapt to meetings that are still far off.
        if (!was_yielding && prediction.first_wait_seconds > WAIT_LOOK_SECONDS)
            continue;

        // A squad tries the re-routes for each other squad once per order.
        bool is_long_wait = prediction.wait_seconds >= MIN_WAIT_SECONDS;
        bool can_try = is_long_wait && !has_tried && !squad->is_avoiding_moving[other_index];
        if (can_try && try_reroutes(squad, other, &own, &other_motion, &prediction))
            return;
        has_tried = has_tried || can_try;

        // Without a re-route, the squad that adapts waits, when its wait ends and the other squad
        // would not meet it then. The cap covers only the first wait without a break: behind a
        // slower squad, the waits of the prediction add up along the way, and a cap for all of
        // them would hold the squad far below the speed of that squad.
        if (!can_wait_usefully(&prediction))
            continue;
        if (!was_yielding && !is_yielding_to(squad, other, &prediction))
            continue;
        float hold_seconds = prediction.first_wait_seconds + prediction.first_wait_length_seconds;
        float hold_speed = prediction.first_wait_travel / hold_seconds;

        // With `--lateyield`, at a crossing, the squad keeps full speed while the cap would stay
        // above `LATE_YIELD_SPEED_FRACTION` of its top speed, so that it slows down only near the
        // crossing, and then firmly. Behind a squad on the same way, a late slowdown would bring
        // the squads close enough for the ORCA avoidance to act much more often.
        float late_speed = LATE_YIELD_SPEED_FRACTION * own.speed;
        bool is_cap_early = is_late_yield_on && prediction.is_crossing && hold_speed > late_speed;
        if (!is_cap_early)
            squad->yield_speed_cap = fminf(squad->yield_speed_cap, hold_speed);
        if (was_yielding || squad->yield_to_index == NO_SQUAD) {
            squad->yield_to_index = other_index;
            squad->last_yield_seconds[other_index] = now_seconds;
            is_yield_kept = true;
        }
    }
    if (!is_yield_kept)
        squad->yield_to_index = NO_SQUAD;
}
