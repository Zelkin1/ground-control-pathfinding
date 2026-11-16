/* coordination.c coordinates moving squads in formation following, in the "coord" mode of
 * `--avoid`. sim.c calls `coordinate_squads` once per tick in both formation modes, before
 * squad.c moves the anchors, and `advance_anchor` in formation.c keeps the speed of each anchor
 * at or below the `coordination_speed_cap` of the squad. The rules below act only in the "coord"
 * mode, so that the "form" mode shows formation following on its own.
 *
 * Terms: The top comment of sim.h defines the anchor, the lane, the shift (`side_offset`) and
 * the clamp. formation.h describes `RouteSamples` and `FormationShape` at the types. In short, a
 * shift moves all lanes of a squad sideways over a stretch of its route, and the clamp is the
 * limit that keeps the bodies clear of blocked ground. This file defines the room: the room of a
 * squad toward a side is how far its lanes can shift that way before the outermost lane on that
 * side reaches its clamp. The route samples are points on the route of a squad,
 * `CROSSING_SAMPLE_METERS` apart.
 *
 * Crossings, with `--crossing on`: Where the routes of two moving squads cross at a clear angle,
 * each squad has a crossing zone. The zone is the stretch of its own route on which its formation
 * can touch the other squad. The time of each squad in its zone is estimated from its cruise
 * speed, the lowest top speed of its members. A top speed is the speed of the kind of a unit in a
 * straight line, without turns or obstacles. When the two times overlap, the squad that would
 * enter later gives way, or with `--speedfirst on` the squad with the lower top speed. It caps
 * the speed of its anchor, so that its front reaches the zone only after the last member of the
 * other squad has left it. A squad that gives way slows down early and smoothly, instead of
 * braking at the last moment or bumping into the other squad. With `--lateyield on`, it drives
 * on at full speed for longer, and then slows more, to at most `LATE_YIELD_SPEED_FRACTION` of
 * its cruise speed.
 *
 *                     | the route of the later squad
 *                     |
 *                   [ | ]  <- its zone, `zone_length` each way from the crossing
 *     -------------[--+--]---------------> the route of the earlier squad, with its own zone
 *                   [ | ]
 *                     |
 *                     v
 *
 * Crossings at a small angle (under 30 degrees) are left out, because there the two squads drive
 * along each other, and the zones would be very long.
 *
 * Meeting head-on, with `--keepright on`: Where the route ahead of one squad comes close to the
 * route ahead of another squad that drives the other way, the two squads would meet in the same
 * corridor. Each squad then shifts its lanes around the meeting, away from the other squad, until
 * the two formations are `HEAD_ON_MARGIN_METERS` apart, or more with `--widegap on`. When the two
 * routes run nearly on top of each other, both squads keep to their own right, so that they pick
 * opposite sides. The shift covers only the stretch of route around the meeting. It fades in and
 * out along the route at `SIDE_SHIFT_SLOPE`, or more gently with `--smoothsway on`, so units do
 * not swerve when they set off long before the meeting. It never goes past the room of the squad
 * around the meeting. shift.c sets, moves and fades the shifts.
 *
 * Taking turns, with `--turns on`: Where the meeting is too narrow for the two squads to pass,
 * one of them waits at the last wide spot on its route before the narrow part, until the rear
 * of the other squad has passed that spot. With `--speedfirst on`, the slower squad is the one
 * that waits, when both can. The meeting then moves to the wide spot, where the two squads pass by
 * shifting their lanes. TODO: confirm. The meeting is predicted at cruise speeds
 * (`get_arrival_seconds`), so it can stay in the narrow part. With `--partialshift on`, when
 * neither squad waits, each one shifts by all its room.
 *
 * Overtaking, with `--overtake on`: Squads that drive one behind the other on the same route pass
 * or follow. overtaking.c holds that rule. It uses the shapes, route samples and side room of
 * this file, and the pass shifts of shift.c. */
#include "sim.h"
#include "formation.h"

#define CROSSING_SAMPLE_METERS 6.0f      /* Routes are sampled this far apart to find crossings. */
#define CROSSING_HORIZON_METERS 250.0f   /* Crossings are sought this far past each anchor. */
#define MIN_CROSSING_SINE 0.5f           /* Crossings at under 30 degrees are left out. */
#define CROSSING_MARGIN_METERS 2.0f      /* Each zone reaches this much further each way. */
#define CROSSING_TIME_MARGIN_SECONDS 0.5f /* A squad enters this long after the other one left. */
#define LATE_YIELD_SPEED_FRACTION 0.5f   /* A late give-way slows to this part of cruise speed. */
#define MIN_PREDICTED_SPEED 0.5f         /* Time estimates use at least this speed. */
#define HELD_SPEED 0.2f                  /* A cap below this holds the anchor nearly still. */
#define GIVE_UP_SECONDS 15.0f            /* After this long held, a squad stops giving way. */
#define IGNORE_SECONDS 5.0f              /* It then ignores the other squads for this long. */
#define HEAD_ON_MAX_COSINE -0.7f         /* Routes this close to opposite count as head-on. */
#define MEETING_REACH_METERS 40.0f       /* Room to pass is checked this far around a meeting. */
#define HEAD_ON_EXTRA_GAP_METERS 4.0f    /* With `--widegap`, head-on gaps grow by this much. */
#define WAIT_ROOM_METERS 24.0f           /* A wait spot needs room over this far back. */

/* A `Crossing` is where the routes of two squads cross, as distances along each route. */
typedef struct {
    float first_distance;  /* The crossing is this far along the route of the first squad. */
    float second_distance; /* The crossing is this far along the route of the second squad. */
    float sine;            /* This is the sine of the angle between the two routes there. */
} Crossing;

/* A `ZoneTimes` gives when the formation of a squad enters and leaves its crossing zone, in
 * seconds from now. A negative entry time means that the front is in the zone already. */
typedef struct {
    float entry_seconds;
    float exit_seconds;
    float entry_distance; /* The front reaches the zone after the anchor drives this far. */
} ZoneTimes;

/* ------------------------------------------------------------------------------------------------
 * `is_coordinated` returns true for a squad that takes part in coordination: one that moves.
 * --------------------------------------------------------------------------------------------- */
static bool is_coordinated(const Squad *squad) {
    return squad->state == SQUAD_MOVING;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_shape` takes the front and the half-width of each member from its place
 * (`place_offset`) and its radius. It takes the rear from the route progress of the member and
 * its radius, not from its place, so that members that lag behind their places still count. It
 * takes the cruise speed from the lowest top speed of the kinds of the members.
 * --------------------------------------------------------------------------------------------- */
FormationShape measure_shape(const Squad *squad) {
    FormationShape shape = {
        .front_meters = 0,
        .half_width = 0,
        .rear_progress = LARGE_VALUE,
        .cruise_speed = LARGE_VALUE,
    };
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        float radius = unit->kind->radius;
        float front_meters = unit->place_offset.y + radius;
        float half_width = fabsf(unit->place_offset.x) + radius;
        float rear_progress = unit->route_progress - radius;
        shape.front_meters = fmaxf(shape.front_meters, front_meters);
        shape.half_width = fmaxf(shape.half_width, half_width);
        shape.rear_progress = fminf(shape.rear_progress, rear_progress);
        shape.cruise_speed = fminf(shape.cruise_speed, unit->kind->max_speed);
    }
    return shape;
}

/* ------------------------------------------------------------------------------------------------
 * `is_fraction` returns true when `value` is from 0 to 1.
 * --------------------------------------------------------------------------------------------- */
static bool is_fraction(float value) {
    return value >= 0 && value <= 1;
}

/* ------------------------------------------------------------------------------------------------
 * `get_search_start` returns how far along the route the crossing search starts for a squad.
 * That is its rear, less the largest zone it could still be in, but not before the start of the
 * route. The largest zone takes `LANE_MAX_DISTANCE_METERS` for both half-widths, a cosine of 1,
 * and the smallest sine, `MIN_CROSSING_SINE`.
 * --------------------------------------------------------------------------------------------- */
static float get_search_start(const FormationShape *shape) {
    float zone_reach = 2 * LANE_MAX_DISTANCE_METERS / MIN_CROSSING_SINE + CROSSING_MARGIN_METERS;
    return fmaxf(0, shape->rear_progress - zone_reach);
}

/* ------------------------------------------------------------------------------------------------
 * `find_crossing` finds the first crossing, along the route of the first squad, of the routes
 * ahead of the two squads. It cuts both routes into straight pieces, and it tests each piece of
 * the first route against every piece of the second route:
 *
 *                                o
 *                               /  other
 *   piece_start o--------------X--------------o piece_end
 *                             /
 *                            o other_start
 *
 *   The crossing X lies `first_fraction` of the way along `piece`, and `second_fraction` of the
 *   way along `other`.
 * Inputs: Each route is searched from `get_search_start` of its squad to
 *   `CROSSING_HORIZON_METERS` past its anchor, or to the end of the route when that is closer,
 *   in pieces of `CROSSING_SAMPLE_METERS`.
 * Result: It returns true and fills `crossing`, or false when the routes do not cross there at a
 *   sine of `MIN_CROSSING_SINE` or more.
 * --------------------------------------------------------------------------------------------- */
static bool find_crossing(const Squad *first, const FormationShape *first_shape,
                          const Squad *second, const FormationShape *second_shape,
                          Crossing *crossing) {
    /* Sample the second route once. `second_points[k]` lies `second_distances[k]` along it. */
    enum { MAX_SAMPLES = 128 }; /* The second route gets at most this many samples. */
    V2 second_points[MAX_SAMPLES];
    float second_distances[MAX_SAMPLES];
    float second_start = get_search_start(second_shape);
    float second_horizon = second->anchor_distance + CROSSING_HORIZON_METERS;
    float second_end = fminf(second->route_length, second_horizon);
    int second_count = 0;
    float distance = second_start;
    while (second_count < MAX_SAMPLES) {
        float sample_distance = fminf(distance, second_end);
        V2 unused_direction;
        second_points[second_count] = get_route_point(second, sample_distance, &unused_direction);
        second_distances[second_count] = sample_distance;
        second_count++;
        if (sample_distance >= second_end)
            break;
        distance += CROSSING_SAMPLE_METERS;
    }

    /* Walk the first route piece by piece, and test each piece against every piece of the
     * second route. The first piece with a crossing wins. */
    float first_start = get_search_start(first_shape);
    float first_horizon = first->anchor_distance + CROSSING_HORIZON_METERS;
    float first_end = fminf(first->route_length, first_horizon);
    V2 unused_direction;
    V2 piece_start = get_route_point(first, first_start, &unused_direction);
    for (float piece_distance = first_start; piece_distance < first_end;
         piece_distance += CROSSING_SAMPLE_METERS) {
        float piece_end_distance = fminf(piece_distance + CROSSING_SAMPLE_METERS, first_end);
        V2 piece_end = get_route_point(first, piece_end_distance, &unused_direction);
        V2 piece = sub(piece_end, piece_start);
        float piece_length = len(piece);
        for (int sample_index = 0; sample_index + 1 < second_count; sample_index++) {
            /* Solve `piece_start + first_fraction * piece = other_start + second_fraction *
             * other`. The pieces cross where both fractions are from 0 to 1. */
            int next_index = sample_index + 1;
            V2 other_start = second_points[sample_index];
            V2 other = sub(second_points[next_index], other_start);
            float determinant = det2(piece, other);
            float determinant_magnitude = fabsf(determinant);
            float length_product = piece_length * len(other);
            if (length_product <= 0 || determinant_magnitude < MIN_CROSSING_SINE * length_product)
                continue;
            V2 to_other_start = sub(other_start, piece_start);
            float first_fraction = det2(to_other_start, other) / determinant;
            float second_fraction = det2(to_other_start, piece) / determinant;
            if (!is_fraction(first_fraction) || !is_fraction(second_fraction))
                continue;
            float other_start_distance = second_distances[sample_index];
            float other_length = second_distances[next_index] - other_start_distance;
            float piece_span = piece_end_distance - piece_distance;
            crossing->first_distance = piece_distance + first_fraction * piece_span;
            crossing->second_distance = other_start_distance + second_fraction * other_length;
            crossing->sine = determinant_magnitude / length_product;
            return true;
        }
        piece_start = piece_end;
    }
    return false;
}

/* ------------------------------------------------------------------------------------------------
 * `get_zone_times` returns when the formation of the squad enters and leaves its zone, which
 * reaches `zone_length` each way from the crossing at `crossing_distance` along its route. The
 * front enters when the anchor plus `front_meters` reaches the start of the zone, and the
 * formation has left when its rear passes the end of the zone. Both times assume the cruise
 * speed, but at least `MIN_PREDICTED_SPEED`:
 *
 *   rear anchor   front             crossing
 *   |=======x=======|- - - - - [--------+--------]- - -> route
 *                   |<-------->|
 *   |<------------------------------------------>|
 *
 *   The front lies `entry_distance` before the start of the zone, and the rear lies
 *   `exit_distance` before the end of the zone.
 * --------------------------------------------------------------------------------------------- */
static ZoneTimes get_zone_times(const Squad *squad, const FormationShape *shape,
                                float crossing_distance, float zone_length) {
    float speed = fmaxf(MIN_PREDICTED_SPEED, shape->cruise_speed);
    float front_distance = squad->anchor_distance + shape->front_meters;
    float entry_distance = crossing_distance - zone_length - front_distance;
    float exit_distance = crossing_distance + zone_length - shape->rear_progress;
    return (ZoneTimes){
        .entry_seconds = entry_distance / speed,
        .exit_seconds = exit_distance / speed,
        .entry_distance = entry_distance,
    };
}

/* ------------------------------------------------------------------------------------------------
 * `give_way_at_crossing` checks one pair of squads for a crossing where both would be at the same
 * time, and caps the speed of the squad that gives way.
 * Result: The squad that gives way gets a `coordination_speed_cap` at which its front reaches its
 *   zone `CROSSING_TIME_MARGIN_SECONDS` after the other squad has left its own, with
 *   `--lateyield on` only once that cap is at most `LATE_YIELD_SPEED_FRACTION` of its cruise
 *   speed. It is the squad that would enter its zone later, and on a tie the one with the
 *   higher index. With `--speedfirst on`, it is instead the squad with the lower top speed, when
 *   the two top speeds differ and the front of that squad is not in its zone yet.
 * Edge cases: Nothing changes when the routes do not cross, when either squad has left its zone
 *   already, when the front of the squad that would give way is in its zone already, or when the
 *   times do not overlap.
 * --------------------------------------------------------------------------------------------- */
static void give_way_at_crossing(Squad *first, Squad *second) {
    /* Find the crossing and both zones. The zone of a squad reaches as far from the crossing,
     * along its route, as its formation can touch the bodies of the other squad. At a right
     * angle, that reach is the half-width of the other squad. A slant stretches it by 1 / sine.
     * A slant also brings one side of its formation nearer to the other route, and so it adds
     * its half-width times cosine / sine. `CROSSING_MARGIN_METERS` comes on top. */
    FormationShape first_shape = measure_shape(first);
    FormationShape second_shape = measure_shape(second);
    Crossing crossing;
    if (!find_crossing(first, &first_shape, second, &second_shape, &crossing))
        return;
    float cosine = sqrtf(1 - crossing.sine * crossing.sine);
    float first_slant = first_shape.half_width * cosine;
    float second_slant = second_shape.half_width * cosine;
    float first_reach = (second_shape.half_width + first_slant) / crossing.sine;
    float second_reach = (first_shape.half_width + second_slant) / crossing.sine;
    float first_zone = first_reach + CROSSING_MARGIN_METERS;
    float second_zone = second_reach + CROSSING_MARGIN_METERS;
    ZoneTimes first_times = get_zone_times(first, &first_shape, crossing.first_distance,
                                           first_zone);
    ZoneTimes second_times = get_zone_times(second, &second_shape, crossing.second_distance,
                                            second_zone);

    /* Leave out squads that have passed, and times that do not overlap. */
    bool has_any_left = first_times.exit_seconds <= 0 || second_times.exit_seconds <= 0;
    if (has_any_left)
        return;
    float margin_seconds = CROSSING_TIME_MARGIN_SECONDS;
    bool is_first_clear = first_times.entry_seconds > second_times.exit_seconds + margin_seconds;
    bool is_second_clear = second_times.entry_seconds > first_times.exit_seconds + margin_seconds;
    if (is_first_clear || is_second_clear)
        return;

    /* Pick the squad that gives way: the later one, or the slower one with `--speedfirst on`. */
    bool is_second_giving_way = second_times.entry_seconds >= first_times.entry_seconds;
    bool has_slower_squad = first_shape.cruise_speed != second_shape.cruise_speed;
    if (is_speed_priority_on && has_slower_squad) {
        bool is_second_slower = second_shape.cruise_speed < first_shape.cruise_speed;
        ZoneTimes slower_times = is_second_slower ? second_times : first_times;
        if (slower_times.entry_distance > 0)
            is_second_giving_way = is_second_slower;
    }

    /* Cap its speed, unless its front is in its zone already. With `--lateyield on`, it drives
     * on at full speed while the speed that brings it to its zone in time is still above
     * `LATE_YIELD_SPEED_FRACTION` of its cruise speed. It then slows to that speed and keeps it,
     * so it reaches the zone at the same time, but slows later and more. */
    Squad *giving_way = is_second_giving_way ? second : first;
    ZoneTimes giving_way_times = is_second_giving_way ? second_times : first_times;
    ZoneTimes passing_times = is_second_giving_way ? first_times : second_times;
    if (giving_way_times.entry_distance <= 0)
        return;
    float wait_seconds = passing_times.exit_seconds + margin_seconds;
    float speed_cap = giving_way_times.entry_distance / wait_seconds;
    if (is_late_yield_on) {
        const FormationShape *giving_way_shape = &first_shape;
        if (is_second_giving_way)
            giving_way_shape = &second_shape;
        float late_speed = LATE_YIELD_SPEED_FRACTION * giving_way_shape->cruise_speed;
        if (speed_cap > late_speed)
            return;
    }
    giving_way->coordination_speed_cap = fminf(giving_way->coordination_speed_cap, speed_cap);
}

/* ------------------------------------------------------------------------------------------------
 * `sample_route_span` steps along the route by `CROSSING_SAMPLE_METERS` and takes each point from
 * `get_route_point`, which limits a distance to the route. A `start` before the route start
 * therefore gives its first samples at the start point, but `distances` still holds the distance
 * that the loop asked for.
 * --------------------------------------------------------------------------------------------- */
void sample_route_span(const Squad *squad, float start, float end, RouteSamples *samples) {
    int max_count = ROUTE_SAMPLE_COUNT;
    samples->count = 0;
    for (float distance = start; distance <= end; distance += CROSSING_SAMPLE_METERS) {
        if (samples->count == max_count)
            break;
        int sample_index = samples->count;
        V2 *direction = &samples->directions[sample_index];
        samples->points[sample_index] = get_route_point(squad, distance, direction);
        samples->distances[sample_index] = distance;
        samples->is_shared[sample_index] = false;
        samples->count++;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `sample_route_ahead` samples the route of the squad from its rear, but not before the start of
 * the route, to `CROSSING_HORIZON_METERS` past its anchor, or to the end of the route when that
 * is closer.
 * --------------------------------------------------------------------------------------------- */
static void sample_route_ahead(const Squad *squad, const FormationShape *shape,
                               RouteSamples *samples) {
    float start = fmaxf(0, shape->rear_progress);
    float horizon = squad->anchor_distance + CROSSING_HORIZON_METERS;
    float end = fminf(squad->route_length, horizon);
    sample_route_span(squad, start, end, samples);
}

/* ------------------------------------------------------------------------------------------------
 * `measure_side_room` finds the lanes of the outermost slots, and it takes the clamps from
 * `get_lane`. Lane x is the sideways offset from the route, to the right:
 *
 *   min_x    left lane       right lane    max_x
 *     |<- room ->|  other lanes   |<- room ->|
 *       side < 0                    side > 0
 *
 *   In single file, both outermost lanes run down the middle of the passage.
 * --------------------------------------------------------------------------------------------- */
float measure_side_room(const Squad *squad, float distance, float side) {
    /* Find the sideways offsets of the outermost slots. */
    float min_slot_x = LARGE_VALUE;
    float max_slot_x = -LARGE_VALUE;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        float slot_x = units[unit_index].place_offset.x;
        min_slot_x = fminf(min_slot_x, slot_x);
        max_slot_x = fmaxf(max_slot_x, slot_x);
    }

    /* Measure from the outermost lane on that side to the clamp on that side. */
    Lane lane = get_lane(squad, distance);
    float right_lane_x = max_slot_x * lane.scale + lane.shift;
    float left_lane_x = min_slot_x * lane.scale + lane.shift;
    if (squad->is_single_file) {
        right_lane_x = lane.middle;
        left_lane_x = lane.middle;
    }
    float room = lane.max_x - right_lane_x;
    if (side < 0)
        room = left_lane_x - lane.min_x;
    return fmaxf(0, room);
}

/* ------------------------------------------------------------------------------------------------
 * `measure_shared_room` returns the smallest room (`measure_side_room`) toward `side` over the
 * samples that `is_shared` marks, or 0 when there are none.
 * --------------------------------------------------------------------------------------------- */
static float measure_shared_room(const Squad *squad, const RouteSamples *samples, float side) {
    float room = LARGE_VALUE;
    for (int sample_index = 0; sample_index < samples->count; sample_index++) {
        if (!samples->is_shared[sample_index])
            continue;
        float distance = samples->distances[sample_index];
        room = fminf(room, measure_side_room(squad, distance, side));
    }
    if (room == LARGE_VALUE)
        return 0;
    return room;
}

/* ------------------------------------------------------------------------------------------------
 * `mark_meeting_samples` marks, in `is_shared`, the samples within `MEETING_REACH_METERS` along
 * the route of the sample at `meeting_index`, and it clears the mark on the other samples.
 * --------------------------------------------------------------------------------------------- */
static void mark_meeting_samples(RouteSamples *samples, int meeting_index) {
    float meeting_distance = samples->distances[meeting_index];
    for (int sample_index = 0; sample_index < samples->count; sample_index++) {
        float gap = fabsf(samples->distances[sample_index] - meeting_distance);
        samples->is_shared[sample_index] = gap <= MEETING_REACH_METERS;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `has_room_back` returns true when the squad alone has `needed_room` toward `side` at the
 * sample at `last_index` and at the samples up to `WAIT_ROOM_METERS` before it, or at as many as
 * there are. That way the whole formation can stand aside there, not just its front. The samples
 * are `CROSSING_SAMPLE_METERS` apart (`sample_route_span`).
 * --------------------------------------------------------------------------------------------- */
static bool has_room_back(const Squad *squad, const RouteSamples *samples, int last_index,
                          float side, float needed_room) {
    int wait_room_samples = (int)roundf(WAIT_ROOM_METERS / CROSSING_SAMPLE_METERS);
    int first_index = last_index - wait_room_samples;
    if (first_index < 0)
        first_index = 0;
    for (int sample_index = first_index; sample_index <= last_index; sample_index++) {
        float distance = samples->distances[sample_index];
        if (measure_side_room(squad, distance, side) < needed_room)
            return false;
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `find_wait_distance` finds the wait spot of the squad. It looks on the route of the squad, from
 * the sample at `meeting_index` back toward the squad, and it takes the first sample where
 * `has_room_back` holds:
 *
 *   front                      wait spot       meeting
 *     |                            |              |
 *   ==>   o----o----o----o----o----o----o----o----o----> route samples
 *              |<------ room ----->|
 *     |<---------- result -------->|
 *
 * Result: It returns how far the anchor can drive before the front of the formation reaches the
 *   wait spot, and it sets `wait_index` to the sample of the spot.
 * Edge case: It returns -1 when no sample between the front and the meeting has that room.
 * --------------------------------------------------------------------------------------------- */
static float find_wait_distance(const Squad *squad, const FormationShape *shape,
                                const RouteSamples *samples, int meeting_index, float side,
                                float needed_room, int *wait_index) {
    float front_distance = squad->anchor_distance + shape->front_meters;
    for (int sample_index = meeting_index; sample_index >= 0; sample_index--) {
        float distance = samples->distances[sample_index];
        if (distance <= front_distance)
            return -1;
        if (has_room_back(squad, samples, sample_index, side, needed_room)) {
            *wait_index = sample_index;
            return distance - front_distance;
        }
    }
    return -1;
}

/* ------------------------------------------------------------------------------------------------
 * `find_closest_sample_index` compares squared distances, which saves a square root for each
 * sample. A sample replaces the best one only when it is strictly closer, so the first sample
 * wins a tie.
 * --------------------------------------------------------------------------------------------- */
int find_closest_sample_index(const RouteSamples *samples, V2 point) {
    int best_index = 0;
    float best_distance_squared = LARGE_VALUE;
    for (int sample_index = 0; sample_index < samples->count; sample_index++) {
        V2 to_sample = sub(samples->points[sample_index], point);
        float distance_squared = length_squared(to_sample);
        if (distance_squared < best_distance_squared) {
            best_distance_squared = distance_squared;
            best_index = sample_index;
        }
    }
    return best_index;
}

/* A `MeetingSide` gives the part of one squad in a narrow meeting. */
typedef struct {
    Squad *squad;
    const FormationShape *shape;
    const RouteSamples *samples;
    int meeting_index; /* The meeting is at this sample of the route of the squad. */
} MeetingSide;

/* ------------------------------------------------------------------------------------------------
 * `take_turns` lets one of two squads that meet in a narrow place wait for the other.
 * Result: The wait spot of each squad is its `find_wait_distance` sample, where it alone has room
 *   to shift by `missing_gap` toward `side`. A squad that gave way to the other one in the last
 *   tick keeps giving way while it can reach its wait spot. Otherwise the squad that can reach
 *   its wait spot gives way, and when both can, the one with the longer way to it, which has
 *   more room to slow down. With `--speedfirst on`, when both can and their top speeds differ,
 *   the slower one gives way instead. The squad that gives way gets a cap at which its front
 *   reaches its wait spot when the rear of the other squad is `CROSSING_MARGIN_METERS` past the
 *   route sample of the other squad closest to the spot, but not sooner than
 *   `CROSSING_TIME_MARGIN_SECONDS` from now. It is also marked as giving way to the other squad
 *   (`yielding_to_index`).
 * Edge case: When neither squad can reach a wait spot, neither waits.
 * --------------------------------------------------------------------------------------------- */
static void take_turns(const MeetingSide *first, const MeetingSide *second, float side,
                       float missing_gap) {
    /* Find the wait spot of each squad. */
    int first_wait_index = 0;
    int second_wait_index = 0;
    float first_wait_distance = find_wait_distance(first->squad, first->shape, first->samples,
                                                   first->meeting_index, side, missing_gap,
                                                   &first_wait_index);
    float second_wait_distance = find_wait_distance(second->squad, second->shape, second->samples,
                                                    second->meeting_index, side, missing_gap,
                                                    &second_wait_index);
    if (first_wait_distance < 0 && second_wait_distance < 0)
        return;

    /* Pick the squad that gives way. */
    bool can_first_wait = first_wait_distance >= 0;
    bool can_second_wait = second_wait_distance >= 0;
    bool is_first_waiting = first_wait_distance >= second_wait_distance;
    float first_speed = first->shape->cruise_speed;
    float second_speed = second->shape->cruise_speed;
    bool can_both_wait = can_first_wait && can_second_wait;
    if (is_speed_priority_on && can_both_wait && first_speed != second_speed)
        is_first_waiting = first_speed < second_speed;

    /* Let a squad that gave way in the last tick keep giving way. */
    bool was_first_waiting = first->squad->yielding_to_index == second->squad->squad_index;
    bool was_second_waiting = second->squad->yielding_to_index == first->squad->squad_index;
    if (was_first_waiting && can_first_wait)
        is_first_waiting = true;
    if (was_second_waiting && can_second_wait)
        is_first_waiting = false;
    const MeetingSide *waiting = is_first_waiting ? first : second;
    const MeetingSide *passing = is_first_waiting ? second : first;
    float wait_distance = is_first_waiting ? first_wait_distance : second_wait_distance;
    int wait_index = is_first_waiting ? first_wait_index : second_wait_index;

    /* Cap the speed of the waiting squad until the rear of the passing squad is past the wait
     * spot. */
    V2 wait_point = waiting->samples->points[wait_index];
    int passing_sample_index = find_closest_sample_index(passing->samples, wait_point);
    float passing_distance = passing->samples->distances[passing_sample_index];
    float passing_speed = fmaxf(MIN_PREDICTED_SPEED, passing->shape->cruise_speed);
    float rear_reach = passing_distance + CROSSING_MARGIN_METERS - passing->shape->rear_progress;
    float pass_seconds = fmaxf(CROSSING_TIME_MARGIN_SECONDS, rear_reach / passing_speed);
    float speed_cap = wait_distance / pass_seconds;
    Squad *waiting_squad = waiting->squad;
    float old_cap = waiting_squad->coordination_speed_cap;
    waiting_squad->coordination_speed_cap = fminf(old_cap, speed_cap);
    waiting_squad->yielding_to_index = passing->squad->squad_index;
    waiting_squad->is_yield_kept = true;
}

/* ------------------------------------------------------------------------------------------------
 * `get_arrival_seconds` returns when the anchor of the squad reaches `distance` along its route
 * at its cruise speed, in seconds from now.
 * --------------------------------------------------------------------------------------------- */
static float get_arrival_seconds(const Squad *squad, const FormationShape *shape, float distance) {
    float speed = fmaxf(MIN_PREDICTED_SPEED, shape->cruise_speed);
    return (distance - squad->anchor_distance) / speed;
}

/* ------------------------------------------------------------------------------------------------
 * `pass_head_on` checks one pair of squads for a head-on meeting, and sets how far each must
 * shift its lanes to pass the other. It also marks both squads in `has_meeting`.
 * Result: `pass_head_on` returns true when the squads meet head-on, and false otherwise.
 * Meeting: A close pair is a pair of samples of the two routes that run opposite ways, at a
 *   cosine of `HEAD_ON_MAX_COSINE` or less, and that are closer than the needed gap. The needed
 *   gap is the half-widths of the formations plus `HEAD_ON_MARGIN_METERS`, and with
 *   `--widegap on` plus `HEAD_ON_EXTRA_GAP_METERS`. The meeting is the close pair that the two
 *   anchors reach at the most nearly equal times:
 *
 *                        meeting
 *     [first]=> o-----o-----o-----o-----o----->  first route
 *                           |  `lateral` (here more than 0)
 *         <-----o-----o-----o-----o-----o <=[second]  second route
 *
 * Shifts: The missing gap is how much the sideways distance between the two routes there falls
 *   short of the needed gap. Each squad shifts away from the other, or both to their own right
 *   when the routes are within `SAME_LINE_METERS` sideways there. Each takes half of the missing
 *   gap, or more when the other has too little room on that side within `MEETING_REACH_METERS`
 *   of the meeting. A larger target from another pair stays.
 * Edge case: When the room of the two squads together is less than the missing gap, the passage
 *   is too narrow, and the squads do not split the gap. With `is_turn_taking_on`, one of them
 *   can wait instead (`take_turns`). With `--partialshift on`, when neither waits, each squad
 *   shifts by all its room.
 * --------------------------------------------------------------------------------------------- */
static bool pass_head_on(Squad *first, Squad *second) {
    /* Sample both routes ahead. */
    FormationShape first_shape = measure_shape(first);
    FormationShape second_shape = measure_shape(second);
    static RouteSamples first_samples;
    static RouteSamples second_samples;
    sample_route_ahead(first, &first_shape, &first_samples);
    sample_route_ahead(second, &second_shape, &second_samples);

    /* Find the meeting among the close pairs. `lateral` is how far the second route lies to the
     * right of the first one there. With `--widegap on`, keep `HEAD_ON_EXTRA_GAP_METERS` more
     * between the bodies. With `--widegap off`, the squads pass so close that the avoidance of
     * their units still steps in. */
    float margin_meters = HEAD_ON_MARGIN_METERS;
    if (is_wide_gap_on)
        margin_meters += HEAD_ON_EXTRA_GAP_METERS;
    float needed_gap = first_shape.half_width + second_shape.half_width + margin_meters;
    float best_time_gap = LARGE_VALUE;
    float lateral = 0;
    int first_meeting_index = -1;
    int second_meeting_index = -1;
    for (int first_index = 0; first_index < first_samples.count; first_index++) {
        V2 point = first_samples.points[first_index];
        V2 direction = first_samples.directions[first_index];
        float first_distance = first_samples.distances[first_index];
        float first_seconds = get_arrival_seconds(first, &first_shape, first_distance);
        for (int second_index = 0; second_index < second_samples.count; second_index++) {
            V2 other_direction = second_samples.directions[second_index];
            if (dot(direction, other_direction) > HEAD_ON_MAX_COSINE)
                continue;
            V2 to_other = sub(second_samples.points[second_index], point);
            if (len(to_other) >= needed_gap)
                continue;
            float second_distance = second_samples.distances[second_index];
            float second_seconds = get_arrival_seconds(second, &second_shape, second_distance);
            float time_gap = fabsf(first_seconds - second_seconds);
            if (time_gap >= best_time_gap)
                continue;
            best_time_gap = time_gap;
            lateral = dot(to_other, get_right(direction));
            first_meeting_index = first_index;
            second_meeting_index = second_index;
        }
    }
    if (first_meeting_index < 0)
        return false;
    mark_meeting_samples(&first_samples, first_meeting_index);
    mark_meeting_samples(&second_samples, second_meeting_index);

    /* Mark both squads as meeting, and pick the sides. The second squad drives the other way, so
     * a shift to its own right moves it to the left of the first squad. */
    has_meeting[first->squad_index] = true;
    has_meeting[second->squad_index] = true;
    float missing_gap = needed_gap - fabsf(lateral);
    float side = 1;
    if (lateral > SAME_LINE_METERS)
        side = -1;

    /* Split the missing gap by the room on each side, or give up when it is too narrow. */
    float first_room = measure_shared_room(first, &first_samples, side);
    float second_room = measure_shared_room(second, &second_samples, side);
    float first_meeting_distance = first_samples.distances[first_meeting_index];
    float second_meeting_distance = second_samples.distances[second_meeting_index];
    if (first_room + second_room < missing_gap) {
        if (is_turn_taking_on) {
            MeetingSide first_side = {
                .squad = first,
                .shape = &first_shape,
                .samples = &first_samples,
                .meeting_index = first_meeting_index,
            };
            MeetingSide second_side = {
                .squad = second,
                .shape = &second_shape,
                .samples = &second_samples,
                .meeting_index = second_meeting_index,
            };
            take_turns(&first_side, &second_side, side, missing_gap);
        }

        /* With `--partialshift on`, when neither squad waits for the other, let each shift by
         * all its room. Less of the gap is then missing, and the avoidance of the units keeps the
         * bodies apart. With `--partialshift off`, wide formations drive straight at each
         * other. For example, two heavy squads (formations 16.5 m to each side) miss 37 m and
         * have 13 m of room each, because the lanes measure free ground only so far from the
         * route, and they squeeze past each other at a crawl. */
        bool is_first_waiting = first->is_yield_kept &&
                                first->yielding_to_index == second->squad_index;
        bool is_second_waiting = second->is_yield_kept &&
                                 second->yielding_to_index == first->squad_index;
        bool is_any_waiting = is_first_waiting || is_second_waiting;
        if (is_partial_shift_on && !is_any_waiting) {
            set_shift_target(first, first_meeting_distance, side * first_room);
            set_shift_target(second, second_meeting_distance, side * second_room);
        }
        return true;
    }
    float half_gap = missing_gap / 2;
    float wanted_first_shift = fmaxf(half_gap, missing_gap - second_room);
    float first_shift = fminf(first_room, wanted_first_shift);
    float second_shift = missing_gap - first_shift;
    set_shift_target(first, first_meeting_distance, side * first_shift);
    set_shift_target(second, second_meeting_distance, side * second_shift);
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `update_give_up` lets a squad stop giving way after `GIVE_UP_SECONDS` held nearly still, for
 * `IGNORE_SECONDS`, so that squads that wait for each other cannot stand still forever.
 * --------------------------------------------------------------------------------------------- */
static void update_give_up(Squad *squad) {
    if (squad->coordination_speed_cap < HELD_SPEED)
        squad->coordination_held_seconds += TICK_SECONDS;
    else
        squad->coordination_held_seconds = 0;
    if (squad->coordination_held_seconds > GIVE_UP_SECONDS) {
        squad->coordination_held_seconds = 0;
        squad->ignore_coordination_seconds = IGNORE_SECONDS;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `coordinate_squads` makes three passes. The first resets the state of each squad for this tick
 * and keeps its last wanted shift. The second tests each pair of moving squads with the rules in
 * the top comment of this file, and these rules set the caps and the wanted shifts. The third
 * fixes up each squad and then moves its shift.
 * --------------------------------------------------------------------------------------------- */
void coordinate_squads(void) {
    /* Start every squad without a cap and without a wanted shift, and remember the last wanted
     * shift and its stretch. `last_targets[k]`, `last_starts[k]` and `last_ends[k]` belong to
     * `squads[k]`. Count down the time that each squad ignores coordination, and clear its marks:
     * `take_turns` and `try_pass` (overtaking.c) set them again for the squads that still give
     * way, pass or drive in single file for a pass. Move the marks of the squads that each squad
     * follows to `was_following`, because `pass_or_follow` (overtaking.c) reads them there and
     * sets `is_following` again for this tick. */
    float last_targets[MAX_SQUADS];
    float last_starts[MAX_SQUADS];
    float last_ends[MAX_SQUADS];
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        Squad *squad = &squads[squad_index];
        squad->coordination_speed_cap = LARGE_VALUE;
        last_targets[squad_index] = squad->side_offset_target;
        last_starts[squad_index] = squad->side_shift_start;
        last_ends[squad_index] = squad->side_shift_end;
        squad->side_offset_target = 0;
        has_meeting[squad_index] = false;
        is_pass_shift[squad_index] = false;
        squad->ignore_coordination_seconds -= TICK_SECONDS;
        squad->is_yield_kept = false;
        squad->is_pass_kept = false;
        squad->is_file_forced = false;
        for (int other_index = 0; other_index < squad_count; other_index++) {
            squad->was_following[other_index] = squad->is_following[other_index];
            squad->is_following[other_index] = false;
        }
    }

    /* Check every pair of moving squads, in the "coord" mode only. Squads that meet head-on pass
     * each other or take turns, so the crossing rule leaves them out: on winding routes, two
     * oncoming routes also cross at the bends. The crossing rule also leaves out squads on one
     * stretch of route, which pass or follow. */
    bool is_coordinating = avoid_mode == AVOID_COORD;
    for (int first_index = 0; first_index < squad_count && is_coordinating; first_index++) {
        Squad *first = &squads[first_index];
        if (!is_coordinated(first))
            continue;
        for (int second_index = first_index + 1; second_index < squad_count; second_index++) {
            Squad *second = &squads[second_index];
            if (!is_coordinated(second))
                continue;
            bool is_head_on = false;
            if (is_keep_right_on)
                is_head_on = pass_head_on(first, second);
            bool is_one_way = false;
            if (is_overtake_on && !is_head_on)
                is_one_way = pass_or_follow(first, second);
            if (is_crossing_yield_on && !is_head_on && !is_one_way)
                give_way_at_crossing(first, second);
        }
    }

    /* Keep the shifts steady with `--smoothsway on`, forget the turns and passes that ended,
     * drop the caps of squads that gave up waiting, and move the shifts of moving squads. */
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        Squad *squad = &squads[squad_index];
        if (is_smooth_sway_on)
            keep_shift_steady(squad, last_targets[squad_index], last_starts[squad_index],
                              last_ends[squad_index]);
        if (!squad->is_yield_kept)
            squad->yielding_to_index = NO_SQUAD;
        if (!squad->is_pass_kept)
            squad->passing_index = NO_SQUAD;
        update_give_up(squad);
        if (squad->ignore_coordination_seconds > 0)
            squad->coordination_speed_cap = LARGE_VALUE;
        if (is_coordinated(squad))
            move_side_offset(squad);
    }
}
