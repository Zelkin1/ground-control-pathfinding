// Routes of formation following: route points at route positions, the formation direction, free
// distances along rays, and the preparation of the route that the base game path search gives a
// squad. The preparation moves the route start toward the squad members, shortens the route where
// a band as wide as the widest squad member is clear, and takes the route of a centered game
// search (a second base game path search from the center of the squad) when it is clearly shorter.
// The smoothing of the prepared route into curves is in smoothing.c.
//
// A route position is a distance along the route from the route start, and `route_distance` holds
// the route position of each route point. A map cell holds a terrain value, and a unit crosses the
// cells whose value is at most its terrain class.
#include <string.h>

#include "sim.h"
#include "formation.h"

#define MIN_CHORD_LENGTH_SQUARED 1.0f                // shorter chords give no formation direction
#define CELL_SKIP_MARGIN_METERS 0.01                 // skipped probes lie this far inside a cell
#define CORNER_CUT_STEPS 6                           // bisection steps of a corner cut
#define CENTERED_SEARCH_BUDGET 500                   // expansion budget of the centered search
#define CENTERED_SEARCH_TOLERANCE_METERS 0           // the centered search ends on the destination
#define CENTERED_ROUTE_MIN_GAIN_METERS 10.0f         // smallest length gain of the centered route
#define FORMATION_DIRECTION_SPAN_METERS 10.0f        // chord reach of the formation direction
#define LEAD_IN_STEP_RADIANS (PI / 4)                // largest turn between two lead-in points
#define LEAD_IN_MIN_SWEEP_RADIANS 0.2f               // smaller turns get no lead-in points
#define LEAD_IN_MAX_POINTS 8                         // lead-in points of a full turn


// A ray from `start` along the unit vector `direction`. `inverse_x` and `inverse_y` hold the
// inverses of the two components, or zero for a zero component.
typedef struct {
    V2 start;
    V2 direction;
    double inverse_x;
    double inverse_y;
} Ray;


// `get_route_point` returns the point of the route of a squad at the route position `distance`,
// and in `direction`, the direction of the route segment there. Positions outside the route read
// the route start or the route end, and a segment of zero length has the final facing.
V2 get_route_point(const Squad *squad, float distance, V2 *direction) {
    // The segment is the first one that ends at or past `distance`.
    int end_index = 1;
    while (end_index < squad->route_count - 1 && distance > squad->route_distance[end_index])
        end_index++;
    int start_index = end_index - 1;
    V2 from = squad->route[start_index];
    V2 segment = sub(squad->route[end_index], from);
    float d_start = squad->route_distance[start_index];        // route position of the segment
    float length = squad->route_distance[end_index] - d_start;
    float fraction;
    if (length > ROUTE_EPSILON_METERS) {
        *direction = mul(segment, 1 / length);
        fraction = (distance - d_start) / length;
    } else {
        *direction = squad->final_facing;
        fraction = 1;
    }
    float clamped_fraction = clamp_float(fraction, 0, 1);
    return add(from, mul(segment, clamped_fraction));
}


// `get_extended_route_point` returns the route point at `distance` as `get_route_point` does, but
// before the route start and past the route end, the route continues straight along its first and
// last segment.
V2 get_extended_route_point(const Squad *squad, float distance, V2 *direction) {
    V2 point = get_route_point(squad, distance, direction);
    if (distance < 0)
        return add(point, mul(*direction, distance));
    if (distance > squad->route_length)
        return add(point, mul(*direction, distance - squad->route_length));
    return point;
}


// `get_formation_direction` returns the formation direction at the route position `distance`: the
// direction of the chord from `FORMATION_DIRECTION_SPAN_METERS` before to as far after it, so that
// slots turn smoothly at route corners. A chord that is too short gives the segment direction.
V2 get_formation_direction(const Squad *squad, float distance) {
    V2 segment_direction;
    V2 unused;
    get_route_point(squad, distance, &segment_direction);
    float d_ahead = distance + FORMATION_DIRECTION_SPAN_METERS;
    float d_behind = distance - FORMATION_DIRECTION_SPAN_METERS;
    V2 ahead = get_route_point(squad, d_ahead, &unused);
    V2 behind = get_route_point(squad, d_behind, &unused);
    V2 chord = sub(ahead, behind);
    return get_direction(chord, MIN_CHORD_LENGTH_SQUARED, segment_direction);
}


// `limit_to_cell_axis` narrows the ray distances `[from, to]` to those at which the coordinate of
// a ray, starting at `start` with the rate `step` per meter, lies inside the cell `cell` of one
// axis, at least `CELL_SKIP_MARGIN_METERS` from its edges. An empty range gets `to = -1`.
static void limit_to_cell_axis(float start, float step, double inverse_step, int cell,
                               double *from, double *to) {
    // A ray parallel to the cell edges stays inside the cell or outside of it.
    double low = cell * (double)GC_CELL_METERS + CELL_SKIP_MARGIN_METERS;
    double high = (cell + 1) * (double)GC_CELL_METERS - CELL_SKIP_MARGIN_METERS;
    if (step == 0) {
        if (start < low || start > high)
            *to = -1;
        return;
    }

    // The ray enters the cell at one edge and leaves it at the other, depending on its sign.
    double enter_side = step > 0 ? low : high;
    double leave_side = step > 0 ? high : low;
    double enter = (enter_side - start) * inverse_step;        // ray distance at the entry
    double leave = (leave_side - start) * inverse_step;        // ray distance at the exit
    if (enter > *from)
        *from = enter;
    if (leave < *to)
        *to = leave;
}


// `find_free_cell_skip` returns how far a ray can skip past the probe at `distance`, which lies in
// the free cell `(cell_x, cell_y)`: past all later probes inside that cell, which are free too, or
// one probe step when the next probe leaves the cell.
static float find_free_cell_skip(const Ray *ray, int cell_x, int cell_y, float distance) {
    double from = 0;
    double to = LANE_MAX_DISTANCE_METERS;
    limit_to_cell_axis(ray->start.x, ray->direction.x, ray->inverse_x, cell_x, &from, &to);
    limit_to_cell_axis(ray->start.y, ray->direction.y, ray->inverse_y, cell_y, &from, &to);

    // `skips` counts the probe steps that stay inside the cell.
    double next = distance + FREE_DISTANCE_STEP_METERS;
    if (next < from || next > to)
        return FREE_DISTANCE_STEP_METERS;
    int skips = (int)((to - distance) / FREE_DISTANCE_STEP_METERS);
    return (skips + 1) * FREE_DISTANCE_STEP_METERS;
}


// `get_clearance_skip` returns how far a ray can skip past a probe in the cell `(cell_x, cell_y)`:
// past all probes within the clearance of the cell, which are free.
static float get_clearance_skip(int cell_x, int cell_y, int terrain_class) {
    float clearance = get_clearance_meters(cell_x, cell_y, terrain_class);
    int free_probes = (int)(clearance / FREE_DISTANCE_STEP_METERS);
    return (free_probes + 1) * FREE_DISTANCE_STEP_METERS;
}


// `get_inverse` returns `1 / value`, or zero for a zero value.
static double get_inverse(float value) {
    if (value == 0)
        return 0;
    return 1.0 / value;
}


// `measure_free_distance` returns the distance from `point` along `direction` to the first probe
// in blocking terrain, with probes `FREE_DISTANCE_STEP_METERS` apart, or `max_distance` without
// one. The skips leave out only probes that are known to be free, so they never change the result.
float measure_free_distance(V2 point, V2 direction, int terrain_class, float max_distance) {
    Ray ray = {
        .start = point,
        .direction = direction,
        .inverse_x = get_inverse(direction.x),
        .inverse_y = get_inverse(direction.y),
    };
    float d = 0;        // ray distance of the probe
    while (d < max_distance) {
        V2 probe = add(point, mul(direction, d));
        int cell_x;
        int cell_y;
        if (gc_read_cell_at(probe.x, probe.y, &cell_x, &cell_y) > terrain_class)
            return d;
        float cell_skip = find_free_cell_skip(&ray, cell_x, cell_y, d);
        float clearance_skip = get_clearance_skip(cell_x, cell_y, terrain_class);
        d += fmaxf(cell_skip, clearance_skip);
    }
    return max_distance;
}


// `measure_points_length` returns the length of the polyline through `points`.
float measure_points_length(const V2 *points, int point_count) {
    float length = 0;
    for (int point_index = 1; point_index < point_count; point_index++)
        length += len(sub(points[point_index], points[point_index - 1]));
    return length;
}


// `is_band_clear` tells whether a band from `from` to `to`, `half_width` to each side, is clear of
// blocking terrain. It tests the middle line and the two edge lines of the band.
static bool is_band_clear(V2 from, V2 to, float half_width, int terrain_class) {
    V2 line = sub(to, from);
    float length = len(line);
    if (length < ROUTE_EPSILON_METERS)
        return true;
    V2 right_offset = mul(get_right(line), half_width / length);
    V2 left_from = sub(from, right_offset);
    V2 left_to = sub(to, right_offset);
    V2 right_from = add(from, right_offset);
    V2 right_to = add(to, right_offset);

    if (!is_line_clear(from, to, terrain_class))
        return false;
    if (!is_line_clear(right_from, right_to, terrain_class))
        return false;
    return is_line_clear(left_from, left_to, terrain_class);
}


// `cut_corners` moves each inner route corner along its next segment, as far as the band from the
// previous route point to the new corner stays clear. The band reaches the corner cut margin of
// the faction tuning past the widest radius to each side, and a bisection of `CORNER_CUT_STEPS`
// steps finds the cut.
static void cut_corners(Squad *squad, float max_radius, int terrain_class) {
    float half_width = max_radius + get_tuning(squad->unit_type)->corner_cut_margin_meters;
    for (int corner_index = 1; corner_index < squad->route_count - 1; corner_index++) {
        // `clear` and `blocked` are fractions of the next segment with a clear and a blocked band.
        V2 before = squad->route[corner_index - 1];
        V2 corner = squad->route[corner_index];
        V2 after = squad->route[corner_index + 1];
        V2 segment = sub(after, corner);
        float clear = 0;
        float blocked = 1;
        for (int step_index = 0; step_index < CORNER_CUT_STEPS; step_index++) {
            float middle = (clear + blocked) / 2;
            V2 point = add(corner, mul(segment, middle));
            if (is_band_clear(before, point, half_width, terrain_class))
                clear = middle;
            else
                blocked = middle;
        }
        squad->route[corner_index] = add(corner, mul(segment, clear));
    }
}


// `shorten_route` removes route corners of a squad: from each kept route point, the route goes
// straight to the farthest later point with a clear band of the widest radius on each side. This
// removes corners and backtracking that would swing every slot around. It then cuts the corners.
void shorten_route(Squad *squad, float max_radius, int terrain_class) {
    int last_index = squad->route_count - 1;
    int kept_count = 1;
    int from_index = 0;
    while (from_index < last_index) {
        V2 from = squad->route[from_index];
        int to_index = last_index;
        while (to_index > from_index + 1) {
            V2 to = squad->route[to_index];
            if (is_band_clear(from, to, max_radius, terrain_class))
                break;
            to_index--;
        }
        squad->route[kept_count] = squad->route[to_index];
        kept_count++;
        from_index = to_index;
    }
    squad->route_count = kept_count;
    cut_corners(squad, max_radius, terrain_class);
}


// `can_start_route_at` tells whether a route can start at `point`: on a cell that the unit can
// cross, with a clear line to `next_point`.
static bool can_start_route_at(V2 point, V2 next_point, int terrain_class) {
    bool is_passable = gc_get_cell_at(point.x, point.y) <= terrain_class;
    return is_passable && is_line_clear(point, next_point, terrain_class);
}


// `move_route_start` moves the start of the route of a squad, which the base game path search puts
// at the front unit, to where the shape of the final parking positions, laid along the first route
// segment, has its mean at `center`. When that point is not usable, the route starts at `center`.
void move_route_start(Squad *squad, const float *parking_positions, V2 facing, V2 center,
                      int terrain_class) {
    // The mean offset of the final parking positions is relative to the route end, in the frame
    // of the final facing.
    V2 end = squad->route[squad->route_count - 1];
    V2 mean_offset = v2(0, 0);
    float mean_weight = 1.0f / squad->unit_count;
    for (int parking_index = 0; parking_index < squad->unit_count; parking_index++) {
        V2 offset = get_parking_offset(parking_positions, parking_index, end, facing);
        mean_offset = add(mean_offset, mul(offset, mean_weight));
    }

    // A route start at neither point keeps the start of the path search.
    V2 next_point = squad->route[1];
    V2 first_segment = get_direction(sub(next_point, center), LENGTH_SQUARED_EPSILON, facing);
    V2 best_fit = sub(center, get_map_offset(mean_offset, first_segment));
    if (can_start_route_at(best_fit, next_point, terrain_class))
        squad->route[0] = best_fit;
    else if (can_start_route_at(center, next_point, terrain_class))
        squad->route[0] = center;
}


// `wrap_angle` returns `angle` moved by whole turns into `[0, 2 * PI)`.
static float wrap_angle(float angle) {
    float wrapped = fmodf(angle, 2 * PI);
    if (wrapped < 0)
        wrapped += 2 * PI;
    return wrapped;
}


// `find_lead_in_points` puts the lead-in points of a turn to the side `side` (1 to the left, -1 to
// the right, seen facing along `motion_direction`) into `points`, and returns their number, or 0
// when the next route point lies inside that turning circle or needs almost no turn.
static int find_lead_in_points(V2 start, V2 motion_direction, V2 next_point, float turn_radius,
                               float side, V2 *points) {
    // On the circle around `center`, the angle `a` gives the point `center + r * (cos a, sin a)`.
    // At the tangent point, the circle tangent along the turn points at the next route point.
    V2 left = v2(-motion_direction.y, motion_direction.x);
    V2 center = add(start, mul(left, side * turn_radius));
    V2 q = sub(next_point, center);        // next route point seen from the center
    float q_length = len(q);
    if (q_length <= turn_radius)
        return 0;
    float a_start = atan2f(start.y - center.y, start.x - center.x);
    float a_tangent = atan2f(q.y, q.x) - side * acosf(turn_radius / q_length);
    float sweep = wrap_angle(side * (a_tangent - a_start));        // turned angle
    if (sweep < LEAD_IN_MIN_SWEEP_RADIANS)
        return 0;

    // The points split the turn into equal steps of at most `LEAD_IN_STEP_RADIANS`.
    int count = (int)ceilf(sweep / LEAD_IN_STEP_RADIANS);
    for (int point_index = 0; point_index < count; point_index++) {
        float a = a_start + side * sweep * (point_index + 1) / count;
        points[point_index] = add(center, mul(v2(cosf(a), sinf(a)), turn_radius));
    }
    return count;
}


// `add_lead_in_points` tries the turn toward the side of the next route point first, and then the
// turn to the other side, which a next route point inside the first turning circle needs.
//
//                     next route point
//                          ^
//                         /  <- tangent of the turning circle
//                   T . '
//                 .        C: center, one turning radius to the turn side of the start
//                .    C    T: last lead-in point, where the tangent points at the next point
//   start --> .
//
void add_lead_in_points(Squad *squad, V2 motion_direction, float turn_radius, float max_radius,
                        int terrain_class) {
    if (squad->route_count < 2)
        return;
    V2 start = squad->route[0];
    V2 next_point = squad->route[1];
    float first_side = 1;
    if (det2(motion_direction, sub(next_point, start)) < 0)
        first_side = -1;

    // A turn whose band from point to point crosses blocking terrain is not taken.
    float sides[] = {first_side, -first_side};
    for (int side_index = 0; side_index < 2; side_index++) {
        V2 points[LEAD_IN_MAX_POINTS];
        int count = find_lead_in_points(start, motion_direction, next_point, turn_radius,
                                        sides[side_index], points);
        bool has_room = count > 0 && squad->route_count + count <= MAX_WAYPOINTS;
        if (!has_room)
            continue;
        bool is_clear = is_band_clear(start, points[0], max_radius, terrain_class);
        for (int point_index = 1; point_index < count && is_clear; point_index++)
            is_clear = is_band_clear(points[point_index - 1], points[point_index], max_radius,
                                     terrain_class);
        is_clear = is_clear && is_band_clear(points[count - 1], next_point, max_radius,
                                             terrain_class);
        if (!is_clear)
            continue;
        size_t moved_bytes = (size_t)(squad->route_count - 1) * sizeof(V2);
        memmove(&squad->route[1 + count], &squad->route[1], moved_bytes);
        memcpy(&squad->route[1], points, (size_t)count * sizeof(V2));
        squad->route_count += count;
        squad->lead_in_point_count = count;
        return;
    }
}


// `take_shorter_centered_route` runs a centered game search from `center` to the destination of a
// squad, prepares its route as the main route, and takes it when it is at least
// `CENTERED_ROUTE_MIN_GAIN_METERS` shorter than the current route of the squad.
void take_shorter_centered_route(Squad *squad, const float *parking_positions, V2 facing,
                                 V2 center, float max_radius, int terrain_class,
                                 float corner_gap) {
    // The base game path search adds corners around the first blocking terrain on the line to the
    // destination, so its start decides the way around. A backward search follows a failure.
    GcRequest search;
    gc_init_request(&search);
    V2 end = squad->destination;
    gc_start_search(&search, GC_SEARCH_FORWARD, center.x, center.y, end.x, end.y, terrain_class,
                    corner_gap, CENTERED_SEARCH_TOLERANCE_METERS);
    bool is_found = gc_update_search(&search, CENTERED_SEARCH_BUDGET) == GC_SEARCH_FOUND;
    if (!is_found) {
        gc_start_search(&search, GC_SEARCH_BACKWARD, center.x, center.y, end.x, end.y,
                        terrain_class, corner_gap, CENTERED_SEARCH_TOLERANCE_METERS);
        is_found = gc_update_search(&search, CENTERED_SEARCH_BUDGET) == GC_SEARCH_FOUND;
    }
    int count = 0;        // number of path points
    if (is_found)
        count = gc_get_path(&search, path_points, MAX_WAYPOINTS - 1);
    gc_free_request(&search);
    if (count == 0)
        return;

    // The candidate is a copy of the squad with the new route, which starts at `center`.
    static Squad candidate;
    candidate = *squad;
    candidate.route[0] = center;
    for (int point_index = 0; point_index < count; point_index++)
        candidate.route[point_index + 1] = get_pair_point(path_points, point_index);
    candidate.route_count = count + 1;
    move_route_start(&candidate, parking_positions, facing, center, terrain_class);
    shorten_route(&candidate, max_radius, terrain_class);

    float length = measure_points_length(squad->route, squad->route_count);
    float candidate_length = measure_points_length(candidate.route, candidate.route_count);
    float gain = length - candidate_length;
    if (gain < CENTERED_ROUTE_MIN_GAIN_METERS)
        return;
    size_t route_bytes = (size_t)candidate.route_count * sizeof(V2);
    memcpy(squad->route, candidate.route, route_bytes);
    squad->route_count = candidate.route_count;
}
