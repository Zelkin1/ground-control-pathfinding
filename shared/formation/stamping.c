// Parked squads as blocking terrain for searches. In the formation modes, the route search of a
// moving squad and the layout of its final parking positions treat parked squads as cliffs:
// `stamp_parked` writes cliff cells over them into the grid of this thread, and `unstamp_parked`
// restores the grid afterward. A re-routing squad stamps whole parking discs; any other squad
// stamps only the bodies of the stopped units of parked squads.
//
// With `--wholeparked`, `shift_parking_positions` moves the whole final formation of a moving
// squad that overlaps a parked squad to the nearest spot where it fits as laid out. With
// `--freeparking`, `free_parking_positions` then moves each final parking position that still
// overlaps the body of a parked unit to the nearest free spot. With `--moveahead`, a squad that
// re-routes around the path of a moving squad also stamps the discs that its trial stored, with
// the radius of the route searches or the smaller radius of the set-up.
#include "sim.h"

#define MAX_STAMPED_CELLS 4096                    // most stamped cells per search
#define PARKED_STAMP_MARGIN_METERS 1.0f           // stamp margin past the parked bodies
#define FREE_PARKING_MAX_RINGS 12                 // rings of candidate spots around a position
#define FREE_PARKING_RING_POINTS 6                // candidate spots per ring, times the ring number
#define NO_PARKED_UNIT (-1)                       // no parked unit overlaps
#define NO_PARKED_AREA (-1)                       // no parked area overlaps
#define WHOLE_PARKING_RING_METERS 2.0f            // distance between rings of formation shifts
#define WHOLE_PARKING_MAX_RINGS 25                // rings of formation shifts around the layout
#define FNV_OFFSET_BASIS 2166136261u              // start value of the FNV-1a hash
#define FNV_PRIME 16777619u                       // multiplier of the FNV-1a hash

// Stamped cells and their original values, in stamping order.
static int stamped_cell_indices[MAX_STAMPED_CELLS];
static int stamped_values[MAX_STAMPED_CELLS];
static _Thread_local int stamped_count;
static _Thread_local StampBox stamp_box;
static _Thread_local bool is_search_stamping;        // whether the stamps are for a route search


// `get_truncated_cell_coordinate` returns the cell coordinate of `meters`, truncated toward zero.
static int get_truncated_cell_coordinate(float meters) {
    return (int)(meters / GC_CELL_METERS);
}


// `get_cell_index` returns the cell index of the cell under `point`.
static int get_cell_index(V2 point) {
    int cell_x = get_truncated_cell_coordinate(point.x);
    int cell_y = get_truncated_cell_coordinate(point.y);
    return make_cell_index(cell_x, cell_y);
}


// `grow_stamp_box` grows the stamp box so that it holds the cell `(cell_x, cell_y)`.
static void grow_stamp_box(int cell_x, int cell_y) {
    if (!stamp_box.has_cells) {
        stamp_box = (StampBox){
            .has_cells = true,
            .min_x = cell_x,
            .min_y = cell_y,
            .max_x = cell_x,
            .max_y = cell_y,
        };
        return;
    }
    if (cell_x < stamp_box.min_x)
        stamp_box.min_x = cell_x;
    if (cell_x > stamp_box.max_x)
        stamp_box.max_x = cell_x;
    if (cell_y < stamp_box.min_y)
        stamp_box.min_y = cell_y;
    if (cell_y > stamp_box.max_y)
        stamp_box.max_y = cell_y;
}


// `get_box_axis_distance` returns the distance from `value` to the range from `min_value` to
// `max_value` on one axis, or zero inside it.
static int get_box_axis_distance(int value, int min_value, int max_value) {
    if (value < min_value)
        return min_value - value;
    if (value > max_value)
        return value - max_value;
    return 0;
}


// `get_stamp_distance_cells` returns the chessboard distance in cells from the cell
// `(cell_x, cell_y)` to the stamp box of this thread, or `NO_STAMP_DISTANCE_CELLS` without stamps.
int get_stamp_distance_cells(int cell_x, int cell_y) {
    if (!stamp_box.has_cells)
        return NO_STAMP_DISTANCE_CELLS;
    int dx = get_box_axis_distance(cell_x, stamp_box.min_x, stamp_box.max_x);
    int dy = get_box_axis_distance(cell_y, stamp_box.min_y, stamp_box.max_y);
    return dx > dy ? dx : dy;
}


// `get_stamp_box` returns the stamp box of this thread.
StampBox get_stamp_box(void) {
    return stamp_box;
}


// `set_stamp_box` replaces the stamp box of this thread.
void set_stamp_box(StampBox box) {
    stamp_box = box;
}


// `stamp_cell` stamps the cell `(cell_x, cell_y)` as a cliff. The cells of the search start and
// end stay free, so the search can start and end, and cells that already are cliffs are not
// stamped. Stamps past `MAX_STAMPED_CELLS` are dropped.
static void stamp_cell(int cell_x, int cell_y, int start_cell_index, int end_cell_index) {
    int cell_index = make_cell_index(cell_x, cell_y);
    int cell_value = gc_get_cell(cell_x, cell_y);
    bool is_search_end = cell_index == start_cell_index || cell_index == end_cell_index;
    bool is_kept = is_search_end || cell_value == GC_CELL_CLIFF;
    if (is_kept || stamped_count >= MAX_STAMPED_CELLS)
        return;
    stamped_cell_indices[stamped_count] = cell_index;
    stamped_values[stamped_count] = cell_value;
    grow_stamp_box(cell_x, cell_y);
    stamped_count++;
    gc_set_cell(cell_x, cell_y, GC_CELL_CLIFF);
}


// `stamp_parking_disc` stamps as cliffs the cells that a disc of `radius` around `center` touches.
static void stamp_parking_disc(V2 center, float radius, int start_cell_index, int end_cell_index) {
    int min_cell_y = get_truncated_cell_coordinate(center.y - radius);
    int max_cell_y = get_truncated_cell_coordinate(center.y + radius);
    int min_cell_x = get_truncated_cell_coordinate(center.x - radius);
    int max_cell_x = get_truncated_cell_coordinate(center.x + radius);
    for (int cell_y = min_cell_y; cell_y <= max_cell_y; cell_y++) {
        for (int cell_x = min_cell_x; cell_x <= max_cell_x; cell_x++) {
            // A cell touches the disc when its closest point to the center lies inside it.
            V2 closest = get_closest_cell_point(cell_x, cell_y, center);
            if (length_squared(sub(closest, center)) < radius * radius)
                stamp_cell(cell_x, cell_y, start_cell_index, end_cell_index);
        }
    }
}


// `get_closest_segment_point` returns the point of the segment from `a` to `b` closest to `point`.
V2 get_closest_segment_point(V2 point, V2 a, V2 b) {
    V2 segment = sub(b, a);
    float l_squared = length_squared(segment);        // squared segment length
    if (l_squared <= 0)
        return a;
    float t = clamp_float(dot(sub(point, a), segment) / l_squared, 0, 1);        // fraction
    return add(a, mul(segment, t));
}


// `get_closest_hull_point` returns the point of the convex polygon `hull` of `count` corners,
// counterclockwise, closest to `point`: `point` itself when it lies inside.
static V2 get_closest_hull_point(V2 point, const V2 *hull, int count) {
    // A point on the left of every edge, seen along the edge, lies inside.
    bool is_inside = count >= 3;
    V2 closest = hull[0];
    float best_squared = length_squared(sub(point, closest));
    for (int corner_index = 0; corner_index < count; corner_index++) {
        V2 a = hull[corner_index];
        V2 b = hull[(corner_index + 1) % count];
        if (det2(sub(b, a), sub(point, a)) < 0)
            is_inside = false;
        V2 on_edge = get_closest_segment_point(point, a, b);
        float d_squared = length_squared(sub(point, on_edge));
        if (d_squared < best_squared) {
            closest = on_edge;
            best_squared = d_squared;
        }
    }
    return is_inside ? point : closest;
}


// `get_hull_distance` returns the distance from `point` to the convex polygon `hull`, 0 inside.
static float get_hull_distance(V2 point, const V2 *hull, int count) {
    return len(sub(point, get_closest_hull_point(point, hull, count)));
}


// `is_stamped_member` tells whether a squad member counts in the stamps of an order search: a
// stopped unit that no APC carries.
static bool is_stamped_member(const Unit *member) {
    return member->is_stopped && !member->is_carried;
}


// `find_stopped_hull` finds the convex hull of the stamped members of `parked` by gift wrapping.
//
//    start: leftmost point; each next corner leaves every other point on its left
//
int find_stopped_hull(const Squad *parked, V2 *hull, float *max_radius) {
    // List the stamped members, and find the leftmost one.
    V2 points[MAX_MEMBERS];
    int point_count = 0;
    int start_index = 0;
    *max_radius = 0;
    int end_index = parked->first_unit_index + parked->unit_count;
    for (int unit_index = parked->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *member = &units[unit_index];
        if (!is_stamped_member(member) || point_count == MAX_MEMBERS)
            continue;
        points[point_count] = member->position;
        if (member->position.x < points[start_index].x)
            start_index = point_count;
        *max_radius = fmaxf(*max_radius, member->unit_type->radius);
        point_count++;
    }
    if (point_count == 0)
        return 0;

    // Each step takes the point that leaves all others on its left, or among points on one line,
    // the farthest. The wrap ends back at the start, or after all points.
    int count = 0;
    int current_index = start_index;
    do {
        hull[count] = points[current_index];
        count++;
        int next_index = (current_index + 1) % point_count;
        for (int candidate_index = 0; candidate_index < point_count; candidate_index++) {
            V2 to_next = sub(points[next_index], points[current_index]);
            V2 to_candidate = sub(points[candidate_index], points[current_index]);
            float side = det2(to_next, to_candidate);
            bool is_farther = length_squared(to_candidate) > length_squared(to_next);
            if (side < 0 || (side == 0 && is_farther))
                next_index = candidate_index;
        }
        current_index = next_index;
    } while (current_index != start_index && count < point_count);
    return count;
}


// `stamp_unit` stamps the disc of the body of a parked unit, plus `PARKED_STAMP_MARGIN_METERS`.
static void stamp_unit(const Unit *parked, int start_cell_index, int end_cell_index) {
    float radius = parked->unit_type->radius + PARKED_STAMP_MARGIN_METERS;
    stamp_parking_disc(parked->position, radius, start_cell_index, end_cell_index);
}


// `stamp_squad_parking_disc` stamps the parking disc of a squad: the disc around the mean point
// of its squad members that holds all their bodies, plus `PARKED_STAMP_MARGIN_METERS`.
// `is_at_parking_positions` reads final parking positions instead of positions.
static void stamp_squad_parking_disc(const Squad *parked, bool is_at_parking_positions,
                                     int start_cell_index, int end_cell_index) {
    V2 sum = v2(0, 0);        // sum of the member points
    int end_index = parked->first_unit_index + parked->unit_count;
    for (int unit_index = parked->first_unit_index; unit_index < end_index; unit_index++) {
        V2 member_point = get_member_point(&units[unit_index], is_at_parking_positions);
        sum = add(sum, member_point);
    }
    V2 center = mul(sum, 1.0f / parked->unit_count);

    float farthest = 0;        // farthest edge of a body from the center
    for (int unit_index = parked->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *member = &units[unit_index];
        V2 member_point = get_member_point(member, is_at_parking_positions);
        float edge = len(sub(member_point, center)) + member->unit_type->radius;
        farthest = fmaxf(farthest, edge);
    }
    float radius = farthest + PARKED_STAMP_MARGIN_METERS;
    stamp_parking_disc(center, radius, start_cell_index, end_cell_index);
}


// `stamp_parked` stamps the parked squads as cliffs for a search of `moving_squad` from `start` to
// `end`, in the formation modes. A re-routing squad stamps the parking discs of the parked and
// settled squads and of the squads that it avoids parking ahead, among the squads that coordinate
// with it (`are_squads_coordinated`); any other squad stamps the bodies of the stopped units of
// parked squads. During a re-route around the path of a moving squad (`--moveahead`), it also
// stamps the discs that the trial of that re-route stored, with their set-up radius.
void stamp_parked(const Squad *moving_squad, V2 start, V2 end) {
    stamped_count = 0;
    stamp_box.has_cells = false;
    if (avoid_mode == AVOID_ORIGINAL)
        return;

    // The parking discs of the squads parking ahead lie on their final parking positions.
    int start_cell_index = get_cell_index(start);
    int end_cell_index = get_cell_index(end);
    if (moving_squad->is_rerouting) {
        for (int squad_index = 0; squad_index < squad_count; squad_index++) {
            const Squad *other_squad = &squads[squad_index];
            bool is_other_squad = other_squad != moving_squad;
            is_other_squad = is_other_squad && are_squads_coordinated(moving_squad, other_squad);
            bool is_parked_squad = !is_squad_busy(other_squad) || other_squad->is_settled;
            bool is_parking_ahead = moving_squad->is_avoiding_parking[squad_index];
            if (is_other_squad && is_parked_squad)
                stamp_squad_parking_disc(other_squad, false, start_cell_index, end_cell_index);
            else if (is_other_squad && is_parking_ahead)
                stamp_squad_parking_disc(other_squad, true, start_cell_index, end_cell_index);
        }
        if (!moving_squad->is_move_ahead_reroute)
            return;
        for (int disc_index = 0; disc_index < moving_squad->move_ahead_stamp_count; disc_index++) {
            V2 center = moving_squad->move_ahead_stamp_centers[disc_index];
            float radius = moving_squad->move_ahead_set_up_stamp_radii[disc_index];
            if (is_search_stamping)
                radius = moving_squad->move_ahead_stamp_radii[disc_index];
            stamp_parking_disc(center, radius, start_cell_index, end_cell_index);
        }
        return;
    }

    // With `--ignoreboarders`, the units of a squad that boards the moving squad get no stamp, so
    // that its route does not go around them.
    for (int other_index = 0; other_index < unit_count; other_index++) {
        const Unit *other = &units[other_index];
        const Squad *other_squad = &squads[other->squad_index];
        bool is_other_squad = other_squad != moving_squad && !other->is_carried;
        bool is_boarding_squad = is_squad_boarding(other_squad, moving_squad);
        is_other_squad = is_other_squad && !(are_boarders_ignored && is_boarding_squad);
        bool is_parked = other->is_stopped && !is_squad_busy(other_squad);
        if (is_other_squad && is_parked)
            stamp_unit(other, start_cell_index, end_cell_index);
    }
}


// `find_parked_overlap` returns the index of a stopped unit of a parked squad other than
// `moving_squad` whose body, plus `PARKED_STAMP_MARGIN_METERS`, overlaps a body of `radius` at
// `point`, or `NO_PARKED_UNIT`. These are the units that the search of an order stamps.
static int find_parked_overlap(const Squad *moving_squad, V2 point, float radius) {
    for (int other_index = 0; other_index < unit_count; other_index++) {
        const Unit *other = &units[other_index];
        const Squad *other_squad = &squads[other->squad_index];
        bool is_other_squad = other_squad != moving_squad && !other->is_carried;
        bool is_parked = other->is_stopped && !is_squad_busy(other_squad);
        if (!is_other_squad || !is_parked)
            continue;
        float gap = other->unit_type->radius + radius + PARKED_STAMP_MARGIN_METERS;
        if (length_squared(sub(other->position, point)) < gap * gap)
            return other_index;
    }
    return NO_PARKED_UNIT;
}


// `is_free_parking_spot` tells whether a squad member of `radius` can park at `point`: clear of
// blocking terrain, of parked units, and of the other final parking positions of its squad, which
// `parking_positions` holds as x, y pairs.
static bool is_free_parking_spot(const Squad *squad, const float *parking_positions,
                                 int parking_index, V2 point, float radius, int terrain_class) {
    for (int other_index = 0; other_index < squad->unit_count; other_index++) {
        V2 other = get_pair_point(parking_positions, other_index);
        bool is_too_close = length_squared(sub(other, point)) < 4 * radius * radius;
        if (other_index != parking_index && is_too_close)
            return false;
    }
    if (find_parked_overlap(squad, point, radius) != NO_PARKED_UNIT)
        return false;
    return is_circle_clear(point, radius, terrain_class);
}


// `get_ring_offset` returns the offset of the candidate spot `point_index` of `point_count` on
// the ring `ring_index`, rings being `spacing` apart. The spots start at the angle `away_angle`
// and alternate sides: 0, +1, -1, +2, -2 steps from it.
static V2 get_ring_offset(float away_angle, int ring_index, int point_index, int point_count,
                          float spacing) {
    float step = 2 * PI / point_count;        // angle between two candidate spots
    int side_steps = (point_index + 1) / 2;
    float side = point_index % 2 == 1 ? 1.0f : -1.0f;
    float a = away_angle + side * side_steps * step;        // angle of the candidate
    return mul(v2(cosf(a), sinf(a)), ring_index * spacing);
}


// `find_free_parking_spot` searches rings of candidate spots around `position`, one unit radius
// apart, and returns true with the first free spot in `spot`. Each ring starts along `away`, the
// direction from the parked unit to `position`, and goes on by turns to both sides of it.
static bool find_free_parking_spot(const Squad *squad, const float *parking_positions,
                                   int parking_index, V2 position, V2 away, float radius,
                                   int terrain_class, V2 *spot) {
    float away_angle = atan2f(away.y, away.x);
    for (int ring_index = 1; ring_index <= FREE_PARKING_MAX_RINGS; ring_index++) {
        int point_count = FREE_PARKING_RING_POINTS * ring_index;
        for (int point_index = 0; point_index < point_count; point_index++) {
            V2 offset = get_ring_offset(away_angle, ring_index, point_index, point_count, radius);
            V2 candidate = add(position, offset);
            if (is_free_parking_spot(squad, parking_positions, parking_index, candidate, radius,
                                     terrain_class)) {
                *spot = candidate;
                return true;
            }
        }
    }
    return false;
}


// `free_parking_positions` moves each final parking position of `squad`, in `parking_positions`
// as x, y pairs, whose body would overlap a parked unit to the nearest free spot. A position
// without a free spot within `FREE_PARKING_MAX_RINGS` unit radii stays.
void free_parking_positions(const Squad *squad, float *parking_positions) {
    const Unit *first = &units[squad->first_unit_index];
    float radius = first->unit_type->radius;
    int terrain_class = first->unit_type->terrain_class;
    for (int parking_index = 0; parking_index < squad->unit_count; parking_index++) {
        V2 position = get_pair_point(parking_positions, parking_index);
        int parked_index = find_parked_overlap(squad, position, radius);
        if (parked_index == NO_PARKED_UNIT)
            continue;
        V2 from_parked = sub(position, units[parked_index].position);
        V2 away = get_direction(from_parked, 0, v2(1, 0));
        V2 spot;
        if (!find_free_parking_spot(squad, parking_positions, parking_index, position, away,
                                    radius, terrain_class, &spot))
            continue;
        parking_positions[2 * parking_index] = spot.x;
        parking_positions[2 * parking_index + 1] = spot.y;
    }
}


// Area of a parked squad for `shift_parking_positions`: the convex hull of the positions of its
// stopped members, and `reach`, the gap that keeps a body center off a body of the squad, their
// largest radius plus `PARKED_STAMP_MARGIN_METERS`.
typedef struct {
    V2 hull[MAX_MEMBERS];
    int corner_count;
    float reach;
} ParkedArea;

// Areas of all parked squads other than the moving squad.
typedef struct {
    ParkedArea areas[MAX_SQUADS];
    int count;
} ParkedAreas;


// `find_parked_areas` fills `parked` with the area of each parked squad other than `squad` that
// has stopped members.
static void find_parked_areas(const Squad *squad, ParkedAreas *parked) {
    parked->count = 0;
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        const Squad *other_squad = &squads[squad_index];
        if (other_squad == squad || is_squad_busy(other_squad))
            continue;
        ParkedArea *area = &parked->areas[parked->count];
        float max_radius;
        area->corner_count = find_stopped_hull(other_squad, area->hull, &max_radius);
        area->reach = max_radius + PARKED_STAMP_MARGIN_METERS;
        if (area->corner_count > 0)
            parked->count++;
    }
}


// `find_overlapped_area` returns the index of the first area of `parked` that a body of `radius`
// at `point` comes closer to than its reach, or `NO_PARKED_AREA`.
static int find_overlapped_area(const ParkedAreas *parked, V2 point, float radius) {
    for (int area_index = 0; area_index < parked->count; area_index++) {
        const ParkedArea *area = &parked->areas[area_index];
        float distance = get_hull_distance(point, area->hull, area->corner_count);
        if (distance < area->reach + radius)
            return area_index;
    }
    return NO_PARKED_AREA;
}


// `get_area_center` returns the mean of the hull corners of `area`.
static V2 get_area_center(const ParkedArea *area) {
    V2 sum = v2(0, 0);
    for (int corner_index = 0; corner_index < area->corner_count; corner_index++)
        sum = add(sum, area->hull[corner_index]);
    return mul(sum, 1.0f / area->corner_count);
}


// `is_formation_spot_free` tells whether the final parking positions of `squad`, in
// `parking_positions` as x, y pairs, moved by `offset`, keep the bodies of its squad members clear
// of blocking terrain and of the areas of `parked`.
static bool is_formation_spot_free(const Squad *squad, const float *parking_positions, V2 offset,
                                   const ParkedAreas *parked) {
    const Unit *first = &units[squad->first_unit_index];
    float radius = first->unit_type->radius;
    int terrain_class = first->unit_type->terrain_class;
    for (int parking_index = 0; parking_index < squad->unit_count; parking_index++) {
        V2 point = add(get_pair_point(parking_positions, parking_index), offset);
        if (find_overlapped_area(parked, point, radius) != NO_PARKED_AREA)
            return false;
        if (!is_circle_clear(point, radius, terrain_class))
            return false;
    }
    return true;
}


// `find_overlap_away_direction` returns the unit direction from the center of the first parked
// area that a final parking position of `squad` overlaps to the center of all its final parking
// positions, or false when no position overlaps an area.
static bool find_overlap_away_direction(const Squad *squad, const float *parking_positions,
                                        const ParkedAreas *parked, V2 *away) {
    float radius = units[squad->first_unit_index].unit_type->radius;
    int overlapped_index = NO_PARKED_AREA;
    V2 sum = v2(0, 0);        // sum of the final parking positions
    for (int parking_index = 0; parking_index < squad->unit_count; parking_index++) {
        V2 position = get_pair_point(parking_positions, parking_index);
        sum = add(sum, position);
        if (overlapped_index == NO_PARKED_AREA)
            overlapped_index = find_overlapped_area(parked, position, radius);
    }
    if (overlapped_index == NO_PARKED_AREA)
        return false;
    V2 center = mul(sum, 1.0f / squad->unit_count);
    V2 area_center = get_area_center(&parked->areas[overlapped_index]);
    *away = get_direction(sub(center, area_center), 0, v2(1, 0));
    return true;
}


// `move_formation_end` moves all final parking positions of `squad` and the end of its route by
// `offset`. A route end left beside the parked squad would narrow the lanes before it, against the
// stamps of the set-up, so the squad would close up into column formation and slow down there.
static void move_formation_end(Squad *squad, float *parking_positions, V2 offset) {
    for (int parking_index = 0; parking_index < squad->unit_count; parking_index++) {
        parking_positions[2 * parking_index] += offset.x;
        parking_positions[2 * parking_index + 1] += offset.y;
    }
    V2 *route_end = &squad->route[squad->route_count - 1];
    *route_end = add(*route_end, offset);
}


// `shift_parking_positions` searches rings of shifts of the whole final formation around its
// layout, `WHOLE_PARKING_RING_METERS` apart, nearest first, and moves the formation by the first
// shift that fits. Each ring starts on the side away from the overlapped area.
void shift_parking_positions(Squad *squad, float *parking_positions) {
    // Only a layout that overlaps a parked area moves; one on blocking terrain alone stays.
    ParkedAreas parked;
    find_parked_areas(squad, &parked);
    V2 away;
    if (!find_overlap_away_direction(squad, parking_positions, &parked, &away))
        return;

    float away_angle = atan2f(away.y, away.x);
    for (int ring_index = 1; ring_index <= WHOLE_PARKING_MAX_RINGS; ring_index++) {
        int point_count = FREE_PARKING_RING_POINTS * ring_index;
        for (int point_index = 0; point_index < point_count; point_index++) {
            V2 offset = get_ring_offset(away_angle, ring_index, point_index, point_count,
                                        WHOLE_PARKING_RING_METERS);
            if (!is_formation_spot_free(squad, parking_positions, offset, &parked))
                continue;
            move_formation_end(squad, parking_positions, offset);
            return;
        }
    }
}


// `stamp_parked_for_search` stamps as `stamp_parked` does, but the discs of a move-ahead re-route
// get their search radius.
void stamp_parked_for_search(const Squad *moving_squad, V2 start, V2 end) {
    is_search_stamping = true;
    stamp_parked(moving_squad, start, end);
    is_search_stamping = false;
}


// `unstamp_parked` restores the original values of the stamped cells, in reverse stamping order.
void unstamp_parked(void) {
    while (stamped_count > 0) {
        stamped_count--;
        int cell_index = stamped_cell_indices[stamped_count];
        int cell_x = get_cell_x(cell_index);
        int cell_y = get_cell_y(cell_index);
        gc_set_cell(cell_x, cell_y, stamped_values[stamped_count]);
    }
    stamp_box.has_cells = false;
}


// `hash_stamped_cells` returns the FNV-1a hash of the stamped cells and their original values,
// which changes whenever the set of stamps changes.
uint32_t hash_stamped_cells(void) {
    uint32_t hash = FNV_OFFSET_BASIS;
    for (int stamp_index = 0; stamp_index < stamped_count; stamp_index++) {
        hash = (hash ^ (uint32_t)stamped_cell_indices[stamp_index]) * FNV_PRIME;
        hash = (hash ^ (uint32_t)stamped_values[stamp_index]) * FNV_PRIME;
    }
    return hash;
}
