// Route finder for `sim.exe --find`, which measures random routes on a map to help pick scenarios.
// For each try, `find_spots` draws two route ends with enough clearance, runs the base game path
// search for a medium tank between them, and prints the straight distance, the ratio of the route
// length to it, the number of turns, and the half width of the narrowest passage on the route.
//
// The clearance here comes from a chamfer distance transform of the grid: steps to the four
// neighbors in a row or column cost 3, diagonal steps cost 4, so a distance in thirds of a cell is
// close to the Euclidean distance to the nearest blocking cell.
#include <stdlib.h>

#include "sim.h"

#define MAP_SIZE_METERS 6400.0f                // side of the square map
#define WORLD_MIN_METERS 10.0f                 // smallest x or y of a route end
#define WORLD_MAX_METERS 6390.0f               // largest x or y of a route end
#define FIND_RANDOM_SEED 12345u                // seed of the random route ends
#define MIN_SPOT_CLEARANCE_METERS 25.0f        // smallest clearance of a route end
#define MAX_START_ATTEMPTS 1000                // most draws of a start
#define MAX_SPOT_ATTEMPTS 2000                 // most draws of a start and an end together
#define MIN_ROUTE_METERS 250.0f                // shortest straight distance of a route
#define ROUTE_RANGE_METERS 450.0f              // random distance on top of that
#define FIND_SEARCH_BUDGET 500                 // expansion budget of each search
#define TURN_DEGREES 30.0f                     // sharper bends count as turns
#define HALF_TURN_DEGREES 180                  // degrees of a half turn
#define MEASURE_STEP_METERS 2.0f               // step of the passage width measurement
#define END_EXCLUSION_METERS 30.0f             // no measurement this close to a route end
#define HALF_WIDTH_SEARCH_CELLS 4              // reach of the half width search, in cells
#define MIN_SEGMENT_METERS 1e-3f               // shorter path segments are skipped
#define NO_PASSAGE_METERS 1e9f                 // half width of a route without a passage

#define STRAIGHT_STEP_COST 3                   // chamfer cost of a step in a row or column
#define DIAGONAL_STEP_COST 4                   // chamfer cost of a diagonal step
#define FAR_COST (1 << 28)                     // start value of a free cell
#define NEIGHBOR_COUNT 4                       // neighbors that one sweep reads

// Direction of a sweep over the grid: forward from cell 0, backward from the last cell.
enum { FORWARD_SWEEP = 0, BACKWARD_SWEEP = 1 };


// A step to a neighbor cell, and its chamfer cost.
typedef struct {
    int offset_x;
    int offset_y;
    int cost;
} NeighborStep;


// Shape of a found route: its length, its number of turns, and the half width of its narrowest
// passage.
typedef struct {
    float length_meters;
    int turn_count;
    float narrowest_half_width;
} RouteShape;

// Clearance of each cell, row by row from cell `(0, 0)`.
static float *cell_clearance;


// `get_cell_index` returns the index of the cell `(cell_x, cell_y)` in `cell_clearance`.
static int get_cell_index(int cell_x, int cell_y) {
    return cell_y * GC_GRID_SIZE + cell_x;
}


// `relax_distances` runs one sweep of the chamfer distance transform: each cell takes the cost
// through each of the four neighbors that the sweep has already visited, when that cost is lower.
static void relax_distances(int *distance_thirds, int sweep) {
    // The forward sweep reads the cell before in the row and three cells of the row before; the
    // backward sweep reads the cell after and three cells of the row after.
    bool is_backward = sweep == BACKWARD_SWEEP;
    int visited_step = is_backward ? 1 : -1;
    const NeighborStep neighbors[NEIGHBOR_COUNT] = {
        {
            .offset_x = visited_step,
            .offset_y = 0,
            .cost = STRAIGHT_STEP_COST,
        },
        {
            .offset_x = 0,
            .offset_y = visited_step,
            .cost = STRAIGHT_STEP_COST,
        },
        {
            .offset_x = visited_step,
            .offset_y = visited_step,
            .cost = DIAGONAL_STEP_COST,
        },
        {
            .offset_x = -visited_step,
            .offset_y = visited_step,
            .cost = DIAGONAL_STEP_COST,
        },
    };

    int cell_count = GC_GRID_SIZE * GC_GRID_SIZE;
    for (int visit_index = 0; visit_index < cell_count; visit_index++) {
        int cell_index = visit_index;
        if (is_backward)
            cell_index = cell_count - 1 - visit_index;
        int cell_x = cell_index % GC_GRID_SIZE;
        int cell_y = cell_index / GC_GRID_SIZE;
        for (int neighbor_index = 0; neighbor_index < NEIGHBOR_COUNT; neighbor_index++) {
            const NeighborStep *step = &neighbors[neighbor_index];
            int neighbor_x = cell_x + step->offset_x;
            int neighbor_y = cell_y + step->offset_y;
            if (gc_is_outside_grid(neighbor_x, neighbor_y))
                continue;
            int neighbor_cell_index = get_cell_index(neighbor_x, neighbor_y);
            int via = distance_thirds[neighbor_cell_index] + step->cost;        // cost through it
            if (via < distance_thirds[cell_index])
                distance_thirds[cell_index] = via;
        }
    }
}


// `compute_clearance` fills `cell_clearance` with the clearance of each cell for `terrain_class`,
// in meters, from a forward and a backward sweep of the chamfer distance transform.
static void compute_clearance(int terrain_class) {
    // Blocking cells start at zero, free cells at `FAR_COST`.
    int cell_count = GC_GRID_SIZE * GC_GRID_SIZE;
    int *distance_thirds = malloc(sizeof(int) * cell_count);
    if (!cell_clearance)
        cell_clearance = malloc(sizeof(float) * cell_count);
    for (int cell_index = 0; cell_index < cell_count; cell_index++) {
        int cell_x = cell_index % GC_GRID_SIZE;
        int cell_y = cell_index / GC_GRID_SIZE;
        bool is_blocking = gc_get_cell(cell_x, cell_y) > terrain_class;
        distance_thirds[cell_index] = is_blocking ? 0 : FAR_COST;
    }

    relax_distances(distance_thirds, FORWARD_SWEEP);
    relax_distances(distance_thirds, BACKWARD_SWEEP);
    for (int cell_index = 0; cell_index < cell_count; cell_index++) {
        float cells = distance_thirds[cell_index] / (float)STRAIGHT_STEP_COST;        // in cells
        cell_clearance[cell_index] = cells * GC_CELL_METERS;
    }
    free(distance_thirds);
}


// `get_clearance` returns the clearance of the cell under `point`, or zero outside the grid.
static float get_clearance(V2 point) {
    int cell_x = get_cell_coordinate(point.x);
    int cell_y = get_cell_coordinate(point.y);
    if (gc_is_outside_grid(cell_x, cell_y))
        return 0;
    int cell_index = get_cell_index(cell_x, cell_y);
    return cell_clearance[cell_index];
}


// `get_half_width` returns the half width of the passage at `point`: the largest clearance within
// `HALF_WIDTH_SEARCH_CELLS` cells, which is the radius of the largest free circle nearby.
static float get_half_width(V2 point) {
    float half_width = 0;
    int r = HALF_WIDTH_SEARCH_CELLS;        // search radius in cells
    int r_squared = r * r;
    for (int offset_y = -r; offset_y <= r; offset_y++) {
        for (int offset_x = -r; offset_x <= r; offset_x++) {
            if (offset_x * offset_x + offset_y * offset_y > r_squared)
                continue;
            V2 offset = v2(offset_x * GC_CELL_METERS, offset_y * GC_CELL_METERS);
            float clearance = get_clearance(add(point, offset));
            if (clearance > half_width)
                half_width = clearance;
        }
    }
    return half_width;
}


// `draw_map_point` draws a random point of the map.
static V2 draw_map_point(unsigned *random_state) {
    float x = next_random(random_state) / RANDOM_RANGE * MAP_SIZE_METERS;
    float y = next_random(random_state) / RANDOM_RANGE * MAP_SIZE_METERS;
    return v2(x, y);
}


// `is_inside_world` tells whether `point` lies within the bounds that units stay in.
static bool is_inside_world(V2 point) {
    bool is_above_min = point.x >= WORLD_MIN_METERS && point.y >= WORLD_MIN_METERS;
    bool is_below_max = point.x <= WORLD_MAX_METERS && point.y <= WORLD_MAX_METERS;
    return is_above_min && is_below_max;
}


// `draw_route_ends` draws a start and an end with `MIN_SPOT_CLEARANCE_METERS`, the end in a
// random direction 250 to 700 m from the start. It returns false when the draws ran out.
static bool draw_route_ends(unsigned *random_state, V2 *start, V2 *end) {
    // A start without enough clearance after `MAX_START_ATTEMPTS` draws stays as drawn.
    int failed_count = 0;        // failed draws of the start and the end together
    for (;;) {
        *start = draw_map_point(random_state);
        if (get_clearance(*start) >= MIN_SPOT_CLEARANCE_METERS)
            break;
        failed_count++;
        if (failed_count >= MAX_START_ATTEMPTS)
            break;
    }

    for (;;) {
        float angle = next_random(random_state) / RANDOM_RANGE * 2 * PI;
        float range_fraction = next_random(random_state) / RANDOM_RANGE;
        float d = MIN_ROUTE_METERS + range_fraction * ROUTE_RANGE_METERS;        // distance
        V2 offset = v2(cosf(angle) * d, sinf(angle) * d);
        *end = add(*start, offset);
        if (is_inside_world(*end) && get_clearance(*end) >= MIN_SPOT_CLEARANCE_METERS)
            break;
        failed_count++;
        if (failed_count >= MAX_SPOT_ATTEMPTS)
            break;
    }
    return failed_count < MAX_SPOT_ATTEMPTS;
}


// `run_test_search` runs one base game path search for a medium tank from `start` to `end`, with
// `FIND_SEARCH_BUDGET` expansions. It returns true when the search found the end.
static bool run_test_search(GcRequest *search, bool is_forward, V2 start, V2 end) {
    const UnitType *unit_type = &MEDIUM_TANK;
    float corner_gap = 2 * unit_type->radius;
    int terrain_class = unit_type->terrain_class;
    float tolerance = 0;
    gc_start_search(search, is_forward, start.x, start.y, end.x, end.y, terrain_class, corner_gap,
                    tolerance);
    return gc_update_search(search, FIND_SEARCH_BUDGET) == GC_SEARCH_FOUND;
}


// `search_test_route` searches a route from `start` to `end`, forward and then backward as the
// game does, and returns the number of its points in `path_points`, or zero.
static int search_test_route(V2 start, V2 end) {
    GcRequest search;
    gc_init_request(&search);
    bool is_found = run_test_search(&search, GC_SEARCH_FORWARD, start, end);
    if (!is_found)
        is_found = run_test_search(&search, GC_SEARCH_BACKWARD, start, end);
    int count = 0;        // number of path points
    if (is_found)
        count = gc_get_path(&search, path_points, MAX_WAYPOINTS);
    gc_free_request(&search);
    return count;
}


// `measure_route` measures the shape of the route from `start` through the `point_count` points of
// `path_points`. The passage width is measured every `MEASURE_STEP_METERS`, but not within
// `END_EXCLUSION_METERS` of the two ends.
static RouteShape measure_route(V2 start, V2 end, int point_count) {
    float turn_cosine = cosf(TURN_DEGREES * PI / HALF_TURN_DEGREES);
    RouteShape shape = {
        .length_meters = 0,
        .turn_count = 0,
        .narrowest_half_width = NO_PASSAGE_METERS,
    };
    V2 previous_point = start;
    V2 previous_direction = v2(0, 0);
    for (int point_index = 0; point_index < point_count; point_index++) {
        // A bend sharper than `TURN_DEGREES` between two segments counts as a turn.
        V2 point = get_pair_point(path_points, point_index);
        V2 segment = sub(point, previous_point);
        float length = len(segment);        // segment length
        if (length < MIN_SEGMENT_METERS)
            continue;
        V2 direction = mul(segment, 1 / length);
        bool has_previous_segment = length_squared(previous_direction) > 0;
        if (has_previous_segment && dot(direction, previous_direction) < turn_cosine)
            shape.turn_count++;

        for (float along = 0; along < length; along += MEASURE_STEP_METERS) {
            V2 measured = add(previous_point, mul(direction, along));
            float d_start = len(sub(measured, start));
            float d_end = len(sub(measured, end));
            bool is_away_from_start = d_start > END_EXCLUSION_METERS;
            bool is_away_from_end = d_end > END_EXCLUSION_METERS;
            if (!is_away_from_start || !is_away_from_end)
                continue;
            float half_width = get_half_width(measured);
            if (half_width < shape.narrowest_half_width)
                shape.narrowest_half_width = half_width;
        }
        shape.length_meters += length;
        previous_point = point;
        previous_direction = direction;
    }
    return shape;
}


// `find_spots` loads the map `name`, measures `try_count` random routes for a medium tank on it,
// and prints one line per found route. Tries without route ends or without a path print nothing.
void find_spots(const char *name, int try_count) {
    load_map(name);
    compute_clearance(MEDIUM_TANK.terrain_class);
    unsigned random_state = FIND_RANDOM_SEED;
    printf("map %s: sx sy dx dy straight ratio turns narrowest(m)\n", name);
    for (int try_index = 0; try_index < try_count; try_index++) {
        V2 start;
        V2 end;
        if (!draw_route_ends(&random_state, &start, &end))
            continue;
        int count = search_test_route(start, end);        // number of path points
        if (count < 1)
            continue;

        RouteShape shape = measure_route(start, end, count);
        float straight = len(sub(end, start));               // straight distance
        float ratio = shape.length_meters / straight;        // route length per straight distance
        printf("%s %.0f %.0f %.0f %.0f ", name, start.x, start.y, end.x, end.y);
        printf("%.0f %.2f ", straight, ratio);
        printf("%d %.1f\n", shape.turn_count, shape.narrowest_half_width);
    }
}
