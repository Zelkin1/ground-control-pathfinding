/* find.c runs `--find`. `find_spots` measures random routes of the path search of the game on a
 * map, so that test scenarios can use the narrow passages and turns of real terrain. sim.c calls
 * it instead of the scenarios when the command line has `--find`, and it loads the map with
 * `load_map` from maps.c.
 *
 * Terms: The clearance of a cell is the distance in world units from its center to the center of
 * the closest cell that blocks a given mover class, as `compute_clearance` estimates it. The
 * half-width of a passage at a point is the largest clearance near that point, as
 * `get_half_width` measures it. */
#include <stdlib.h>

#include "sim.h"

/* These constants are for `--find`. */
#define MAP_SIZE_METERS 6400.0f          /* The side of the map is `GC_GRID_SIZE` cells wide. */
#define WORLD_MIN_METERS 10.0f           /* Both coordinates of a route end are at least this. */
#define WORLD_MAX_METERS 6390.0f         /* Its coordinates are also at most this. */
#define FIND_RANDOM_SEED 12345u          /* `--find` always draws its routes from this seed. */
#define MIN_SPOT_CLEARANCE_METERS 25.0f  /* Route ends must be this far from blocked cells. */
#define MAX_START_ATTEMPTS 1000          /* After this many failed draws, the last one is used. */
#define MAX_SPOT_ATTEMPTS 2000           /* After this many for both ends, a route is skipped. */
#define MIN_ROUTE_METERS 250.0f          /* Route ends are at least this far apart. */
#define ROUTE_RANGE_METERS 450.0f        /* A random distance of up to this is added to that. */
#define FIND_SEARCH_BUDGET 500           /* Game searches end after this many expansions. */
#define TURN_DEGREES 30.0f               /* A bend of more than this counts as a turn. */
#define HALF_TURN_DEGREES 180            /* Half a turn, PI radians, is this many degrees. */
#define SAMPLE_STEP_METERS 2.0f          /* The passage is sampled this often along a route. */
#define END_EXCLUSION_METERS 30.0f       /* It is not sampled this close to a route end. */
#define HALF_WIDTH_SEARCH_CELLS 4        /* The half-width search reaches this far (25 m). */
#define MIN_SEGMENT_METERS 1e-3f         /* Shorter route segments are skipped. */
#define NO_PASSAGE_METERS 1e9f           /* A route without samples has this narrowest passage. */

/* These constants are for the distance transform in `compute_clearance`. */
#define STRAIGHT_STEP_COST 3             /* A step to a cell in the same row or column costs 3. */
#define DIAGONAL_STEP_COST 4             /* A diagonal step costs 4, close to 3 times sqrt(2). */
#define FAR_COST (1 << 28)               /* Free cells start at this cost, above any real one. */
#define NEIGHBOR_COUNT 4                 /* A pass looks at this many neighbors of each cell. */

/* These are the values of `pass` in `relax_distances`. */
enum { FORWARD_PASS = 0, BACKWARD_PASS = 1 };

/* A `NeighborStep` holds the step from a cell to one of its neighbors, in cells, and the cost of
 * the step. */
typedef struct {
    int offset_x;
    int offset_y;
    int cost;
} NeighborStep;

/* `cell_clearance` holds the clearance of each cell for the mover class of the last
 * `compute_clearance` call, in the order of `get_cell_index`. The array is on the heap, because a
 * static array this large would push the reserved block of game.c past 0x400000, the fixed base
 * of the image of gc.exe. */
static float *cell_clearance;

/* ------------------------------------------------------------------------------------------------
 * `get_cell_index` returns the index of a cell in `cell_clearance` and in the `distance_thirds`
 * array of `compute_clearance`. Both hold the grid row by row, from row 0 up, and each row from
 * x = 0 up.
 * --------------------------------------------------------------------------------------------- */
static int get_cell_index(int cell_x, int cell_y) {
    return cell_y * GC_GRID_SIZE + cell_x;
}

/* ------------------------------------------------------------------------------------------------
 * `relax_distances` makes one pass of the distance transform over `distance_thirds`.
 * Inputs: With `FORWARD_PASS`, `pass` visits the cells from the first index to the last, and
 *   with `BACKWARD_PASS`, from the last to the first.
 * Result: Each cell takes the smallest of its own distance and, for each neighbor that the pass
 *   visited before it, the distance of the neighbor plus the cost of the step to it.
 *
 * The sketch shows a cell C and the neighbors that each pass reads, by their index in
 * `neighbors`. Rows with a higher y are drawn higher.
 *
 *     forward pass:   . . .     backward pass:   3 1 2
 *                     0 C .                      . C 0
 *                     2 1 3                      . . .
 * --------------------------------------------------------------------------------------------- */
static void relax_distances(int *distance_thirds, int pass) {
    /* Find the four neighbors that the pass visits before a cell: the one next to it in the same
     * row, and three in the row that the pass visits before the row of the cell. */
    bool is_backward = pass == BACKWARD_PASS;
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

    /* Visit the cells in the order of the pass. */
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
            int neighbor_distance = distance_thirds[neighbor_cell_index];
            int via_neighbor = neighbor_distance + step->cost;
            if (via_neighbor < distance_thirds[cell_index])
                distance_thirds[cell_index] = via_neighbor;
        }
    }
}

/* ------------------------------------------------------------------------------------------------
 * `compute_clearance` fills `cell_clearance` for `mover_class`. It uses a chamfer distance
 * transform, a quick way to find the distance of each cell to the closest blocked cell. The
 * transform measures the shortest chain of straight and diagonal steps from cell to cell, with
 * whole-number costs per step, in two passes over the grid.
 * --------------------------------------------------------------------------------------------- */
static void compute_clearance(int mover_class) {
    /* Start the blocked cells at 0 and the free cells at `FAR_COST`. `distance_thirds` counts in
     * thirds of a cell, so that a straight step costs `STRAIGHT_STEP_COST`. */
    int cell_count = GC_GRID_SIZE * GC_GRID_SIZE;
    int *distance_thirds = malloc(sizeof(int) * cell_count);
    if (!cell_clearance)
        cell_clearance = malloc(sizeof(float) * cell_count);
    for (int cell_index = 0; cell_index < cell_count; cell_index++) {
        int cell_x = cell_index % GC_GRID_SIZE;
        int cell_y = cell_index / GC_GRID_SIZE;
        bool is_blocked = gc_get_cell(cell_x, cell_y) > mover_class;
        distance_thirds[cell_index] = is_blocked ? 0 : FAR_COST;
    }

    /* Spread the distances with a forward and a backward pass, and convert them to world units. */
    relax_distances(distance_thirds, FORWARD_PASS);
    relax_distances(distance_thirds, BACKWARD_PASS);
    for (int cell_index = 0; cell_index < cell_count; cell_index++) {
        float distance_cells = distance_thirds[cell_index] / (float)STRAIGHT_STEP_COST;
        cell_clearance[cell_index] = distance_cells * GC_CELL_METERS;
    }
    free(distance_thirds);
}

/* ------------------------------------------------------------------------------------------------
 * `get_clearance` returns the clearance of the cell that holds `point`, or 0 outside the grid.
 * --------------------------------------------------------------------------------------------- */
static float get_clearance(V2 point) {
    int cell_x = get_cell_coordinate(point.x);
    int cell_y = get_cell_coordinate(point.y);
    if (gc_is_outside_grid(cell_x, cell_y))
        return 0;
    int cell_index = get_cell_index(cell_x, cell_y);
    return cell_clearance[cell_index];
}

/* ------------------------------------------------------------------------------------------------
 * `get_half_width` returns half the width of the passage at `point`. This is the largest
 * clearance at the points that lie a whole number of cells along each axis from `point`, within a
 * circle of `HALF_WIDTH_SEARCH_CELLS` cells. The clearance at `point` itself says little, because
 * the routes of the game run right along blocked cells.
 *
 * The sketch shows the points that it reads, marked x, for a circle of 2 cells around P, which is
 * `point`. P itself is read too.
 *
 *     . . x . .
 *     . x x x .
 *     x x P x x
 *     . x x x .
 *     . . x . .
 * --------------------------------------------------------------------------------------------- */
static float get_half_width(V2 point) {
    float half_width = 0;
    int radius_cells = HALF_WIDTH_SEARCH_CELLS;
    int radius_squared = radius_cells * radius_cells;
    for (int offset_y = -radius_cells; offset_y <= radius_cells; offset_y++) {
        for (int offset_x = -radius_cells; offset_x <= radius_cells; offset_x++) {
            if (offset_x * offset_x + offset_y * offset_y > radius_squared)
                continue;
            V2 offset_meters = v2(offset_x * GC_CELL_METERS, offset_y * GC_CELL_METERS);
            float clearance = get_clearance(add(point, offset_meters));
            if (clearance > half_width)
                half_width = clearance;
        }
    }
    return half_width;
}

/* ------------------------------------------------------------------------------------------------
 * `draw_map_point` returns a random point on the map.
 * --------------------------------------------------------------------------------------------- */
static V2 draw_map_point(unsigned *random_state) {
    float x = next_random(random_state) / RANDOM_RANGE * MAP_SIZE_METERS;
    float y = next_random(random_state) / RANDOM_RANGE * MAP_SIZE_METERS;
    return v2(x, y);
}

/* ------------------------------------------------------------------------------------------------
 * `is_inside_world` returns true when both coordinates of `point` are from `WORLD_MIN_METERS` to
 * `WORLD_MAX_METERS`, so that the point is at least 10 m inside each edge of the map.
 * --------------------------------------------------------------------------------------------- */
static bool is_inside_world(V2 point) {
    bool is_above_min = point.x >= WORLD_MIN_METERS && point.y >= WORLD_MIN_METERS;
    bool is_below_max = point.x <= WORLD_MAX_METERS && point.y <= WORLD_MAX_METERS;
    return is_above_min && is_below_max;
}

/* ------------------------------------------------------------------------------------------------
 * `draw_route_ends` draws a random route start and end on open ground.
 * Results: `start` is the first random point with `MIN_SPOT_CLEARANCE_METERS` of clearance. `end`
 *   is the first random point that has that clearance, passes `is_inside_world`, and lies
 *   `MIN_ROUTE_METERS` plus up to `ROUTE_RANGE_METERS` from `start`. `draw_route_ends` returns
 *   true when it finds `end` before the failed draws of both ends together reach
 *   `MAX_SPOT_ATTEMPTS`, and false otherwise.
 * Edge case: After `MAX_START_ATTEMPTS` failed draws, `start` keeps the last point drawn, even
 *   without clearance.
 * --------------------------------------------------------------------------------------------- */
static bool draw_route_ends(unsigned *random_state, V2 *start, V2 *end) {
    /* Draw the start. */
    int failed_draw_count = 0;
    for (;;) {
        *start = draw_map_point(random_state);
        if (get_clearance(*start) >= MIN_SPOT_CLEARANCE_METERS)
            break;
        failed_draw_count++;
        if (failed_draw_count >= MAX_START_ATTEMPTS)
            break;
    }

    /* Draw the end at a random angle and distance from the start. */
    for (;;) {
        float angle = next_random(random_state) / RANDOM_RANGE * 2 * PI;
        float range_fraction = next_random(random_state) / RANDOM_RANGE;
        float distance = MIN_ROUTE_METERS + range_fraction * ROUTE_RANGE_METERS;
        V2 offset = v2(cosf(angle) * distance, sinf(angle) * distance);
        *end = add(*start, offset);
        if (is_inside_world(*end) && get_clearance(*end) >= MIN_SPOT_CLEARANCE_METERS)
            break;
        failed_draw_count++;
        if (failed_draw_count >= MAX_SPOT_ATTEMPTS)
            break;
    }
    return failed_draw_count < MAX_SPOT_ATTEMPTS;
}

/* ------------------------------------------------------------------------------------------------
 * `get_find_kind` returns the unit type whose routes `--find` measures: marines with
 * `--findinfantry on`, and medium tanks otherwise.
 * --------------------------------------------------------------------------------------------- */
static const Kind *get_find_kind(void) {
    return is_find_infantry_on ? &MARINE : &MEDIUM_TANK;
}

/* ------------------------------------------------------------------------------------------------
 * `run_test_search` starts the search of the game for a unit of `get_find_kind` from `start` to
 * `end`, and runs it for up to `FIND_SEARCH_BUDGET` expansions.
 * Inputs: `is_quiet` is `GC_SEARCH_QUIET` or `GC_SEARCH_ACTIVE`.
 * Result: It returns true when the search found a route, and false otherwise.
 * --------------------------------------------------------------------------------------------- */
static bool run_test_search(GcRequest *search, bool is_quiet, V2 start, V2 end) {
    /* Search like the squad searches of the game: with a clearance of twice the radius of the
     * unit, and with a goal tolerance of 0, so that the route must reach `end` itself. */
    const Kind *kind = get_find_kind();
    float clearance = 2 * kind->radius;
    int mover_class = kind->mover_class;
    float tolerance = 0;
    gc_start_search(search, is_quiet, start.x, start.y, end.x, end.y, mover_class, clearance,
                    tolerance);
    return gc_update_search(search, FIND_SEARCH_BUDGET) == GC_SEARCH_FOUND;
}

/* ------------------------------------------------------------------------------------------------
 * `search_test_route` puts the route of the game for a unit of `get_find_kind` from `start` to
 * `end` into `path_points`. Like the synchronous searches in the game, it runs the quiet search,
 * and the active one when the quiet one fails.
 * Result: It returns the number of route points, or 0 when neither search found a route.
 * --------------------------------------------------------------------------------------------- */
static int search_test_route(V2 start, V2 end) {
    GcRequest search;
    gc_init_request(&search);
    bool is_found = run_test_search(&search, GC_SEARCH_QUIET, start, end);
    if (!is_found)
        is_found = run_test_search(&search, GC_SEARCH_ACTIVE, start, end);
    int point_count = 0;
    if (is_found)
        point_count = gc_get_path(&search, path_points, MAX_WAYPOINTS);
    gc_free_request(&search);
    return point_count;
}

/* A `RouteShape` holds what `measure_route` finds out about a route. */
typedef struct {
    float length_meters;
    int turn_count;
    float narrowest_half_width; /* The narrowest passage is twice this wide, in meters. */
} RouteShape;

/* ------------------------------------------------------------------------------------------------
 * `measure_route` measures the route from `start` through the `point_count` points in
 * `path_points`, which holds them as x, y pairs.
 * Result: It returns the length of the route, its number of turns, and the half-width of its
 *   narrowest passage. The passage is sampled every `SAMPLE_STEP_METERS` along each segment, but
 *   not within `END_EXCLUSION_METERS` of `start` or `end`.
 * Edge cases: A segment shorter than `MIN_SEGMENT_METERS` is skipped, and the next segment starts
 *   where the skipped one started. `narrowest_half_width` stays `NO_PASSAGE_METERS` when no
 *   sample counts.
 *
 * In the sketch, p0 and p1 are route points, and p1 is the end of the route. x marks a sample that
 * counts, and + a sample that is too close to `start` or `end`. The bend at p0 is a turn when it
 * is sharper than `TURN_DEGREES`.
 *
 *                           + p1
 *                          +
 *                         x
 *                        x
 *       o--+--+--x--x--x p0
 *     start
 * --------------------------------------------------------------------------------------------- */
static RouteShape measure_route(V2 start, V2 end, int point_count) {
    /* Start with an empty shape at `start`. */
    float turn_cosine = cosf(TURN_DEGREES * PI / HALF_TURN_DEGREES);
    RouteShape shape = {
        .length_meters = 0,
        .turn_count = 0,
        .narrowest_half_width = NO_PASSAGE_METERS,
    };
    V2 previous_point = start;
    V2 previous_direction = v2(0, 0);
    for (int point_index = 0; point_index < point_count; point_index++) {
        /* Count a turn where the direction of the segment differs from the direction of the
         * previous segment by more than `TURN_DEGREES`. */
        V2 point = get_pair_point(path_points, point_index);
        V2 segment = sub(point, previous_point);
        float segment_length = len(segment);
        if (segment_length < MIN_SEGMENT_METERS)
            continue;
        V2 direction = mul(segment, 1 / segment_length);
        bool has_previous_segment = length_squared(previous_direction) > 0;
        if (has_previous_segment && dot(direction, previous_direction) < turn_cosine)
            shape.turn_count++;

        /* Sample the passage along the segment. */
        for (float along = 0; along < segment_length; along += SAMPLE_STEP_METERS) {
            V2 sample = add(previous_point, mul(direction, along));
            float start_distance = len(sub(sample, start));
            float end_distance = len(sub(sample, end));
            bool is_away_from_start = start_distance > END_EXCLUSION_METERS;
            bool is_away_from_end = end_distance > END_EXCLUSION_METERS;
            if (!is_away_from_start || !is_away_from_end)
                continue;
            float half_width = get_half_width(sample);
            if (half_width < shape.narrowest_half_width)
                shape.narrowest_half_width = half_width;
        }
        shape.length_meters += segment_length;
        previous_point = point;
        previous_direction = direction;
    }
    return shape;
}

/* ------------------------------------------------------------------------------------------------
 * `find_spots` measures each route with `measure_route`. The detour ratio of a route is its
 * length divided by the straight distance between its ends. A ratio near 1 means an open way,
 * and a high ratio means that the route winds around obstacles. The program prints it in the
 * column "detour".
 * --------------------------------------------------------------------------------------------- */
void find_spots(const char *name, int sample_count) {
    /* Load the map, and find the clearance of every cell for the units whose routes it
     * measures. */
    load_map(name);
    compute_clearance(get_find_kind()->mover_class);
    unsigned random_state = FIND_RANDOM_SEED;
    printf("map %s: sx sy dx dy straight detour turns narrowest(m)\n", name);
    for (int sample_index = 0; sample_index < sample_count; sample_index++) {
        /* Draw a random route on open ground, and search the route of the game for it. */
        V2 start;
        V2 end;
        if (!draw_route_ends(&random_state, &start, &end))
            continue;
        int point_count = search_test_route(start, end);
        if (point_count < 1)
            continue;

        /* Print one line per route. */
        RouteShape shape = measure_route(start, end, point_count);
        float straight_distance = len(sub(end, start));
        float detour_ratio = shape.length_meters / straight_distance;
        printf("%s %.0f %.0f %.0f %.0f ", name, start.x, start.y, end.x, end.y);
        printf("%.0f %.2f ", straight_distance, detour_ratio);
        printf("%d %.1f\n", shape.turn_count, shape.narrowest_half_width);
    }
}
