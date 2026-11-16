/* This file loads the maps of the game and tests their grids. `load_map` puts the grid of a
 * mission into the `PathFinder` of the game, the game object that holds the grid and runs the
 * path search. `is_circle_clear`, `is_line_clear` and `is_way_clear` test the grid for the other
 * parts. find.c runs `--find` on these maps.
 *
 * Clearance field: `load_map` also builds a clearance field for mover classes 1 and 2, those of
 * the tanks and the marines. For each cell it holds how many cells away the nearest cell lies
 * that blocks the class, counted along the axis that gives the larger count, so that the 8 cells
 * around a blocking cell have 1. Cells outside the grid count as blocking, as `gc_get_cell` and
 * the cell lookup of the game read them. The field lets `is_circle_clear` and the free-distance
 * rays of route.c skip most cell reads in open ground, with exactly the same results. The fields
 * of each map are built at its first load and kept for later loads.
 * - Buildings: The map editor writes the box of every solid prop and of every building of the
 *   mission into the grid as value 3, which blocks every class. On the maps of the scenarios,
 *   93 to 100% of the cells under such boxes hold 3. Props without collision, such as weeds,
 *   block nothing. So the field of the grid holds the buildings.
 * - Changes: `PathFinder::LoadGrid` (0x4e8200) is the only writer of the grid in the game, so the
 *   grid stays the same for a whole mission, also after a building is destroyed. The simulator
 *   changes it only while `stamp_parked` stamps parked units. `get_clearance_meters` counts those
 *   stamps through the box around them (`get_stamp_distance_cells`).
 *
 * Map file: The grid of a map is the file `<mission>/<mission>.SLP` in the Missions.sdf archive
 * of the game, which `tools/sdf.py` extracts. `load_map` reads the first `SLP_FILE_BYTES`
 * (256 KB) of the file, and `gc_load_slp` copies them into the grid as `PathFinder::LoadGrid`
 * (0x4e8200) does (re/PATHFINDING.md). The bytes have this layout:
 * - They hold `GC_GRID_SIZE` (1024) rows of 256 bytes each.
 * - Each byte holds 4 cells of 2 bits. Byte k of a row holds these cells, by their x in the row:
 *       bits:  7-6     5-4         3-2         1-0
 *       cell:  x = 4k  x = 4k + 1  x = 4k + 2  x = 4k + 3
 * - The last row of the file becomes grid row 0, which covers world y from 0 to `GC_CELL_METERS`.
 *   So the file starts with the row of the highest y.
 * - Each cell holds a value from 0 to 3. game.h describes which mover classes each value
 *   blocks. */
#include <stdlib.h>
#include <string.h>

#include "sim.h"

#define CELLS_PER_BYTE 4                 /* An .SLP file holds 2 bits per cell. */
#define SLP_FILE_BYTES (GC_GRID_SIZE * GC_GRID_SIZE / CELLS_PER_BYTE) /* This is 256 KB. */
#define MAX_PATH_LENGTH 512              /* The path of a map file is cut to this many bytes. */
#define FIRST_CLEARANCE_CLASS 1          /* The clearance field starts at mover class 1 (tanks). */
#define CLEARANCE_CLASS_COUNT 2          /* It covers classes 1 and 2, those of the unit kinds. */
#define MAX_CLEARANCE_CELLS 255          /* A clearance is one byte, so it stops at this many. */
#define CLEARANCE_MARGIN_METERS 0.01f    /* Clearances stay this far inside the true distance. */
#define FIELD_ROW_CELLS (GC_GRID_SIZE + 2) /* A field row adds a border cell at each end. */
#define MAX_CACHED_MAPS 8                /* The field cache keeps the fields of this many maps. */

const char *maps_dir = "build/sdf/Missions";
char map_name[MAP_NAME_SIZE];

/* `MapClearance` holds the clearance fields of the map `map_name`. `fields[k]` is the field of
 * mover class `FIRST_CLEARANCE_CLASS` + k, on the heap. It holds `FIELD_ROW_CELLS` rows of
 * `FIELD_ROW_CELLS` bytes: the grid with a border of one cell all around, which stands for the
 * cells outside the grid and so holds 0. Cell (x, y) of the grid is at `get_field_index`. */
typedef struct {
    char map_name[MAP_NAME_SIZE];
    uint8_t *fields[CLEARANCE_CLASS_COUNT];
} MapClearance;

/* `map_clearances` holds the fields of the first `cached_map_count` maps loaded, so that the
 * scenarios, which switch between a few maps, build the fields of each map once.
 * `current_clearance` points to the entry of the loaded map, or is NULL before the first load. */
static MapClearance map_clearances[MAX_CACHED_MAPS];
static int cached_map_count;
static MapClearance *current_clearance;

/* ------------------------------------------------------------------------------------------------
 * `min_int` returns the smaller of two ints.
 * --------------------------------------------------------------------------------------------- */
static int min_int(int first, int second) {
    return first < second ? first : second;
}

/* ------------------------------------------------------------------------------------------------
 * `get_field_index` returns the index of grid cell (`cell_x`, `cell_y`) in a clearance field.
 * --------------------------------------------------------------------------------------------- */
static int get_field_index(int cell_x, int cell_y) {
    return (cell_y + 1) * FIELD_ROW_CELLS + cell_x + 1;
}

/* ------------------------------------------------------------------------------------------------
 * `get_lowest_neighbor` returns the smallest clearance among the 4 cells of `field` at the index
 * offsets `offsets[0..3]` from `field_index`.
 * --------------------------------------------------------------------------------------------- */
static int get_lowest_neighbor(const uint8_t *field, int field_index, const int *offsets) {
    int lowest = field[field_index + offsets[0]];
    for (int offset_index = 1; offset_index < 4; offset_index++)
        lowest = min_int(lowest, field[field_index + offsets[offset_index]]);
    return lowest;
}

/* ------------------------------------------------------------------------------------------------
 * `build_class_clearance` builds the clearance field of one mover class with two passes over the
 * grid, the method of a chamfer distance transform. The border and the blocking cells start at 0,
 * and the free cells at `MAX_CLEARANCE_CELLS`. The forward pass gives each cell at most one more
 * than the smallest of the 4 neighbors that it has passed already (left, and the three below),
 * and the backward pass does the same the other way around. With a step of 1 to all 8 neighbors,
 * the two passes give the exact count of cells along the larger axis to the nearest blocking
 * cell, capped at `MAX_CLEARANCE_CELLS`.
 * --------------------------------------------------------------------------------------------- */
static void build_class_clearance(uint8_t *field, int mover_class) {
    /* Start the border and the blocking cells at 0, and the free cells at the cap. */
    memset(field, 0, FIELD_ROW_CELLS * FIELD_ROW_CELLS);
    for (int cell_y = 0; cell_y < GC_GRID_SIZE; cell_y++) {
        for (int cell_x = 0; cell_x < GC_GRID_SIZE; cell_x++) {
            bool is_blocking = gc_get_cell(cell_x, cell_y) > mover_class;
            field[get_field_index(cell_x, cell_y)] = is_blocking ? 0 : MAX_CLEARANCE_CELLS;
        }
    }

    /* Pass forward over the grid, then backward. The offsets reach the 4 neighbors that each pass
     * has already passed. */
    const int row_cells = FIELD_ROW_CELLS;
    const int forward_offsets[4] = {-1, -row_cells - 1, -row_cells, -row_cells + 1};
    const int backward_offsets[4] = {1, row_cells + 1, row_cells, row_cells - 1};
    for (int cell_y = 0; cell_y < GC_GRID_SIZE; cell_y++) {
        for (int cell_x = 0; cell_x < GC_GRID_SIZE; cell_x++) {
            int field_index = get_field_index(cell_x, cell_y);
            int through_neighbor = get_lowest_neighbor(field, field_index, forward_offsets) + 1;
            if (through_neighbor < field[field_index])
                field[field_index] = (uint8_t)through_neighbor;
        }
    }
    for (int cell_y = GC_GRID_SIZE - 1; cell_y >= 0; cell_y--) {
        for (int cell_x = GC_GRID_SIZE - 1; cell_x >= 0; cell_x--) {
            int field_index = get_field_index(cell_x, cell_y);
            int through_neighbor = get_lowest_neighbor(field, field_index, backward_offsets) + 1;
            if (through_neighbor < field[field_index])
                field[field_index] = (uint8_t)through_neighbor;
        }
    }
}

/* ------------------------------------------------------------------------------------------------
 * `find_map_clearance` returns the cache entry of the loaded map. It builds the fields only for
 * a map that is not in the cache yet, because the grid of a map is the same at every load. It
 * must run right after the grid is loaded, before `stamp_parked` changes it.
 * --------------------------------------------------------------------------------------------- */
static MapClearance *find_map_clearance(void) {
    /* Take the entry of the map from the cache when the map was loaded before. */
    for (int cache_index = 0; cache_index < cached_map_count; cache_index++) {
        MapClearance *cached = &map_clearances[cache_index];
        if (strcmp(cached->map_name, map_name) == 0)
            return cached;
    }

    /* Build the fields into a new entry, or into the last entry once the cache is full. */
    if (cached_map_count < MAX_CACHED_MAPS) {
        MapClearance *added = &map_clearances[cached_map_count];
        for (int class_offset = 0; class_offset < CLEARANCE_CLASS_COUNT; class_offset++)
            added->fields[class_offset] = malloc(FIELD_ROW_CELLS * FIELD_ROW_CELLS);
        cached_map_count++;
    }
    MapClearance *entry = &map_clearances[cached_map_count - 1];
    snprintf(entry->map_name, sizeof entry->map_name, "%s", map_name);
    for (int class_offset = 0; class_offset < CLEARANCE_CLASS_COUNT; class_offset++)
        build_class_clearance(entry->fields[class_offset], FIRST_CLEARANCE_CLASS + class_offset);
    return entry;
}

/* ------------------------------------------------------------------------------------------------
 * `get_clearance_meters` takes the smaller of two counts of cells: the clearance field of the
 * class, and the count to the box around the stamped cells. Between a cell and a blocking cell k
 * cells away along the larger axis lie k - 1 whole cells, so no blocking cell comes closer than
 * (k - 1) cells to any point of the cell. The margin covers the float error of positions and the
 * shift of the cell edges of the cell lookup of the game from multiples of `GC_CELL_METERS`, which
 * both stay under 1 mm.
 * --------------------------------------------------------------------------------------------- */
float get_clearance_meters(int cell_x, int cell_y, int mover_class) {
    int class_offset = mover_class - FIRST_CLEARANCE_CLASS;
    bool is_known_class = class_offset >= 0 && class_offset < CLEARANCE_CLASS_COUNT;
    if (!is_known_class || !current_clearance || gc_is_outside_grid(cell_x, cell_y))
        return 0;
    const uint8_t *field = current_clearance->fields[class_offset];
    int field_cells = field[get_field_index(cell_x, cell_y)];
    int stamp_cells = get_stamp_distance_cells(cell_x, cell_y);
    int clearance_cells = min_int(field_cells, stamp_cells);
    if (clearance_cells <= 1)
        return 0;
    return (clearance_cells - 1) * GC_CELL_METERS - CLEARANCE_MARGIN_METERS;
}

/* ------------------------------------------------------------------------------------------------
 * `load_map` reads the file into one static buffer of `SLP_FILE_BYTES`, so that the large buffer
 * stays off the stack, and `gc_load_slp` copies the bytes into the grid of the game.
 * --------------------------------------------------------------------------------------------- */
void load_map(const char *name) {
    /* Look for the map among the campaign missions and the multiplayer maps. Each path format
     * takes `maps_dir`, then the name of the map twice. A file shorter than `SLP_FILE_BYTES` ends
     * the search. */
    static uint8_t slp_bytes[SLP_FILE_BYTES];
    const char *path_formats[] = {"%s/MISSIONS/%s/%s.SLP", "%s/MULTIPLAYER/MISSIONS/%s/%s.SLP"};
    int format_count = sizeof path_formats / sizeof path_formats[0];
    for (int format_index = 0; format_index < format_count; format_index++) {
        char path[MAX_PATH_LENGTH];
        snprintf(path, sizeof path, path_formats[format_index], maps_dir, name, name);
        FILE *file = fopen(path, "rb");
        if (!file)
            continue;
        size_t byte_count = fread(slp_bytes, 1, sizeof slp_bytes, file);
        fclose(file);
        if (byte_count != sizeof slp_bytes)
            break;
        gc_load_slp(slp_bytes);
        snprintf(map_name, sizeof map_name, "%s", name);
        current_clearance = find_map_clearance();
        return;
    }

    /* Stop the program when no map was loaded. */
    fprintf(stderr, "cannot load map %s from %s (extract Missions.sdf with tools/sdf.py)\n", name,
            maps_dir);
    exit(EXIT_FAILURE);
}

/* ------------------------------------------------------------------------------------------------
 * `is_circle_clear` first takes the clearance of the cell that holds `point`: a circle that stays
 * inside it overlaps no blocking cell. Otherwise it tests each cell that the bounding box of the
 * circle touches. A blocking cell overlaps the circle when the point of the cell closest to
 * `point` is closer than `radius`.
 *
 * The sketch shows the test for one cell. # marks a cell that blocks `mover_class`, P is
 * `point`, and * is the point of the blocking cell closest to P.
 *
 *     +-----+-----+
 *     |     |#####|
 *     |     |#####|
 *     +-----*-----+
 *     |  P  |     |
 *     +-----+-----+
 * --------------------------------------------------------------------------------------------- */
bool is_circle_clear(V2 point, float radius, int mover_class) {
    /* Answer at once when the circle stays inside the clearance of the cell of `point`. */
    int point_cell_x = get_cell_coordinate(point.x);
    int point_cell_y = get_cell_coordinate(point.y);
    if (radius < get_clearance_meters(point_cell_x, point_cell_y, mover_class))
        return true;

    /* Test the cells, and stop at the first blocking cell that overlaps the circle. */
    int min_cell_x = get_cell_coordinate(point.x - radius);
    int max_cell_x = get_cell_coordinate(point.x + radius);
    int min_cell_y = get_cell_coordinate(point.y - radius);
    int max_cell_y = get_cell_coordinate(point.y + radius);
    float radius_squared = radius * radius;
    for (int cell_y = min_cell_y; cell_y <= max_cell_y; cell_y++) {
        for (int cell_x = min_cell_x; cell_x <= max_cell_x; cell_x++) {
            if (gc_get_cell(cell_x, cell_y) <= mover_class)
                continue;
            V2 closest = get_closest_cell_point(cell_x, cell_y, point);
            V2 to_closest = sub(closest, point);
            if (length_squared(to_closest) < radius_squared)
                return false;
        }
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `is_line_clear` casts a ray with the raycast of the game (`gc_cast_ray`). The line is clear when
 * the ray result is `GC_RAY_CLEAR`.
 * --------------------------------------------------------------------------------------------- */
bool is_line_clear(V2 from, V2 to, int mover_class) {
    return gc_cast_ray(from.x, from.y, to.x, to.y, mover_class) == GC_RAY_CLEAR;
}

/* ------------------------------------------------------------------------------------------------
 * `is_way_clear` casts the same ray as `is_line_clear`, from the position of the unit and with the
 * mover class of its kind.
 * --------------------------------------------------------------------------------------------- */
bool is_way_clear(const Unit *unit, V2 point) {
    V2 from = unit->position;
    int ray_result = gc_cast_ray(from.x, from.y, point.x, point.y, unit->kind->mover_class);
    return ray_result == GC_RAY_CLEAR;
}
