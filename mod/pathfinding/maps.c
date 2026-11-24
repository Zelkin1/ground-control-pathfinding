// Maps: the loading of a .SLP grid into the path finder, the clearance field of each map, and the
// tests of circles and lines against blocking terrain. The map editor bakes every blocking element
// (cliffs, buildings, rocks, trees) into the .SLP grid, so the grid alone holds them.
//
// The clearance field of a terrain class holds, for each cell, the chessboard distance in cells to
// the nearest blocking cell, up to `MAX_CLEARANCE_CELLS`. The field is built once per map, for the
// terrain classes 1 and 2 of the unit types in scenarios.c, and kept for up to `MAX_CACHED_MAPS`
// maps. `get_clearance_meters` turns it into a distance that is known to be free, so that
// `is_circle_clear` and the rays of `measure_free_distance` can skip cells.
//
// A field has a border of one cell around the grid, which counts as blocking. A field index counts
// cells row by row, with `FIELD_ROW_CELLS` cells per row.
#include <stdlib.h>
#include <string.h>

#include "sim.h"

#define SLP_FILE_BYTES GC_GRID_BYTES              // size of a .SLP file, one grid
#define MAX_PATH_LENGTH 512                       // longest map file path
#define FIRST_CLEARANCE_CLASS 1                   // first class with a field
#define CLEARANCE_CLASS_COUNT 2                   // number of classes with one
#define MAX_CLEARANCE_CELLS 255                   // largest field value
#define CLEARANCE_MARGIN_METERS 0.01f             // margin of a free distance
#define FIELD_ROW_CELLS (GC_GRID_SIZE + 2)        // field cells per row
#define MAX_CACHED_MAPS 8                         // most maps with fields

const char *maps_dir = "build/sdf/Missions";
char map_name[MAP_NAME_SIZE];


// The clearance fields of one map, one per terrain class from `FIRST_CLEARANCE_CLASS` on.
typedef struct {
    char map_name[MAP_NAME_SIZE];
    uint8_t *fields[CLEARANCE_CLASS_COUNT];
} MapClearance;

static MapClearance map_clearances[MAX_CACHED_MAPS];
static int cached_map_count;
static MapClearance *current_clearance;


// `min_int` returns the smaller of two integers.
static int min_int(int first, int second) {
    return first < second ? first : second;
}


// `get_field_index` returns the field index of the cell `(cell_x, cell_y)`, past the border.
static int get_field_index(int cell_x, int cell_y) {
    return (cell_y + 1) * FIELD_ROW_CELLS + cell_x + 1;
}


// `get_lowest_neighbor` returns the lowest field value among the four neighbors of a field index
// at the index offsets `offsets`.
static int get_lowest_neighbor(const uint8_t *field, int field_index, const int *offsets) {
    int lowest = field[field_index + offsets[0]];
    for (int offset_index = 1; offset_index < 4; offset_index++)
        lowest = min_int(lowest, field[field_index + offsets[offset_index]]);
    return lowest;
}


// `build_class_clearance` builds the clearance field of a terrain class from the grid, with the
// two passes of a chessboard distance transform. Each pass reads the four neighbors that the pass
// has already visited.
static void build_class_clearance(uint8_t *field, int terrain_class) {
    // Blocking cells and the border start at zero, free cells at the largest value.
    memset(field, 0, FIELD_ROW_CELLS * FIELD_ROW_CELLS);
    for (int cell_y = 0; cell_y < GC_GRID_SIZE; cell_y++) {
        for (int cell_x = 0; cell_x < GC_GRID_SIZE; cell_x++) {
            bool is_blocking = gc_get_cell(cell_x, cell_y) > terrain_class;
            field[get_field_index(cell_x, cell_y)] = is_blocking ? 0 : MAX_CLEARANCE_CELLS;
        }
    }

    // The forward pass reads the cell before and the three cells of the row before; the backward
    // pass reads the cell after and the three cells of the row after.
    const int row_cells = FIELD_ROW_CELLS;
    const int forward_offsets[4] = {-1, -row_cells - 1, -row_cells, -row_cells + 1};
    const int backward_offsets[4] = {1, row_cells + 1, row_cells, row_cells - 1};
    for (int cell_y = 0; cell_y < GC_GRID_SIZE; cell_y++) {
        for (int cell_x = 0; cell_x < GC_GRID_SIZE; cell_x++) {
            int field_index = get_field_index(cell_x, cell_y);
            int through = get_lowest_neighbor(field, field_index, forward_offsets) + 1;
            if (through < field[field_index])
                field[field_index] = (uint8_t)through;
        }
    }
    for (int cell_y = GC_GRID_SIZE - 1; cell_y >= 0; cell_y--) {
        for (int cell_x = GC_GRID_SIZE - 1; cell_x >= 0; cell_x--) {
            int field_index = get_field_index(cell_x, cell_y);
            int through = get_lowest_neighbor(field, field_index, backward_offsets) + 1;
            if (through < field[field_index])
                field[field_index] = (uint8_t)through;
        }
    }
}


// `find_map_clearance` returns the clearance fields of the current map from the cache, or builds
// them. A full cache reuses its last entry for the new map.
static MapClearance *find_map_clearance(void) {
    for (int cache_index = 0; cache_index < cached_map_count; cache_index++) {
        MapClearance *cached = &map_clearances[cache_index];
        if (strcmp(cached->map_name, map_name) == 0)
            return cached;
    }

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


// `get_clearance_meters` returns a distance from any point of the cell `(cell_x, cell_y)` within
// which no blocking cell and no stamped cell lies, or zero when nothing is known. Terrain classes
// without a field get zero.
float get_clearance_meters(int cell_x, int cell_y, int terrain_class) {
    int class_offset = terrain_class - FIRST_CLEARANCE_CLASS;
    bool is_known_class = class_offset >= 0 && class_offset < CLEARANCE_CLASS_COUNT;
    if (!is_known_class || !current_clearance || gc_is_outside_grid(cell_x, cell_y))
        return 0;

    // A blocking cell `n` cells away lies at least `n - 1` whole cells away from any point of this
    // cell. The chessboard distance to the stamp box bounds the distance to the stamped cells.
    const uint8_t *field = current_clearance->fields[class_offset];
    int field_cells = field[get_field_index(cell_x, cell_y)];
    int stamp_cells = get_stamp_distance_cells(cell_x, cell_y);
    int n = min_int(field_cells, stamp_cells);        // free distance in cells
    if (n <= 1)
        return 0;
    return (n - 1) * GC_CELL_METERS - CLEARANCE_MARGIN_METERS;
}


// `select_map_clearance` makes `name` the current map, whose grid the path finder holds now, and
// selects its clearance fields, which it builds on the first use of that map.
void select_map_clearance(const char *name) {
    snprintf(map_name, sizeof map_name, "%s", name);
    current_clearance = find_map_clearance();
}


// `load_map` loads the grid of the map `name` from its .SLP file under `maps_dir`, from the
// campaign missions or the multiplayer missions, and selects its clearance fields. The program
// ends when the map cannot be loaded.
void load_map(const char *name) {
    static uint8_t slp_bytes[SLP_FILE_BYTES];
    const char *path_formats[] = {"%s/MISSIONS/%s/%s.SLP", "%s/MULTIPLAYER/MISSIONS/%s/%s.SLP"};
    int format_count = sizeof path_formats / sizeof path_formats[0];
    for (int format_index = 0; format_index < format_count; format_index++) {
        // A file of the wrong size ends the search.
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
        select_map_clearance(name);
        return;
    }

    fprintf(stderr, "cannot load map %s from %s (extract Missions.sdf with tools/sdf.py)\n", name,
            maps_dir);
    exit(EXIT_FAILURE);
}


// `is_circle_clear` tells whether the circle of `radius` around `point` holds no blocking cell of
// `terrain_class`. The clearance field answers most tests without reading the cells.
bool is_circle_clear(V2 point, float radius, int terrain_class) {
    int point_cell_x = get_cell_coordinate(point.x);
    int point_cell_y = get_cell_coordinate(point.y);
    if (radius < get_clearance_meters(point_cell_x, point_cell_y, terrain_class))
        return true;

    // A blocking cell blocks the circle when its closest point to the center lies inside it.
    int min_cell_x = get_cell_coordinate(point.x - radius);
    int max_cell_x = get_cell_coordinate(point.x + radius);
    int min_cell_y = get_cell_coordinate(point.y - radius);
    int max_cell_y = get_cell_coordinate(point.y + radius);
    float r_squared = radius * radius;
    for (int cell_y = min_cell_y; cell_y <= max_cell_y; cell_y++) {
        for (int cell_x = min_cell_x; cell_x <= max_cell_x; cell_x++) {
            if (gc_get_cell(cell_x, cell_y) <= terrain_class)
                continue;
            V2 closest = get_closest_cell_point(cell_x, cell_y, point);
            V2 to_closest = sub(closest, point);
            if (length_squared(to_closest) < r_squared)
                return false;
        }
    }
    return true;
}


// `is_line_clear` tells whether the zero-width ray of the game from `from` to `to` meets no
// blocking cell of `terrain_class`.
bool is_line_clear(V2 from, V2 to, int terrain_class) {
    return gc_cast_ray(from.x, from.y, to.x, to.y, terrain_class) == GC_RAY_CLEAR;
}


// `is_way_clear` tells whether the line from a unit to `point` meets no cell that blocks the unit.
bool is_way_clear(const Unit *unit, V2 point) {
    V2 from = unit->position;
    int ray = gc_cast_ray(from.x, from.y, point.x, point.y, unit->unit_type->terrain_class);
    return ray == GC_RAY_CLEAR;
}
