// Calls into the code of the real gc.exe (docs/PATHFINDING.md): the base game path search, the
// raycast, the formation layout and the grid of the path finder, at their fixed addresses. The
// simulator maps gc.exe at its base address first (game_image.c); the game mod runs inside gc.exe
// and calls the same addresses. Nodes of game lists go back through `operator delete` of the game,
// which the simulator redirects to `free`.
//
// The passability grid of the path finder holds 1024 x 1024 cells of 2 bits each, 4 cells per
// byte with the first cell in the highest bits, and 256 bytes per row. A worker thread can read
// its own copy of the grid (`gc_use_grid`). The path finder functions use the MSVC `thiscall`
// convention.
#include "game.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

#define THISCALL __attribute__((thiscall))

#define GC_PATHFINDER ((void *)0x6eb470)                        // global PathFinder object
#define PATHFINDER_LAST_RAY_SIDE 0x00                           // side of the last raycast step
#define PATHFINDER_GRID 0x0c                                    // passability grid
#define GC_INVERSE_CELL_SIZE ((const float *)0x82f4ec)          // 1 / 6.25 once initialized

#define CELLS_PER_BYTE 4                                        // cells in one grid byte
#define BITS_PER_CELL 2                                         // bits of one cell
#define GRID_ROW_BYTES (GC_GRID_SIZE / CELLS_PER_BYTE)          // bytes of one grid row

#define REQUEST_IS_FOUND 0x38                                   // PathRequest: target found
#define REQUEST_IS_SEARCHING 0x3a                               // PathRequest: still searching


// A node of a list of the game: a point and the links to its neighbors.
typedef struct GcNode {
    float x;
    float y;
    struct GcNode *next;
    struct GcNode *previous;
} GcNode;


// A list of the game: its head, its iterator, its tail, and its number of nodes.
typedef struct {
    GcNode *head;
    GcNode *iterator;
    GcNode *tail;
    int count;
} GcList;

static const GcList EMPTY_LIST = {
    .head = NULL,
    .iterator = NULL,
    .tail = NULL,
    .count = 0,
};

// Functions of gc.exe, by their addresses.
typedef void(THISCALL *RequestFunction)(GcRequest *);
typedef void(THISCALL *StartFunction)(GcRequest *, float, float, float, float, int, float, float);
typedef int(THISCALL *UpdateFunction)(void *, GcRequest *, int);
typedef void(THISCALL *GetPathFunction)(GcRequest *, GcList *);
typedef int(THISCALL *RaycastFunction)(void *, float, float, float, float, int);
typedef char (*FormationFunction)(float, float, float, float, GcList *, int, float, int, int);

#define REQUEST_CONSTRUCTOR ((RequestFunction)0x4eb800)
#define REQUEST_DESTRUCTOR ((RequestFunction)0x4eb820)
#define START_SEARCH_QUIET ((StartFunction)0x4eb8d0)
#define START_SEARCH ((StartFunction)0x4eb9a0)
#define PATHFINDER_UPDATE ((UpdateFunction)0x4e8760)
#define GET_PATH ((GetPathFunction)0x4ebbb0)
#define RAYCAST ((RaycastFunction)0x4e8400)
#define CALCULATE_FORMATION_POINTS ((FormationFunction)0x4e7be0)
#define OPERATOR_DELETE ((void (*)(void *))GC_OPERATOR_DELETE)

// Grid copy of this thread, or NULL for the grid of the path finder.
static _Thread_local uint8_t *thread_grid;


// `get_pathfinder_field` returns the address of the field at `offset` in the path finder object.
static uint8_t *get_pathfinder_field(uint32_t offset) {
    uint8_t *pathfinder = GC_PATHFINDER;
    return &pathfinder[offset];
}


// `get_game_grid` returns the passability grid of the path finder.
static uint8_t *get_game_grid(void) {
    return get_pathfinder_field(PATHFINDER_GRID);
}


// `get_grid` returns the grid that this thread reads: its own copy, or the grid of the path
// finder.
static uint8_t *get_grid(void) {
    if (thread_grid)
        return thread_grid;
    return get_game_grid();
}


// `gc_is_outside_grid` tells whether the cell `(cell_x, cell_y)` lies outside the grid.
bool gc_is_outside_grid(int cell_x, int cell_y) {
    bool is_negative = cell_x < 0 || cell_y < 0;
    bool is_too_large = cell_x >= GC_GRID_SIZE || cell_y >= GC_GRID_SIZE;
    return is_negative || is_too_large;
}


// `get_cell_shift` returns the bit shift of the cell in column `cell_x` within its byte.
static int get_cell_shift(int cell_x) {
    int index_in_byte = cell_x % CELLS_PER_BYTE;
    return (CELLS_PER_BYTE - 1 - index_in_byte) * BITS_PER_CELL;
}


// `get_cell_byte` returns the byte of the grid of this thread that holds the cell
// `(cell_x, cell_y)`.
static uint8_t *get_cell_byte(int cell_x, int cell_y) {
    uint8_t *grid = get_grid();
    int byte_index = cell_y * GRID_ROW_BYTES + cell_x / CELLS_PER_BYTE;
    return &grid[byte_index];
}


// `gc_set_cell` sets the value of the cell `(cell_x, cell_y)` in the grid of this thread. A cell
// outside the grid stays unchanged.
void gc_set_cell(int cell_x, int cell_y, int value) {
    if (gc_is_outside_grid(cell_x, cell_y))
        return;
    uint8_t *cell_byte = get_cell_byte(cell_x, cell_y);
    int shift = get_cell_shift(cell_x);
    int mask = GC_CELL_VALUE_MASK << shift;
    int others = *cell_byte & ~mask;        // bits of the other cells
    int bits = (value & GC_CELL_VALUE_MASK) << shift;
    *cell_byte = (uint8_t)(others | bits);
}


// `gc_get_cell` returns the value of the cell `(cell_x, cell_y)` in the grid of this thread, or
// `GC_CELL_CLIFF` outside the grid.
int gc_get_cell(int cell_x, int cell_y) {
    if (gc_is_outside_grid(cell_x, cell_y))
        return GC_CELL_CLIFF;
    uint8_t cell_byte = *get_cell_byte(cell_x, cell_y);
    int shift = get_cell_shift(cell_x);
    return (cell_byte >> shift) & GC_CELL_VALUE_MASK;
}


// `gc_clear_grid` sets every cell of the grid of the path finder to free terrain.
void gc_clear_grid(void) {
    memset(get_game_grid(), 0, GC_GRID_BYTES);
}


// `gc_load_slp` copies the grid of a .SLP map file into the path finder, in reverse row order as
// `LoadGrid` (0x4e8200) does: the last row of the file becomes grid row 0.
void gc_load_slp(const uint8_t *slp_bytes) {
    uint8_t *grid = get_game_grid();
    for (int row_index = 0; row_index < GC_GRID_SIZE; row_index++) {
        int file_row_index = GC_GRID_SIZE - 1 - row_index;
        uint8_t *grid_row = &grid[row_index * GRID_ROW_BYTES];
        const uint8_t *file_row = &slp_bytes[file_row_index * GRID_ROW_BYTES];
        memcpy(grid_row, file_row, GRID_ROW_BYTES);
    }
}


// `gc_copy_grid` copies the grid of the path finder into `grid_copy`, which holds
// `GC_GRID_BYTES`.
void gc_copy_grid(uint8_t *grid_copy) {
    memcpy(grid_copy, get_game_grid(), GC_GRID_BYTES);
}


// `gc_use_grid` makes this thread read and write `grid_copy` instead of the grid of the path
// finder, or the grid of the path finder again for NULL.
void gc_use_grid(const uint8_t *grid_copy) {
    thread_grid = (uint8_t *)grid_copy;
}


// `gc_init_request` builds a path request of the game in `request`.
void gc_init_request(GcRequest *request) {
    memset(request, 0, sizeof *request);
    REQUEST_CONSTRUCTOR(request);
}


// `gc_free_request` frees the nodes of a path request of the game.
void gc_free_request(GcRequest *request) {
    REQUEST_DESTRUCTOR(request);
}


// `gc_start_search` starts a path search from `(from_x, from_y)` to `(to_x, to_y)`: the forward
// search (`StartSearchQuiet`) or the backward search (`StartSearch`), which searches from the
// destination toward the unit. Both take the start and the destination in this order.
void gc_start_search(GcRequest *request, bool is_forward, float from_x, float from_y, float to_x,
                     float to_y, int terrain_class, float corner_gap, float tolerance) {
    StartFunction start_search = is_forward ? START_SEARCH_QUIET : START_SEARCH;
    start_search(request, from_x, from_y, to_x, to_y, terrain_class, corner_gap, tolerance);
}


// `gc_update_search` runs up to `expansion_budget` expansions of a path search, and returns
// whether it found the destination, still runs, or failed.
int gc_update_search(GcRequest *request, int expansion_budget) {
    PATHFINDER_UPDATE(GC_PATHFINDER, request, expansion_budget);
    if (request->bytes[REQUEST_IS_FOUND])
        return GC_SEARCH_FOUND;
    return request->bytes[REQUEST_IS_SEARCHING] ? GC_SEARCH_RUNNING : GC_SEARCH_FAILED;
}


// `copy_and_free_list` copies up to `max_points` points of a list of the game into `points`, as
// x, y pairs, frees every node of the list, and returns the number of copied points.
static int copy_and_free_list(const GcList *list, float *points, int max_points) {
    int count = 0;        // number of copied points
    GcNode *node = list->head;
    while (node) {
        GcNode *next = node->next;
        if (count < max_points) {
            int x_index = 2 * count;
            points[x_index] = node->x;
            points[x_index + 1] = node->y;
            count++;
        }
        OPERATOR_DELETE(node);
        node = next;
    }
    return count;
}


// `gc_get_path` copies up to `max_points` points of the path of a finished search into `points`,
// as x, y pairs from the unit to the destination, and returns their number.
int gc_get_path(GcRequest *request, float *points, int max_points) {
    GcList list = EMPTY_LIST;
    GET_PATH(request, &list);
    return copy_and_free_list(&list, points, max_points);
}


// `gc_cast_ray` casts the zero-width ray of the game from `(from_x, from_y)` to `(to_x, to_y)`.
// It returns `GC_RAY_CLEAR`, or the index `(cell_y << 10) | cell_x` of the first blocking cell.
int gc_cast_ray(float from_x, float from_y, float to_x, float to_y, int terrain_class) {
    return RAYCAST(GC_PATHFINDER, from_x, from_y, to_x, to_y, terrain_class);
}


// `gc_get_last_ray_side` returns the side of the last step of the last ray: `GC_RAY_STEP_ALONG_Y`
// for a step across a y side of a cell.
int gc_get_last_ray_side(void) {
    return *get_pathfinder_field(PATHFINDER_LAST_RAY_SIDE);
}


// `gc_get_cell_at` returns the value of the cell under the map point `(x, y)`.
int gc_get_cell_at(float x, float y) {
    int cell_x;
    int cell_y;
    return gc_read_cell_at(x, y, &cell_x, &cell_y);
}


// `gc_read_cell_at` returns the value of the cell under the map point `(x, y)`, and the cell
// coordinates in `cell_x` and `cell_y`, as *CellAt* (0x4e82e0) of the game does. A point outside
// the map gives `GC_CELL_OUTSIDE` and leaves the coordinates unset.
int gc_read_cell_at(float x, float y, int *cell_x, int *cell_y) {
    // The game multiplies by its own inverse cell size, in double precision, so the cells match.
    float grid_meters = GC_GRID_SIZE * GC_CELL_METERS;
    bool is_x_inside = x > 0 && x < grid_meters;
    bool is_y_inside = y > 0 && y < grid_meters;
    if (!is_x_inside || !is_y_inside)
        return GC_CELL_OUTSIDE;
    double inverse = *GC_INVERSE_CELL_SIZE;        // inverse cell size
    *cell_x = (int)(x * inverse);
    *cell_y = (int)(y * inverse);
    uint8_t cell_byte = *get_cell_byte(*cell_x, *cell_y);
    return (cell_byte >> get_cell_shift(*cell_x)) & GC_CELL_VALUE_MASK;
}


// `gc_lay_out_formation` lays out the final parking positions of a squad with
// `CalculateFormationPoints` (0x4e7be0) of the game, into `positions` as x, y pairs. It returns
// true when the game gave exactly `position_count` positions.
bool gc_lay_out_formation(float row_center_x, float row_center_y, float direction_x,
                          float direction_y, int position_count, float unit_radius,
                          int terrain_class, int formation_type, float *positions) {
    GcList list = EMPTY_LIST;
    bool is_laid_out = CALCULATE_FORMATION_POINTS(row_center_x, row_center_y, direction_x,
                                                  direction_y, &list, position_count,
                                                  unit_radius, terrain_class, formation_type);
    int count = copy_and_free_list(&list, positions, position_count);
    return is_laid_out && count == position_count;
}
