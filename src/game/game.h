// Calls into the code of the real gc.exe (docs/PATHFINDING.md): the passability grid, the base game
// path search, the zero-width ray and the formation layout of the game (game.c). The simulator
// maps gc.exe into its own memory with `gc_load` (game_image.c), which must run before every other
// function; the game mod runs inside gc.exe and needs no `gc_load`.
//
// The grid has `GC_GRID_SIZE` x `GC_GRID_SIZE` cells of `GC_CELL_METERS`, with cell (0, 0) at the
// map origin. A cell value goes from 0 to 3, and a cell blocks a unit when its value is larger
// than the terrain class of the unit. The cell functions of this file read and write the grid of
// the calling thread (see `gc_use_grid`), but the path search and the ray of the game always read
// the grid of the path finder.
#ifndef GAME_H
#define GAME_H

#include <stdbool.h>
#include <stdint.h>

#define GC_CELL_METERS 6.25f                                   // side of a grid cell
#define GC_GRID_SIZE 1024                                      // cells on each side of the grid
#define GC_GRID_BYTES (GC_GRID_SIZE * GC_GRID_SIZE / 4)        // grid size, 4 cells per byte
#define GC_CELL_VALUE_MASK 3                                   // bits of one cell value
#define GC_CELL_CLIFF 3                                        // value that blocks every unit
#define GC_CELL_OUTSIDE 4                                      // value of a point off the map
#define GC_CELL_Y_SHIFT 10                                     // shift of `cell_y` in a cell index
#define GC_CELL_X_MASK 0x3ff                                   // bits of `cell_x` in a cell index
#define GC_SEARCH_FORWARD true                                 // forward search
#define GC_SEARCH_BACKWARD false                               // backward search
#define GC_RAY_CLEAR (-1)                                      // ray result without blocking cell
#define GC_RAY_STEP_ALONG_Y 0                                  // last ray step crossed a y side

#define GC_REQUEST_SIZE 0x40                                   // size of a PathRequest of the game
#define GC_OPERATOR_NEW 0x5e89be                               // `operator new` of the game
#define GC_OPERATOR_DELETE 0x5e89e7                            // `operator delete` of the game

// Path request of the game, with its search state and its nodes. `gc_init_request` builds it,
// and `gc_free_request` frees its nodes.
typedef struct GcRequest {
    uint8_t bytes[GC_REQUEST_SIZE];
} GcRequest;

// `gc_load` maps gc.exe from `exe_path` and patches it for calls from the simulator. It returns
// false when the file cannot be read or mapped.
bool gc_load(const char *exe_path);

// `gc_is_outside_grid` tells whether the cell `(cell_x, cell_y)` lies outside the grid.
bool gc_is_outside_grid(int cell_x, int cell_y);

// `gc_set_cell` sets the value of a cell. A cell outside the grid stays unchanged.
void gc_set_cell(int cell_x, int cell_y, int value);

// `gc_get_cell` returns the value of a cell, or `GC_CELL_CLIFF` outside the grid.
int gc_get_cell(int cell_x, int cell_y);

// `gc_clear_grid` sets every cell of the grid of the path finder to free terrain.
void gc_clear_grid(void);

// `gc_load_slp` copies the grid of a .SLP map file, `GC_GRID_BYTES` long, into the path finder,
// with its rows in reverse order as the game loads them.
void gc_load_slp(const uint8_t *slp_bytes);

// `gc_copy_grid` copies the grid of the path finder into `grid_copy`, which holds
// `GC_GRID_BYTES`.
void gc_copy_grid(uint8_t *grid_copy);

// `gc_use_grid` makes the cell functions of this thread use `grid_copy`, or the grid of the path
// finder for NULL. The caller keeps `grid_copy` alive while the thread uses it.
void gc_use_grid(const uint8_t *grid_copy);

// `gc_init_request` builds a path request of the game in `request`.
void gc_init_request(GcRequest *request);

// `gc_free_request` frees the nodes of a path request of the game.
void gc_free_request(GcRequest *request);

// `gc_start_search` starts a base game path search from `(from_x, from_y)` to `(to_x, to_y)`.
// `is_forward` picks the forward search or the backward search, which runs from the destination
// toward the start. `corner_gap` is the gap that the path keeps at corners, and `tolerance` is how
// close to the destination the path can end.
void gc_start_search(GcRequest *request, bool is_forward, float from_x, float from_y, float to_x,
                     float to_y, int terrain_class, float corner_gap, float tolerance);

// Results of `gc_update_search`.
enum {
    GC_SEARCH_FAILED = -1,
    GC_SEARCH_RUNNING = 0,
    GC_SEARCH_FOUND = 1,
};

// `gc_update_search` runs up to `expansion_budget` expansions of a path search, and returns
// whether it found the destination, still runs, or failed.
int gc_update_search(GcRequest *request, int expansion_budget);

// `gc_get_path` copies up to `max_points` points of the path of a finished search into `points`,
// as x, y pairs from the start to the destination, and returns their number.
int gc_get_path(GcRequest *request, float *points, int max_points);

// `gc_cast_ray` casts the zero-width ray of the game from `(from_x, from_y)` to `(to_x, to_y)`.
// It returns `GC_RAY_CLEAR`, or the cell index (`cell_y << GC_CELL_Y_SHIFT | cell_x`) of the
// first cell that blocks `terrain_class`.
int gc_cast_ray(float from_x, float from_y, float to_x, float to_y, int terrain_class);

// `gc_get_last_ray_side` returns the cell side that the last step of the last ray crossed, which
// is `GC_RAY_STEP_ALONG_Y` for a y side.
int gc_get_last_ray_side(void);

// `gc_get_cell_at` returns the value of the cell under the map point `(x, y)`, or
// `GC_CELL_OUTSIDE` off the map.
int gc_get_cell_at(float x, float y);

// `gc_read_cell_at` works as `gc_get_cell_at`, and also returns the cell coordinates, which stay
// unset off the map.
int gc_read_cell_at(float x, float y, int *cell_x, int *cell_y);

// `gc_lay_out_formation` lays out `position_count` final parking positions of a squad with the
// formation layout of the game, its first row centered on the row center and facing `direction`.
// It writes them into `positions` as x, y pairs, and returns false when the layout fails.
bool gc_lay_out_formation(float row_center_x, float row_center_y, float direction_x,
                          float direction_y, int position_count, float unit_radius,
                          int terrain_class, int formation_type, float *positions);

#endif
