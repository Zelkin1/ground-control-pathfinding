/* game.h is the interface from the simulator to the real gc.exe 1.0.1.1 code. The rest of the
 * simulator calls a few functions of the game through the wrappers below: the path search, the
 * grid raycast, the cell lookup and the formation layout. Call `gc_load` first, because it
 * loads the exe into this process.
 *
 * Terms:
 *   World units: the unit of all positions and distances below. They are meters.
 *   Cell: one square of the passability grid. See `GC_CELL_METERS` and the grid comment below.
 *   Mover class: a number from 0 to 2 that says which ground a mover can cross. A mover of
 *     class 1 is a tank and a mover of class 2 is infantry.
 *
 * re/PATHFINDING.md describes each game function and its contract. */
#ifndef GAME_H
#define GAME_H

#include <stdbool.h>
#include <stdint.h>

#define GC_CELL_METERS 6.25f   /* A grid cell is this many world units (meters) wide. */
#define GC_GRID_SIZE 1024      /* The grid has this many cells along each side. */
#define GC_GRID_BYTES (GC_GRID_SIZE * GC_GRID_SIZE / 4) /* The grid takes 256 KB. */
#define GC_CELL_VALUE_MASK 3   /* This mask keeps the 2 bits of the value of one cell. */
#define GC_CELL_CLIFF 3        /* Cliffs have this value, which blocks mover classes 0 to 2. */
#define GC_CELL_OUTSIDE 4      /* `gc_get_cell_at` returns this outside the grid. */
#define GC_CELL_Y_SHIFT 10     /* A cell index is `cell_y << GC_CELL_Y_SHIFT | cell_x`. */
#define GC_CELL_X_MASK 0x3ff   /* This mask takes `cell_x` out of a cell index. */
#define GC_SEARCH_QUIET true   /* This `is_quiet` value picks the forward search. */
#define GC_SEARCH_ACTIVE false /* This `is_quiet` value picks the backward search. */
#define GC_RAY_CLEAR (-1)      /* `gc_cast_ray` returns this when the line is clear. */
#define GC_RAY_STEP_ALONG_Y 0  /* `gc_get_last_ray_side` returns this after a step along y. */

#define GC_REQUEST_SIZE 0x40 /* PathRequest in the game takes this many bytes. */

/* `GcRequest` holds the state of one search, a PathRequest of the game, as raw bytes that only
 * game.c reads. */
typedef struct GcRequest {
    uint8_t bytes[GC_REQUEST_SIZE];
} GcRequest;

/* `gc_load` loads gc.exe from `exe_path` into this process, so that the functions below work.
 * Call it once, before the other functions below.
 * Result: It returns true on success. It returns false when it cannot read the file or map the
 *   image, and it prints a message for each of these errors except a failed read. */
bool gc_load(const char *exe_path);

/* The passability grid has `GC_GRID_SIZE` x `GC_GRID_SIZE` cells. Cell (`cell_x`, `cell_y`)
 * covers world x from `cell_x * GC_CELL_METERS` to `(cell_x + 1) * GC_CELL_METERS`, and the same
 * for y. Each cell holds a value from 0 to 3, and it blocks a mover when its value is greater
 * than the class of the mover: 2 (hills) blocks tanks (class 1) but not infantry (class 2). */

/* `gc_is_outside_grid` returns true when cell (`cell_x`, `cell_y`) is not in the grid, and false
 * when it is. */
bool gc_is_outside_grid(int cell_x, int cell_y);

/* `gc_set_cell` stores the lowest 2 bits of `value` in a cell. It ignores cells outside the
 * grid. */
void gc_set_cell(int cell_x, int cell_y, int value);

/* `gc_get_cell` returns the value of a cell, or `GC_CELL_CLIFF` for a cell outside the grid, so
 * that movers treat the ground outside the map as cliffs. */
int gc_get_cell(int cell_x, int cell_y);

/* `gc_clear_grid` sets every cell to 0, the value of free ground. */
void gc_clear_grid(void);

/* `gc_load_slp` copies the grid of a map from `slp_bytes`, the contents of its .SLP file. The
 * file holds `GC_GRID_SIZE` rows in the format of the grid itself, and its last row becomes grid
 * row 0. */
void gc_load_slp(const uint8_t *slp_bytes);

/* Grid copies let worker threads read a grid while the main thread changes the grid of the game.
 * Only the cell reads of the simulator use a copy: `gc_get_cell`, `gc_get_cell_at` and
 * `gc_read_cell_at`. The code of the game itself, such as the search and `gc_cast_ray`, always
 * reads the grid of the game. */

/* `gc_copy_grid` copies the grid of the game, as it is now, into `grid_copy`, which must hold
 * `GC_GRID_BYTES` bytes. The caller owns `grid_copy`. */
void gc_copy_grid(uint8_t *grid_copy);

/* `gc_use_grid` makes the cell reads of the calling thread read `grid_copy`, a copy that
 * `gc_copy_grid` filled, until the next call. NULL makes them read the grid of the game again,
 * which each thread does from its start. */
void gc_use_grid(const uint8_t *grid_copy);

/* The path search is code of the game itself (re/PATHFINDING.md, "The path search"). It builds
 * a route from straight lines between corner points of the obstacles in the way. A `GcRequest`
 * holds the state of one search. Prepare it with `gc_init_request` and start a search with
 * `gc_start_search`. Then call `gc_update_search` while it returns `GC_SEARCH_RUNNING`, read the
 * route with `gc_get_path` after `GC_SEARCH_FOUND`, and free the request with
 * `gc_free_request`. */

/* `gc_init_request` prepares an empty request. Call it once for each request, before
 * `gc_start_search`. */
void gc_init_request(GcRequest *request);

/* `gc_free_request` frees the memory that the search of the request holds. Call it once for each
 * request that `gc_init_request` prepared. */
void gc_free_request(GcRequest *request);

/* `gc_start_search` starts a search for a mover of `mover_class` between `from` =
 * (`from_x`, `from_y`) and `to` = (`to_x`, `to_y`).
 * Inputs: `is_quiet` picks the direction. `GC_SEARCH_QUIET` searches forward from `from` to
 *   `to`. `GC_SEARCH_ACTIVE` searches backward from `to` to `from`. The corner points of the
 *   route keep `clearance` from the edges of blocking cells, and the game passes twice the
 *   radius of the unit.
 *   The search checks the lines between corner points with no width. It ends at a point within
 *   `tolerance` of its target, which is `to` for the forward search and `from` for the backward
 *   one. */
void gc_start_search(GcRequest *request, bool is_quiet, float from_x, float from_y, float to_x,
                     float to_y, int mover_class, float clearance, float tolerance);

/* These are the results of `gc_update_search`. */
enum {
    GC_SEARCH_FAILED = -1, /* The search ended without a route. */
    GC_SEARCH_RUNNING = 0, /* The budget ran out before the search ended. */
    GC_SEARCH_FOUND = 1,   /* The search reached its target. */
};

/* `gc_update_search` runs up to `expansion_budget` expansions of the search. An expansion is
 * one step of the search.
 * Result: It returns `GC_SEARCH_FAILED`, `GC_SEARCH_RUNNING` or `GC_SEARCH_FOUND`.
 * Edge case: A search fails after 500 expansions in total, whatever the budgets. */
int gc_update_search(GcRequest *request, int expansion_budget);

/* `gc_get_path` reads the route of a search that returned `GC_SEARCH_FOUND`.
 * Result: `points` gets the route from the `from` of `gc_start_search` to its `to`, as x, y pairs
 *   and without the point at `from`. The caller owns `points`, which must have room for
 *   `2 * max_points` floats. `gc_get_path` returns how many points it copied.
 * Edge case: When the route has more than `max_points` points, it copies only the first
 *   `max_points`. */
int gc_get_path(GcRequest *request, float *points, int max_points);

/* `gc_cast_ray` follows the straight line from (`from_x`, `from_y`) to (`to_x`, `to_y`) through
 * the grid. The line has no width.
 * Result: It returns `GC_RAY_CLEAR` when no cell on the line blocks `mover_class`, or else the
 *   cell index of the first blocking cell. */
int gc_cast_ray(float from_x, float from_y, float to_x, float to_y, int mover_class);

/* `gc_get_last_ray_side` tells through which edge the line of the last `gc_cast_ray` entered its
 * blocking cell.
 * Result: It returns `GC_RAY_STEP_ALONG_Y` when the last step of the line through the grid
 *   went along y, so that the line crossed an edge of the cell that runs along x. It returns
 *   another value when the last step went along x. */
int gc_get_last_ray_side(void);

/* `gc_get_cell_at` returns the value of the cell at world position (`x`, `y`), exactly as CellAt
 * of the game does. Outside the grid, and on its lower edges x = 0 or y = 0, it returns
 * `GC_CELL_OUTSIDE`, which blocks every mover class. */
int gc_get_cell_at(float x, float y);

/* `gc_read_cell_at` returns the same value as `gc_get_cell_at`, and sets `cell_x` and `cell_y` to
 * the cell that it read. Where it returns `GC_CELL_OUTSIDE`, it leaves both unchanged. */
int gc_read_cell_at(float x, float y, int *cell_x, int *cell_y);

/* `gc_place_formation` lays out the slots of a squad.
 * Inputs: The first row of slots is centered on (`anchor_x`, `anchor_y`), and each next row is one
 *   spacing further in the opposite direction of (`direction_x`, `direction_y`). `unit_radius`
 *   sets the spacing, `mover_class` decides which cells block a slot, and `formation_type` picks
 *   the shape: 0 a line, 1 or 3 a box, 2 a column, and any other value rows of 4.
 * Result: `slots` gets up to `slot_count` slots as x, y pairs. The caller owns `slots`, which
 *   must have room for `2 * slot_count` floats. `gc_place_formation` returns true when the game
 *   reports success and placed exactly `slot_count` slots, and false otherwise. */
bool gc_place_formation(float anchor_x, float anchor_y, float direction_x, float direction_y,
                        int slot_count, float unit_radius, int mover_class, int formation_type,
                        float *slots);

#endif
