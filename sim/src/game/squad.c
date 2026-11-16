/* This file reconstructs the squad movement of the game from the decompiled gc.exe, as movement.c
 * does for the units. It fits into the tick of the simulator like this: sim.c calls `order_move`
 * for each order, and then in every tick it calls `update_squad` for each squad, before movement.c
 * moves the units.
 *
 * Life of a squad:
 *   A squad gets a move order, searches its route, sets up the move and arrives. In game mode,
 *   each member then follows its waypoints from `set_move`. In the formation modes, where
 *   `avoid_mode` is `AVOID_FORM` or `AVOID_COORD`, formation.c sets up the move and moves the
 *   anchor of the squad (a virtual leader on the squad route).
 *
 * Parked squads:
 *   A parked squad is a squad that neither moves nor searches. In formation mode, `stamp_parked`
 *   and `unstamp_parked` make the searches and the formation layouts go around parked squads.
 *   The own searches of a squad use them here, and the searches of single units use them in
 *   movement.c. In the coord mode, a squad that arrives, or that settles at the end of its route,
 *   makes the moving squads whose routes pass it search again, through `replan_round_parked`.
 *
 * Fields of `Unit` that this file sets:
 *   `set_up_move` sets `slot`, `reach_last` and `reach_mid` of each member in game mode, and
 *   `set_move` fills in its waypoints. `update_squad` sets `reach_last` to `STOP_REACH_METERS`
 *   and calls `stop_at` when the squad arrives. The units start with these Mover values (see
 *   sim.h), and the code of the game sets them again in the functions named below.
 *   This file only reads the other fields of `Unit` that it uses, such as `position`,
 *   `is_stopped`, `is_spinning`, `velocity` and `last_turn_rate`.
 *
 * Game addresses:
 *   Each function that follows a game function names it and its address in gc.exe 1.0.1.1, whose
 *   image always loads at 0x400000. re/PATHFINDING.md describes these functions. The constants
 *   are the values of the game itself unless their comments say otherwise. */
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "sim.h"
#include "formation.h"

/* These values belong to the simulator itself and serve formation mode.
 * While a search or a formation layout runs, formation mode stamps the parked units: it writes
 * cliffs (cells that block every unit) into the grid under them, so that routes go around
 * them. */
#define MAX_STAMPED_CELLS 4096          /* `stamp_parked` changes at most this many cells. */
#define PARKED_STAMP_MARGIN_METERS 1.0f /* Cells this close to parked unit bodies are stamped. */

/* The search, set-up and arrival of a squad use these values. A straggler is the last member that
 * still moves. */
#define SQUAD_SEARCH_BUDGET 60          /* A squad search runs this many expansions a tick. */
#define SQUAD_SEARCH_TOLERANCE_METERS 0.0f /* Squad searches end only at their target. */
#define MIN_FACING_LENGTH_SQUARED 1e-5f /* Facings with this squared length or less become zero. */
#define MIN_LAST_REACH_METERS 1.0f      /* SetReachLast stores at least this reach. */
#define STRAGGLER_MIN_MEMBERS 2         /* Only squads larger than this run the straggler timer. */
#define STRAGGLER_WAIT_SECONDS 20.0f    /* Squads arrive this long after all but one stopped. */
#define NO_STRAGGLER_TIMER -1.0f        /* The straggler timer holds this while it does not run. */

/* These values belong to the simulator itself and serve the coord mode: the new
 * searches around parked squads, and the test for squads that settle. */
#define PARKED_ROUTE_STEP_METERS 2.0f   /* Routes are checked this often for parked squads. */
#define PARKED_ROUTE_MARGIN_METERS 2.0f /* A route passes a parked unit within this gap. */
#define TIGHT_CIRCLE_RADII 2.0f         /* A unit circles tightly within this many body radii. */
#define PARKED_SPEED_FRACTION 0.1f      /* With `--parkspeed`, slower units count as parked. */
#define PARK_AHEAD_SECONDS 16.0f      /* With `--parkahead`, look this long ahead for slots. */
#define NO_PASS_DISTANCE (-1.0f)        /* `find_route_pass_distance` returns this for no pass. */

/* This value belongs to the simulator itself and serves `--fullsearch on` in the formation modes.
 * It is the limit of the game on the expansions of one search, so a search that gets it in one
 * call always ends in that call. */
#define FULL_SEARCH_BUDGET 500          /* The game ends each search after this many expansions. */

/* These values belong to the simulator itself and serve `--setupslice on` in the formation
 * modes. There, the route set-up of a squad (formation.c and route.c) runs over several ticks,
 * so that no tick takes long. One squad sets up at a time, and the others wait their turn. The
 * phase of a squad in its set-up is in `set_up_phase`.
 * With `--setupchain on`, the set-ups of a tick share a work budget. The work is counted in room
 * tests (tests of the room around one sample of a smoothed route, about 0.5 us each). The begin
 * and the finish run no room tests, so their work is an estimate from timed runs: about 30 us per
 * route point for the begin, and about 0.8 us per lane sample for the finish. */
enum {
    SET_UP_NONE = 0,  /* The squad is not in a sliced set-up. */
    SET_UP_WAITING,   /* The search found its path, and the set-up waits for its turn. */
    SET_UP_SMOOTHING, /* The smoothing of the route runs, part by part. */
    SET_UP_FINISHING, /* The lanes and places come next, and then the move starts. */
};
#define SET_UP_ROOM_TESTS_PER_TICK 2000 /* A sliced smoothing runs this many room tests a tick. */
#define SET_UP_WORK_PER_TICK 4000       /* With `--setupchain on`, set-ups share this a tick. */
#define SET_UP_WORK_PER_ROUTE_POINT 60  /* The begin counts as this much work per route point. */
#define SET_UP_WORK_PER_LANE_SAMPLE 2   /* The finish counts as this much work per lane sample. */
#define FNV_OFFSET_BASIS 2166136261u   /* `hash_stamped_cells` uses the FNV-1a hash ... */
#define FNV_PRIME 16777619u            /* ... with these constants. */

/* ------------------------------------------------------------------------------------------------
 * `get_truncated_cell_coordinate` returns the grid column or row that holds the world coordinate
 * `meters`, like `get_cell_coordinate` in sim.h, but it rounds toward zero instead of down. The
 * two differ only for negative coordinates. TODO: confirm which rounding is meant here.
 * --------------------------------------------------------------------------------------------- */
static int get_truncated_cell_coordinate(float meters) {
    return (int)(meters / GC_CELL_METERS);
}

/* ------------------------------------------------------------------------------------------------
 * `get_cell_index` returns the index of the cell that holds `point`.
 * --------------------------------------------------------------------------------------------- */
static int get_cell_index(V2 point) {
    int cell_x = get_truncated_cell_coordinate(point.x);
    int cell_y = get_truncated_cell_coordinate(point.y);
    return make_cell_index(cell_x, cell_y);
}

/* `stamped_cell_indices[0..stamped_count)` holds the indices of the cells that `stamp_parked`
 * changed, in the order they changed, and `stamped_values` holds their old values at the same
 * positions. */
/* Only the main thread stamps, so the lists are shared. `stamped_count` is per thread
 * (`_Thread_local`): on a worker thread of the route set-up it stays 0, so that
 * `hash_stamped_cells` there never reads the lists while the main thread changes them. */
static int stamped_cell_indices[MAX_STAMPED_CELLS];
static int stamped_values[MAX_STAMPED_CELLS];
static _Thread_local int stamped_count;

/* A `StampBox` holds the box around the stamped cells: they lie from column `min_x` to `max_x`
 * and from row `min_y` to `max_y`, ends included. The other fields mean nothing while `has_cells`
 * is false. */
typedef struct {
    bool has_cells;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
} StampBox;

/* `stamp_box` is the box of the stamps that the cell reads of this thread see (`_Thread_local`).
 * On the main thread, `stamp_parked` and `unstamp_parked` keep it, so `has_cells` is true while
 * `stamped_count` is above 0. A worker thread of the route set-up takes the box of the squad that
 * it smooths, together with a grid copy that holds the stamps of that squad. */
static _Thread_local StampBox stamp_box;

/* ------------------------------------------------------------------------------------------------
 * `grow_stamp_box` grows the box around the stamped cells so that it holds cell (`cell_x`,
 * `cell_y`), which is about to be stamped. The first stamped cell of a call of `stamp_parked`
 * starts the box.
 * --------------------------------------------------------------------------------------------- */
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

/* ------------------------------------------------------------------------------------------------
 * `get_box_axis_distance` returns how many cells `value` lies outside the range from `min_value`
 * to `max_value` along one axis, or 0 inside it.
 * --------------------------------------------------------------------------------------------- */
static int get_box_axis_distance(int value, int min_value, int max_value) {
    if (value < min_value)
        return min_value - value;
    if (value > max_value)
        return value - max_value;
    return 0;
}

/* ------------------------------------------------------------------------------------------------
 * `get_stamp_distance_cells` takes the larger of the distances along the two axes, so that a cell
 * k cells away has k - 1 whole cells between it and the nearest stamped cell.
 * --------------------------------------------------------------------------------------------- */
int get_stamp_distance_cells(int cell_x, int cell_y) {
    if (!stamp_box.has_cells)
        return NO_STAMP_DISTANCE_CELLS;
    int distance_x = get_box_axis_distance(cell_x, stamp_box.min_x, stamp_box.max_x);
    int distance_y = get_box_axis_distance(cell_y, stamp_box.min_y, stamp_box.max_y);
    return distance_x > distance_y ? distance_x : distance_y;
}

/* ------------------------------------------------------------------------------------------------
 * `stamp_disc` stamps the cells that come closer than `radius` to `center`. It leaves out the
 * cells `start_cell_index` and `end_cell_index`, cells that already are cliffs, and every cell
 * once the list of changed cells is full. It keeps the two end cells as they are so that a
 * search never starts or ends in a stamped cliff. It stops at `MAX_STAMPED_CELLS` changed cells
 * because the list of changed cells has that many entries.
 *
 *   +---+---+---+
 *   |   | x |   |    x: a stamped cell
 *   +---+---+---+
 *   | x | o | x |    o: the stamped cell that holds `center`
 *   +---+---+---+
 *   |   | x |   |
 *   +---+---+---+
 *
 *   Here `center` lies in the middle of cell o, and `radius` is a little more than half a cell.
 *   The disc then extends into the four side cells, but not into the corner cells, whose points
 *   closest to `center` lie further away.
 * --------------------------------------------------------------------------------------------- */
static void stamp_disc(V2 center, float radius, int start_cell_index, int end_cell_index) {
    /* Go through the square of cells around the disc. */
    int min_cell_y = get_truncated_cell_coordinate(center.y - radius);
    int max_cell_y = get_truncated_cell_coordinate(center.y + radius);
    int min_cell_x = get_truncated_cell_coordinate(center.x - radius);
    int max_cell_x = get_truncated_cell_coordinate(center.x + radius);
    for (int cell_y = min_cell_y; cell_y <= max_cell_y; cell_y++) {
        for (int cell_x = min_cell_x; cell_x <= max_cell_x; cell_x++) {
            /* Skip the cell when its point closest to `center` lies `radius` or more from it,
             * when it holds an end of the search or is a cliff already, or when the list is
             * full. */
            V2 closest = get_closest_cell_point(cell_x, cell_y, center);
            int cell_index = make_cell_index(cell_x, cell_y);
            int cell_value = gc_get_cell(cell_x, cell_y);
            bool is_in_disc = length_squared(sub(closest, center)) < radius * radius;
            bool is_search_end = cell_index == start_cell_index || cell_index == end_cell_index;
            bool is_kept = is_search_end || cell_value == GC_CELL_CLIFF;
            bool has_room = stamped_count < MAX_STAMPED_CELLS;
            if (!is_in_disc || is_kept || !has_room)
                continue;

            /* Remember the old value, grow the box around the stamped cells, and write a
             * cliff. */
            stamped_cell_indices[stamped_count] = cell_index;
            stamped_values[stamped_count] = cell_value;
            grow_stamp_box(cell_x, cell_y);
            stamped_count++;
            gc_set_cell(cell_x, cell_y, GC_CELL_CLIFF);
        }
    }
}

/* ------------------------------------------------------------------------------------------------
 * `stamp_unit` stamps the cells that come within `PARKED_STAMP_MARGIN_METERS` of the body of
 * `parked`. It calls `stamp_disc` with the radius of the body plus the margin.
 * --------------------------------------------------------------------------------------------- */
static void stamp_unit(const Unit *parked, int start_cell_index, int end_cell_index) {
    float radius = parked->kind->radius + PARKED_STAMP_MARGIN_METERS;
    stamp_disc(parked->position, radius, start_cell_index, end_cell_index);
}

/* ------------------------------------------------------------------------------------------------
 * `get_member_point` returns where a member counts for the parked-squad tests: its final slot
 * when `is_at_slots` is true, as for a squad that will park there soon (`--parkahead`), and its
 * position otherwise.
 * --------------------------------------------------------------------------------------------- */
static V2 get_member_point(const Unit *member, bool is_at_slots) {
    return is_at_slots ? member->slot : member->position;
}

/* ------------------------------------------------------------------------------------------------
 * `stamp_squad_disc` stamps one disc around all members of the parked squad `parked`, at their
 * positions, or at their slots when `is_at_slots` is true. Its center is the mean of those
 * points, and its edge lies `PARKED_STAMP_MARGIN_METERS` past the farthest body edge from there.
 * It takes every member, also one that is not `is_stopped` yet: a squad that arrives tells its
 * members to stop where they stand, and they count as stopped only in the next tick, while the
 * searches that its arrival starts run in this one.
 * --------------------------------------------------------------------------------------------- */
static void stamp_squad_disc(const Squad *parked, bool is_at_slots, int start_cell_index,
                             int end_cell_index) {
    /* Find the center of the members. */
    V2 center_sum = v2(0, 0);
    int end_index = parked->first_unit_index + parked->unit_count;
    for (int unit_index = parked->first_unit_index; unit_index < end_index; unit_index++)
        center_sum = add(center_sum, get_member_point(&units[unit_index], is_at_slots));
    V2 center = mul(center_sum, 1.0f / parked->unit_count);

    /* Find the farthest body edge from the center, and stamp the disc. */
    float farthest_edge_distance = 0;
    for (int unit_index = parked->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *member = &units[unit_index];
        V2 member_point = get_member_point(member, is_at_slots);
        float edge_distance = len(sub(member_point, center)) + member->kind->radius;
        farthest_edge_distance = fmaxf(farthest_edge_distance, edge_distance);
    }
    float stamp_radius = farthest_edge_distance + PARKED_STAMP_MARGIN_METERS;
    stamp_disc(center, stamp_radius, start_cell_index, end_cell_index);
}

/* ------------------------------------------------------------------------------------------------
 * `stamp_parked` writes cliffs into the grid of the game on the cells within
 * `PARKED_STAMP_MARGIN_METERS` of the bodies of the parked units. The search and the layout of
 * formation code then go around them without knowing about parked units. It keeps the cells that
 * it changed in `stamped_cell_indices`, with their old values in `stamped_values`, so that
 * `unstamp_parked` can undo the stamps. Each cell changes at most once, because a stamped cell is
 * a cliff and `stamp_disc` skips cliffs. This also means that the list never holds more than one
 * call of `stamp_parked`, which is why each call needs its `unstamp_parked`.
 * --------------------------------------------------------------------------------------------- */
void stamp_parked(const Squad *moving_squad, V2 start, V2 end) {
    /* Start an empty list of changed cells, which stays empty in game mode. */
    stamped_count = 0;
    stamp_box.has_cells = false;
    if (avoid_mode == AVOID_GAME)
        return;

    /* For a squad that searches again around squads that parked on its route, stamp one disc
     * around each other parked squad, so that the new route keeps the whole squad on one side.
     * Stamps of single units leave gaps between the members, through which the search can lead
     * the formation. A squad that `is_settled` counts as parked here too. With `--parkahead`, a
     * squad that still drives to its slots, but parks there before the moving squad gets there
     * (`is_avoiding_slots`), gets its disc at its slots. */
    int start_cell_index = get_cell_index(start);
    int end_cell_index = get_cell_index(end);
    if (moving_squad->is_replanning) {
        for (int squad_index = 0; squad_index < squad_count; squad_index++) {
            const Squad *other_squad = &squads[squad_index];
            bool is_other_squad = other_squad != moving_squad;
            bool is_parked_squad = !is_squad_busy(other_squad) || other_squad->is_settled;
            bool is_parking_ahead = moving_squad->is_avoiding_slots[squad_index];
            if (is_other_squad && is_parked_squad)
                stamp_squad_disc(other_squad, false, start_cell_index, end_cell_index);
            else if (is_other_squad && is_parking_ahead)
                stamp_squad_disc(other_squad, true, start_cell_index, end_cell_index);
        }
        return;
    }

    /* Otherwise stamp the parked units of the other squads. */
    for (int other_index = 0; other_index < unit_count; other_index++) {
        const Unit *other = &units[other_index];
        const Squad *other_squad = &squads[other->squad_index];
        bool is_other_squad = other_squad != moving_squad;
        bool is_parked = other->is_stopped && !is_squad_busy(other_squad);
        if (is_other_squad && is_parked)
            stamp_unit(other, start_cell_index, end_cell_index);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `unstamp_parked` goes through the list of changed cells from the last entry to the first, so
 * the last stamped cell gets its old value back first. It empties the list on the way.
 * --------------------------------------------------------------------------------------------- */
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

/* ------------------------------------------------------------------------------------------------
 * `hash_stamped_cells` hashes the list of changed cells with FNV-1a, over each cell index and
 * each old value in list order. The hash is 32 bits wide, so two different lists share a hash
 * only by a rare chance.
 * --------------------------------------------------------------------------------------------- */
uint32_t hash_stamped_cells(void) {
    uint32_t hash = FNV_OFFSET_BASIS;
    for (int stamp_index = 0; stamp_index < stamped_count; stamp_index++) {
        hash = (hash ^ (uint32_t)stamped_cell_indices[stamp_index]) * FNV_PRIME;
        hash = (hash ^ (uint32_t)stamped_values[stamp_index]) * FNV_PRIME;
    }
    return hash;
}

/* ------------------------------------------------------------------------------------------------
 * `find_closest_member` returns the index of the member of the squad closest to `point`, the first
 * one on a tie. The members of a squad are
 * `units[first_unit_index .. first_unit_index + unit_count)`.
 * --------------------------------------------------------------------------------------------- */
static int find_closest_member(const Squad *squad, V2 point) {
    int best_index = squad->first_unit_index;
    float best_distance_squared = length_squared(sub(units[best_index].position, point));
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        float distance_squared = length_squared(sub(units[unit_index].position, point));
        if (distance_squared < best_distance_squared) {
            best_index = unit_index;
            best_distance_squared = distance_squared;
        }
    }
    return best_index;
}

/* ------------------------------------------------------------------------------------------------
 * `start_route_search` starts the quiet search of the squad (the forward search of the game) to
 * its `destination` from the lead, the member closest to the destination. The search takes the
 * mover class and clearance of the lead (the gap that the route keeps at obstacle corners).
 * `start_route_search` also sets the state to `SQUAD_SEARCHING`.
 * --------------------------------------------------------------------------------------------- */
static void start_route_search(Squad *squad) {
    /* Start from the lead. */
    int lead_index = find_closest_member(squad, squad->destination);
    const Unit *lead = &units[lead_index];
    squad->start_position = lead->position;
    squad->is_active_search = false;

    /* Start the search with the parked units stamped. */
    V2 start = squad->start_position;
    V2 end = squad->destination;
    int mover_class = lead->kind->mover_class;
    float clearance = get_search_clearance(lead);
    stamp_parked(squad, start, end);
    gc_start_search(&squad->search, GC_SEARCH_QUIET, start.x, start.y, end.x, end.y, mover_class,
                    clearance, SQUAD_SEARCH_TOLERANCE_METERS);
    unstamp_parked();
    squad->state = SQUAD_SEARCHING;

    /* A new search ends a sliced set-up that was still running. */
    squad->set_up_phase = SET_UP_NONE;
}

/* ------------------------------------------------------------------------------------------------
 * `order_move` follows the move order of Squad::ProcessOrder (0x50ac20). It clears the flags of
 * an earlier replan and settle, because the new order starts a new route. It takes `facing` from
 * `start_position`, which `start_route_search` just set to the position of the lead, because
 * `set_up_move` lays out the formation along this facing.
 * --------------------------------------------------------------------------------------------- */
void order_move(Squad *squad, V2 destination) {
    squad->destination = destination;
    squad->is_replanning = false;
    squad->is_settled = false;
    memset(squad->is_avoiding_slots, 0, sizeof squad->is_avoiding_slots);
    start_route_search(squad);
    squad->facing = sub(destination, squad->start_position);
    squad->order_time_seconds = now_seconds;
    squad->start_time_seconds = NOT_STARTED_SECONDS;
}

/* ------------------------------------------------------------------------------------------------
 * `find_route_pass_distance` returns the first distance along the route of `moving`, from `from`
 * to `to` in steps of `PARKED_ROUTE_STEP_METERS`, at which the route comes within `half_width`
 * (the half-width of the formation of `moving`) plus the radius of a member plus
 * `PARKED_ROUTE_MARGIN_METERS` of a member of `parked`. It takes the members at their positions,
 * or at their slots when `is_at_slots` is true.
 * Result: It returns `NO_PASS_DISTANCE` when the route stays clear of them over that stretch.
 * --------------------------------------------------------------------------------------------- */
static float find_route_pass_distance(const Squad *moving, float half_width, const Squad *parked,
                                      bool is_at_slots, float from, float to) {
    float distance = from;
    int end_index = parked->first_unit_index + parked->unit_count;
    while (distance <= to) {
        V2 unused_direction;
        V2 route_point = get_route_point(moving, distance, &unused_direction);
        for (int unit_index = parked->first_unit_index; unit_index < end_index; unit_index++) {
            const Unit *member = &units[unit_index];
            V2 member_point = get_member_point(member, is_at_slots);
            float touch_distance = half_width + member->kind->radius;
            float pass_distance = touch_distance + PARKED_ROUTE_MARGIN_METERS;
            if (length_squared(sub(member_point, route_point)) < pass_distance * pass_distance)
                return distance;
        }
        distance += PARKED_ROUTE_STEP_METERS;
    }
    return NO_PASS_DISTANCE;
}

/* ------------------------------------------------------------------------------------------------
 * `find_slot_distance` returns the distance along the route of `moving` at which the route comes
 * closest to the slots of `parker`. It starts at `pass_distance`, where the route first comes
 * within reach of them (`find_route_pass_distance`), and steps on by `PARKED_ROUTE_STEP_METERS`
 * while the route gets closer to the nearest slot, up to the end of the route.
 * --------------------------------------------------------------------------------------------- */
static float find_slot_distance(const Squad *moving, const Squad *parker, float pass_distance) {
    float best_distance = pass_distance;
    float best_gap_squared = LARGE_VALUE;
    int end_index = parker->first_unit_index + parker->unit_count;
    for (float distance = pass_distance; distance <= moving->route_length;
         distance += PARKED_ROUTE_STEP_METERS) {
        /* Measure the gap from the route point to the nearest slot. */
        V2 unused_direction;
        V2 route_point = get_route_point(moving, distance, &unused_direction);
        float gap_squared = LARGE_VALUE;
        for (int unit_index = parker->first_unit_index; unit_index < end_index; unit_index++) {
            float slot_gap_squared = length_squared(sub(units[unit_index].slot, route_point));
            gap_squared = fminf(gap_squared, slot_gap_squared);
        }

        /* Stop once the route no longer gets closer. */
        if (gap_squared >= best_gap_squared)
            break;
        best_distance = distance;
        best_gap_squared = gap_squared;
    }
    return best_distance;
}

/* ------------------------------------------------------------------------------------------------
 * `does_route_pass_squad` returns true when the route that `moving` still has to drive, from the
 * rear of its bodies to the end of the route, passes a member of `parked` where it stands, in the
 * sense of `find_route_pass_distance`.
 * --------------------------------------------------------------------------------------------- */
static bool does_route_pass_squad(const Squad *moving, const Squad *parked) {
    FormationShape shape = measure_shape(moving);
    float from = fmaxf(0, shape.rear_progress);
    float to = moving->route_length;
    float pass_distance = find_route_pass_distance(moving, shape.half_width, parked, false, from,
                                                   to);
    return pass_distance != NO_PASS_DISTANCE;
}

/* ------------------------------------------------------------------------------------------------
 * `replan_round_parked` lets every other moving squad whose route passes `parked`, a squad that
 * just arrived or settled, search its route again from where it stands, with `--parkreplan on` in
 * the coord mode of `--avoid`. Its route was searched before `parked` stood there, so it can lead
 * through the parked members. The new search, its formation layout and its lanes go around each
 * parked squad as a whole, through `is_replanning` in `stamp_parked`. The squad keeps its order
 * time, so its arrival still counts from its order, and its facing, so its formation still ends
 * as ordered. With `--keepspeed on`, its anchor also keeps its speed on the new route
 * (`finish_formation_set_up`), so that the members do not brake.
 * --------------------------------------------------------------------------------------------- */
static void replan_round_parked(const Squad *parked) {
    if (!is_park_replan_on || avoid_mode != AVOID_COORD)
        return;
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        Squad *other = &squads[squad_index];
        bool is_moving_other = other != parked && other->state == SQUAD_MOVING;
        if (!is_moving_other || !does_route_pass_squad(other, parked))
            continue;
        other->is_replanning = true;
        start_route_search(other);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `avoid_parking_ahead` lets a moving squad plan around a squad that will park on its way before
 * the squad gets there, with `--parkahead on` and `--parkreplan on` in the coord mode of
 * `--avoid`. Without it, the squad searches again only once the other squad has parked
 * (`replan_round_parked`), often right in front of it.
 * It looks for the slots of each other moving squad on its own route, from its front over the
 * distance that its anchor drives in `PARK_AHEAD_SECONDS` at its current speed, so that it does
 * not plan around a squad that parks far ahead, and a fast squad looks further than a slow one. A
 * squad that stands still looks nowhere. Where
 * it finds them, it compares two times at the cruise speeds: the other squad until its anchor
 * reaches the end of its route, and this squad until its front reaches the slots. When the other
 * squad gets there first, this squad searches its route again at once, around a disc at those
 * slots (`is_avoiding_slots`, see `stamp_parked`). It does so at most once for each other squad
 * until its next order.
 * The time of the other squad ends where its anchor reaches the end of its route, which is where
 * its slots are. With `--slotpoint off`, the time of this squad ends earlier, at "pass", where its
 * route first comes within reach of the slots. So a squad that follows the other squad at the
 * same speed seems to get there first. With `--slotpoint on`, the time of this squad ends at
 * "slot", where its route comes closest to the slots (`find_slot_distance`):
 *
 *        front              pass         slot
 *   ======>------------------|------------|----------> end of the route of this squad
 *                             \_ reach _   S
 *                                          S = a slot of the other squad, beside the route
 * --------------------------------------------------------------------------------------------- */
static void avoid_parking_ahead(Squad *squad) {
    bool is_used = is_park_ahead_on && is_park_replan_on && avoid_mode == AVOID_COORD;
    if (!is_used)
        return;

    /* Look from the front of the formation as far as the anchor drives in `PARK_AHEAD_SECONDS`. */
    FormationShape shape = measure_shape(squad);
    float front = squad->anchor_distance + shape.front_meters;
    float look_distance = PARK_AHEAD_SECONDS * squad->anchor_speed;
    float look_end = fminf(front + look_distance, squad->route_length);
    for (int other_index = 0; other_index < squad_count; other_index++) {
        /* Skip the squads that do not drive to their slots, and those it avoids already. */
        const Squad *other = &squads[other_index];
        bool is_parking_other = other != squad && other->state == SQUAD_MOVING;
        bool is_new_other = is_parking_other && !other->is_settled;
        if (!is_new_other || squad->is_avoiding_slots[other_index])
            continue;
        float pass_distance = find_route_pass_distance(squad, shape.half_width, other, true, front,
                                                       look_end);
        if (pass_distance == NO_PASS_DISTANCE)
            continue;

        /* Search again when the other squad parks before this squad gets to its slots. */
        float other_cruise_speed = measure_shape(other).cruise_speed;
        float other_remaining = other->route_length - other->anchor_distance;
        float other_seconds = other_remaining / other_cruise_speed;
        float arrival_distance = pass_distance;
        if (is_slot_point_on)
            arrival_distance = find_slot_distance(squad, other, pass_distance);
        float own_seconds = (arrival_distance - front) / shape.cruise_speed;
        if (other_seconds >= own_seconds)
            continue;
        squad->is_avoiding_slots[other_index] = true;
        squad->is_replanning = true;
        start_route_search(squad);
        return;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `is_circling_tightly` returns true when the unit drove its last tick on a circle smaller than
 * `TIGHT_CIRCLE_RADII` times its body radius. The radius of the circle is the speed over the turn
 * rate, so the test is `speed < TIGHT_CIRCLE_RADII * radius * |last_turn_rate|`. A unit that
 * drives a bend at speed draws a much larger circle: a heavy tank at its top speed and its full
 * turn rate draws one of about 17 m, against 12 m here.
 * --------------------------------------------------------------------------------------------- */
static bool is_circling_tightly(const Unit *unit) {
    float circle_speed = TIGHT_CIRCLE_RADII * unit->kind->radius * fabsf(unit->last_turn_rate);
    return len(unit->velocity) < circle_speed;
}

/* ------------------------------------------------------------------------------------------------
 * `update_settled` lets a moving squad count as parked before it arrives, with `--parkleeway on`
 * and `--parkreplan on` in the coord mode of `--avoid`.
 * Result: The squad `is_settled` while its anchor is within `ROUTE_END_EPSILON_METERS` (the
 *   distance from the end of the route that counts as at the end) of the end of the route and
 *   each member is parked. Here a member is parked when it is stopped, turns
 *   on the spot, or circles tightly, as when the members circle in small turns to their slots and
 *   final facing. With `--parkspeed on`, a member slower than `PARKED_SPEED_FRACTION` of its top
 *   speed is parked too, so that the other squads have a little more time to plan around the
 *   squad. Any other member, such as one that drives on, is not parked, so that a squad that
 *   still moves to its slots does not count as parked. The test of the anchor keeps squads that
 *   start, or that wait on their way, from counting.
 *   When the squad becomes settled, the moving squads whose routes pass it search again around
 *   it, through `replan_round_parked`.
 * Edge case: Turning on the spot is `is_spinning`, which `measure` in metrics.c sets after each
 *   tick, and circling tightly is `is_circling_tightly`, from the turn of the last tick. So this
 *   check sees the previous tick. A member that circles a little faster than the spin test
 *   allows still counts as parked.
 * --------------------------------------------------------------------------------------------- */
static void update_settled(Squad *squad) {
    /* Check the switches and the anchor. */
    bool is_used = is_park_leeway_on && is_park_replan_on && avoid_mode == AVOID_COORD;
    float end_distance = squad->route_length - ROUTE_END_EPSILON_METERS;
    bool is_at_route_end = squad->anchor_distance >= end_distance;
    if (!is_used || !is_at_route_end) {
        squad->is_settled = false;
        return;
    }

    /* Check the members, and search again around a squad that just settled. */
    bool are_members_parked = true;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *member = &units[unit_index];
        bool is_turning = member->is_spinning || is_circling_tightly(member);
        float parked_speed = PARKED_SPEED_FRACTION * member->kind->max_speed;
        bool is_crawling = is_park_speed_on && len(member->velocity) < parked_speed;
        if (!member->is_stopped && !is_turning && !is_crawling)
            are_members_parked = false;
    }
    bool has_just_settled = are_members_parked && !squad->is_settled;
    squad->is_settled = are_members_parked;
    if (has_just_settled)
        replan_round_parked(squad);
}

/* ------------------------------------------------------------------------------------------------
 * `do_segments_cross` returns true when the segment from `first_start` to `first_end` and the
 * segment from `second_start` to `second_end` cross. The ends of each segment must lie strictly
 * on opposite sides of the line of the other segment, so segments that only touch do not count.
 * --------------------------------------------------------------------------------------------- */
static bool do_segments_cross(V2 first_start, V2 first_end, V2 second_start, V2 second_end) {
    V2 first = sub(first_end, first_start);
    V2 second = sub(second_end, second_start);
    float second_start_side = det2(first, sub(second_start, first_start));
    float second_end_side = det2(first, sub(second_end, first_start));
    float first_start_side = det2(second, sub(first_start, second_start));
    float first_end_side = det2(second, sub(first_end, second_start));
    return second_start_side * second_end_side < 0 && first_start_side * first_end_side < 0;
}

/* ------------------------------------------------------------------------------------------------
 * `count_path_crossings` counts the pairs of members whose straight lines from `start_position`
 * to `slot` cross. The metrics report the count.
 * --------------------------------------------------------------------------------------------- */
static int count_path_crossings(const Squad *squad) {
    int crossing_count = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int first_index = squad->first_unit_index; first_index < end_index; first_index++) {
        const Unit *first = &units[first_index];
        for (int second_index = first_index + 1; second_index < end_index; second_index++) {
            const Unit *second = &units[second_index];
            crossing_count += do_segments_cross(first->start_position, first->slot,
                                                second->start_position, second->slot);
        }
    }
    return crossing_count;
}

/* ------------------------------------------------------------------------------------------------
 * `set_spread_reference` sets `spread_reference` for the spread metric: the largest distance of
 * the slot of a member from the center of `slots`, which holds x, y pairs, one per member.
 * --------------------------------------------------------------------------------------------- */
static void set_spread_reference(Squad *squad, const float *slots) {
    /* Find the center of the slots. */
    V2 slot_center = v2(0, 0);
    for (int slot_index = 0; slot_index < squad->unit_count; slot_index++)
        slot_center = add(slot_center, get_pair_point(slots, slot_index));
    slot_center = mul(slot_center, 1.0f / squad->unit_count);

    /* Take the largest distance of the slot of a member from it. */
    squad->spread_reference = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        float slot_spread = len(sub(unit->slot, slot_center));
        if (slot_spread > squad->spread_reference)
            squad->spread_reference = slot_spread;
    }
}

/* `saved_route[0..saved_route_count)` holds the squad route from just before the last call of
 * `store_found_route`, so that `fail_move` can put it back with `--oldroute on`. Only the main
 * thread stores routes, and `fail_move` runs right after the store, so one copy serves every
 * squad. */
static V2 saved_route[MAX_WAYPOINTS];
static int saved_route_count;

/* ------------------------------------------------------------------------------------------------
 * `store_found_route` stores the squad route: the position of the lead, then the found path. It
 * frees the search, so that the next order can start one.
 * Result: It returns the number of path points, which it also puts into `route`, without the
 *   position of the lead.
 * --------------------------------------------------------------------------------------------- */
static int store_found_route(Squad *squad, V2 *route) {
    /* Keep the old route for `fail_move`. */
    saved_route_count = squad->route_count;
    memcpy(saved_route, squad->route, (size_t)squad->route_count * sizeof(V2));

    /* Store the new route. */
    int point_count = gc_get_path(&squad->search, path_points, MAX_WAYPOINTS);
    for (int point_index = 0; point_index < point_count; point_index++)
        route[point_index] = get_pair_point(path_points, point_index);
    squad->route[0] = squad->start_position;
    size_t path_bytes = (size_t)point_count * sizeof(V2);
    memcpy(&squad->route[1], &route[0], path_bytes);
    squad->route_count = point_count + 1;
    gc_free_request(&squad->search);
    gc_init_request(&squad->search);
    return point_count;
}

/* ------------------------------------------------------------------------------------------------
 * `lay_out_slots` lays out the slots at the end of the route, along the facing of the formation,
 * as the formation setter (0x507ee0) does with the radius and mover class of the first member.
 * In formation mode the layout treats parked units as cliffs too. When the layout works, it
 * toggles `is_flipped` where the new facing points against the previous one, as the formation
 * setter does, so that SlotFor counts the slots from the other end.
 * Inputs: `route[0..point_count)` is the found path, without the position of the lead.
 * Result: It returns true when the slots are placed. `slots` then holds x, y pairs, one per
 *   slot, and `facing` the facing of the formation with length 1, or zero for a facing too short
 *   to have a direction.
 * --------------------------------------------------------------------------------------------- */
static bool lay_out_slots(Squad *squad, const V2 *route, int point_count, float *slots,
                          V2 *facing) {
    /* Find the facing. */
    *facing = squad->facing;
    if (length_squared(*facing) <= MIN_FACING_LENGTH_SQUARED)
        *facing = v2(0, 0);
    else
        *facing = mul(*facing, 1 / len(*facing));

    /* Place the slots with the parked units stamped. */
    const Unit *first = &units[squad->first_unit_index];
    stamp_parked(squad, squad->start_position, squad->destination);
    bool is_placed = false;
    if (point_count > 0) {
        V2 end = route[point_count - 1];
        float radius = first->kind->radius;
        int mover_class = first->kind->mover_class;
        is_placed = gc_place_formation(end.x, end.y, facing->x, facing->y, squad->unit_count,
                                       radius, mover_class, squad->formation_type, slots);
    }
    unstamp_parked();
    if (!is_placed)
        return false;

    /* Count the slots from the other end after a turnaround. */
    if (dot(*facing, squad->last_facing) < 0)
        squad->is_flipped = !squad->is_flipped;
    squad->last_facing = *facing;
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `fail_move` ends a move whose search or slot layout failed. It denies the move
 * (`SQUAD_DENIED`), as the game does, unless `--oldroute` is on and the squad searched again
 * around parked squads (`is_replanning`). Such a squad goes on along its old route to its old
 * slots instead, so that a failed new search never stops a squad that is already on its way. Its
 * lanes, places and anchor are still those of the old route, because only a finished set-up
 * replaces them.
 * Inputs: `has_stored_route` is true when `store_found_route` already replaced the route. The
 *   old route then comes back from `saved_route`. Otherwise the search failed, and `fail_move`
 *   frees it, as `store_found_route` does.
 * --------------------------------------------------------------------------------------------- */
static void fail_move(Squad *squad, bool has_stored_route) {
    if (!is_old_route_on || !squad->is_replanning) {
        squad->state = SQUAD_DENIED;
        return;
    }
    if (has_stored_route) {
        memcpy(squad->route, saved_route, (size_t)saved_route_count * sizeof(V2));
        squad->route_count = saved_route_count;
    } else {
        gc_free_request(&squad->search);
        gc_init_request(&squad->search);
    }
    squad->state = SQUAD_MOVING;
}

/* ------------------------------------------------------------------------------------------------
 * `start_squad_move` prepares the metrics of the move and starts it. It keeps the time of the
 * first start after an order, for the responsiveness (column "start" of metrics.c); a squad that
 * searches again around parked squads starts again later, but its order is still the same one.
 * --------------------------------------------------------------------------------------------- */
static void start_squad_move(Squad *squad, const float *slots) {
    set_spread_reference(squad, slots);
    squad->crossing_count += count_path_crossings(squad);
    squad->state = SQUAD_MOVING;
    squad->straggler_seconds = NO_STRAGGLER_TIMER;
    if (squad->start_time_seconds == NOT_STARTED_SECONDS)
        squad->start_time_seconds = now_seconds;
}

/* ------------------------------------------------------------------------------------------------
 * `set_up_move` follows Squad::SetupCalculatedMove (0x50a7d0), once the search of the squad found
 * a path. It stores the squad route, lays out the slots of the formation at the end of the route,
 * and gives each member its move. When the path is empty or the layout fails, it ends the move
 * through `fail_move`.
 * --------------------------------------------------------------------------------------------- */
static void set_up_move(Squad *squad) {
    /* Store the route, and lay out the slots. `route[0..point_count)` holds the path alone. */
    static V2 route[MAX_WAYPOINTS];
    float slots[2 * MAX_UNITS];
    V2 facing;
    int point_count = store_found_route(squad, route);
    if (!lay_out_slots(squad, route, point_count, slots, &facing)) {
        fail_move(squad, true);
        return;
    }

    /* Give each member its move. In formation mode, formation following sets up the whole squad,
     * with parked units stamped as cliffs. In game mode, SlotFor (0x508030) gives member k slot
     * k, counted from the end while `is_flipped` is true. The slot then counts as reached within
     * the radius of the member, but never within less than `MIN_LAST_REACH_METERS`, and every
     * other waypoint counts as reached within `MID_REACH_METERS`. The game does the same in
     * SetReachLast, which stores at least that reach, and SetReachMid, which ignores its argument
     * and always sets `MID_REACH_METERS`. */
    if (is_formation_mode()) {
        stamp_parked(squad, squad->start_position, squad->destination);
        set_up_formation(squad, slots, facing);
        unstamp_parked();
    } else {
        for (int member_index = 0; member_index < squad->unit_count; member_index++) {
            int unit_index = squad->first_unit_index + member_index;
            Unit *unit = &units[unit_index];
            int slot_index = member_index;
            if (squad->is_flipped)
                slot_index = squad->unit_count - 1 - member_index;
            unit->slot = get_pair_point(slots, slot_index);
            float radius = unit->kind->radius;
            unit->reach_last = radius < MIN_LAST_REACH_METERS ? MIN_LAST_REACH_METERS : radius;
            unit->reach_mid = MID_REACH_METERS;
            set_move(unit, route, point_count, facing, unit->slot);
        }
    }
    start_squad_move(squad, slots);
}

/* ------------------------------------------------------------------------------------------------
 * `is_other_squad_setting_up` returns true when a squad other than `squad` runs a sliced set-up
 * now, past its wait.
 * --------------------------------------------------------------------------------------------- */
static bool is_other_squad_setting_up(const Squad *squad) {
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        const Squad *other = &squads[squad_index];
        if (other != squad && other->set_up_phase > SET_UP_WAITING)
            return true;
    }
    return false;
}

/* ------------------------------------------------------------------------------------------------
 * `begin_sliced_set_up` starts the sliced set-up of a squad whose search found its path, unless
 * another squad sets up now; then the squad waits a tick and tries again. Squads take their turn
 * in index order, because `update_squad` runs in that order. This first part stores the route,
 * lays out the slots, and runs `begin_formation_set_up`, which ends with the start of the
 * smoothing.
 * --------------------------------------------------------------------------------------------- */
static void begin_sliced_set_up(Squad *squad) {
    static V2 route[MAX_WAYPOINTS];
    if (is_other_squad_setting_up(squad))
        return;

    /* Store the route and lay out the slots, as `set_up_move` does. */
    int point_count = store_found_route(squad, route);
    if (!lay_out_slots(squad, route, point_count, squad->set_up_slots, &squad->set_up_facing)) {
        squad->set_up_phase = SET_UP_NONE;
        fail_move(squad, true);
        return;
    }

    /* Fit the route, and start the smoothing. */
    stamp_parked(squad, squad->start_position, squad->destination);
    bool is_smoothing = begin_formation_set_up(squad, squad->set_up_slots, squad->set_up_facing);
    unstamp_parked();
    squad->set_up_phase = is_smoothing ? SET_UP_SMOOTHING : SET_UP_FINISHING;
}

/* Threaded set-ups (`--setupthreads on`, with `--setupslice off` and `--smooth on` in the
 * formation modes): The smoothing takes most of the time of a route set-up, and it reads only the
 * squad and the grid. So worker threads (workers.c) smooth the routes of the squads whose search
 * ended in this tick, all at the same time, while the main thread goes on with the next squads.
 * The other parts stay on the main thread: those before the smoothing call code of the game, and
 * the finish writes the members, which the main thread reads. The finish and the start of each
 * move then run in the order of the squads, as without threads.
 * Exactness: The result is the same as without threads. A squad whose smoothing waits keeps
 * `SQUAD_SEARCHING`, so it still counts as busy, as a moving squad does, and the searches of the
 * next squads stamp the same parked units. A moving squad can read the routes of other squads,
 * so the smoothings end before a moving squad runs its tick (`update_squad`).
 * Timeline: The sketch shows one tick in which the searches of squads 0, 1 and 2 end, as after an
 * order to three squads. "prep k" is the search of squad k and the parts before its smoothing,
 * "smooth k" is its smoothing, and "finish k" its finish and the start of its move. A free worker
 * takes each smoothing at once. When no worker is free, the main thread runs the smoothing itself
 * while it waits. The workers start at the first hand-over of the program, sleep between jobs and
 * stop at the end of the program (workers.c).
 *
 *     main thread:  prep 0 | prep 1 | prep 2 | wait ...... | finish 0, 1, 2 | units move
 *     worker A:              smooth 0 ------------>
 *     worker B:                       smooth 1 -------->
 *     worker C:                                smooth 2 -->
 *
 * A `ThreadedSetUp` holds the set-up of one squad: `grid` is a copy of the grid with the parked
 * units stamped for that squad, on the heap, and `stamp_box` the box around those stamps.
 * `is_waiting` is true from the hand-over of the smoothing until the set-up is finished. Entry k
 * of `threaded_set_ups` belongs to squad k, and `waiting_set_up_count` entries are waiting. */
typedef struct {
    Squad *squad;
    uint8_t *grid;
    StampBox stamp_box;
    bool is_waiting;
} ThreadedSetUp;

static ThreadedSetUp threaded_set_ups[MAX_SQUADS];
static int waiting_set_up_count;

/* ------------------------------------------------------------------------------------------------
 * `smooth_in_thread` is the job of a worker thread, which can also run on the main thread inside
 * `wait_for_jobs`. It makes the cell reads of the thread see the grid copy and the stamp box of
 * the set-up, begins the smoothing, and runs it to its end. Then it gives the thread its own grid
 * and box back.
 * --------------------------------------------------------------------------------------------- */
static void smooth_in_thread(void *argument) {
    ThreadedSetUp *set_up = argument;
    Squad *squad = set_up->squad;
    StampBox own_stamp_box = stamp_box;
    gc_use_grid(set_up->grid);
    stamp_box = set_up->stamp_box;
    begin_formation_smoothing(squad, squad->set_up_slots, squad->set_up_facing);
    run_smoothing(INT_MAX);
    stamp_box = own_stamp_box;
    gc_use_grid(NULL);
}

/* ------------------------------------------------------------------------------------------------
 * `begin_threaded_set_up` runs the first parts of the set-up of a squad whose search found its
 * path, on the main thread, and hands the smoothing to the worker threads. It stores the route and
 * lays out the slots, as `set_up_move` does. Then it fits the route with the parked units stamped
 * and copies the stamped grid for the smoothing.
 * --------------------------------------------------------------------------------------------- */
static void begin_threaded_set_up(Squad *squad) {
    /* Store the route, and lay out the slots. */
    static V2 route[MAX_WAYPOINTS];
    int point_count = store_found_route(squad, route);
    if (!lay_out_slots(squad, route, point_count, squad->set_up_slots, &squad->set_up_facing)) {
        fail_move(squad, true);
        return;
    }

    /* Fit the route, and copy the grid and the box of the stamps. */
    ThreadedSetUp *set_up = &threaded_set_ups[squad->squad_index];
    if (!set_up->grid)
        set_up->grid = malloc(GC_GRID_BYTES);
    stamp_parked(squad, squad->start_position, squad->destination);
    prepare_formation_route(squad, squad->set_up_slots, squad->set_up_facing);
    gc_copy_grid(set_up->grid);
    set_up->stamp_box = stamp_box;
    unstamp_parked();

    /* Hand the smoothing over. */
    set_up->squad = squad;
    set_up->is_waiting = true;
    waiting_set_up_count++;
    submit_job(smooth_in_thread, set_up);
}

/* ------------------------------------------------------------------------------------------------
 * `finish_threaded_set_ups` stamps the parked units again for each finish, which stamps the same
 * cells as for the first parts: no unit has moved since, and every squad that took part still
 * counts as busy.
 * --------------------------------------------------------------------------------------------- */
void finish_threaded_set_ups(void) {
    if (waiting_set_up_count == 0)
        return;
    wait_for_jobs();
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        ThreadedSetUp *set_up = &threaded_set_ups[squad_index];
        if (!set_up->is_waiting)
            continue;
        Squad *squad = set_up->squad;
        stamp_parked(squad, squad->start_position, squad->destination);
        finish_formation_set_up(squad, squad->set_up_slots, squad->set_up_facing);
        unstamp_parked();
        start_squad_move(squad, squad->set_up_slots);
        set_up->is_waiting = false;
    }
    waiting_set_up_count = 0;
}

/* With `--setupchain on`, the sliced set-ups may still do this much work in this tick, in room
 * tests. `reset_set_up_work` refills it at the start of each tick. */
static int set_up_work_left;

/* ------------------------------------------------------------------------------------------------
 * `reset_set_up_work` refills the budget with `SET_UP_WORK_PER_TICK`.
 * --------------------------------------------------------------------------------------------- */
void reset_set_up_work(void) {
    set_up_work_left = SET_UP_WORK_PER_TICK;
}

/* ------------------------------------------------------------------------------------------------
 * `run_set_up_part` runs the next part of the sliced set-up of `squad`, with the parked units
 * stamped: the begin, a part of the smoothing, or the finish. The finish lays out the lanes and
 * places and starts the move, as `set_up_move` does.
 * Inputs: `max_room_tests` limits the room tests of a smoothing part, as in `run_smoothing`.
 * Result: It returns the work of the part, in room tests. For the smoothing, this is the count of
 *   its room tests. For the begin and the finish, it is the estimate from the route points and
 *   the lane samples. A squad that must wait for its turn stays at `SET_UP_WAITING`, and its part
 *   does no work.
 * --------------------------------------------------------------------------------------------- */
static int run_set_up_part(Squad *squad, int max_room_tests) {
    /* Begin, while the squad waits. */
    if (squad->set_up_phase == SET_UP_WAITING) {
        begin_sliced_set_up(squad);
        if (squad->set_up_phase == SET_UP_WAITING)
            return 0;
        return SET_UP_WORK_PER_ROUTE_POINT * squad->route_count;
    }

    /* Smooth. */
    stamp_parked(squad, squad->start_position, squad->destination);
    if (squad->set_up_phase == SET_UP_SMOOTHING) {
        int first_test_count = get_room_test_count();
        if (run_smoothing(max_room_tests))
            squad->set_up_phase = SET_UP_FINISHING;
        unstamp_parked();
        return get_room_test_count() - first_test_count;
    }

    /* Finish. With `--lanes off`, the finish lays out no lane samples. */
    finish_formation_set_up(squad, squad->set_up_slots, squad->set_up_facing);
    unstamp_parked();
    squad->set_up_phase = SET_UP_NONE;
    start_squad_move(squad, squad->set_up_slots);
    int lane_sample_count = are_lanes_on ? squad->lane_count : 0;
    return SET_UP_WORK_PER_LANE_SAMPLE * lane_sample_count;
}

/* ------------------------------------------------------------------------------------------------
 * `continue_sliced_set_up` runs the sliced set-up of `squad` for this tick.
 * With `--setupchain off`, it runs one part, and a smoothing part runs
 * `SET_UP_ROOM_TESTS_PER_TICK` room tests.
 * With `--setupchain on`, it runs one part after the other while `set_up_work_left` lasts. A
 * short set-up then begins, smooths and finishes in one tick, and the next squad can begin in the
 * same tick. A part starts only while work is left, and it always runs to its end, so the last
 * part of a tick can go past the budget.
 * --------------------------------------------------------------------------------------------- */
static void continue_sliced_set_up(Squad *squad) {
    if (!is_set_up_chain_on) {
        run_set_up_part(squad, SET_UP_ROOM_TESTS_PER_TICK);
        return;
    }
    while (squad->set_up_phase != SET_UP_NONE && set_up_work_left > 0) {
        bool was_waiting = squad->set_up_phase == SET_UP_WAITING;
        set_up_work_left -= run_set_up_part(squad, set_up_work_left);
        bool is_still_waiting = was_waiting && squad->set_up_phase == SET_UP_WAITING;
        if (is_still_waiting)
            return;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `update_search` follows Squad::CalculatingMoveOrderLogic (0x50a6a0), which runs the search of
 * the squad for `SQUAD_SEARCH_BUDGET` expansions in each tick. When the quiet search fails, it
 * starts the active search (the backward search of the game), as the game does with the mover
 * class and clearance of the first member, not of the lead. When the active search fails too, the
 * move is denied (`fail_move`), and the game says "Movement denied". When a search finds a path,
 * `update_search` sets up the move.
 * With `--fullsearch on` in the formation modes, each search call gets `FULL_SEARCH_BUDGET`
 * expansions, and the active search runs in the same tick as the failed quiet search. So the
 * search of a squad always ends in the tick where it starts.
 * --------------------------------------------------------------------------------------------- */
static void update_search(Squad *squad) {
    /* Go on with a sliced set-up once the search found its path. */
    if (squad->set_up_phase != SET_UP_NONE) {
        continue_sliced_set_up(squad);
        return;
    }

    /* Run the search, and start the active one when the quiet one failed. */
    bool is_full_search = is_full_search_on && is_formation_mode();
    int budget = is_full_search ? FULL_SEARCH_BUDGET : SQUAD_SEARCH_BUDGET;
    stamp_parked(squad, squad->start_position, squad->destination);
    int search_result = gc_update_search(&squad->search, budget);
    bool has_failed = search_result == GC_SEARCH_FAILED;
    bool must_start_active = has_failed && !squad->is_active_search;
    if (must_start_active) {
        const Unit *first = &units[squad->first_unit_index];
        V2 start = squad->start_position;
        V2 end = squad->destination;
        int mover_class = first->kind->mover_class;
        float clearance = get_search_clearance(first);
        gc_start_search(&squad->search, GC_SEARCH_ACTIVE, start.x, start.y, end.x, end.y,
                        mover_class, clearance, SQUAD_SEARCH_TOLERANCE_METERS);
        squad->is_active_search = true;
    }
    if (must_start_active && is_full_search)
        search_result = gc_update_search(&squad->search, budget);
    unstamp_parked();

    /* Act on the result. With `--setupslice on` in the formation modes, the set-up runs over
     * the next ticks instead of at once. A failed quiet search is no denial while its active
     * search still has to run in the next ticks. */
    bool is_sliced = is_set_up_slice_on && is_formation_mode();
    bool can_thread = is_formation_mode() && is_smooth_on && !is_sliced;
    bool is_threaded = is_set_up_threads_on && can_thread;
    bool has_active_to_run = must_start_active && !is_full_search;
    bool is_denied = search_result == GC_SEARCH_FAILED && !has_active_to_run;
    if (search_result == GC_SEARCH_FOUND && is_sliced) {
        squad->set_up_phase = SET_UP_WAITING;
        continue_sliced_set_up(squad);
    } else if (search_result == GC_SEARCH_FOUND && is_threaded)
        begin_threaded_set_up(squad);
    else if (search_result == GC_SEARCH_FOUND)
        set_up_move(squad);
    else if (is_denied)
        fail_move(squad, false);
}

/* ------------------------------------------------------------------------------------------------
 * `has_squad_arrived` follows the arrival check of the moving state (0x507070) for a squad
 * without queued orders. It also runs the straggler timer, which starts when at most one member
 * of a squad of more than `STRAGGLER_MIN_MEMBERS` still moves.
 * Result: It returns true once every member stopped, or once the straggler timer ran out.
 * --------------------------------------------------------------------------------------------- */
static bool has_squad_arrived(Squad *squad) {
    /* Count the stopped members. */
    int stopped_count = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++)
        stopped_count += units[unit_index].is_stopped;
    bool has_all_stopped = stopped_count == squad->unit_count;

    /* Count a running straggler timer down. */
    if (squad->straggler_seconds >= 0) {
        squad->straggler_seconds -= TICK_SECONDS;
        return squad->straggler_seconds <= 0 || has_all_stopped;
    }

    /* Start the timer when the squad has more than `STRAGGLER_MIN_MEMBERS` members and at most
     * one of them still moves. */
    int moving_count = squad->unit_count - stopped_count;
    bool must_start_timer = squad->unit_count > STRAGGLER_MIN_MEMBERS && moving_count <= 1;
    if (must_start_timer)
        squad->straggler_seconds = STRAGGLER_WAIT_SECONDS;
    return has_all_stopped;
}

/* ------------------------------------------------------------------------------------------------
 * `update_squad` follows the search state (0x50a6a0) and the moving state (0x507070) of
 * Squad::Update (0x504950). It dispatches on the state of the squad. It moves the anchor before
 * the arrival check, so that the check sees the new position of the anchor, and it runs the
 * settle test only while the squad has not arrived.
 * --------------------------------------------------------------------------------------------- */
void update_squad(Squad *squad) {
    /* Run the search while the squad searches. Do nothing else unless it moves. */
    if (squad->state == SQUAD_SEARCHING) {
        update_search(squad);
        return;
    }
    if (squad->state != SQUAD_MOVING)
        return;

    /* End the threaded set-ups of this tick first, because a moving squad can read the routes of
     * other squads, for example when its arrival makes them search again. */
    finish_threaded_set_ups();

    /* In formation mode, move the anchor, the virtual leader on the squad route. Then check
     * arrival. */
    if (is_formation_mode())
        advance_anchor(squad);
    if (!has_squad_arrived(squad)) {
        update_settled(squad);
        avoid_parking_ahead(squad);
        return;
    }

    /* Stop every member where it stands, as Squad::Stop (0x504bd0) does with a last-leg reach
     * of `STOP_REACH_METERS`. */
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        Unit *unit = &units[unit_index];
        unit->reach_last = STOP_REACH_METERS;
        stop_at(unit, unit->position);
    }
    squad->state = SQUAD_ARRIVED;
    squad->arrive_time_seconds = now_seconds;
    squad->is_settled = false;
    replan_round_parked(squad);
}
