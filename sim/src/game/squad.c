// Squads: orders, the squad path search, the set-up of a move, and arrival. This part rebuilds
// the base game (docs/PATHFINDING.md): `order_move` starts the forward search of the squad from the
// front unit, the squad member closest to the destination, as `Squad::ProcessOrder` (0x50ac20)
// does. The search runs `SQUAD_SEARCH_BUDGET` expansions per tick, as
// `Squad::CalculatingMoveOrderLogic` (0x50a6a0) does, and the found route becomes a move, as in
// `Squad::SetupCalculatedMove` (0x50a7d0). The arrival test and the straggler timer follow the
// moving state 0x507070.
//
// In the formation modes, the set-up of a move is the route smoothing set-up (formation.c). It
// runs in one call, spread over ticks under a work budget (`--setupaccumulate`), or with its
// smoothing on worker threads (`--setupthreads`). The searches of a squad treat parked squads as
// cliffs (formation/stamping.c).
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "sim.h"
#include "formation.h"

#define SQUAD_SEARCH_BUDGET 60                   // expansions per tick of a squad search
#define SQUAD_SEARCH_TOLERANCE_METERS 0.0f        // the search ends on the destination
#define MIN_FACING_LENGTH_SQUARED 1e-5f           // shorter facings count as none
#define MIN_LAST_REACH_METERS 1.0f                // smallest reach of the last waypoint
#define STRAGGLER_MIN_MEMBERS 2                   // larger squads use the straggler timer
#define STRAGGLER_WAIT_SECONDS 20.0f              // start value of the straggler timer
#define NO_STRAGGLER_TIMER -1.0f                  // the straggler timer does not run
#define FULL_SEARCH_BUDGET 500                    // expansions per tick with `--fullsearch`
#define SET_UP_WORK_PER_TICK 4000                 // work budget per tick, in clearance tests
#define SET_UP_WORK_PER_ROUTE_CORNER 60           // work of the first part, per route corner
#define SET_UP_WORK_PER_CROSS_SECTION 2           // work of the last part, per cross-section

// Phases of an accumulated set-up (`--setupaccumulate`), in the order in which they run.
enum {
    SET_UP_NONE = 0,
    SET_UP_WAITING,
    SET_UP_SMOOTHING,
    SET_UP_FINISHING,
};


// A route smoothing set-up that runs its smoothing on a worker thread: the squad, its own grid
// copy with the stamps of this squad, the stamp box of those stamps, and whether its finish waits.
typedef struct {
    Squad *squad;
    uint8_t *grid;
    StampBox stamp_box;
    bool is_waiting;
} ThreadedSetUp;

// Route of a squad before its last new route, which a failed re-route restores (`--parkrobust`).
static V2 saved_route[MAX_WAYPOINTS];
static int saved_route_count;

static ThreadedSetUp threaded_set_ups[MAX_SQUADS];
static int waiting_set_up_count;

// Work left in the current tick for accumulated set-ups, in clearance tests.
static int set_up_work_left;


// `find_closest_member` returns the unit index of the squad member closest to `point`.
static int find_closest_member(const Squad *squad, V2 point) {
    int best_index = squad->first_unit_index;
    float best_squared = length_squared(sub(units[best_index].position, point));
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        float d_squared = length_squared(sub(units[unit_index].position, point));
        if (d_squared < best_squared) {
            best_index = unit_index;
            best_squared = d_squared;
        }
    }
    return best_index;
}


// `start_route_search` starts the forward search of a squad from its front unit to its
// destination, with the terrain class and the corner gap of the front unit. The squad then
// searches, and an accumulated set-up of an earlier search ends.
void start_route_search(Squad *squad) {
    int front_unit_index = find_closest_member(squad, squad->destination);
    const Unit *front_unit = &units[front_unit_index];
    squad->start_position = front_unit->position;
    squad->is_backward_search = false;

    V2 start = squad->start_position;
    V2 end = squad->destination;
    int terrain_class = front_unit->unit_type->terrain_class;
    float corner_gap = get_corner_gap(front_unit);
    stamp_parked(squad, start, end);
    gc_start_search(&squad->search, GC_SEARCH_FORWARD, start.x, start.y, end.x, end.y,
                    terrain_class, corner_gap, SQUAD_SEARCH_TOLERANCE_METERS);
    unstamp_parked();
    squad->state = SQUAD_SEARCHING;
    squad->set_up_phase = SET_UP_NONE;
}


// `order_move` orders a squad to `destination`: it clears the re-route state and starts the route
// search. The facing points from the front unit to the destination.
void order_move(Squad *squad, V2 destination) {
    squad->destination = destination;
    squad->is_rerouting = false;
    squad->is_settled = false;
    memset(squad->is_avoiding_parking, 0, sizeof squad->is_avoiding_parking);
    start_route_search(squad);
    squad->facing = sub(destination, squad->start_position);
    squad->order_time_seconds = now_seconds;
    squad->start_time_seconds = NOT_STARTED_SECONDS;
}


// `do_segments_cross` tells whether two segments cross each other at a point inside both. The two
// ends of each segment must lie strictly on different sides of the other segment.
static bool do_segments_cross(V2 first_start, V2 first_end, V2 second_start, V2 second_end) {
    V2 first = sub(first_end, first_start);
    V2 second = sub(second_end, second_start);
    float second_start_side = det2(first, sub(second_start, first_start));
    float second_end_side = det2(first, sub(second_end, first_start));
    float first_start_side = det2(second, sub(first_start, second_start));
    float first_end_side = det2(second, sub(first_end, second_start));
    return second_start_side * second_end_side < 0 && first_start_side * first_end_side < 0;
}


// `count_path_crossings` counts the pairs of squad members whose straight lines from their start
// positions to their final parking positions cross, for the metrics.
static int count_path_crossings(const Squad *squad) {
    int count = 0;        // number of crossings
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int first_index = squad->first_unit_index; first_index < end_index; first_index++) {
        const Unit *first = &units[first_index];
        for (int second_index = first_index + 1; second_index < end_index; second_index++) {
            const Unit *second = &units[second_index];
            count += do_segments_cross(first->start_position, first->parking_position,
                                       second->start_position, second->parking_position);
        }
    }
    return count;
}


// `set_spread_reference` stores the spread reference of a squad for the metrics: the largest
// distance of the final parking position of a squad member from the center of all of them.
static void set_spread_reference(Squad *squad, const float *parking_positions) {
    V2 center = v2(0, 0);        // center of the final parking positions
    for (int parking_index = 0; parking_index < squad->unit_count; parking_index++)
        center = add(center, get_pair_point(parking_positions, parking_index));
    center = mul(center, 1.0f / squad->unit_count);

    squad->spread_reference = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        float spread = len(sub(unit->parking_position, center));
        if (spread > squad->spread_reference)
            squad->spread_reference = spread;
    }
}


// `store_found_route` stores the path of the finished search of a squad in `route`, and as the
// route of the squad from its start position. It saves the old route first, and it returns the
// number of path points.
static int store_found_route(Squad *squad, V2 *route) {
    saved_route_count = squad->route_count;
    memcpy(saved_route, squad->route, (size_t)squad->route_count * sizeof(V2));

    // The search request is ready for the next search afterward.
    int count = gc_get_path(&squad->search, path_points, MAX_WAYPOINTS);        // path points
    for (int point_index = 0; point_index < count; point_index++)
        route[point_index] = get_pair_point(path_points, point_index);
    squad->route[0] = squad->start_position;
    size_t path_bytes = (size_t)count * sizeof(V2);
    memcpy(&squad->route[1], &route[0], path_bytes);
    squad->route_count = count + 1;
    gc_free_request(&squad->search);
    gc_init_request(&squad->search);
    return count;
}


// `lay_out_parking_positions` lays out the final parking positions of a squad at the end of
// `route` with the formation layout of the game, facing the normalized facing of the squad. It
// returns false when the layout fails.
static bool lay_out_parking_positions(Squad *squad, const V2 *route, int point_count,
                                      float *parking_positions, V2 *facing) {
    // A facing shorter than `MIN_FACING_LENGTH_SQUARED` becomes zero, as in the game.
    *facing = squad->facing;
    if (length_squared(*facing) <= MIN_FACING_LENGTH_SQUARED)
        *facing = v2(0, 0);
    else
        *facing = mul(*facing, 1 / len(*facing));

    // The layout uses the radius and the terrain class of the first squad member. With
    // `--gamelayout`, it ignores the parked squads, as the game lays out the final parking
    // positions of the mod without the units; without it, they count as cliffs.
    const Unit *first = &units[squad->first_unit_index];
    if (!is_game_layout_on)
        stamp_parked(squad, squad->start_position, squad->destination);
    bool is_laid_out = false;
    if (point_count > 0) {
        V2 end = route[point_count - 1];
        float radius = first->unit_type->radius;
        int terrain_class = first->unit_type->terrain_class;
        is_laid_out = gc_lay_out_formation(end.x, end.y, facing->x, facing->y, squad->unit_count,
                                           radius, terrain_class, squad->formation_type,
                                           parking_positions);
    }
    unstamp_parked();
    if (!is_laid_out)
        return false;
    if (is_whole_parked_on && is_formation_mode())
        shift_parking_positions(squad, parking_positions);
    if (is_free_parking_on && is_formation_mode())
        free_parking_positions(squad, parking_positions);

    // As the formation setter of the game (0x507ee0) does, a facing against the last one flips
    // the order of the final parking positions, so the squad members keep their sides.
    if (dot(*facing, squad->last_facing) < 0)
        squad->is_flipped = !squad->is_flipped;
    squad->last_facing = *facing;
    return true;
}


// `fail_move` ends a move whose search or layout failed: the game denies it. With `--parkrobust`,
// a re-routing squad keeps moving on its old route instead, which `has_stored_route` restores.
static void fail_move(Squad *squad, bool has_stored_route) {
    if (!is_park_robust_on || !squad->is_rerouting) {
        squad->state = SQUAD_DENIED;
        return;
    }

    // Without a stored route, the search request is reset for the next search.
    if (has_stored_route) {
        memcpy(squad->route, saved_route, (size_t)saved_route_count * sizeof(V2));
        squad->route_count = saved_route_count;
    } else {
        gc_free_request(&squad->search);
        gc_init_request(&squad->search);
    }
    squad->state = SQUAD_MOVING;
}


// `start_squad_move` starts the move of a squad after its set-up: the metrics references, the
// moving state, and the start time of its first move after the order.
static void start_squad_move(Squad *squad, const float *parking_positions) {
    set_spread_reference(squad, parking_positions);
    squad->crossing_count += count_path_crossings(squad);
    squad->state = SQUAD_MOVING;
    squad->straggler_seconds = NO_STRAGGLER_TIMER;
    if (squad->start_time_seconds == NOT_STARTED_SECONDS)
        squad->start_time_seconds = now_seconds;
}


// `set_up_move` sets up the move of a squad in one call after its search found the route. In the
// formation modes, this is the whole route smoothing set-up; in the original game mode, each squad
// member gets the route as in `SetupCalculatedMove`.
static void set_up_move(Squad *squad) {
    static V2 route[MAX_WAYPOINTS];
    float parking_positions[2 * MAX_UNITS];
    V2 facing;
    int count = store_found_route(squad, route);        // number of path points
    if (!lay_out_parking_positions(squad, route, count, parking_positions, &facing)) {
        fail_move(squad, true);
        return;
    }

    // In the original game mode, the final parking position of a squad member is the one with its
    // own index, counted from the end when the order is flipped (*SlotFor*, 0x508030).
    if (is_formation_mode()) {
        stamp_parked(squad, squad->start_position, squad->destination);
        set_up_formation(squad, parking_positions, facing);
        unstamp_parked();
    } else {
        for (int member_index = 0; member_index < squad->unit_count; member_index++) {
            int unit_index = squad->first_unit_index + member_index;
            Unit *unit = &units[unit_index];
            int parking_index = member_index;
            if (squad->is_flipped)
                parking_index = squad->unit_count - 1 - member_index;
            unit->parking_position = get_pair_point(parking_positions, parking_index);
            float radius = unit->unit_type->radius;
            unit->reach_last = radius < MIN_LAST_REACH_METERS ? MIN_LAST_REACH_METERS : radius;
            unit->reach_mid = MID_REACH_METERS;
            set_move(unit, route, count, facing, unit->parking_position);
        }
    }
    start_squad_move(squad, parking_positions);
}


// `is_other_squad_setting_up` tells whether another squad has an accumulated set-up past its
// waiting phase.
static bool is_other_squad_setting_up(const Squad *squad) {
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        const Squad *other = &squads[squad_index];
        if (other != squad && other->set_up_phase > SET_UP_WAITING)
            return true;
    }
    return false;
}


// `begin_accumulated_set_up` runs the first part of an accumulated set-up: the route, the final
// parking positions, the route preparation and the start of the smoothing. Only one squad at a
// time runs a set-up past its waiting phase, so a squad waits while another one does.
static void begin_accumulated_set_up(Squad *squad) {
    static V2 route[MAX_WAYPOINTS];
    if (is_other_squad_setting_up(squad))
        return;

    int count = store_found_route(squad, route);        // number of path points
    if (!lay_out_parking_positions(squad, route, count, squad->set_up_parking_positions,
                                   &squad->set_up_facing)) {
        squad->set_up_phase = SET_UP_NONE;
        fail_move(squad, true);
        return;
    }

    // Without `--smooth`, the set-up goes straight to its finish.
    stamp_parked(squad, squad->start_position, squad->destination);
    bool is_smoothing = begin_formation_set_up(squad, squad->set_up_parking_positions,
                                               squad->set_up_facing);
    unstamp_parked();
    squad->set_up_phase = is_smoothing ? SET_UP_SMOOTHING : SET_UP_FINISHING;
}


// `smooth_in_thread` runs the whole smoothing of a threaded set-up on a worker thread. The thread
// reads the grid copy of the set-up, and the stamp box of its stamps, for the time of the job.
static void smooth_in_thread(void *argument) {
    ThreadedSetUp *set_up = argument;
    Squad *squad = set_up->squad;
    StampBox own_stamp_box = get_stamp_box();
    gc_use_grid(set_up->grid);
    set_stamp_box(set_up->stamp_box);
    begin_formation_smoothing(squad, squad->set_up_parking_positions, squad->set_up_facing);
    run_smoothing(INT_MAX);
    set_stamp_box(own_stamp_box);
    gc_use_grid(NULL);
}


// `begin_threaded_set_up` runs the first part of a threaded set-up on the main thread, and hands
// the smoothing to a worker thread. The path search, the raycasts and the formation layout use
// game code, so they stay on the main thread; the smoothing reads only grid cells.
static void begin_threaded_set_up(Squad *squad) {
    static V2 route[MAX_WAYPOINTS];
    int count = store_found_route(squad, route);        // number of path points
    if (!lay_out_parking_positions(squad, route, count, squad->set_up_parking_positions,
                                   &squad->set_up_facing)) {
        fail_move(squad, true);
        return;
    }

    // The grid copy holds the stamps of this squad, so the worker thread sees them.
    ThreadedSetUp *set_up = &threaded_set_ups[squad->squad_index];
    if (!set_up->grid)
        set_up->grid = malloc(GC_GRID_BYTES);
    stamp_parked(squad, squad->start_position, squad->destination);
    prepare_formation_route(squad, squad->set_up_parking_positions, squad->set_up_facing);
    gc_copy_grid(set_up->grid);
    set_up->stamp_box = get_stamp_box();
    unstamp_parked();

    set_up->squad = squad;
    set_up->is_waiting = true;
    waiting_set_up_count++;
    submit_job(smooth_in_thread, set_up);
}


// `finish_threaded_set_ups` waits for the smoothings of the worker threads, and then finishes each
// waiting set-up on the main thread, in squad order, and starts its move.
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
        finish_formation_set_up(squad, squad->set_up_parking_positions, squad->set_up_facing);
        unstamp_parked();
        start_squad_move(squad, squad->set_up_parking_positions);
        set_up->is_waiting = false;
    }
    waiting_set_up_count = 0;
}


// `reset_set_up_work` refills the work budget of the accumulated set-ups for a new tick.
void reset_set_up_work(void) {
    set_up_work_left = SET_UP_WORK_PER_TICK;
}


// `run_set_up_part` runs the next part of the accumulated set-up of a squad and returns its work,
// in clearance tests: the first part, the smoothing up to `max_clearance_tests`, or the finish.
// The work of the first part and the finish is an estimate from route corners and cross-sections.
static int run_set_up_part(Squad *squad, int max_clearance_tests) {
    // A squad that still waits for another set-up costs no work.
    if (squad->set_up_phase == SET_UP_WAITING) {
        begin_accumulated_set_up(squad);
        if (squad->set_up_phase == SET_UP_WAITING)
            return 0;
        return SET_UP_WORK_PER_ROUTE_CORNER * squad->route_count;
    }

    // The smoothing work is the number of clearance tests that it ran.
    stamp_parked(squad, squad->start_position, squad->destination);
    if (squad->set_up_phase == SET_UP_SMOOTHING) {
        int first_test_count = get_clearance_test_count();
        if (run_smoothing(max_clearance_tests))
            squad->set_up_phase = SET_UP_FINISHING;
        unstamp_parked();
        return get_clearance_test_count() - first_test_count;
    }

    finish_formation_set_up(squad, squad->set_up_parking_positions, squad->set_up_facing);
    unstamp_parked();
    squad->set_up_phase = SET_UP_NONE;
    start_squad_move(squad, squad->set_up_parking_positions);
    int cross_section_count = are_lanes_on ? squad->cross_section_count : 0;
    return SET_UP_WORK_PER_CROSS_SECTION * cross_section_count;
}


// `continue_accumulated_set_up` runs the parts of the accumulated set-up of a squad back to back,
// while the work budget of the tick lasts. A squad that still waits for another set-up stops.
static void continue_accumulated_set_up(Squad *squad) {
    while (squad->set_up_phase != SET_UP_NONE && set_up_work_left > 0) {
        bool was_waiting = squad->set_up_phase == SET_UP_WAITING;
        set_up_work_left -= run_set_up_part(squad, set_up_work_left);
        bool is_still_waiting = was_waiting && squad->set_up_phase == SET_UP_WAITING;
        if (is_still_waiting)
            return;
    }
}


// `update_search` runs the route search of a squad for one tick, and the set-up of its move once
// the search found the route. A failed forward search starts the backward search with the
// terrain class and the corner gap of the first squad member, as the game does.
static void update_search(Squad *squad) {
    if (squad->set_up_phase != SET_UP_NONE) {
        continue_accumulated_set_up(squad);
        return;
    }

    // With `--fullsearch`, the backward search also runs in the tick in which the forward search
    // fails.
    bool is_full_search = is_full_search_on;
    int budget = is_full_search ? FULL_SEARCH_BUDGET : SQUAD_SEARCH_BUDGET;
    stamp_parked(squad, squad->start_position, squad->destination);
    int result = gc_update_search(&squad->search, budget);
    bool has_failed = result == GC_SEARCH_FAILED;
    bool must_start_backward = has_failed && !squad->is_backward_search;
    if (must_start_backward) {
        const Unit *first = &units[squad->first_unit_index];
        V2 start = squad->start_position;
        V2 end = squad->destination;
        int terrain_class = first->unit_type->terrain_class;
        float corner_gap = get_corner_gap(first);
        gc_start_search(&squad->search, GC_SEARCH_BACKWARD, start.x, start.y, end.x, end.y,
                        terrain_class, corner_gap, SQUAD_SEARCH_TOLERANCE_METERS);
        squad->is_backward_search = true;
    }
    if (must_start_backward && is_full_search)
        result = gc_update_search(&squad->search, budget);
    unstamp_parked();

    // The set-up runs accumulated, on worker threads, or in one call. A failure with a backward
    // search still to run waits for the next tick.
    bool is_accumulated = is_set_up_accumulate_on && is_formation_mode();
    bool can_thread = is_formation_mode() && is_smooth_on && !is_accumulated;
    bool is_threaded = is_set_up_threads_on && can_thread;
    bool has_backward_to_run = must_start_backward && !is_full_search;
    bool is_denied = result == GC_SEARCH_FAILED && !has_backward_to_run;
    if (result == GC_SEARCH_FOUND && is_accumulated) {
        squad->set_up_phase = SET_UP_WAITING;
        continue_accumulated_set_up(squad);
    } else if (result == GC_SEARCH_FOUND && is_threaded)
        begin_threaded_set_up(squad);
    else if (result == GC_SEARCH_FOUND)
        set_up_move(squad);
    else if (is_denied)
        fail_move(squad, false);
}


// `has_squad_arrived` tells whether every squad member of a squad has stopped, or the straggler
// timer has run out: a squad of more than `STRAGGLER_MIN_MEMBERS` squad members with at most one
// still moving counts as arrived after `STRAGGLER_WAIT_SECONDS`.
static bool has_squad_arrived(Squad *squad) {
    int stopped_count = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++)
        stopped_count += units[unit_index].is_stopped;
    bool has_all_stopped = stopped_count == squad->unit_count;

    // Once it runs, the straggler timer counts down in every tick.
    if (squad->straggler_seconds >= 0) {
        squad->straggler_seconds -= TICK_SECONDS;
        return squad->straggler_seconds <= 0 || has_all_stopped;
    }
    int moving_count = squad->unit_count - stopped_count;
    bool must_start_timer = squad->unit_count > STRAGGLER_MIN_MEMBERS && moving_count <= 1;
    if (must_start_timer)
        squad->straggler_seconds = STRAGGLER_WAIT_SECONDS;
    return has_all_stopped;
}


// `update_squad` updates a squad for one tick: its search and set-up, or its move, its arrival,
// and the checks for parked squads. An arrived squad stops each squad member where it stands, with
// the reach of a stop.
void update_squad(Squad *squad) {
    if (squad->state == SQUAD_SEARCHING) {
        update_search(squad);
        return;
    }
    if (squad->state != SQUAD_MOVING)
        return;

    // The threaded set-ups of the earlier squads of this tick finish before this squad moves, as
    // they would without threads.
    finish_threaded_set_ups();
    if (is_formation_mode())
        advance_anchor(squad);
    if (!has_squad_arrived(squad)) {
        update_settled(squad);
        avoid_parking_ahead(squad);
        return;
    }

    // The moving squads re-route around the arrived squad.
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        Unit *unit = &units[unit_index];
        unit->reach_last = STOP_REACH_METERS;
        stop_at(unit, unit->position);
    }
    squad->state = SQUAD_ARRIVED;
    squad->arrive_time_seconds = now_seconds;
    squad->is_settled = false;
    reroute_around_parked(squad);
}
