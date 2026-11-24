// Detours for stuck units in formation following (`--detour`). A unit that makes no progress, is
// far from its slot and has blocking terrain on the line to its target gets a detour: a path that
// the base game path search finds from the unit to its own lane ahead. The unit follows the detour
// until the line to its target is clear, or until a time limit. The path search does not see
// moving units, so units that only other units block rarely get a detour.
#include "sim.h"
#include "formation.h"

#define PROGRESS_CHECK_SECONDS 2.0f         // time between two progress checks
#define MIN_PROGRESS_METERS 1.0f            // a shorter move between two checks is no progress
#define STUCK_SLOT_METERS 6.0f              // a unit this close to its slot is never stuck
#define DETOUR_AHEAD_METERS 20.0f           // farthest detour end past the route progress
#define DETOUR_SEARCH_BUDGET 500            // expansion budget of the detour path search
#define DETOUR_TOLERANCE_METERS 2.0f        // the path can end this close to the detour end
#define DETOUR_LIMIT_SECONDS 20.0f          // longest time on one detour


// `reset_detour` clears the detour of a unit and starts a new progress check at its position.
void reset_detour(Unit *unit) {
    unit->detour_count = 0;
    unit->detour_index = 0;
    unit->progress_check_position = unit->position;
    unit->progress_check_seconds = 0;
}


// `is_on_detour` tells whether a unit still has detour points to follow.
static bool is_on_detour(const Unit *unit) {
    return unit->detour_index < unit->detour_count;
}


// `has_made_no_progress` runs the progress check of a unit, once every `PROGRESS_CHECK_SECONDS`.
// It returns true when the unit moved less than `MIN_PROGRESS_METERS` since the last check while
// farther than `STUCK_SLOT_METERS` from its slot.
static bool has_made_no_progress(Unit *unit, V2 slot) {
    // Between two checks, only the timer advances.
    unit->progress_check_seconds += TICK_SECONDS;
    if (unit->progress_check_seconds < PROGRESS_CHECK_SECONDS)
        return false;

    // The next check measures from the current position.
    float moved = len(sub(unit->position, unit->progress_check_position));
    unit->progress_check_position = unit->position;
    unit->progress_check_seconds = 0;
    float d_slot = len(sub(slot, unit->position));        // distance to the slot
    return moved < MIN_PROGRESS_METERS && d_slot > STUCK_SLOT_METERS;
}


// `find_detour` searches a detour for a unit, from the unit to its lane at the slot position, but
// at most `DETOUR_AHEAD_METERS` past its route progress. When the base game path search finds no
// path, the detour stays empty.
static void find_detour(const Squad *squad, Unit *unit, float slot_distance) {
    // When the body of the unit does not fit at the lane point, the detour ends on the route.
    float progress = unit->route_progress;
    float d_max = progress + DETOUR_AHEAD_METERS;                     // farthest detour end
    float d_end = clamp_float(slot_distance, progress, d_max);        // route position of the end
    V2 end = get_lane_point(squad, unit, d_end);
    if (!is_circle_clear(end, unit->unit_type->radius, unit->unit_type->terrain_class)) {
        V2 unused;
        end = get_extended_route_point(squad, d_end, &unused);
    }

    // `path_points` holds the path as x, y pairs. Only the first `MAX_DETOUR_POINTS` path points
    // become detour points.
    GcRequest search;
    bool is_found = search_path(unit, end, DETOUR_TOLERANCE_METERS, DETOUR_SEARCH_BUDGET, &search);
    int count = 0;        // number of path points
    if (is_found)
        count = gc_get_path(&search, path_points, MAX_DETOUR_POINTS);
    gc_free_request(&search);
    for (int point_index = 0; point_index < count; point_index++)
        unit->detour[point_index] = get_pair_point(path_points, point_index);
    unit->detour_count = count;
    unit->detour_index = 0;
    unit->detour_seconds = 0;
}


// `follow_detour` starts a detour for a stuck unit, and replaces the target of a unit on a detour
// with its next detour point. It returns false when the unit has no detour, and `target` then
// stays unchanged.
bool follow_detour(const Squad *squad, Unit *unit, V2 slot, float slot_distance, V2 *target) {
    // A new detour needs a stuck unit with blocking terrain on the line to its target.
    bool is_stuck = !is_on_detour(unit) && has_made_no_progress(unit, slot);
    if (is_stuck && !is_way_clear(unit, *target))
        find_detour(squad, unit, slot_distance);
    if (!is_on_detour(unit))
        return false;

    // Detour points within the reach of the unit count as passed.
    unit->detour_seconds += TICK_SECONDS;
    while (is_on_detour(unit)) {
        V2 point = unit->detour[unit->detour_index];
        if (len(sub(point, unit->position)) > unit->reach)
            break;
        unit->detour_index++;
    }

    // The detour ends when the target comes in sight, or after `DETOUR_LIMIT_SECONDS`. The
    // progress check then starts again.
    bool is_target_in_sight = is_way_clear(unit, *target);
    if (is_target_in_sight || unit->detour_seconds > DETOUR_LIMIT_SECONDS)
        unit->detour_index = unit->detour_count;
    if (!is_on_detour(unit)) {
        reset_detour(unit);
        return false;
    }
    *target = unit->detour[unit->detour_index];
    return true;
}
