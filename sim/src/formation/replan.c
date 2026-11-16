/* replan.c gives a unit in formation following a new way, a detour, when the unit makes no
 * progress.
 *
 * Problem: A unit can stand still for good behind a small island of blocked cells. Formation
 * following gives each member a lane, a path beside the squad route, and the unit drives at a
 * point on its lane. When that lane point lies past the island, the line to it is blocked. The
 * brake check (the test of steering.c that stops a unit before blocked ground) then holds the
 * unit, because the ground along its heading is blocked. Nothing else in formation following
 * plans a way around such an island.
 *
 * Method: Every `PROGRESS_CHECK_SECONDS`, the progress check compares the position of the unit
 * with its position at the start of the check. The target place of the unit is the point where
 * formation following wants the unit to be now. When the unit moved less than
 * `MIN_PROGRESS_METERS`, its place is more than `STUCK_PLACE_METERS` away, and blocked ground
 * lies on the straight line to the point that it drives at, the search of the game itself finds
 * a way around the obstacles to a point on the lane of the unit ahead. The unit then drives along
 * the points of that way, one after the other. Formation following takes over again when the
 * unit reaches the last one, when the straight way to its target is clear, or after
 * `DETOUR_LIMIT_SECONDS`. */
#include "sim.h"
#include "formation.h"

#define PROGRESS_CHECK_SECONDS 2.0f  /* The progress check compares positions this far apart. */
#define MIN_PROGRESS_METERS 1.0f     /* A unit that moved less than this made no progress. */
#define STUCK_PLACE_METERS 6.0f      /* A unit this close to its place never counts as stuck. */
#define DETOUR_AHEAD_METERS 20.0f    /* A detour ends at most this far along the route ahead. */
#define DETOUR_SEARCH_BUDGET 500     /* The detour search makes up to this many expansions. */
#define DETOUR_TOLERANCE_METERS 2.0f /* The search can end this close to the detour end. */
#define DETOUR_LIMIT_SECONDS 20.0f   /* A unit drops a detour that takes longer than this. */

/* ------------------------------------------------------------------------------------------------
 * `reset_detour` empties the detour list and restarts the progress check at the current position
 * of the unit, with a check time of 0.
 * --------------------------------------------------------------------------------------------- */
void reset_detour(Unit *unit) {
    unit->detour_count = 0;
    unit->detour_index = 0;
    unit->progress_check_position = unit->position;
    unit->progress_check_seconds = 0;
}

/* ------------------------------------------------------------------------------------------------
 * `is_on_detour` returns true while the unit has detour points left to drive to.
 * --------------------------------------------------------------------------------------------- */
static bool is_on_detour(const Unit *unit) {
    return unit->detour_index < unit->detour_count;
}

/* ------------------------------------------------------------------------------------------------
 * `has_made_no_progress` runs the progress check of the unit for one tick.
 * Result: It returns true when a check of `PROGRESS_CHECK_SECONDS` ends now, the unit moved less
 *   than `MIN_PROGRESS_METERS` during it, and `place` is more than `STUCK_PLACE_METERS` away.
 *   Otherwise it returns false. A new check starts when one ends.
 * --------------------------------------------------------------------------------------------- */
static bool has_made_no_progress(Unit *unit, V2 place) {
    /* Let the check run until it lasted `PROGRESS_CHECK_SECONDS`. */
    unit->progress_check_seconds += TICK_SECONDS;
    if (unit->progress_check_seconds < PROGRESS_CHECK_SECONDS)
        return false;

    /* Measure the move of the unit during the check, and start a new check. */
    float moved_distance = len(sub(unit->position, unit->progress_check_position));
    unit->progress_check_position = unit->position;
    unit->progress_check_seconds = 0;
    float distance_to_place = len(sub(place, unit->position));
    return moved_distance < MIN_PROGRESS_METERS && distance_to_place > STUCK_PLACE_METERS;
}

/* ------------------------------------------------------------------------------------------------
 * `find_detour` searches a detour for the unit and stores it in `detour`.
 * Inputs: `place_distance` is how far along the route the place of the unit is.
 * Result: The detour ends on the lane of the unit at the distance of the place along the route,
 *   but at most `DETOUR_AHEAD_METERS` past the route progress of the unit, and not before that
 *   progress. Where the lane point lacks room for the body of the unit, the detour ends on the
 *   squad route instead, which has room for the widest member.
 * Edge cases: When the search finds no way, the detour stays empty. A way with more than
 *   `MAX_DETOUR_POINTS` points keeps only its first points.
 * --------------------------------------------------------------------------------------------- */
static void find_detour(const Squad *squad, Unit *unit, float place_distance) {
    /* Pick the end of the detour. */
    float progress = unit->route_progress;
    float farthest_distance = progress + DETOUR_AHEAD_METERS;
    float end_distance = clamp_float(place_distance, progress, farthest_distance);
    V2 end = get_lane_point(squad, unit, end_distance);
    if (!is_circle_clear(end, unit->kind->radius, unit->kind->mover_class)) {
        V2 unused_direction;
        end = get_extended_route_point(squad, end_distance, &unused_direction);
    }

    /* Search the way, and keep its points. */
    GcRequest search;
    bool is_found = search_path(unit, end, DETOUR_TOLERANCE_METERS, DETOUR_SEARCH_BUDGET, &search);
    int point_count = 0;
    if (is_found)
        point_count = gc_get_path(&search, path_points, MAX_DETOUR_POINTS);
    gc_free_request(&search);
    for (int point_index = 0; point_index < point_count; point_index++)
        unit->detour[point_index] = get_pair_point(path_points, point_index);
    unit->detour_count = point_count;
    unit->detour_index = 0;
    unit->detour_seconds = 0;
}

/* ------------------------------------------------------------------------------------------------
 * `follow_detour` runs the progress check in every tick without a detour, and `find_detour`
 * starts a detour when the check fails and `is_way_clear` finds blocked ground on the line to
 * `*target`. While a detour runs, `follow_detour` skips the detour points that the unit reached.
 * It ends the detour when the last point is reached, when the straight way to `*target` is
 * clear, or after `DETOUR_LIMIT_SECONDS`, and it then resets the detour state.
 * --------------------------------------------------------------------------------------------- */
bool follow_detour(const Squad *squad, Unit *unit, V2 place, float place_distance, V2 *target) {
    /* Start a detour when the unit is not on one, made no progress, and blocked ground lies on
     * the straight line to its target. The test of the line leaves out most units that only other
     * units hold up. A detour does not help them, because the search does not see moving units. */
    if (!is_on_detour(unit) && has_made_no_progress(unit, place) && !is_way_clear(unit, *target))
        find_detour(squad, unit, place_distance);
    if (!is_on_detour(unit))
        return false;

    /* Skip the detour points within the `reach` of the unit. End the detour as soon as the
     * straight way to the target of the unit is clear again, because a detour point can lie
     * behind the unit, among squad mates that follow it. Drop the detour after
     * `DETOUR_LIMIT_SECONDS` too, so that a unit that cannot follow it gets a new check. */
    unit->detour_seconds += TICK_SECONDS;
    while (is_on_detour(unit)) {
        V2 point = unit->detour[unit->detour_index];
        if (len(sub(point, unit->position)) > unit->reach)
            break;
        unit->detour_index++;
    }
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
