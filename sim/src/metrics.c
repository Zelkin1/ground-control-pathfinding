/* This file computes the metrics, prints the result rows and writes the JSON trace of the
 * simulator. sim.c calls `measure` after every tick, `report_squad` for each squad after a run,
 * and the trace functions while `--trace` is on. sim/tools/average.py reads the result rows by
 * column. It finds each column by its name in the header of `print_result_header`, so a new
 * column name must change that script too. sim/replay/make_replay.py turns the trace into a
 * replay page. sim/replay/replay_trace.js and sim/tools/check_lanes.py read the keys of the
 * trace, and the table rows (`TABLE_ROWS`) in sim/replay/replay_side.js read the metric keys of
 * each squad.
 *
 * Trace layout: The trace is one JSON object. sim.c writes its outer part,
 * {"dt", "frame_dt", "runs": [...]}, and this file writes each run. A run has this layout, in
 * which "..." stands for more items of the same kind:
 *     {"name", "mode", "map", "desc", "cell",
 *      "units": [{"squad", "r", "v"}, ...],
 *      "frames": [[time, then for each unit: x, y, angle, waypoint x, waypoint y, bits], ...],
 *      "squads": [{"kind", "moving", "route", "slots", "lanes", then the metrics}, ...],
 *      "grid": {"x0", "y0", "w", "h", "cells"}}
 * A squad has "lanes" only in formation mode with lanes and an order, and it has metrics only
 * with an order.
 *
 * Trace keys: "dt" is the tick time and "frame_dt" the time between two frames, in seconds.
 * "cell" is the width of a grid cell in meters. Each unit has the index of its squad "squad", its
 * radius "r" and its top speed "v". A frame gives the angle of each unit in radians and its
 * `TRACE_` state bits. "moving" is 1 for a squad with an order and 0 otherwise. "grid" holds the
 * cells from column "x0" and row "y0" on, "w" cells wide and "h" cells high. */
#include "sim.h"
#include "formation.h"

#define FAST_SPEED_FRACTION 0.5f     /* Above this part of its top speed, a unit is driving. */
#define SLOW_SPEED_FRACTION 0.15f    /* Below this part of its top speed, a unit is slow. */
#define TOP_SPEED_FRACTION 0.95f     /* This part of top speed or more counts as top speed. */
#define PERCENT 100.0f               /* A share times this is a percentage. */
#define STUCK_AFTER_SECONDS 3.0f     /* A unit that is slow for longer than this is stuck. */
#define ARRIVING_RADII 3.0f          /* This many radii or less from its slot, a unit arrives. */
#define SPIN_RADIANS_PER_SECOND 0.4f /* Turning on the spot is a turn faster than this. */
#define SPIN_SPEED_FRACTION 0.2f     /* The unit is also below this part of its top speed. */
#define OBSTACLE_DEPTH_METERS 0.5f   /* A body that is deeper than this in blocked cells counts. */
#define TOUCH_MARGIN_METERS 0.35f    /* Bodies this close touch. Physics puts them 0.3 m apart. */
#define LOST_RADII 3.0f              /* A unit ending further from its slot than this is lost. */
#define TRACE_GRID_MARGIN_METERS 60.0f /* The trace map reaches this far past the used area. */
#define TRACE_LANE_STEP_METERS 4.0f  /* The trace has a lane point at this step along the route. */
#define UNSEEN_EDGE_METERS 1e9f      /* The used area starts empty, with its edges this far out. */
#define NOT_ARRIVED_SECONDS -1.0f    /* A squad that did not arrive has this arrival time. */

/* These are the state bits of a unit in a trace frame. */
enum {
    TRACE_STOPPED = 1,
    TRACE_WAITING = 2,
    TRACE_SWERVING = 4,
    TRACE_SPINNING = 8,
    TRACE_TOUCHING = 16,
    TRACE_ADJUSTING = 32,
};

/* ------------------------------------------------------------------------------------------------
 * `measure_unit` updates the metric fields of the `Unit` after a tick, from its velocity,
 * position, angle and slot, and from the flags `is_walled` and `is_adjusting`.
 * Stops, slow time, stuck time and the time at top speed count only while the squad of the unit
 * moves, `is_stopped` is false, and the unit is more than `ARRIVING_RADII` radii from its slot
 * (its place in the formation at the end of the move), so that braking onto the slot does not
 * count. That time is the travel time of the unit. The other metrics count while the squad
 * moves, which includes the final turn to the facing of the formation. Stuck time
 * (`stuck_seconds`) is the part of a run of slow ticks that lasts longer than
 * `STUCK_AFTER_SECONDS`.
 * --------------------------------------------------------------------------------------------- */
static void measure_unit(Unit *unit) {
    bool is_moving_squad = squads[unit->squad_index].state == SQUAD_MOVING;
    float speed = len(unit->velocity);
    float max_speed = unit->kind->max_speed;
    float slow_speed = SLOW_SPEED_FRACTION * max_speed;

    /* Count a stop when the unit becomes slow after it was driving. */
    float slot_distance = len(sub(unit->position, unit->slot));
    float arriving_distance = ARRIVING_RADII * unit->kind->radius;
    bool is_away_from_slot = slot_distance > arriving_distance;
    bool is_heading_to_slot = is_moving_squad && !unit->is_stopped && is_away_from_slot;
    bool is_slow = speed < slow_speed;
    if (speed > FAST_SPEED_FRACTION * max_speed)
        unit->was_fast = true;
    if (is_heading_to_slot && unit->was_fast && is_slow) {
        unit->stop_count++;
        unit->was_fast = false;
    }

    /* Add the tick to the slow time, and to the stuck time once the unit is slow for longer than
     * `STUCK_AFTER_SECONDS` without a break. */
    if (is_heading_to_slot && is_slow) {
        unit->slow_seconds += TICK_SECONDS;
        unit->slow_run_seconds += TICK_SECONDS;
        if (unit->slow_run_seconds > STUCK_AFTER_SECONDS)
            unit->stuck_seconds += TICK_SECONDS;
    } else {
        unit->slow_run_seconds = 0;
    }

    /* Add the tick to the travel time, and to the time at top speed when the unit is that fast. */
    if (is_heading_to_slot) {
        unit->travel_seconds += TICK_SECONDS;
        if (speed >= TOP_SPEED_FRACTION * max_speed)
            unit->top_speed_seconds += TICK_SECONDS;
    }

    /* Find the turn of this tick, from -PI to PI, and count the time turning on the spot. */
    float angle_change = unit->angle - unit->previous_angle;
    if (angle_change > PI)
        angle_change -= 2 * PI;
    if (angle_change < -PI)
        angle_change += 2 * PI;
    bool is_turning_fast = fabsf(angle_change) > SPIN_RADIANS_PER_SECOND * TICK_SECONDS;
    bool is_slow_for_spin = speed < SPIN_SPEED_FRACTION * max_speed;
    unit->is_spinning = is_moving_squad && is_turning_fast && is_slow_for_spin;
    if (unit->is_spinning)
        unit->spin_seconds += TICK_SECONDS;
    unit->previous_angle = unit->angle;

    /* Count the time with the body more than `OBSTACLE_DEPTH_METERS` deep in blocked cells. The
     * collisions of the game keep only the center of the unit out of them. */
    float inner_radius = unit->kind->radius - OBSTACLE_DEPTH_METERS;
    int mover_class = unit->kind->mover_class;
    if (is_moving_squad && !is_circle_clear(unit->position, inner_radius, mover_class))
        unit->in_obstacle_seconds += TICK_SECONDS;

    /* Count a wall scrape when the unit starts to slide along blocked cells. */
    if (unit->is_walled && !unit->was_walled && is_moving_squad)
        unit->wall_hit_count++;
    unit->was_walled = unit->is_walled;

    /* Count an avoidance change when the new avoidance starts to change the velocity of the
     * unit. */
    if (unit->is_adjusting && !unit->was_adjusting && is_moving_squad)
        unit->adjust_count++;
    unit->was_adjusting = unit->is_adjusting;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_contacts` counts the bumps between unit `unit_index` and each unit with a higher index,
 * so that each pair is tested once. A bump is the tick at which two units start to touch, and it
 * counts for both units (`own_bump_count` within a squad, `other_bump_count` between squads). The
 * unit with the lower index keeps the touch state of the pair: `is_touching` holds one entry for
 * each other unit, at the index of that unit. `measure_contacts` also sets `is_in_contact` for
 * both units of each pair that touches.
 * --------------------------------------------------------------------------------------------- */
static void measure_contacts(int unit_index) {
    Unit *unit = &units[unit_index];
    for (int other_index = unit_index + 1; other_index < unit_count; other_index++) {
        /* Find out whether the two units touch. */
        Unit *other = &units[other_index];
        float touch_distance = unit->kind->radius + other->kind->radius + TOUCH_MARGIN_METERS;
        float distance_squared = length_squared(sub(unit->position, other->position));
        bool is_touching = distance_squared < touch_distance * touch_distance;

        /* Count a bump when they start to touch, as one within the squad or with another squad. */
        bool is_new_touch = is_touching && !unit->is_touching[other_index];
        if (is_new_touch) {
            if (unit->squad_index == other->squad_index) {
                unit->own_bump_count++;
                other->own_bump_count++;
            } else {
                unit->other_bump_count++;
                other->other_bump_count++;
            }
        }
        unit->is_touching[other_index] = is_touching;
        if (is_touching) {
            unit->is_in_contact = true;
            other->is_in_contact = true;
        }
    }
}

/* ------------------------------------------------------------------------------------------------
 * `measure_spread` measures how far the members of the squad spread out beyond the size of the
 * formation itself. The spread is the distance from the mean position of the members to the member
 * furthest from it, less `spread_reference`, the same distance for the slots of the formation,
 * and at least 0. `measure_spread` keeps the largest spread in `max_spread`, and adds the spread
 * to `spread_sum` for the mean.
 *
 * In the sketch, o marks a member and c the mean position of the members. The line from c to the
 * furthest member gives the spread before `spread_reference` comes off.
 *
 *         o
 *            c-------o
 *         o
 * --------------------------------------------------------------------------------------------- */
static void measure_spread(Squad *squad) {
    /* Find the mean position of the members. The members are the units from `first_unit_index` up
     * to, but not including, `end_index`. */
    int first_index = squad->first_unit_index;
    int end_index = squad->first_unit_index + squad->unit_count;
    V2 center = v2(0, 0);
    for (int unit_index = first_index; unit_index < end_index; unit_index++)
        center = add(center, units[unit_index].position);
    center = mul(center, 1.0f / squad->unit_count);

    /* Find the distance of the member furthest from it. */
    float spread = 0;
    for (int unit_index = first_index; unit_index < end_index; unit_index++) {
        float distance = len(sub(units[unit_index].position, center));
        if (distance > spread)
            spread = distance;
    }

    /* Keep the spread beyond the size of the formation itself. */
    spread -= squad->spread_reference;
    if (spread < 0)
        spread = 0;
    if (spread > squad->max_spread)
        squad->max_spread = spread;
    squad->spread_sum += spread;
    squad->spread_sample_count++;
}

/* ------------------------------------------------------------------------------------------------
 * `measure` clears the contact flags first, and then measures each unit and its contacts, and the
 * spread of each moving squad.
 * --------------------------------------------------------------------------------------------- */
void measure(void) {
    /* Clear the contact flags first, because `measure_contacts` also sets them for units with
     * higher indexes. */
    for (int unit_index = 0; unit_index < unit_count; unit_index++)
        units[unit_index].is_in_contact = false;

    /* Measure each unit and its contacts, then the spread of each moving squad. */
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        measure_unit(&units[unit_index]);
        measure_contacts(unit_index);
    }
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (squads[squad_index].state == SQUAD_MOVING)
            measure_spread(&squads[squad_index]);
    }
}

/* The functions from here on write the trace. */

/* `seen_min` and `seen_max` are the corners of the area that the units and routes used in this
 * run. The map excerpt of the trace covers this area. */
static V2 seen_min;
static V2 seen_max;

/* ------------------------------------------------------------------------------------------------
 * `extend_seen_area` grows the used area, if needed, so that it holds `point`.
 * --------------------------------------------------------------------------------------------- */
static void extend_seen_area(V2 point) {
    seen_min.x = fminf(seen_min.x, point.x);
    seen_min.y = fminf(seen_min.y, point.y);
    seen_max.x = fmaxf(seen_max.x, point.x);
    seen_max.y = fmaxf(seen_max.y, point.y);
}

/* ------------------------------------------------------------------------------------------------
 * `get_separator` returns the text that goes before an item of a JSON list: nothing before the
 * first item, and a comma before the others.
 * --------------------------------------------------------------------------------------------- */
static const char *get_separator(bool is_first_item) {
    return is_first_item ? "" : ",";
}

/* ------------------------------------------------------------------------------------------------
 * `get_trace_bits` returns the `TRACE_` state bits of the unit for a trace frame.
 * --------------------------------------------------------------------------------------------- */
static int get_trace_bits(const Unit *unit) {
    int bits = 0;
    if (unit->is_stopped)
        bits |= TRACE_STOPPED;
    if (unit->wait_seconds > 0)
        bits |= TRACE_WAITING;
    if (unit->is_swerving)
        bits |= TRACE_SWERVING;
    if (unit->is_spinning)
        bits |= TRACE_SPINNING;
    if (unit->is_in_contact)
        bits |= TRACE_TOUCHING;
    if (unit->is_adjusting)
        bits |= TRACE_ADJUSTING;
    return bits;
}

/* ------------------------------------------------------------------------------------------------
 * `write_trace_frame` writes one frame of the trace, and it also grows the used area
 * (`extend_seen_area`) with the position of every unit, because `write_trace_grid` takes the map
 * excerpt from that area.
 * --------------------------------------------------------------------------------------------- */
void write_trace_frame(bool is_first) {
    fprintf(trace_file, "%s[%.2f", get_separator(is_first), now_seconds);
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        const Unit *unit = &units[unit_index];
        V2 position = unit->position;
        V2 waypoint = unit->waypoint;
        fprintf(trace_file, ",%.1f,%.1f,%.2f", position.x, position.y, unit->angle);
        fprintf(trace_file, ",%.1f,%.1f,%d", waypoint.x, waypoint.y, get_trace_bits(unit));
        extend_seen_area(unit->position);
    }
    fputc(']', trace_file);
}

/* ------------------------------------------------------------------------------------------------
 * `get_clamped_cell` returns the grid coordinate of the cell that holds the world coordinate
 * `world_meters`, along either axis. A coordinate outside the grid gives the closest edge cell.
 * --------------------------------------------------------------------------------------------- */
static int get_clamped_cell(float world_meters) {
    int cell = get_cell_coordinate(world_meters);
    if (cell < 0)
        return 0;
    return cell >= GC_GRID_SIZE ? GC_GRID_SIZE - 1 : cell;
}

/* ------------------------------------------------------------------------------------------------
 * `write_trace_grid` takes the used area from `seen_min` and `seen_max`. `begin_trace_run` resets
 * them, and `write_trace_frame` grows them with the positions of the units. `write_trace_grid`
 * adds the route points of every squad itself.
 * --------------------------------------------------------------------------------------------- */
void write_trace_grid(void) {
    /* Find the cells of the area that the units and routes used, plus a margin. */
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        const Squad *squad = &squads[squad_index];
        for (int point_index = 0; point_index < squad->route_count; point_index++)
            extend_seen_area(squad->route[point_index]);
    }
    int min_cell_x = get_clamped_cell(seen_min.x - TRACE_GRID_MARGIN_METERS);
    int min_cell_y = get_clamped_cell(seen_min.y - TRACE_GRID_MARGIN_METERS);
    int max_cell_x = get_clamped_cell(seen_max.x + TRACE_GRID_MARGIN_METERS);
    int max_cell_y = get_clamped_cell(seen_max.y + TRACE_GRID_MARGIN_METERS);
    int width_cells = max_cell_x - min_cell_x + 1;
    int height_cells = max_cell_y - min_cell_y + 1;

    /* Write one digit per cell, the value of the cell from 0 to 3, row by row from `min_cell_y`
     * up. */
    fprintf(trace_file, ",\"grid\":{\"x0\":%d,\"y0\":%d,", min_cell_x, min_cell_y);
    fprintf(trace_file, "\"w\":%d,\"h\":%d,\"cells\":\"", width_cells, height_cells);
    for (int cell_y = min_cell_y; cell_y <= max_cell_y; cell_y++) {
        for (int cell_x = min_cell_x; cell_x <= max_cell_x; cell_x++)
            fputc('0' + gc_get_cell(cell_x, cell_y), trace_file);
    }
    fputc('"', trace_file);
    fputc('}', trace_file);
}

/* ------------------------------------------------------------------------------------------------
 * `begin_trace_run` writes the start of a run object of the trace, up to the open list of frames.
 * It writes the map name and the units, so `set_up_scenario` must have run before it.
 * --------------------------------------------------------------------------------------------- */
void begin_trace_run(int scenario_index) {
    /* Start the used area empty, so that the first unit position sets it. */
    seen_min = v2(UNSEEN_EDGE_METERS, UNSEEN_EDGE_METERS);
    seen_max = v2(-UNSEEN_EDGE_METERS, -UNSEEN_EDGE_METERS);

    /* Write the names of the run and the cell size, then the squad, radius and top speed of each
     * unit. */
    const char *scenario_name = SCENARIO_NAME[scenario_index];
    const char *description = SCENARIO_DESCRIPTION[scenario_index];
    fprintf(trace_file, "{\"name\":\"%s\",\"mode\":\"%s\",", scenario_name, MODE_NAME[avoid_mode]);
    fprintf(trace_file, "\"map\":\"%s\",", map_name);
    fprintf(trace_file, "\"desc\":\"%s\",\"cell\":%g,", description, GC_CELL_METERS);
    fprintf(trace_file, "\"units\":[");
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        const Unit *unit = &units[unit_index];
        const char *separator = get_separator(unit_index == 0);
        float radius = unit->kind->radius;
        float max_speed = unit->kind->max_speed;
        fprintf(trace_file, "%s{\"squad\":%d,\"r\":%g,", separator, unit->squad_index, radius);
        fprintf(trace_file, "\"v\":%g}", max_speed);
    }
    fprintf(trace_file, "],\"frames\":[");
}

/* ------------------------------------------------------------------------------------------------
 * `write_trace_lane` writes the points of the lane of a member every `TRACE_LANE_STEP_METERS`
 * along the route of the squad, with null where the squad does not fit side by side.
 * --------------------------------------------------------------------------------------------- */
static void write_trace_lane(const Squad *squad, const Unit *unit) {
    float route_length = squad->route_length;
    for (float distance = 0; distance <= route_length; distance += TRACE_LANE_STEP_METERS) {
        const char *separator = get_separator(distance <= 0);
        V2 point = get_lane_point(squad, unit, distance);
        Lane lane = get_lane(squad, distance);
        if (lane.is_wide_enough)
            fprintf(trace_file, "%s[%.1f,%.1f]", separator, point.x, point.y);
        else
            fprintf(trace_file, "%snull", separator);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `write_trace_squad_shape` writes the kind, route and slots of the squad into the trace, and in
 * formation mode with lanes, the lane of each member when the squad has an order. It leaves the
 * JSON object of the squad open for the metrics.
 * --------------------------------------------------------------------------------------------- */
static void write_trace_squad_shape(Squad *squad, int squad_index) {
    /* Write the kind, whether the squad has an order, and the route. */
    const char *kind_name = squad->kind->name;
    fprintf(trace_file, "%s{\"kind\":\"%s\",", get_separator(squad_index == 0), kind_name);
    fprintf(trace_file, "\"moving\":%d,\"route\":[", has_order[squad_index]);
    for (int point_index = 0; point_index < squad->route_count; point_index++) {
        V2 point = squad->route[point_index];
        const char *separator = get_separator(point_index == 0);
        fprintf(trace_file, "%s[%.1f,%.1f]", separator, point.x, point.y);
    }

    /* Write the slots. */
    int first_index = squad->first_unit_index;
    int end_index = squad->first_unit_index + squad->unit_count;
    fprintf(trace_file, "],\"slots\":[");
    for (int unit_index = first_index; unit_index < end_index; unit_index++) {
        V2 slot = units[unit_index].slot;
        const char *separator = get_separator(unit_index == first_index);
        fprintf(trace_file, "%s[%.1f,%.1f]", separator, slot.x, slot.y);
    }
    fprintf(trace_file, "]");
    bool has_lanes = is_formation_mode() && are_lanes_on && has_order[squad_index];
    if (!has_lanes)
        return;

    /* Write the lane of each member. `get_lane_point` returns the line of the single file while
     * `is_single_file` is true, so the function clears the flag while it writes the lanes, and
     * then restores it. */
    bool was_single_file = squad->is_single_file;
    squad->is_single_file = false;
    fprintf(trace_file, ",\"lanes\":[");
    for (int unit_index = first_index; unit_index < end_index; unit_index++) {
        fprintf(trace_file, "%s[", get_separator(unit_index == first_index));
        write_trace_lane(squad, &units[unit_index]);
        fputc(']', trace_file);
    }
    fputc(']', trace_file);
    squad->is_single_file = was_single_file;
}

/* A `ResultRow` holds the values of the result row of a squad that `make_result_row` works out.
 * The row also shows some fields of the squad as they are. The values whose names end in
 * `_per_member` are averages over the members. The counts, `swerve_seconds` and `adjust_seconds`
 * are totals for the squad. */
typedef struct {
    float arrive_seconds;
    float stops_per_member;
    float slow_seconds_per_member;
    float stuck_seconds_per_member;
    float spin_seconds_per_member;
    int wait_count;
    float swerve_seconds;
    float adjust_seconds;
    int own_bump_count;   /* Column "bmpo" counts the bumps within the squad. */
    int other_bump_count; /* Column "bmpx" counts the bumps with other squads. */
    int wall_hit_count;
    float in_obstacle_seconds_per_member;
    float adjust_count_per_member; /* Column "adjn" counts the starts of avoidance changes. */
    float top_speed_percent;     /* Column "top" is the member time at top speed, in percent. */
    float start_seconds;         /* Column "start" is the time from the order to the move. */
    float mean_spread;           /* Column "sep" is the mean of the spread over the move. */
    float slot_error_per_member; /* Column "slot" is the mean distance to the slots. */
    float slot_error_max;        /* Column "slotx" is the largest such distance. */
    int lost_count;
    const char *state_text;
} ResultRow;

/* ------------------------------------------------------------------------------------------------
 * `get_state_text` returns the text at the end of the result row of a squad: nothing for a squad
 * that arrived, and otherwise why it did not arrive.
 * --------------------------------------------------------------------------------------------- */
static const char *get_state_text(const Squad *squad) {
    if (squad->state == SQUAD_DENIED)
        return " DENIED";
    if (squad->state != SQUAD_ARRIVED)
        return " NOT ARRIVED";
    return "";
}

/* ------------------------------------------------------------------------------------------------
 * `make_result_row` sums the metrics of the members and works out the result row of the squad. A
 * bump within the squad counts for both of its units, so the sum over the members is halved.
 * --------------------------------------------------------------------------------------------- */
static ResultRow make_result_row(const Squad *squad) {
    /* Sum the metrics of the members. */
    float slot_error_sum = 0;
    float slot_error_max = 0;
    int stop_count = 0;
    int wait_count = 0;
    int own_bump_sum = 0;
    int other_bump_count = 0;
    int lost_count = 0;
    int wall_hit_count = 0;
    int adjust_count = 0;
    float slow_seconds = 0;
    float stuck_seconds = 0;
    float travel_seconds = 0;
    float top_speed_seconds = 0;
    float swerve_seconds = 0;
    float adjust_seconds = 0;
    float spin_seconds = 0;
    float in_obstacle_seconds = 0;
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        float slot_error = len(sub(unit->position, unit->slot));
        slot_error_sum += slot_error;
        if (slot_error > slot_error_max)
            slot_error_max = slot_error;
        bool is_lost = slot_error > LOST_RADII * unit->kind->radius;
        lost_count += is_lost;
        stop_count += unit->stop_count;
        wait_count += unit->wait_count;
        own_bump_sum += unit->own_bump_count;
        other_bump_count += unit->other_bump_count;
        wall_hit_count += unit->wall_hit_count;
        adjust_count += unit->adjust_count;
        in_obstacle_seconds += unit->in_obstacle_seconds;
        slow_seconds += unit->slow_seconds;
        stuck_seconds += unit->stuck_seconds;
        travel_seconds += unit->travel_seconds;
        top_speed_seconds += unit->top_speed_seconds;
        swerve_seconds += unit->swerve_seconds;
        adjust_seconds += unit->adjust_seconds;
        spin_seconds += unit->spin_seconds;
    }

    /* Find the times from the order to the start of the move (the responsiveness) and to the
     * arrival, and the mean spread. A squad that never started or arrived gets
     * `NOT_ARRIVED_SECONDS` for that time. */
    float start_seconds = NOT_ARRIVED_SECONDS;
    if (squad->start_time_seconds != NOT_STARTED_SECONDS)
        start_seconds = squad->start_time_seconds - squad->order_time_seconds;
    float arrive_seconds = NOT_ARRIVED_SECONDS;
    if (squad->state == SQUAD_ARRIVED)
        arrive_seconds = squad->arrive_time_seconds - squad->order_time_seconds;
    float mean_spread = 0;
    if (squad->spread_sample_count)
        mean_spread = squad->spread_sum / squad->spread_sample_count;

    /* Find the share of the travel time of the members at top speed. */
    float top_speed_percent = 0;
    if (travel_seconds > 0)
        top_speed_percent = PERCENT * top_speed_seconds / travel_seconds;

    /* Fill the row, with the averages over the members. */
    float member_count = (float)squad->unit_count;
    return (ResultRow){
        .arrive_seconds = arrive_seconds,
        .stops_per_member = stop_count / member_count,
        .slow_seconds_per_member = slow_seconds / member_count,
        .stuck_seconds_per_member = stuck_seconds / member_count,
        .spin_seconds_per_member = spin_seconds / member_count,
        .wait_count = wait_count,
        .swerve_seconds = swerve_seconds,
        .adjust_seconds = adjust_seconds,
        .own_bump_count = own_bump_sum / 2,
        .other_bump_count = other_bump_count,
        .wall_hit_count = wall_hit_count,
        .in_obstacle_seconds_per_member = in_obstacle_seconds / member_count,
        .adjust_count_per_member = adjust_count / member_count,
        .top_speed_percent = top_speed_percent,
        .start_seconds = start_seconds,
        .mean_spread = mean_spread,
        .slot_error_per_member = slot_error_sum / member_count,
        .slot_error_max = slot_error_max,
        .lost_count = lost_count,
        .state_text = get_state_text(squad),
    };
}

/* ------------------------------------------------------------------------------------------------
 * `print_result_header` and `print_result_row` write the same columns in the same order, so the
 * two functions change together. The names of the header are the names by which
 * sim/tools/average.py finds the columns, so a new name must change that script too.
 * --------------------------------------------------------------------------------------------- */
void print_result_header(void) {
    printf("%-8s %-5s %s %-13s %2s ", "scen", "mode", "q", "kind", "n");
    printf("%6s %5s %6s %6s %5s ", "arrive", "stops", "slow_s", "stuck", "spin");
    printf("%5s %6s %6s ", "waits", "swerve", "adjust");
    printf("%4s %4s %4s ", "bmpo", "bmpx", "wall");
    printf("%5s %5s %6s %5s %5s ", "inobs", "sep", "sepmax", "slot", "slotx");
    printf("%4s %4s %5s %5s %5s %5s\n", "lost", "xing", "swaps", "adjn", "top", "start");
}

/* ------------------------------------------------------------------------------------------------
 * `print_result_row` prints the result row of the squad, in the columns of
 * `print_result_header`.
 * --------------------------------------------------------------------------------------------- */
static void print_result_row(int scenario_index, int squad_index, const Squad *squad,
                             const ResultRow *row) {
    /* Print the scenario, mode, squad index, kind and member count. */
    printf("%-8s %-5s %d ", SCENARIO_NAME[scenario_index], MODE_NAME[avoid_mode], squad_index);
    printf("%-13s %2d ", squad->kind->name, squad->unit_count);

    /* Print the metrics. `crossing_count`, column "xing", counts the pairs of members whose
     * straight lines from start to slot cross, and `reassign_count`, column "swaps", counts the
     * changes of the match between members and places. */
    printf("%6.1f %5.2f ", row->arrive_seconds, row->stops_per_member);
    printf("%6.1f %6.1f ", row->slow_seconds_per_member, row->stuck_seconds_per_member);
    printf("%5.1f %5d ", row->spin_seconds_per_member, row->wait_count);
    printf("%6.1f %6.1f ", row->swerve_seconds, row->adjust_seconds);
    printf("%4d %4d %4d ", row->own_bump_count, row->other_bump_count, row->wall_hit_count);
    printf("%5.1f %5.1f ", row->in_obstacle_seconds_per_member, row->mean_spread);
    printf("%6.1f %5.1f ", squad->max_spread, row->slot_error_per_member);
    printf("%5.1f %4d ", row->slot_error_max, row->lost_count);
    printf("%4d %5d ", squad->crossing_count, squad->reassign_count);
    printf("%5.1f %5.1f ", row->adjust_count_per_member, row->top_speed_percent);
    printf("%5.2f%s\n", row->start_seconds, row->state_text);
}

/* ------------------------------------------------------------------------------------------------
 * `write_trace_result` writes the metrics of the squad into the trace and closes its JSON
 * object.
 * --------------------------------------------------------------------------------------------- */
static void write_trace_result(const Squad *squad, const ResultRow *row) {
    fprintf(trace_file, ",\"arrive\":%.1f", row->arrive_seconds);
    fprintf(trace_file, ",\"stops\":%.2f", row->stops_per_member);
    fprintf(trace_file, ",\"slow\":%.1f", row->slow_seconds_per_member);
    fprintf(trace_file, ",\"stuck\":%.1f", row->stuck_seconds_per_member);
    fprintf(trace_file, ",\"spin\":%.1f", row->spin_seconds_per_member);
    fprintf(trace_file, ",\"waits\":%d", row->wait_count);
    fprintf(trace_file, ",\"swerve\":%.1f", row->swerve_seconds);
    fprintf(trace_file, ",\"adjust\":%.1f", row->adjust_seconds);
    fprintf(trace_file, ",\"bumps_own\":%d", row->own_bump_count);
    fprintf(trace_file, ",\"bumps_other\":%d", row->other_bump_count);
    fprintf(trace_file, ",\"wall\":%d", row->wall_hit_count);
    fprintf(trace_file, ",\"inobs\":%.1f", row->in_obstacle_seconds_per_member);
    fprintf(trace_file, ",\"spread\":%.1f", row->mean_spread);
    fprintf(trace_file, ",\"spread_max\":%.1f", squad->max_spread);
    fprintf(trace_file, ",\"slot_err\":%.1f", row->slot_error_per_member);
    fprintf(trace_file, ",\"slot_err_max\":%.1f", row->slot_error_max);
    fprintf(trace_file, ",\"lost\":%d", row->lost_count);
    fprintf(trace_file, ",\"adjust_count\":%.1f", row->adjust_count_per_member);
    fprintf(trace_file, ",\"top_speed\":%.1f", row->top_speed_percent);
    fprintf(trace_file, ",\"start\":%.2f", row->start_seconds);
    fprintf(trace_file, ",\"swaps\":%d}", squad->reassign_count);
}

/* ------------------------------------------------------------------------------------------------
 * `report_squad` writes the shape of the squad into the trace, and for a squad with an order it
 * also builds the result row once and uses it for both the printed row and the trace metrics.
 * --------------------------------------------------------------------------------------------- */
void report_squad(int scenario_index, int squad_index) {
    /* Write the shape of the squad into the trace. A squad without an order has no result row,
     * so its JSON object closes here. */
    Squad *squad = &squads[squad_index];
    if (trace_file)
        write_trace_squad_shape(squad, squad_index);
    if (!has_order[squad_index]) {
        if (trace_file)
            fputc('}', trace_file);
        return;
    }

    /* Print the result row, and write its metrics into the trace. */
    ResultRow row = make_result_row(squad);
    print_result_row(scenario_index, squad_index, squad, &row);
    if (trace_file)
        write_trace_result(squad, &row);
}
