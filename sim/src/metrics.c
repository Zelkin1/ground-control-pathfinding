// Metrics of a run: the counts and times that `measure` adds up in each tick, the result row of
// each squad with an order, and the trace that the replay reads.
//
// The stops, the slow and stuck time and the time at top speed count only while a unit heads to
// its final parking position: its squad moves, the unit is not stopped, and it is further than
// `ARRIVING_RADII` radii from that position. The other counts and times of a unit count while its
// squad moves, and bumps count at any time. The result row adds them up over the squad members,
// and gives some of them per squad member.
//
// The trace is one JSON object per run: the scenario, the units, one frame per tick, the squads
// with their routes, final parking positions, lanes and results, and the grid cells around the
// area that the units and routes cover.
#include "sim.h"
#include "formation.h"

#define FAST_SPEED_FRACTION 0.5f                    // share of top speed of a driving unit
#define SLOW_SPEED_FRACTION 0.15f                   // share of top speed of a slow unit
#define TOP_SPEED_FRACTION 0.95f                    // share of top speed that counts as top
#define PERCENT 100.0f                              // percent in a whole
#define STUCK_AFTER_SECONDS 3.0f                    // slow time after which a unit is stuck
#define ARRIVING_RADII 3.0f                         // distance in radii of an arriving unit
#define TURN_ON_SPOT_RADIANS_PER_SECOND 0.4f        // slowest turn rate of a turn on the spot
#define TURN_ON_SPOT_SPEED_FRACTION 0.2f            // share of top speed below which it counts
#define TERRAIN_DEPTH_METERS 0.5f                   // depth into blocking terrain that counts
#define TOUCH_MARGIN_METERS 0.35f                   // largest gap between touching bodies
#define LOST_RADII 3.0f                             // end distance in radii of a lost unit
#define TRACE_GRID_MARGIN_METERS 60.0f              // trace grid margin around the seen area
#define TRACE_LANE_STEP_METERS 4.0f                 // route step between trace lane points
#define UNSEEN_EDGE_METERS 1e9f                     // start edges of the seen area
#define NOT_ARRIVED_SECONDS -1.0f                   // time of an event that did not happen

// Bits of the state of a unit in a trace frame.
enum {
    TRACE_STOPPED = 1,
    TRACE_WAITING = 2,
    TRACE_AVOIDANCE_TURNING = 4,
    TRACE_TURNING_ON_SPOT = 8,
    TRACE_TOUCHING = 16,
    TRACE_ORCA_AVOIDING = 32,
};


// `measure_turn_on_spot` flags a unit of a moving squad that turns fast at a low speed, and adds
// the tick to its time of turns on the spot.
static void measure_turn_on_spot(Unit *unit, bool is_moving_squad, float speed) {
    // The angle change wraps into `[-PI, PI]`.
    float top_speed = unit->top_speed;
    float da = unit->angle - unit->previous_angle;        // angle change in this tick
    if (da > PI)
        da -= 2 * PI;
    if (da < -PI)
        da += 2 * PI;
    bool is_turning_fast = fabsf(da) > TURN_ON_SPOT_RADIANS_PER_SECOND * TICK_SECONDS;
    bool is_slow_for_turn_on_spot = speed < TURN_ON_SPOT_SPEED_FRACTION * top_speed;
    unit->is_turning_on_spot = is_moving_squad && is_turning_fast && is_slow_for_turn_on_spot;
    if (unit->is_turning_on_spot)
        unit->turn_on_spot_seconds += TICK_SECONDS;
    unit->previous_angle = unit->angle;
}


// `measure_unit` adds one tick to the counts and times of a unit: stops, slow and stuck time,
// time at top speed, turns on the spot, time in blocking terrain, blocking terrain slides and
// avoidance onsets.
static void measure_unit(Unit *unit) {
    bool is_moving_squad = squads[unit->squad_index].state == SQUAD_MOVING;
    float speed = len(unit->velocity);
    float top_speed = unit->top_speed;
    float slow_speed = SLOW_SPEED_FRACTION * top_speed;

    // A stop is a unit that was driving and then gets slow.
    float d_parking = len(sub(unit->position, unit->parking_position));
    float d_arriving = ARRIVING_RADII * unit->unit_type->radius;
    bool is_away_from_parking = d_parking > d_arriving;
    bool is_heading_to_parking = is_moving_squad && !unit->is_stopped && is_away_from_parking;
    bool is_slow = speed < slow_speed;
    if (speed > FAST_SPEED_FRACTION * top_speed)
        unit->was_fast = true;
    if (is_heading_to_parking && unit->was_fast && is_slow) {
        unit->stop_count++;
        unit->was_fast = false;
    }

    // The stuck time counts only the part of a slow stretch after its first
    // `STUCK_AFTER_SECONDS`.
    if (is_heading_to_parking && is_slow) {
        unit->slow_seconds += TICK_SECONDS;
        unit->slow_run_seconds += TICK_SECONDS;
        if (unit->slow_run_seconds > STUCK_AFTER_SECONDS)
            unit->stuck_seconds += TICK_SECONDS;
    } else {
        unit->slow_run_seconds = 0;
    }
    if (is_heading_to_parking) {
        unit->travel_seconds += TICK_SECONDS;
        if (speed >= TOP_SPEED_FRACTION * top_speed)
            unit->top_speed_seconds += TICK_SECONDS;
    }

    measure_turn_on_spot(unit, is_moving_squad, speed);

    // A body counts as in blocking terrain when a circle `TERRAIN_DEPTH_METERS` smaller is not
    // clear. Slides and avoidance count once per start, not per tick.
    float inner_radius = unit->unit_type->radius - TERRAIN_DEPTH_METERS;
    int terrain_class = unit->unit_type->terrain_class;
    if (is_moving_squad && !is_circle_clear(unit->position, inner_radius, terrain_class))
        unit->in_terrain_seconds += TICK_SECONDS;
    if (unit->is_terrain_sliding && !unit->was_terrain_sliding && is_moving_squad)
        unit->terrain_slide_count++;
    unit->was_terrain_sliding = unit->is_terrain_sliding;
    if (unit->is_orca_avoiding && !unit->was_orca_avoiding && is_moving_squad)
        unit->avoidance_onset_count++;
    unit->was_orca_avoiding = unit->is_orca_avoiding;
}


// `measure_contacts` flags the touches between the unit `unit_index` and each unit after it, and
// counts each new touch as a bump within the squad or a bump between squads, for both units.
static void measure_contacts(int unit_index) {
    // A unit inside an APC touches nothing.
    Unit *unit = &units[unit_index];
    if (unit->is_carried)
        return;
    for (int other_index = unit_index + 1; other_index < unit_count; other_index++) {
        Unit *other = &units[other_index];
        if (other->is_carried) {
            unit->is_touching[other_index] = false;
            continue;
        }
        float r_sum = unit->unit_type->radius + other->unit_type->radius;        // sum of radii
        float d_touch = r_sum + TOUCH_MARGIN_METERS;
        float d_squared = length_squared(sub(unit->position, other->position));
        bool is_touching = d_squared < d_touch * d_touch;

        // `is_touching` of the unit with the lower index holds the touch of each pair.
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


// `measure_spread` adds one tick to the spread of a squad: the largest distance of a squad member
// from the center of the squad, minus the same distance in the final formation.
static void measure_spread(Squad *squad) {
    int first_index = squad->first_unit_index;
    int end_index = squad->first_unit_index + squad->unit_count;
    V2 center = v2(0, 0);
    for (int unit_index = first_index; unit_index < end_index; unit_index++)
        center = add(center, units[unit_index].position);
    center = mul(center, 1.0f / squad->unit_count);

    float spread = 0;
    for (int unit_index = first_index; unit_index < end_index; unit_index++) {
        float d = len(sub(units[unit_index].position, center));        // distance to the center
        if (d > spread)
            spread = d;
    }

    spread -= squad->spread_reference;
    if (spread < 0)
        spread = 0;
    if (spread > squad->max_spread)
        squad->max_spread = spread;
    squad->spread_sum += spread;
    squad->spread_count++;
}


// `measure` adds the current tick to the metrics of every unit and of every moving squad.
void measure(void) {
    for (int unit_index = 0; unit_index < unit_count; unit_index++)
        units[unit_index].is_in_contact = false;

    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        measure_unit(&units[unit_index]);
        measure_contacts(unit_index);
    }
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (squads[squad_index].state == SQUAD_MOVING)
            measure_spread(&squads[squad_index]);
    }
}


// Bounding box of the unit positions and routes of the current trace run, for its grid.
static V2 seen_min;
static V2 seen_max;


// `extend_seen_area` grows the seen area to hold `point`.
static void extend_seen_area(V2 point) {
    seen_min.x = fminf(seen_min.x, point.x);
    seen_min.y = fminf(seen_min.y, point.y);
    seen_max.x = fmaxf(seen_max.x, point.x);
    seen_max.y = fmaxf(seen_max.y, point.y);
}


// `get_separator` returns the JSON separator before a list item: none before the first one.
static const char *get_separator(bool is_first_item) {
    return is_first_item ? "" : ",";
}


// `get_trace_bits` returns the state bits of a unit for a trace frame.
static int get_trace_bits(const Unit *unit) {
    int bits = 0;
    if (unit->is_stopped)
        bits |= TRACE_STOPPED;
    if (unit->wait_seconds > 0)
        bits |= TRACE_WAITING;
    if (unit->is_avoidance_turning)
        bits |= TRACE_AVOIDANCE_TURNING;
    if (unit->is_turning_on_spot)
        bits |= TRACE_TURNING_ON_SPOT;
    if (unit->is_in_contact)
        bits |= TRACE_TOUCHING;
    if (unit->is_orca_avoiding)
        bits |= TRACE_ORCA_AVOIDING;
    return bits;
}


// `write_trace_frame` writes the frame of the current tick: the time, then for each unit its
// position, angle, base game waypoint and state bits.
void write_trace_frame(bool is_first) {
    // The frames lie 0.1176 s apart at the game frame. With two decimals, the time between two
    // frames would round to 0.23 s and 0.24 s by turns, and the replay speeds would wobble by
    // about 2 % every 4 frames.
    fprintf(trace_file, "%s[%.4f", get_separator(is_first), now_seconds);
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


// `get_clamped_cell` returns the cell coordinate of a map coordinate, limited to the grid.
static int get_clamped_cell(float world_meters) {
    int cell = get_cell_coordinate(world_meters);
    if (cell < 0)
        return 0;
    return cell >= GC_GRID_SIZE ? GC_GRID_SIZE - 1 : cell;
}


// `write_trace_grid` writes the grid cells of the seen area and the routes, with a margin of
// `TRACE_GRID_MARGIN_METERS`. Each cell is one digit, its value in the grid, row by row.
void write_trace_grid(void) {
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

    fprintf(trace_file, ",\"grid\":{\"x0\":%d,\"y0\":%d,", min_cell_x, min_cell_y);
    fprintf(trace_file, "\"w\":%d,\"h\":%d,\"cells\":\"", width_cells, height_cells);
    for (int cell_y = min_cell_y; cell_y <= max_cell_y; cell_y++) {
        for (int cell_x = min_cell_x; cell_x <= max_cell_x; cell_x++)
            fputc('0' + gc_get_cell(cell_x, cell_y), trace_file);
    }
    fputc('"', trace_file);
    fputc('}', trace_file);
}


// `begin_trace_run` starts the trace object of a run: the scenario, the mode, the map, the cell
// size and the units, and opens the list of frames. It also empties the seen area.
void begin_trace_run(int scenario_index) {
    seen_min = v2(UNSEEN_EDGE_METERS, UNSEEN_EDGE_METERS);
    seen_max = v2(-UNSEEN_EDGE_METERS, -UNSEEN_EDGE_METERS);

    const char *scenario_name = SCENARIO_NAME[scenario_index];
    const char *description = SCENARIO_DESCRIPTION[scenario_index];
    fprintf(trace_file, "{\"name\":\"%s\",\"mode\":\"%s\",", scenario_name, MODE_NAME[avoid_mode]);
    fprintf(trace_file, "\"map\":\"%s\",", map_name);
    fprintf(trace_file, "\"desc\":\"%s\",\"cell\":%g,", description, GC_CELL_METERS);
    fprintf(trace_file, "\"units\":[");
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        const Unit *unit = &units[unit_index];
        const char *separator = get_separator(unit_index == 0);
        float radius = unit->unit_type->radius;
        float top_speed = unit->top_speed;
        fprintf(trace_file, "%s{\"squad\":%d,\"r\":%g,", separator, unit->squad_index, radius);
        fprintf(trace_file, "\"v\":%g}", top_speed);
    }
    fprintf(trace_file, "],\"frames\":[");
}


// `write_trace_lane` writes the points of the lane of a squad member every
// `TRACE_LANE_STEP_METERS` along the route, with `null` where the lanes do not fit.
static void write_trace_lane(const Squad *squad, const Unit *unit) {
    float route_length = squad->route_length;
    for (float d = 0; d <= route_length; d += TRACE_LANE_STEP_METERS) {
        const char *separator = get_separator(d <= 0);
        V2 point = get_lane_point(squad, unit, d);
        LaneCrossSection cs = get_lane_cross_section(squad, d);
        if (cs.is_wide_enough)
            fprintf(trace_file, "%s[%.1f,%.1f]", separator, point.x, point.y);
        else
            fprintf(trace_file, "%snull", separator);
    }
}


// `write_trace_squad_shape` writes the start of the trace object of a squad: its unit type, its
// order flag, its boarding flag when its members board an APC, its route and the final parking
// positions of its squad members. In the formation modes, a squad with an order also gets the
// lanes of its squad members.
static void write_trace_squad_shape(Squad *squad, int squad_index) {
    // Every member of a boarding squad has the same target, so the first member tells.
    const char *unit_type_name = squad->unit_type->name;
    fprintf(trace_file, "%s{\"type\":\"%s\",", get_separator(squad_index == 0), unit_type_name);
    if (units[squad->first_unit_index].target_unit_index != NO_TARGET_UNIT)
        fprintf(trace_file, "\"boarding\":1,");
    fprintf(trace_file, "\"moving\":%d,\"route\":[", has_order[squad_index]);
    for (int point_index = 0; point_index < squad->route_count; point_index++) {
        V2 point = squad->route[point_index];
        const char *separator = get_separator(point_index == 0);
        fprintf(trace_file, "%s[%.1f,%.1f]", separator, point.x, point.y);
    }

    int first_index = squad->first_unit_index;
    int end_index = squad->first_unit_index + squad->unit_count;
    fprintf(trace_file, "],\"parking\":[");
    for (int unit_index = first_index; unit_index < end_index; unit_index++) {
        V2 parking_position = units[unit_index].parking_position;
        const char *separator = get_separator(unit_index == first_index);
        fprintf(trace_file, "%s[%.1f,%.1f]", separator, parking_position.x, parking_position.y);
    }
    fprintf(trace_file, "]");
    bool has_lanes = is_formation_mode() && are_lanes_on && has_order[squad_index];
    if (!has_lanes)
        return;

    // The lanes are those of the square formation, so column formation is off while they are
    // written. A squad in column formation would otherwise give every lane the route middle.
    bool was_in_column = squad->is_in_column;
    squad->is_in_column = false;
    fprintf(trace_file, ",\"lanes\":[");
    for (int unit_index = first_index; unit_index < end_index; unit_index++) {
        fprintf(trace_file, "%s[", get_separator(unit_index == first_index));
        write_trace_lane(squad, &units[unit_index]);
        fputc(']', trace_file);
    }
    fputc(']', trace_file);
    squad->is_in_column = was_in_column;
}


// Results of one squad with an order, at the end of a run. "Per member" values are averages over
// its squad members.
typedef struct {
    float arrive_seconds;
    float stops_per_member;
    float slow_seconds_per_member;
    float stuck_seconds_per_member;
    float turn_on_spot_seconds_per_member;
    int wait_count;
    float avoidance_turn_seconds;
    float orca_avoidance_seconds;
    int own_bump_count;
    int other_bump_count;
    int terrain_slide_count;
    float in_terrain_seconds_per_member;
    float avoidance_onset_count_per_member;
    float top_speed_percent;
    float start_seconds;
    float mean_spread;
    float parking_error_per_member;
    float parking_error_max;
    int lost_count;
    const char *state_text;
} ResultRow;


// Sums of the counts and times of the squad members of one squad, and their parking errors: the
// distances of the units from their final parking positions.
typedef struct {
    float parking_error_sum;
    float parking_error_max;
    int stop_count;
    int wait_count;
    int own_bump_sum;
    int other_bump_count;
    int lost_count;
    int terrain_slide_count;
    int avoidance_onset_count;
    float slow_seconds;
    float stuck_seconds;
    float travel_seconds;
    float top_speed_seconds;
    float avoidance_turn_seconds;
    float orca_avoidance_seconds;
    float turn_on_spot_seconds;
    float in_terrain_seconds;
} MemberSums;


// `get_state_text` returns the note of the result row for a squad that was denied a route or did
// not arrive, or an empty text.
static const char *get_state_text(const Squad *squad) {
    if (squad->state == SQUAD_DENIED)
        return " DENIED";
    if (squad->state != SQUAD_ARRIVED)
        return " NOT ARRIVED";
    return "";
}


// `add_up_members` adds up the counts and times of the squad members of a squad, and their
// parking errors. A unit further than `LOST_RADII` radii from its final parking position is lost.
static MemberSums add_up_members(const Squad *squad) {
    MemberSums sums = {0};
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        float parking_error = len(sub(unit->position, unit->parking_position));
        sums.parking_error_sum += parking_error;
        if (parking_error > sums.parking_error_max)
            sums.parking_error_max = parking_error;
        bool is_lost = parking_error > LOST_RADII * unit->unit_type->radius;
        sums.lost_count += is_lost;
        sums.stop_count += unit->stop_count;
        sums.wait_count += unit->wait_count;
        sums.own_bump_sum += unit->own_bump_count;
        sums.other_bump_count += unit->other_bump_count;
        sums.terrain_slide_count += unit->terrain_slide_count;
        sums.avoidance_onset_count += unit->avoidance_onset_count;
        sums.in_terrain_seconds += unit->in_terrain_seconds;
        sums.slow_seconds += unit->slow_seconds;
        sums.stuck_seconds += unit->stuck_seconds;
        sums.travel_seconds += unit->travel_seconds;
        sums.top_speed_seconds += unit->top_speed_seconds;
        sums.avoidance_turn_seconds += unit->avoidance_turn_seconds;
        sums.orca_avoidance_seconds += unit->orca_avoidance_seconds;
        sums.turn_on_spot_seconds += unit->turn_on_spot_seconds;
    }
    return sums;
}


// `make_result_row` builds the result row of a squad from the sums over its squad members and
// from its own times. The start and arrival times count from the order.
static ResultRow make_result_row(const Squad *squad) {
    MemberSums sums = add_up_members(squad);

    // Times of events that did not happen stay at `NOT_ARRIVED_SECONDS`.
    float start_seconds = NOT_ARRIVED_SECONDS;
    if (squad->start_time_seconds != NOT_STARTED_SECONDS)
        start_seconds = squad->start_time_seconds - squad->order_time_seconds;
    float arrive_seconds = NOT_ARRIVED_SECONDS;
    if (squad->state == SQUAD_ARRIVED)
        arrive_seconds = squad->arrive_time_seconds - squad->order_time_seconds;
    float mean_spread = 0;
    if (squad->spread_count)
        mean_spread = squad->spread_sum / squad->spread_count;
    float top_speed_percent = 0;
    if (sums.travel_seconds > 0)
        top_speed_percent = PERCENT * sums.top_speed_seconds / sums.travel_seconds;

    // Both units of a bump within the squad count it, so the sum counts each bump twice.
    float n = (float)squad->unit_count;        // number of squad members
    return (ResultRow){
        .arrive_seconds = arrive_seconds,
        .stops_per_member = sums.stop_count / n,
        .slow_seconds_per_member = sums.slow_seconds / n,
        .stuck_seconds_per_member = sums.stuck_seconds / n,
        .turn_on_spot_seconds_per_member = sums.turn_on_spot_seconds / n,
        .wait_count = sums.wait_count,
        .avoidance_turn_seconds = sums.avoidance_turn_seconds,
        .orca_avoidance_seconds = sums.orca_avoidance_seconds,
        .own_bump_count = sums.own_bump_sum / 2,
        .other_bump_count = sums.other_bump_count,
        .terrain_slide_count = sums.terrain_slide_count,
        .in_terrain_seconds_per_member = sums.in_terrain_seconds / n,
        .avoidance_onset_count_per_member = sums.avoidance_onset_count / n,
        .top_speed_percent = top_speed_percent,
        .start_seconds = start_seconds,
        .mean_spread = mean_spread,
        .parking_error_per_member = sums.parking_error_sum / n,
        .parking_error_max = sums.parking_error_max,
        .lost_count = sums.lost_count,
        .state_text = get_state_text(squad),
    };
}


// `print_result_header` prints the column names of the result rows.
void print_result_header(void) {
    printf("%-13s %-8s %s %-13s %2s ", "scen", "mode", "q", "type", "n");
    printf("%6s %5s %6s %6s %6s ", "arrive", "stops", "slow_s", "stuck", "onspot");
    printf("%5s %6s %6s ", "waits", "avturn", "orca");
    printf("%4s %4s %7s ", "bmpo", "bmpx", "terrain");
    printf("%9s %5s %6s %5s %5s ", "interrain", "sep", "sepmax", "park", "parkx");
    printf("%4s %4s %5s %5s %5s %5s\n", "lost", "xing", "swaps", "onset", "top", "start");
}


// `print_result_row` prints the result row of a squad, in the columns of `print_result_header`.
static void print_result_row(int scenario_index, int squad_index, const Squad *squad,
                             const ResultRow *row) {
    printf("%-13s %-8s %d ", SCENARIO_NAME[scenario_index], MODE_NAME[avoid_mode], squad_index);
    printf("%-13s %2d ", squad->unit_type->name, squad->unit_count);

    printf("%6.1f %5.2f ", row->arrive_seconds, row->stops_per_member);
    printf("%6.1f %6.1f ", row->slow_seconds_per_member, row->stuck_seconds_per_member);
    printf("%6.1f %5d ", row->turn_on_spot_seconds_per_member, row->wait_count);
    printf("%6.1f %6.1f ", row->avoidance_turn_seconds, row->orca_avoidance_seconds);
    printf("%4d %4d %7d ", row->own_bump_count, row->other_bump_count, row->terrain_slide_count);
    printf("%9.1f %5.1f ", row->in_terrain_seconds_per_member, row->mean_spread);
    printf("%6.1f %5.1f ", squad->max_spread, row->parking_error_per_member);
    printf("%5.1f %4d ", row->parking_error_max, row->lost_count);
    printf("%4d %5d ", squad->crossing_count, squad->swap_count);
    printf("%5.1f %5.1f ", row->avoidance_onset_count_per_member, row->top_speed_percent);
    printf("%5.2f%s\n", row->start_seconds, row->state_text);
}


// `write_trace_result` writes the results of a squad and closes its trace object.
static void write_trace_result(const Squad *squad, const ResultRow *row) {
    fprintf(trace_file, ",\"arrive\":%.1f", row->arrive_seconds);
    fprintf(trace_file, ",\"stops\":%.2f", row->stops_per_member);
    fprintf(trace_file, ",\"slow\":%.1f", row->slow_seconds_per_member);
    fprintf(trace_file, ",\"stuck\":%.1f", row->stuck_seconds_per_member);
    fprintf(trace_file, ",\"onspot\":%.1f", row->turn_on_spot_seconds_per_member);
    fprintf(trace_file, ",\"waits\":%d", row->wait_count);
    fprintf(trace_file, ",\"avoid_turn\":%.1f", row->avoidance_turn_seconds);
    fprintf(trace_file, ",\"orca\":%.1f", row->orca_avoidance_seconds);
    fprintf(trace_file, ",\"bumps_own\":%d", row->own_bump_count);
    fprintf(trace_file, ",\"bumps_other\":%d", row->other_bump_count);
    fprintf(trace_file, ",\"terrain\":%d", row->terrain_slide_count);
    fprintf(trace_file, ",\"in_terrain\":%.1f", row->in_terrain_seconds_per_member);
    fprintf(trace_file, ",\"spread\":%.1f", row->mean_spread);
    fprintf(trace_file, ",\"spread_max\":%.1f", squad->max_spread);
    fprintf(trace_file, ",\"park_err\":%.1f", row->parking_error_per_member);
    fprintf(trace_file, ",\"park_err_max\":%.1f", row->parking_error_max);
    fprintf(trace_file, ",\"lost\":%d", row->lost_count);
    fprintf(trace_file, ",\"onset_count\":%.1f", row->avoidance_onset_count_per_member);
    fprintf(trace_file, ",\"top_speed\":%.1f", row->top_speed_percent);
    fprintf(trace_file, ",\"start\":%.2f", row->start_seconds);
    fprintf(trace_file, ",\"swaps\":%d}", squad->swap_count);
}


// `report_squad` reports a squad at the end of a run: its trace object when a trace is open, and
// its result row when it has an order.
void report_squad(int scenario_index, int squad_index) {
    Squad *squad = &squads[squad_index];
    if (trace_file)
        write_trace_squad_shape(squad, squad_index);
    if (!has_order[squad_index]) {
        if (trace_file)
            fputc('}', trace_file);
        return;
    }

    ResultRow row = make_result_row(squad);
    print_result_row(scenario_index, squad_index, squad, &row);
    if (trace_file)
        write_trace_result(squad, &row);
}
