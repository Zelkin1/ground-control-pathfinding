// Entry point and main loop of the simulator. `main` reads the command line, maps gc.exe, and runs
// the chosen scenarios in the chosen modes, or the route finder (`--find`). Each run gives the
// orders in its first tick and then simulates ticks of `TICK_SECONDS` until no squad is busy for
// `SETTLE_SECONDS`, or until `MAX_RUN_SECONDS`. Each run prints one result row per squad on
// stdout, and its tick times on stderr. With `--trace`, the runs also go into a JSON trace file
// for the replay.
//
// The switches of the command line set the global flags below: a switch name followed by `off`
// turns its feature off, and any other word turns it on (docs/SWITCHES.md). Unknown words are
// ignored, unless they name a scenario.
#include <stdlib.h>
#include <string.h>

#include "sim.h"
#include "formation.h"

#define MAX_RUN_SECONDS 240.0f             // longest simulated time of one run
#define SETTLE_SECONDS 3.0f                // time after the last busy squad until the run ends
#define TRACE_FRAME_SECONDS 0.12f          // time between two trace frames, rounded to ticks
#define ALL_SCENARIOS (-1)                 // no single scenario chosen
#define MICROSECONDS_PER_SECOND 1e6        // microseconds in one second

float tick_seconds = GAME_FRAME_SECONDS;
Unit units[MAX_UNITS];
Squad squads[MAX_SQUADS];
int unit_count;
int squad_count;
float now_seconds;
float path_points[2 * MAX_WAYPOINTS];
FILE *trace_file;
int avoid_mode;
const char *MODE_NAME[] = {
    [AVOID_ORIGINAL] = "original",
    [AVOID_FORM] = "form",
    [AVOID_COORD] = "coord",
};

static unsigned seed;


// Choices of the command line other than the switches.
typedef struct {
    const char *exe_path;
    int only_scenario_index;
    int first_mode;
    int last_mode;
    const char *find_map_name;
    int find_try_count;
} CommandLine;


// Tick times of one run: the moving ticks, and the slowest order tick. An order tick is the first
// tick, or a tick that starts while a squad searches its route.
typedef struct {
    double move_sum_seconds;
    double move_max_seconds;
    int move_tick_count;
    double order_max_seconds;
} TickTimes;

// `is_any_squad_busy` tells whether a squad still moves or searches.
static bool is_any_squad_busy(void) {
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (is_squad_busy(&squads[squad_index]))
            return true;
    }
    return false;
}


// `is_any_unit_chasing` tells whether a unit still drives to board an APC.
static bool is_any_unit_chasing(void) {
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        if (is_chasing(&units[unit_index]))
            return true;
    }
    return false;
}


// `is_any_squad_searching` tells whether a squad searches its route.
static bool is_any_squad_searching(void) {
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (squads[squad_index].state == SQUAD_SEARCHING)
            return true;
    }
    return false;
}


// `add_tick_time` adds the time of one tick to the order ticks or to the moving ticks.
static void add_tick_time(TickTimes *times, double tick_seconds, bool is_order_tick) {
    if (is_order_tick) {
        times->order_max_seconds = fmax(times->order_max_seconds, tick_seconds);
        return;
    }
    times->move_sum_seconds += tick_seconds;
    times->move_max_seconds = fmax(times->move_max_seconds, tick_seconds);
    times->move_tick_count++;
}


// `get_trace_every_ticks` returns the number of ticks between two trace frames: the ticks in
// `TRACE_FRAME_SECONDS`, at least one, so that a short tick does not make the trace larger.
static int get_trace_every_ticks(void) {
    int tick_count = (int)lroundf(TRACE_FRAME_SECONDS / TICK_SECONDS);
    return tick_count > 1 ? tick_count : 1;
}


// `print_tick_times` prints the tick times of a run on stderr, in microseconds.
static void print_tick_times(int scenario_index, const TickTimes *times) {
    int tick_count = times->move_tick_count > 0 ? times->move_tick_count : 1;
    double mean_microseconds = MICROSECONDS_PER_SECOND * times->move_sum_seconds / tick_count;
    double max_microseconds = MICROSECONDS_PER_SECOND * times->move_max_seconds;
    double order_microseconds = MICROSECONDS_PER_SECOND * times->order_max_seconds;
    fprintf(stderr, "timing %s %s: moving ticks mean %.0f us, max %.0f us; ",
            SCENARIO_NAME[scenario_index], MODE_NAME[avoid_mode], mean_microseconds,
            max_microseconds);
    fprintf(stderr, "order ticks max %.0f us\n", order_microseconds);
}


// `give_order` orders the squad `squad_index` to its destination, with the facing of the order
// when it has one.
static void give_order(int squad_index) {
    Squad *squad = &squads[squad_index];
    order_move(squad, order_destinations[squad_index]);
    V2 order_facing = order_facings[squad_index];
    if (length_squared(order_facing) > 0)
        squad->facing = order_facing;
}


// `is_first_tick_order` tells whether the squad `squad_index` gets its order in the first tick.
static bool is_first_tick_order(int squad_index) {
    return has_order[squad_index] && order_seconds[squad_index] <= 0;
}


// `give_orders` gives each squad of the scenario whose order has no time its order.
static void give_orders(void) {
    // Every ordered squad counts as busy before the first search, so that no search stamps another
    // ordered squad as parked.
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (is_first_tick_order(squad_index))
            squads[squad_index].state = SQUAD_SEARCHING;
    }
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (is_first_tick_order(squad_index))
            give_order(squad_index);
    }
}


// `give_late_orders` gives each squad whose order has a time its order in the first tick at or
// after that time, once.
static void give_late_orders(void) {
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        float seconds = order_seconds[squad_index];        // time of the order
        bool is_due = seconds > 0 && now_seconds >= seconds;
        if (!has_order[squad_index] || !is_due)
            continue;
        order_seconds[squad_index] = 0;
        give_order(squad_index);
    }
}


// `give_second_orders` gives each squad its second order in the first tick at or after the time of
// that order, once.
static void give_second_orders(void) {
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        bool is_due = now_seconds >= second_order_seconds[squad_index];
        if (!has_second_order[squad_index] || !is_due)
            continue;
        has_second_order[squad_index] = false;
        order_move(&squads[squad_index], second_order_destinations[squad_index]);
    }
}


// `give_timed_orders` gives each timed order of the scenario in the first tick at or after its
// time, once.
static void give_timed_orders(void) {
    for (int order_index = 0; order_index < timed_order_count; order_index++) {
        TimedOrder *order = &timed_orders[order_index];
        if (order->is_given || now_seconds < order->seconds)
            continue;
        order->is_given = true;
        order_move(&squads[order->squad_index], order->destination);
    }
}


// `has_pending_timed_order` tells whether a timed order of the scenario is still to come.
static bool has_pending_timed_order(void) {
    for (int order_index = 0; order_index < timed_order_count; order_index++) {
        if (!timed_orders[order_index].is_given)
            return true;
    }
    return false;
}


// `run_tick` simulates one tick: the speed effects, the orders, the coordination between squads,
// the squads, the parked boxes of the ORCA avoidance, and then the units.
static void run_tick(bool is_first_tick) {
    update_speed_effects();
    if (is_first_tick)
        give_orders();
    give_late_orders();
    give_second_orders();
    give_timed_orders();
    if (is_formation_mode())
        coordinate_squads();
    reset_set_up_work();
    for (int squad_index = 0; squad_index < squad_count; squad_index++)
        update_squad(&squads[squad_index]);
    finish_threaded_set_ups();
    update_parked_boxes();
    for (int unit_index = 0; unit_index < unit_count; unit_index++)
        tick_mover(unit_index);
}


// `run_scenario` runs one scenario in the current mode and reports its squads. The tick time
// covers the simulation of the tick, but not the metrics and the trace.
static void run_scenario(int scenario_index) {
    set_up_scenario(scenario_index, seed);
    if (trace_file)
        begin_trace_run(scenario_index);

    // The run ends `SETTLE_SECONDS` after the first tick without a busy squad, a unit that drives
    // to board an APC or a timed order still to come.
    bool is_settling = false;
    float idle_since_seconds = 0;
    int tick_index = 0;
    TickTimes times = {
        .move_sum_seconds = 0,
        .move_max_seconds = 0,
        .move_tick_count = 0,
        .order_max_seconds = 0,
    };
    for (now_seconds = 0; now_seconds < MAX_RUN_SECONDS; now_seconds += TICK_SECONDS) {
        double tick_start_seconds = read_clock_seconds();
        bool is_order_tick = tick_index == 0 || is_any_squad_searching();
        run_tick(tick_index == 0);
        double tick_seconds = read_clock_seconds() - tick_start_seconds;
        add_tick_time(&times, tick_seconds, is_order_tick);

        measure();
        if (trace_file && tick_index % get_trace_every_ticks() == 0)
            write_trace_frame(tick_index == 0);
        bool is_any_busy = is_any_squad_busy() || is_any_unit_chasing();
        is_any_busy = is_any_busy || has_pending_timed_order();
        if (!is_any_busy && !is_settling) {
            is_settling = true;
            idle_since_seconds = now_seconds;
        }
        if (is_settling && now_seconds > idle_since_seconds + SETTLE_SECONDS)
            break;
        tick_index++;
    }

    // The trace of a run ends with its squads and its map grid.
    print_tick_times(scenario_index, &times);
    if (trace_file)
        fprintf(trace_file, "],\"squads\":[");
    for (int squad_index = 0; squad_index < squad_count; squad_index++)
        report_squad(scenario_index, squad_index);
    if (trace_file) {
        fputc(']', trace_file);
        write_trace_grid();
        fputc('}', trace_file);
    }
}


// `is_switch_on` tells whether the value of a switch turns it on: any word but `off` does.
static bool is_switch_on(const char *value) {
    return strcmp(value, "off") != 0;
}


// `find_name_index` returns the index of `value` among the `name_count` names of `names`, or
// `fallback` when no name matches.
static int find_name_index(const char *value, const char *const *names, int name_count,
                           int fallback) {
    for (int name_index = 0; name_index < name_count; name_index++) {
        if (!strcmp(value, names[name_index]))
            return name_index;
    }
    return fallback;
}


// `parse_variant` reads the value of the scenario variant `argument` into its global, and returns
// false when `argument` names no variant. An unknown value keeps the variant unchanged.
static bool parse_variant(const char *argument, const char *value) {
    if (!strcmp(argument, "--faction"))
        scenario_faction = find_name_index(value, FACTION_NAME, FACTION_COUNT, scenario_faction);
    else if (!strcmp(argument, "--layout"))
        scenario_layout = find_name_index(value, LAYOUT_NAME, LAYOUT_COUNT, scenario_layout);
    else if (!strcmp(argument, "--profile"))
        scenario_profile = find_name_index(value, PROFILE_NAME, PROFILE_COUNT, scenario_profile);
    else if (!strcmp(argument, "--members"))
        max_member_count = atoi(value);
    else if (!strcmp(argument, "--boost"))
        boost_start_seconds = (float)atof(value);
    else if (!strcmp(argument, "--boostcycle"))
        boost_cycle_seconds = (float)atof(value);
    else if (!strcmp(argument, "--booststagger"))
        boost_stagger_seconds = (float)atof(value);
    else if (!strcmp(argument, "--tick"))
        tick_seconds = (float)atof(value);
    else
        return false;
    return true;
}


// `parse_command_line` reads the command line into the switches, the scenario variants and
// `command_line`. It returns false when the trace file cannot be opened. A word that names no
// option or scenario is ignored, so a removed switch has no effect.
static bool parse_command_line(int argc, char **argv, CommandLine *command_line) {
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        // An option without its value at the end of the line is ignored.
        const char *argument = argv[argument_index];
        bool has_value = argument_index + 1 < argc;
        bool has_two_values = argument_index + 2 < argc;
        bool *on_off_value = find_on_off_switch(argument);
        if (on_off_value && has_value) {
            argument_index++;
            *on_off_value = is_switch_on(argv[argument_index]);
        } else if (has_value && parse_variant(argument, argv[argument_index + 1])) {
            argument_index++;
        } else if (!strcmp(argument, "--exe") && has_value) {
            argument_index++;
            command_line->exe_path = argv[argument_index];
        } else if (!strcmp(argument, "--maps") && has_value) {
            argument_index++;
            maps_dir = argv[argument_index];
        } else if (!strcmp(argument, "--find") && has_two_values) {
            command_line->find_map_name = argv[argument_index + 1];
            command_line->find_try_count = atoi(argv[argument_index + 2]);
            argument_index += 2;
        } else if (!strcmp(argument, "--avoid") && has_value) {
            argument_index++;
            for (int mode = AVOID_ORIGINAL; mode <= AVOID_COORD; mode++) {
                if (!strcmp(argv[argument_index], MODE_NAME[mode])) {
                    command_line->first_mode = mode;
                    command_line->last_mode = mode;
                }
            }
        } else if (!strcmp(argument, "--seed") && has_value) {
            argument_index++;
            seed = (unsigned)atoi(argv[argument_index]);
        } else if (!strcmp(argument, "--trace") && has_value) {
            argument_index++;
            trace_file = fopen(argv[argument_index], "w");
            if (!trace_file) {
                fprintf(stderr, "cannot write %s\n", argv[argument_index]);
                return false;
            }
        } else {
            for (int scenario_index = 0; scenario_index < SCENARIO_COUNT; scenario_index++) {
                if (!strcmp(argument, SCENARIO_NAME[scenario_index]))
                    command_line->only_scenario_index = scenario_index;
            }
        }
    }
    return true;
}


// `run_chosen_scenarios` runs the chosen scenarios in `SCENARIO_RUN_ORDER`, each in the modes from
// `first_mode` to `last_mode`, and writes the frame of the trace file around the runs.
static void run_chosen_scenarios(int only_scenario_index, int first_mode, int last_mode) {
    if (trace_file) {
        float frame_seconds = get_trace_every_ticks() * TICK_SECONDS;
        fprintf(trace_file, "{\"dt\":%g,\"frame_dt\":%g,\"runs\":[", TICK_SECONDS, frame_seconds);
    }
    print_result_header();

    // The runs in the trace are separated by commas.
    bool are_all_scenarios_chosen = only_scenario_index == ALL_SCENARIOS;
    int run_count = 0;
    for (int order_index = 0; order_index < SCENARIO_COUNT; order_index++) {
        int scenario_index = SCENARIO_RUN_ORDER[order_index];
        for (avoid_mode = first_mode; avoid_mode <= last_mode; avoid_mode++) {
            if (!are_all_scenarios_chosen && only_scenario_index != scenario_index)
                continue;
            if (trace_file && run_count > 0)
                fputc(',', trace_file);
            run_count++;
            run_scenario(scenario_index);
        }
    }

    if (trace_file) {
        fputs("]}\n", trace_file);
        fclose(trace_file);
    }
}


// `main` runs the simulator. Without `--exe`, it maps gc.exe from the default folder of the Steam
// edition, and without `--avoid`, it runs every mode.
int main(int argc, char **argv) {
    CommandLine command_line = {
        .exe_path = ("C:\\Program Files (x86)\\Steam\\steamapps\\common\\"
                     "Ground Control Anthology\\gc.exe"),
        .only_scenario_index = ALL_SCENARIOS,
        .first_mode = AVOID_ORIGINAL,
        .last_mode = AVOID_COORD,
        .find_map_name = NULL,
        .find_try_count = 0,
    };
    if (!parse_command_line(argc, argv, &command_line))
        return EXIT_FAILURE;

    if (!gc_load(command_line.exe_path))
        return EXIT_FAILURE;
    allocate_lanes();
    if (command_line.find_map_name) {
        find_spots(command_line.find_map_name, command_line.find_try_count);
        return EXIT_SUCCESS;
    }

    int only_scenario_index = command_line.only_scenario_index;
    run_chosen_scenarios(only_scenario_index, command_line.first_mode, command_line.last_mode);
    return EXIT_SUCCESS;
}
