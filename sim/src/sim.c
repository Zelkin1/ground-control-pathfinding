/* This program is an offline traffic simulator for the unit movement of Ground Control. This file
 * reads the command line and runs the scenarios.
 *
 * Game code: The path search, the raycast, the cell lookup and the formation layout are the real
 * gc.exe functions (game.c). The squad and Mover logic around them is a C reconstruction of the
 * decompiled game code (squad.c and movement.c). Mover is the object of the game that moves one
 * unit. Each reconstructed function that follows a game function names it. re/PATHFINDING.md
 * describes the game code, and sim.h lists the other parts. The simulator does not model
 * weapons, targets and stances (there are no enemies), aircraft, terrain height or buildings.
 *
 * Usage: sim.exe [--exe PATH] [--maps DIR] [--avoid game|form|coord] [--lanes on|off]
 *                [--reassign on|off] [--pursuit on|off] [--rejoinroom on|off]
 *                [--smooth on|off] [--wideturns on|off]
 *                [--yield on|off] [--crossing on|off] [--keepright on|off] [--turns on|off]
 *                [--speedfirst on|off] [--smoothsway on|off] [--replan on|off]
 *                [--linearsteer on|off] [--bendlook on|off] [--turnslow on|off]
 *                [--aheadslow on|off] [--endstop on|off] [--overtake on|off]
 *                [--centersearch on|off] [--lateyield on|off] [--formfirst on|off]
 *                [--keeproute on|off] [--parkreplan on|off] [--leaderfile on|off]
 *                [--parkleeway on|off] [--endcap on|off] [--cutcorners on|off]
 *                [--shortreturn on|off] [--lateshift on|off] [--partialshift on|off]
 *                [--widegap on|off] [--parkspeed on|off] [--setupslice on|off]
 *                [--setupchain on|off] [--widenstop on|off] [--keepfollow on|off]
 *                [--waitahead on|off] [--reformmatch on|off] [--fullsearch on|off]
 *                [--setupthreads on|off] [--keepspeed on|off] [--parkahead on|off]
 *                [--crossfollow on|off] [--oldroute on|off] [--slotpoint on|off]
 *                [--seed N] [--trace FILE.json]
 *                [scenario]
 *        sim.exe --find MAP N [--findinfantry on|off]
 * Without a scenario, the simulator runs all of them, and without --avoid, it runs each scenario
 * in every mode. sim.h describes each on/off switch next to the global that it sets.
 *
 * Output: The simulator prints one row per squad, and --trace writes a JSON replay of every run.
 * On stderr, it prints one line per run with the wall-clock time per tick. With --find, it makes
 * N tries at a random route on the map MAP instead, and measures each route that the search of
 * the game finds. This helps to pick places for scenarios. */
#include <stdlib.h>
#include <string.h>

#include "sim.h"
#include "formation.h"

#define MAX_RUN_SECONDS 240.0f /* A run ends after this much simulated time at the latest. */
#define SETTLE_SECONDS 3.0f    /* A run goes on this long after no squad is busy any more. */
#define TRACE_EVERY_TICKS 3    /* The trace keeps every third tick. */
#define ALL_SCENARIOS (-1)     /* With this as the chosen scenario, every scenario runs. */
#define MICROSECONDS_PER_SECOND 1e6 /* Tick times are printed in microseconds. */

Unit units[MAX_UNITS];
Squad squads[MAX_SQUADS];
int unit_count;
int squad_count;
float now_seconds;
float path_points[2 * MAX_WAYPOINTS];
FILE *trace_file;
int avoid_mode;
const char *MODE_NAME[] = {
    [AVOID_GAME] = "game",
    [AVOID_FORM] = "form",
    [AVOID_COORD] = "coord",
};
bool are_lanes_on = true;
bool is_reassign_on = true;
bool is_yield_on = true;
bool is_smooth_on = true;
bool is_wide_turn_on = true;
bool is_pursuit_on = true;
bool is_rejoin_room_on = true;
bool is_crossing_yield_on = true;
bool is_keep_right_on = true;
bool is_turn_taking_on = true;
bool is_speed_priority_on = true;
bool is_smooth_sway_on = true;
bool is_find_infantry_on = false;
bool is_replan_on = true;
bool is_linear_steer_on = true;
bool is_bend_look_on = true;
bool is_turn_slowdown_on = false;
bool is_ahead_slowdown_on = true;
bool is_end_stop_on = true;
bool is_overtake_on = true;
bool is_center_search_on = true;
bool is_late_yield_on = true;
bool is_form_first_on = true;
bool is_keep_route_on = true;
bool is_park_replan_on = true;
bool is_leader_file_on = false;
bool is_park_leeway_on = true;
bool is_end_cap_on = true;
bool is_corner_cut_on = true;
bool is_short_return_on = true;
bool is_late_shift_on = true;
bool is_partial_shift_on = true;
bool is_wide_gap_on = true;
bool is_park_speed_on = true;
bool is_set_up_slice_on = false;
bool is_set_up_chain_on = true;
bool is_widen_stop_on = true;
bool is_keep_follow_on = true;
bool is_wait_ahead_on = true;
bool is_reform_match_on = true;
bool is_full_search_on = true;
bool is_set_up_threads_on = true;
bool is_keep_speed_on = true;
bool is_park_ahead_on = true;
bool is_cross_follow_on = true;
bool is_old_route_on = true;
bool is_slot_point_on = true;

/* `seed` is the value of `--seed`. With 0, the runs use the starts that the scenarios set. */
static unsigned seed;

/* A `CommandLine` holds the choices of the command line, apart from those that set globals. */
typedef struct {
    const char *exe_path;
    int only_scenario_index;   /* It is `ALL_SCENARIOS` without a scenario name. */
    int first_mode;            /* The runs use the `avoid_mode` values from this one on. */
    int last_mode;             /* They use this value last. */
    const char *find_map_name; /* It is NULL without `--find`. */
    int find_sample_count;     /* `--find` makes this many tries at a random route. */
} CommandLine;

/* An `OnOffSwitch` is a switch of the command line that takes "on" or "off", and the global that
 * it sets. */
typedef struct {
    const char *name;
    bool *value;
} OnOffSwitch;

static const OnOffSwitch ON_OFF_SWITCHES[] = {
    {
        .name = "--pursuit",
        .value = &is_pursuit_on,
    },
    {
        .name = "--rejoinroom",
        .value = &is_rejoin_room_on,
    },
    {
        .name = "--smooth",
        .value = &is_smooth_on,
    },
    {
        .name = "--wideturns",
        .value = &is_wide_turn_on,
    },
    {
        .name = "--yield",
        .value = &is_yield_on,
    },
    {
        .name = "--reassign",
        .value = &is_reassign_on,
    },
    {
        .name = "--lanes",
        .value = &are_lanes_on,
    },
    {
        .name = "--crossing",
        .value = &is_crossing_yield_on,
    },
    {
        .name = "--keepright",
        .value = &is_keep_right_on,
    },
    {
        .name = "--turns",
        .value = &is_turn_taking_on,
    },
    {
        .name = "--speedfirst",
        .value = &is_speed_priority_on,
    },
    {
        .name = "--smoothsway",
        .value = &is_smooth_sway_on,
    },
    {
        .name = "--findinfantry",
        .value = &is_find_infantry_on,
    },
    {
        .name = "--replan",
        .value = &is_replan_on,
    },
    {
        .name = "--linearsteer",
        .value = &is_linear_steer_on,
    },
    {
        .name = "--bendlook",
        .value = &is_bend_look_on,
    },
    {
        .name = "--turnslow",
        .value = &is_turn_slowdown_on,
    },
    {
        .name = "--aheadslow",
        .value = &is_ahead_slowdown_on,
    },
    {
        .name = "--endstop",
        .value = &is_end_stop_on,
    },
    {
        .name = "--overtake",
        .value = &is_overtake_on,
    },
    {
        .name = "--centersearch",
        .value = &is_center_search_on,
    },
    {
        .name = "--lateyield",
        .value = &is_late_yield_on,
    },
    {
        .name = "--formfirst",
        .value = &is_form_first_on,
    },
    {
        .name = "--keeproute",
        .value = &is_keep_route_on,
    },
    {
        .name = "--parkreplan",
        .value = &is_park_replan_on,
    },
    {
        .name = "--leaderfile",
        .value = &is_leader_file_on,
    },
    {
        .name = "--parkleeway",
        .value = &is_park_leeway_on,
    },
    {
        .name = "--endcap",
        .value = &is_end_cap_on,
    },
    {
        .name = "--cutcorners",
        .value = &is_corner_cut_on,
    },
    {
        .name = "--shortreturn",
        .value = &is_short_return_on,
    },
    {
        .name = "--lateshift",
        .value = &is_late_shift_on,
    },
    {
        .name = "--partialshift",
        .value = &is_partial_shift_on,
    },
    {
        .name = "--widegap",
        .value = &is_wide_gap_on,
    },
    {
        .name = "--parkspeed",
        .value = &is_park_speed_on,
    },
    {
        .name = "--setupslice",
        .value = &is_set_up_slice_on,
    },
    {
        .name = "--setupchain",
        .value = &is_set_up_chain_on,
    },
    {
        .name = "--widenstop",
        .value = &is_widen_stop_on,
    },
    {
        .name = "--keepfollow",
        .value = &is_keep_follow_on,
    },
    {
        .name = "--waitahead",
        .value = &is_wait_ahead_on,
    },
    {
        .name = "--reformmatch",
        .value = &is_reform_match_on,
    },
    {
        .name = "--fullsearch",
        .value = &is_full_search_on,
    },
    {
        .name = "--setupthreads",
        .value = &is_set_up_threads_on,
    },
    {
        .name = "--keepspeed",
        .value = &is_keep_speed_on,
    },
    {
        .name = "--parkahead",
        .value = &is_park_ahead_on,
    },
    {
        .name = "--crossfollow",
        .value = &is_cross_follow_on,
    },
    {
        .name = "--oldroute",
        .value = &is_old_route_on,
    },
    {
        .name = "--slotpoint",
        .value = &is_slot_point_on,
    },
};

/* ------------------------------------------------------------------------------------------------
 * `is_any_squad_busy` returns true while at least one squad moves or still searches its route.
 * --------------------------------------------------------------------------------------------- */
static bool is_any_squad_busy(void) {
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (is_squad_busy(&squads[squad_index]))
            return true;
    }
    return false;
}

/* ------------------------------------------------------------------------------------------------
 * `is_any_squad_searching` returns true while at least one squad searches its route.
 * --------------------------------------------------------------------------------------------- */
static bool is_any_squad_searching(void) {
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (squads[squad_index].state == SQUAD_SEARCHING)
            return true;
    }
    return false;
}

/* A `TickTimes` holds the wall-clock times of the ticks of one run, without the metrics and the
 * trace, which the game does not have. Order ticks are the first tick, which gives the orders, and
 * every tick that starts with a squad still searching its route. They hold the route search of
 * the game itself, which the game spreads over several frames, and the one-time set-up of
 * formation following, so `TickTimes` keeps them apart from the moving ticks, which are all
 * other ticks. */
typedef struct {
    double move_sum_seconds;  /* The moving ticks took this long together. */
    double move_max_seconds;  /* The longest moving tick took this long. */
    int move_tick_count;      /* `move_sum_seconds` adds up this many ticks. */
    double order_max_seconds; /* The longest order tick took this long. */
} TickTimes;

/* ------------------------------------------------------------------------------------------------
 * `add_tick_time` adds the time of one tick to `times`.
 * --------------------------------------------------------------------------------------------- */
static void add_tick_time(TickTimes *times, double tick_seconds, bool is_order_tick) {
    if (is_order_tick) {
        times->order_max_seconds = fmax(times->order_max_seconds, tick_seconds);
        return;
    }
    times->move_sum_seconds += tick_seconds;
    times->move_max_seconds = fmax(times->move_max_seconds, tick_seconds);
    times->move_tick_count++;
}

/* ------------------------------------------------------------------------------------------------
 * `print_tick_times` prints one line on stderr with the mean and the longest time of the moving
 * ticks of the run and the longest time of its order ticks, in microseconds. A run without moving
 * ticks gets a mean of 0, because the mean divides by at least 1.
 * --------------------------------------------------------------------------------------------- */
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

/* ------------------------------------------------------------------------------------------------
 * `give_orders` gives every squad with an order its move at the first tick. It marks all of them
 * as searching before any of them starts its search, so that no squad plans its route around
 * another one as a parked squad. An order with a facing in `order_facings` replaces the facing
 * that `order_move` takes from the way to the destination, as a player sets it in the game by
 * dragging the order. The search of the squad does not use the facing, but its formation layout
 * at the end of the route and its final facing do.
 * --------------------------------------------------------------------------------------------- */
static void give_orders(void) {
    /* Mark the squads with an order as searching. */
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (has_order[squad_index])
            squads[squad_index].state = SQUAD_SEARCHING;
    }

    /* Give each order, and the facing of the order when it has one. */
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        if (!has_order[squad_index])
            continue;
        Squad *squad = &squads[squad_index];
        order_move(squad, order_destinations[squad_index]);
        V2 order_facing = order_facings[squad_index];
        if (length_squared(order_facing) > 0)
            squad->facing = order_facing;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `run_scenario` runs one scenario in the current `avoid_mode` and reports each squad. The run
 * ends `SETTLE_SECONDS` after no squad is busy any more, or after `MAX_RUN_SECONDS`.
 *
 * Time step: The loop steps `now_seconds` by `TICK_SECONDS`. The game moves units once per
 * rendered frame, and Timing::CalculateTime (0x428550 in gc.exe 1.0.1.1) measures the frame time.
 * Its fixed-step mode, which the flag at 0x69b6ac turns on, uses exactly 0.04 s, and the simulator
 * uses that fixed step. re/PATHFINDING.md describes the timing of the game.
 * --------------------------------------------------------------------------------------------- */
static void run_scenario(int scenario_index) {
    /* Set up the scenario. */
    set_up_scenario(scenario_index, seed);
    if (trace_file)
        begin_trace_run(scenario_index);

    /* Simulate tick by tick. `is_settling` becomes true at the first tick at which no squad is
     * busy, and `idle_since_seconds` keeps the time of that tick. */
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
        /* Run one tick of the squads and units, and time it. The first tick gives the orders. */
        double tick_start_seconds = read_clock_seconds();
        bool is_order_tick = tick_index == 0 || is_any_squad_searching();
        if (tick_index == 0)
            give_orders();
        if (is_formation_mode())
            coordinate_squads();
        reset_set_up_work();
        for (int squad_index = 0; squad_index < squad_count; squad_index++)
            update_squad(&squads[squad_index]);
        finish_threaded_set_ups();
        for (int unit_index = 0; unit_index < unit_count; unit_index++)
            tick_mover(unit_index);
        double tick_seconds = read_clock_seconds() - tick_start_seconds;
        add_tick_time(&times, tick_seconds, is_order_tick);

        /* Measure the tick, and write every `TRACE_EVERY_TICKS`-th tick into the trace. */
        measure();
        if (trace_file && tick_index % TRACE_EVERY_TICKS == 0)
            write_trace_frame(tick_index == 0);

        /* Stop `SETTLE_SECONDS` after the run starts to settle. */
        if (!is_any_squad_busy() && !is_settling) {
            is_settling = true;
            idle_since_seconds = now_seconds;
        }
        if (is_settling && now_seconds > idle_since_seconds + SETTLE_SECONDS)
            break;
        tick_index++;
    }

    /* Report each squad and the tick times, and close the part of the run in the trace with the
     * map excerpt. */
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

/* ------------------------------------------------------------------------------------------------
 * `is_switch_on` returns false when `value` is "off", and true for every other value.
 * --------------------------------------------------------------------------------------------- */
static bool is_switch_on(const char *value) {
    return strcmp(value, "off") != 0;
}

/* ------------------------------------------------------------------------------------------------
 * `find_on_off_switch` returns the global that the on/off switch `name` sets, or NULL when `name`
 * is no such switch.
 * --------------------------------------------------------------------------------------------- */
static bool *find_on_off_switch(const char *name) {
    int switch_count = sizeof ON_OFF_SWITCHES / sizeof ON_OFF_SWITCHES[0];
    for (int switch_index = 0; switch_index < switch_count; switch_index++) {
        const OnOffSwitch *on_off_switch = &ON_OFF_SWITCHES[switch_index];
        if (!strcmp(name, on_off_switch->name))
            return on_off_switch->value;
    }
    return NULL;
}

/* ------------------------------------------------------------------------------------------------
 * `parse_command_line` reads the arguments into `command_line` and into the globals that they
 * set. An argument that is not a switch chooses the scenario of that name. A switch without its
 * value, and an unknown word, have no effect.
 * Result: It returns false when the `--trace` file cannot be opened, and true otherwise.
 * --------------------------------------------------------------------------------------------- */
static bool parse_command_line(int argc, char **argv, CommandLine *command_line) {
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        const char *argument = argv[argument_index];
        bool has_value = argument_index + 1 < argc;
        bool has_two_values = argument_index + 2 < argc;
        bool *on_off_value = find_on_off_switch(argument);
        if (on_off_value && has_value) {
            argument_index++;
            *on_off_value = is_switch_on(argv[argument_index]);
        } else if (!strcmp(argument, "--exe") && has_value) {
            argument_index++;
            command_line->exe_path = argv[argument_index];
        } else if (!strcmp(argument, "--maps") && has_value) {
            argument_index++;
            maps_dir = argv[argument_index];
        } else if (!strcmp(argument, "--find") && has_two_values) {
            command_line->find_map_name = argv[argument_index + 1];
            command_line->find_sample_count = atoi(argv[argument_index + 2]);
            argument_index += 2;
        } else if (!strcmp(argument, "--avoid") && has_value) {
            argument_index++;
            for (int mode = AVOID_GAME; mode <= AVOID_COORD; mode++) {
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

/* ------------------------------------------------------------------------------------------------
 * `run_chosen_scenarios` runs every chosen scenario in every mode from `first_mode` to
 * `last_mode`, and writes the outer part of the trace around the runs.
 * --------------------------------------------------------------------------------------------- */
static void run_chosen_scenarios(int only_scenario_index, int first_mode, int last_mode) {
    /* Start the trace with the tick times, and print the header of the result rows. */
    if (trace_file) {
        float frame_seconds = TRACE_EVERY_TICKS * TICK_SECONDS;
        fprintf(trace_file, "{\"dt\":%g,\"frame_dt\":%g,\"runs\":[", TICK_SECONDS, frame_seconds);
    }
    print_result_header();

    /* Run the scenarios in `SCENARIO_RUN_ORDER`, and separate their runs in the trace with
     * commas. */
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

    /* Close the trace. */
    if (trace_file) {
        fputs("]}\n", trace_file);
        fclose(trace_file);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `main` reads the command line, maps gc.exe, and then runs `--find` or the chosen scenarios.
 * --------------------------------------------------------------------------------------------- */
int main(int argc, char **argv) {
    /* Read the command line. Without `--exe`, the simulator reads the gc.exe of a Steam install
     * in its default place. */
    CommandLine command_line = {
        .exe_path = ("C:\\Program Files (x86)\\Steam\\steamapps\\common\\"
                     "Ground Control Anthology\\gc.exe"),
        .only_scenario_index = ALL_SCENARIOS,
        .first_mode = AVOID_GAME,
        .last_mode = AVOID_COORD,
        .find_map_name = NULL,
        .find_sample_count = 0,
    };
    if (!parse_command_line(argc, argv, &command_line))
        return EXIT_FAILURE;

    /* Map gc.exe. `--find` needs nothing else. */
    if (!gc_load(command_line.exe_path))
        return EXIT_FAILURE;
    allocate_lanes();
    if (command_line.find_map_name) {
        find_spots(command_line.find_map_name, command_line.find_sample_count);
        return EXIT_SUCCESS;
    }

    /* Run every chosen scenario in every chosen mode. */
    int only_scenario_index = command_line.only_scenario_index;
    run_chosen_scenarios(only_scenario_index, command_line.first_mode, command_line.last_mode);
    return EXIT_SUCCESS;
}
