// Switches of the simulator and of the game mod. Each switch turns one feature of formation
// following or coordination on or off (docs/SWITCHES.md). The simulator sets them from its command
// line, and the mod from its configuration file, both through `find_on_off_switch`.
#include <string.h>

#include "sim.h"

bool are_lanes_on = true;
bool is_formation_keeping_on = true;
bool is_smooth_on = true;
bool is_wide_bend_on = true;
bool is_driving_on = true;
bool is_crossing_give_way_on = true;
bool is_keep_right_on = true;
bool is_turn_taking_on = true;
bool is_speed_priority_on = true;
bool is_detour_on = true;
bool is_pass_on = true;
bool is_form_first_on = true;
bool is_keep_route_on = true;
bool is_park_reroute_on = true;
bool is_park_robust_on = true;
bool is_partial_shift_on = true;
bool is_set_up_accumulate_on = false;
bool is_full_search_on = true;
bool is_set_up_threads_on = true;
bool is_speed_replan_on = true;
bool is_timed_shift_on = true;
bool is_narrow_pass_on = true;
bool is_smoothing_stop_on = true;
bool is_tight_bend_on = true;
bool is_faction_tuning_on = true;
bool is_smooth_reorder_on = true;
bool is_friend_coordination_on = true;
bool is_least_delay_on = true;
bool are_boarders_ignored = true;
bool is_infantry_shape_kept = true;
bool is_parked_push_on = true;
bool is_squad_friend_on = true;
bool is_game_layout_on = true;
bool is_free_parking_on = true;
bool is_whole_parked_on = false;
bool is_parked_box_on = false;


// A switch name and the flag that it sets.
typedef struct {
    const char *name;
    bool *value;
} OnOffSwitch;


static const OnOffSwitch ON_OFF_SWITCHES[] = {
    {
        .name = "--driving",
        .value = &is_driving_on,
    },
    {
        .name = "--smooth",
        .value = &is_smooth_on,
    },
    {
        .name = "--widebends",
        .value = &is_wide_bend_on,
    },
    {
        .name = "--formation",
        .value = &is_formation_keeping_on,
    },
    {
        .name = "--lanes",
        .value = &are_lanes_on,
    },
    {
        .name = "--crossing",
        .value = &is_crossing_give_way_on,
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
        .name = "--detour",
        .value = &is_detour_on,
    },
    {
        .name = "--pass",
        .value = &is_pass_on,
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
        .name = "--parkreroute",
        .value = &is_park_reroute_on,
    },
    {
        .name = "--parkrobust",
        .value = &is_park_robust_on,
    },
    {
        .name = "--partialshift",
        .value = &is_partial_shift_on,
    },
    {
        .name = "--setupaccumulate",
        .value = &is_set_up_accumulate_on,
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
        .name = "--speedreplan",
        .value = &is_speed_replan_on,
    },
    {
        .name = "--timedshift",
        .value = &is_timed_shift_on,
    },
    {
        .name = "--narrowpass",
        .value = &is_narrow_pass_on,
    },
    {
        .name = "--smoothstop",
        .value = &is_smoothing_stop_on,
    },
    {
        .name = "--tightbends",
        .value = &is_tight_bend_on,
    },
    {
        .name = "--factiontuning",
        .value = &is_faction_tuning_on,
    },
    {
        .name = "--smoothreorder",
        .value = &is_smooth_reorder_on,
    },
    {
        .name = "--friendcoord",
        .value = &is_friend_coordination_on,
    },
    {
        .name = "--leastdelay",
        .value = &is_least_delay_on,
    },
    {
        .name = "--ignoreboarders",
        .value = &are_boarders_ignored,
    },
    {
        .name = "--infantryshape",
        .value = &is_infantry_shape_kept,
    },
    {
        .name = "--parkedpush",
        .value = &is_parked_push_on,
    },
    {
        .name = "--squadfriends",
        .value = &is_squad_friend_on,
    },
    {
        .name = "--gamelayout",
        .value = &is_game_layout_on,
    },
    {
        .name = "--freeparking",
        .value = &is_free_parking_on,
    },
    {
        .name = "--wholeparked",
        .value = &is_whole_parked_on,
    },
    {
        .name = "--parkedbox",
        .value = &is_parked_box_on,
    },
};


// `find_on_off_switch` returns the flag of the switch `name`, or NULL for any other name.
bool *find_on_off_switch(const char *name) {
    int switch_count = sizeof ON_OFF_SWITCHES / sizeof ON_OFF_SWITCHES[0];
    for (int switch_index = 0; switch_index < switch_count; switch_index++) {
        const OnOffSwitch *on_off_switch = &ON_OFF_SWITCHES[switch_index];
        if (!strcmp(name, on_off_switch->name))
            return on_off_switch->value;
    }
    return NULL;
}
