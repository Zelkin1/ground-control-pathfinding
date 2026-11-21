// Unit types of the three factions of the game, and the speed modifiers of the armor profiles. The
// values come from the unit files `ENTFILES/*.ENT` of the game archives, where DATA2.sdf overrides
// Data0.sdf, and a unit file without `TurnOffset` has the close range 5 of the game.
//
// The scenarios name the Crayven unit type of each role. With `--faction`, `get_faction_unit_type`
// replaces it by the unit type of the same role in another faction. The Order units and the
// Phoenix command APC are hover units and the others are vehicles or foot units, but the
// simulator moves all of them alike.
// TODO: confirm that the game moves hover units as vehicles.
#include "sim.h"

const char *FACTION_NAME[FACTION_COUNT] = {
    [FACTION_CRAYVEN] = "crayven",
    [FACTION_ORDER] = "order",
    [FACTION_PHOENIX] = "phoenix",
};

// In MANAGEMENTDATA/ARMOR.TXT, the four armor profiles of every ground unit type have these
// `SPEEDMODIFIER` values. The game multiplies the top speed of a unit by it (0x4f9fd0).
const char *PROFILE_NAME[PROFILE_COUNT] = {
    [PROFILE_BALANCED] = "balanced",
    [PROFILE_DEFENSIVE] = "defensive",
    [PROFILE_OFFENSIVE] = "offensive",
    [PROFILE_SPEED] = "speed",
};
const float PROFILE_SPEED_MODIFIER[PROFILE_COUNT] = {
    [PROFILE_BALANCED] = 1.0f,
    [PROFILE_DEFENSIVE] = 0.75f,
    [PROFILE_OFFENSIVE] = 0.75f,
    [PROFILE_SPEED] = 1.2f,
};

// Crayven Corporation (CC).
const UnitType MEDIUM_TANK = {
    .name = "cc_mediumtank",
    .radius = 3.5f,
    .top_speed = 20,
    .acceleration = 70,
    .friction = 65,
    .turn_rate = 1.5f,
    .close_range = 5,
    .terrain_class = 1,
    .units_per_squad = 4,
};
const UnitType LIGHT_TANK = {
    .name = "cc_lighttank",
    .radius = 3.5f,
    .top_speed = 28,
    .acceleration = 87,
    .friction = 80,
    .turn_rate = 1.8f,
    .close_range = 10,
    .terrain_class = 1,
    .units_per_squad = 4,
};
const UnitType HEAVY_TANK = {
    .name = "cc_heavytank",
    .radius = 6.0f,
    .top_speed = 15,
    .acceleration = 87,
    .friction = 80,
    .turn_rate = 0.9f,
    .close_range = 12,
    .terrain_class = 1,
    .units_per_squad = 2,
};
const UnitType MARINE = {
    .name = "cc_marine",
    .radius = 1.0f,
    .top_speed = 8,
    .acceleration = 110,
    .friction = 100,
    .turn_rate = 3.2f,
    .close_range = 5,
    .terrain_class = 2,
    .units_per_squad = 8,
};
const UnitType COMMAND_APC = {
    .name = "cc_commandapc",
    .radius = 4.1f,
    .top_speed = 20,
    .acceleration = 70,
    .friction = 65,
    .turn_rate = 1.7f,
    .close_range = 5,
    .terrain_class = 1,
    .units_per_squad = 1,
};

// Order of the New Dawn (WC).
static const UnitType ORDER_MEDIUM_TANK = {
    .name = "wc_mediumtank",
    .faction = FACTION_ORDER,
    .radius = 5.0f,
    .top_speed = 28,
    .acceleration = 80,
    .friction = 70,
    .turn_rate = 2.2f,
    .close_range = 11,
    .terrain_class = 1,
    .units_per_squad = 4,
};
static const UnitType ORDER_LIGHT_TANK = {
    .name = "wc_lighttank",
    .faction = FACTION_ORDER,
    .radius = 6.5f,
    .top_speed = 30,
    .acceleration = 45,
    .friction = 30,
    .turn_rate = 2.2f,
    .close_range = 10,
    .terrain_class = 1,
    .units_per_squad = 4,
};
static const UnitType ORDER_HEAVY_TANK = {
    .name = "wc_heavytank",
    .faction = FACTION_ORDER,
    .radius = 5.8f,
    .top_speed = 25,
    .acceleration = 90,
    .friction = 75,
    .turn_rate = 2.2f,
    .close_range = 10,
    .terrain_class = 1,
    .units_per_squad = 2,
};
static const UnitType ORDER_CRUSADER = {
    .name = "wc_crusader",
    .faction = FACTION_ORDER,
    .radius = 1.3f,
    .top_speed = 8,
    .acceleration = 110,
    .friction = 100,
    .turn_rate = 2.5f,
    .close_range = 5,
    .terrain_class = 2,
    .units_per_squad = 8,
};
static const UnitType ORDER_COMMAND_APC = {
    .name = "wc_commandapc",
    .faction = FACTION_ORDER,
    .radius = 4.5f,
    .top_speed = 28,
    .acceleration = 90,
    .friction = 75,
    .turn_rate = 2.2f,
    .close_range = 10,
    .terrain_class = 1,
    .units_per_squad = 1,
};

// Phoenix (PX).
static const UnitType PHOENIX_MEDIUM_TANK = {
    .name = "px_mediumtank",
    .faction = FACTION_PHOENIX,
    .radius = 5.6f,
    .top_speed = 23,
    .acceleration = 70,
    .friction = 68.5f,
    .turn_rate = 1.1f,
    .close_range = 5,
    .terrain_class = 1,
    .units_per_squad = 4,
};
static const UnitType PHOENIX_LIGHT_TANK = {
    .name = "px_lighttank",
    .faction = FACTION_PHOENIX,
    .radius = 5.1f,
    .top_speed = 29,
    .acceleration = 87,
    .friction = 84,
    .turn_rate = 1.1f,
    .close_range = 10,
    .terrain_class = 1,
    .units_per_squad = 4,
};
static const UnitType PHOENIX_HEAVY_TANK = {
    .name = "px_heavytank",
    .faction = FACTION_PHOENIX,
    .radius = 6.1f,
    .top_speed = 19,
    .acceleration = 87,
    .friction = 86,
    .turn_rate = 1.1f,
    .close_range = 12,
    .terrain_class = 1,
    .units_per_squad = 2,
};
static const UnitType PHOENIX_BASIC = {
    .name = "px_basic",
    .faction = FACTION_PHOENIX,
    .radius = 1.0f,
    .top_speed = 8,
    .acceleration = 110,
    .friction = 100,
    .turn_rate = 3.2f,
    .close_range = 5,
    .terrain_class = 2,
    .units_per_squad = 8,
};
static const UnitType PHOENIX_COMMAND_APC = {
    .name = "px_commandapc",
    .faction = FACTION_PHOENIX,
    .radius = 4.1f,
    .top_speed = 23,
    .acceleration = 70,
    .friction = 67,
    .turn_rate = 1.7f,
    .close_range = 5,
    .terrain_class = 1,
    .units_per_squad = 1,
};

// Unit types by role and then by faction.
static const UnitType *const ROLE_UNIT_TYPES[][FACTION_COUNT] = {
    {
        [FACTION_CRAYVEN] = &MEDIUM_TANK,
        [FACTION_ORDER] = &ORDER_MEDIUM_TANK,
        [FACTION_PHOENIX] = &PHOENIX_MEDIUM_TANK,
    },
    {
        [FACTION_CRAYVEN] = &LIGHT_TANK,
        [FACTION_ORDER] = &ORDER_LIGHT_TANK,
        [FACTION_PHOENIX] = &PHOENIX_LIGHT_TANK,
    },
    {
        [FACTION_CRAYVEN] = &HEAVY_TANK,
        [FACTION_ORDER] = &ORDER_HEAVY_TANK,
        [FACTION_PHOENIX] = &PHOENIX_HEAVY_TANK,
    },
    {
        [FACTION_CRAYVEN] = &MARINE,
        [FACTION_ORDER] = &ORDER_CRUSADER,
        [FACTION_PHOENIX] = &PHOENIX_BASIC,
    },
    {
        [FACTION_CRAYVEN] = &COMMAND_APC,
        [FACTION_ORDER] = &ORDER_COMMAND_APC,
        [FACTION_PHOENIX] = &PHOENIX_COMMAND_APC,
    },
};


// `get_faction_unit_type` returns the unit type of `faction` with the role of the Crayven unit
// type `unit_type`, or `unit_type` itself when no role has it.
const UnitType *get_faction_unit_type(const UnitType *unit_type, int faction) {
    int role_count = sizeof ROLE_UNIT_TYPES / sizeof ROLE_UNIT_TYPES[0];
    for (int role_index = 0; role_index < role_count; role_index++) {
        if (ROLE_UNIT_TYPES[role_index][FACTION_CRAYVEN] == unit_type)
            return ROLE_UNIT_TYPES[role_index][faction];
    }
    return unit_type;
}
