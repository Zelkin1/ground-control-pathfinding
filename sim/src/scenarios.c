/* This file holds the unit types and the test scenarios of the simulator. Each scenario uses one
 * of the maps of the game itself, and its squads stand where the formation layout of the game
 * puts them. With `--seed`, the starts move a little, the same way in every mode.
 *
 * Coordinates: Every point is a world position in meters, with x to the east and y to the north.
 * A grid cell is `GC_CELL_METERS` (6.25 m) wide. Directions, such as v2(1, 0) for east, have
 * length 1 or close to it. `--find` helped to pick the points by the passage widths, turns and
 * detours of real routes.
 *
 * Terms: The formation layout of the game lays out the slots (sim.h). The anchor of a layout is
 * the center of its first row of slots. The span of a squad is the largest distance
 * between two of its slots, or the width of the unit when that is larger. Jitter is the small
 * random move of the starts that `--seed` makes. */
#include <stdlib.h>
#include <string.h>

#include "sim.h"

#define PLAYER_TEAM 1                    /* All squads of the scenarios are on this team. */
#define SQUAD_FORMATION_TYPE 1           /* Game squads use this formation by default. */
#define START_CLEARANCE_METERS 0.5f      /* Moved units keep at least this gap to other units. */
#define SQUAD_JITTER_ATTEMPTS 50         /* `--seed` tries this many moves of each squad. */
#define SQUAD_JITTER_SPANS 1.5f          /* `--seed` moves a squad by up to this many spans. */
#define UNIT_JITTER_METERS 1.5f          /* `--seed` moves each unit by up to this on each axis. */
#define UNIT_JITTER_RADIANS 0.05f        /* `--seed` turns each unit by up to this. */
#define SEED_HASH_MULTIPLIER 2654435761u /* This is close to 2^32 divided by the golden ratio. */
#define SCENARIO_HASH_MULTIPLIER 40503u  /* This is close to 2^16 divided by the golden ratio. */

/* These constants set the layouts of the scenarios, in world units. */
#define SIDE_BY_SIDE_METERS 50.0f       /* In `ridge`, the two squads drive this far apart. */
#define FOLLOW_GAP_METERS 45.0f         /* In `corridor`, the squads start this far apart. */
#define SPREAD_DESTINATION_METERS 40.0f /* In `corridor`, the rear squads end this far aside. */
#define WINDING_TANK_SQUAD_COUNT 3      /* `winding` has a light, a medium and a heavy squad. */
#define WINDING_GAP_METERS 60.0f        /* In `winding`, squads start and end this far apart. */
#define LINE_SQUAD_COUNT 6              /* The line scenario, `lineplain`, has six squads. */
#define LINE_GAP_METERS 40.0f           /* Its squads stand this far apart in each line. */
#define LINE_ADVANCE_METERS 250.0f      /* The middle of the new line lies this far ahead. */

/* The unit types have the values of the game from the ENTFILES .ENT files in Data0.sdf, which
 * tools/sdf.py extracts. `Kind` in sim.h names the .ENT field of each value. The loader of the
 * game (EntityData::LoadMe, 0x4212c0) and the Mover set-up (0x50e450) use these values as they
 * are.
 *
 * `close_range` is the field `TurnOffset`. The game keeps it at byte offset 0xc0 of its entity
 * type object, and the EntityData constructor (0x420da0) sets it to 5 when the .ENT file has
 * none. */
const Kind MEDIUM_TANK = {
    .name = "cc_mediumtank",
    .radius = 3.5f,
    .max_speed = 20,
    .acceleration = 70,
    .friction = 65,
    .turn_rate = 1.5f,
    .close_range = 5,
    .mover_class = 1,
    .units_per_squad = 4,
};
const Kind LIGHT_TANK = {
    .name = "cc_lighttank",
    .radius = 3.5f,
    .max_speed = 28,
    .acceleration = 87,
    .friction = 80,
    .turn_rate = 1.8f,
    .close_range = 10,
    .mover_class = 1,
    .units_per_squad = 4,
};
const Kind HEAVY_TANK = {
    .name = "cc_heavytank",
    .radius = 6.0f,
    .max_speed = 15,
    .acceleration = 87,
    .friction = 80,
    .turn_rate = 0.9f,
    .close_range = 12,
    .mover_class = 1,
    .units_per_squad = 2,
};
/* Marines are infantry. Their mover class 2 lets them cross hills (cell value 2), which block the
 * tanks. Their .ENT file has no `TurnOffset`, so `close_range` is the default of the game, 5. */
const Kind MARINE = {
    .name = "cc_marine",
    .radius = 1.0f,
    .max_speed = 8,
    .acceleration = 110,
    .friction = 100,
    .turn_rate = 3.2f,
    .close_range = 5,
    .mover_class = 2,
    .units_per_squad = 8,
};

/* `jitter_state` is the random state of `--seed`. The starts move only while it is not 0, and it
 * is 0 without a seed. */
static unsigned jitter_state;

/* ------------------------------------------------------------------------------------------------
 * `draw_jitter` returns the next random number of `--seed`, from -1 up to but not including 1.
 * --------------------------------------------------------------------------------------------- */
static float draw_jitter(void) {
    return next_random(&jitter_state) / (RANDOM_RANGE / 2) - 1;
}

/* ------------------------------------------------------------------------------------------------
 * `is_free_spot` returns true when the cell at `point` does not block `mover_class`, and a unit of
 * `radius` at `point` keeps a gap of at least `START_CLEARANCE_METERS` to each unit placed so
 * far. It returns false otherwise.
 * --------------------------------------------------------------------------------------------- */
static bool is_free_spot(V2 point, float radius, int mover_class) {
    if (gc_get_cell_at(point.x, point.y) > mover_class)
        return false;
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        float min_distance = radius + units[unit_index].kind->radius + START_CLEARANCE_METERS;
        float distance_squared = length_squared(sub(units[unit_index].position, point));
        if (distance_squared < min_distance * min_distance)
            return false;
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_squad_span` returns the span of a squad of `kind` whose `slot_count` slots `slots`
 * holds as x, y pairs.
 * --------------------------------------------------------------------------------------------- */
static float measure_squad_span(const Kind *kind, const float *slots, int slot_count) {
    float span_meters = 2 * kind->radius;
    for (int first_index = 0; first_index < slot_count; first_index++) {
        for (int second_index = first_index + 1; second_index < slot_count; second_index++) {
            V2 first = get_pair_point(slots, first_index);
            V2 second = get_pair_point(slots, second_index);
            float slot_distance = len(sub(first, second));
            span_meters = fmaxf(span_meters, slot_distance);
        }
    }
    return span_meters;
}

/* ------------------------------------------------------------------------------------------------
 * `jitter_squad` moves the slots of a squad for `--seed`, by up to `SQUAD_JITTER_SPANS` times the
 * span of the squad (`max_offset_meters`). It draws an offset of up to that length along each
 * axis, and uses it when all of these hold: the offset is no longer than `max_offset_meters`, the
 * straight line from `anchor` to the moved anchor is clear for the mover class of the squad, the
 * formation layout of the game works at the moved anchor, and every moved slot passes
 * `is_free_spot`. The moved slots then replace `slots`.
 * Edge case: When none of `SQUAD_JITTER_ATTEMPTS` offsets can be used, `slots` stays as it was.
 * --------------------------------------------------------------------------------------------- */
static void jitter_squad(const Kind *kind, V2 anchor, V2 direction, int formation_type,
                         float *slots) {
    int slot_count = kind->units_per_squad;
    int mover_class = kind->mover_class;
    float span_meters = measure_squad_span(kind, slots, slot_count);
    float max_offset_meters = SQUAD_JITTER_SPANS * span_meters;
    for (int attempt_index = 0; attempt_index < SQUAD_JITTER_ATTEMPTS; attempt_index++) {
        /* Draw the offset, and test its length and the line to the moved anchor. */
        float offset_x = draw_jitter();
        float offset_y = draw_jitter();
        V2 offset = mul(v2(offset_x, offset_y), max_offset_meters);
        V2 moved_anchor = add(anchor, offset);
        bool is_short_enough = len(offset) <= max_offset_meters;
        if (!is_short_enough)
            continue;
        int ray_result = gc_cast_ray(anchor.x, anchor.y, moved_anchor.x, moved_anchor.y,
                                     mover_class);
        if (ray_result != GC_RAY_CLEAR)
            continue;

        /* Lay out the formation at the moved anchor, and use it when every slot is free.
         * `moved_slots` holds the slots as x, y pairs. */
        float moved_slots[2 * MAX_UNITS];
        bool is_placed = gc_place_formation(moved_anchor.x, moved_anchor.y, direction.x,
                                            direction.y, slot_count, kind->radius, mover_class,
                                            formation_type, moved_slots);
        if (!is_placed)
            continue;
        bool is_free = true;
        for (int slot_index = 0; slot_index < slot_count && is_free; slot_index++) {
            V2 slot = get_pair_point(moved_slots, slot_index);
            is_free = is_free_spot(slot, kind->radius, mover_class);
        }
        if (is_free) {
            memcpy(slots, moved_slots, sizeof(float) * 2 * slot_count);
            return;
        }
    }
}

/* ------------------------------------------------------------------------------------------------
 * `add_unit` adds a parked unit of `kind` at `position` to the squad that is being built,
 * `squads[squad_count]`. Its slot is `position`, and its `facing` is `direction`. A negative
 * `angle`, or one of 2 * PI or more, is moved by one full turn toward 0..2 * PI. The Mover values
 * are the start values that sim.h lists next to `STOP_REACH_METERS`.
 * --------------------------------------------------------------------------------------------- */
static void add_unit(const Kind *kind, int team, V2 position, float angle, V2 direction) {
    /* Take the next free unit of `units`, and make it a member of the squad. */
    Unit *unit = &units[unit_count];
    unit_count++;
    memset(unit, 0, sizeof *unit);
    unit->kind = kind;
    unit->squad_index = squad_count;
    unit->team = team;

    /* Place the unit, and turn it to `angle`. */
    unit->position = position;
    unit->start_position = position;
    unit->slot = position;
    unit->angle = angle;
    if (unit->angle < 0)
        unit->angle += 2 * PI;
    if (unit->angle >= 2 * PI)
        unit->angle -= 2 * PI;
    unit->heading = v2(sinf(unit->angle), cosf(unit->angle));
    unit->previous_angle = unit->angle;
    unit->facing = direction;

    /* Give it the Mover start values of a parked unit. */
    unit->waypoint = unit->position;
    unit->reach = STOP_REACH_METERS;
    unit->reach_last = STOP_REACH_METERS;
    unit->reach_mid = MID_REACH_METERS;
    unit->is_stopped = true;
    unit->blocker_index = NO_BLOCKER;
    unit->wait_seconds = WAIT_TIMER_AT_ORDER_SECONDS;
}

/* ------------------------------------------------------------------------------------------------
 * `add_squad` adds a squad of `kind` whose formation stands at `anchor`, facing `direction`, with
 * a parked unit on each slot of the formation layout of the game.
 * Edge case: When the layout fails, the program prints an error and exits.
 * --------------------------------------------------------------------------------------------- */
static void add_squad(const Kind *kind, int team, V2 anchor, V2 direction, int formation_type) {
    /* Set up the squad. */
    int member_count = kind->units_per_squad;
    Squad *squad = &squads[squad_count];
    memset(squad, 0, sizeof *squad);
    squad->kind = kind;
    squad->squad_index = squad_count;
    squad->first_unit_index = unit_count;
    squad->unit_count = member_count;
    squad->formation_type = formation_type;
    squad->last_facing = direction;
    squad->yielding_to_index = NO_SQUAD;
    squad->passing_index = NO_SQUAD;
    gc_init_request(&squad->search);

    /* Lay out its slots with the formation layout of the game, and move them with `--seed`.
     * `slots` holds the slots as x, y pairs. */
    float slots[2 * MAX_UNITS];
    bool is_placed = gc_place_formation(anchor.x, anchor.y, direction.x, direction.y, member_count,
                                        kind->radius, kind->mover_class, formation_type, slots);
    if (!is_placed) {
        fprintf(stderr, "cannot place squad %d\n", squad_count);
        exit(EXIT_FAILURE);
    }
    if (jitter_state)
        jitter_squad(kind, anchor, direction, formation_type, slots);

    /* Put a unit on each slot, facing `direction`. With `--seed`, move each unit a little when the
     * moved spot passes `is_free_spot`, and turn it a little. */
    float direction_angle = atan2f(direction.x, direction.y);
    for (int slot_index = 0; slot_index < member_count; slot_index++) {
        V2 position = get_pair_point(slots, slot_index);
        float angle = direction_angle;
        if (jitter_state) {
            float offset_x = draw_jitter() * UNIT_JITTER_METERS;
            float offset_y = draw_jitter() * UNIT_JITTER_METERS;
            V2 moved = add(position, v2(offset_x, offset_y));
            if (is_free_spot(moved, kind->radius, kind->mover_class))
                position = moved;
            angle += draw_jitter() * UNIT_JITTER_RADIANS;
        }
        add_unit(kind, team, position, angle, direction);
    }
    squad_count++;
}

/* These are the scenarios, as indexes into `SCENARIO_NAME`. */
enum {
    SCENARIO_CROSS,
    SCENARIO_HEADON,
    SCENARIO_PARKED,
    SCENARIO_PASS,
    SCENARIO_RIDGE,
    SCENARIO_CORRIDOR,
    SCENARIO_INFANTRY,
    SCENARIO_HEAVIES,
    SCENARIO_WINDING,
    SCENARIO_LINE_PLAIN,
    SCENARIO_HEADON_LIGHT,
    SCENARIO_HEADON_HEAVY,
};

/* The scenarios run, and the replay lists them, in this order, with the head-on cases together.
 * This separate order keeps the indexes fixed, because `--seed` mixes the index of each scenario
 * into its starts. */
const int SCENARIO_RUN_ORDER[SCENARIO_COUNT] = {
    SCENARIO_CROSS,
    SCENARIO_HEADON,
    SCENARIO_HEADON_LIGHT,
    SCENARIO_HEADON_HEAVY,
    SCENARIO_HEAVIES,
    SCENARIO_PARKED,
    SCENARIO_PASS,
    SCENARIO_RIDGE,
    SCENARIO_CORRIDOR,
    SCENARIO_INFANTRY,
    SCENARIO_WINDING,
    SCENARIO_LINE_PLAIN,
};

const char *SCENARIO_NAME[SCENARIO_COUNT] = {
    [SCENARIO_CROSS] = "cross",
    [SCENARIO_HEADON] = "headon",
    [SCENARIO_PARKED] = "parked",
    [SCENARIO_PASS] = "pass",
    [SCENARIO_RIDGE] = "ridge",
    [SCENARIO_CORRIDOR] = "corridor",
    [SCENARIO_INFANTRY] = "infantry",
    [SCENARIO_HEAVIES] = "heavies",
    [SCENARIO_WINDING] = "winding",
    [SCENARIO_LINE_PLAIN] = "lineplain",
    [SCENARIO_HEADON_LIGHT] = "headonlight",
    [SCENARIO_HEADON_HEAVY] = "headonheavy",
};

/* `SCENARIO_MAP` holds the name of the map of each scenario, as `load_map` takes it. */
static const char *SCENARIO_MAP[SCENARIO_COUNT] = {
    [SCENARIO_CROSS] = "FZ1",
    [SCENARIO_HEADON] = "FZ7",
    [SCENARIO_PARKED] = "FZ1",
    [SCENARIO_PASS] = "MISSION10",
    [SCENARIO_RIDGE] = "W_MISSION4",
    [SCENARIO_CORRIDOR] = "MISSION10",
    [SCENARIO_INFANTRY] = "MISSION10",
    [SCENARIO_HEAVIES] = "FZ7",
    [SCENARIO_WINDING] = "MISSION6",
    [SCENARIO_LINE_PLAIN] = "FZ1",
    [SCENARIO_HEADON_LIGHT] = "FZ7",
    [SCENARIO_HEADON_HEAVY] = "FZ7",
};

const char *SCENARIO_DESCRIPTION[SCENARIO_COUNT] = {
    [SCENARIO_CROSS] = ("FZ1, open field: two medium-tank squads cross at right angles and reach "
                        "the crossing together."),
    [SCENARIO_HEADON] = ("FZ7, inside a cliff-walled plateau: a light-tank squad and a "
                         "medium-tank squad swap places head-on."),
    [SCENARIO_PARKED] = ("FZ1, open field: a medium-tank squad drives through a parked squad "
                         "standing on its line."),
    [SCENARIO_PASS] = ("MISSION10: two medium-tank squads meet head-on in the winding valleys "
                       "between hill ridges."),
    [SCENARIO_RIDGE] = ("W_MISSION4: medium and heavy tanks side by side drive around a long "
                        "cliff band."),
    [SCENARIO_CORRIDOR] = ("MISSION10: medium, light and heavy tanks follow each other through "
                           "valleys with seven turns."),
    [SCENARIO_INFANTRY] = ("MISSION10: a marine squad walks over two hill ridges and crosses "
                           "the valley of a medium-tank squad."),
    [SCENARIO_HEAVIES] = ("FZ7, inside a cliff-walled plateau: two heavy-tank squads swap "
                          "places head-on, and a third crosses both of their paths."),
    [SCENARIO_WINDING] = ("MISSION6: light, medium, heavy and marine squads drive one after "
                          "another, the fastest first, through wide cliff valleys with frequent, "
                          "sharp turns, and never meet."),
    [SCENARIO_LINE_PLAIN] = ("FZ1, open field: six light, medium and heavy squads stand in a "
                             "line facing east and form a new line along their way, 250 m "
                             "ahead, facing south."),
    [SCENARIO_HEADON_LIGHT] = ("FZ7, inside a cliff-walled plateau: two light-tank squads swap "
                               "places head-on."),
    [SCENARIO_HEADON_HEAVY] = ("FZ7, inside a cliff-walled plateau: two heavy-tank squads swap "
                               "places head-on."),
};
V2 order_destinations[MAX_SQUADS];
V2 order_facings[MAX_SQUADS];
bool has_order[MAX_SQUADS];

/* ------------------------------------------------------------------------------------------------
 * `set_order` gives squad `squad_index` the order to move to `destination`.
 * --------------------------------------------------------------------------------------------- */
static void set_order(int squad_index, V2 destination) {
    order_destinations[squad_index] = destination;
    has_order[squad_index] = true;
}

/* ------------------------------------------------------------------------------------------------
 * `get_direction_between` returns the direction from `from` to `to`, with length 1.
 * --------------------------------------------------------------------------------------------- */
static V2 get_direction_between(V2 from, V2 to) {
    V2 difference = sub(to, from);
    return mul(difference, 1 / len(difference));
}

/* ------------------------------------------------------------------------------------------------
 * `add_default_squad` adds a squad of the team of the player with the default formation of the
 * game.
 * --------------------------------------------------------------------------------------------- */
static void add_default_squad(const Kind *kind, V2 anchor, V2 direction) {
    add_squad(kind, PLAYER_TEAM, anchor, direction, SQUAD_FORMATION_TYPE);
}

/* ------------------------------------------------------------------------------------------------
 * `add_line_squads` sets up the line scenario, `lineplain`. Six squads, a light, a medium and a
 * heavy one twice, stand side by side `LINE_GAP_METERS` apart on a line through `center` across
 * `forward`, all facing `forward`, with the first squad on the right of `forward`. Their new line
 * runs along `forward`, with its middle `LINE_ADVANCE_METERS` ahead of `center`, and the squads
 * keep the same gaps there. The first squad goes to the spot of the new line nearest to the old
 * line, and the last one to the farthest, so their paths do not cross. They end facing to the
 * right of `forward`, across the new line: the whole formation turns a quarter turn clockwise,
 * its line and its facing alike.
 *
 *                  5 >
 *                  4 >
 *                  3 >      end: The new line runs along `forward`, and each squad faces right.
 *                  2 >
 *                  1 >
 *                  0 >
 *
 *            5 4 3 2 1 0    start: The old line runs across `forward` through `center`, and
 *            ^ ^ ^ ^ ^ ^    each squad faces `forward`, which points up.
 *
 * With `--seed`, the starts move as in the other scenarios, so the squads can leave the line a
 * little.
 * --------------------------------------------------------------------------------------------- */
static void add_line_squads(V2 center, V2 forward) {
    static const Kind *const line_kinds[LINE_SQUAD_COUNT] = {
        &LIGHT_TANK, &MEDIUM_TANK, &HEAVY_TANK, &LIGHT_TANK, &MEDIUM_TANK, &HEAVY_TANK,
    };
    V2 right = get_right(forward);
    V2 end_center = add(center, mul(forward, LINE_ADVANCE_METERS));
    float middle_index = (LINE_SQUAD_COUNT - 1) / 2.0f;
    for (int squad_index = 0; squad_index < LINE_SQUAD_COUNT; squad_index++) {
        float offset = (middle_index - squad_index) * LINE_GAP_METERS;
        V2 start = add(center, mul(right, offset));
        V2 end = sub(end_center, mul(forward, offset));
        add_default_squad(line_kinds[squad_index], start, forward);
        set_order(squad_index, end);
        order_facings[squad_index] = right;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `set_up_scenario` hashes `seed` and `scenario_index` into `jitter_state`, so that every mode
 * gets the same starts. It loads the map named in `SCENARIO_MAP`, and a `switch` on
 * `scenario_index` places the squads and sets the orders. The comment of each case describes its
 * layout.
 * --------------------------------------------------------------------------------------------- */
void set_up_scenario(int scenario_index, unsigned seed) {
    /* Restart the random state of `--seed` from the seed and the scenario, so that every mode
     * gets the same starts. The multiplicative hash of Knuth spreads close seeds and scenarios
     * over very different states. */
    unsigned scenario_hash = (unsigned)scenario_index * SCENARIO_HASH_MULTIPLIER + 1;
    jitter_state = 0;
    if (seed)
        jitter_state = (seed * SEED_HASH_MULTIPLIER) ^ scenario_hash;

    /* Clear the previous run and load the map. */
    unit_count = 0;
    squad_count = 0;
    memset(has_order, 0, sizeof has_order);
    memset(order_facings, 0, sizeof order_facings);
    load_map(SCENARIO_MAP[scenario_index]);

    /* Place the squads and give their orders. */
    switch (scenario_index) {
    case SCENARIO_CROSS: {
        /* Two squads cross at right angles at (3100, 2700). The first drives east and the second
         * north, and each starts 250 m from the crossing. */
        add_default_squad(&MEDIUM_TANK, v2(2850, 2700), v2(1, 0));
        add_default_squad(&MEDIUM_TANK, v2(3100, 2450), v2(0, 1));
        set_order(0, v2(3350, 2700));
        set_order(1, v2(3100, 2950));
        break;
    }
    case SCENARIO_HEADON: {
        /* Two squads swap places along the plateau: the light tanks start at its south end,
         * `start`, and the medium tanks at its north end, `end`, about 535 m away. */
        V2 start = v2(3030, 2950);
        V2 end = v2(3100, 3480);
        add_default_squad(&LIGHT_TANK, start, get_direction_between(start, end));
        add_default_squad(&MEDIUM_TANK, end, get_direction_between(end, start));
        set_order(0, end);
        set_order(1, start);
        break;
    }
    case SCENARIO_PARKED:
        /* The second squad drives 500 m east through the first squad, which stays parked at the
         * middle of that drive and faces north. */
        add_default_squad(&MEDIUM_TANK, v2(3100, 2700), v2(0, 1));
        add_default_squad(&MEDIUM_TANK, v2(2850, 2700), v2(1, 0));
        set_order(1, v2(3350, 2700));
        break;
    case SCENARIO_PASS: {
        /* Two squads swap places through the valleys, between `start` in the east and `end`
         * about 690 m to the west. */
        V2 start = v2(2102, 816);
        V2 end = v2(1409, 839);
        add_default_squad(&MEDIUM_TANK, start, get_direction_between(start, end));
        add_default_squad(&MEDIUM_TANK, end, get_direction_between(end, start));
        set_order(0, end);
        set_order(1, start);
        break;
    }
    case SCENARIO_RIDGE: {
        /* Two squads drive side by side around the cliff band, from `start` to `end`, about
         * 650 m to the south-east. The heavy squad drives `SIDE_BY_SIDE_METERS` to the left of
         * the medium one. */
        V2 start = v2(2528, 4265);
        V2 end = v2(2923, 3745);
        V2 direction = get_direction_between(start, end);
        V2 left = v2(-direction.y, direction.x);
        V2 side_offset = mul(left, SIDE_BY_SIDE_METERS);
        add_default_squad(&MEDIUM_TANK, start, direction);
        add_default_squad(&HEAVY_TANK, add(start, side_offset), direction);
        set_order(0, end);
        set_order(1, add(end, side_offset));
        break;
    }
    case SCENARIO_CORRIDOR: {
        /* Three squads drive one after another through the valleys, toward `end`, about 650 m
         * to the west-north-west. The medium squad leads from `start`, and the light and the
         * heavy squads start `FOLLOW_GAP_METERS` and twice that behind it. The light squad ends
         * `SPREAD_DESTINATION_METERS` to the left of `end`, and the heavy squad as far to the
         * right. */
        V2 start = v2(2407, 1450);
        V2 end = v2(1850, 1778);
        V2 direction = get_direction_between(start, end);
        V2 left = v2(-direction.y, direction.x);
        V2 destination_offset = mul(left, SPREAD_DESTINATION_METERS);
        V2 light_start = sub(start, mul(direction, FOLLOW_GAP_METERS));
        V2 heavy_start = sub(start, mul(direction, 2 * FOLLOW_GAP_METERS));
        add_default_squad(&MEDIUM_TANK, start, direction);
        add_default_squad(&LIGHT_TANK, light_start, direction);
        add_default_squad(&HEAVY_TANK, heavy_start, direction);
        set_order(0, end);
        set_order(1, add(end, destination_offset));
        set_order(2, sub(end, destination_offset));
        break;
    }
    case SCENARIO_INFANTRY: {
        /* The marines walk east over two hill ridges, and the tanks drive north along the valley
         * between the ridges. The starts make the two squads reach the crossing together. */
        V2 marine_start = v2(1369, 839);
        V2 marine_end = v2(1745, 823);
        V2 tank_start = v2(1610, 580);
        V2 tank_end = v2(1610, 1090);
        add_default_squad(&MARINE, marine_start, get_direction_between(marine_start, marine_end));
        add_default_squad(&MEDIUM_TANK, tank_start, get_direction_between(tank_start, tank_end));
        set_order(0, marine_end);
        set_order(1, tank_end);
        break;
    }
    case SCENARIO_HEAVIES: {
        /* Two squads swap the ends of `headon`, and the third starts beside the first and drives
         * diagonally across the plateau, through the paths of both. */
        V2 south = v2(3030, 2950);
        V2 north = v2(3100, 3480);
        V2 cross_start = v2(3130, 2980);
        V2 cross_end = v2(3040, 3440);
        add_default_squad(&HEAVY_TANK, south, get_direction_between(south, north));
        add_default_squad(&HEAVY_TANK, north, get_direction_between(north, south));
        add_default_squad(&HEAVY_TANK, cross_start, get_direction_between(cross_start, cross_end));
        set_order(0, north);
        set_order(1, south);
        set_order(2, cross_end);
        break;
    }
    case SCENARIO_WINDING: {
        /* The tank squads drive the same winding valleys one after another, the fastest first,
         * so that none catches up with the one ahead. They start on the open ground along the
         * first leg of their route, south-south-east of `rear_start`, each `WINDING_GAP_METERS`
         * ahead of the next slower one. They end on the open ground in the west: the light squad
         * at `front_end`, and each slower squad `WINDING_GAP_METERS` further along
         * `back_from_end`, which points back toward the last passage. So every tank squad drives
         * about the same distance, and none passes a parked squad. `first_leg` and
         * `back_from_end` are directions of about length 1.
         * Marines walk over hills, which would cut the valleys of the tanks short, so they have
         * a course of their own between cliffs, to the west of the start of the tanks. It ends
         * about 160 m west of `rear_start`, long after the tanks left. */
        static const Kind *const tank_kinds[WINDING_TANK_SQUAD_COUNT] = {
            &LIGHT_TANK, &MEDIUM_TANK, &HEAVY_TANK,
        };
        V2 rear_start = v2(2574, 5566);
        V2 first_leg = v2(0.25f, -0.97f);
        V2 front_end = v2(2237, 5087);
        V2 back_from_end = v2(0.62f, -0.79f);
        for (int squad_index = 0; squad_index < WINDING_TANK_SQUAD_COUNT; squad_index++) {
            int slower_count = WINDING_TANK_SQUAD_COUNT - 1 - squad_index;
            V2 start = add(rear_start, mul(first_leg, slower_count * WINDING_GAP_METERS));
            V2 end = add(front_end, mul(back_from_end, squad_index * WINDING_GAP_METERS));
            add_default_squad(tank_kinds[squad_index], start, get_direction_between(start, end));
            set_order(squad_index, end);
        }
        V2 marine_start = v2(1872, 5697);
        V2 marine_end = v2(2413, 5565);
        add_default_squad(&MARINE, marine_start, get_direction_between(marine_start, marine_end));
        set_order(WINDING_TANK_SQUAD_COUNT, marine_end);
        break;
    }
    case SCENARIO_LINE_PLAIN:
        /* The old line runs north to south through (2850, 2700), and its squads face east. The
         * lines stand on the open field of `cross`, which is open about 400 m around them. */
        add_line_squads(v2(2850, 2700), v2(1, 0));
        break;
    case SCENARIO_HEADON_LIGHT:
    case SCENARIO_HEADON_HEAVY: {
        /* Two squads of one kind swap the ends of `headon`: light tanks, which meet at the top
         * speed of the fastest tanks, or heavy tanks, the widest and slowest-turning ones. */
        const Kind *kind = &LIGHT_TANK;
        if (scenario_index == SCENARIO_HEADON_HEAVY)
            kind = &HEAVY_TANK;
        V2 start = v2(3030, 2950);
        V2 end = v2(3100, 3480);
        add_default_squad(kind, start, get_direction_between(start, end));
        add_default_squad(kind, end, get_direction_between(end, start));
        set_order(0, end);
        set_order(1, start);
        break;
    }
    }
}
