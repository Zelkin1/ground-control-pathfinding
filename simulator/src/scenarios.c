// Scenarios of the simulator: for each scenario its map, its squads and their orders. Each
// scenario lays out its squads with the formation layout of the game at a row center, the center
// of the first row of the squad. The map coordinates are in meters, with x to the east and y to
// the north.
//
// A seed other than 0 (`--seed`) moves the start of each squad by up to `SQUAD_JITTER_SPANS`
// times the size of the squad, and then each unit by up to `UNIT_JITTER_METERS` on each axis and
// `UNIT_JITTER_RADIANS` in angle, onto free terrain. Seed 0 keeps the layouts as written. The
// random numbers come in the order of the squads, so the order of the `add_default_squad` calls is
// part of each scenario.
//
// The variants of the command line change every squad of a scenario: its faction (`--faction`),
// its number of squad members, as after losses (`--members`), its layout (`--layout`) and its
// armor profile (`--profile`). `--boost` gives every unit the speed booster of the game during a
// run, `--boostcycle` turns it on and off by turns, and `--booststagger` starts it later for each
// squad. The defaults keep the scenarios as written.
#include <stdlib.h>
#include <string.h>

#include "sim.h"

#define PLAYER_TEAM 1                           // team of every squad
#define SQUARE_FORMATION_TYPE 1                 // game formation type of about sqrt(n) per row
#define LINE_FORMATION_TYPE 0                   // game formation type of one row
#define ALL_MEMBERS 0                           // `max_member_count` that keeps every member
#define START_GAP_METERS 0.5f                   // smallest gap between two bodies at the start
#define SQUAD_JITTER_ATTEMPTS 50                // most tries to move a squad
#define SQUAD_JITTER_SPANS 1.5f                 // largest squad move, in squad spans
#define UNIT_JITTER_METERS 1.5f                 // largest unit move on each axis
#define UNIT_JITTER_RADIANS 0.05f               // largest unit turn, either way
#define SEED_HASH_MULTIPLIER 2654435761u        // 2^32 divided by the golden ratio
#define SCENARIO_HASH_MULTIPLIER 40503u         // 2^16 divided by the golden ratio

#define RIDGE_GAP_METERS 50.0f                  // `ridge`: gap between the two squads
#define FOLLOW_GAP_METERS 45.0f                 // `corridor`: gap between the squads in line
#define SPREAD_DESTINATION_METERS 40.0f         // `corridor`: side offset of rear destinations
#define WINDING_TANK_SQUAD_COUNT 3              // `winding`: number of tank squads
#define WINDING_GAP_METERS 60.0f                // `winding`: gap between the squads
#define SIX_SQUADS_COUNT 6                      // `sixsquads`: number of squads
#define SIX_SQUADS_GAP_METERS 40.0f             // `sixsquads`: gap between squads in a row
#define SIX_SQUADS_ADVANCE_METERS 250.0f        // `sixsquads`: distance of the new row
#define REORDER_SECONDS 6.0f                    // `reorder`: time of the second orders
#define CLOSE_RETURN_SECONDS 16.0f              // `closequarters`: time of the return orders
#define BOARDING_SQUAD_GAP_METERS 18.0f         // `boarding`: distance of the marine row centers
#define BOARDING_APC_BEHIND_METERS 60.0f        // `boarding`: APC start behind the marines
#define BOARDING_APC_PAST_METERS 150.0f         // `boarding`: APC destination past the marines
#define BOARDING_APC_ORDER_SECONDS 2.0f         // `boarding`: time of the APC order
#define MARINE_ORDERS_SQUAD_GAP_METERS 12.0f    // `marineorders`: distance of the marine rows
#define MARINE_ORDERS_PERIOD_SECONDS 8.0f       // `marineorders`: time between two orders
#define MARINE_ORDERS_COUNT 5                   // `marineorders`: orders of each marine squad
#define STRESS_COLUMN_COUNT 8                   // `stress`: squads in each row of the grid
#define STRESS_ROW_COUNT 4                      // `stress`: rows of the grid
#define STRESS_TYPE_COUNT 4                     // `stress`: unit types, by turns along a row
#define STRESS_GAP_METERS 40.0f                 // `stress`: distance of neighbor grid places
#define STRESS_ADVANCE_METERS 300.0f            // `stress`: distance of the east grid
#define STRESS_ORDER_GAP_SECONDS 0.1f           // `stress`: time between two squad orders
#define STRESS_WAVE_SECONDS 6.0f                // `stress`: time between two order waves

#define BOOSTER_FACTOR 1.5f                     // SPEED.ATM: top speed and acceleration factor
#define BOOSTER_SECONDS 60.0f                   // SPEED.ATM: duration of the speed booster

const char *LAYOUT_NAME[LAYOUT_COUNT] = {
    [LAYOUT_SQUARE] = "square",
    [LAYOUT_LINE] = "line",
};
static const int LAYOUT_FORMATION_TYPE[LAYOUT_COUNT] = {
    [LAYOUT_SQUARE] = SQUARE_FORMATION_TYPE,
    [LAYOUT_LINE] = LINE_FORMATION_TYPE,
};

// Variants of the scenarios, from the command line.
int scenario_faction = FACTION_CRAYVEN;
int max_member_count = ALL_MEMBERS;
int scenario_layout = LAYOUT_SQUARE;
int scenario_profile = PROFILE_BALANCED;
float boost_start_seconds = NO_BOOST;
float boost_cycle_seconds = NO_BOOST_CYCLE;
float boost_stagger_seconds = 0;

static const V2 EAST = {
    .x = 1,
    .y = 0,
};
static const V2 NORTH = {
    .x = 0,
    .y = 1,
};

// Scenarios, in the order of their indexes.
enum {
    SCENARIO_CROSS,
    SCENARIO_HEADON,
    SCENARIO_PARKED,
    SCENARIO_HEADON_VALLEYS,
    SCENARIO_RIDGE,
    SCENARIO_CORRIDOR,
    SCENARIO_HEAVIES,
    SCENARIO_WINDING,
    SCENARIO_SIX_SQUADS,
    SCENARIO_HEADON_LIGHT,
    SCENARIO_HEADON_HEAVY,
    SCENARIO_SINGLE_UNITS,
    SCENARIO_REORDER,
    SCENARIO_CLOSE_QUARTERS,
    SCENARIO_BOARDING,
    SCENARIO_MARINE_ORDERS,
    SCENARIO_PARKED_END,
    SCENARIO_STRESS,
    SCENARIO_REVERSE,
};

// Key of the jitter of each scenario: its index at the time when its jitter was fixed. A removed
// scenario leaves the keys of the others as they are, so each seed keeps its layouts.
static const unsigned SCENARIO_JITTER_KEY[SCENARIO_COUNT] = {
    [SCENARIO_CROSS] = 0,
    [SCENARIO_HEADON] = 1,
    [SCENARIO_PARKED] = 2,
    [SCENARIO_HEADON_VALLEYS] = 3,
    [SCENARIO_RIDGE] = 4,
    [SCENARIO_CORRIDOR] = 5,
    [SCENARIO_HEAVIES] = 7,
    [SCENARIO_WINDING] = 8,
    [SCENARIO_SIX_SQUADS] = 9,
    [SCENARIO_HEADON_LIGHT] = 10,
    [SCENARIO_HEADON_HEAVY] = 11,
    [SCENARIO_SINGLE_UNITS] = 12,
    [SCENARIO_REORDER] = 13,
    [SCENARIO_CLOSE_QUARTERS] = 14,
    [SCENARIO_BOARDING] = 15,
    [SCENARIO_MARINE_ORDERS] = 16,
    [SCENARIO_PARKED_END] = 17,
    [SCENARIO_STRESS] = 18,
    [SCENARIO_REVERSE] = 19,
};

const int SCENARIO_RUN_ORDER[RUN_ORDER_COUNT] = {
    SCENARIO_CROSS,
    SCENARIO_HEADON,
    SCENARIO_HEADON_LIGHT,
    SCENARIO_HEADON_HEAVY,
    SCENARIO_HEADON_VALLEYS,
    SCENARIO_HEAVIES,
    SCENARIO_PARKED,
    SCENARIO_RIDGE,
    SCENARIO_CORRIDOR,
    SCENARIO_WINDING,
    SCENARIO_SIX_SQUADS,
    SCENARIO_SINGLE_UNITS,
    SCENARIO_REORDER,
    SCENARIO_CLOSE_QUARTERS,
    SCENARIO_BOARDING,
    SCENARIO_MARINE_ORDERS,
    SCENARIO_PARKED_END,
};

const char *SCENARIO_NAME[SCENARIO_COUNT] = {
    [SCENARIO_CROSS] = "cross",
    [SCENARIO_HEADON] = "headon",
    [SCENARIO_PARKED] = "parked",
    [SCENARIO_HEADON_VALLEYS] = "headonvalleys",
    [SCENARIO_RIDGE] = "ridge",
    [SCENARIO_CORRIDOR] = "corridor",
    [SCENARIO_HEAVIES] = "heavies",
    [SCENARIO_WINDING] = "winding",
    [SCENARIO_SIX_SQUADS] = "sixsquads",
    [SCENARIO_HEADON_LIGHT] = "headonlight",
    [SCENARIO_HEADON_HEAVY] = "headonheavy",
    [SCENARIO_SINGLE_UNITS] = "singleunits",
    [SCENARIO_REORDER] = "reorder",
    [SCENARIO_CLOSE_QUARTERS] = "closequarters",
    [SCENARIO_BOARDING] = "boarding",
    [SCENARIO_MARINE_ORDERS] = "marineorders",
    [SCENARIO_PARKED_END] = "parkedend",
    [SCENARIO_STRESS] = "stress",
    [SCENARIO_REVERSE] = "reverse",
};

static const char *SCENARIO_MAP[SCENARIO_COUNT] = {
    [SCENARIO_CROSS] = "FZ1",
    [SCENARIO_HEADON] = "FZ7",
    [SCENARIO_PARKED] = "FZ1",
    [SCENARIO_HEADON_VALLEYS] = "MISSION10",
    [SCENARIO_RIDGE] = "W_MISSION4",
    [SCENARIO_CORRIDOR] = "MISSION10",
    [SCENARIO_HEAVIES] = "FZ7",
    [SCENARIO_WINDING] = "MISSION6",
    [SCENARIO_SIX_SQUADS] = "FZ1",
    [SCENARIO_HEADON_LIGHT] = "FZ7",
    [SCENARIO_HEADON_HEAVY] = "FZ7",
    [SCENARIO_SINGLE_UNITS] = "FZ7",
    [SCENARIO_REORDER] = "FZ1",
    [SCENARIO_CLOSE_QUARTERS] = "FZ1",
    [SCENARIO_BOARDING] = "FZ1",
    [SCENARIO_MARINE_ORDERS] = "FZ1",
    [SCENARIO_PARKED_END] = "FZ1",
    [SCENARIO_STRESS] = "FZ1",
    [SCENARIO_REVERSE] = "FZ1",
};

const char *SCENARIO_DESCRIPTION[SCENARIO_COUNT] = {
    [SCENARIO_CROSS] = ("FZ1, open field: two medium-tank squads cross at right angles and reach "
                        "the crossing together."),
    [SCENARIO_HEADON] = ("FZ7, on a plateau bounded by cliffs: a light-tank squad and a "
                         "medium-tank squad meet head-on as they swap ends."),
    [SCENARIO_PARKED] = ("FZ1, open field: a medium-tank squad drives through a parked squad "
                         "that stands in its way."),
    [SCENARIO_HEADON_VALLEYS] = ("MISSION10: two medium-tank squads meet head-on in the winding "
                                 "valleys between hill ridges."),
    [SCENARIO_RIDGE] = ("W_MISSION4: medium and heavy tanks drive next to each other around a "
                        "long cliff band."),
    [SCENARIO_CORRIDOR] = ("MISSION10: medium, light and heavy tanks follow each other through "
                           "valleys with seven turns."),
    [SCENARIO_HEAVIES] = ("FZ7, on a plateau bounded by cliffs: two heavy-tank squads meet "
                          "head-on as they swap ends, and a third crosses both of their routes."),
    [SCENARIO_WINDING] = ("MISSION6: light, medium, heavy and marine squads drive one after "
                          "another, the fastest first, through wide cliff valleys with frequent, "
                          "sharp turns, and never meet."),
    [SCENARIO_SIX_SQUADS] = ("FZ1, open field: six light, medium and heavy squads stand in a "
                             "row facing east and form a new row along their way, 250 m "
                             "ahead, facing south."),
    [SCENARIO_HEADON_LIGHT] = ("FZ7, on a plateau bounded by cliffs: two light-tank squads meet "
                               "head-on as they swap ends."),
    [SCENARIO_HEADON_HEAVY] = ("FZ7, on a plateau bounded by cliffs: two heavy-tank squads meet "
                               "head-on as they swap ends."),
    [SCENARIO_SINGLE_UNITS] = ("FZ7, on a plateau bounded by cliffs: a medium-tank squad drives "
                               "north, and three command APCs, each a squad of one unit, meet it "
                               "head-on, cross its route and follow it. South of the plateau, a "
                               "fourth APC turns around and drives alone."),
    [SCENARIO_REORDER] = ("FZ1, open field: a light, a medium and a heavy squad drive east, and "
                          "6 s after the start each gets a new order: the light squad turns left "
                          "at right angles, the medium squad turns back to the left at 135 "
                          "degrees, and the heavy squad turns around."),
    [SCENARIO_CLOSE_QUARTERS] = ("FZ1, open field: two marine squads meet head-on, and a medium "
                                 "and a light tank squad meet head-on across their way, all four "
                                 "at the same point and the same time; 16 s after the start, "
                                 "each squad turns back to its start, and they meet again."),
    [SCENARIO_BOARDING] = ("FZ1, open field: two marine squads stand next to each other and get "
                           "the order to board a command APC 60 m behind them. 2 s later, the APC "
                           "gets the order to drive to a point 150 m past them, so its way leads "
                           "through them."),
    [SCENARIO_MARINE_ORDERS] = ("FZ1, open field: two marine squads stand next to each other, "
                                "close to a parked light-tank squad, and get five move orders "
                                "8 s apart, each in another direction, that lead past the "
                                "tanks."),
    [SCENARIO_PARKED_END] = ("FZ1, open field: a medium-tank squad gets an order to the spot "
                             "where a parked medium-tank squad stands, so its final formation "
                             "overlaps the parked squad."),
    [SCENARIO_STRESS] = ("FZ1, open field, stress test: 32 light, medium, heavy and marine squads "
                         "stand in a grid of 8 columns and 4 rows. In three waves 6 s apart, the "
                         "squads get orders one after another, 0.1 s apart: to a grid 300 m east, "
                         "then to the mirrored places of that grid, then back to the start "
                         "grid."),
    [SCENARIO_REVERSE] = ("FZ1, open field: four squads face east and get backward move orders. "
                          "A medium-tank squad backs to a point behind it, a heavy-tank squad to a "
                          "point behind and to its right, and a light-tank squad turns around on "
                          "the spot and backs to a point ahead of it; a marine squad walks "
                          "forward, since infantry never reverses."),
};

// Orders of the squads of the current scenario, by squad index. A zero facing means none.
V2 order_destinations[MAX_SQUADS];
V2 order_facings[MAX_SQUADS];
bool has_order[MAX_SQUADS];
float order_seconds[MAX_SQUADS];
V2 second_order_destinations[MAX_SQUADS];
float second_order_seconds[MAX_SQUADS];
bool has_second_order[MAX_SQUADS];
TimedOrder timed_orders[MAX_TIMED_ORDERS];
int timed_order_count;

// State of the random numbers of the seed, or 0 without jitter.
static unsigned jitter_state;


// `draw_jitter` draws a random number from -1 up to 1.
static float draw_jitter(void) {
    return next_random(&jitter_state) / (RANDOM_RANGE / 2) - 1;
}


// `is_free_spot` tells whether a body of `radius` at `point` stands on a cell that it can cross,
// at least `START_GAP_METERS` from the bodies of the units already placed.
static bool is_free_spot(V2 point, float radius, int terrain_class) {
    if (gc_get_cell_at(point.x, point.y) > terrain_class)
        return false;
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        float d_min = radius + units[unit_index].unit_type->radius + START_GAP_METERS;
        float d_squared = length_squared(sub(units[unit_index].position, point));
        if (d_squared < d_min * d_min)
            return false;
    }
    return true;
}


// `measure_squad_span` returns the span of a squad: the largest distance between two of its
// final parking positions, and at least the width of one body.
static float measure_squad_span(const UnitType *unit_type, const float *parking_positions,
                                int parking_count) {
    float span = 2 * unit_type->radius;
    for (int first_index = 0; first_index < parking_count; first_index++) {
        for (int second_index = first_index + 1; second_index < parking_count; second_index++) {
            V2 first = get_pair_point(parking_positions, first_index);
            V2 second = get_pair_point(parking_positions, second_index);
            span = fmaxf(span, len(sub(first, second)));
        }
    }
    return span;
}


// `jitter_squad` moves the layout of a squad of `count` squad members by a random offset of up
// to `SQUAD_JITTER_SPANS` spans, in `SQUAD_JITTER_ATTEMPTS` tries at most. A try needs a clear
// line from the row center, a layout of the game, and free spots; without one, the layout stays.
static void jitter_squad(const UnitType *unit_type, int count, V2 row_center, V2 direction,
                         int formation_type, float *parking_positions) {
    // Offsets outside the disc of the largest offset are drawn again.
    int terrain_class = unit_type->terrain_class;
    float span = measure_squad_span(unit_type, parking_positions, count);
    float max_offset = SQUAD_JITTER_SPANS * span;
    for (int attempt_index = 0; attempt_index < SQUAD_JITTER_ATTEMPTS; attempt_index++) {
        float offset_x = draw_jitter();
        float offset_y = draw_jitter();
        V2 offset = mul(v2(offset_x, offset_y), max_offset);
        V2 moved_center = add(row_center, offset);
        bool is_short_enough = len(offset) <= max_offset;
        if (!is_short_enough)
            continue;
        int ray = gc_cast_ray(row_center.x, row_center.y, moved_center.x, moved_center.y,
                              terrain_class);
        if (ray != GC_RAY_CLEAR)
            continue;

        // Every final parking position of the moved layout must be a free spot.
        float moved[2 * MAX_UNITS];        // moved final parking positions
        bool is_laid_out = gc_lay_out_formation(moved_center.x, moved_center.y, direction.x,
                                                direction.y, count, unit_type->radius,
                                                terrain_class, formation_type, moved);
        if (!is_laid_out)
            continue;
        bool is_free = true;
        for (int parking_index = 0; parking_index < count && is_free; parking_index++) {
            V2 parking_position = get_pair_point(moved, parking_index);
            is_free = is_free_spot(parking_position, unit_type->radius, terrain_class);
        }
        if (is_free) {
            memcpy(parking_positions, moved, sizeof(float) * 2 * count);
            return;
        }
    }
}


// `add_unit` adds a unit to the current squad, standing at `position` with the heading angle
// `angle` and the facing `direction`, ready for an order. Its top speed follows the armor profile
// of the scenario.
static void add_unit(const UnitType *unit_type, int team, V2 position, float angle, V2 direction) {
    Unit *unit = &units[unit_count];
    unit_count++;
    memset(unit, 0, sizeof *unit);
    unit->unit_type = unit_type;
    unit->squad_index = squad_count;
    unit->team = team;
    unit->target_unit_index = NO_TARGET_UNIT;
    unit->base_top_speed = unit_type->top_speed * PROFILE_SPEED_MODIFIER[scenario_profile];
    unit->top_speed = unit->base_top_speed;
    unit->acceleration = unit_type->acceleration;

    // The angle wraps into `[0, 2 * PI)`, clockwise from north.
    unit->position = position;
    unit->start_position = position;
    unit->parking_position = position;
    unit->angle = angle;
    if (unit->angle < 0)
        unit->angle += 2 * PI;
    if (unit->angle >= 2 * PI)
        unit->angle -= 2 * PI;
    unit->heading = v2(sinf(unit->angle), cosf(unit->angle));
    unit->previous_angle = unit->angle;
    unit->facing = direction;

    // A new unit stands as after a stop of the game.
    unit->waypoint = unit->position;
    unit->reach = STOP_REACH_METERS;
    unit->reach_last = STOP_REACH_METERS;
    unit->reach_mid = MID_REACH_METERS;
    unit->is_stopped = true;
    unit->blocker_index = NO_BLOCKER;
    unit->wait_seconds = WAIT_TIMER_AT_ORDER_SECONDS;
}


// `add_squad` adds a squad whose units stand on the formation layout of the game at `row_center`,
// facing `direction`, moved by the jitter of the seed. The squad has the squad size of its unit
// type, or `max_member_count` members when that is positive and smaller. The program ends when
// the layout fails.
static void add_squad(const UnitType *unit_type, int team, V2 row_center, V2 direction,
                      int formation_type) {
    int member_count = unit_type->units_per_squad;
    bool is_reduced = max_member_count > ALL_MEMBERS && max_member_count < member_count;
    if (is_reduced)
        member_count = max_member_count;
    Squad *squad = &squads[squad_count];
    memset(squad, 0, sizeof *squad);
    squad->unit_type = unit_type;
    squad->squad_index = squad_count;
    squad->first_unit_index = unit_count;
    squad->unit_count = member_count;
    squad->formation_type = formation_type;
    squad->last_facing = direction;
    gc_init_request(&squad->search);

    float parking_positions[2 * MAX_UNITS];
    bool is_laid_out = gc_lay_out_formation(row_center.x, row_center.y, direction.x, direction.y,
                                            member_count, unit_type->radius,
                                            unit_type->terrain_class, formation_type,
                                            parking_positions);
    if (!is_laid_out) {
        fprintf(stderr, "cannot lay out squad %d\n", squad_count);
        exit(EXIT_FAILURE);
    }
    if (jitter_state)
        jitter_squad(unit_type, member_count, row_center, direction, formation_type,
                     parking_positions);

    // A unit move that would not land on a free spot is dropped, but the unit still turns.
    float direction_angle = atan2f(direction.x, direction.y);
    for (int parking_index = 0; parking_index < member_count; parking_index++) {
        V2 position = get_pair_point(parking_positions, parking_index);
        float angle = direction_angle;
        if (jitter_state) {
            float offset_x = draw_jitter() * UNIT_JITTER_METERS;
            float offset_y = draw_jitter() * UNIT_JITTER_METERS;
            V2 moved = add(position, v2(offset_x, offset_y));
            if (is_free_spot(moved, unit_type->radius, unit_type->terrain_class))
                position = moved;
            angle += draw_jitter() * UNIT_JITTER_RADIANS;
        }
        add_unit(unit_type, team, position, angle, direction);
    }
    squad_count++;
}


// `set_order` orders the squad `squad_index` to `destination`.
static void set_order(int squad_index, V2 destination) {
    order_destinations[squad_index] = destination;
    has_order[squad_index] = true;
}


// `set_second_order` orders the squad `squad_index` to `destination` once the run reaches
// `seconds`.
static void set_second_order(int squad_index, float seconds, V2 destination) {
    second_order_destinations[squad_index] = destination;
    second_order_seconds[squad_index] = seconds;
    has_second_order[squad_index] = true;
}


// `add_timed_order` orders the squad `squad_index` to `destination` once the run reaches
// `seconds`. Orders past `MAX_TIMED_ORDERS` are dropped.
static void add_timed_order(int squad_index, float seconds, V2 destination) {
    if (timed_order_count == MAX_TIMED_ORDERS)
        return;
    timed_orders[timed_order_count] = (TimedOrder){
        .squad_index = squad_index,
        .seconds = seconds,
        .destination = destination,
        .is_given = false,
    };
    timed_order_count++;
}


// `get_direction_between` returns the unit direction from `from` to `to`.
static V2 get_direction_between(V2 from, V2 to) {
    V2 difference = sub(to, from);
    return mul(difference, 1 / len(difference));
}


// `add_default_squad` adds a squad of the player team, with the unit type of the scenario faction
// in the role of `unit_type` and the formation type of the scenario layout.
static void add_default_squad(const UnitType *unit_type, V2 row_center, V2 direction) {
    const UnitType *faction_type = get_faction_unit_type(unit_type, scenario_faction);
    int formation_type = LAYOUT_FORMATION_TYPE[scenario_layout];
    add_squad(faction_type, PLAYER_TEAM, row_center, direction, formation_type);
}


// `set_up_cross` sets up `cross` on FZ1: two medium-tank squads, one driving east and one
// driving north, cross at right angles and reach the crossing together.
static void set_up_cross(void) {
    add_default_squad(&MEDIUM_TANK, v2(2850, 2700), EAST);
    add_default_squad(&MEDIUM_TANK, v2(3100, 2450), NORTH);
    set_order(0, v2(3350, 2700));
    set_order(1, v2(3100, 2950));
}


// `set_up_plateau_head_on` sets up the head-on scenarios on the plateau of FZ7: a squad of
// `south_type` at the south end and a squad of `north_type` at the north end swap ends.
static void set_up_plateau_head_on(const UnitType *south_type, const UnitType *north_type) {
    V2 south = v2(3030, 2950);
    V2 north = v2(3100, 3480);
    add_default_squad(south_type, south, get_direction_between(south, north));
    add_default_squad(north_type, north, get_direction_between(north, south));
    set_order(0, north);
    set_order(1, south);
}


// `set_up_parked` sets up `parked` on FZ1: a medium-tank squad drives east through a parked
// medium-tank squad that has no order.
static void set_up_parked(void) {
    add_default_squad(&MEDIUM_TANK, v2(3100, 2700), NORTH);
    add_default_squad(&MEDIUM_TANK, v2(2850, 2700), EAST);
    set_order(1, v2(3350, 2700));
}


// `set_up_parked_end` sets up `parkedend` on FZ1: a medium-tank squad gets an order to a point at
// the west edge of a parked medium-tank squad that has no order. The game lays out the front row
// of the final formation on that point, so it overlaps the west row of the parked squad.
static void set_up_parked_end(void) {
    add_default_squad(&MEDIUM_TANK, v2(3100, 2700), NORTH);
    add_default_squad(&MEDIUM_TANK, v2(2850, 2700), EAST);
    set_order(1, v2(3092, 2694));
}


// `set_up_head_on_valleys` sets up `headonvalleys` on MISSION10: two medium-tank squads swap
// ends through the winding valleys between hill ridges and meet head-on.
static void set_up_head_on_valleys(void) {
    V2 east = v2(2102, 816);
    V2 west = v2(1409, 839);
    add_default_squad(&MEDIUM_TANK, east, get_direction_between(east, west));
    add_default_squad(&MEDIUM_TANK, west, get_direction_between(west, east));
    set_order(0, west);
    set_order(1, east);
}


// `set_up_ridge` sets up `ridge` on W_MISSION4: a medium-tank squad and a heavy-tank squad
// `RIDGE_GAP_METERS` to its left drive side by side around a long cliff band.
static void set_up_ridge(void) {
    // The left is the left of the drive direction, as seen facing along it.
    V2 start = v2(2528, 4265);
    V2 end = v2(2923, 3745);
    V2 direction = get_direction_between(start, end);
    V2 left = v2(-direction.y, direction.x);
    V2 left_offset = mul(left, RIDGE_GAP_METERS);
    add_default_squad(&MEDIUM_TANK, start, direction);
    add_default_squad(&HEAVY_TANK, add(start, left_offset), direction);
    set_order(0, end);
    set_order(1, add(end, left_offset));
}


// `set_up_corridor` sets up `corridor` on MISSION10: a medium, a light and a heavy squad start in
// line, `FOLLOW_GAP_METERS` apart, and follow each other through valleys with seven turns.
//
//   heavy ->  light ->  medium ->  ...  -> medium destination
//                                          light and heavy destinations
//                                          `SPREAD_DESTINATION_METERS` to each side
//
static void set_up_corridor(void) {
    // The light squad ends to the left of the medium squad, and the heavy squad to its right.
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
}


// `set_up_heavies` sets up `heavies` on the plateau of FZ7: two heavy-tank squads swap ends and
// meet head-on, and a third heavy-tank squad crosses both of their routes.
static void set_up_heavies(void) {
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
}


// `set_up_winding` sets up `winding` on MISSION6: a light, a medium and a heavy squad drive one
// after another through wide cliff valleys with sharp turns, the fastest first, and a marine
// squad drives elsewhere. The squads never meet.
static void set_up_winding(void) {
    // The faster squads start further along the first route segment, `WINDING_GAP_METERS` per
    // slower squad behind them, and end further back from the route end in the same way.
    static const UnitType *const tank_types[WINDING_TANK_SQUAD_COUNT] = {
        &LIGHT_TANK, &MEDIUM_TANK, &HEAVY_TANK,
    };
    V2 rear_start = v2(2574, 5566);
    V2 first_segment = v2(0.25f, -0.97f);
    V2 front_end = v2(2237, 5087);
    V2 back_from_end = v2(0.62f, -0.79f);
    for (int squad_index = 0; squad_index < WINDING_TANK_SQUAD_COUNT; squad_index++) {
        int slower_count = WINDING_TANK_SQUAD_COUNT - 1 - squad_index;
        V2 start = add(rear_start, mul(first_segment, slower_count * WINDING_GAP_METERS));
        V2 end = add(front_end, mul(back_from_end, squad_index * WINDING_GAP_METERS));
        add_default_squad(tank_types[squad_index], start, get_direction_between(start, end));
        set_order(squad_index, end);
    }

    V2 marine_start = v2(1872, 5697);
    V2 marine_end = v2(2413, 5565);
    add_default_squad(&MARINE, marine_start, get_direction_between(marine_start, marine_end));
    set_order(WINDING_TANK_SQUAD_COUNT, marine_end);
}


// `set_up_six_squads` sets up `sixsquads` on FZ1: six squads stand in a row across the east
// direction and drive to a new row `SIX_SQUADS_ADVANCE_METERS` ahead, which runs along the way and
// faces south. The squad at one end of the old row goes to the near end of the new row.
//
//   old row (facing east)          new row (facing south)
//      0                           5  4  3  2  1  0
//      1      ------------->
//      ...
//      5
//
static void set_up_six_squads(void) {
    // `offset` places a squad along the old row, and the same offset back from the far end
    // places it along the new row.
    static const UnitType *const six_squads_types[SIX_SQUADS_COUNT] = {
        &LIGHT_TANK, &MEDIUM_TANK, &HEAVY_TANK, &LIGHT_TANK, &MEDIUM_TANK, &HEAVY_TANK,
    };
    V2 center = v2(2850, 2700);
    V2 forward = EAST;
    V2 right = get_right(forward);
    V2 end_center = add(center, mul(forward, SIX_SQUADS_ADVANCE_METERS));
    float middle_index = (SIX_SQUADS_COUNT - 1) / 2.0f;
    for (int squad_index = 0; squad_index < SIX_SQUADS_COUNT; squad_index++) {
        float offset = (middle_index - squad_index) * SIX_SQUADS_GAP_METERS;
        V2 start = add(center, mul(right, offset));
        V2 end = sub(end_center, mul(forward, offset));
        add_default_squad(six_squads_types[squad_index], start, forward);
        set_order(squad_index, end);
        order_facings[squad_index] = right;
    }
}


// `set_up_single_units` sets up `singleunits` on FZ7: a medium-tank squad drives north over the
// plateau, and squads of one command APC each meet it head-on, cross its route near its start and
// follow it. A fourth APC drives alone south of the cliffs that close the plateau.
//
//   north up                F    M   F = follower APC end; M = medium end, head-on APC start
//                               /
//   crossing APC end  <--------/--------<  crossing APC start
//                             M   medium start, head-on APC end
//                             F   follower APC start
//   ------------- cliffs that close the plateau -------------
//      lone APC end  <------------------  lone APC start, facing east
//
static void set_up_single_units(void) {
    V2 medium_start = v2(3030, 2950);
    V2 medium_end = v2(3100, 3480);
    V2 medium_direction = get_direction_between(medium_start, medium_end);
    add_default_squad(&MEDIUM_TANK, medium_start, medium_direction);
    set_order(0, medium_end);

    // The follower starts 50 m behind the medium squad and ends 40 m to the left of its end, as
    // seen facing north. The crossing APC reaches the medium route 150 m after its start, at
    // about the time when the medium squad passes there.
    V2 opposite_direction = mul(medium_direction, -1);
    V2 cross_start = v2(3210, 3100);
    V2 cross_end = v2(2960, 3100);
    V2 follow_start = v2(3024, 2900);
    V2 follow_end = v2(3060, 3485);
    add_default_squad(&COMMAND_APC, medium_end, opposite_direction);
    add_default_squad(&COMMAND_APC, cross_start, get_direction_between(cross_start, cross_end));
    add_default_squad(&COMMAND_APC, follow_start, medium_direction);
    set_order(1, medium_start);
    set_order(2, cross_end);
    set_order(3, follow_end);

    // The lone APC faces away from its destination, so it starts with a turn around.
    V2 lone_start = v2(3300, 2715);
    V2 lone_end = v2(2850, 2715);
    add_default_squad(&COMMAND_APC, lone_start, EAST);
    set_order(4, lone_end);
}


// `set_up_reorder` sets up `reorder` on FZ1: three squads drive east in separate parts of the open
// field and get new orders `REORDER_SECONDS` after the start, while they drive at about their top
// speeds. The second destinations lie away from the other squads, so the squads never meet.
//
//   medium end (2)              light end (2)
//         ^                           ^
//          \                          |
//           medium ->                 light ->
//   heavy end (2) <---  heavy ->
//
//   (2) marks the destination of the second order.
//
static void set_up_reorder(void) {
    add_default_squad(&LIGHT_TANK, v2(3000, 2650), EAST);
    add_default_squad(&MEDIUM_TANK, v2(2650, 2750), EAST);
    add_default_squad(&HEAVY_TANK, v2(2650, 2500), EAST);
    set_order(0, v2(3400, 2650));
    set_order(1, v2(3300, 2750));
    set_order(2, v2(3300, 2500));
    set_second_order(0, REORDER_SECONDS, v2(3110, 3000));
    set_second_order(1, REORDER_SECONDS, v2(2530, 2950));
    set_second_order(2, REORDER_SECONDS, v2(2560, 2500));
}


// `set_up_reverse` sets up `reverse` on FZ1: four squads face east and get backward move orders
// (`--reverse`). The medium squad backs straight west, the heavy squad backs to a point behind it
// and to its right, the light squad turns around on the spot and backs to a point east of it, and
// the marines turn and walk west, as infantry never drives backward.
//
//                            light  ----------->  light end
//   marine end <-----------  marines
//   medium end <-----------  medium   (all facing east)
//                            heavy
//          heavy end  <-'
//
static void set_up_reverse(void) {
    add_default_squad(&MEDIUM_TANK, v2(3100, 2700), EAST);
    add_default_squad(&HEAVY_TANK, v2(3100, 2560), EAST);
    add_default_squad(&MARINE, v2(3100, 2830), EAST);
    add_default_squad(&LIGHT_TANK, v2(3100, 2960), EAST);
    set_order(0, v2(2900, 2700));
    set_order(1, v2(2960, 2440));
    set_order(2, v2(2950, 2830));
    set_order(3, v2(3300, 2960));
    for (int squad_index = 0; squad_index < squad_count; squad_index++)
        squads[squad_index].is_reverse_order = true;
}


// `set_up_close_quarters` sets up `closequarters` on FZ1: two marine squads walk at each other
// along the east-west line, and a medium and a light tank squad drive at each other along the
// north-south line. The start distances match the top speeds, so all four reach the meeting point
// at about the same time. At `CLOSE_RETURN_SECONDS`, each squad turns back to its start.
//
//   north up                light start
//                               |
//   marine start  ---->    meeting point   <----  marine start (6 m further north)
//                               |
//                          medium start
//
static void set_up_close_quarters(void) {
    // Each offset from the meeting point is the distance that a squad covers in about 7.5 s.
    V2 meeting_point = v2(3100, 2700);
    V2 west_marines = add(meeting_point, v2(-60, -3));
    V2 east_marines = add(meeting_point, v2(60, 3));
    V2 south_medium = add(meeting_point, v2(0, -150));
    V2 north_light = add(meeting_point, v2(4, 200));
    V2 starts[] = {west_marines, east_marines, south_medium, north_light};
    V2 ends[] = {
        add(meeting_point, v2(60, -3)),
        add(meeting_point, v2(-60, 3)),
        add(meeting_point, v2(0, 150)),
        add(meeting_point, v2(4, -200)),
    };
    add_default_squad(&MARINE, starts[0], EAST);
    add_default_squad(&MARINE, starts[1], mul(EAST, -1));
    add_default_squad(&MEDIUM_TANK, starts[2], NORTH);
    add_default_squad(&LIGHT_TANK, starts[3], mul(NORTH, -1));
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        set_order(squad_index, ends[squad_index]);
        set_second_order(squad_index, CLOSE_RETURN_SECONDS, starts[squad_index]);
    }
}


// `set_boarding` gives each member of the squad `squad_index` the order to board the first member
// of the squad `carrier_squad_index`, an APC.
static void set_boarding(int squad_index, int carrier_squad_index) {
    int carrier_index = squads[carrier_squad_index].first_unit_index;
    const Squad *squad = &squads[squad_index];
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++)
        units[unit_index].target_unit_index = carrier_index;
}


// `set_up_boarding` sets up `boarding` on FZ1: two marine squads stand next to each other, facing
// east, and walk to board a command APC west of them. `BOARDING_APC_ORDER_SECONDS` later, the
// APC drives to a point east of them, so its straight way leads through them.
//
//   north up                     marines
//   APC start  ---------------->        ----------------------->  APC destination
//                                marines
//
static void set_up_boarding(void) {
    // The APC is added last, so the replay draws it over the marines that ride inside it.
    V2 marine_center = v2(3100, 2700);
    V2 north_offset = mul(NORTH, BOARDING_SQUAD_GAP_METERS / 2);
    V2 apc_start = sub(marine_center, mul(EAST, BOARDING_APC_BEHIND_METERS));
    V2 apc_end = add(marine_center, mul(EAST, BOARDING_APC_PAST_METERS));
    add_default_squad(&MARINE, add(marine_center, north_offset), EAST);
    add_default_squad(&MARINE, sub(marine_center, north_offset), EAST);
    add_default_squad(&COMMAND_APC, apc_start, EAST);
    set_order(2, apc_end);
    order_seconds[2] = BOARDING_APC_ORDER_SECONDS;
    set_boarding(0, 2);
    set_boarding(1, 2);
}


// `set_up_marine_orders` sets up `marineorders` on FZ1: two marine squads stand next to each
// other, facing east, with a parked light-tank squad north-east of them. Every
// `MARINE_ORDERS_PERIOD_SECONDS`, both marine squads get a new order in another direction, the
// north squad to the north of the order point and the south squad to its south.
//
//   north up                   2   5
//                 3
//                          tanks
//                   marines -------> 1
//                         4
//
// The numbers mark the order points in their order. Each order leads in another direction: east,
// north-west, west, south-east, then north-east.
//
static void set_up_marine_orders(void) {
    // The order points lead past the tanks, at `tank_center`, on several sides.
    static const V2 ORDER_OFFSETS[MARINE_ORDERS_COUNT] = {
        {.x = 70, .y = 0},
        {.x = 40, .y = 45},
        {.x = -10, .y = 30},
        {.x = 20, .y = -15},
        {.x = 60, .y = 40},
    };
    V2 marine_center = v2(3100, 2700);
    V2 tank_center = add(marine_center, v2(30, 20));
    V2 north_offset = mul(NORTH, MARINE_ORDERS_SQUAD_GAP_METERS / 2);
    add_default_squad(&MARINE, add(marine_center, north_offset), EAST);
    add_default_squad(&MARINE, sub(marine_center, north_offset), EAST);
    add_default_squad(&LIGHT_TANK, tank_center, EAST);

    // The first orders come in the first tick, the others as timed orders.
    for (int order_index = 0; order_index < MARINE_ORDERS_COUNT; order_index++) {
        V2 order_point = add(marine_center, ORDER_OFFSETS[order_index]);
        V2 north_point = add(order_point, north_offset);
        V2 south_point = sub(order_point, north_offset);
        float seconds = order_index * MARINE_ORDERS_PERIOD_SECONDS;
        if (order_index == 0) {
            set_order(0, north_point);
            set_order(1, south_point);
            continue;
        }
        add_timed_order(0, seconds, north_point);
        add_timed_order(1, seconds, south_point);
    }
}


// `get_stress_place` returns the place `place_index` of a `stress` grid around `grid_center`. The
// places run east along a row, and the rows follow each other to the north.
static V2 get_stress_place(V2 grid_center, int place_index) {
    float middle_column = (STRESS_COLUMN_COUNT - 1) / 2.0f;
    float middle_row = (STRESS_ROW_COUNT - 1) / 2.0f;
    int column_index = place_index % STRESS_COLUMN_COUNT;
    int row_index = place_index / STRESS_COLUMN_COUNT;
    float east_offset = (column_index - middle_column) * STRESS_GAP_METERS;
    float north_offset = (row_index - middle_row) * STRESS_GAP_METERS;
    return add(grid_center, v2(east_offset, north_offset));
}


// `set_up_stress` sets up `stress` on FZ1, a stress test of the route planning and of the
// coordination: 32 squads stand in a grid, facing east, and get orders one after another in three
// waves, while the squads of the earlier waves still drive.
//
//   north up    start grid               east grid, 300 m east
//               24 25 ... 31             24 25 ... 31
//               ...            ----->    ...
//                0  1 ...  7              0  1 ...  7
//
// Wave 1 sends squad `i` to place `i` of the east grid, wave 2 sends it to place `31 - i` of that
// grid, across the other squads, and wave 3 sends it back to place `i` of the start grid. In each
// wave, squad `i` gets its order `i * STRESS_ORDER_GAP_SECONDS` after the start of the wave.
//
static void set_up_stress(void) {
    static const UnitType *const stress_types[STRESS_TYPE_COUNT] = {
        &LIGHT_TANK, &MEDIUM_TANK, &HEAVY_TANK, &MARINE,
    };
    V2 start_center = v2(2900, 2700);
    V2 east_center = add(start_center, mul(EAST, STRESS_ADVANCE_METERS));
    int stress_squad_count = STRESS_COLUMN_COUNT * STRESS_ROW_COUNT;
    for (int squad_index = 0; squad_index < stress_squad_count; squad_index++) {
        const UnitType *unit_type = stress_types[squad_index % STRESS_TYPE_COUNT];
        add_default_squad(unit_type, get_stress_place(start_center, squad_index), EAST);
    }

    // Wave 1 uses the orders of the squads, which mark them as ordered in the result rows; the
    // later waves are timed orders.
    int last_index = stress_squad_count - 1;
    for (int squad_index = 0; squad_index < stress_squad_count; squad_index++) {
        float delay_seconds = squad_index * STRESS_ORDER_GAP_SECONDS;
        set_order(squad_index, get_stress_place(east_center, squad_index));
        order_seconds[squad_index] = delay_seconds;
        V2 mirrored_place = get_stress_place(east_center, last_index - squad_index);
        add_timed_order(squad_index, STRESS_WAVE_SECONDS + delay_seconds, mirrored_place);
        V2 start_place = get_stress_place(start_center, squad_index);
        add_timed_order(squad_index, 2 * STRESS_WAVE_SECONDS + delay_seconds, start_place);
    }
}


// `set_up_scenario` sets up the scenario `scenario_index` with the jitter of `seed`: it loads the
// map, adds the squads and gives them their orders. The jitter depends on both the seed and the
// scenario, so each scenario gets its own random numbers.
void set_up_scenario(int scenario_index, unsigned seed) {
    unsigned scenario_key = SCENARIO_JITTER_KEY[scenario_index];
    unsigned scenario_hash = scenario_key * SCENARIO_HASH_MULTIPLIER + 1;
    jitter_state = 0;
    if (seed)
        jitter_state = (seed * SEED_HASH_MULTIPLIER) ^ scenario_hash;

    unit_count = 0;
    squad_count = 0;
    memset(has_order, 0, sizeof has_order);
    memset(order_seconds, 0, sizeof order_seconds);
    memset(has_second_order, 0, sizeof has_second_order);
    timed_order_count = 0;
    memset(order_facings, 0, sizeof order_facings);
    load_map(SCENARIO_MAP[scenario_index]);

    switch (scenario_index) {
    case SCENARIO_CROSS:
        set_up_cross();
        break;
    case SCENARIO_HEADON:
        set_up_plateau_head_on(&LIGHT_TANK, &MEDIUM_TANK);
        break;
    case SCENARIO_PARKED:
        set_up_parked();
        break;
    case SCENARIO_HEADON_VALLEYS:
        set_up_head_on_valleys();
        break;
    case SCENARIO_RIDGE:
        set_up_ridge();
        break;
    case SCENARIO_CORRIDOR:
        set_up_corridor();
        break;
    case SCENARIO_HEAVIES:
        set_up_heavies();
        break;
    case SCENARIO_WINDING:
        set_up_winding();
        break;
    case SCENARIO_SIX_SQUADS:
        set_up_six_squads();
        break;
    case SCENARIO_HEADON_LIGHT:
        set_up_plateau_head_on(&LIGHT_TANK, &LIGHT_TANK);
        break;
    case SCENARIO_HEADON_HEAVY:
        set_up_plateau_head_on(&HEAVY_TANK, &HEAVY_TANK);
        break;
    case SCENARIO_SINGLE_UNITS:
        set_up_single_units();
        break;
    case SCENARIO_REORDER:
        set_up_reorder();
        break;
    case SCENARIO_CLOSE_QUARTERS:
        set_up_close_quarters();
        break;
    case SCENARIO_BOARDING:
        set_up_boarding();
        break;
    case SCENARIO_MARINE_ORDERS:
        set_up_marine_orders();
        break;
    case SCENARIO_PARKED_END:
        set_up_parked_end();
        break;
    case SCENARIO_STRESS:
        set_up_stress();
        break;
    case SCENARIO_REVERSE:
        set_up_reverse();
        break;
    }
}


// `is_squad_boosted` tells whether the squad `squad_index` has the speed booster in this tick. Its
// boost starts `boost_stagger_seconds` per squad index after `boost_start_seconds`, and it lasts
// `BOOSTER_SECONDS`, or with a cycle, it is on and off for `boost_cycle_seconds` by turns.
static bool is_squad_boosted(int squad_index) {
    if (boost_start_seconds == NO_BOOST)
        return false;
    float squad_start_seconds = boost_start_seconds + squad_index * boost_stagger_seconds;
    float since_start = now_seconds - squad_start_seconds;        // time since the boost start
    if (since_start < 0)
        return false;
    if (boost_cycle_seconds == NO_BOOST_CYCLE)
        return since_start < BOOSTER_SECONDS;
    return fmodf(since_start, 2 * boost_cycle_seconds) < boost_cycle_seconds;
}


// `update_speed_effects` sets the top speed and the acceleration of every unit for the current
// tick, as 0x4fb940 of the game resets them to their bases and applies the active effects. With
// `--boost`, the units have the speed booster, as `is_squad_boosted` tells for their squads. A
// unit that drives backward then gets its lower top speed, as in the mod (`--reverse`).
void update_speed_effects(void) {
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        Unit *unit = &units[unit_index];
        unit->top_speed = unit->base_top_speed;
        unit->acceleration = unit->unit_type->acceleration;
        if (is_squad_boosted(unit->squad_index)) {
            unit->top_speed *= BOOSTER_FACTOR;
            unit->acceleration *= BOOSTER_FACTOR;
        }
        limit_reverse_speed(unit);
    }
}
