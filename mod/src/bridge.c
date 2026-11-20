// Bridge between the game and the shared formation code (src/formation). The formation code
// works on the arrays `units` and `squads` of sim.h, where the members of a squad are contiguous.
// At the start of each game frame, `begin_frame` lays out these arrays from the squads that the
// game updated in the last frame, copies the state of each game unit into them, sets up the
// formation of squads that got a new route, and moves the anchors. Then, for each member of a
// squad in formation following, `tick_formation_mover` steers with the formation code and lets
// the game turn, slow down and move the unit, as its own *Tick* (0x50e5b0) would.
//
// A squad keeps its index in `squads` for as long as the game updates it, so its formation
// state stays in place; the units of all squads are laid out again whenever a squad or its
// members change. Offsets of game objects come from docs/PATHFINDING.md.
#include <string.h>
#include <windows.h>

#include "mod.h"
#include "sim.h"
#include "formation.h"

#define THISCALL __attribute__((thiscall))

// Entity (a unit of the game). The map plane is (x, z) of the game, which is (x, y) here.
#define ENTITY_X 0x0c                       // float
#define ENTITY_Y 0x14                       // float, z of the game
#define ENTITY_ANGLE 0x1c                   // float, heading angle
#define ENTITY_TYPE 0x54                    // pointer to the entity type
#define ENTITY_MOVER 0x58                   // pointer to the mover, or NULL
#define ENTITY_ACTIVE 0x88                  // byte; Update and Steer skip an inactive unit
// Pointer to the APC that carries the unit, or NULL. *Tick* sets it when a unit that drives to
// an APC comes within twice the APC radius (0x50f634), so the unit boards it.
#define ENTITY_CARRIER 0xac

// Entity type, from the ENT file of the unit (`EntityData::LoadMe` 0x4212c0).
#define TYPE_FORCE 0x70                     // byte: CRAYVEN 0, WRATH 1, PHOENIX 2
#define TYPE_RADIUS 0x88                    // float
#define TYPE_ACCELERATION 0x94              // float
#define TYPE_FRICTION 0x98                  // float
#define TYPE_TOP_SPEED 0x9c                 // float
#define TYPE_TURN_RATE 0xa4                 // float, radians per second
#define TYPE_TERRAIN_CLASS 0xb0             // byte
#define TYPE_CLOSE_RANGE 0xc0               // float, TurnOffset
#define AIRCRAFT_TERRAIN_CLASS 4            // aircraft fly over the grid

// Mover (the movement state of a unit).
#define MOVER_VELOCITY_X 0x08               // float
#define MOVER_VELOCITY_Y 0x0c               // float
#define MOVER_HEADING_X 0x18                // float, sin of the angle
#define MOVER_HEADING_Y 0x1c                // float, cos of the angle
#define MOVER_TOP_SPEED 0x40                // float, with the speed effects of this frame
#define MOVER_BASE_TOP_SPEED 0x44           // float, top speed times the profile modifier
#define MOVER_ACCELERATION 0x48             // float, with the speed effects of this frame
#define MOVER_TARGET 0x50                   // pointer to the chased entity, or NULL
#define MOVER_STOPPED 0x54                  // byte, set on the last waypoint
#define MOVER_FACING_X 0x60                 // float, final facing
#define MOVER_FACING_Y 0x64                 // float
#define MOVER_ENTITY 0x74                   // pointer to the entity
#define MOVER_AVOID_COUNTER 0x98            // int, ticks of the current avoidance
#define MOVER_FRICTION_SLOT 1               // vtable slots of *Friction*, *Integrate* and *Turn*
#define MOVER_INTEGRATE_SLOT 2
#define MOVER_TURN_SLOT 3

// Squad. The member vector holds entity pointers from `begin` to `end`.
#define SQUAD_MEMBER_VECTOR 0x14            // pointer to the member vector
#define VECTOR_BEGIN 0x00
#define VECTOR_END 0x0c
#define SQUAD_PARKING_LIST 0x28             // pointer to the list of final parking positions
#define SQUAD_STATE 0x30                    // int: 1 moving, 13 searching its route
#define SQUAD_FORMATION_TYPE 0x91           // byte
#define SQUAD_FACING_X 0x94                 // float, normalized facing of the order
#define SQUAD_FACING_Y 0x98                 // float
#define GAME_SQUAD_MOVING 1
#define GAME_SQUAD_SEARCHING 13

// List of the game: the head pointer, then nodes of x, y, next, previous.
#define LIST_HEAD 0x00
#define NODE_X 0x00
#define NODE_Y 0x04
#define NODE_NEXT 0x08

// Frame time and clock of the game (*Timing::CalculateTime* 0x428550 and 0x428670).
#define GAME_FRAME_SECONDS_ADDRESS 0x82f5cc         // float, time step of the movement
#define GAME_TIMING_ADDRESS 0x6a8094                // pointer to the timing object
#define TIMING_CLOCK 0x198                          // double, game clock in seconds
#define APPLY_PENDING_VELOCITY_ADDRESS 0x50ee80     // thiscall: pending velocity replaces velocity
// Vtable of `AiSquad`, the squads of the computer: its third entry is `AiSquad::Update` 0x502ae0,
// next to the vtable of `Squad` at 0x613330, whose third entry is `Squad::Update` 0x504950.
#define AI_SQUAD_VTABLE 0x613304
// `Unit.team` holds the side of a unit: whether a player or the computer orders its squad. The
// int16 at entity +0x3e is no side: it differs between the squads of one player.
#define PLAYER_SIDE 0
#define COMPUTER_SIDE 1
#define REROUTE_SEARCH_BUDGET 500           // expansions of each re-route search, as `--fullsearch`
#define REROUTE_SEARCH_TOLERANCE_METERS 0.0f        // the re-route search ends on the destination
#define REPEATED_ORDER_METERS 1.0f          // closer destinations repeat the order of a squad
// thiscall (mover, point): waypoint = point (x, height, z), no target, no waypoint list; the stop
// of `Squad::Stop` 0x504bd0 for each member.
#define STOP_MOVER_ADDRESS 0x50e690

#define MAX_UNIT_TYPES 128                  // most entity types that the bridge caches
#define KEY_DOWN_BIT 0x8000                 // bit of `GetAsyncKeyState` while a key is down
#define MISSION_UPDATE_MILLISECONDS 1000    // squad updates this recent mean a mission runs

typedef void(THISCALL *MoverFunction)(void *mover);
typedef void(THISCALL *TurnFunction)(void *mover, float steering);
typedef void(THISCALL *StopMoverFunction)(void *mover, const void *point);

// A squad of the game and its place in `squads`.
typedef struct {
    void *squad_object;          // game squad, or NULL for a free slot
    bool was_updated;            // the game updated it since the last frame start
    bool is_forming;             // in formation following
    bool has_new_route;          // `note_route` stored a route for a set-up
    bool has_logged_repeat;      // the log shows that the current order repeats
} SquadSlot;

Unit units[MAX_UNITS];
Squad squads[MAX_SQUADS];
int unit_count;
int squad_count;
float now_seconds;
float path_points[2 * MAX_WAYPOINTS];
int avoid_mode = AVOID_COORD;
float tick_seconds;

static SquadSlot squad_slots[MAX_SQUADS];
static void *unit_entities[MAX_UNITS];        // entity of each unit index
static Unit laid_out_units[MAX_UNITS];        // units in their new order during a layout
static void *laid_out_entities[MAX_UNITS];
static UnitType unit_types[MAX_UNIT_TYPES];
static const void *unit_type_objects[MAX_UNIT_TYPES];
static int unit_type_count;
static double last_clock_seconds = -1;
static bool is_mod_on = true;
static bool was_toggle_key_down;
static int toggle_key;
static DWORD last_squad_update_ticks;         // `GetTickCount` at the last squad update


// `read_float` reads the float at `offset` in the game object `object`.
static float read_float(const void *object, int offset) {
    float value;
    memcpy(&value, &((const uint8_t *)object)[offset], sizeof value);
    return value;
}


// `write_float` writes the float at `offset` in the game object `object`.
static void write_float(void *object, int offset, float value) {
    memcpy(&((uint8_t *)object)[offset], &value, sizeof value);
}


// `read_pointer` reads the pointer at `offset` in the game object `object`.
static void *read_pointer(const void *object, int offset) {
    void *value;
    memcpy(&value, &((const uint8_t *)object)[offset], sizeof value);
    return value;
}


// `read_byte` reads the byte at `offset` in the game object `object`.
static uint8_t read_byte(const void *object, int offset) {
    return ((const uint8_t *)object)[offset];
}


// `write_byte` writes the byte at `offset` in the game object `object`.
static void write_byte(void *object, int offset, uint8_t value) {
    ((uint8_t *)object)[offset] = value;
}


// `read_int` reads the 32-bit integer at `offset` in the game object `object`.
static int read_int(const void *object, int offset) {
    int value;
    memcpy(&value, &((const uint8_t *)object)[offset], sizeof value);
    return value;
}


// `get_vtable_function` returns the function in slot `slot_index` of the vtable of `object`.
static void *get_vtable_function(const void *object, int slot_index) {
    void **vtable = read_pointer(object, 0);
    return vtable[slot_index];
}


// `start_bridge` allocates the lane table of the formation code, as the simulator does once at its
// start, and stores the key that `update_toggle` watches.
void start_bridge(int virtual_key) {
    allocate_lanes();
    toggle_key = virtual_key;
}


// `find_unit_type` returns the unit type for the entity type `type_object`, which it copies from
// the game on its first use. The types of a mission fit into `MAX_UNIT_TYPES`; past that, a new
// type reuses the last entry.
static const UnitType *find_unit_type(const void *type_object) {
    for (int type_index = 0; type_index < unit_type_count; type_index++) {
        if (unit_type_objects[type_index] == type_object)
            return &unit_types[type_index];
    }
    if (unit_type_count < MAX_UNIT_TYPES)
        unit_type_count++;
    int type_index = unit_type_count - 1;
    int force = read_byte(type_object, TYPE_FORCE);
    unit_type_objects[type_index] = type_object;
    unit_types[type_index] = (UnitType){
        .name = "game unit",
        .radius = read_float(type_object, TYPE_RADIUS),
        .top_speed = read_float(type_object, TYPE_TOP_SPEED),
        .acceleration = read_float(type_object, TYPE_ACCELERATION),
        .friction = read_float(type_object, TYPE_FRICTION),
        .turn_rate = read_float(type_object, TYPE_TURN_RATE),
        .close_range = read_float(type_object, TYPE_CLOSE_RANGE),
        .terrain_class = read_byte(type_object, TYPE_TERRAIN_CLASS),
        .units_per_squad = 0,
        .faction = force < FACTION_COUNT ? force : FACTION_CRAYVEN,
    };
    return &unit_types[type_index];
}


// `is_ground_entity` tells whether a squad member moves on the ground: it has a mover, is not an
// aircraft and is not inside an APC. Aircraft keep the movement of the game and are no obstacle
// on the ground. A unit inside an APC would be an obstacle that the APC can never leave behind,
// and the ORCA avoidance would push the APC away from it in a fixed direction.
static bool is_ground_entity(const void *entity) {
    if (!read_pointer(entity, ENTITY_MOVER) || read_pointer(entity, ENTITY_CARRIER))
        return false;
    const void *type_object = read_pointer(entity, ENTITY_TYPE);
    return read_byte(type_object, TYPE_TERRAIN_CLASS) != AIRCRAFT_TERRAIN_CLASS;
}


// `find_squad_slot` returns the slot index of `squad_object`, or -1 when it has none.
static int find_squad_slot(const void *squad_object) {
    for (int slot_index = 0; slot_index < MAX_SQUADS; slot_index++) {
        if (squad_slots[slot_index].squad_object == squad_object)
            return slot_index;
    }
    return -1;
}


// `add_squad_slot` gives `squad_object` a free slot and a cleared squad there. It returns the
// slot index, or -1 when every slot is taken.
static int add_squad_slot(void *squad_object) {
    int slot_index = find_squad_slot(NULL);
    if (slot_index < 0)
        return -1;
    squad_slots[slot_index] = (SquadSlot){
        .squad_object = squad_object,
        .was_updated = false,
        .is_forming = false,
        .has_new_route = false,
        .has_logged_repeat = false,
    };
    Squad *squad = &squads[slot_index];
    memset(squad, 0, sizeof *squad);
    squad->squad_index = slot_index;
    squad->state = SQUAD_IDLE;
    squad->giving_way_to_index = NO_SQUAD;
    squad->passing_index = NO_SQUAD;
    return slot_index;
}


// `write_int` writes the 32-bit integer at `offset` in the game object `object`.
static void write_int(void *object, int offset, int value) {
    memcpy(&((uint8_t *)object)[offset], &value, sizeof value);
}


// `get_members` returns the member vector of a game squad and its length in `member_count`.
static void **get_members(const void *squad_object, int *member_count) {
    const void *vector = read_pointer(squad_object, SQUAD_MEMBER_VECTOR);
    void **begin = read_pointer(vector, VECTOR_BEGIN);
    void **end = read_pointer(vector, VECTOR_END);
    *member_count = (int)(end - begin);
    return begin;
}


// `find_unit_index` returns the unit index of `entity`, or -1 when it has none.
static int find_unit_index(const void *entity) {
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        if (unit_entities[unit_index] == entity)
            return unit_index;
    }
    return -1;
}


// `free_squad_slot` forgets the squad of a slot and ends its formation following.
static void free_squad_slot(int slot_index) {
    squad_slots[slot_index].squad_object = NULL;
    squad_slots[slot_index].is_forming = false;
    squad_slots[slot_index].has_new_route = false;
    squads[slot_index].state = SQUAD_IDLE;
    squads[slot_index].unit_count = 0;
}


// `free_stale_slots` frees the slots of squads that the game did not update in the last frame:
// the squad was destroyed, or the mission ended.
static void free_stale_slots(void) {
    for (int slot_index = 0; slot_index < MAX_SQUADS; slot_index++) {
        SquadSlot *slot = &squad_slots[slot_index];
        if (!slot->squad_object || slot->was_updated)
            continue;
        free_squad_slot(slot_index);
    }
}


// `list_laid_out_entities` fills `laid_out_entities` with the ground members of the squads in
// slot order, sets the unit range of each squad and `squad_count`, and returns the unit count.
static int list_laid_out_entities(void) {
    int laid_out_count = 0;
    squad_count = 0;
    for (int slot_index = 0; slot_index < MAX_SQUADS; slot_index++) {
        Squad *squad = &squads[slot_index];
        squad->first_unit_index = laid_out_count;
        squad->unit_count = 0;
        const void *squad_object = squad_slots[slot_index].squad_object;
        if (!squad_object)
            continue;
        squad_count = slot_index + 1;

        // Units past `MAX_UNITS` stay with the movement of the game.
        int member_count;
        void **members = get_members(squad_object, &member_count);
        for (int member_index = 0; member_index < member_count; member_index++) {
            void *entity = members[member_index];
            bool has_room = laid_out_count < MAX_UNITS && squad->unit_count < MAX_MEMBERS;
            if (!has_room || !is_ground_entity(entity))
                continue;
            laid_out_entities[laid_out_count] = entity;
            laid_out_count++;
            squad->unit_count++;
        }
    }
    return laid_out_count;
}


// `is_computer_squad` tells whether the computer orders a game squad. Computer squads are `AiSquad`
// objects, whose vtable is at `AI_SQUAD_VTABLE`; every other squad belongs to a player, so other
// human players in a multiplayer game would count as players too (TODO: confirm a side field).
static bool is_computer_squad(const void *squad_object) {
    return read_pointer(squad_object, 0) == (void *)AI_SQUAD_VTABLE;
}


// `read_unit_state` copies the state of the game unit `entity` that the formation code reads.
// The stop flag and the final facing stay with the formation code while the squad forms.
static void read_unit_state(Unit *unit, const void *entity) {
    const void *mover = read_pointer(entity, ENTITY_MOVER);
    unit->unit_type = find_unit_type(read_pointer(entity, ENTITY_TYPE));
    unit->position = v2(read_float(entity, ENTITY_X), read_float(entity, ENTITY_Y));
    unit->angle = read_float(entity, ENTITY_ANGLE);
    unit->heading = v2(read_float(mover, MOVER_HEADING_X), read_float(mover, MOVER_HEADING_Y));
    unit->velocity = v2(read_float(mover, MOVER_VELOCITY_X), read_float(mover, MOVER_VELOCITY_Y));
    unit->base_top_speed = read_float(mover, MOVER_BASE_TOP_SPEED);
    unit->top_speed = read_float(mover, MOVER_TOP_SPEED);
    unit->acceleration = read_float(mover, MOVER_ACCELERATION);
}


// `start_unit` clears a unit that the bridge sees for the first time and sets it up as a unit
// standing after a stop of the game, as the simulator does for a new unit (scenarios.c).
static void start_unit(Unit *unit, const void *entity) {
    memset(unit, 0, sizeof *unit);
    read_unit_state(unit, entity);
    unit->start_position = unit->position;
    unit->parking_position = unit->position;
    unit->previous_angle = unit->angle;
    unit->waypoint = unit->position;
    unit->reach = STOP_REACH_METERS;
    unit->reach_last = STOP_REACH_METERS;
    unit->reach_mid = MID_REACH_METERS;
    unit->is_stopped = true;
    unit->blocker_index = NO_BLOCKER;
    unit->target_unit_index = NO_TARGET_UNIT;
    unit->wait_seconds = WAIT_TIMER_AT_ORDER_SECONDS;
}


// `lay_out_units` puts the units of every squad together in `units`, in slot order. A unit that
// was already laid out keeps its state; a new unit starts cleared. The units move only when the
// list of entities changed.
static void lay_out_units(void) {
    int laid_out_count = list_laid_out_entities();
    size_t entity_bytes = (size_t)laid_out_count * sizeof(void *);
    bool is_same_count = laid_out_count == unit_count;
    if (is_same_count && memcmp(laid_out_entities, unit_entities, entity_bytes) == 0)
        return;

    for (int unit_index = 0; unit_index < laid_out_count; unit_index++) {
        int old_index = find_unit_index(laid_out_entities[unit_index]);
        Unit *unit = &laid_out_units[unit_index];
        if (old_index >= 0) {
            *unit = units[old_index];
            continue;
        }
        start_unit(unit, laid_out_entities[unit_index]);
    }
    memcpy(units, laid_out_units, (size_t)laid_out_count * sizeof(Unit));
    memcpy(unit_entities, laid_out_entities, entity_bytes);
    unit_count = laid_out_count;
}


// `stop_forming` hands a squad back to the movement of the game.
static void stop_forming(int slot_index, int game_state) {
    squad_slots[slot_index].is_forming = false;
    squads[slot_index].state = SQUAD_IDLE;
    write_log("squad %d: formation following ends (game state %d)", slot_index, game_state);
}


// `find_target_unit_index` returns the unit index of the entity that `mover` drives to, such as an
// APC that its unit boards or a unit that it attacks, or `NO_TARGET_UNIT`. Aircraft and units
// inside an APC have no unit index.
static int find_target_unit_index(const void *mover) {
    const void *target = read_pointer(mover, MOVER_TARGET);
    if (!target)
        return NO_TARGET_UNIT;
    int unit_index = find_unit_index(target);
    return unit_index >= 0 ? unit_index : NO_TARGET_UNIT;
}


// `read_game_state` copies the state of every unit and squad from the game, and ends formation
// following for squads that the game no longer moves: they arrived, stopped, attack, or lost
// every member. While the game searches a new route (state 13), the old formation goes on. In the
// coordination mode, the moving squads then re-route around each squad that has just parked, as
// in the simulator at an arrival (game/squad.c `update_squad`).
static void read_game_state(void) {
    bool is_newly_parked[MAX_SQUADS] = {false};
    for (int slot_index = 0; slot_index < squad_count; slot_index++) {
        SquadSlot *slot = &squad_slots[slot_index];
        Squad *squad = &squads[slot_index];
        if (!slot->squad_object)
            continue;
        int side = is_computer_squad(slot->squad_object) ? COMPUTER_SIDE : PLAYER_SIDE;
        int end_index = squad->first_unit_index + squad->unit_count;
        for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
            Unit *unit = &units[unit_index];
            const void *entity = unit_entities[unit_index];
            const void *mover = read_pointer(entity, ENTITY_MOVER);
            read_unit_state(unit, entity);
            unit->squad_index = slot_index;
            unit->team = side;
            unit->target_unit_index = find_target_unit_index(mover);
            if (slot->is_forming)
                continue;
            unit->is_stopped = read_byte(mover, MOVER_STOPPED) != 0;
            unit->facing = v2(read_float(mover, MOVER_FACING_X), read_float(mover, MOVER_FACING_Y));
        }

        // The squad takes the unit type of its first member, as the formation layout does.
        if (squad->unit_count > 0)
            squad->unit_type = units[squad->first_unit_index].unit_type;
        squad->formation_type = read_byte(slot->squad_object, SQUAD_FORMATION_TYPE);
        int game_state = read_int(slot->squad_object, SQUAD_STATE);
        bool is_moved_by_game = game_state == GAME_SQUAD_MOVING;
        is_moved_by_game = is_moved_by_game || game_state == GAME_SQUAD_SEARCHING;
        bool has_members = squad->unit_count > 0;
        if (slot->is_forming && !(is_mod_on && is_moved_by_game && has_members)) {
            stop_forming(slot_index, game_state);
            is_newly_parked[slot_index] = has_members;
        }
    }

    // The re-routes come after the loop, when every unit has its position of this frame.
    if (!is_mod_on)
        return;
    for (int slot_index = 0; slot_index < squad_count; slot_index++) {
        if (is_newly_parked[slot_index])
            reroute_around_parked(&squads[slot_index]);
    }
}


// `read_parking_positions` copies up to `max_count` final parking positions of a game squad
// into `parking_positions`, as x, y pairs in layout order, and returns their number.
static int read_parking_positions(const void *squad_object, float *parking_positions,
                                  int max_count) {
    const void *list = read_pointer(squad_object, SQUAD_PARKING_LIST);
    if (!list)
        return 0;
    int count = 0;
    const void *node = read_pointer(list, LIST_HEAD);
    while (node && count < max_count) {
        parking_positions[2 * count] = read_float(node, NODE_X);
        parking_positions[2 * count + 1] = read_float(node, NODE_Y);
        count++;
        node = read_pointer(node, NODE_NEXT);
    }
    return count;
}


// `is_any_member_chasing` tells whether a member of a squad chases a target entity, as in an
// attack order. Such moves keep the movement of the game.
static bool is_any_member_chasing(const Squad *squad) {
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const void *mover = read_pointer(unit_entities[unit_index], ENTITY_MOVER);
        if (read_pointer(mover, MOVER_TARGET))
            return true;
    }
    return false;
}


// `find_front_unit_index` returns the unit index of the squad member closest to `destination`, the
// front unit from which the game searches the route of a squad.
static int find_front_unit_index(const Squad *squad, V2 destination) {
    int front_index = squad->first_unit_index;
    float best_squared = length_squared(sub(units[front_index].position, destination));
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index + 1; unit_index < end_index; unit_index++) {
        float distance_squared = length_squared(sub(units[unit_index].position, destination));
        if (distance_squared < best_squared) {
            best_squared = distance_squared;
            front_index = unit_index;
        }
    }
    return front_index;
}


// Routes that `note_route` stored, by slot: the route points after the start, and their count.
static V2 new_routes[MAX_SQUADS][MAX_WAYPOINTS];
static int new_route_counts[MAX_SQUADS];


// `get_order_source` names who gave the orders of a game squad.
static const char *get_order_source(const void *squad_object) {
    return is_computer_squad(squad_object) ? "computer" : "player";
}


// `find_game_movement_reason` returns why a squad with a new route keeps the movement of the
// game, or NULL when formation following can start.
static const char *find_game_movement_reason(int slot_index, int parking_count) {
    const Squad *squad = &squads[slot_index];
    int game_state = read_int(squad_slots[slot_index].squad_object, SQUAD_STATE);
    if (!is_mod_on)
        return "mod off";
    if (game_state != GAME_SQUAD_MOVING)
        return "squad not moving";
    if (squad->unit_count == 0)
        return "no ground members";
    if (is_any_member_chasing(squad))
        return "target entity (attack or APC boarding)";
    if (parking_count != squad->unit_count || new_route_counts[slot_index] == 0)
        return "final parking positions do not match the members";
    return NULL;
}


// `set_up_squad_route` gives a squad the route from its front unit through the `point_count`
// points of `route_points`, the last one its destination, moves its final parking positions off
// parked squads (`--wholeparked`, `--freeparking`), and runs the formation set-up on the route
// with the parked squads stamped as cliffs.
static void set_up_squad_route(Squad *squad, const V2 *route_points, int point_count,
                               float *parking_positions) {
    V2 destination = route_points[point_count - 1];
    int front_index = find_front_unit_index(squad, destination);
    squad->start_position = units[front_index].position;
    squad->destination = destination;
    squad->route[0] = squad->start_position;
    memcpy(&squad->route[1], route_points, (size_t)point_count * sizeof(V2));
    squad->route_count = point_count + 1;

    // The whole formation moves first, together with the route end; single positions that still
    // overlap a parked unit move after it.
    if (is_whole_parked_on)
        shift_parking_positions(squad, parking_positions);
    if (is_free_parking_on)
        free_parking_positions(squad, parking_positions);
    stamp_parked(squad, squad->start_position, squad->destination);
    set_up_formation(squad, parking_positions, squad->facing);
    unstamp_parked();
}


// `start_formation` sets up formation following for a squad with a new route, as the simulator
// does after the route search (game/squad.c `order_move` and `set_up_move`): the route of the game
// from the front unit, the final parking positions of the game layout, and the facing of the
// order. An order that keeps the movement of the game goes to the log with its source and the
// reason.
static void start_formation(int slot_index) {
    SquadSlot *slot = &squad_slots[slot_index];
    Squad *squad = &squads[slot_index];
    slot->has_new_route = false;
    float parking_positions[2 * MAX_MEMBERS];
    int parking_count = read_parking_positions(slot->squad_object, parking_positions,
                                               MAX_MEMBERS);
    const char *source = get_order_source(slot->squad_object);
    const char *game_movement_reason = find_game_movement_reason(slot_index, parking_count);
    if (game_movement_reason) {
        write_log("t %.1f order: squad %d (%s), game movement: %s", last_clock_seconds,
                  slot_index, source, game_movement_reason);
        return;
    }

    // A new order ends the re-routes of the last order.
    squad->is_rerouting = false;
    squad->is_settled = false;
    memset(squad->is_avoiding_parking, 0, sizeof squad->is_avoiding_parking);
    squad->facing = v2(read_float(slot->squad_object, SQUAD_FACING_X),
                       read_float(slot->squad_object, SQUAD_FACING_Y));
    set_up_squad_route(squad, new_routes[slot_index], new_route_counts[slot_index],
                       parking_positions);
    squad->state = SQUAD_MOVING;
    slot->is_forming = true;
}


// `start_route_search` re-routes a squad in formation following at once, for the re-routes
// around parked squads (game/reroute.c), which set `is_rerouting` first: a full base game path
// search from the front unit to the destination, with the parked squads stamped as cliffs, then a
// new set-up. The simulator spreads that search over ticks (game/squad.c). Without a path, the
// squad keeps its route, as with `--parkrobust`.
void start_route_search(Squad *squad) {
    int slot_index = squad->squad_index;
    float parking_positions[2 * MAX_MEMBERS];
    int parking_count = read_parking_positions(squad_slots[slot_index].squad_object,
                                               parking_positions, MAX_MEMBERS);
    if (parking_count != squad->unit_count)
        return;

    // The route of the game has room for the front unit before the path. `search_path` stamps the
    // parked squads itself: its stamp would forget a stamp made here, which would then stay in the
    // grid of the game as cliffs, and the game would move units off them.
    const Unit *front = &units[find_front_unit_index(squad, squad->destination)];
    GcRequest search;
    gc_init_request(&search);
    bool is_found = search_path(front, squad->destination, REROUTE_SEARCH_TOLERANCE_METERS,
                                REROUTE_SEARCH_BUDGET, &search);
    int point_count = 0;
    if (is_found)
        point_count = gc_get_path(&search, path_points, MAX_WAYPOINTS - 1);
    gc_free_request(&search);
    if (point_count == 0) {
        write_log("t %.1f re-route: squad %d found no path, keeps its route", last_clock_seconds,
                  slot_index);
        return;
    }

    V2 route_points[MAX_WAYPOINTS];
    for (int point_index = 0; point_index < point_count; point_index++)
        route_points[point_index] = get_pair_point(path_points, point_index);
    set_up_squad_route(squad, route_points, point_count, parking_positions);
}


// TEMPORARY DEBUG: `log_player_squads_debug` logs, once per second of the game clock, each player
// squad in formation following: its anchor, and for each member its distance to its slot, its
// speed and its detour points, to find members that get stuck or drive away.
static void log_player_squads_debug(void) {
    static double next_log_seconds;
    if (last_clock_seconds < next_log_seconds)
        return;
    next_log_seconds = last_clock_seconds + 1;
    for (int slot_index = 0; slot_index < squad_count; slot_index++) {
        // A free slot below `squad_count` has no squad object to read, but it is never forming.
        const SquadSlot *slot = &squad_slots[slot_index];
        const Squad *squad = &squads[slot_index];
        if (!slot->is_forming)
            continue;
        bool is_player = strcmp(get_order_source(slot->squad_object), "player") == 0;
        if (!is_player)
            continue;
        char line[480];
        int length = snprintf(line, sizeof line, "t %.1f dbg squad %d: anchor %.0f/%.0f v %.1f;",
                              last_clock_seconds, slot_index, squad->anchor_distance,
                              squad->route_length, squad->anchor_speed);
        int end_index = squad->first_unit_index + squad->unit_count;
        for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
            const Unit *unit = &units[unit_index];
            V2 forward;
            V2 slot_position = get_target_slot(squad, unit, &forward);
            float d_slot = len(sub(slot_position, unit->position));
            bool has_room = length < (int)sizeof line;
            if (has_room)
                length += snprintf(&line[length], sizeof line - (size_t)length,
                                   " [%.0f m %.0f m/s det %d/%d]", d_slot, len(unit->velocity),
                                   unit->detour_index, unit->detour_count);
        }
        write_log("%s", line);
    }
}


#define SLIP_DEBUG_SPEED 1.5f               // TEMPORARY DEBUG: least sideways speed in the log
#define SLIP_DEBUG_MAX_UNITS 6              // TEMPORARY DEBUG: most units in the log each second

// TEMPORARY DEBUG: largest sideways speed of each unit since the last slip log, after the steering
// of the formation code and after the movement steps of the game (*Friction*, *Integrate*).
static float steered_slip_speeds[MAX_UNITS];
static float moved_slip_speeds[MAX_UNITS];


// TEMPORARY DEBUG: `get_slip_speed` returns the speed of a unit across its heading.
static float get_slip_speed(const Unit *unit) {
    return fabsf(det2(unit->heading, unit->velocity));
}


// TEMPORARY DEBUG: `note_slip_debug` keeps the largest sideways speed of the unit `unit_index` in
// `slip_speeds`.
static void note_slip_debug(float *slip_speeds, int unit_index) {
    float slip_speed = get_slip_speed(&units[unit_index]);
    slip_speeds[unit_index] = fmaxf(slip_speeds[unit_index], slip_speed);
}


// TEMPORARY DEBUG: `log_slips_debug` logs, once per second of the game clock, up to
// `SLIP_DEBUG_MAX_UNITS` units of forming squads that moved sideways faster than
// `SLIP_DEBUG_SPEED`, with their unit type, to find where sliding comes from.
static void log_slips_debug(void) {
    static double next_log_seconds;
    if (last_clock_seconds < next_log_seconds)
        return;
    next_log_seconds = last_clock_seconds + 1;

    int logged_count = 0;
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        const Unit *unit = &units[unit_index];
        float steered_slip = steered_slip_speeds[unit_index];
        float moved_slip = moved_slip_speeds[unit_index];
        bool is_slipping = fmaxf(steered_slip, moved_slip) > SLIP_DEBUG_SPEED;
        if (is_slipping && logged_count < SLIP_DEBUG_MAX_UNITS) {
            logged_count++;
            const UnitType *type = unit->unit_type;
            write_log("t %.1f dbg slip: squad %d unit %d (stopped %d, orca %d) speed %.1f, "
                      "sideways %.1f now, max %.1f steered / %.1f moved; type top %.1f friction "
                      "%.1f acceleration %.1f turn %.2f; frame %.1f ms", last_clock_seconds,
                      unit->squad_index, unit_index, unit->is_stopped, unit->is_orca_avoiding,
                      len(unit->velocity), get_slip_speed(unit), steered_slip, moved_slip,
                      type->top_speed, type->friction, type->acceleration, type->turn_rate,
                      tick_seconds * 1000);
        }
        steered_slip_speeds[unit_index] = 0;
        moved_slip_speeds[unit_index] = 0;
    }
}


#define TARGET_DEBUG_MAX_LINES 8            // TEMPORARY DEBUG: most target lines each second


// TEMPORARY DEBUG: `find_nearest_unit_debug` returns the index of the unit nearest to the unit
// `unit_index`, or -1, and its center distance in `distance`.
static int find_nearest_unit_debug(int unit_index, float *distance) {
    int nearest_index = -1;
    float best_squared = 1e18f;
    for (int other_index = 0; other_index < unit_count; other_index++) {
        V2 offset = sub(units[other_index].position, units[unit_index].position);
        float d_squared = length_squared(offset);
        if (other_index != unit_index && d_squared < best_squared) {
            nearest_index = other_index;
            best_squared = d_squared;
        }
    }
    *distance = sqrtf(best_squared);
    return nearest_index;
}


// TEMPORARY DEBUG: `log_targets_debug` logs, once per second of the game clock, up to
// `TARGET_DEBUG_MAX_LINES` units whose game mover has a target: the target pointer, the unit that
// the bridge found for it, both teams and squad states, and the unit nearest to the target, to
// check that `--ignoreboarders` finds the units that board an APC.
static void log_targets_debug(void) {
    static double next_log_seconds;
    if (last_clock_seconds < next_log_seconds)
        return;
    next_log_seconds = last_clock_seconds + 1;

    int logged_count = 0;
    for (int unit_index = 0; unit_index < unit_count; unit_index++) {
        const void *mover = read_pointer(unit_entities[unit_index], ENTITY_MOVER);
        const void *target = read_pointer(mover, MOVER_TARGET);
        if (!target || logged_count >= TARGET_DEBUG_MAX_LINES)
            continue;
        logged_count++;
        const Unit *unit = &units[unit_index];
        int game_state = read_int(squad_slots[unit->squad_index].squad_object, SQUAD_STATE);
        int target_index = unit->target_unit_index;
        if (target_index < 0) {
            write_log("t %.1f dbg target: unit %d squad %d team %d state %d -> %p, no unit",
                      last_clock_seconds, unit_index, unit->squad_index, unit->team, game_state,
                      target);
            continue;
        }

        // The unit nearest to the target shows what the target could avoid.
        const Unit *target_unit = &units[target_index];
        float d_nearest = 0;                    // distance of the nearest unit to the target
        int nearest_index = find_nearest_unit_debug(target_index, &d_nearest);
        int nearest_target = nearest_index >= 0 ? units[nearest_index].target_unit_index : -1;
        write_log("t %.1f dbg target: unit %d squad %d team %d state %d -> unit %d squad %d "
                  "team %d forming %d orca %d boarding %d; nearest to it: unit %d at %.1f m, "
                  "its target %d", last_clock_seconds, unit_index, unit->squad_index, unit->team,
                  game_state, target_index, target_unit->squad_index, target_unit->team,
                  squad_slots[target_unit->squad_index].is_forming, target_unit->is_orca_avoiding,
                  is_boarding(unit, target_index), nearest_index, d_nearest, nearest_target);
    }
}


#define CONE_DEBUG_MAX_UNITS 8              // TEMPORARY DEBUG: most cone lines each second
#define DEGREES_PER_RADIAN (180 / PI)       // TEMPORARY DEBUG

// TEMPORARY DEBUG: the strongest push of a vehicle cone on one infantry unit since the last cone
// log, with the state of both units at that frame. A push of 0 means none.
typedef struct {
    float push_speed;
    double clock_seconds;
    int squad_index;
    V2 position;
    V2 heading;
    float speed;
    int vehicle_index;
    int vehicle_squad_index;
    const char *vehicle_type_name;
    bool is_vehicle_forming;
    bool is_vehicle_stopped;
    float vehicle_speed;
    V2 relative_position;
    float combined_radius;
    V2 push_direction;
} ConePushDebug;

static ConePushDebug cone_pushes_debug[MAX_UNITS];        // TEMPORARY DEBUG, by unit index

// TEMPORARY DEBUG: the strongest push of a vehicle line on each unit in its current steering.
static float frame_vehicle_pushes_debug[MAX_UNITS];


// TEMPORARY DEBUG: `note_cone_push_debug` keeps, for each infantry unit, the cone of a vehicle
// whose ORCA line moves the preferred velocity of the unit the most (`orca_line_debug_hook`).
static void note_cone_push_debug(int unit_index, int other_index, V2 relative_position,
                                 float combined_radius, float violation, V2 push_direction) {
    const Unit *unit = &units[unit_index];
    const Unit *other = &units[other_index];
    bool is_infantry = unit->unit_type->terrain_class == INFANTRY_TERRAIN_CLASS;
    bool is_vehicle = other->unit_type->terrain_class != INFANTRY_TERRAIN_CLASS;
    if (!is_infantry || !is_vehicle)
        return;
    float *frame_push = &frame_vehicle_pushes_debug[unit_index];
    *frame_push = fmaxf(*frame_push, violation);
    ConePushDebug *push = &cone_pushes_debug[unit_index];
    if (violation <= push->push_speed)
        return;
    *push = (ConePushDebug){
        .push_speed = violation,
        .clock_seconds = last_clock_seconds,
        .squad_index = unit->squad_index,
        .position = unit->position,
        .heading = unit->heading,
        .speed = len(unit->velocity),
        .vehicle_index = other_index,
        .vehicle_squad_index = other->squad_index,
        .vehicle_type_name = other->unit_type->name,
        .is_vehicle_forming = squad_slots[other->squad_index].is_forming,
        .is_vehicle_stopped = other->is_stopped,
        .vehicle_speed = len(other->velocity),
        .relative_position = relative_position,
        .combined_radius = combined_radius,
        .push_direction = push_direction,
    };
}


// TEMPORARY DEBUG: `get_angle_from_heading_debug` returns the angle of `direction` from `heading`
// in degrees, counterclockwise in the map plane (x, y).
static float get_angle_from_heading_debug(V2 heading, V2 direction) {
    float angle = atan2f(det2(heading, direction), dot(heading, direction));
    return angle * DEGREES_PER_RADIAN;
}


// TEMPORARY DEBUG: `log_cone_pushes_debug` logs, once per second of the game clock, up to
// `CONE_DEBUG_MAX_UNITS` infantry units that a vehicle cone pushed in that second: the strongest
// push, where the vehicle was, seen from the infantry unit, and the state of the vehicle.
static void log_cone_pushes_debug(void) {
    static double next_log_seconds;
    if (last_clock_seconds < next_log_seconds)
        return;
    next_log_seconds = last_clock_seconds + 1;

    // The unit indexes can change within the second, so each line uses the state noted with it.
    int logged_count = 0;
    for (int unit_index = 0; unit_index < MAX_UNITS; unit_index++) {
        ConePushDebug *push = &cone_pushes_debug[unit_index];
        if (push->push_speed <= 0)
            continue;
        if (logged_count < CONE_DEBUG_MAX_UNITS) {
            logged_count++;
            float distance = len(push->relative_position);
            float gap = distance - push->combined_radius;        // gap past the combined radius
            float vehicle_angle = get_angle_from_heading_debug(push->heading,
                                                               push->relative_position);
            float push_angle = get_angle_from_heading_debug(push->heading, push->push_direction);
            write_log("t %.1f dbg cone: infantry unit %d squad %d at (%.0f, %.0f) speed %.1f <- "
                      "%s unit %d squad %d (forming %d, stopped %d, speed %.1f) at %.1f m, %+.0f "
                      "deg from heading, gap %.1f m%s; push %.1f m/s at %+.0f deg from heading",
                      push->clock_seconds, unit_index, push->squad_index, push->position.x,
                      push->position.y, push->speed, push->vehicle_type_name, push->vehicle_index,
                      push->vehicle_squad_index, push->is_vehicle_forming,
                      push->is_vehicle_stopped, push->vehicle_speed, distance, vehicle_angle, gap,
                      gap <= 0 ? " (contact)" : "", push->push_speed, push_angle);
        }
        push->push_speed = 0;
    }
}


#define ENTRY_DEBUG_FRAMES 6                // TEMPORARY DEBUG: frames logged up to a body entry
#define ENTRY_DEBUG_RANGE_METERS 15.0f      // TEMPORARY DEBUG: farthest vehicle in a frame record
#define ENTRY_DEBUG_MAX_EVENTS 3            // TEMPORARY DEBUG: most entries logged each second
#define NO_VEHICLE_DEBUG (-1)               // TEMPORARY DEBUG: no vehicle within range

// TEMPORARY DEBUG: one frame of an infantry unit in formation following beside its nearest
// vehicle: at the start of its move, after the steering of the formation code, and after the game
// moved it (*Friction*, *Integrate*). The vehicle positions are read from the game at both times.
// A gap below 0 means that the two bodies overlap.
typedef struct {
    double clock_seconds;
    int vehicle_index;
    bool is_stopped;
    V2 start_position;
    V2 start_velocity;
    V2 steered_velocity;
    float vehicle_push_speed;
    V2 moved_position;
    V2 moved_velocity;
    V2 vehicle_start_position;
    V2 vehicle_moved_position;
    float start_gap;
    float moved_gap;
} EntryFrameDebug;

// TEMPORARY DEBUG: the last `ENTRY_DEBUG_FRAMES` frames of each unit, a ring by unit index, and
// the number of frames written for each unit. A new layout of the units can mix two units.
static EntryFrameDebug entry_frames_debug[MAX_UNITS][ENTRY_DEBUG_FRAMES];
static int entry_frame_counts_debug[MAX_UNITS];


// TEMPORARY DEBUG: `get_live_position_debug` returns the position of a unit as the game holds it
// now, which can be newer than `units` for a unit that the game moves itself.
static V2 get_live_position_debug(int unit_index) {
    const void *entity = unit_entities[unit_index];
    return v2(read_float(entity, ENTITY_X), read_float(entity, ENTITY_Y));
}


// TEMPORARY DEBUG: `find_nearest_vehicle_debug` returns the index of the vehicle whose center is
// nearest to `position`, within `ENTRY_DEBUG_RANGE_METERS`, or `NO_VEHICLE_DEBUG`.
static int find_nearest_vehicle_debug(V2 position) {
    int nearest_index = NO_VEHICLE_DEBUG;
    float best_squared = ENTRY_DEBUG_RANGE_METERS * ENTRY_DEBUG_RANGE_METERS;
    for (int other_index = 0; other_index < unit_count; other_index++) {
        if (units[other_index].unit_type->terrain_class == INFANTRY_TERRAIN_CLASS)
            continue;
        V2 offset = sub(get_live_position_debug(other_index), position);
        float d_squared = length_squared(offset);
        if (d_squared < best_squared) {
            nearest_index = other_index;
            best_squared = d_squared;
        }
    }
    return nearest_index;
}


// TEMPORARY DEBUG: `get_body_gap_debug` returns the distance between the bodies of two units at the
// given positions: the center distance minus both radii.
static float get_body_gap_debug(int unit_index, int other_index, V2 position, V2 other_position) {
    float r = units[unit_index].unit_type->radius + units[other_index].unit_type->radius;
    return len(sub(other_position, position)) - r;
}


// TEMPORARY DEBUG: `begin_entry_frame_debug` starts the frame record of an infantry unit at the
// start of its move, and returns it, or NULL for a unit of another kind.
static EntryFrameDebug *begin_entry_frame_debug(int unit_index) {
    const Unit *unit = &units[unit_index];
    if (unit->unit_type->terrain_class != INFANTRY_TERRAIN_CLASS)
        return NULL;
    int ring_index = entry_frame_counts_debug[unit_index] % ENTRY_DEBUG_FRAMES;
    EntryFrameDebug *frame = &entry_frames_debug[unit_index][ring_index];
    int vehicle_index = find_nearest_vehicle_debug(unit->position);
    *frame = (EntryFrameDebug){
        .clock_seconds = last_clock_seconds,
        .vehicle_index = vehicle_index,
        .is_stopped = unit->is_stopped,
        .start_position = unit->position,
        .start_velocity = unit->velocity,
    };
    if (vehicle_index != NO_VEHICLE_DEBUG) {
        frame->vehicle_start_position = get_live_position_debug(vehicle_index);
        frame->start_gap = get_body_gap_debug(unit_index, vehicle_index, unit->position,
                                              frame->vehicle_start_position);
    }
    frame_vehicle_pushes_debug[unit_index] = 0;
    return frame;
}


// TEMPORARY DEBUG: `log_entry_debug` logs the frames of the unit `unit_index` up to the frame in
// which its body entered the body of a vehicle, oldest first.
static void log_entry_debug(int unit_index, const EntryFrameDebug *entry_frame) {
    const Unit *vehicle = &units[entry_frame->vehicle_index];
    write_log("t %.3f dbg entry: infantry unit %d squad %d enters vehicle unit %d squad %d "
              "(forming %d, stopped %d)", entry_frame->clock_seconds, unit_index,
              units[unit_index].squad_index, entry_frame->vehicle_index, vehicle->squad_index,
              squad_slots[vehicle->squad_index].is_forming, vehicle->is_stopped);
    int count = entry_frame_counts_debug[unit_index];
    int first_count = count > ENTRY_DEBUG_FRAMES ? count - ENTRY_DEBUG_FRAMES : 0;
    for (int frame_count = first_count; frame_count < count; frame_count++) {
        int ring_index = frame_count % ENTRY_DEBUG_FRAMES;
        const EntryFrameDebug *f = &entry_frames_debug[unit_index][ring_index];        // frame
        write_log("  t %.3f vehicle %d, stopped %d, gap %.2f -> %.2f m; unit (%.2f, %.2f) v "
                  "(%.1f, %.1f), steered (%.1f, %.1f) push %.1f, moved (%.2f, %.2f) v (%.1f, "
                  "%.1f); vehicle (%.2f, %.2f) -> (%.2f, %.2f)", f->clock_seconds,
                  f->vehicle_index, f->is_stopped, f->start_gap, f->moved_gap,
                  f->start_position.x, f->start_position.y, f->start_velocity.x,
                  f->start_velocity.y, f->steered_velocity.x, f->steered_velocity.y,
                  f->vehicle_push_speed, f->moved_position.x, f->moved_position.y,
                  f->moved_velocity.x, f->moved_velocity.y, f->vehicle_start_position.x,
                  f->vehicle_start_position.y, f->vehicle_moved_position.x,
                  f->vehicle_moved_position.y);
    }
}


// TEMPORARY DEBUG: `finish_entry_frame_debug` completes the frame record after the game moved the
// unit, and logs the last frames when the body of the unit entered the body of the same vehicle
// since its last frame: between its moves, or in this move.
static void finish_entry_frame_debug(EntryFrameDebug *frame, int unit_index) {
    static double next_second;
    static int event_count;
    const Unit *unit = &units[unit_index];
    frame->moved_position = unit->position;
    frame->moved_velocity = unit->velocity;
    int count = entry_frame_counts_debug[unit_index];
    entry_frame_counts_debug[unit_index] = count + 1;
    int vehicle_index = frame->vehicle_index;
    if (vehicle_index == NO_VEHICLE_DEBUG)
        return;
    frame->vehicle_moved_position = get_live_position_debug(vehicle_index);
    frame->moved_gap = get_body_gap_debug(unit_index, vehicle_index, unit->position,
                                          frame->vehicle_moved_position);

    // The frame before must have the same vehicle and no overlap at its end.
    if (count == 0)
        return;
    int previous_index = (count - 1) % ENTRY_DEBUG_FRAMES;
    const EntryFrameDebug *previous = &entry_frames_debug[unit_index][previous_index];
    bool was_apart = previous->vehicle_index == vehicle_index && previous->moved_gap >= 0;
    bool is_inside = frame->start_gap < 0 || frame->moved_gap < 0;
    if (!was_apart || !is_inside)
        return;
    if (last_clock_seconds >= next_second) {
        next_second = last_clock_seconds + 1;
        event_count = 0;
    }
    if (event_count >= ENTRY_DEBUG_MAX_EVENTS)
        return;
    event_count++;
    log_entry_debug(unit_index, frame);
}


// `update_toggle` turns the mod on or off when the toggle key goes down.
static void update_toggle(void) {
    bool is_key_down = (GetAsyncKeyState(toggle_key) & KEY_DOWN_BIT) != 0;
    if (is_key_down && !was_toggle_key_down) {
        is_mod_on = !is_mod_on;
        write_log("mod %s", is_mod_on ? "on" : "off");
    }
    was_toggle_key_down = is_key_down;
}


// `begin_frame` prepares the formation code for one game frame, in the order of the simulator
// (sim.c `run_tick`): the new set-ups, the coordination between squads, the anchors, then the
// parked boxes of the ORCA avoidance.
static void begin_frame(void) {
    tick_seconds = *(const float *)GAME_FRAME_SECONDS_ADDRESS;
    now_seconds += tick_seconds;
    update_toggle();
    free_stale_slots();
    lay_out_units();
    read_game_state();

    // The updates of this frame mark the slots again.
    for (int slot_index = 0; slot_index < squad_count; slot_index++) {
        if (squad_slots[slot_index].has_new_route)
            start_formation(slot_index);
    }
    // As in the simulator (game/squad.c `update_squad`), a moving squad checks whether it has
    // settled and whether another squad will park ahead of it, after its anchor moves.
    coordinate_squads();
    for (int slot_index = 0; slot_index < squad_count; slot_index++) {
        if (squad_slots[slot_index].is_forming) {
            Squad *squad = &squads[slot_index];
            advance_anchor(squad);
            update_settled(squad);
            avoid_parking_ahead(squad);
        }
        squad_slots[slot_index].was_updated = false;
    }
    update_parked_boxes();
    log_player_squads_debug();
    log_slips_debug();
    log_targets_debug();
    orca_line_debug_hook = note_cone_push_debug;        // TEMPORARY DEBUG
    log_cone_pushes_debug();
}


// `begin_frame_if_new` runs `begin_frame` once per game frame, when the game clock has moved.
// Before the game timing exists, nothing happens.
static void begin_frame_if_new(void) {
    const void *timing = *(void *const *)GAME_TIMING_ADDRESS;
    if (!timing)
        return;
    double clock_seconds;
    memcpy(&clock_seconds, &((const uint8_t *)timing)[TIMING_CLOCK], sizeof clock_seconds);
    if (clock_seconds == last_clock_seconds)
        return;
    last_clock_seconds = clock_seconds;
    begin_frame();
}


// `get_mod_status` reports whether the mod is on, and returns true while a mission runs, which
// means that the game updated a squad within the last `MISSION_UPDATE_MILLISECONDS`.
bool get_mod_status(bool *is_on) {
    *is_on = is_mod_on;
    DWORD since_update = GetTickCount() - last_squad_update_ticks;
    return last_squad_update_ticks != 0 && since_update < MISSION_UPDATE_MILLISECONDS;
}


// `note_squad_update` starts a new frame when the game clock moved, then marks the squad.
void note_squad_update(void *squad_object) {
    last_squad_update_ticks = GetTickCount();
    begin_frame_if_new();
    int slot_index = find_squad_slot(squad_object);
    if (slot_index < 0)
        slot_index = add_squad_slot(squad_object);
    if (slot_index >= 0)
        squad_slots[slot_index].was_updated = true;
}


// `is_repeated_order` tells whether a squad in formation following gets an order to the
// destination that it already drives to. Computer squads can repeat their order in every frame;
// a new set-up in each of them would put the anchor back at the route start every time.
static bool is_repeated_order(int slot_index, V2 destination) {
    const Squad *squad = &squads[slot_index];
    if (!squad_slots[slot_index].is_forming)
        return false;
    V2 offset = sub(destination, squad->destination);
    return length_squared(offset) < REPEATED_ORDER_METERS * REPEATED_ORDER_METERS;
}


// `note_route` copies the route list of a move order into `new_routes`. The formation set-up
// waits for the next frame, when the game has finished the order and the squad moves. A repeated
// order keeps the formation, and the log shows only its first repetition.
void note_route(void *squad_object, const void *route_list) {
    int slot_index = find_squad_slot(squad_object);
    if (slot_index < 0)
        slot_index = add_squad_slot(squad_object);
    if (slot_index < 0)
        return;
    int count = 0;
    const void *node = read_pointer(route_list, LIST_HEAD);
    while (node && count < MAX_WAYPOINTS - 1) {
        new_routes[slot_index][count] = v2(read_float(node, NODE_X), read_float(node, NODE_Y));
        count++;
        node = read_pointer(node, NODE_NEXT);
    }
    SquadSlot *slot = &squad_slots[slot_index];
    if (count > 0 && is_repeated_order(slot_index, new_routes[slot_index][count - 1])) {
        if (!slot->has_logged_repeat)
            write_log("t %.1f order: squad %d (%s) repeats its order, keeps its formation",
                      last_clock_seconds, slot_index, get_order_source(squad_object));
        slot->has_logged_repeat = true;
        return;
    }
    new_route_counts[slot_index] = count;
    slot->has_new_route = true;
    slot->has_logged_repeat = false;
}


// `move_formation_member` moves one formation member for a frame as *Tick* does, with the
// steering of the formation code in place of the waypoint following, *Steer* and *AvoidUnits*.
static void move_formation_member(void *mover, const void *entity, int unit_index) {
    Unit *unit = &units[unit_index];
    int avoid_counter = read_int(mover, MOVER_AVOID_COUNTER) - 1;
    write_int(mover, MOVER_AVOID_COUNTER, avoid_counter < 0 ? 0 : avoid_counter);
    ((MoverFunction)APPLY_PENDING_VELOCITY_ADDRESS)(mover);

    // The formation code accelerates the unit and returns its steering; the game turns it.
    EntryFrameDebug *entry_frame = NULL;        // TEMPORARY DEBUG
    if (read_byte(entity, ENTITY_ACTIVE)) {
        read_unit_state(unit, entity);
        entry_frame = begin_entry_frame_debug(unit_index);        // TEMPORARY DEBUG
        float steering = 0;
        if (unit->is_stopped)
            steering = step_aside_for_members(unit_index);
        else
            steering = steer_in_formation(unit_index);
        write_float(mover, MOVER_VELOCITY_X, unit->velocity.x);
        write_float(mover, MOVER_VELOCITY_Y, unit->velocity.y);
        write_byte(mover, MOVER_STOPPED, unit->is_stopped);
        note_slip_debug(steered_slip_speeds, unit_index);
        if (entry_frame) {
            // TEMPORARY DEBUG
            entry_frame->steered_velocity = unit->velocity;
            entry_frame->vehicle_push_speed = frame_vehicle_pushes_debug[unit_index];
        }
        TurnFunction turn = (TurnFunction)get_vtable_function(mover, MOVER_TURN_SLOT);
        turn(mover, steering);
    }

    // The other units of this frame see the new state.
    MoverFunction apply_friction = (MoverFunction)get_vtable_function(mover, MOVER_FRICTION_SLOT);
    MoverFunction integrate = (MoverFunction)get_vtable_function(mover, MOVER_INTEGRATE_SLOT);
    apply_friction(mover);
    integrate(mover);
    read_unit_state(unit, entity);
    note_slip_debug(moved_slip_speeds, unit_index);
    if (entry_frame)
        finish_entry_frame_debug(entry_frame, unit_index);        // TEMPORARY DEBUG
}


// `tick_formation_mover` moves the unit of `mover` when its squad is in formation following.
bool tick_formation_mover(void *mover) {
    begin_frame_if_new();
    if (!is_mod_on)
        return false;
    const void *entity = read_pointer(mover, MOVER_ENTITY);
    int unit_index = find_unit_index(entity);
    if (unit_index < 0)
        return false;
    if (!squad_slots[units[unit_index].squad_index].is_forming)
        return false;
    move_formation_member(mover, entity, unit_index);
    return true;
}


// `note_squad_destroyed` frees the slot of a squad that the game destroys now. Without it, the
// next frame could read the member vector of the destroyed squad.
void note_squad_destroyed(void *squad_object) {
    int slot_index = find_squad_slot(squad_object);
    if (slot_index >= 0)
        free_squad_slot(slot_index);
}


// `stop_bridge` stops the members of each squad in formation following where they stand, as
// `Squad::Stop` does, before the core unloads; the game then sees them arrive. Without the stop,
// the game would take over in the middle of a move, with waypoints that the formation code did
// not follow. The members come from the member vector of the game, which is current between
// frames, while `unit_entities` could still hold a unit destroyed in the last frame.
void stop_bridge(void) {
    StopMoverFunction stop_mover = (StopMoverFunction)STOP_MOVER_ADDRESS;
    for (int slot_index = 0; slot_index < MAX_SQUADS; slot_index++) {
        const SquadSlot *slot = &squad_slots[slot_index];
        if (!slot->squad_object || !slot->is_forming)
            continue;
        int member_count;
        void **members = get_members(slot->squad_object, &member_count);
        for (int member_index = 0; member_index < member_count; member_index++) {
            void *entity = members[member_index];
            if (!is_ground_entity(entity))
                continue;

            // The position of an entity is three floats from `ENTITY_X` on: x, height, z.
            const void *position = &((const uint8_t *)entity)[ENTITY_X];
            stop_mover(read_pointer(entity, ENTITY_MOVER), position);
        }
        write_log("squad %d: stopped for the core reload", slot_index);
    }
}


// `note_map_loaded` forgets every squad and unit of the last mission, and selects the clearance
// fields of the new map, which it builds on first use.
void note_map_loaded(const char *map_name) {
    for (int slot_index = 0; slot_index < MAX_SQUADS; slot_index++)
        free_squad_slot(slot_index);
    unit_count = 0;
    squad_count = 0;
    last_clock_seconds = -1;
    select_map_clearance(map_name);
    write_log("map %s loaded", map_name);
}
