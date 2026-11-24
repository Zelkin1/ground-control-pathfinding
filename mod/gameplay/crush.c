// Run-over kills (`--crush`). A ground vehicle that drives at `CRUSH_MIN_SPEED` or faster kills
// each foot unit of a team that is not allied with its own when their bodies touch, with a body
// that counts as `CRUSH_CONTACT_MARGIN_METERS` larger than its radius. The kill goes
// through the death function of the game, with the vehicle's squad as the killer, as with a
// weapon. The ORCA avoidance lets a vehicle in formation following drive straight at such
// infantry (mod/pathfinding/formation/avoidance.c).
//
// Touch is the contact test of the game itself. Each unit moves in its own tick, and then
// *Physics* 0x510b20 pushes it out of every unit whose body it overlaps, to 0.3 m apart (0x612004),
// and takes away its speed toward that unit. A hook on *Physics* therefore checks for contact
// before the push, while the bodies overlap and the vehicle still has its speed.
//
// The kills wait until the frame ends (core.c `note_frame_shown`), when no squad or entity update
// of the game is under way: the death function removes the unit from its squad, and a squad that
// loses its last member deletes itself at once (0x5048d0). At the same time, the lists of the
// vehicles and foot units are made for the next frame.
#include "mod.h"
#include "game_entity.h"
#include "sim.h"

// More fields of the entity, from *Physics* 0x510b20, the damage function 0x4fa770 and the death
// function 0x4fa0f0.
#define ENTITY_RADIUS 0x24                  // float, body radius in the contact test of *Physics*
#define ENTITY_ALIVE 0x3c                   // byte; the death function clears it
#define ENTITY_SQUAD_ID 0x3e                // int16, id of the squad of the unit
#define ENTITY_INVULNERABLE 0xa8            // byte; damage skips a unit with it set
#define ENTITY_KILLER_SQUAD_ID 0xb8         // int16, id of the squad that killed the unit
#define ENTITY_KILLER_CLIENT 0xbc           // 4 bytes, `SQUAD_CLIENT` of the killer squad

// MoverType of the entity type, an int that the ENT parser sets (0x421419).
#define TYPE_MOVER_TYPE 0x38
#define MOVER_TYPE_FOOT 1
#define MOVER_TYPE_VEHICLE 3
#define MOVER_TYPE_HOVER 4

// The damage function hits a unit only when the byte at `DAMAGE_LIMIT_ADDRESS` is 0, or the
// squad of the unit has the byte `SQUAD_DAMAGE_FLAG` set (TODO: confirm what both mean).
#define DAMAGE_LIMIT_ADDRESS 0x6eb424
#define SQUAD_DAMAGE_FLAG 0x19
// Client of a squad of a player, which the damage function copies into `ENTITY_KILLER_CLIENT`
// (TODO: confirm its value for squads of the computer).
#define SQUAD_CLIENT 0xa4

// thiscall (entity, with effect): the death of a unit, as the damage function and the script
// action KILL call it with the effect on.
#define KILL_UNIT_ADDRESS 0x4fa0f0
#define WITH_DEATH_EFFECT 1

// thiscall (mover, position before the move), `ret 4`: *Physics*, which both *Integrate*
// functions (0x5109e0, and 0x513010 of vehicles) call after the move of a unit.
#define PHYSICS_ADDRESS 0x510b20
#define PHYSICS_STOLEN_BYTES 6              // mov eax, fs:[0]

#define CRUSH_MIN_SPEED 3.0f                // meters per second of the vehicle
// Gap between the bodies, by their radii, that already counts as touch. With the radii alone, the
// game nearly always pushed the units apart before they overlapped (in game, 3 kills in a test
// with many vehicle and infantry contacts), so its own contact test most likely uses larger bodies
// (TODO: confirm the radius of the collision object, `+0xc`, which *Physics* reads).
#define CRUSH_CONTACT_MARGIN_METERS 0.7f

typedef void(THISCALL *KillUnitFunction)(void *entity, uint8_t with_effect);
typedef uint8_t(THISCALL *PhysicsFunction)(void *mover, void *old_position);

// A ground unit that can kill or be killed, as it was at the end of the last frame, with a copy
// of the fields of its squad that a run-over needs. `entity` is only compared until a kill finds
// it again in the member vector of its squad.
typedef struct {
    void *entity;
    int slot_index;
    V2 position;
    float radius;
    float speed;
    uint8_t team;                // team byte of the squad, for the alliance test of the game
    bool can_take_damage;        // the damage function of the game would hit the unit
    uint16_t squad_id;
    uint32_t squad_client;
} CrushUnit;

// A foot unit that a vehicle ran over in this frame, and the killer data that the game records.
typedef struct {
    void *entity;
    int slot_index;
    int killer_slot_index;
    uint16_t killer_squad_id;
    uint32_t killer_client;
} RunOver;

static CrushUnit vehicles[MAX_UNITS];
static CrushUnit foot_units[MAX_UNITS];
static int vehicle_count;
static int foot_count;
static RunOver run_overs[MAX_UNITS];
static int run_over_count;
static PhysicsFunction original_physics;


// `write_u16` writes the unsigned 16-bit number at `offset` in the game object `object`.
static void write_u16(void *object, int offset, uint16_t value) {
    memcpy(&((uint8_t *)object)[offset], &value, sizeof value);
}


// `read_entity_position` returns the position of the unit `entity` on the map plane.
static V2 read_entity_position(const void *entity) {
    return v2(read_float(entity, ENTITY_X), read_float(entity, ENTITY_Y));
}


// `read_entity_speed` returns the speed of the unit `entity`.
static float read_entity_speed(const void *entity) {
    const void *mover = read_pointer(entity, ENTITY_MOVER);
    return len(v2(read_float(mover, MOVER_VELOCITY_X), read_float(mover, MOVER_VELOCITY_Y)));
}


// `can_take_damage` tells whether the damage function of the game would hit `entity` of the squad
// `squad_object`, so that a run-over never kills a unit that the game protects, such as a unit of
// a mission script.
static bool can_take_damage(const void *entity, const void *squad_object) {
    if (read_byte(entity, ENTITY_INVULNERABLE))
        return false;
    bool is_damage_limited = *(const uint8_t *)DAMAGE_LIMIT_ADDRESS != 0;
    return !is_damage_limited || read_byte(squad_object, SQUAD_DAMAGE_FLAG) != 0;
}


// `make_crush_unit` copies the place, body radius and speed of the unit `entity`, and the fields
// of its squad that a run-over needs.
static CrushUnit make_crush_unit(void *entity, const void *squad_object, int slot_index) {
    return (CrushUnit){
        .entity = entity,
        .slot_index = slot_index,
        .position = read_entity_position(entity),
        .radius = read_float(entity, ENTITY_RADIUS),
        .speed = read_entity_speed(entity),
        .team = read_byte(squad_object, SQUAD_TEAM),
        .can_take_damage = can_take_damage(entity, squad_object),
        .squad_id = read_u16(entity, ENTITY_SQUAD_ID),
        .squad_client = (uint32_t)read_int(squad_object, SQUAD_CLIENT),
    };
}


// `is_on_ground` tells whether a squad member is alive, active and moves on the ground, outside
// an APC.
static bool is_on_ground(const void *entity) {
    bool is_alive = read_byte(entity, ENTITY_ALIVE) != 0;
    bool is_active = read_byte(entity, ENTITY_ACTIVE) != 0;
    bool is_carried = read_pointer(entity, ENTITY_CARRIER) != NULL;
    bool has_mover = read_pointer(entity, ENTITY_MOVER) != NULL;
    return is_alive && is_active && has_mover && !is_carried;
}


// `list_crush_units` lists the vehicles and the foot units of every squad that the bridge knows.
static void list_crush_units(void) {
    vehicle_count = 0;
    foot_count = 0;
    for (int slot_index = 0; slot_index < MAX_SQUADS; slot_index++) {
        const void *squad_object = get_game_squad(slot_index);
        if (!squad_object)
            continue;
        int member_count;
        void **members = get_members(squad_object, &member_count);
        for (int member_index = 0; member_index < member_count; member_index++) {
            void *entity = members[member_index];
            if (!is_on_ground(entity))
                continue;
            const void *type_object = read_pointer(entity, ENTITY_TYPE);
            int mover_type = read_int(type_object, TYPE_MOVER_TYPE);
            bool is_vehicle = mover_type == MOVER_TYPE_VEHICLE || mover_type == MOVER_TYPE_HOVER;
            CrushUnit unit = make_crush_unit(entity, squad_object, slot_index);
            if (is_vehicle && vehicle_count < MAX_UNITS) {
                vehicles[vehicle_count] = unit;
                vehicle_count++;
            } else if (mover_type == MOVER_TYPE_FOOT && foot_count < MAX_UNITS) {
                foot_units[foot_count] = unit;
                foot_count++;
            }
        }
    }
}


// `find_crush_unit` returns the index of `entity` among the `count` units of `list`, or -1.
static int find_crush_unit(const CrushUnit *list, int count, const void *entity) {
    for (int unit_index = 0; unit_index < count; unit_index++) {
        if (list[unit_index].entity == entity)
            return unit_index;
    }
    return -1;
}


// `is_running_over` tells whether `vehicle` runs over `foot_unit`: their bodies touch, the
// vehicle drives fast enough, and its team is no ally of the foot unit's team.
static bool is_running_over(const CrushUnit *vehicle, const CrushUnit *foot_unit) {
    float touch_distance = vehicle->radius + foot_unit->radius + CRUSH_CONTACT_MARGIN_METERS;
    float distance_squared = length_squared(sub(foot_unit->position, vehicle->position));
    bool is_touching = distance_squared < touch_distance * touch_distance;
    if (!is_touching || vehicle->speed < CRUSH_MIN_SPEED || !foot_unit->can_take_damage)
        return false;
    IsAlliedTeamFunction is_allied_team = (IsAlliedTeamFunction)IS_ALLIED_TEAM_ADDRESS;
    return is_allied_team(&vehicle->team, &foot_unit->team) == 0;
}


// `record_run_over` adds `foot_unit` to the kills at the end of the frame, once, with
// `vehicle` as its killer.
static void record_run_over(const CrushUnit *foot_unit, const CrushUnit *vehicle) {
    for (int run_over_index = 0; run_over_index < run_over_count; run_over_index++) {
        if (run_overs[run_over_index].entity == foot_unit->entity)
            return;
    }
    if (run_over_count == MAX_UNITS)
        return;
    run_overs[run_over_count] = (RunOver){
        .entity = foot_unit->entity,
        .slot_index = foot_unit->slot_index,
        .killer_slot_index = vehicle->slot_index,
        .killer_squad_id = vehicle->squad_id,
        .killer_client = vehicle->squad_client,
    };
    run_over_count++;
}


// `check_contacts` records the run-overs of the unit `entity`, which has just moved, with the
// units that stood in its way at the end of the last frame. A moving vehicle runs over foot
// units, and a foot unit can walk into a moving vehicle.
static void check_contacts(const void *entity) {
    int vehicle_index = find_crush_unit(vehicles, vehicle_count, entity);
    if (vehicle_index >= 0) {
        CrushUnit vehicle = vehicles[vehicle_index];
        vehicle.position = read_entity_position(entity);
        vehicle.speed = read_entity_speed(entity);
        for (int foot_index = 0; foot_index < foot_count; foot_index++) {
            if (is_running_over(&vehicle, &foot_units[foot_index]))
                record_run_over(&foot_units[foot_index], &vehicle);
        }
        return;
    }

    // The speed of a vehicle is that of the end of the last frame, before its own move.
    int foot_index = find_crush_unit(foot_units, foot_count, entity);
    if (foot_index < 0)
        return;
    CrushUnit foot_unit = foot_units[foot_index];
    foot_unit.position = read_entity_position(entity);
    for (int index = 0; index < vehicle_count; index++) {
        if (is_running_over(&vehicles[index], &foot_unit))
            record_run_over(&foot_unit, &vehicles[index]);
    }
}


// `hook_physics` checks the unit of `mover` for run-overs after its move and before the
// original pushes it out of the bodies it overlaps.
static uint8_t THISCALL hook_physics(void *mover, void *old_position) {
    if (vehicle_count > 0 && foot_count > 0)
        check_contacts(read_pointer(mover, MOVER_ENTITY));
    return original_physics(mover, old_position);
}


// `is_squad_member` tells whether `entity` is still a member of the game squad in the slot
// `slot_index`. A unit that died in this frame may have left its squad, and its squad may be gone.
static bool is_squad_member(const void *entity, int slot_index) {
    const void *squad_object = get_game_squad(slot_index);
    if (!squad_object)
        return false;
    int member_count;
    void **members = get_members(squad_object, &member_count);
    for (int member_index = 0; member_index < member_count; member_index++) {
        if (members[member_index] == entity)
            return true;
    }
    return false;
}


// `kill_run_over_units` kills the foot units that vehicles ran over in this frame. The killer
// fields are those that the damage function writes before it calls the death function, so the
// game counts the kill for the squad of the vehicle.
static void kill_run_over_units(void) {
    KillUnitFunction kill_unit = (KillUnitFunction)KILL_UNIT_ADDRESS;
    for (int run_over_index = 0; run_over_index < run_over_count; run_over_index++) {
        const RunOver *run_over = &run_overs[run_over_index];
        bool is_member = is_squad_member(run_over->entity, run_over->slot_index);
        if (!is_member || !read_byte(run_over->entity, ENTITY_ALIVE))
            continue;
        write_u16(run_over->entity, ENTITY_KILLER_SQUAD_ID, run_over->killer_squad_id);
        write_int(run_over->entity, ENTITY_KILLER_CLIENT, (int)run_over->killer_client);
        kill_unit(run_over->entity, WITH_DEATH_EFFECT);
        write_log("crush: a unit of squad %d run over by squad %d", run_over->slot_index,
                  run_over->killer_slot_index);
    }
    run_over_count = 0;
}


// `note_crush_frame` kills the units that vehicles ran over in the frame that ends, then lists
// the vehicles and foot units for the next frame. Outside a mission, with the mod off or without
// the switch, it forgets them all, so the hook finds no unit.
void note_crush_frame(bool is_active) {
    if (!is_active || !is_crush_on) {
        vehicle_count = 0;
        foot_count = 0;
        run_over_count = 0;
        return;
    }
    kill_run_over_units();
    list_crush_units();
}


// `install_crush_hook` hooks *Physics* with `install_hook` while the switch is on.
void install_crush_hook(InstallHookFunction install_hook) {
    if (!is_crush_on)
        return;
    original_physics = install_hook(PHYSICS_ADDRESS, PHYSICS_STOLEN_BYTES,
                                    (const void *)hook_physics);
    if (!original_physics)
        write_log("hook of the unit physics failed: no run-over kills");
}
