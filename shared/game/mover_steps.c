// Steps of the base game movement that formation following uses too: the steering input of
// *SteerTo* (0x511400), the acceleration step 0x511500, and the unit path search of
// `Mover::FindPath` (0x50eb60). The simulator and the game mod both compile this file; in the
// mod, these steps act on the copy of a game unit that the bridge keeps.
#include "sim.h"


// `get_corner_gap` returns the corner gap of a unit for the base game path search: twice its
// radius, as the game uses.
float get_corner_gap(const Unit *unit) {
    return 2 * unit->unit_type->radius;
}


// `search_path` runs the base game path search of a unit to `destination` in one call: the forward
// search, then the backward search when the forward search fails, each with `budget` expansions.
// The parked squads count as cliffs during the search (`stamp_parked_for_search`, stamping.c).
bool search_path(const Unit *unit, V2 destination, float tolerance, int budget,
                 GcRequest *search) {
    gc_init_request(search);
    V2 from = unit->position;
    int terrain_class = unit->unit_type->terrain_class;
    float corner_gap = get_corner_gap(unit);
    stamp_parked_for_search(&squads[unit->squad_index], from, destination);

    gc_start_search(search, GC_SEARCH_FORWARD, from.x, from.y, destination.x, destination.y,
                    terrain_class, corner_gap, tolerance);
    bool is_found = gc_update_search(search, budget) == GC_SEARCH_FOUND;
    if (!is_found) {
        gc_start_search(search, GC_SEARCH_BACKWARD, from.x, from.y, destination.x, destination.y,
                        terrain_class, corner_gap, tolerance);
        is_found = gc_update_search(search, budget) == GC_SEARCH_FOUND;
    }

    unstamp_parked();
    return is_found;
}


// `steer_to` returns the steering input of *SteerTo* (0x511400) toward `direction`: full lock for
// a direction behind the unit, and `1 - c^6` for a direction ahead, where `c` is the cosine of the
// angle to it. A negative input turns left, counterclockwise as seen from above.
float steer_to(const Unit *unit, V2 direction) {
    bool is_left = direction.x * unit->heading.y < direction.y * unit->heading.x;
    float side = is_left ? -1.0f : 1.0f;
    float alignment = dot(direction, unit->heading);
    if (alignment >= 0) {
        float cos_squared = alignment * alignment / length_squared(direction);
        return side * (1 - cos_squared * cos_squared * cos_squared);
    }
    return side;
}


// `steer_to_facing` returns the steering input toward the final facing of a unit, or zero when
// the unit has no final facing.
float steer_to_facing(const Unit *unit) {
    if (length_squared(unit->facing) > 0)
        return steer_to(unit, unit->facing);
    return 0;
}


// `accelerate` speeds a unit up along its heading for one tick, with `throttle` times its
// acceleration, as the acceleration step 0x511500 does with the Mover acceleration `+0x48`. A unit
// that drives backward speeds up opposite to its heading (`--reverse`), which the game has no
// step for.
void accelerate(Unit *unit, float throttle) {
    float gain = unit->acceleration * TICK_SECONDS * throttle;        // speed gain
    unit->velocity = add(unit->velocity, mul(get_drive_direction(unit), gain));
}


// `accelerate_toward_speed` sets the throttle that closes the gap between the speed of a unit
// along its drive direction and `target_speed` in about `time_constant_seconds`. The throttle also
// covers the friction, which `apply_friction` takes away in the same tick.
void accelerate_toward_speed(Unit *unit, float target_speed, float time_constant_seconds) {
    float v = dot(unit->velocity, get_drive_direction(unit));       // speed along the drive
    float rate = (target_speed - v) / time_constant_seconds;        // rate of change of the speed
    float a = rate + unit->unit_type->friction;                     // needed acceleration
    float throttle = a / unit->acceleration;
    accelerate(unit, clamp_float(throttle, 0, 1));
}
