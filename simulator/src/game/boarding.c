// Rebuilt boarding of an APC (docs/PATHFINDING.md). A unit whose mover has a target unit drives
// straight at it, as `Mover::Update` (0x50eec0) does while the line to the target is clear, and
// *Tick* (0x50f634) puts it inside the target once it comes within `BOARDING_RADII` radii of the
// target. The simulator models open ground only: a chasing unit never searches a path. A unit
// inside an APC rides at the APC position; the movement, the avoidance, the physics and the
// metrics of the other units leave it out, as the mod leaves out units inside an APC.
#include "sim.h"

#define BOARDING_RADII 2.0f                    // a unit boards within this many APC radii


// `is_chasing` tells whether a unit drives to its target unit and is not inside it yet.
bool is_chasing(const Unit *unit) {
    return unit->target_unit_index != NO_TARGET_UNIT && !unit->is_carried;
}


// `board_if_close` puts a chasing unit inside its target APC once their centers are within
// `BOARDING_RADII` radii of the APC.
void board_if_close(Unit *unit) {
    const Unit *carrier = &units[unit->target_unit_index];
    float boarding_distance = BOARDING_RADII * carrier->unit_type->radius;
    float d_squared = length_squared(sub(carrier->position, unit->position));
    if (d_squared <= boarding_distance * boarding_distance)
        unit->is_carried = true;
}


// `aim_at_target` makes the position of the target unit the only waypoint of a chasing unit.
// A standing unit starts with a turn on the spot, as after an order.
void aim_at_target(Unit *unit) {
    if (unit->is_stopped) {
        unit->is_stopped = false;
        unit->is_reorienting = true;
    }

    // The waypoint does not count as the last one, so the unit does not brake toward the moving
    // target (TODO: confirm how the game brakes toward a target).
    unit->waypoint = units[unit->target_unit_index].position;
    unit->waypoint_count = 0;
    unit->is_last_waypoint = false;
    unit->reach = unit->reach_mid;
}


// `ride_in_carrier` keeps a carried unit at the position and heading of its APC, standing.
void ride_in_carrier(Unit *unit) {
    const Unit *carrier = &units[unit->target_unit_index];
    unit->position = carrier->position;
    unit->angle = carrier->angle;
    unit->heading = carrier->heading;
    unit->velocity = v2(0, 0);
    unit->waypoint = carrier->position;
    unit->is_stopped = true;
    unit->is_reorienting = false;
}
