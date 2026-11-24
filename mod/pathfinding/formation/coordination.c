// Coordination between squads in the coordination mode (`--avoid coord`). The squads adapt to the
// paths of each other through the predictions of moving_ahead.c (`--moveahead`), which re-route a
// squad or give it a yield speed cap. `coordinate_squads` applies those caps once per tick, and it
// makes a squad that its cap keeps nearly still for too long ignore the cap for a while, so that
// two squads cannot block each other forever. It runs in every formation mode, but only the
// coordination mode sets caps.
#include "sim.h"
#include "formation.h"

#define CAP_WAIT_SPEED 0.2f                                // slower caps count as waiting
#define GIVE_UP_SECONDS 15.0f                              // longest wait before giving up
#define IGNORE_SECONDS 5.0f                                // coordination pause after giving up


// `measure_shape` measures the formation of a squad for the coordination: how far the bodies reach
// ahead of the anchor and to the side of the route, the route position of the rear end of the
// formation, and the top speed of the squad.
FormationShape measure_shape(const Squad *squad) {
    FormationShape shape = {
        .front_meters = 0,
        .half_width = 0,
        .rear_progress = LARGE_VALUE,
        .squad_top_speed = LARGE_VALUE,
    };
    int end_index = squad->first_unit_index + squad->unit_count;
    for (int unit_index = squad->first_unit_index; unit_index < end_index; unit_index++) {
        const Unit *unit = &units[unit_index];
        float r = unit->unit_type->radius;
        V2 slot = unit->slot_position;
        float front = slot.y + r;                         // body front, anchor frame
        float side = fabsf(slot.x) + r;                   // body side, off the route
        float rear = unit->route_progress - r;            // body rear, route position
        shape.front_meters = fmaxf(shape.front_meters, front);
        shape.half_width = fmaxf(shape.half_width, side);
        shape.rear_progress = fminf(shape.rear_progress, rear);
        shape.squad_top_speed = fminf(shape.squad_top_speed, unit->top_speed);
    }
    return shape;
}


// `update_give_up` measures how long the speed cap of a squad keeps its anchor nearly still. After
// `GIVE_UP_SECONDS`, the squad ignores the other squads for `IGNORE_SECONDS`.
static void update_give_up(Squad *squad) {
    if (squad->speed_cap < CAP_WAIT_SPEED)
        squad->speed_cap_wait_seconds += TICK_SECONDS;
    else
        squad->speed_cap_wait_seconds = 0;
    if (squad->speed_cap_wait_seconds > GIVE_UP_SECONDS) {
        squad->speed_cap_wait_seconds = 0;
        squad->ignore_coordination_seconds = IGNORE_SECONDS;
    }
}


// `coordinate_squads` sets the speed cap of each squad for one tick: the yield speed cap of the
// predictions of the last tick, unless the squad ignores the coordination.
void coordinate_squads(void) {
    for (int squad_index = 0; squad_index < squad_count; squad_index++) {
        Squad *squad = &squads[squad_index];
        squad->ignore_coordination_seconds -= TICK_SECONDS;
        squad->speed_cap = LARGE_VALUE;
        if (is_move_ahead_on)
            squad->speed_cap = squad->yield_speed_cap;
        update_give_up(squad);
        if (squad->ignore_coordination_seconds > 0)
            squad->speed_cap = LARGE_VALUE;
    }
}
