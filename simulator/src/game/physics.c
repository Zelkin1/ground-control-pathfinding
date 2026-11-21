// Rebuilt movement physics of the base game (docs/PATHFINDING.md): *Turn* (0x511560), *Friction*
// (0x50fdf0), *Integrate* (0x5109e0), and *Physics* (0x510b20) with its collision push, its
// blocking terrain slide (*WallSlide*, 0x511170) and its slope rescue (0x511000). Where the base
// game tests collision boxes in its collision map, the rebuild tests circles of the unit radii.
// The rebuild keeps the behavior of the base game, except for three changes:
// - the slope rescue gives up after `SLOPE_RESCUE_MAX_CELLS`, where the base game would search on
//   forever when no free cell lies on its four lines;
// - in the formation modes, the slide jump guard ignores a hit cell far from the ray. For a tiny
//   move along a cell edge, the ray of the base game can return a blocking cell hundreds of
//   meters away, and the slide would move the unit there in one tick;
// - in the formation modes, collision pushes never move the units of parked squads.
//
// The heading angle of a unit is measured clockwise from the y axis, as seen from above.
#include "sim.h"

#define MIN_PUSH_DISTANCE_SQUARED 1e-5f        // closer centers give no push direction
#define PUSH_GAP_METERS 0.3f                   // gap between the bodies after a push
#define MIN_PUSHED_AWAY_SPEED -0.1f            // faster approaches lose their approach speed
#define SLOW_PUSH_SPEED 0.1f                   // slower units get `MIN_PUSH_SPEED`
#define MIN_PUSH_SPEED 0.2f                    // push speed of a slow unit
#define SLIDE_OFFSET_CELLS 0.55f               // slide distance from the blocking cell center
#define SIDE_COOLDOWN_SECONDS 2.0f             // shortest time between two side flips
#define SLOPE_RESCUE_DIRECTION_COUNT 4         // rescue directions: +x, -x, +y, -y
#define SLOPE_RESCUE_STEP_CELLS 0.5f           // step of the slope rescue
#define SLOPE_RESCUE_MAX_CELLS 20.0f           // the slope rescue gives up at this distance
#define HIT_REACH_CELLS 1.0f                   // slide jump guard: farthest hit cell from the ray
#define WORLD_MIN_METERS 10.0f                 // smallest x or y of a unit
#define WORLD_MAX_METERS 6390.0f               // largest x or y of a unit
#define NO_UNIT -1                             // no overlapping unit


// `turn` turns a unit for one tick with `steering`, the steering input from -1 to 1 of its turn
// rate. A positive input turns the unit clockwise, and the angle wraps into `[0, 2 * PI)`.
void turn(Unit *unit, float steering) {
    float s = clamp_float(steering, -1, 1);        // clamped steering input
    unit->last_turn_rate = unit->unit_type->turn_rate * s;
    unit->angle += unit->unit_type->turn_rate * TICK_SECONDS * s;
    if (unit->angle >= 2 * PI)
        unit->angle -= 2 * PI;
    if (unit->angle < 0)
        unit->angle += 2 * PI;
    unit->heading = v2(sinf(unit->angle), cosf(unit->angle));
}


// `apply_friction` slows a unit by its friction for one tick and caps its speed at its top speed,
// as *Friction* (0x50fdf0) caps it at the Mover top speed `+0x40`. A velocity component that would
// change its sign becomes zero.
void apply_friction(Unit *unit) {
    float speed = len(unit->velocity);
    if (speed > 0) {
        V2 direction = mul(unit->velocity, 1 / speed);
        float loss = unit->unit_type->friction * TICK_SECONDS;        // speed loss
        V2 v = sub(unit->velocity, mul(direction, loss));
        if (v.x * unit->velocity.x <= 0)
            v.x = 0;
        if (v.y * unit->velocity.y <= 0)
            v.y = 0;
        unit->velocity = v;
    }

    float top_speed = unit->top_speed;
    if (length_squared(unit->velocity) > top_speed * top_speed)
        unit->velocity = mul(unit->velocity, top_speed / len(unit->velocity));
}


// `find_overlapping_unit` returns the index of the first unit whose circle overlaps the circle of
// the unit `unit_index`, or `NO_UNIT`.
static int find_overlapping_unit(int unit_index) {
    const Unit *unit = &units[unit_index];
    for (int other_index = 0; other_index < unit_count; other_index++) {
        const Unit *other = &units[other_index];
        if (other_index == unit_index || other->is_carried)
            continue;
        float r = unit->unit_type->radius + other->unit_type->radius;        // combined radius
        float d_squared = length_squared(sub(other->position, unit->position));
        if (d_squared < r * r)
            return other_index;
    }
    return NO_UNIT;
}


// `get_cell_center_coordinate` returns the x or y of the center of the cell `cell_coordinate`.
static float get_cell_center_coordinate(int cell_coordinate) {
    return (cell_coordinate + 0.5f) * GC_CELL_METERS;
}


// `measure_segment_distance` returns the distance from `point` to the segment from `from` to `to`.
static float measure_segment_distance(V2 point, V2 from, V2 to) {
    V2 segment = sub(to, from);
    float l_squared = length_squared(segment);        // squared segment length
    float t = 0;                                      // fraction of the projection
    if (l_squared > 0)
        t = dot(sub(point, from), segment) / l_squared;
    float t_clamped = clamp_float(t, 0, 1);
    V2 closest = add(from, mul(segment, t_clamped));
    return len(sub(point, closest));
}


// `get_slide_coordinate` returns the x or y at which a slide puts a unit next to the blocking
// cell with the center `cell_center`: on the side that the unit comes from, given by the sign of
// `move`.
static float get_slide_coordinate(float cell_center, float move) {
    float offset = GC_CELL_METERS * SLIDE_OFFSET_CELLS;
    if (move <= 0)
        return cell_center + offset;
    return cell_center - offset;
}


// `is_hit_off_ray` runs the slide jump guard: it tells whether the hit cell with the center
// `hit_center` lies more than `HIT_REACH_CELLS` from the ray from `from` to `to`.
static bool is_hit_off_ray(V2 hit_center, V2 from, V2 to) {
    float hit_reach = HIT_REACH_CELLS * GC_CELL_METERS;
    return measure_segment_distance(hit_center, from, to) > hit_reach;
}


// `slide_along_terrain` runs one blocking terrain slide: when the move from `old_position` to
// `new_position` hits blocking terrain, the velocity component across the hit cell side becomes
// zero and `new_position` moves next to that side. It returns true when the move is clear.
static bool slide_along_terrain(Unit *unit, V2 old_position, V2 *new_position) {
    V2 from = old_position;
    V2 to = *new_position;
    int terrain_class = unit->unit_type->terrain_class;
    int hit_cell_index = gc_cast_ray(from.x, from.y, to.x, to.y, terrain_class);
    if (hit_cell_index == GC_RAY_CLEAR)
        return true;

    // The slide jump guard counts a hit cell far from the ray as a clear move.
    float hit_x = get_cell_center_coordinate(get_cell_x(hit_cell_index));
    float hit_y = get_cell_center_coordinate(get_cell_y(hit_cell_index));
    V2 hit_center = v2(hit_x, hit_y);
    if (is_formation_mode() && is_hit_off_ray(hit_center, from, to))
        return true;

    // The ray reports which side of the hit cell it crossed last.
    if (gc_get_last_ray_side() == GC_RAY_STEP_ALONG_Y) {
        unit->velocity.y = 0;
        new_position->y = get_slide_coordinate(hit_y, new_position->y - old_position.y);
    } else {
        unit->velocity.x = 0;
        new_position->x = get_slide_coordinate(hit_x, new_position->x - old_position.x);
    }
    return false;
}


// `push_out` pushes the unit `unit_index` out of the circle of `other`, to touching distance plus
// `PUSH_GAP_METERS`, along the line between the two centers. When the unit then still overlaps a
// unit, it goes back to `old_position`, and `push_out` returns false.
static bool push_out(int unit_index, const Unit *other, V2 old_position) {
    // Centers that almost coincide give no direction, and the unit then stays on `other`.
    Unit *unit = &units[unit_index];
    V2 offset = sub(unit->position, other->position);
    float d = len(offset);                                               // center distance
    V2 away = v2(0, 0);
    if (d * d > MIN_PUSH_DISTANCE_SQUARED)
        away = mul(offset, 1 / d);
    float r = other->unit_type->radius + unit->unit_type->radius;        // combined radius
    float d_push = r + PUSH_GAP_METERS;
    unit->position = add(other->position, mul(away, d_push));

    // A unit that moves toward `other` faster than `MIN_PUSHED_AWAY_SPEED` loses that part of its
    // velocity. Any other unit gets its speed, or at least `MIN_PUSH_SPEED`, away from `other`.
    float v_away = dot(away, unit->velocity);
    if (v_away >= MIN_PUSHED_AWAY_SPEED) {
        float speed = len(unit->velocity);
        float v_push = speed < SLOW_PUSH_SPEED ? MIN_PUSH_SPEED : speed;
        unit->velocity = add(unit->velocity, mul(away, v_push));
    } else {
        unit->velocity = sub(unit->velocity, mul(away, v_away));
    }

    if (find_overlapping_unit(unit_index) != NO_UNIT) {
        unit->position = old_position;
        return false;
    }
    return true;
}


// `rescue_from_slope` moves a unit that stands on a cell that blocks it to the first cell of free
// terrain on the lines +x, -x, +y and -y, at steps of `SLOPE_RESCUE_STEP_CELLS`. It gives up at
// `SLOPE_RESCUE_MAX_CELLS` and leaves the unit where it is.
static void rescue_from_slope(Unit *unit) {
    int terrain_class = unit->unit_type->terrain_class;
    float d_cells = SLOPE_RESCUE_STEP_CELLS;        // rescue distance in cells
    while (d_cells < SLOPE_RESCUE_MAX_CELLS) {
        float d = GC_CELL_METERS * d_cells;
        V2 position = unit->position;
        V2 candidates[SLOPE_RESCUE_DIRECTION_COUNT] = {
            v2(position.x + d, position.y),
            v2(position.x - d, position.y),
            v2(position.x, position.y + d),
            v2(position.x, position.y - d),
        };
        for (int direction_index = 0; direction_index < SLOPE_RESCUE_DIRECTION_COUNT;
             direction_index++) {
            V2 candidate = candidates[direction_index];
            if (gc_get_cell_at(candidate.x, candidate.y) <= terrain_class) {
                unit->position = candidate;
                return;
            }
        }
        d_cells += SLOPE_RESCUE_STEP_CELLS;
    }
}


// `apply_physics` resolves the move of a unit from `old_position` in this tick: the collision
// push, up to three blocking terrain slides, and the slope rescue.
static void apply_physics(int unit_index, V2 old_position) {
    // A failed push leaves the unit at `old_position` for this tick. Without `--parkedpush`, the
    // formation modes never push the units of a squad that is not busy; the game pushes them.
    Unit *unit = &units[unit_index];
    int other_index = find_overlapping_unit(unit_index);
    bool is_parked = !is_squad_busy(&squads[unit->squad_index]);
    bool is_never_pushed = !is_parked_push_on && is_formation_mode() && is_parked;
    bool must_push = other_index != NO_UNIT && !is_never_pushed;
    if (must_push && !push_out(unit_index, &units[other_index], old_position))
        return;

    // When the third slide is still blocked, the unit keeps its position. A slide flips the
    // preferred side of the avoidance turn, at most once per `SIDE_COOLDOWN_SECONDS`.
    V2 new_position = unit->position;
    unit->is_terrain_sliding = false;
    if (!slide_along_terrain(unit, old_position, &new_position)) {
        unit->is_terrain_sliding = true;
        bool is_clear = slide_along_terrain(unit, old_position, &new_position);
        if (!is_clear)
            is_clear = slide_along_terrain(unit, old_position, &new_position);
        if (!is_clear)
            return;
        if (unit->side_cooldown_seconds < 0) {
            unit->side_cooldown_seconds = SIDE_COOLDOWN_SECONDS;
            unit->avoidance_turn_side = !unit->avoidance_turn_side;
        }
        unit->avoid_ticks = AVOID_TICKS;
        unit->position = new_position;
    }

    int cell_value = gc_get_cell_at(unit->position.x, unit->position.y);
    if (cell_value > unit->unit_type->terrain_class)
        rescue_from_slope(unit);
}


// `integrate` moves a unit by its velocity for one tick, within `WORLD_MIN_METERS` and
// `WORLD_MAX_METERS` on both axes, and then resolves the move with `apply_physics`.
void integrate(int unit_index) {
    Unit *unit = &units[unit_index];
    V2 old_position = unit->position;
    unit->position = add(unit->position, mul(unit->velocity, TICK_SECONDS));
    unit->position.x = clamp_float(unit->position.x, WORLD_MIN_METERS, WORLD_MAX_METERS);
    unit->position.y = clamp_float(unit->position.y, WORLD_MIN_METERS, WORLD_MAX_METERS);
    apply_physics(unit_index, old_position);
}
