/* This file reconstructs the motion part of the unit movement of the game itself, from the
 * decompiled gc.exe: Turn, Friction, Integrate and Physics. `tick_mover` in movement.c calls
 * `turn`, `apply_friction` and `integrate` in this order for each unit in every tick, after it
 * found the steering of the unit.
 *
 * Game code: Each function that follows a game function names it and its address in gc.exe
 * 1.0.1.1, whose image always loads at 0x400000. re/PATHFINDING.md describes these functions. The
 * constants are values of the game itself unless their comments say otherwise.
 *
 * Mover fields: `angle`, `heading`, `velocity`, `last_turn_rate` and `is_walled` of `Unit` hold
 * the motion, and `turn`, `apply_friction` and `apply_physics` change them. A wall slide also
 * changes `avoid_ticks`, `swerve_side` and `side_cooldown_seconds`, which Steer and AvoidUnits in
 * movement.c use.
 *
 * Modes: In the formation modes, Physics never pushes the units of parked squads, and a wall
 * slide ignores a hit cell far from its ray. The rest is the same in every mode. */
#include "sim.h"

/* Physics uses these values. A legal cell is a cell that does not block the unit. A wall slide
 * stops the part of a move that goes into a blocked cell, so that the unit slides along the side
 * of that cell. */
#define MIN_PUSH_DISTANCE_SQUARED 1e-5f /* Centres this close, squared, give no push direction. */
#define PUSH_GAP_METERS 0.3f            /* A pushed unit ends with this gap to the other body. */
#define MIN_PUSHED_AWAY_SPEED -0.1f     /* Units moving away slower than this get no push speed. */
#define SLOW_PUSH_SPEED 0.1f            /* A pushed unit slower than this gets `MIN_PUSH_SPEED`. */
#define MIN_PUSH_SPEED 0.2f             /* A slow pushed unit gets this speed away. */
#define WALL_PUSH_CELLS 0.55f           /* A slide stops this many cells off the cell center. */
#define SIDE_COOLDOWN_SECONDS 2.0f      /* Slides flip the swerve side at most once in this. */
#define TELEPORT_DIRECTION_COUNT 4      /* The search for a legal cell tries +x, -x, +y and -y. */
#define TELEPORT_STEP_CELLS 0.5f        /* It steps out by this many cells. */
#define TELEPORT_MAX_CELLS 20.0f        /* The simulator, unlike the game, gives up at this. */
#define HIT_REACH_CELLS 1.0f            /* A hit cell center lies this close to its ray. */
#define WORLD_MIN_METERS 10.0f          /* Integrate keeps each coordinate at least this. */
#define WORLD_MAX_METERS 6390.0f        /* Integrate keeps each coordinate at most this. */
#define NO_UNIT -1                      /* Unit searches return this when they find no unit. */

/* ------------------------------------------------------------------------------------------------
 * `turn` follows Turn (0x511560). It keeps `angle` within 0..2 * PI. It also keeps the rate of
 * the turn in `last_turn_rate`, which the game does not, for the test of units that circle in
 * squad.c.
 * --------------------------------------------------------------------------------------------- */
void turn(Unit *unit, float steering) {
    float clamped_steering = clamp_float(steering, -1, 1);
    unit->last_turn_rate = unit->kind->turn_rate * clamped_steering;
    unit->angle += unit->kind->turn_rate * TICK_SECONDS * clamped_steering;
    if (unit->angle >= 2 * PI)
        unit->angle -= 2 * PI;
    if (unit->angle < 0)
        unit->angle += 2 * PI;
    unit->heading = v2(sinf(unit->angle), cosf(unit->angle));
}

/* ------------------------------------------------------------------------------------------------
 * `apply_friction` follows Friction (0x50fdf0).
 * --------------------------------------------------------------------------------------------- */
void apply_friction(Unit *unit) {
    /* Slow the unit by one tick of `friction` along its motion. When the velocity on an axis
     * then changes sign or becomes 0, that axis stops. */
    float speed = len(unit->velocity);
    if (speed > 0) {
        V2 direction = mul(unit->velocity, 1 / speed);
        V2 new_velocity = sub(unit->velocity, mul(direction, unit->kind->friction * TICK_SECONDS));
        if (new_velocity.x * unit->velocity.x <= 0)
            new_velocity.x = 0;
        if (new_velocity.y * unit->velocity.y <= 0)
            new_velocity.y = 0;
        unit->velocity = new_velocity;
    }

    /* Cap the speed at `max_speed`. */
    float max_speed = unit->kind->max_speed;
    if (length_squared(unit->velocity) > max_speed * max_speed)
        unit->velocity = mul(unit->velocity, max_speed / len(unit->velocity));
}

/* ------------------------------------------------------------------------------------------------
 * `find_overlapping_unit` returns the index of the first unit whose body overlaps the body of
 * unit `unit_index`, or `NO_UNIT`. The game tests its collision map (0x4fecf0) here, and the
 * simulator tests the circles of the units.
 * --------------------------------------------------------------------------------------------- */
static int find_overlapping_unit(int unit_index) {
    const Unit *unit = &units[unit_index];
    for (int other_index = 0; other_index < unit_count; other_index++) {
        if (other_index == unit_index)
            continue;
        const Unit *other = &units[other_index];
        float combined_radius = unit->kind->radius + other->kind->radius;
        float distance_squared = length_squared(sub(other->position, unit->position));
        if (distance_squared < combined_radius * combined_radius)
            return other_index;
    }
    return NO_UNIT;
}

/* ------------------------------------------------------------------------------------------------
 * `get_cell_center_coordinate` returns the world coordinate of the center of the grid column or
 * row `cell_coordinate`.
 * --------------------------------------------------------------------------------------------- */
static float get_cell_center_coordinate(int cell_coordinate) {
    return (cell_coordinate + 0.5f) * GC_CELL_METERS;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_segment_distance` returns the distance from `point` to the segment from `from` to
 * `to`, which can have length 0.
 * --------------------------------------------------------------------------------------------- */
static float measure_segment_distance(V2 point, V2 from, V2 to) {
    V2 segment = sub(to, from);
    float segment_length_squared = length_squared(segment);
    float projection_fraction = 0;
    if (segment_length_squared > 0)
        projection_fraction = dot(sub(point, from), segment) / segment_length_squared;
    float closest_fraction = clamp_float(projection_fraction, 0, 1);
    V2 closest = add(from, mul(segment, closest_fraction));
    return len(sub(point, closest));
}

/* ------------------------------------------------------------------------------------------------
 * `get_wall_coordinate` returns where a wall slide puts the unit on one axis: `WALL_PUSH_CELLS`
 * from `cell_center`, the center of the blocked cell on that axis, on the side that the unit came
 * from. `move` is the move of the unit on that axis in this tick.
 * --------------------------------------------------------------------------------------------- */
static float get_wall_coordinate(float cell_center, float move) {
    float push_distance = GC_CELL_METERS * WALL_PUSH_CELLS;
    if (move <= 0)
        return cell_center + push_distance;
    return cell_center - push_distance;
}

/* ------------------------------------------------------------------------------------------------
 * `slide_along_wall` follows WallSlide (0x511170). It casts the ray of the game from
 * `old_position` to `*new_position` for the mover class of the unit. This sketch shows a ray
 * that enters a blocked cell with center c through a side that runs along x:
 *
 *          +-----------------+
 *          |       c     n   |   blocked cell; n is `*new_position` before the slide
 *          +------------/----+   the ray enters the cell through this side
 *     - - - - - - - - -/- - - -  the slide puts y on this line, `WALL_PUSH_CELLS` cells from c
 *                     /
 *                    o           `old_position`
 *
 * Result: It returns true when the move is clear. Otherwise it returns false and makes the unit
 *   slide: it zeroes the velocity across the side of the blocked cell that the ray hit, and moves
 *   `*new_position` on that axis to `WALL_PUSH_CELLS` from the center of that cell.
 * --------------------------------------------------------------------------------------------- */
static bool slide_along_wall(Unit *unit, V2 old_position, V2 *new_position) {
    V2 from = old_position;
    V2 to = *new_position;
    int mover_class = unit->kind->mover_class;
    int hit_cell_index = gc_cast_ray(from.x, from.y, to.x, to.y, mover_class);
    if (hit_cell_index == GC_RAY_CLEAR)
        return true;

    /* In the formation modes, ignore a hit cell whose center lies further than `HIT_REACH_CELLS`
     * cells from the ray. For a ray of a few millimeters that lies on a cell edge, the ray cast
     * of the game returns the first blocked cell along that whole grid line, up to hundreds of
     * meters away. Without this check, the slide below moves the unit there in one tick. Game
     * mode keeps the behavior of the game itself. */
    float hit_center_x = get_cell_center_coordinate(get_cell_x(hit_cell_index));
    float hit_center_y = get_cell_center_coordinate(get_cell_y(hit_cell_index));
    float hit_reach = HIT_REACH_CELLS * GC_CELL_METERS;
    V2 hit_center = v2(hit_center_x, hit_center_y);
    bool is_hit_off_ray = measure_segment_distance(hit_center, from, to) > hit_reach;
    if (is_formation_mode() && is_hit_off_ray)
        return true;

    /* `gc_get_last_ray_side` describes the ray just cast. When its last grid step went along y,
     * the ray hit a side of the cell that runs along x, so the unit loses its y velocity.
     * Otherwise it loses its x velocity. */
    if (gc_get_last_ray_side() == GC_RAY_STEP_ALONG_Y) {
        unit->velocity.y = 0;
        new_position->y = get_wall_coordinate(hit_center_y, new_position->y - old_position.y);
    } else {
        unit->velocity.x = 0;
        new_position->x = get_wall_coordinate(hit_center_x, new_position->x - old_position.x);
    }
    return false;
}

/* ------------------------------------------------------------------------------------------------
 * `push_out` follows the first step of Physics. It pushes the unit out of `other`, the unit that
 * it overlaps, along the line between their centers (`away`), until the bodies are
 * `PUSH_GAP_METERS` apart:
 *
 *     `other` O----o--------------------->o
 *                  unit                   unit after the push
 *             |<----- `push_distance` --->|
 *
 * Result: It returns true when the unit is clear. When it still overlaps a unit, it goes back to
 *   `old_position`, and `push_out` returns false.
 * Edge case: When the squared distance between the centers is at most
 *   `MIN_PUSH_DISTANCE_SQUARED`, there is no direction to push in. The unit then lands on the
 *   center of the other unit and goes back.
 * --------------------------------------------------------------------------------------------- */
static bool push_out(int unit_index, const Unit *other, V2 old_position) {
    /* Move the unit out. */
    Unit *unit = &units[unit_index];
    V2 offset = sub(unit->position, other->position);
    float distance = len(offset);
    V2 away = v2(0, 0);
    if (distance * distance > MIN_PUSH_DISTANCE_SQUARED)
        away = mul(offset, 1 / distance);
    float combined_radius = other->kind->radius + unit->kind->radius;
    float push_distance = combined_radius + PUSH_GAP_METERS;
    unit->position = add(other->position, mul(away, push_distance));

    /* A unit that moves away at `MIN_PUSHED_AWAY_SPEED` or faster gets its own speed added away
     * from the other unit, or `MIN_PUSH_SPEED` when it is slower than `SLOW_PUSH_SPEED`. A
     * negative `MIN_PUSHED_AWAY_SPEED` lets this include units that move slowly toward the other
     * one. A unit that moves toward it faster only loses its speed toward it. */
    float away_speed = dot(away, unit->velocity);
    if (away_speed >= MIN_PUSHED_AWAY_SPEED) {
        float speed = len(unit->velocity);
        float push_speed = speed < SLOW_PUSH_SPEED ? MIN_PUSH_SPEED : speed;
        unit->velocity = add(unit->velocity, mul(away, push_speed));
    } else {
        unit->velocity = sub(unit->velocity, mul(away, away_speed));
    }

    /* Go back when the unit still overlaps a unit. */
    if (find_overlapping_unit(unit_index) != NO_UNIT) {
        unit->position = old_position;
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `teleport_to_legal_cell` follows the slope rescue (0x511000) of Physics. It moves the unit to
 * the first point in a legal cell that it finds `TELEPORT_STEP_CELLS` from the unit, then twice
 * that, and so on, trying +x, -x, +y and -y at each distance.
 * Edge case: When no legal cell lies closer than `TELEPORT_MAX_CELLS`, the unit stays where it
 *   is.
 * --------------------------------------------------------------------------------------------- */
static void teleport_to_legal_cell(Unit *unit) {
    int mover_class = unit->kind->mover_class;
    float distance_cells = TELEPORT_STEP_CELLS;
    while (distance_cells < TELEPORT_MAX_CELLS) {
        /* `candidates` holds the points +x, -x, +y and -y of the unit, in this order. */
        float distance_meters = GC_CELL_METERS * distance_cells;
        V2 position = unit->position;
        V2 candidates[TELEPORT_DIRECTION_COUNT] = {
            v2(position.x + distance_meters, position.y),
            v2(position.x - distance_meters, position.y),
            v2(position.x, position.y + distance_meters),
            v2(position.x, position.y - distance_meters),
        };
        for (int i = 0; i < TELEPORT_DIRECTION_COUNT; i++) {
            V2 candidate = candidates[i];
            if (gc_get_cell_at(candidate.x, candidate.y) <= mover_class) {
                unit->position = candidate;
                return;
            }
        }
        distance_cells += TELEPORT_STEP_CELLS;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `apply_physics` follows Physics (0x510b20), after `integrate` moved the unit from
 * `old_position`. It resolves collisions with other units, then with blocked cells, then with
 * ground that is too steep.
 * --------------------------------------------------------------------------------------------- */
static void apply_physics(int unit_index, V2 old_position) {
    /* Push the unit out of the first unit that it overlaps. In formation mode, a unit of a parked
     * squad is never pushed. When the push fails, the unit is back at `old_position`, and the
     * function skips the other steps. */
    Unit *unit = &units[unit_index];
    int other_index = find_overlapping_unit(unit_index);
    bool is_never_pushed = is_formation_mode() && !is_squad_busy(&squads[unit->squad_index]);
    bool must_push = other_index != NO_UNIT && !is_never_pushed;
    if (must_push && !push_out(unit_index, &units[other_index], old_position))
        return;

    /* Slide along blocked cells. `slide_along_wall` checks the move, and when it slides, the next
     * call checks the slid move, up to three checks in all. When the first check finds a blocked
     * cell, `is_walled` marks the tick for the metrics. When a later check finds the slid move
     * clear, the unit takes the slid position, `avoid_ticks` restarts, and the swerve side flips
     * at most once per `SIDE_COOLDOWN_SECONDS`. `!` flips it, because `swerve_side` holds 0
     * (`SWERVE_LEFT` in movement.c) or 1 (`SWERVE_RIGHT`). When all three checks find a blocked
     * cell, the unit keeps the position that `integrate` gave it, and the function skips the
     * steep-ground step, as in the game. */
    V2 new_position = unit->position;
    unit->is_walled = false;
    if (!slide_along_wall(unit, old_position, &new_position)) {
        unit->is_walled = true;
        bool is_clear = slide_along_wall(unit, old_position, &new_position);
        if (!is_clear)
            is_clear = slide_along_wall(unit, old_position, &new_position);
        if (!is_clear)
            return;
        if (unit->side_cooldown_seconds < 0) {
            unit->side_cooldown_seconds = SIDE_COOLDOWN_SECONDS;
            unit->swerve_side = !unit->swerve_side;
        }
        unit->avoid_ticks = AVOID_TICKS;
        unit->position = new_position;
    }

    /* Move the unit off ground that is too steep for it. */
    int cell_value = gc_get_cell_at(unit->position.x, unit->position.y);
    if (cell_value > unit->kind->mover_class)
        teleport_to_legal_cell(unit);
}

/* ------------------------------------------------------------------------------------------------
 * `integrate` follows Integrate (0x5109e0), and then runs Physics with `apply_physics`.
 * --------------------------------------------------------------------------------------------- */
void integrate(int unit_index) {
    Unit *unit = &units[unit_index];
    V2 old_position = unit->position;
    unit->position = add(unit->position, mul(unit->velocity, TICK_SECONDS));
    unit->position.x = clamp_float(unit->position.x, WORLD_MIN_METERS, WORLD_MAX_METERS);
    unit->position.y = clamp_float(unit->position.y, WORLD_MIN_METERS, WORLD_MAX_METERS);
    apply_physics(unit_index, old_position);
}
