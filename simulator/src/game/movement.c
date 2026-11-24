// Rebuilt unit movement of the base game (docs/PATHFINDING.md): the waypoint following of
// `Mover::Update` (0x50eec0), the throttle and the steering of *Steer* (0x50ff40), and the base
// game avoidance *AvoidUnits* (0x5101a0). `tick_mover` runs once per unit and tick, as *Tick*
// (0x50e5b0) does: the steering, then *Turn*, *Friction* and *Integrate* (physics.c). In the
// formation modes, a squad member of a squad that is not idle steers with `steer_in_formation`
// (steering.c) instead.
//
// In the formation modes, a line of sight also fails when it passes within
// `SIGHT_TOUCH_MARGIN_METERS` of a unit of a parked squad, so that routes go around parked squads. The game can use a line of sight to decide whether a unit fires, so this change could
// affect gameplay.
//
// The reach of a unit is the distance at which it counts its current waypoint as reached, and its
// close range is the distance below which *Steer* treats a waypoint as close.
#include <string.h>

#include "sim.h"
#include "formation.h"

#define SIGHT_TOUCH_MARGIN_METERS 0.5f                 // gap of a line of sight to parked units
#define SET_MOVE_SEARCH_BUDGET 200                     // expansion budget of a move search
#define REPATH_SEARCH_BUDGET 100                       // expansion budget of a waypoint search
#define WAYPOINT_TOLERANCE_METERS 1.0f                 // the search of a move ends this close
#define LOOK_AHEAD_PHASES 3                            // look-ahead phases per visibility check
#define REACH_EPSILON_METERS 0.1f                      // reach margin that skips a search

#define ARRIVAL_STOP_DISTANCE_METERS 3.0f              // planned stop short of the waypoint
#define BRAKING_START_FRACTION 0.5f                    // friction share at which braking starts
#define WAYPOINT_BEHIND_METERS -0.2f                   // offset along the heading for behind
#define BEHIND_THROTTLE 0.5f                           // throttle toward a waypoint behind
#define MIN_THROTTLE 0.6f                              // smallest throttle toward a waypoint
#define REORIENT_DONE_COSINE 0.95f                     // a turn on the spot ends within 18 degrees

#define QUERY_RADIUS_FACTOR 1.1f                       // radius widening for corridor queries
#define QUERY_RADIUS_MARGIN_FACTOR 1.25f               // further widening by the avoidance
#define LAST_WAYPOINT_TURN_COSINE 0.7f                 // turn limit toward the last waypoint
#define PARKING_QUERY_SHORTFALL_METERS 1.0f            // that check ends this short of it
#define LOOK_DISTANCE_FACTOR 0.6f                      // corridor share of the waypoint distance
#define MAX_LOOK_RADII 4.0f                            // longest corridor, in unit radii
#define MIN_LOOK_RADII 1.5f                            // shortest corridor, in unit radii
#define BEHIND_LOOK_FACTOR 0.5f                        // corridor share when facing away
#define MAX_BLOCKER_TANGENT_SQUARED 100000.0f          // farthest unit in the way, squared
#define NEW_BLOCKER_RADII 2.4f                         // blocker spacing that picks a new side
#define DRIVING_ON_AVOID_TICKS 2                       // avoidance of a blocker that drives on
#define AVOIDANCE_TURN_STEERING 4.0f                   // steering bias of an avoidance turn
#define WAIT_SECONDS 2.0f                              // wait behind a faster unit in the way
#define AVOIDANCE_TURN_AFTER_WAIT_SECONDS -2.0f        // wait timer from which a unit turns
#define AVOIDANCE_TURN_LEFT 0                          // avoidance turn side: left
#define AVOIDANCE_TURN_RIGHT 1                         // avoidance turn side: right


// `pop_waypoint` removes the first waypoint from the waypoint list of a unit and returns it.
static V2 pop_waypoint(Unit *unit) {
    V2 next = unit->waypoints[0];
    size_t moved_bytes = (size_t)(unit->waypoint_count - 1) * sizeof(V2);
    memmove(&unit->waypoints[0], &unit->waypoints[1], moved_bytes);
    unit->waypoint_count--;
    return next;
}


// `update_waypoint_state` marks whether the current waypoint of a unit is its last one, and sets
// the reach that belongs to it.
static void update_waypoint_state(Unit *unit) {
    unit->is_last_waypoint = unit->waypoint_count == 0;
    unit->reach = unit->is_last_waypoint ? unit->reach_last : unit->reach_mid;
}


// `is_in_sight` tells whether a unit sees `target`: the zero-width ray of the game reaches it,
// and in the formation modes, the line keeps `SIGHT_TOUCH_MARGIN_METERS` from the bodies of the
// units of parked squads.
static bool is_in_sight(const Unit *unit, V2 target) {
    V2 from = unit->position;
    int terrain_class = unit->unit_type->terrain_class;
    if (gc_cast_ray(from.x, from.y, target.x, target.y, terrain_class) != GC_RAY_CLEAR)
        return false;
    if (!is_formation_mode())
        return true;

    // The test reads the point of the line closest to each parked unit.
    V2 line = sub(target, from);
    float l_squared = length_squared(line);        // squared line length
    for (int other_index = 0; other_index < unit_count; other_index++) {
        const Unit *other = &units[other_index];
        bool is_same_squad = other->squad_index == unit->squad_index;
        bool is_skipped = is_same_squad || other->is_carried;
        if (is_skipped || is_squad_busy(&squads[other->squad_index]))
            continue;
        float t = 0;                               // fraction of the closest point
        if (l_squared > 0) {
            float proj = dot(sub(other->position, from), line);
            t = clamp_float(proj / l_squared, 0, 1);
        }
        V2 closest = add(from, mul(line, t));
        float r = other->unit_type->radius + unit->unit_type->radius;        // combined radius
        float d_touch = r + SIGHT_TOUCH_MARGIN_METERS;
        if (length_squared(sub(other->position, closest)) < d_touch * d_touch)
            return false;
    }
    return true;
}


// `find_path` searches a path of a unit to `destination` and puts it in front of the remaining
// waypoints, as `Mover::FindPath` (0x50eb60) does. Its first point becomes the current waypoint,
// and the unit turns on the spot toward it. It returns false when the search fails.
static bool find_path(Unit *unit, V2 destination, float tolerance, int budget) {
    GcRequest search;
    bool is_found = search_path(unit, destination, tolerance, budget, &search);
    if (is_found) {
        // The path takes at most the room left in the waypoint list.
        int room = MAX_UNIT_WAYPOINTS - unit->waypoint_count;
        int count = gc_get_path(&search, path_points, room);        // number of path points
        size_t kept_bytes = (size_t)unit->waypoint_count * sizeof(V2);
        memmove(&unit->waypoints[count], &unit->waypoints[0], kept_bytes);
        for (int point_index = 0; point_index < count; point_index++)
            unit->waypoints[point_index] = get_pair_point(path_points, point_index);
        unit->waypoint_count += count;
        unit->is_reorienting = true;
        unit->waypoint = pop_waypoint(unit);
        update_waypoint_state(unit);
    }
    gc_free_request(&search);
    return is_found;
}


// `set_move` gives a unit the route of its squad, as `Mover::SetMove` (0x50e7e0) does: the unit
// searches only from itself to the second route point and reuses the rest of the route. The last
// route point becomes its final parking position, and the way there is not searched.
void set_move(Unit *unit, const V2 *route, int route_count, V2 facing, V2 parking_position) {
    // The current waypoint skips the first route point when more points remain.
    unit->blocker_index = NO_BLOCKER;
    unit->facing = facing;
    memcpy(&unit->waypoints[0], &route[0], (size_t)route_count * sizeof(V2));
    unit->waypoint_count = route_count;
    unit->waypoint = pop_waypoint(unit);
    if (unit->waypoint_count > 0)
        unit->waypoint = pop_waypoint(unit);

    // A failed search leaves the unit where it stands.
    float tolerance = unit->waypoint_count == 0 ? unit->reach_last : WAYPOINT_TOLERANCE_METERS;
    if (!find_path(unit, unit->waypoint, tolerance, SET_MOVE_SEARCH_BUDGET)) {
        unit->waypoint_count = 0;
        unit->waypoint = unit->position;
    } else if (unit->waypoint_count == 0) {
        unit->waypoint = parking_position;
    } else {
        unit->waypoints[unit->waypoint_count - 1] = parking_position;
    }

    update_waypoint_state(unit);
    unit->is_stopped = false;
    unit->is_reorienting = true;
    unit->wait_seconds = WAIT_TIMER_AT_ORDER_SECONDS;
}


// `stop_at` sends a unit straight to `point` and clears its other waypoints.
void stop_at(Unit *unit, V2 point) {
    unit->waypoint = point;
    unit->is_stopped = false;
    unit->reach = unit->reach_last;
    unit->is_last_waypoint = false;
    unit->blocker_index = NO_BLOCKER;
    unit->waypoint_count = 0;
}


// `get_unreached_distance_squared` returns a squared distance just beyond the reach of a unit, so
// that a waypoint that has just changed does not count as reached in the same tick.
static float get_unreached_distance_squared(const Unit *unit) {
    return unit->reach * unit->reach + 1;
}


// `look_ahead` skips to the waypoint `ahead_index` places after the next one when the unit sees
// it. `distance_squared` then becomes unreached.
static void look_ahead(Unit *unit, int ahead_index, float *distance_squared) {
    bool has_waypoint = unit->waypoint_count > ahead_index;
    if (has_waypoint && is_in_sight(unit, unit->waypoints[ahead_index])) {
        for (int skip_index = 0; skip_index < ahead_index; skip_index++)
            pop_waypoint(unit);
        unit->waypoint = pop_waypoint(unit);
        update_waypoint_state(unit);
        *distance_squared = get_unreached_distance_squared(unit);
    }
}


// `repath_if_hidden` searches a new path to the current waypoint of a unit when the unit does not
// see it. A unit on its last waypoint whose reach is not above the last reach plus
// `REACH_EPSILON_METERS` skips the search.
static void repath_if_hidden(Unit *unit, float *distance_squared) {
    // The search goes to the next waypoint, when there is one, instead of the hidden one.
    float skip_reach = unit->reach_last + REACH_EPSILON_METERS;
    bool is_skipped = unit->waypoint_count == 0 && !(skip_reach > unit->reach);
    if (is_skipped || is_in_sight(unit, unit->waypoint))
        return;
    if (unit->waypoint_count > 0)
        unit->waypoint = pop_waypoint(unit);
    find_path(unit, unit->waypoint, 0, REPATH_SEARCH_BUDGET);
    *distance_squared = get_unreached_distance_squared(unit);
}


// `update_mover` follows the waypoints of a unit for one tick, as `Mover::Update` (0x50eec0)
// does. A phase counter makes each check run every fourth tick: three look-ahead checks, then a
// visibility check of the current waypoint.
static void update_mover(Unit *unit) {
    float d_squared = length_squared(sub(unit->waypoint, unit->position));
    if (!unit->is_stopped) {
        int phase = unit->look_ahead_phase;
        unit->look_ahead_phase++;
        if (phase < LOOK_AHEAD_PHASES) {
            look_ahead(unit, phase, &d_squared);
        } else {
            unit->look_ahead_phase = 0;
            repath_if_hidden(unit, &d_squared);
        }
    }

    // Within reach, the unit takes the next waypoint only when it sees it, and it stops on the
    // last waypoint.
    if (d_squared <= unit->reach * unit->reach) {
        if (unit->waypoint_count == 0) {
            unit->is_stopped = true;
        } else if (is_in_sight(unit, unit->waypoints[0])) {
            unit->waypoint = pop_waypoint(unit);
            update_waypoint_state(unit);
        }
    }
}


// `compute_arrival_factor` returns the arrival factor 0x511280 of a unit: 1 before the last
// waypoint, and toward the last waypoint, a throttle that brakes the unit to a stop
// `ARRIVAL_STOP_DISTANCE_METERS` short of it.
static float compute_arrival_factor(const Unit *unit) {
    // `k` compares the braking that the stop needs with the friction alone.
    if (!unit->is_last_waypoint)
        return unit->is_stopped ? 0.0f : 1.0f;
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float d = len(to_waypoint);                                    // distance to the waypoint
    if (d < unit->reach)
        return 0;
    float v = dot(mul(to_waypoint, 1 / d), unit->velocity);        // speed toward the waypoint
    float d_stop = d - ARRIVAL_STOP_DISTANCE_METERS;
    float decel = v * v / (2 * d_stop);                            // needed deceleration
    float k = decel / unit->unit_type->friction;
    if (k < BRAKING_START_FRACTION)
        return 1;

    // The throttle falls from the share that balances the friction down to zero.
    float excess = 2 * (k - BRAKING_START_FRACTION);
    if (excess < 1)
        return (1 - excess) * unit->unit_type->friction / unit->acceleration;
    return 0;
}


// `compute_game_throttle` returns the throttle of *Steer*: half throttle toward a far waypoint
// behind the unit while it does not avoid, and otherwise the arrival factor, but at least
// `MIN_THROTTLE`.
static float compute_game_throttle(const Unit *unit) {
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float close_squared = unit->unit_type->close_range * unit->unit_type->close_range;
    bool is_avoiding = unit->avoid_ticks >= 1;
    bool is_behind = dot(unit->heading, to_waypoint) < WAYPOINT_BEHIND_METERS;
    bool is_far = close_squared < length_squared(to_waypoint);
    if (!is_avoiding && is_behind && is_far)
        return BEHIND_THROTTLE;

    float throttle = compute_arrival_factor(unit);
    return throttle <= MIN_THROTTLE ? MIN_THROTTLE : throttle;
}


// `compute_game_steering` returns the steering of *Steer* toward the current waypoint. A unit that
// steers freely gives no steering for a close waypoint that does not lie ahead of its velocity.
static float compute_game_steering(const Unit *unit) {
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float d_squared = length_squared(to_waypoint);
    float close_squared = unit->unit_type->close_range * unit->unit_type->close_range;
    bool is_steering_freely = !unit->is_reorienting && unit->avoid_ticks < 1;
    bool is_close = d_squared <= close_squared;
    bool is_not_ahead = dot(to_waypoint, unit->velocity) <= 0;
    if (is_steering_freely && is_close && is_not_ahead)
        return 0;
    if (d_squared > 0)
        return steer_to(unit, to_waypoint);
    return 0;
}


// `end_reorient` ends the turn on the spot of a unit once it faces within about 18 degrees of its
// current waypoint.
static void end_reorient(Unit *unit) {
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float d_squared = length_squared(to_waypoint);
    if (d_squared > 0) {
        V2 direction = mul(to_waypoint, 1 / sqrtf(d_squared));
        if (dot(unit->heading, direction) > REORIENT_DONE_COSINE)
            unit->is_reorienting = false;
    }
}


// `steer` runs *Steer* (0x50ff40) for one tick: it sets the throttle and returns the steering
// input. A waiting unit steers to its saved heading, and neither a waiting unit nor a unit that
// turns on the spot accelerates, so the friction stops it.
static float steer(Unit *unit) {
    unit->wait_seconds -= TICK_SECONDS;
    if (unit->wait_seconds > 0)
        return steer_to(unit, unit->wait_heading);
    if (!unit->is_reorienting)
        accelerate(unit, compute_game_throttle(unit));
    else
        end_reorient(unit);
    return compute_game_steering(unit);
}


// `are_game_friends` tells whether the base game avoidance treats two units as friends. The game
// compares the int16 at entity +0x3e (0x51039a, 0x5105f2, 0x51084b), which the units of one squad
// share and which differs between squads, also between the squads of one player (mod log,
// MISSION4). Without `--squadfriends`, the units of one team count as friends.
static bool are_game_friends(const Unit *unit, const Unit *other) {
    if (is_squad_friend_on)
        return unit->squad_index == other->squad_index;
    return unit->team == other->team;
}


// `query_corridor` collects in `hit_indices` the units ahead of a unit along `direction` whose
// near edge lies within `length`, in a corridor of the widened radius of the unit plus their own
// radius to each side. It returns their number. It replaces the collision map query of the game.
static int query_corridor(int unit_index, V2 direction, float length, int *hit_indices) {
    const Unit *unit = &units[unit_index];
    float radius = unit->unit_type->radius;
    float r_query = radius * QUERY_RADIUS_FACTOR * QUERY_RADIUS_MARGIN_FACTOR;
    int count = 0;        // number of hits
    for (int other_index = 0; other_index < unit_count; other_index++) {
        if (other_index == unit_index || units[other_index].is_carried)
            continue;

        // `ahead` is the distance of the other unit along `direction`, and `near_edge` that of
        // the point of its circle nearest to the unit.
        const Unit *other = &units[other_index];
        V2 to_other = sub(other->position, unit->position);
        float ahead = dot(to_other, direction);
        if (ahead <= 0)
            continue;
        V2 closest = add(unit->position, mul(direction, ahead));
        float side_squared = length_squared(sub(other->position, closest));
        float half_width = r_query + other->unit_type->radius;
        float r_squared = other->unit_type->radius * other->unit_type->radius;
        float near_edge = ahead - sqrtf(r_squared + side_squared);
        bool is_within_width = side_squared <= half_width * half_width;
        if (is_within_width && near_edge <= length) {
            hit_indices[count] = other_index;
            count++;
        }
    }
    return count;
}


// `turn_at_last_waypoint` makes a unit turn on the spot toward its last waypoint when that lies
// more than about 45.6 degrees off its heading, unless an enemy unit stands in the corridor to it,
// off the waypoint and apart from the unit. It returns true when the unit turns.
static bool turn_at_last_waypoint(int unit_index, V2 direction, float distance) {
    Unit *unit = &units[unit_index];
    if (!unit->is_last_waypoint || dot(direction, unit->heading) >= LAST_WAYPOINT_TURN_COSINE)
        return false;

    int hit_indices[MAX_UNITS];
    float query_length = distance - PARKING_QUERY_SHORTFALL_METERS;
    int count = query_corridor(unit_index, direction, query_length, hit_indices);
    for (int hit_index = 0; hit_index < count; hit_index++) {
        const Unit *other = &units[hit_indices[hit_index]];
        float r = unit->unit_type->radius + other->unit_type->radius;        // combined radius
        float r_squared = other->unit_type->radius * other->unit_type->radius;
        float d_parking_squared = length_squared(sub(other->position, unit->waypoint));
        float d_squared = length_squared(sub(other->position, unit->position));
        bool is_enemy = !are_game_friends(unit, other);
        bool is_off_parking_position = d_parking_squared - r_squared > 0;
        bool is_apart = d_squared - r * r > 0;
        if (is_enemy && is_off_parking_position && is_apart)
            return false;
    }
    unit->is_reorienting = true;
    unit->blocker_index = NO_BLOCKER;
    return true;
}


// `find_blocker` returns the unit in the way among the hits of a corridor: the one with the
// shortest tangent from the unit, below `MAX_BLOCKER_TANGENT_SQUARED`. Toward the last waypoint,
// an enemy unit that stands on the waypoint does not count.
static int find_blocker(const Unit *unit, const int *hit_indices, int hit_count) {
    // The tangent length from a point to a circle is `sqrt(d^2 - r^2)`.
    int best_index = NO_BLOCKER;
    float best_squared = MAX_BLOCKER_TANGENT_SQUARED;        // shortest squared tangent
    for (int hit_index = 0; hit_index < hit_count; hit_index++) {
        int other_index = hit_indices[hit_index];
        const Unit *other = &units[other_index];
        float r_squared = other->unit_type->radius * other->unit_type->radius;
        float d_squared = length_squared(sub(other->position, unit->position));
        float tangent_squared = d_squared - r_squared;
        float d_parking_squared = length_squared(sub(other->position, unit->waypoint));
        bool is_friend = are_game_friends(unit, other);
        bool is_off_parking_position = d_parking_squared - r_squared > 0;
        bool is_relevant = !unit->is_last_waypoint || is_friend || is_off_parking_position;
        bool is_closer = tangent_squared < best_squared && tangent_squared >= 0;
        if (is_relevant && is_closer) {
            best_index = other_index;
            best_squared = tangent_squared;
        }
    }
    return best_index;
}


// `end_wait` ends the wait of a unit behind a unit in its way.
static void end_wait(Unit *unit) {
    if (unit->wait_seconds > 0)
        unit->wait_seconds = 0;
}


// `get_look_distance` returns the length of the corridor that a unit checks for units in its way:
// `LOOK_DISTANCE_FACTOR` of the distance to its waypoint, at most `MAX_LOOK_RADII` radii, halved
// when the unit faces away from the waypoint, and at least `MIN_LOOK_RADII` radii.
static float get_look_distance(const Unit *unit, V2 direction, float distance) {
    float d = distance * LOOK_DISTANCE_FACTOR;        // look distance
    float d_max = unit->unit_type->radius * MAX_LOOK_RADII;
    if (d > d_max)
        d = d_max;
    if (dot(direction, unit->heading) < 0)
        d *= BEHIND_LOOK_FACTOR;
    float d_min = unit->unit_type->radius * MIN_LOOK_RADII;
    if (d < d_min)
        d = d_min;
    return d;
}


// `switch_blocker` makes `blocker_index` the unit in the way of a unit and ends its wait. It
// returns true when the unit picks a new side for its avoidance turn: for its first blocker, or
// for a blocker far enough from the previous one.
static bool switch_blocker(Unit *unit, int blocker_index) {
    if (blocker_index == unit->blocker_index)
        return false;
    end_wait(unit);

    // Two blockers closer than `NEW_BLOCKER_RADII` radii of the unit plus their radii count as
    // one obstacle, so the side stays.
    bool is_new_side = true;
    if (unit->blocker_index >= 0) {
        const Unit *previous = &units[unit->blocker_index];
        const Unit *blocker = &units[blocker_index];
        float own_spacing = unit->unit_type->radius * NEW_BLOCKER_RADII;
        float previous_radius = previous->unit_type->radius;
        float spacing = own_spacing + previous_radius + blocker->unit_type->radius;
        V2 between = sub(previous->position, blocker->position);
        float d_squared = length_squared(between);
        is_new_side = spacing * spacing <= d_squared;
    }
    unit->blocker_index = blocker_index;
    return is_new_side;
}


// `make_avoidance_turn` returns the steering bias of an avoidance turn around `blocker`: full lock
// to one side after the clamp in `turn`, or zero while the unit waits. The unit copies the side of
// a blocker that avoids it.
static float make_avoidance_turn(int unit_index, const Unit *blocker, bool is_new_side) {
    // A new side turns away from the blocker: left for a blocker on the right or straight ahead.
    Unit *unit = &units[unit_index];
    if (is_new_side) {
        V2 to_blocker = sub(blocker->position, unit->position);
        bool is_blocker_right = to_blocker.y * unit->heading.x <= to_blocker.x * unit->heading.y;
        unit->avoidance_turn_side = is_blocker_right ? AVOIDANCE_TURN_LEFT : AVOIDANCE_TURN_RIGHT;
    }
    if (blocker->blocker_index == unit_index)
        unit->avoidance_turn_side = blocker->avoidance_turn_side;

    unit->avoid_ticks = AVOID_TICKS;
    if (unit->wait_seconds <= 0) {
        unit->avoidance_turn_seconds += TICK_SECONDS;
        unit->is_avoidance_turning = true;
        bool is_left = unit->avoidance_turn_side == AVOIDANCE_TURN_LEFT;
        return is_left ? -AVOIDANCE_TURN_STEERING : AVOIDANCE_TURN_STEERING;
    }
    return 0;
}


// `avoid_units` runs the base game avoidance *AvoidUnits* (0x5101a0) of a unit and returns its
// steering bias. The unit keeps driving into a friendly blocker that drives on, waits behind a
// faster one, and otherwise makes an avoidance turn.
static float avoid_units(int unit_index) {
    // A unit that turns on the spot does not avoid.
    Unit *unit = &units[unit_index];
    unit->side_cooldown_seconds -= TICK_SECONDS;
    if (unit->is_reorienting)
        return 0;
    V2 to_waypoint = sub(unit->waypoint, unit->position);
    float d = len(to_waypoint);        // distance to the waypoint
    V2 direction = v2(0, 0);
    if (d > 0)
        direction = mul(to_waypoint, 1 / d);
    if (turn_at_last_waypoint(unit_index, direction, d))
        return 0;

    // The corridor runs along the heading of the unit.
    int hit_indices[MAX_UNITS];
    float look = get_look_distance(unit, direction, d);
    int count = query_corridor(unit_index, unit->heading, look, hit_indices);        // hits
    if (count == 0) {
        unit->blocker_index = NO_BLOCKER;
        end_wait(unit);
        return 0;
    }
    int blocker_index = find_blocker(unit, hit_indices, count);
    if (blocker_index == NO_BLOCKER) {
        end_wait(unit);
        return 0;
    }
    bool is_new_side = switch_blocker(unit, blocker_index);

    // A friendly blocker drives on when it moves, does not wait, and has avoided for at most
    // `DRIVING_ON_AVOID_TICKS`.
    const Unit *blocker = &units[blocker_index];
    bool is_friend = are_game_friends(unit, blocker);
    bool is_blocker_moving = !blocker->is_stopped && blocker->wait_seconds <= 0;
    bool is_driving_on = is_blocker_moving && blocker->avoid_ticks <= DRIVING_ON_AVOID_TICKS;
    if (is_friend && is_driving_on)
        return 0;

    // A unit that waited recently turns instead of waiting again.
    bool has_waited_recently = unit->wait_seconds >= AVOIDANCE_TURN_AFTER_WAIT_SECONDS;
    bool is_blocker_not_faster = blocker->top_speed <= unit->top_speed;
    if (has_waited_recently || blocker->is_stopped || is_blocker_not_faster)
        return make_avoidance_turn(unit_index, blocker, is_new_side);
    unit->wait_seconds = WAIT_SECONDS;
    unit->wait_heading = unit->heading;
    unit->wait_count++;
    return 0;
}


// `steer_unit` returns the steering input of a unit for one tick. A stopped unit of a moving squad
// steps aside for its squad members in the formation modes, and any other stopped unit turns to
// its final facing.
static float steer_unit(int unit_index, bool is_forming) {
    Unit *unit = &units[unit_index];
    if (!unit->is_stopped) {
        if (is_forming)
            return steer_in_formation(unit_index);
        float steering = steer(unit);
        return steering + avoid_units(unit_index);
    }

    int squad_state = squads[unit->squad_index].state;
    if (is_formation_mode() && squad_state == SQUAD_MOVING)
        return step_aside_for_members(unit_index);
    return steer_to_facing(unit);
}


// `tick_mover` moves a unit for one tick: waypoint following outside formation following, then
// steering, turning, friction and integration.
void tick_mover(int unit_index) {
    Unit *unit = &units[unit_index];
    unit->is_avoidance_turning = false;
    unit->is_orca_avoiding = false;
    unit->avoid_ticks--;
    if (unit->avoid_ticks < 0)
        unit->avoid_ticks = 0;

    // A unit that boards an APC in this tick rides inside it from now on.
    if (is_chasing(unit))
        board_if_close(unit);
    if (unit->is_carried) {
        ride_in_carrier(unit);
        return;
    }

    // Formation following replaces the waypoint following for squads that are not idle, and a
    // unit that drives to board an APC aims at it instead.
    const Squad *squad = &squads[unit->squad_index];
    bool is_forming = is_formation_mode() && squad->state != SQUAD_IDLE;
    if (is_chasing(unit))
        aim_at_target(unit);
    else if (!is_forming)
        update_mover(unit);
    float steering = steer_unit(unit_index, is_forming);
    turn(unit, steering);
    apply_friction(unit);
    integrate(unit_index);
}
