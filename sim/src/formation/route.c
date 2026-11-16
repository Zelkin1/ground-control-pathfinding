/* route.c builds the squad route of formation following and reads points and directions from it.
 *
 * Callers: formation.c calls `move_route_start`, `shorten_route`, `take_shorter_center_route` and
 *   `smooth_route` from `set_up_formation` when a squad gets its route. lanes.c, formation.c and
 *   steering.c read the route with `get_route_point`, `get_extended_route_point` and
 *   `get_formation_direction`. lanes.c also measures the free ground with `measure_free_distance`.
 *
 * Why smooth: The routes of the game turn sharply at the corners of obstacles, so a unit drives
 *   straight at a corner and must nearly stop to turn there. The lanes also bulge around each
 *   obstacle. A smooth curve avoids both.
 *
 * Smoothing: `smooth_route` replaces the route with a cubic B-spline (a smooth curve whose
 *   direction and curvature change continuously). Control points, about every
 *   `CONTROL_SPACING_METERS` along the route, shape the curve. Straight stretches stay straight,
 *   and each corner turns into a curve about 30-50 m long. The curve is sampled at points about
 *   `CURVE_SAMPLE_SPACING_METERS` apart. A sample has a given room when the circle of that radius
 *   around it is free of blocked cells. Where a sample has less than the wanted room, a push
 *   moves the control points that shape that part of the curve away from the obstacle. The pushes
 *   go on in rounds, at most `SMOOTHING_ROUNDS`, until the curve has the room everywhere. The
 *   lanes then stay parallel to one smooth curve.
 *
 * Wide turns: A unit drives a bend at full speed only when the radius of the bend is at least its
 *   top speed over its turn rate, with a margin (steering.c slows units before sharper bends).
 *   With `--wideturns on`, `widen_curve` widens the bends of the curve that are sharper than that
 *   for the inner lanes of the squad. At each control point A where the bend is too sharp, it
 *   moves A into the bend and its neighbors B and C out of it, so that the curve swings out before
 *   the bend, cuts in at its apex and swings out after it. Then the pushes restore the wanted
 *   room, as for the first curve. A wider curve replaces the first one only if it has room for the
 *   body everywhere, lacks the wanted room at no more samples than the first one, and has a
 *   smaller total excess over the wanted bends (`measure_bend_excess`).
 *
 *
 *                 C
 *                 o          s = (B - A) + (C - A) points into the bend at A.
 *                 |          A moves along s to A', and B and C move against s,
 *          A' +   |          so the turn spreads over more of the route.
 *               \ |
 *    B o--------o A
 */
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "sim.h"
#include "formation.h"

/* These values measure the route and the ground beside it. */
#define ROUTE_EPSILON_METERS 1e-3f     /* Route segments and lines this short count as points. */
#define MIN_CHORD_LENGTH_SQUARED 1.0f  /* A direction chord needs a squared length above this. */
#define FREE_DISTANCE_STEP_METERS 0.5f /* Free distances are measured in steps this long. */
#define CELL_SKIP_MARGIN_METERS 0.01   /* Skipped probes stay this far inside their free cell. */
#define CORNER_CUT_STEPS 6             /* Corner cuts halve their search this many times. */
#define CORNER_CUT_MARGIN_METERS 8.0f  /* The band of a cut leg is this much wider each side. */

/* These values limit the second search from the center of the squad, and the smoothing of
 * replanned routes. A replanned route comes from a search again around squads that parked on the
 * route. */
#define CENTER_SEARCH_BUDGET 500 /* The search can use the whole cap of the game on expansions. */
#define CENTER_SEARCH_TOLERANCE_METERS 0 /* The searches must reach their targets exactly. */
#define CENTER_ROUTE_MIN_GAIN_METERS 10.0f /* The route of the center must be this much shorter. */
#define REPLAN_MAX_STRETCH 1.25f /* Smoothing stretches replanned routes by this factor at most. */

/* This value shapes the direction of the formation. */
#define FORMATION_DIRECTION_SPAN_METERS 10.0f /* Directions span this much route each way. */

/* These values shape the smooth route. */
#define CONTROL_SPACING_METERS 16.0f           /* Control points are at least this far apart. */
#define CONTROL_SPARE_POINTS 8                 /* `control` has this room past `MAX_WAYPOINTS`. */
#define CURVE_SAMPLE_SPACING_METERS 4.0f       /* Curve samples are at least this far apart. */
#define MAX_FULL_ROUTE_METERS 8000.0f          /* Longer routes get wider spacings to fit. */
#define CURVE_PIECE_POINTS 4                   /* Each curve piece has this many control points. */
#define START_REPEATS 3                        /* The first point goes in this many times. */
#define END_REPEATS 2                          /* The last point goes in this many more times. */
#define FIXED_END_POINTS 3                     /* This many points at each end never move. */
#define SMOOTHING_ROUNDS 60                    /* Pushes run for at most this many rounds. */
#define BODY_MARGIN_METERS 0.5f                /* Curves keep bodies this far off blocked cells. */
#define PASSAGE_MIDDLE_MARGIN_METERS 0.25f     /* Narrow passages want half width less this. */
#define AWAY_SEARCH_MARGIN_METERS 1.0f         /* Obstacles are sought this far past the room. */
#define PUSH_STEP_METERS 0.25f                 /* A push moves the closest point this far. */
#define PUSH_SPREAD_METERS 64.0f               /* Pushes reach this far each way from the spot. */
#define PUSH_TAPER_METERS 80.0f                /* Pushes fade out over this distance. */
#define WIDEN_ROUNDS 40                        /* Bends are widened in at most this many rounds. */
#define WIDEN_STEP_FRACTION 0.5f               /* A widening step takes this part of the excess. */
#define WIDEN_PATIENCE_PASSES 30               /* `--widenstop` ends this long after the best. */
#define NO_BEST_PASS (-1)                      /* No pass of the widening beat the first curve. */
#define CUT_SECOND_DERIVATIVE_GAIN 6.0f /* A lone cut shrinks by this times the step. */
#define BEND_STENCIL_METERS 8.0f        /* Bends are measured over this distance each way. */

/* These values size the two caches of the smoothing (see `find_sample_room` and
 * `get_cached_cos`).
 *
 * Purpose: Each cache is a table that keeps results, so that the smoothing does not compute the
 *   same result twice. The key of a result is its input: the numbers of a sample, or an angle.
 *
 * Technique: The caches use memoization (remembering the result of a computation from its
 *   input) in a hash table. A hash table is an array in which the entry of a key is computed
 *   from the key itself, so that a lookup reads one entry instead of searching the whole table.
 *
 * Place in the table: To find the place (entry) of a key, the code multiplies each number of
 *   the key by a large fixed number (a multiplier), mixes the products into one number with
 *   exclusive or, and takes its top bits as the entry number. The multipliers scramble the
 *   numbers so that keys that differ only a little, such as two samples close together, get
 *   entries far apart and the table fills evenly. The top bits are used because they depend on
 *   every bit of the key, while the low bits of a product depend only on the low bits of the
 *   key. In decimal, keeping the first two digits of 1000 x 6180 and of 1050 x 6180 gives
 *   entries 61 and 64. A real multiplier is much larger, and the product wraps around at 2^32,
 *   so even keys 1 apart land far apart.
 *
 * Number of bits: The code keeps as many top bits as the table needs to number its entries.
 *   The room memo starts with 2^14 entries, and 14 bits count from 0 to 16,383, exactly one
 *   number for each entry. The cosine cache has 2^12 entries, so it keeps 12 bits, from 0 to
 *   4,095. So each table size is a power of 2, and the top bits of the product are the entry
 *   number as they are.
 *
 * Values: The multipliers are well-known values of the hash function xxHash32. They are odd, so
 *   that a multiplication keeps every bit of the key. Their values never change a result,
 *   because each entry also stores its whole key: a key that finds another key in its entry is
 *   computed again. */
#define ROOM_MEMO_START_BITS 14               /* The room memo starts with 2^14 entries. */
#define ROOM_MEMO_MAX_LOAD 0.5f               /* The memo doubles before it gets fuller. */
#define ROOM_KEY_WORDS 4                      /* A key holds the bits of x, y and the tangent. */
#define COS_CACHE_BITS 12                     /* The cosine cache has 2^12 entries. */
#define COS_CACHE_SIZE (1 << COS_CACHE_BITS)  /* The cosine cache has this many entries. */
#define HASH_BITS 32                          /* Hashes are 32-bit, and use their top bits. */
#define HASH_MULTIPLIER_X 2654435761u         /* These multipliers scramble the key numbers. */
#define HASH_MULTIPLIER_Y 2246822519u
#define HASH_MULTIPLIER_TANGENT_X 3266489917u
#define HASH_MULTIPLIER_TANGENT_Y 668265263u

/* ------------------------------------------------------------------------------------------------
 * `get_route_point` walks along the route points until it finds the segment that holds
 * `distance`, and then interpolates linearly on that segment. The search stops at the last
 * segment, and the fraction on the segment is limited to the range from 0 to 1, so a distance
 * before the start or past the end gives the first or the last point.
 * A segment shorter than `ROUTE_EPSILON_METERS` (1 mm) counts as a point. Dividing by its length
 * would be unsafe, so `get_route_point` takes `final_facing` as the direction there.
 * --------------------------------------------------------------------------------------------- */
V2 get_route_point(const Squad *squad, float distance, V2 *direction) {
    /* Find the segment that holds `distance`: from point `start_index` to point `end_index`. */
    int end_index = 1;
    while (end_index < squad->route_count - 1 && distance > squad->route_distance[end_index])
        end_index++;
    int start_index = end_index - 1;
    V2 from = squad->route[start_index];
    V2 segment = sub(squad->route[end_index], from);
    float segment_start_distance = squad->route_distance[start_index];
    float segment_length = squad->route_distance[end_index] - segment_start_distance;

    /* Find the point on that segment. */
    float fraction;
    if (segment_length > ROUTE_EPSILON_METERS) {
        *direction = mul(segment, 1 / segment_length);
        fraction = (distance - segment_start_distance) / segment_length;
    } else {
        *direction = squad->final_facing;
        fraction = 1;
    }
    float clamped_fraction = clamp_float(fraction, 0, 1);
    return add(from, mul(segment, clamped_fraction));
}

/* ------------------------------------------------------------------------------------------------
 * `get_extended_route_point` takes the clamped point of `get_route_point` and adds the part of
 * `distance` that lies before the start or past the end, along the direction of the first or the
 * last segment.
 * --------------------------------------------------------------------------------------------- */
V2 get_extended_route_point(const Squad *squad, float distance, V2 *direction) {
    V2 point = get_route_point(squad, distance, direction);
    if (distance < 0)
        return add(point, mul(*direction, distance));
    if (distance > squad->route_length)
        return add(point, mul(*direction, distance - squad->route_length));
    return point;
}

/* ------------------------------------------------------------------------------------------------
 * `get_formation_direction` takes the direction of the chord from the route point
 * `FORMATION_DIRECTION_SPAN_METERS` (10 m) behind `distance` to the one that far ahead of it. The
 * direction of the chord turns gradually through a corner, but the direction of one segment jumps
 * there.
 * The chord needs a squared length above `MIN_CHORD_LENGTH_SQUARED`, which means a length above
 * 1 m. A shorter chord has no reliable direction, so `get_formation_direction` takes the
 * direction of the route segment at `distance` instead.
 *
 *                 ahead o
 *                     / |
 *             chord /   | route
 *                 /     |
 *               /       |
 *      behind o----+----+ corner
 *              distance
 * --------------------------------------------------------------------------------------------- */
V2 get_formation_direction(const Squad *squad, float distance) {
    V2 segment_direction;
    V2 unused_direction;
    get_route_point(squad, distance, &segment_direction);
    float ahead_distance = distance + FORMATION_DIRECTION_SPAN_METERS;
    float behind_distance = distance - FORMATION_DIRECTION_SPAN_METERS;
    V2 ahead = get_route_point(squad, ahead_distance, &unused_direction);
    V2 behind = get_route_point(squad, behind_distance, &unused_direction);
    V2 chord = sub(ahead, behind);
    return get_direction(chord, MIN_CHORD_LENGTH_SQUARED, segment_direction);
}

/* ------------------------------------------------------------------------------------------------
 * `limit_to_cell_axis` narrows the stretch [`from`, `to`] of a ray (distances from its start
 * `start` along it) to where the ray stays inside cell `cell` on one axis, at least
 * `CELL_SKIP_MARGIN_METERS` off both sides of the cell. `step` is the change of that coordinate
 * per meter along the ray, and `inverse_step` is 1 / `step`. A ray that runs along the axis
 * without moving on it keeps the stretch only when its start lies inside those limits; otherwise
 * `to` becomes negative.
 * --------------------------------------------------------------------------------------------- */
static void limit_to_cell_axis(float start, float step, double inverse_step, int cell,
                               double *from, double *to) {
    double low = cell * (double)GC_CELL_METERS + CELL_SKIP_MARGIN_METERS;
    double high = (cell + 1) * (double)GC_CELL_METERS - CELL_SKIP_MARGIN_METERS;
    if (step == 0) {
        if (start < low || start > high)
            *to = -1;
        return;
    }
    double enter_side = step > 0 ? low : high;
    double leave_side = step > 0 ? high : low;
    double enter_distance = (enter_side - start) * inverse_step;
    double leave_distance = (leave_side - start) * inverse_step;
    if (enter_distance > *from)
        *from = enter_distance;
    if (leave_distance < *to)
        *to = leave_distance;
}

/* A `Ray` is a ray of `measure_free_distance`: its start and its direction with length 1, and
 * the inverse of each component of the direction (0 for a component of 0), which
 * `limit_to_cell_axis` multiplies by. A multiplication is faster than a division, and the two
 * differ by about 1e-15 m here, far less than `CELL_SKIP_MARGIN_METERS`. */
typedef struct {
    V2 start;
    V2 direction;
    double inverse_x;
    double inverse_y;
} Ray;

/* ------------------------------------------------------------------------------------------------
 * `find_free_cell_skip` returns how far `measure_free_distance` can move on from the probe at
 * `distance` along `ray`, which read the free cell (`cell_x`, `cell_y`). Probes that lie inside
 * that cell, at least `CELL_SKIP_MARGIN_METERS` off its sides, read the same free cell, so
 * `find_free_cell_skip` steps over them. That margin is far larger than the float error of a
 * probe (about 0.3 mm) and than the shift of the cell edges of CellAt from multiples of
 * `GC_CELL_METERS` (under 0.2 mm), so a skipped probe can never lie in another cell. The result
 * is a whole number of steps, so the probes that follow lie exactly where they lie without the
 * skip.
 *
 *     +-------------------------+   the free cell of the probe
 *     |  +-------------------+  |   the same cell, less the margin
 *     |  |   p   s   s   s   |  | n   p: probe, s: skipped probes, n: next probe
 *     |  +-------------------+  |
 *     +-------------------------+
 * Result: It returns `FREE_DISTANCE_STEP_METERS` when the next probe does not lie inside those
 *   limits, and otherwise the step to the first probe past them.
 * --------------------------------------------------------------------------------------------- */
static float find_free_cell_skip(const Ray *ray, int cell_x, int cell_y, float distance) {
    /* Find the stretch of the ray that stays inside the cell, less the margin. */
    double from = 0;
    double to = LANE_MAX_DISTANCE_METERS;
    limit_to_cell_axis(ray->start.x, ray->direction.x, ray->inverse_x, cell_x, &from, &to);
    limit_to_cell_axis(ray->start.y, ray->direction.y, ray->inverse_y, cell_y, &from, &to);

    /* Skip the probes in that stretch. */
    double next_distance = distance + FREE_DISTANCE_STEP_METERS;
    if (next_distance < from || next_distance > to)
        return FREE_DISTANCE_STEP_METERS;
    int skip_count = (int)((to - distance) / FREE_DISTANCE_STEP_METERS);
    return (skip_count + 1) * FREE_DISTANCE_STEP_METERS;
}

/* ------------------------------------------------------------------------------------------------
 * `get_clearance_skip` returns how far `measure_free_distance` can move on from a probe that read
 * the free cell (`cell_x`, `cell_y`): to the first probe past the clearance of that cell
 * (`get_clearance_meters`). Every probe within the clearance of the probe lies in a free cell, so
 * it cannot end the ray. The result is a whole number of steps, at least one, so the probes that
 * follow lie exactly where they lie without the skip.
 * --------------------------------------------------------------------------------------------- */
static float get_clearance_skip(int cell_x, int cell_y, int mover_class) {
    float clearance = get_clearance_meters(cell_x, cell_y, mover_class);
    int free_probe_count = (int)(clearance / FREE_DISTANCE_STEP_METERS);
    return (free_probe_count + 1) * FREE_DISTANCE_STEP_METERS;
}

/* ------------------------------------------------------------------------------------------------
 * `get_inverse` returns 1 / `value`, or 0 when `value` is 0.
 * --------------------------------------------------------------------------------------------- */
static double get_inverse(float value) {
    if (value == 0)
        return 0;
    return 1.0 / value;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_free_distance` probes along `direction` in steps of `FREE_DISTANCE_STEP_METERS`
 * (0.5 m), and takes the distance of the first probe that lies in a blocked cell. So the result
 * is a multiple of the step, and it can lie up to one step past the true edge of the blocked
 * cell. The probing stops at `max_distance`. To save time, it skips the probes that
 * `find_free_cell_skip` finds inside a free cell, or the probes within the clearance of that cell
 * (`get_clearance_skip`) when that reaches further: they cannot change the result. The probes and
 * the skips do not depend on `max_distance`, so the probes before it are the same for every
 * limit. A probe outside the grid reads `GC_CELL_OUTSIDE`, which blocks every mover class, so
 * every skip starts from a cell inside the grid.
 * --------------------------------------------------------------------------------------------- */
float measure_free_distance(V2 point, V2 direction, int mover_class, float max_distance) {
    Ray ray = {
        .start = point,
        .direction = direction,
        .inverse_x = get_inverse(direction.x),
        .inverse_y = get_inverse(direction.y),
    };
    float distance = 0;
    while (distance < max_distance) {
        V2 probe = add(point, mul(direction, distance));
        int cell_x;
        int cell_y;
        if (gc_read_cell_at(probe.x, probe.y, &cell_x, &cell_y) > mover_class)
            return distance;
        float cell_skip = find_free_cell_skip(&ray, cell_x, cell_y, distance);
        float clearance_skip = get_clearance_skip(cell_x, cell_y, mover_class);
        distance += fmaxf(cell_skip, clearance_skip);
    }
    return max_distance;
}

/* ------------------------------------------------------------------------------------------------
 * `find_away_direction` returns the direction, with length 1, from the closest point of a cell
 * that blocks `mover_class` to `point`, among the cells closer than `radius` to `point`.
 * Edge case: It returns zero when no such cell is that close, or when `point` lies in one.
 * --------------------------------------------------------------------------------------------- */
static V2 find_away_direction(V2 point, float radius, int mover_class) {
    /* Find the closest blocked point in the cells that the square around the circle touches. */
    float best_distance_squared = radius * radius;
    V2 closest = point;
    int min_cell_y = get_cell_coordinate(point.y - radius);
    int max_cell_y = get_cell_coordinate(point.y + radius);
    int min_cell_x = get_cell_coordinate(point.x - radius);
    int max_cell_x = get_cell_coordinate(point.x + radius);
    for (int cell_y = min_cell_y; cell_y <= max_cell_y; cell_y++) {
        for (int cell_x = min_cell_x; cell_x <= max_cell_x; cell_x++) {
            if (gc_get_cell(cell_x, cell_y) <= mover_class)
                continue;
            V2 cell_point = get_closest_cell_point(cell_x, cell_y, point);
            float distance_squared = length_squared(sub(cell_point, point));
            if (distance_squared < best_distance_squared) {
                best_distance_squared = distance_squared;
                closest = cell_point;
            }
        }
    }

    /* Point away from it. */
    V2 away = sub(point, closest);
    return get_direction(away, LENGTH_SQUARED_EPSILON, v2(0, 0));
}

/* ------------------------------------------------------------------------------------------------
 * `compute_bspline_weights` puts into `weights` the weights of the four control points of a curve
 * piece at `fraction` along it, from 0 to 1. It puts their slopes into `slopes`. A slope tells
 * how fast its weight changes with `fraction`. These are the standard weights of a uniform cubic
 * B-spline and their derivatives.
 * --------------------------------------------------------------------------------------------- */
static void compute_bspline_weights(float fraction, float weights[CURVE_PIECE_POINTS],
                                    float slopes[CURVE_PIECE_POINTS]) {
    /* Find the weights. */
    float fraction_squared = fraction * fraction;
    float fraction_cubed = fraction_squared * fraction;
    float rest = 1 - fraction;
    weights[0] = rest * rest * rest / 6;
    weights[1] = (3 * fraction_cubed - 6 * fraction_squared + 4) / 6;
    weights[2] = (-3 * fraction_cubed + 3 * fraction_squared + 3 * fraction + 1) / 6;
    weights[3] = fraction_cubed / 6;

    /* Find their slopes. */
    slopes[0] = -rest * rest / 2;
    slopes[1] = (3 * fraction_squared - 4 * fraction) / 2;
    slopes[2] = (-3 * fraction_squared + 2 * fraction + 1) / 2;
    slopes[3] = fraction_squared / 2;
}

/* ------------------------------------------------------------------------------------------------
 * `evaluate_bspline` returns the point at `fraction` along the curve piece that starts at control
 * point `piece_index`, and sets `tangent` to the direction of motion of the curve there.
 * `tangent` does not have length 1.
 * --------------------------------------------------------------------------------------------- */
static V2 evaluate_bspline(const V2 *control, int piece_index, float fraction, V2 *tangent) {
    float weights[CURVE_PIECE_POINTS];
    float slopes[CURVE_PIECE_POINTS];
    compute_bspline_weights(fraction, weights, slopes);

    /* Add up the control points of the piece, times their weights for the point and times their
     * slopes for the tangent. */
    V2 point = v2(0, 0);
    *tangent = v2(0, 0);
    for (int weight_index = 0; weight_index < CURVE_PIECE_POINTS; weight_index++) {
        V2 control_point = control[piece_index + weight_index];
        point = add(point, mul(control_point, weights[weight_index]));
        *tangent = add(*tangent, mul(control_point, slopes[weight_index]));
    }
    return point;
}

/* A `CosEntry` holds the cosine of one angle: the angle as float bits, and whether the entry is
 * filled. */
typedef struct {
    uint32_t angle_bits;
    bool is_used;
    float cosine;
} CosEntry;

/* Entry k of the cosine cache holds an angle whose hash is k. The caches live on the heap,
 * because large static arrays can move the image space of game.c. `smooth_route` allocates
 * them. Threads: Each thread has its own caches, smoothing state and work arrays
 * (`_Thread_local`), because with `--setupthreads on` worker threads smooth the routes of several
 * squads at the same time (squad.c). */
static _Thread_local CosEntry *cos_cache;

/* ------------------------------------------------------------------------------------------------
 * `read_float_bits` returns the bits of `value`, for use as an exact key.
 * --------------------------------------------------------------------------------------------- */
static uint32_t read_float_bits(float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}

/* ------------------------------------------------------------------------------------------------
 * `get_cached_cos` returns `cosf(angle)`, from the cosine cache when the same angle came before.
 * cosf of MinGW computes in long double and is slow, and the tapers of the pushes repeat a few
 * angles many times. cosf has one result for each angle, so the cache returns exactly what the
 * call would. A new angle replaces the entry with its hash.
 * --------------------------------------------------------------------------------------------- */
static float get_cached_cos(float angle) {
    uint32_t bits = read_float_bits(angle);
    uint32_t hash = bits * HASH_MULTIPLIER_X;
    CosEntry *entry = &cos_cache[hash >> (HASH_BITS - COS_CACHE_BITS)];
    if (entry->is_used && entry->angle_bits == bits)
        return entry->cosine;
    entry->angle_bits = bits;
    entry->is_used = true;
    entry->cosine = cosf(angle);
    return entry->cosine;
}

/* A `CurveRoom` holds the values that a pass along the curve uses to find where it lacks room (a
 * circle of the wanted radius around a sample that is free of blocked cells), and to push the
 * curve and measure its bends there. `begin_smoothing` computes the counts of control points and
 * samples from the reaches in meters (`PUSH_SPREAD_METERS` and the like) and from the
 * spacings of the control points and samples of the route. */
typedef struct {
    float sample_spacing;      /* The curve is sampled about this far apart. */
    float body_clearance;      /* Each sample needs this much room for the widest member body. */
    float formation_clearance; /* Each sample wants this much room for the whole formation. */
    int mover_class;
    int push_spread;           /* Pushes reach this many control points each way. */
    float push_taper_points;   /* Pushes fade out over this many control points. */
    int bend_stencil_samples;  /* Bends are measured over this many samples each way. */
} CurveRoom;

/* ------------------------------------------------------------------------------------------------
 * `push_control_points` pushes the control points around a spot of the curve (a sample that lacks
 * room) away from an obstacle. The control point closest to the spot moves most, and the next
 * ones on each side less and less, so that the detour is one wide, gentle curve instead of a
 * local bump.
 *
 *                ^ away
 *            ^   |   ^
 *        ^   |   |   |   ^
 *     ---o---o---o---o---o---   control points, and their moves along `away`
 *                ^
 *                spot at `curve_position`
 *
 * Inputs: `curve_position` is the position of the spot along the curve, in control-point numbers
 *   with a fraction. `away` is the direction of the push.
 * Result: `push_control_points` moves the control points from `push_spread` before to
 *   `push_spread` after control point `spot_index`, the whole part of `curve_position`.
 *   Each one moves by `PUSH_STEP_METERS` times a taper (a cosine bell). The taper falls from 1 at
 *   the spot to 0 at `push_taper_points` control points from it. Both counts come from `room`.
 * Edge case: The `FIXED_END_POINTS` control points at each end never move. They are the copies
 *   of the end points (`START_REPEATS`, `END_REPEATS`) that pin the ends of the curve, so their
 *   number does not depend on the spacing.
 * --------------------------------------------------------------------------------------------- */
static void push_control_points(V2 *control, int control_count, const CurveRoom *room,
                                float curve_position, V2 away) {
    int spot_index = (int)curve_position;
    int first_index = spot_index - room->push_spread;
    int last_index = spot_index + room->push_spread;
    for (int control_index = first_index; control_index <= last_index; control_index++) {
        bool is_at_start = control_index < FIXED_END_POINTS;
        bool is_at_end = control_index >= control_count - FIXED_END_POINTS;
        if (is_at_start || is_at_end)
            continue;
        float spread_points = fabsf(control_index - curve_position) / room->push_taper_points;
        float spread = fminf(1, spread_points);
        float taper = 0.5f * (1 + get_cached_cos(PI * spread));
        control[control_index] = add(control[control_index], mul(away, PUSH_STEP_METERS * taper));
    }
}

/* ------------------------------------------------------------------------------------------------
 * `place_control_points` puts the control points of the smooth route into `control` and returns
 * their count. They lie along the squad route, at most `control_spacing` apart. The first point
 * of the route goes in `START_REPEATS` times, and the last point `END_REPEATS` more times, so
 * that the curve starts and ends on them.
 * Edge case: When the route needs more than `MAX_WAYPOINTS` control points, the copies of the
 *   last point follow the last control point that fits.
 * --------------------------------------------------------------------------------------------- */
static int place_control_points(const Squad *squad, float control_spacing, V2 *control) {
    /* Repeat the first point. */
    int control_count = 0;
    for (int repeat_index = 0; repeat_index < START_REPEATS; repeat_index++) {
        control[control_count] = squad->route[0];
        control_count++;
    }

    /* Split each leg of the route into equal pieces of at most `control_spacing`, and add the end
     * of each piece. */
    for (int point_index = 1; point_index < squad->route_count; point_index++) {
        if (control_count >= MAX_WAYPOINTS)
            break;
        V2 from = squad->route[point_index - 1];
        V2 leg = sub(squad->route[point_index], from);
        int piece_count = (int)ceilf(len(leg) / control_spacing);
        for (int piece_index = 1; piece_index <= piece_count; piece_index++) {
            if (control_count >= MAX_WAYPOINTS)
                break;
            float piece_end_fraction = (float)piece_index / piece_count;
            control[control_count] = add(from, mul(leg, piece_end_fraction));
            control_count++;
        }
    }

    /* Repeat the last point. */
    V2 route_end = squad->route[squad->route_count - 1];
    for (int repeat_index = 0; repeat_index < END_REPEATS; repeat_index++) {
        control[control_count] = route_end;
        control_count++;
    }
    return control_count;
}


/* ------------------------------------------------------------------------------------------------
 * `find_wanted_room` returns the room that the curve wants around `point`, where it runs along
 * `tangent`: half the width of the passage across the curve less `PASSAGE_MIDDLE_MARGIN_METERS`,
 * but at least `body_clearance` and at most `formation_clearance`. `tangent` does not need to
 * have length 1.
 *
 *     ///////// blocked //////////
 *                  ^
 *                  | left_free
 *     -------------o------------>   the curve at `point`, along `tangent`
 *                  | right_free
 *                  v
 *     ///////// blocked //////////
 *
 * Short rays: A passage at least `wide_width` wide gives the room for the formation, whatever its
 * exact width, so `find_wanted_room` stops each ray once the passage is known to be that wide.
 * When the right ray alone reaches it, the left ray is not cast at all. This gives exactly the
 * room of two full rays. The free distances are multiples of the 0.5 m step up to 30 m, so their
 * sums, halves and the margin of 0.25 m have no rounding. A ray that stops at its limit stands
 * for a true distance at least that long, and `wide_width` has one step of spare width, far more
 * than the rounding of the limits themselves.
 * --------------------------------------------------------------------------------------------- */
static float find_wanted_room(V2 point, V2 tangent, const CurveRoom *room) {
    /* Find the width of passage that surely gives the room for the formation. */
    float formation_width = 2 * (room->formation_clearance + PASSAGE_MIDDLE_MARGIN_METERS);
    float wide_width = formation_width + FREE_DISTANCE_STEP_METERS;

    /* Measure the free distance to the right. When it alone gives the room for the formation,
     * the passage gives it too. */
    V2 direction = get_direction(tangent, LENGTH_SQUARED_EPSILON, v2(1, 0));
    V2 right = get_right(direction);
    V2 left = mul(right, -1);
    float right_limit = fminf(wide_width, LANE_MAX_DISTANCE_METERS);
    float right_free = measure_free_distance(point, right, room->mover_class, right_limit);
    float right_room = right_free / 2 - PASSAGE_MIDDLE_MARGIN_METERS;
    if (right_room >= room->formation_clearance)
        return room->formation_clearance;

    /* Measure the free distance to the left, up to where the passage is `wide_width` wide. */
    float left_limit = fminf(wide_width - right_free, LANE_MAX_DISTANCE_METERS);
    float left_free = measure_free_distance(point, left, room->mover_class, left_limit);
    float passage_width = right_free + left_free;

    /* Take half of the width less the margin, and limit that to the range from the room for the
     * body to the room for the formation. */
    float passage_room = passage_width / 2 - PASSAGE_MIDDLE_MARGIN_METERS;
    return clamp_float(passage_room, room->body_clearance, room->formation_clearance);
}

/* A `RoomEntry` holds the room tests of one curve sample: the room from `find_wanted_room`, and
 * whether the circles of the body room and of that room are clear. `key` holds the float bits of
 * the point and the tangent of the sample, and `generation` tells which smoothing filled the
 * entry (0: never). */
typedef struct {
    uint32_t key[ROOM_KEY_WORDS];
    uint32_t generation;
    float wanted;
    bool is_body_clear;
    bool is_wanted_clear;
} RoomEntry;

/* The room memo is a hash table of `RoomEntry` on the heap, with 2^`room_memo_bits` entries.
 *
 * Memory: `start_smoothing_caches` allocates the table, and `grow_room_memo` doubles it.
 *
 * Generations: `room_generation` counts the smoothings, so that a new smoothing finds the
 *   entries of the earlier ones stale without clearing the table. `room_memo_count` counts the
 *   entries of the current smoothing.
 *
 * Size: The first size, 2^`ROOM_MEMO_START_BITS` entries, fits most smoothings. Filled at most
 *   to `ROOM_MEMO_MAX_LOAD`, it holds 8,192 samples, which was enough for most of the smoothings
 *   after testing it. When a smoothing needs more, `find_sample_room` doubles the memo while the
 *   smoothing runs. Once doubled, it keeps its size. */
static _Thread_local RoomEntry *room_memo;
static _Thread_local int room_memo_bits;
static _Thread_local int room_memo_count;
static _Thread_local uint32_t room_generation;

/* `room_test_count` counts the room tests that `find_sample_room` had to run, because the memo
 * did not hold them. They take nearly all the time of a smoothing, so `run_smoothing` measures
 * its work in them. */
static _Thread_local int room_test_count;

/* ------------------------------------------------------------------------------------------------
 * `test_sample_room` fills `entry` with the room tests of the sample at `point`.
 * It tests the circle of the wanted room first. The wanted room is never smaller than the body
 * room, and `is_circle_clear` finds every blocking cell of a circle also in each larger circle
 * around the same point. So when the wanted circle is clear, the body circle is clear too, and
 * `test_sample_room` skips that test.
 * --------------------------------------------------------------------------------------------- */
static void test_sample_room(V2 point, V2 tangent, const CurveRoom *room, RoomEntry *entry) {
    entry->wanted = find_wanted_room(point, tangent, room);
    entry->is_wanted_clear = is_circle_clear(point, entry->wanted, room->mover_class);
    entry->is_body_clear = entry->is_wanted_clear;
    bool must_test_body = !entry->is_wanted_clear && entry->wanted > room->body_clearance;
    if (must_test_body)
        entry->is_body_clear = is_circle_clear(point, room->body_clearance, room->mover_class);
}

/* ------------------------------------------------------------------------------------------------
 * `start_room_generation` makes every entry of the room memo stale.
 * --------------------------------------------------------------------------------------------- */
static void start_room_generation(void) {
    room_generation++;
    room_memo_count = 0;
}

/* ------------------------------------------------------------------------------------------------
 * `hash_room_key` returns the hash of a key of the room memo, from its `ROOM_KEY_WORDS` words.
 * --------------------------------------------------------------------------------------------- */
static uint32_t hash_room_key(const uint32_t *key) {
    uint32_t point_hash = key[0] * HASH_MULTIPLIER_X ^ key[1] * HASH_MULTIPLIER_Y;
    uint32_t tangent_x_hash = key[2] * HASH_MULTIPLIER_TANGENT_X;
    uint32_t tangent_y_hash = key[3] * HASH_MULTIPLIER_TANGENT_Y;
    return point_hash ^ tangent_x_hash ^ tangent_y_hash;
}

/* ------------------------------------------------------------------------------------------------
 * `find_room_entry` returns the entry of the room memo that holds `key` in this generation, or
 * else the stale entry where `key` belongs. The table is open addressing: the search starts at
 * the entry that the top bits of the hash pick, and goes on entry by entry, back to the first
 * entry after the last. It always ends, because the memo is never full (`ROOM_MEMO_MAX_LOAD`).
 * --------------------------------------------------------------------------------------------- */
static RoomEntry *find_room_entry(const uint32_t *key) {
    uint32_t index_mask = (1u << room_memo_bits) - 1;
    uint32_t entry_index = hash_room_key(key) >> (HASH_BITS - room_memo_bits);
    size_t key_bytes = ROOM_KEY_WORDS * sizeof *key;
    while (true) {
        RoomEntry *entry = &room_memo[entry_index];
        bool is_stale = entry->generation != room_generation;
        if (is_stale || memcmp(entry->key, key, key_bytes) == 0)
            return entry;
        entry_index = (entry_index + 1) & index_mask;
    }
}

/* ------------------------------------------------------------------------------------------------
 * `grow_room_memo` doubles the room memo. It moves each entry of this generation to the place of
 * its key in the new table, which has one more bit of entry numbers, and frees the old table.
 * --------------------------------------------------------------------------------------------- */
static void grow_room_memo(void) {
    RoomEntry *old_memo = room_memo;
    int old_size = 1 << room_memo_bits;
    room_memo_bits++;
    room_memo = calloc((size_t)1 << room_memo_bits, sizeof *room_memo);
    for (int entry_index = 0; entry_index < old_size; entry_index++) {
        const RoomEntry *old_entry = &old_memo[entry_index];
        if (old_entry->generation != room_generation)
            continue;
        RoomEntry *new_entry = find_room_entry(old_entry->key);
        *new_entry = *old_entry;
    }
    free(old_memo);
}

/* ------------------------------------------------------------------------------------------------
 * `find_sample_room` returns the room tests of the sample at `point` with `tangent`, from the
 * room memo when the same sample came before in this smoothing. Each pass of the smoothing
 * samples the whole curve again, but a push moves only the curve near its spot, so most samples
 * come back with the same point and tangent, bit for bit. During one smoothing the grid and
 * `room` do not change, and the tests read nothing else, so a stored result is exactly what the
 * tests would give again.
 * Result: The returned entry stays valid until the next call.
 * --------------------------------------------------------------------------------------------- */
static const RoomEntry *find_sample_room(V2 point, V2 tangent, const CurveRoom *room) {
    /* Return the stored entry of the key: the bits of the point and the tangent. */
    uint32_t key[ROOM_KEY_WORDS] = {
        read_float_bits(point.x),
        read_float_bits(point.y),
        read_float_bits(tangent.x),
        read_float_bits(tangent.y),
    };
    RoomEntry *entry = find_room_entry(key);
    if (entry->generation == room_generation)
        return entry;

    /* Double the memo first when it would get too full. The entries move then, so find the
     * place of the key again. */
    float max_count = ROOM_MEMO_MAX_LOAD * (1 << room_memo_bits);
    if (room_memo_count + 1 > max_count) {
        grow_room_memo();
        entry = find_room_entry(key);
    }

    /* Test the sample, and store the result. */
    entry->generation = room_generation;
    memcpy(entry->key, key, sizeof key);
    test_sample_room(point, tangent, room, entry);
    room_memo_count++;
    room_test_count++;
    return entry;
}

/* ------------------------------------------------------------------------------------------------
 * `start_smoothing_caches` allocates the room memo and the cosine cache once, and makes the room
 * memo of earlier smoothings stale. The cosine cache stays valid, because cosines never change.
 * --------------------------------------------------------------------------------------------- */
static void start_smoothing_caches(void) {
    if (room_memo == NULL) {
        room_memo_bits = ROOM_MEMO_START_BITS;
        room_memo = calloc((size_t)1 << room_memo_bits, sizeof *room_memo);
        cos_cache = calloc(COS_CACHE_SIZE, sizeof *cos_cache);
    }
    start_room_generation();
}

/* ------------------------------------------------------------------------------------------------
 * `round_up_to_int` returns the smallest whole number that is at least `value`, which must lie
 * from 0 to 2^24. It gives the same result as `(int)ceilf(value)`, but ceilf of MinGW is slow.
 * --------------------------------------------------------------------------------------------- */
static int round_up_to_int(float value) {
    int truncated = (int)value;
    if ((float)truncated < value)
        return truncated + 1;
    return truncated;
}

/* ------------------------------------------------------------------------------------------------
 * `push_curve_clear` makes one pass along the curve of `control`. It samples the curve into
 * `curve`, and wherever a sample lacks the room from `find_wanted_room`, it pushes the control
 * points that shape that spot away from the closest obstacle. A push changes the curve at once,
 * so the later samples of the same pass already use it.
 * Result: It returns true when every sample had the wanted room, and false otherwise.
 *   `curve_count` becomes the number of samples. `is_curve_clear` becomes true when every sample
 *   had room for the body, and false otherwise.
 * --------------------------------------------------------------------------------------------- */
static bool push_curve_clear(V2 *control, int control_count, const CurveRoom *room, V2 *curve,
                             int *curve_count, bool *is_curve_clear) {
    int mover_class = room->mover_class;
    bool has_room = true;
    *is_curve_clear = true;
    *curve_count = 0;
    int piece_count = control_count - CURVE_PIECE_POINTS + 1;
    for (int piece_index = 0; piece_index < piece_count; piece_index++) {
        /* Sample the piece about every `sample_spacing`. The piece runs from about control point
         * `piece_start_index` to control point `piece_end_index`. */
        int piece_start_index = piece_index + 1;
        int piece_end_index = piece_index + 2;
        V2 chord = sub(control[piece_end_index], control[piece_start_index]);
        float piece_length = len(chord);
        int piece_sample_count = 1;
        if (piece_length > 0)
            piece_sample_count = round_up_to_int(piece_length / room->sample_spacing);
        for (int sample_index = 0; sample_index < piece_sample_count; sample_index++) {
            if (*curve_count >= MAX_WAYPOINTS - 1)
                break;
            float piece_fraction = (float)sample_index / piece_sample_count;
            V2 tangent;
            V2 point = evaluate_bspline(control, piece_index, piece_fraction, &tangent);
            curve[*curve_count] = point;
            (*curve_count)++;

            /* Where a circle of the wanted room around the point is not clear, push the control
             * points that shape this spot away from the closest obstacle. */
            const RoomEntry *sample_room = find_sample_room(point, tangent, room);
            if (sample_room->is_body_clear && sample_room->is_wanted_clear)
                continue;
            *is_curve_clear &= sample_room->is_body_clear;
            has_room = false;
            float search_radius = sample_room->wanted + AWAY_SEARCH_MARGIN_METERS;
            V2 away = find_away_direction(point, search_radius, mover_class);
            float curve_position = piece_start_index + piece_fraction;
            push_control_points(control, control_count, room, curve_position, away);
        }
    }
    return has_room;
}

/* ------------------------------------------------------------------------------------------------
 * `widen_turns` makes one step that widens the bends of the curve where they are sharper than
 * `max_curvature`, the inverse of the wanted radius.
 *
 * At control point `j`, the uniform cubic B-spline has the first derivative
 * `(P[j+1] - P[j-1]) / 2` and the second derivative `P[j-1] - 2 * P[j] + P[j+1]`, both per unit
 * of the parameter of the curve. Its curvature there is `|cross(first, second)| / |first|^3`.
 * Where the curvature is above `max_curvature`, the point gets a cut: the part of its second
 * derivative that makes the bend too sharp. The cut is the second derivative times the part of the
 * curvature above `max_curvature`, as a fraction of the curvature. The step moves the point toward
 * the inside of the bend by twice its cut, and its two neighbors toward the outside by one cut
 * each, times `WIDEN_STEP_FRACTION / CUT_SECOND_DERIVATIVE_GAIN`. The file comment sketches this
 * move. The move is the gradient step that lowers the sum of the squared cuts. On its own, a cut
 * changes the second derivative of its point by `CUT_SECOND_DERIVATIVE_GAIN` times the step, so
 * the step takes away about `WIDEN_STEP_FRACTION` of the excess.
 * Inputs: A control point whose entry of `is_frozen` is true neither moves nor gets a cut.
 * Result: It returns true when some control point that is not frozen had a bend sharper than
 *   `max_curvature`.
 * Edge case: The `FIXED_END_POINTS` control points at each end never move and have no cut.
 * --------------------------------------------------------------------------------------------- */
static bool widen_turns(V2 *control, int control_count, float max_curvature,
                        const bool *is_frozen) {
    /* Entry k is the cut of control point k. The array is on the heap, one per thread. */
    static _Thread_local V2 *cuts;
    if (!cuts)
        cuts = malloc(sizeof(V2) * (MAX_WAYPOINTS + CONTROL_SPARE_POINTS));

    /* Find the cut of each control point. */
    bool is_any_tight = false;
    int first_free_index = FIXED_END_POINTS;
    int end_free_index = control_count - FIXED_END_POINTS;
    for (int control_index = 0; control_index < control_count; control_index++) {
        cuts[control_index] = v2(0, 0);
        bool is_fixed = control_index < first_free_index || control_index >= end_free_index;
        if (is_fixed || is_frozen[control_index])
            continue;
        V2 before = control[control_index - 1];
        V2 here = control[control_index];
        V2 after = control[control_index + 1];
        V2 first_derivative = mul(sub(after, before), 0.5f);
        V2 second_derivative = add(sub(before, here), sub(after, here));
        float speed = len(first_derivative);
        if (speed < ROUTE_EPSILON_METERS)
            continue;
        float bend = fabsf(det2(first_derivative, second_derivative));
        float curvature = bend / (speed * speed * speed);
        if (curvature <= max_curvature)
            continue;
        is_any_tight = true;
        float excess_fraction = (curvature - max_curvature) / curvature;
        cuts[control_index] = mul(second_derivative, excess_fraction);
    }

    /* Move the free control points. */
    float step = WIDEN_STEP_FRACTION / CUT_SECOND_DERIVATIVE_GAIN;
    for (int control_index = first_free_index; control_index < end_free_index; control_index++) {
        if (is_frozen[control_index])
            continue;
        V2 neighbor_cuts = add(cuts[control_index - 1], cuts[control_index + 1]);
        V2 change = sub(mul(cuts[control_index], 2), neighbor_cuts);
        control[control_index] = add(control[control_index], mul(change, step));
    }
    return is_any_tight;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_bend_excess` returns how much the curve `curve[0..curve_count)` turns beyond bends of
 * `max_curvature`: the sum, over the samples, of the curvature above `max_curvature` times the
 * distance to the next sample. The curvature at a sample is that of the circle through it and the
 * samples `stencil_samples` before and after it: `2 * |cross| / (a * b * c)`, where a, b and c
 * are the sides of the triangle of the three points and `cross` is twice its area.
 *
 *                   here
 *                    o
 *                a /   \ b
 *                /       \
 *       before o - - c - - o after
 * --------------------------------------------------------------------------------------------- */
static float measure_bend_excess(const V2 *curve, int curve_count, float max_curvature,
                                 int stencil_samples) {
    float excess = 0;
    int end_index = curve_count - stencil_samples;
    for (int sample_index = stencil_samples; sample_index < end_index; sample_index++) {
        /* Find the curvature of the circle through the three samples. */
        V2 before = curve[sample_index - stencil_samples];
        V2 here = curve[sample_index];
        V2 after = curve[sample_index + stencil_samples];
        V2 to_here = sub(here, before);
        V2 to_after = sub(after, before);
        float side_product = len(to_here) * len(sub(after, here)) * len(to_after);
        if (side_product < ROUTE_EPSILON_METERS)
            continue;
        float curvature = 2 * fabsf(det2(to_here, to_after)) / side_product;
        if (curvature <= max_curvature)
            continue;

        /* Add the excess at this sample. */
        float step_length = len(sub(curve[sample_index + 1], here));
        excess += (curvature - max_curvature) * step_length;
    }
    return excess;
}

/* ------------------------------------------------------------------------------------------------
 * `count_cramped_samples` returns how many samples of the curve `curve[0..curve_count)` lack the
 * room from `find_wanted_room`, without the first and the last sample. The tangent at a sample is
 * the direction from the sample before it to the sample after it.
 * Result: `is_body_clear` becomes true when every one of those samples has room for the body, and
 *   false otherwise.
 * --------------------------------------------------------------------------------------------- */
static int count_cramped_samples(const V2 *curve, int curve_count, const CurveRoom *room,
                                 bool *is_body_clear) {
    int cramped_count = 0;
    *is_body_clear = true;
    for (int sample_index = 1; sample_index < curve_count - 1; sample_index++) {
        V2 point = curve[sample_index];
        V2 tangent = sub(curve[sample_index + 1], curve[sample_index - 1]);
        const RoomEntry *sample_room = find_sample_room(point, tangent, room);
        if (!sample_room->is_body_clear)
            *is_body_clear = false;
        if (!sample_room->is_wanted_clear)
            cramped_count++;
    }
    return cramped_count;
}

/* ------------------------------------------------------------------------------------------------
 * `measure_points_length` returns the length of the line through `points[0..point_count)`.
 * --------------------------------------------------------------------------------------------- */
static float measure_points_length(const V2 *points, int point_count) {
    float length = 0;
    for (int point_index = 1; point_index < point_count; point_index++)
        length += len(sub(points[point_index], points[point_index - 1]));
    return length;
}

/* `SmoothingPhase` tells which loop of the smoothing runs next. */
typedef enum {
    SMOOTHING_PUSHES,   /* Rounds of pushes give the first curve its room. */
    SMOOTHING_WIDENING, /* Passes widen the bends of a copy, with `--wideturns on`. */
    SMOOTHING_FINISHED, /* The squad route holds the result. */
} SmoothingPhase;

/* A `Smoothing` holds the state of the smoothing that runs on this thread, so that it can go on
 * over several ticks (`--setupslice on`). `begin_smoothing` fills it, and `run_smoothing` runs its
 * steps: a round of pushes, or a pass of the widening. */
typedef struct {
    Squad *squad;
    CurveRoom room;
    SmoothingPhase phase;
    float total_length;     /* The route was this long before the smoothing, in m. */
    float turn_radius;      /* The widening widens bends sharper than this circle, in m. */
    int control_count;      /* `smoothing_control` holds this many control points. */
    int curve_count;        /* `smoothing_curve` holds this many samples. */
    int round_index;        /* This many rounds of pushes ran. */
    bool has_room;          /* The last round found the wanted room at every sample. */
    bool is_curve_clear;    /* The last round found room for the body at every sample. */
    float max_curvature;    /* This is 1 / `turn_radius`. */
    float best_excess;      /* The best curve turns this much beyond the wanted bends. */
    int max_cramped_count;  /* The first curve lacks the wanted room at this many samples. */
    int pass_index;         /* This many passes of the widening ran. */
    int best_pass_index;    /* The pass of the best curve, or `NO_BEST_PASS` for the first one. */
    bool is_widening;       /* The last widening step found a bend that was too sharp. */
    uint32_t stamp_hash;    /* This was `hash_stamped_cells` at the last call. */
} Smoothing;

static _Thread_local Smoothing smoothing;

/* Entry k of these arrays belongs to control point k: the control points of the first curve,
 * of the widened copy, and of that copy before its last pushes, and whether a push moved it.
 * Each holds `MAX_WAYPOINTS + CONTROL_SPARE_POINTS` entries on the heap, one set per thread,
 * which `allocate_smoothing_arrays` allocates. */
static _Thread_local V2 *smoothing_control;
static _Thread_local V2 *widened;
static _Thread_local V2 *before_pushes;
static _Thread_local bool *is_frozen;

/* These arrays hold the samples of the best curve so far and of the widened copy, with
 * `MAX_WAYPOINTS` entries each, on the heap like the arrays above. */
static _Thread_local V2 *smoothing_curve;
static _Thread_local V2 *widened_curve;

/* ------------------------------------------------------------------------------------------------
 * `allocate_smoothing_arrays` allocates the work arrays of the smoothing on this thread, once.
 * --------------------------------------------------------------------------------------------- */
static void allocate_smoothing_arrays(void) {
    if (smoothing_control)
        return;
    size_t control_capacity = MAX_WAYPOINTS + CONTROL_SPARE_POINTS;
    smoothing_control = malloc(sizeof(V2) * control_capacity);
    widened = malloc(sizeof(V2) * control_capacity);
    before_pushes = malloc(sizeof(V2) * control_capacity);
    is_frozen = malloc(sizeof(bool) * control_capacity);
    smoothing_curve = malloc(sizeof(V2) * MAX_WAYPOINTS);
    widened_curve = malloc(sizeof(V2) * MAX_WAYPOINTS);
}

/* ------------------------------------------------------------------------------------------------
 * `finish_smoothing` ends the smoothing: it adds the end point of the route to the best curve and
 * makes that curve the squad route, unless the squad `is_replanning` and the curve is longer than
 * `REPLAN_MAX_STRETCH` times the route before the smoothing.
 * With `--parkreplan on`, a squad can search again around a squad that parked on its route. The
 * new search starts beside the disc of the parked squad (the cliff that squad.c stamps around all
 * its members), often less than the half-width of a formation from it. The smoothing then pushes
 * the start of the route away from the disc, which can bend the route back the way the squad came
 * and around in a hairpin. So a replanned route keeps its unsmoothed corners when it would
 * stretch that much. The squad drives slowly there anyway, so it can take the corners. First
 * routes stay smoothed, because their smoothed arcs around parked squads can be much longer than
 * the route of the search.
 * --------------------------------------------------------------------------------------------- */
static void finish_smoothing(void) {
    Squad *squad = smoothing.squad;
    smoothing.phase = SMOOTHING_FINISHED;
    smoothing_curve[smoothing.curve_count] = squad->route[squad->route_count - 1];
    smoothing.curve_count++;

    /* Keep a replanned route unsmoothed when smoothing stretches it too much. */
    float max_length = REPLAN_MAX_STRETCH * smoothing.total_length;
    float curve_length = measure_points_length(smoothing_curve, smoothing.curve_count);
    if (squad->is_replanning && curve_length > max_length)
        return;
    memcpy(squad->route, smoothing_curve, sizeof(V2) * smoothing.curve_count);
    squad->route_count = smoothing.curve_count;
}

/* ------------------------------------------------------------------------------------------------
 * `start_widening` measures the first curve and starts the widening from a copy of its control
 * points, with no control point frozen.
 * --------------------------------------------------------------------------------------------- */
static void start_widening(void) {
    float max_curvature = 1 / smoothing.turn_radius;
    int curve_count = smoothing.curve_count;
    bool is_first_body_clear = false;
    smoothing.max_curvature = max_curvature;
    int stencil_samples = smoothing.room.bend_stencil_samples;
    smoothing.best_excess = measure_bend_excess(smoothing_curve, curve_count, max_curvature,
                                                stencil_samples);
    smoothing.max_cramped_count = count_cramped_samples(smoothing_curve, curve_count,
                                                        &smoothing.room, &is_first_body_clear);
    memcpy(widened, smoothing_control, sizeof(V2) * smoothing.control_count);
    memset(is_frozen, 0, sizeof(bool) * smoothing.control_count);
    smoothing.is_widening = true;
    smoothing.pass_index = 0;
    smoothing.best_pass_index = NO_BEST_PASS;
    smoothing.phase = SMOOTHING_WIDENING;
}

/* ------------------------------------------------------------------------------------------------
 * `end_pushes` decides what follows the rounds of pushes. The curve counts only if the body is
 * clear at every sample of the last round; otherwise the route stays as it was. So the pushes
 * can fail, which happens at tight turns around cliff tips, in about 1 route of 40. The route as
 * it was is clear, and tests showed that partly smoothed routes drive worse. With
 * `--wideturns on`, the widening follows, so that every member can drive the bends at full speed.
 * --------------------------------------------------------------------------------------------- */
static void end_pushes(void) {
    if (!smoothing.is_curve_clear) {
        smoothing.phase = SMOOTHING_FINISHED;
        return;
    }
    if (is_wide_turn_on)
        start_widening();
    else
        finish_smoothing();
}

/* ------------------------------------------------------------------------------------------------
 * `run_push_round` pushes the control points away from blocked ground wherever the curve lacks
 * room. The rounds end when the curve has room everywhere or after `SMOOTHING_ROUNDS` rounds.
 * --------------------------------------------------------------------------------------------- */
static void run_push_round(void) {
    smoothing.has_room = push_curve_clear(smoothing_control, smoothing.control_count,
                                          &smoothing.room, smoothing_curve,
                                          &smoothing.curve_count, &smoothing.is_curve_clear);
    smoothing.round_index++;
    if (smoothing.has_room || smoothing.round_index >= SMOOTHING_ROUNDS)
        end_pushes();
}

/* ------------------------------------------------------------------------------------------------
 * `run_widening_pass` runs one pass of the widening of the bends that are sharper than a circle
 * of `turn_radius`. Each pass makes one `widen_turns` step and one pass of `push_curve_clear`,
 * for up to `WIDEN_ROUNDS` passes and until no bend of a free control point is too sharp. Then
 * the passes go on with the pushes alone, until the curve has the room or
 * `WIDEN_ROUNDS + SMOOTHING_ROUNDS` passes ran in all. Where walls stand close to a bend, the
 * pushes work against the widening. So a control point that a pass had to push stays frozen for
 * the widening from then on, and the pushes can settle there. Even so, the curve of the last pass
 * is not always the best, so the widening keeps the best curve of all passes: one with room for
 * the body everywhere, no more cramped samples than the first curve (`count_cramped_samples`),
 * and a smaller `measure_bend_excess`.
 * With `--widenstop on`, the passes also stop after `WIDEN_PATIENCE_PASSES` passes in a row
 * without a better curve. In runs of 40 seeds (1417 widenings), the passes found their last
 * better curve by pass 55 at the latest, and always less than 30 passes after the better curve
 * before it, or after the start. Many widenings in narrow valleys still ran all 100 passes. So
 * the stop leaves the route as it was in nearly every case, and it saves more than half of the
 * work of the widening.
 * --------------------------------------------------------------------------------------------- */
static void run_widening_pass(void) {
    /* Widen the bends while a free control point has one that is too sharp. */
    int control_count = smoothing.control_count;
    float max_curvature = smoothing.max_curvature;
    bool may_widen = smoothing.is_widening && smoothing.pass_index < WIDEN_ROUNDS;
    if (may_widen)
        smoothing.is_widening = widen_turns(widened, control_count, max_curvature, is_frozen);
    else
        smoothing.is_widening = false;

    /* Push the curve back where it lacks room, and freeze the control points that the pushes
     * moved. */
    size_t control_bytes = sizeof(V2) * control_count;
    memcpy(before_pushes, widened, control_bytes);
    int widened_count = 0;
    bool is_widened_clear = false;
    const CurveRoom *room = &smoothing.room;
    bool has_widened_room = push_curve_clear(widened, control_count, room, widened_curve,
                                             &widened_count, &is_widened_clear);
    for (int control_index = 0; control_index < control_count; control_index++) {
        V2 push = sub(widened[control_index], before_pushes[control_index]);
        if (length_squared(push) > 0)
            is_frozen[control_index] = true;
    }

    /* Keep the curve when it is the best so far. */
    float excess = measure_bend_excess(widened_curve, widened_count, max_curvature,
                                       room->bend_stencil_samples);
    if (excess < smoothing.best_excess) {
        bool is_body_clear = false;
        int cramped_count = count_cramped_samples(widened_curve, widened_count, room,
                                                  &is_body_clear);
        if (is_body_clear && cramped_count <= smoothing.max_cramped_count) {
            smoothing.best_excess = excess;
            memcpy(smoothing_curve, widened_curve, sizeof(V2) * widened_count);
            smoothing.curve_count = widened_count;
            smoothing.best_pass_index = smoothing.pass_index;
        }
    }

    /* Once the widening has ended, stop as soon as the curve has the room. With
     * `--widenstop on`, also stop when the passes stopped finding better curves. */
    int stale_pass_count = smoothing.pass_index - smoothing.best_pass_index;
    bool is_stale = is_widen_stop_on && stale_pass_count >= WIDEN_PATIENCE_PASSES;
    smoothing.pass_index++;
    bool has_ended = !smoothing.is_widening && has_widened_room;
    bool is_out_of_passes = smoothing.pass_index >= WIDEN_ROUNDS + SMOOTHING_ROUNDS;
    if (has_ended || is_stale || is_out_of_passes)
        finish_smoothing();
}

/* ------------------------------------------------------------------------------------------------
 * `count_spacings` returns how many steps of `spacing` make up `meters`, rounded to the nearest
 * whole number, but at least 1. A reach of 0 steps would make a push move a single control point
 * and the bend measure compare a sample with itself, which happens when the spacing grows on
 * very long routes.
 * --------------------------------------------------------------------------------------------- */
static int count_spacings(float meters, float spacing) {
    int count = (int)roundf(meters / spacing);
    if (count < 1)
        return 1;
    return count;
}

/* ------------------------------------------------------------------------------------------------
 * `begin_smoothing` places the control points along the route and sets the room that the curve
 * wants; the top comment of this file describes the method. The body room is `max_radius` plus
 * `BODY_MARGIN_METERS` (0.5 m). The formation room is `half_width` plus `max_radius` plus
 * `LANE_MARGIN_METERS`. In a narrower passage, the wanted room is half the width of the passage
 * less `PASSAGE_MIDDLE_MARGIN_METERS` (0.25 m), but never less than the body room.
 * Spacings: The control points are at most `CONTROL_SPACING_METERS` (16 m) apart, and the samples
 * about `CURVE_SAMPLE_SPACING_METERS` (4 m), on every route up to `MAX_FULL_ROUTE_METERS`
 * (8 km), so that all routes get the same resolution. `MAX_WAYPOINTS` (sim.h) holds such a
 * route. A longer route gets both spacings scaled up by its length over that limit, so that its
 * points still fit. The reaches of the pushes and of the bend measure are distances, and
 * `begin_smoothing` turns them into counts of control points and of samples at these spacings.
 * --------------------------------------------------------------------------------------------- */
void begin_smoothing(Squad *squad, float max_radius, float half_width, float full_speed_radius,
                     int mover_class) {
    start_smoothing_caches();
    allocate_smoothing_arrays();

    /* Place the control points at their spacing. */
    float total_length = measure_points_length(squad->route, squad->route_count);
    float length_scale = fmaxf(1, total_length / MAX_FULL_ROUTE_METERS);
    float control_spacing = CONTROL_SPACING_METERS * length_scale;
    float sample_spacing = CURVE_SAMPLE_SPACING_METERS * length_scale;
    int control_count = place_control_points(squad, control_spacing, smoothing_control);

    /* Start with the pushes. The bends get widened for a circle that the inner lanes can drive
     * at full speed: they run up to `half_width` inside the route.
     * The reach of the pushes counts control points from the one at or before the spot, while
     * the taper is measured from the spot itself. At the default spacing, the 4 points of
     * `PUSH_SPREAD_METERS` behind the spot cover the whole taper on that side. Ahead, they leave
     * out the fifth point, which the taper would move by 0 to 9.5% of the full step, depending
     * on where the spot lies between two control points. In 100 seeds of the coord mode, that
     * gave 1.4 fewer bumps within squads and 0.1 fewer stops than a reach of 5 points, and
     * reaches of 3 points or fewer drove clearly worse. */
    smoothing = (Smoothing){
        .squad = squad,
        .room = {
            .sample_spacing = sample_spacing,
            .body_clearance = max_radius + BODY_MARGIN_METERS,
            .formation_clearance = half_width + max_radius + LANE_MARGIN_METERS,
            .mover_class = mover_class,
            .push_spread = count_spacings(PUSH_SPREAD_METERS, control_spacing),
            .push_taper_points = PUSH_TAPER_METERS / control_spacing,
            .bend_stencil_samples = count_spacings(BEND_STENCIL_METERS, sample_spacing),
        },
        .phase = SMOOTHING_PUSHES,
        .total_length = total_length,
        .turn_radius = full_speed_radius + half_width,
        .control_count = control_count,
        .stamp_hash = hash_stamped_cells(),
    };
}

/* ------------------------------------------------------------------------------------------------
 * `run_smoothing` runs the steps of the smoothing until it ends or until the steps of this call
 * ran `max_room_tests` room tests that the memo did not hold. It checks the work between steps,
 * so a step that it starts always runs to its end.
 * The memo of the smoothing stays valid between calls, unless the cells that `stamp_parked`
 * changed differ from those of the last call: other squads may have parked or moved in the
 * meantime, and the memo holds tests of the old grid.
 * --------------------------------------------------------------------------------------------- */
bool run_smoothing(int max_room_tests) {
    /* Make the memo stale when the stamped cells changed. */
    uint32_t stamp_hash = hash_stamped_cells();
    if (stamp_hash != smoothing.stamp_hash) {
        start_room_generation();
        smoothing.stamp_hash = stamp_hash;
    }

    /* Run steps while the work of this call stays under the limit. */
    int first_test_count = room_test_count;
    while (smoothing.phase != SMOOTHING_FINISHED) {
        if (room_test_count - first_test_count >= max_room_tests)
            return false;
        if (smoothing.phase == SMOOTHING_PUSHES)
            run_push_round();
        else
            run_widening_pass();
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `get_room_test_count` returns `room_test_count`.
 * --------------------------------------------------------------------------------------------- */
int get_room_test_count(void) {
    return room_test_count;
}

/* ------------------------------------------------------------------------------------------------
 * `smooth_route` runs the whole smoothing at once.
 * --------------------------------------------------------------------------------------------- */
void smooth_route(Squad *squad, float max_radius, float half_width, float full_speed_radius,
                  int mover_class) {
    begin_smoothing(squad, max_radius, half_width, full_speed_radius, mover_class);
    run_smoothing(INT_MAX);
}

/* ------------------------------------------------------------------------------------------------
 * `is_band_clear` returns true when a band of width `2 * half_width` along the straight line from
 * `from` to `to` is clear for `mover_class`. It checks the line itself and the two lines
 * `half_width` to each side of it:
 *
 *      left_from o------------------o left_to
 *           from o------------------o to   ->  the line, from `from` to `to`
 *     right_from o------------------o right_to
 *
 * Edge case: A line shorter than `ROUTE_EPSILON_METERS` counts as clear.
 * --------------------------------------------------------------------------------------------- */
static bool is_band_clear(V2 from, V2 to, float half_width, int mover_class) {
    /* Find the two side lines. */
    V2 line = sub(to, from);
    float line_length = len(line);
    if (line_length < ROUTE_EPSILON_METERS)
        return true;
    V2 right_offset = mul(get_right(line), half_width / line_length);
    V2 left_from = sub(from, right_offset);
    V2 left_to = sub(to, right_offset);
    V2 right_from = add(from, right_offset);
    V2 right_to = add(to, right_offset);

    /* Check the three lines. */
    if (!is_line_clear(from, to, mover_class))
        return false;
    if (!is_line_clear(right_from, right_to, mover_class))
        return false;
    return is_line_clear(left_from, left_to, mover_class);
}

/* ------------------------------------------------------------------------------------------------
 * `cut_corners` moves each corner of the route along the leg after it, as far as a band still
 * reaches it in a straight line from the point before it. The band reaches the radius of the
 * widest member plus `CORNER_CUT_MARGIN_METERS` (8 m) to each side. A band of the
 * radius alone lets the cut legs graze terrain corners, and the formation then squeezes past
 * them. The game route turns
 * at corners of cells, so a start a few meters off can give an L-shaped route. For example, with
 * `--cutcorners off`, the route of the `pass` scenario with seed 66 runs 74 m due west and then
 * 158 m south, where a slanted first leg works. *
 *     before                     before
 *       |                          \
 *       |                           \
 *     corner ---- after    ->        new ---- after
 *                                   (new lies on the old leg from corner to after)
 *
 * A fraction of the leg is a part of its length, from `corner`. The search halves the searched
 * part of the leg `CORNER_CUT_STEPS` times. It takes the band to be clear up to some fraction of
 * the leg and blocked past it. So the new corner lies at most 1/64 of the leg before the furthest
 * point that the band reaches.
 * --------------------------------------------------------------------------------------------- */
static void cut_corners(Squad *squad, float max_radius, int mover_class) {
    float half_width = max_radius + CORNER_CUT_MARGIN_METERS;
    for (int corner_index = 1; corner_index < squad->route_count - 1; corner_index++) {
        /* The band reaches the point at `clear_fraction` of the leg, but not the one at
         * `blocked_fraction`. `shorten_route` kept the corner, so the band does not reach
         * `after`. */
        V2 before = squad->route[corner_index - 1];
        V2 corner = squad->route[corner_index];
        V2 after = squad->route[corner_index + 1];
        V2 leg = sub(after, corner);
        float clear_fraction = 0;
        float blocked_fraction = 1;
        for (int step_index = 0; step_index < CORNER_CUT_STEPS; step_index++) {
            float middle_fraction = (clear_fraction + blocked_fraction) / 2;
            V2 point = add(corner, mul(leg, middle_fraction));
            if (is_band_clear(before, point, half_width, mover_class))
                clear_fraction = middle_fraction;
            else
                blocked_fraction = middle_fraction;
        }
        squad->route[corner_index] = add(corner, mul(leg, clear_fraction));
    }
}

/* ------------------------------------------------------------------------------------------------
 * `shorten_route` skips route points wherever a band as wide as the widest member is clear. From
 * each kept point, it keeps the last later route point that such a band reaches, or else the next
 * point. This removes corners and the back-and-forth steps that the game routes sometimes have.
 * With `--cutcorners on`, `cut_corners` then moves each corner along the leg after it.
 * --------------------------------------------------------------------------------------------- */
void shorten_route(Squad *squad, float max_radius, int mover_class) {
    /* From each kept point, keep the last later point that a band as wide as the widest member
     * reaches, or else the next point. */
    int last_index = squad->route_count - 1;
    int kept_count = 1;
    int from_index = 0;
    while (from_index < last_index) {
        V2 from = squad->route[from_index];
        int to_index = last_index;
        while (to_index > from_index + 1) {
            V2 to = squad->route[to_index];
            if (is_band_clear(from, to, max_radius, mover_class))
                break;
            to_index--;
        }
        squad->route[kept_count] = squad->route[to_index];
        kept_count++;
        from_index = to_index;
    }
    squad->route_count = kept_count;

    /* Cut the corners that are left. */
    if (is_corner_cut_on)
        cut_corners(squad, max_radius, mover_class);
}

/* ------------------------------------------------------------------------------------------------
 * `can_start_route_at` returns true when `point` is on free ground and the straight line from it
 * to `next_point`, the second point of the route, is clear.
 * --------------------------------------------------------------------------------------------- */
static bool can_start_route_at(V2 point, V2 next_point, int mover_class) {
    bool is_passable = gc_get_cell_at(point.x, point.y) <= mover_class;
    return is_passable && is_line_clear(point, next_point, mover_class);
}

/* ------------------------------------------------------------------------------------------------
 * The route of the game starts at the lead. Without `move_route_start`, places laid out around
 * the route start half a formation-width to one side, and the squad swerves as it sets off. The
 * best fit is `center`, the mean position of the members, less the mean offset of the slots,
 * turned to the first leg from `center` to the second point of the route.
 * --------------------------------------------------------------------------------------------- */
void move_route_start(Squad *squad, const float *slots, V2 facing, V2 center, int mover_class) {
    /* Find the mean offset of the slots from the end of the route, with x to the right of
     * `facing`. */
    V2 end = squad->route[squad->route_count - 1];
    V2 mean_offset = v2(0, 0);
    float mean_weight = 1.0f / squad->unit_count;
    for (int slot_index = 0; slot_index < squad->unit_count; slot_index++) {
        V2 slot_offset = get_slot_offset(slots, slot_index, end, facing);
        mean_offset = add(mean_offset, mul(slot_offset, mean_weight));
    }

    /* Turn it to the first leg, and take it from `center`. */
    V2 next_point = squad->route[1];
    V2 first_leg = get_direction(sub(next_point, center), LENGTH_SQUARED_EPSILON, facing);
    V2 best_fit = sub(center, get_map_offset(mean_offset, first_leg));

    /* Start at the best fit, or else at `center`, where the way on is clear. */
    if (can_start_route_at(best_fit, next_point, mover_class))
        squad->route[0] = best_fit;
    else if (can_start_route_at(center, next_point, mover_class))
        squad->route[0] = center;
}

/* ------------------------------------------------------------------------------------------------
 * The search of the game casts a straight line toward the destination and adds corners around the
 * obstacle that the line hits first, so where the route starts decides which way around the
 * obstacles it goes. From the lead, a small obstacle can hide the way down a valley that the
 * center of the members sees. So `take_shorter_center_route` searches a second route from
 * `center`, gives it the same start and shortening as the squad route, and keeps it when it is
 * at least `CENTER_ROUTE_MIN_GAIN_METERS` (10 m) shorter.
 * --------------------------------------------------------------------------------------------- */
void take_shorter_center_route(Squad *squad, const float *slots, V2 facing, V2 center,
                               float max_radius, int mover_class, float clearance) {
    /* Search from `center` like the synchronous searches in the game: the quiet search, and
     * the active one when the quiet one fails. The game has no such second search. A game version
     * of it must run over several ticks, like the first search of the squad, but the simulator
     * lets it finish at once. Parked units are stamped as cliffs, because `set_up_formation` runs
     * while they are. */
    GcRequest search;
    gc_init_request(&search);
    V2 end = squad->destination;
    gc_start_search(&search, GC_SEARCH_QUIET, center.x, center.y, end.x, end.y, mover_class,
                    clearance, CENTER_SEARCH_TOLERANCE_METERS);
    bool is_found = gc_update_search(&search, CENTER_SEARCH_BUDGET) == GC_SEARCH_FOUND;
    if (!is_found) {
        gc_start_search(&search, GC_SEARCH_ACTIVE, center.x, center.y, end.x, end.y,
                        mover_class, clearance, CENTER_SEARCH_TOLERANCE_METERS);
        is_found = gc_update_search(&search, CENTER_SEARCH_BUDGET) == GC_SEARCH_FOUND;
    }
    int point_count = 0;
    if (is_found)
        point_count = gc_get_path(&search, path_points, MAX_WAYPOINTS - 1);
    gc_free_request(&search);
    if (point_count == 0)
        return;

    /* Build the second route in a copy of the squad: `center`, then the found path, which leaves
     * out its own start. Give it the same start and shortening as the squad route. */
    static Squad candidate;
    candidate = *squad;
    candidate.route[0] = center;
    for (int point_index = 0; point_index < point_count; point_index++)
        candidate.route[point_index + 1] = get_pair_point(path_points, point_index);
    candidate.route_count = point_count + 1;
    move_route_start(&candidate, slots, facing, center, mover_class);
    shorten_route(&candidate, max_radius, mover_class);

    /* Keep the shorter route. `set_up_formation` fills `route_length` only later, so measure
     * both routes here. */
    float squad_route_length = measure_points_length(squad->route, squad->route_count);
    float candidate_length = measure_points_length(candidate.route, candidate.route_count);
    float gain = squad_route_length - candidate_length;
    if (gain < CENTER_ROUTE_MIN_GAIN_METERS)
        return;
    size_t route_bytes = (size_t)candidate.route_count * sizeof(V2);
    memcpy(squad->route, candidate.route, route_bytes);
    squad->route_count = candidate.route_count;
}
