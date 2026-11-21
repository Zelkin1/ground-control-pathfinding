// Route smoothing of formation following (`--smooth`). The smoothing replaces the prepared route
// of a squad with a cubic B-spline whose control points start on the route. Control points next to
// blocking terrain move away from it, so that the curve gets wide gentle bends and the lanes stay
// parallel, instead of local bulges at blocking terrain.
//
// Each displacement iteration samples the curve at curve positions,
// `CURVE_POSITION_SPACING_METERS` apart. At each curve position, a clearance test checks the
// wanted clearance (the clearance that the formation needs there) and the body clearance (the
// clearance for the body of the widest squad member). A cramped position, a curve position
// without its wanted clearance, displaces the control points near it away from the closest
// blocking terrain. With `--widebends`, widening iterations then spread bends that are too sharp
// for the top speed of the squad. With `--tightbends`, they also shorten the curve and pull bends
// that are wider than needed to their inner side, while the displacements keep the clearance. A
// curve with a curve position that lacks body clearance leaves the route unchanged.
//
// A smoothing can run in parts over several ticks (`run_smoothing` with a budget of clearance
// tests) and in worker threads, so its state is thread-local. A memo of clearance tests and a
// cache of cosines make repeated tests cheap without changing any result.
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "sim.h"
#include "formation.h"

#define REROUTE_MAX_LENGTH_FACTOR 1.25f              // longest smoothed re-route, share of route
#define CONTROL_SPACING_METERS 16.0f                 // largest control point spacing
#define CONTROL_SPARE_POINTS 8                       // room for end repeats past `MAX_WAYPOINTS`
#define CURVE_POSITION_SPACING_METERS 4.0f           // spacing of the curve positions
#define MAX_FULL_ROUTE_METERS 8000.0f                // longer routes get larger spacings
#define CURVE_PIECE_POINTS 4                         // control points of one B-spline piece
#define START_REPEATS 3                              // copies of the route start
#define END_REPEATS 2                                // copies of the route end
#define FIXED_END_POINTS 3                           // fixed control points at each end
#define SMOOTHING_ITERATIONS 60                      // most displacement iterations
#define BODY_PATIENCE_ITERATIONS 10                  // iterations to gain body clearance
#define BODY_MARGIN_METERS 0.5f                      // body clearance past the widest radius
#define PASSAGE_MIDDLE_MARGIN_METERS 0.25f           // margin to the middle of a passage
#define AWAY_SEARCH_MARGIN_METERS 1.0f               // away search past the wanted clearance
#define DISPLACEMENT_STEP_METERS 0.25f               // move of a displacement at full strength
#define DISPLACEMENT_SPREAD_METERS 64.0f             // reach of a displacement, each way
#define DISPLACEMENT_FADE_METERS 80.0f               // transition length of a displacement
#define WIDEN_ITERATIONS 40                          // widening iterations that move the curve
#define WIDEN_STEP_FRACTION 0.5f                     // excess share removed per iteration
#define WIDEN_PATIENCE_ITERATIONS 30                 // iterations after the last better curve
#define NO_BEST_ITERATION (-1)                       // no better curve yet
#define WIDENING_SECOND_DERIVATIVE_GAIN 6.0f         // second difference per unit widening step
#define BEND_STENCIL_METERS 8.0f                     // reach of a bend measurement, each way
#define TIGHTEN_STEP_FRACTION 0.5f                   // share of the way to the neighbor midpoint
#define TIGHTEN_CURVATURE_FRACTION 0.9f              // tightening limit, share of the widening one
#define TIGHTEN_MIN_MOVE_METERS 0.05f                // smallest move that counts as tightening
#define TIGHTEN_MIN_GAIN_METERS 0.5f                 // shortening that makes a better curve

#define CLEARANCE_MEMO_START_BITS 14                 // first memo size, as a power of 2
#define CLEARANCE_MEMO_MAX_LOAD 0.5f                 // largest filled share before growth
#define CLEARANCE_KEY_WORDS 4                        // key: bits of point x, y, tangent x, y
#define COS_CACHE_BITS 12                            // cosine cache size, as a power of 2
#define COS_CACHE_SIZE (1 << COS_CACHE_BITS)         // entries of the cosine cache
#define HASH_BITS 32                                 // bits of a hash
#define HASH_MULTIPLIER_X 2654435761u                // first 32-bit prime of xxHash
#define HASH_MULTIPLIER_Y 2246822519u                // second 32-bit prime of xxHash
#define HASH_MULTIPLIER_TANGENT_X 3266489917u        // third 32-bit prime of xxHash
#define HASH_MULTIPLIER_TANGENT_Y 668265263u         // fourth 32-bit prime of xxHash


// One entry of the cosine cache: the bits of the angle and its cosine.
typedef struct {
    uint32_t angle_bits;
    bool is_used;
    float cosine;
} CosEntry;


// Clearances and spacings of one smoothing. `displacement_spread` counts control points each way,
// `displacement_fade_points` is the transition length in control points, and
// `bend_stencil_positions` counts curve positions each way.
typedef struct {
    float position_spacing;
    float body_clearance;
    float formation_clearance;
    int terrain_class;
    int displacement_spread;
    float displacement_fade_points;
    int bend_stencil_positions;
} CurveClearance;


// One entry of the clearance memo, an open-addressing hash table with linear probing. An entry of
// an older generation counts as empty, so a new generation empties the memo at once.
typedef struct {
    uint32_t key[CLEARANCE_KEY_WORDS];
    uint32_t generation;
    float wanted;
    bool is_body_clear;
    bool is_wanted_clear;
} ClearanceEntry;


// Phase of a smoothing, in the order in which they run.
typedef enum {
    SMOOTHING_DISPLACEMENTS,
    SMOOTHING_WIDENING,
    SMOOTHING_FINISHED,
} SmoothingPhase;


// State of the smoothing of one squad. `total_length` is the length of the prepared route, and
// `turn_radius` is the smallest radius of the route at which the innermost lane keeps the
// full-speed minimum radius. `lead_in_control_count` counts the first control points, up to the
// last lead-in point of the route. `best_length` is the length of the best curve, which only
// `--tightbends` uses. `stamp_hash` identifies the stamped cells at the start.
typedef struct {
    Squad *squad;
    CurveClearance clearance;
    SmoothingPhase phase;
    float total_length;
    float turn_radius;
    int control_count;
    int lead_in_control_count;
    int curve_count;
    int displacement_iteration_index;
    bool has_clearance;
    bool is_curve_clear;
    float max_curvature;
    float best_excess;
    float best_length;
    int max_cramped_count;
    int widening_iteration_index;
    int best_widening_iteration_index;
    bool is_widening;
    uint32_t stamp_hash;
} Smoothing;

static _Thread_local CosEntry *cos_cache;

static _Thread_local ClearanceEntry *clearance_memo;
static _Thread_local int clearance_memo_bits;
static _Thread_local int clearance_memo_count;
static _Thread_local uint32_t clearance_generation;

// Number of clearance tests of this thread so far, the misses of the memo.
static _Thread_local int clearance_test_count;

static _Thread_local Smoothing smoothing;

// Control points of the curve, of the widened curve, and of the widened curve before its
// displacements, and the control points that the widening may no longer move.
static _Thread_local V2 *smoothing_control;
static _Thread_local V2 *widened;
static _Thread_local V2 *before_displacements;
static _Thread_local bool *is_frozen;

// Curve positions of the best curve so far and of the widened curve.
static _Thread_local V2 *smoothing_curve;
static _Thread_local V2 *widened_curve;


// `find_away_direction` returns the unit direction from the closest point of blocking terrain
// within `radius` to `point`, or zero when no blocking terrain lies that close.
static V2 find_away_direction(V2 point, float radius, int terrain_class) {
    float best_squared = radius * radius;        // smallest squared distance
    V2 closest = point;
    int min_cell_y = get_cell_coordinate(point.y - radius);
    int max_cell_y = get_cell_coordinate(point.y + radius);
    int min_cell_x = get_cell_coordinate(point.x - radius);
    int max_cell_x = get_cell_coordinate(point.x + radius);
    for (int cell_y = min_cell_y; cell_y <= max_cell_y; cell_y++) {
        for (int cell_x = min_cell_x; cell_x <= max_cell_x; cell_x++) {
            if (gc_get_cell(cell_x, cell_y) <= terrain_class)
                continue;
            V2 cell_point = get_closest_cell_point(cell_x, cell_y, point);
            float d_squared = length_squared(sub(cell_point, point));
            if (d_squared < best_squared) {
                best_squared = d_squared;
                closest = cell_point;
            }
        }
    }

    V2 away = sub(point, closest);
    return get_direction(away, LENGTH_SQUARED_EPSILON, v2(0, 0));
}


// `compute_bspline_weights` computes the weights of the four control points of a uniform cubic
// B-spline piece at `fraction` of the piece, and in `slopes`, their derivatives by `fraction`.
static void compute_bspline_weights(float fraction, float weights[CURVE_PIECE_POINTS],
                                    float slopes[CURVE_PIECE_POINTS]) {
    float t_squared = fraction * fraction;
    float t_cubed = t_squared * fraction;
    float rest = 1 - fraction;
    weights[0] = rest * rest * rest / 6;
    weights[1] = (3 * t_cubed - 6 * t_squared + 4) / 6;
    weights[2] = (-3 * t_cubed + 3 * t_squared + 3 * fraction + 1) / 6;
    weights[3] = t_cubed / 6;

    slopes[0] = -rest * rest / 2;
    slopes[1] = (3 * t_squared - 4 * fraction) / 2;
    slopes[2] = (-3 * t_squared + 2 * fraction + 1) / 2;
    slopes[3] = t_squared / 2;
}


// `evaluate_bspline` returns the point of the piece `piece_index` of the curve at `fraction`, and
// in `tangent`, its derivative by `fraction`. The piece uses the control points from
// `piece_index` to `piece_index + 3`.
static V2 evaluate_bspline(const V2 *control, int piece_index, float fraction, V2 *tangent) {
    float weights[CURVE_PIECE_POINTS];
    float slopes[CURVE_PIECE_POINTS];
    compute_bspline_weights(fraction, weights, slopes);

    V2 point = v2(0, 0);
    *tangent = v2(0, 0);
    for (int weight_index = 0; weight_index < CURVE_PIECE_POINTS; weight_index++) {
        V2 control_point = control[piece_index + weight_index];
        point = add(point, mul(control_point, weights[weight_index]));
        *tangent = add(*tangent, mul(control_point, slopes[weight_index]));
    }
    return point;
}


// `read_float_bits` returns the bits of `value`.
static uint32_t read_float_bits(float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}


// `get_cached_cos` returns the cosine of `angle` from the cosine cache, or computes it and stores
// it in the slot of the hash of its bits. The cache only replaces calls with the same result.
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


// `displace_control_points` moves the control points around a cramped position along `away`, by
// `DISPLACEMENT_STEP_METERS` times a raised cosine of their distance from it. `control_coordinate`
// places the cramped position among the control points, with a fraction.
static void displace_control_points(V2 *control, int control_count,
                                    const CurveClearance *clearance, float control_coordinate,
                                    V2 away) {
    // The first and the last `FIXED_END_POINTS` control points keep the curve on the route ends.
    int cramped_control_index = (int)control_coordinate;
    int first_index = cramped_control_index - clearance->displacement_spread;
    int last_index = cramped_control_index + clearance->displacement_spread;
    for (int control_index = first_index; control_index <= last_index; control_index++) {
        bool is_at_start = control_index < FIXED_END_POINTS;
        bool is_at_end = control_index >= control_count - FIXED_END_POINTS;
        if (is_at_start || is_at_end)
            continue;
        float d = fabsf(control_index - control_coordinate);        // distance in control points
        float spread_points = d / clearance->displacement_fade_points;
        float spread = fminf(1, spread_points);
        float fade = 0.5f * (1 + get_cached_cos(PI * spread));
        V2 displacement = mul(away, DISPLACEMENT_STEP_METERS * fade);
        control[control_index] = add(control[control_index], displacement);
    }
}


// `put_control_points` lays control points along the route of a squad, at most `control_spacing`
// apart, and returns their number. The repeated end points make the curve start and end on the
// route ends. `lead_in_control_count` receives the number of control points up to the last
// lead-in point, or 0 without lead-in points.
static int put_control_points(const Squad *squad, float control_spacing, V2 *control,
                              int *lead_in_control_count) {
    int count = 0;        // number of control points
    for (int repeat_index = 0; repeat_index < START_REPEATS; repeat_index++) {
        control[count] = squad->route[0];
        count++;
    }

    // `CONTROL_SPARE_POINTS` leaves room for the end repeats after `MAX_WAYPOINTS`.
    *lead_in_control_count = 0;
    for (int point_index = 1; point_index < squad->route_count; point_index++) {
        if (count >= MAX_WAYPOINTS)
            break;
        V2 from = squad->route[point_index - 1];
        V2 segment = sub(squad->route[point_index], from);
        int pieces = (int)ceilf(len(segment) / control_spacing);        // pieces of the segment
        for (int piece_index = 1; piece_index <= pieces; piece_index++) {
            if (count >= MAX_WAYPOINTS)
                break;
            float end_fraction = (float)piece_index / pieces;
            control[count] = add(from, mul(segment, end_fraction));
            count++;
        }
        if (point_index == squad->lead_in_point_count)
            *lead_in_control_count = count;
    }

    V2 route_end = squad->route[squad->route_count - 1];
    for (int repeat_index = 0; repeat_index < END_REPEATS; repeat_index++) {
        control[count] = route_end;
        count++;
    }
    return count;
}


// `find_wanted_clearance` returns the wanted clearance at a curve position: the formation
// clearance, or in a narrow passage, half the passage width minus a margin, so that the curve runs
// down the middle, but at least the body clearance.
static float find_wanted_clearance(V2 point, V2 tangent, const CurveClearance *clearance) {
    // The rays stop once they measure a passage wide enough for the formation clearance, which
    // keeps the result exact, because free distances are multiples of the probe step.
    float formation_width = 2 * (clearance->formation_clearance + PASSAGE_MIDDLE_MARGIN_METERS);
    float wide_width = formation_width + FREE_DISTANCE_STEP_METERS;

    // When the free distance to the right alone holds the formation clearance in its middle, the
    // left ray is not needed. Each ray reaches at least half the wide width, so that a passage of
    // that width counts as wide also for a formation clearance above `LANE_MAX_DISTANCE_METERS`.
    V2 direction = get_direction(tangent, LENGTH_SQUARED_EPSILON, v2(1, 0));
    V2 right = get_right(direction);
    V2 left = mul(right, -1);
    float ray_limit = fmaxf(LANE_MAX_DISTANCE_METERS, wide_width / 2);
    float right_limit = fminf(wide_width, ray_limit);
    float right_free = measure_free_distance(point, right, clearance->terrain_class, right_limit);
    float right_clearance = right_free / 2 - PASSAGE_MIDDLE_MARGIN_METERS;
    if (right_clearance >= clearance->formation_clearance)
        return clearance->formation_clearance;

    float left_limit = fminf(wide_width - right_free, ray_limit);
    float left_free = measure_free_distance(point, left, clearance->terrain_class, left_limit);
    float passage_width = right_free + left_free;
    float passage_clearance = passage_width / 2 - PASSAGE_MIDDLE_MARGIN_METERS;
    return clamp_float(passage_clearance, clearance->body_clearance,
                       clearance->formation_clearance);
}


// `test_position_clearance` runs the clearance test of a curve position into `entry`. The body
// circle needs no test when the wanted circle, which is at least as large, is clear.
static void test_position_clearance(V2 point, V2 tangent, const CurveClearance *clearance,
                                    ClearanceEntry *entry) {
    entry->wanted = find_wanted_clearance(point, tangent, clearance);
    entry->is_wanted_clear = is_circle_clear(point, entry->wanted, clearance->terrain_class);
    entry->is_body_clear = entry->is_wanted_clear;
    bool must_test_body = !entry->is_wanted_clear && entry->wanted > clearance->body_clearance;
    if (must_test_body)
        entry->is_body_clear = is_circle_clear(point, clearance->body_clearance,
                                               clearance->terrain_class);
}


// `start_clearance_generation` empties the clearance memo by starting a new generation.
static void start_clearance_generation(void) {
    clearance_generation++;
    clearance_memo_count = 0;
}


// `hash_clearance_key` returns the hash of a key of the clearance memo, a multiplicative hash of
// its four words.
static uint32_t hash_clearance_key(const uint32_t *key) {
    uint32_t point_hash = key[0] * HASH_MULTIPLIER_X ^ key[1] * HASH_MULTIPLIER_Y;
    uint32_t tangent_x_hash = key[2] * HASH_MULTIPLIER_TANGENT_X;
    uint32_t tangent_y_hash = key[3] * HASH_MULTIPLIER_TANGENT_Y;
    return point_hash ^ tangent_x_hash ^ tangent_y_hash;
}


// `find_clearance_entry` returns the entry of the clearance memo for `key`, or the empty entry
// where `key` belongs. The top bits of the hash give the first slot to probe.
static ClearanceEntry *find_clearance_entry(const uint32_t *key) {
    // The memo never fills up, so the probing always ends.
    uint32_t index_mask = (1u << clearance_memo_bits) - 1;
    uint32_t entry_index = hash_clearance_key(key) >> (HASH_BITS - clearance_memo_bits);
    size_t key_bytes = CLEARANCE_KEY_WORDS * sizeof *key;
    while (true) {
        ClearanceEntry *entry = &clearance_memo[entry_index];
        bool is_stale = entry->generation != clearance_generation;
        if (is_stale || memcmp(entry->key, key, key_bytes) == 0)
            return entry;
        entry_index = (entry_index + 1) & index_mask;
    }
}


// `grow_clearance_memo` doubles the size of the clearance memo and moves the entries of the
// current generation into it.
static void grow_clearance_memo(void) {
    ClearanceEntry *old_memo = clearance_memo;
    int old_size = 1 << clearance_memo_bits;
    clearance_memo_bits++;
    clearance_memo = calloc((size_t)1 << clearance_memo_bits, sizeof *clearance_memo);
    for (int entry_index = 0; entry_index < old_size; entry_index++) {
        const ClearanceEntry *old_entry = &old_memo[entry_index];
        if (old_entry->generation != clearance_generation)
            continue;
        ClearanceEntry *new_entry = find_clearance_entry(old_entry->key);
        *new_entry = *old_entry;
    }
    free(old_memo);
}


// `find_position_clearance` returns the clearance test of a curve position, from the memo when
// the same point and tangent were tested in this generation. Each miss runs a test and counts it.
static const ClearanceEntry *find_position_clearance(V2 point, V2 tangent,
                                                     const CurveClearance *clearance) {
    uint32_t key[CLEARANCE_KEY_WORDS] = {
        read_float_bits(point.x),
        read_float_bits(point.y),
        read_float_bits(tangent.x),
        read_float_bits(tangent.y),
    };
    ClearanceEntry *entry = find_clearance_entry(key);
    if (entry->generation == clearance_generation)
        return entry;

    // The memo grows before it would be more than `CLEARANCE_MEMO_MAX_LOAD` full.
    float max_count = CLEARANCE_MEMO_MAX_LOAD * (1 << clearance_memo_bits);
    if (clearance_memo_count + 1 > max_count) {
        grow_clearance_memo();
        entry = find_clearance_entry(key);
    }
    entry->generation = clearance_generation;
    memcpy(entry->key, key, sizeof key);
    test_position_clearance(point, tangent, clearance, entry);
    clearance_memo_count++;
    clearance_test_count++;
    return entry;
}


// `start_smoothing_caches` allocates the clearance memo and the cosine cache of the thread once,
// and empties the memo for a new smoothing.
static void start_smoothing_caches(void) {
    if (clearance_memo == NULL) {
        clearance_memo_bits = CLEARANCE_MEMO_START_BITS;
        clearance_memo = calloc((size_t)1 << clearance_memo_bits, sizeof *clearance_memo);
        cos_cache = calloc(COS_CACHE_SIZE, sizeof *cos_cache);
    }
    start_clearance_generation();
}


// `round_up_to_int` returns the smallest integer at or above a positive `value`, as a faster
// replacement of `ceilf` of MinGW with the same results.
static int round_up_to_int(float value) {
    int truncated = (int)value;
    if ((float)truncated < value)
        return truncated + 1;
    return truncated;
}


// `displace_curve_clear` runs one displacement iteration: it samples the curve of `control` into
// `curve`, and each cramped position displaces the control points near it. It returns true when
// every curve position has its wanted clearance, and `is_curve_clear` tells whether each has body
// clearance.
static bool displace_curve_clear(V2 *control, int control_count, const CurveClearance *clearance,
                                 V2 *curve, int *curve_count, bool *is_curve_clear) {
    int terrain_class = clearance->terrain_class;
    bool has_clearance = true;
    *is_curve_clear = true;
    *curve_count = 0;
    int pieces = control_count - CURVE_PIECE_POINTS + 1;        // number of curve pieces
    for (int piece_index = 0; piece_index < pieces; piece_index++) {
        // A piece runs near the chord between its two middle control points, and the number of
        // its curve positions follows the chord length.
        int start_index = piece_index + 1;
        int end_index = piece_index + 2;
        V2 chord = sub(control[end_index], control[start_index]);
        float chord_length = len(chord);
        int count = 1;        // curve positions of the piece
        if (chord_length > 0)
            count = round_up_to_int(chord_length / clearance->position_spacing);
        for (int position_index = 0; position_index < count; position_index++) {
            // The last curve position stays free for the route end.
            if (*curve_count >= MAX_WAYPOINTS - 1)
                break;
            float fraction = (float)position_index / count;
            V2 tangent;
            V2 point = evaluate_bspline(control, piece_index, fraction, &tangent);
            curve[*curve_count] = point;
            (*curve_count)++;

            // A cramped position displaces the control points at once, so later curve positions
            // of the same iteration already read the moved curve.
            const ClearanceEntry *test = find_position_clearance(point, tangent, clearance);
            if (test->is_body_clear && test->is_wanted_clear)
                continue;
            *is_curve_clear &= test->is_body_clear;
            has_clearance = false;
            float search_radius = test->wanted + AWAY_SEARCH_MARGIN_METERS;
            V2 away = find_away_direction(point, search_radius, terrain_class);
            float control_coordinate = start_index + fraction;
            displace_control_points(control, control_count, clearance, control_coordinate, away);
        }
    }
    return has_clearance;
}


// `measure_polygon_curvature` returns the curvature of the control polygon at an inner control
// point, and its second difference in `d2`. A control point between two coinciding neighbors has
// the curvature 0.
static float measure_polygon_curvature(const V2 *control, int control_index, V2 *d2) {
    // The curvature of the polygon is `|d1 x d2| / |d1|^3`, from its first and second
    // differences `d1` and `d2` at the control point.
    V2 before = control[control_index - 1];
    V2 here = control[control_index];
    V2 after = control[control_index + 1];
    V2 d1 = mul(sub(after, before), 0.5f);
    *d2 = add(sub(before, here), sub(after, here));
    float s = len(d1);        // length of the first difference
    if (s < ROUTE_EPSILON_METERS)
        return 0;
    float bend = fabsf(det2(d1, *d2));
    return bend / (s * s * s);
}


// `widen_bends` runs one widening iteration: each free control point where the curvature of the
// control polygon exceeds `max_curvature` moves to take away `WIDEN_STEP_FRACTION` of the excess.
// It returns true when any bend was too sharp.
static bool widen_bends(V2 *control, int control_count, float max_curvature,
                        const bool *is_frozen) {
    // `widening_steps` holds the widening step of each control point. The fixed ends and the
    // frozen control points have none.
    static _Thread_local V2 *widening_steps;
    if (!widening_steps)
        widening_steps = malloc(sizeof(V2) * (MAX_WAYPOINTS + CONTROL_SPARE_POINTS));
    bool is_any_tight = false;
    int first_free_index = FIXED_END_POINTS;
    int end_free_index = control_count - FIXED_END_POINTS;
    for (int control_index = 0; control_index < control_count; control_index++) {
        widening_steps[control_index] = v2(0, 0);
        bool is_fixed = control_index < first_free_index || control_index >= end_free_index;
        if (is_fixed || is_frozen[control_index])
            continue;
        V2 d2;
        float curvature = measure_polygon_curvature(control, control_index, &d2);
        if (curvature <= max_curvature)
            continue;
        is_any_tight = true;
        float excess_fraction = (curvature - max_curvature) / curvature;
        widening_steps[control_index] = mul(d2, excess_fraction);
    }

    // The move `2 * s_i - s_(i-1) - s_(i+1)` changes the second difference at a control point by
    // `-6 * s_i`, which `WIDENING_SECOND_DERIVATIVE_GAIN` cancels.
    float move_scale = WIDEN_STEP_FRACTION / WIDENING_SECOND_DERIVATIVE_GAIN;
    for (int control_index = first_free_index; control_index < end_free_index; control_index++) {
        if (is_frozen[control_index])
            continue;
        V2 before_step = widening_steps[control_index - 1];
        V2 after_step = widening_steps[control_index + 1];
        V2 neighbor_steps = add(before_step, after_step);
        V2 change = sub(mul(widening_steps[control_index], 2), neighbor_steps);
        control[control_index] = add(control[control_index], mul(change, move_scale));
    }
    return is_any_tight;
}


// `is_tighter_than_allowed` tells whether the control polygon at a control point turns tighter
// than `max_curvature` and tighter than `old_curvature`, its curvature before a move.
static bool is_tighter_than_allowed(const V2 *control, int control_index, float max_curvature,
                                    float old_curvature) {
    V2 unused;
    float curvature = measure_polygon_curvature(control, control_index, &unused);
    return curvature > fmaxf(max_curvature, old_curvature);
}


// `tighten_bends` runs one tightening iteration (`--tightbends`): each free control point moves
// part of the way to the midpoint of its neighbors, which shortens the curve and pulls bends to
// their inner side. It returns true when a control point moved `TIGHTEN_MIN_MOVE_METERS` or more.
static bool tighten_bends(V2 *control, int control_count, float max_curvature,
                          const bool *is_frozen) {
    // A move that would turn the polygon tighter than `max_curvature`, at the control point or
    // at one of its two neighbors, is undone, so the curve gets no kink where it is held back.
    bool has_moved = false;
    float min_move_squared = TIGHTEN_MIN_MOVE_METERS * TIGHTEN_MIN_MOVE_METERS;
    int end_free_index = control_count - FIXED_END_POINTS;
    for (int control_index = FIXED_END_POINTS; control_index < end_free_index; control_index++) {
        if (is_frozen[control_index])
            continue;
        V2 here = control[control_index];
        V2 neighbor_sum = add(control[control_index - 1], control[control_index + 1]);
        V2 to_midpoint = sub(mul(neighbor_sum, 0.5f), here);
        V2 move = mul(to_midpoint, TIGHTEN_STEP_FRACTION);
        if (length_squared(move) < min_move_squared)
            continue;

        // The curvatures before the move count for the three control points that it bends.
        float old_curvatures[3];
        for (int offset = -1; offset <= 1; offset++) {
            V2 unused;
            int index = control_index + offset;        // control point bent by the move
            old_curvatures[offset + 1] = measure_polygon_curvature(control, index, &unused);
        }
        control[control_index] = add(here, move);
        bool is_too_tight = false;
        for (int offset = -1; offset <= 1; offset++) {
            int index = control_index + offset;        // control point bent by the move
            float old_curvature = old_curvatures[offset + 1];
            is_too_tight |= is_tighter_than_allowed(control, index, max_curvature, old_curvature);
        }
        if (is_too_tight)
            control[control_index] = here;
        else
            has_moved = true;
    }
    return has_moved;
}


// `measure_bend_excess` returns the bend excess of a curve: the sum over the curve positions of
// the curvature above `max_curvature`, times the length of the next curve step. The curvature
// comes from the circle through the curve positions `stencil_positions` before, at, and after.
static float measure_bend_excess(const V2 *curve, int curve_count, float max_curvature,
                                 int stencil_positions) {
    // The circle through three points has the curvature `4 * area / (a * b * c)`, from the area
    // of their triangle and its three side lengths.
    float excess = 0;
    int end_index = curve_count - stencil_positions;
    for (int position_index = stencil_positions; position_index < end_index; position_index++) {
        V2 before = curve[position_index - stencil_positions];
        V2 here = curve[position_index];
        V2 after = curve[position_index + stencil_positions];
        V2 to_here = sub(here, before);
        V2 to_after = sub(after, before);
        float sides = len(to_here) * len(sub(after, here)) * len(to_after);        // side product
        if (sides < ROUTE_EPSILON_METERS)
            continue;
        float curvature = 2 * fabsf(det2(to_here, to_after)) / sides;
        if (curvature <= max_curvature)
            continue;

        float step = len(sub(curve[position_index + 1], here));
        excess += (curvature - max_curvature) * step;
    }
    return excess;
}


// `count_cramped_positions` returns the number of inner curve positions of a curve without their
// wanted clearance, and in `is_body_clear`, whether every inner curve position has body clearance.
static int count_cramped_positions(const V2 *curve, int curve_count,
                                   const CurveClearance *clearance, bool *is_body_clear) {
    // The tangent here is the chord between the two neighbors, so these tests rarely match the
    // memo entries of the displacement iterations.
    int count = 0;        // number of cramped positions
    *is_body_clear = true;
    for (int position_index = 1; position_index < curve_count - 1; position_index++) {
        V2 point = curve[position_index];
        V2 tangent = sub(curve[position_index + 1], curve[position_index - 1]);
        const ClearanceEntry *test = find_position_clearance(point, tangent, clearance);
        if (!test->is_body_clear)
            *is_body_clear = false;
        if (!test->is_wanted_clear)
            count++;
    }
    return count;
}


// `allocate_smoothing_arrays` allocates the control points and curve positions of the thread
// once. The control points need room for the end repeats past `MAX_WAYPOINTS`.
static void allocate_smoothing_arrays(void) {
    if (smoothing_control)
        return;
    size_t control_capacity = MAX_WAYPOINTS + CONTROL_SPARE_POINTS;
    smoothing_control = malloc(sizeof(V2) * control_capacity);
    widened = malloc(sizeof(V2) * control_capacity);
    before_displacements = malloc(sizeof(V2) * control_capacity);
    is_frozen = malloc(sizeof(bool) * control_capacity);
    smoothing_curve = malloc(sizeof(V2) * MAX_WAYPOINTS);
    widened_curve = malloc(sizeof(V2) * MAX_WAYPOINTS);
}


// `is_hairpin_curve` tells whether the curve of a re-routing squad is more than
// `REROUTE_MAX_LENGTH_FACTOR` times as long as its prepared route. A displacement could otherwise
// turn a start beside a parking disc into a backward hairpin.
static bool is_hairpin_curve(const Squad *squad) {
    if (!squad->is_rerouting)
        return false;
    float max_length = REROUTE_MAX_LENGTH_FACTOR * smoothing.total_length;
    float curve_length = measure_points_length(smoothing_curve, smoothing.curve_count);
    return curve_length > max_length;
}


// `finish_smoothing` ends the smoothing and replaces the route of the squad with the best curve,
// which ends on the route end, unless the curve is a hairpin.
static void finish_smoothing(void) {
    Squad *squad = smoothing.squad;
    smoothing.phase = SMOOTHING_FINISHED;
    smoothing_curve[smoothing.curve_count] = squad->route[squad->route_count - 1];
    smoothing.curve_count++;
    if (is_hairpin_curve(squad))
        return;
    memcpy(squad->route, smoothing_curve, sizeof(V2) * smoothing.curve_count);
    squad->route_count = smoothing.curve_count;
}


// `start_widening` starts the widening phase from the curve of the displacement iterations. A
// widened curve must reduce the bend excess of that curve, or with `--tightbends` keep it and be
// shorter, without more cramped positions.
static void start_widening(void) {
    float max_curvature = 1 / smoothing.turn_radius;
    int curve_count = smoothing.curve_count;
    bool is_first_body_clear = false;
    smoothing.max_curvature = max_curvature;
    int stencil_positions = smoothing.clearance.bend_stencil_positions;
    smoothing.best_excess = measure_bend_excess(smoothing_curve, curve_count, max_curvature,
                                                stencil_positions);
    smoothing.best_length = measure_points_length(smoothing_curve, curve_count);
    smoothing.max_cramped_count = count_cramped_positions(smoothing_curve, curve_count,
                                                          &smoothing.clearance,
                                                          &is_first_body_clear);
    memcpy(widened, smoothing_control, sizeof(V2) * smoothing.control_count);
    memset(is_frozen, 0, sizeof(bool) * smoothing.control_count);

    // The lead-in turn is tighter than the bend limit of the route on purpose. Without the freeze,
    // the widening would move it outward, and the squad would turn wider than its new order needs.
    for (int control_index = 0; control_index < smoothing.lead_in_control_count; control_index++)
        is_frozen[control_index] = true;
    smoothing.is_widening = true;
    smoothing.widening_iteration_index = 0;
    smoothing.best_widening_iteration_index = NO_BEST_ITERATION;
    smoothing.phase = SMOOTHING_WIDENING;
}


// `end_displacements` ends the displacement phase. A curve without body clearance everywhere ends
// the smoothing and leaves the route unchanged.
static void end_displacements(void) {
    if (!smoothing.is_curve_clear) {
        smoothing.phase = SMOOTHING_FINISHED;
        return;
    }
    if (is_wide_bend_on)
        start_widening();
    else
        finish_smoothing();
}


// `run_displacement_iteration` runs one displacement iteration on the curve. The phase ends when
// the curve has its wanted clearance everywhere, or after `SMOOTHING_ITERATIONS`. With
// `--smoothstop`, it also ends when the curve still lacks body clearance after
// `BODY_PATIENCE_ITERATIONS`, which leaves the route unchanged.
static void run_displacement_iteration(void) {
    smoothing.has_clearance = displace_curve_clear(smoothing_control, smoothing.control_count,
                                                   &smoothing.clearance, smoothing_curve,
                                                   &smoothing.curve_count,
                                                   &smoothing.is_curve_clear);
    smoothing.displacement_iteration_index++;
    int iteration_count = smoothing.displacement_iteration_index;

    // In the test runs, curves that ended with body clearance had it after 5 iterations, except
    // for rare curves that lost and gained it by turns. A curve that still lacks it later would
    // nearly always fail after all iterations, so the stop saves their cost.
    bool is_patience_over = iteration_count >= BODY_PATIENCE_ITERATIONS;
    bool is_hopeless = is_smoothing_stop_on && is_patience_over && !smoothing.is_curve_clear;
    bool is_out_of_iterations = iteration_count >= SMOOTHING_ITERATIONS;
    if (smoothing.has_clearance || is_out_of_iterations || is_hopeless)
        end_displacements();
}


// `is_better_curve` tells whether a widened curve with the bend excess `excess` and the length
// `length` is better than the best curve: a smaller bend excess, or with `--tightbends` the same
// or a smaller one and a curve shorter by `TIGHTEN_MIN_GAIN_METERS`.
static bool is_better_curve(float excess, float length) {
    if (excess < smoothing.best_excess)
        return true;
    bool is_shorter = length < smoothing.best_length - TIGHTEN_MIN_GAIN_METERS;
    return is_tight_bend_on && excess <= smoothing.best_excess && is_shorter;
}


// `run_widening_iteration` runs one widening iteration: it widens the bends (and tightens them
// with `--tightbends`), displaces the widened curve away from blocking terrain, and keeps the
// widened curve as the best curve when it is better. Control points that a displacement moved
// are frozen for the widening.
static void run_widening_iteration(void) {
    // Widening steps stop after `WIDEN_ITERATIONS`, but the displacements go on. The tightening
    // stays below `max_curvature`, so that it does not undo the widening.
    int control_count = smoothing.control_count;
    float max_curvature = smoothing.max_curvature;
    bool has_steps_left = smoothing.widening_iteration_index < WIDEN_ITERATIONS;
    bool may_widen = smoothing.is_widening && has_steps_left;
    if (may_widen)
        smoothing.is_widening = widen_bends(widened, control_count, max_curvature, is_frozen);
    else
        smoothing.is_widening = false;
    if (may_widen && is_tight_bend_on) {
        float tight_curvature = TIGHTEN_CURVATURE_FRACTION * max_curvature;
        smoothing.is_widening |= tighten_bends(widened, control_count, tight_curvature, is_frozen);
    }

    // Without the freeze, the widening could undo the displacements in each iteration.
    size_t control_bytes = sizeof(V2) * control_count;
    memcpy(before_displacements, widened, control_bytes);
    int widened_count = 0;
    bool is_widened_clear = false;
    const CurveClearance *clearance = &smoothing.clearance;
    bool has_widened_clearance = displace_curve_clear(widened, control_count, clearance,
                                                      widened_curve, &widened_count,
                                                      &is_widened_clear);
    for (int control_index = 0; control_index < control_count; control_index++) {
        V2 displacement = sub(widened[control_index], before_displacements[control_index]);
        if (length_squared(displacement) > 0)
            is_frozen[control_index] = true;
    }

    // A better curve has a smaller bend excess, body clearance everywhere, and no more cramped
    // positions than the curve before the widening.
    float excess = measure_bend_excess(widened_curve, widened_count, max_curvature,
                                       clearance->bend_stencil_positions);
    float length = 0;
    if (is_tight_bend_on)
        length = measure_points_length(widened_curve, widened_count);
    if (is_better_curve(excess, length)) {
        bool is_body_clear = false;
        int cramped_count = count_cramped_positions(widened_curve, widened_count, clearance,
                                                    &is_body_clear);
        if (is_body_clear && cramped_count <= smoothing.max_cramped_count) {
            smoothing.best_excess = excess;
            smoothing.best_length = length;
            memcpy(smoothing_curve, widened_curve, sizeof(V2) * widened_count);
            smoothing.curve_count = widened_count;
            smoothing.best_widening_iteration_index = smoothing.widening_iteration_index;
        }
    }

    // The phase ends when the widening found no sharp bend and the displacements cleared the
    // curve, `WIDEN_PATIENCE_ITERATIONS` after the last better curve, or after all iterations.
    int best_index = smoothing.best_widening_iteration_index;
    int stale_count = smoothing.widening_iteration_index - best_index;
    bool is_stale = stale_count >= WIDEN_PATIENCE_ITERATIONS;
    smoothing.widening_iteration_index++;
    bool has_ended = !smoothing.is_widening && has_widened_clearance;
    int max_count = WIDEN_ITERATIONS + SMOOTHING_ITERATIONS;        // most widening iterations
    bool is_out_of_iterations = smoothing.widening_iteration_index >= max_count;
    if (has_ended || is_stale || is_out_of_iterations)
        finish_smoothing();
}


// `count_spacings` returns the number of spacings in `meters`, rounded to the nearest whole
// number, and at least 1.
static int count_spacings(float meters, float spacing) {
    int count = (int)roundf(meters / spacing);
    if (count < 1)
        return 1;
    return count;
}


// `begin_smoothing` starts the smoothing of the route of a squad in this thread. On a route longer
// than `MAX_FULL_ROUTE_METERS`, the spacings grow with the route length. The widening limits the
// bend radius to `full_speed_min_radius + half_width`, so that the innermost lane keeps it.
void begin_smoothing(Squad *squad, float max_radius, float half_width, float full_speed_min_radius,
                     int terrain_class) {
    start_smoothing_caches();
    allocate_smoothing_arrays();
    float total_length = measure_points_length(squad->route, squad->route_count);
    float length_scale = fmaxf(1, total_length / MAX_FULL_ROUTE_METERS);
    float control_spacing = CONTROL_SPACING_METERS * length_scale;
    float position_spacing = CURVE_POSITION_SPACING_METERS * length_scale;
    int lead_in_control_count;
    int control_count = put_control_points(squad, control_spacing, smoothing_control,
                                           &lead_in_control_count);

    // `stamp_hash` lets `run_smoothing` notice a change of the stamped parked squads.
    smoothing = (Smoothing){
        .squad = squad,
        .clearance = {
            .position_spacing = position_spacing,
            .body_clearance = max_radius + BODY_MARGIN_METERS,
            .formation_clearance = half_width + max_radius + LANE_MARGIN_METERS,
            .terrain_class = terrain_class,
            .displacement_spread = count_spacings(DISPLACEMENT_SPREAD_METERS, control_spacing),
            .displacement_fade_points = DISPLACEMENT_FADE_METERS / control_spacing,
            .bend_stencil_positions = count_spacings(BEND_STENCIL_METERS, position_spacing),
        },
        .phase = SMOOTHING_DISPLACEMENTS,
        .total_length = total_length,
        .turn_radius = full_speed_min_radius + half_width,
        .control_count = control_count,
        .lead_in_control_count = lead_in_control_count,
        .stamp_hash = hash_stamped_cells(),
    };
}


// `run_smoothing` runs the smoothing of this thread until it finishes, or until it has run
// `max_clearance_tests` clearance tests in this call. It returns true when the smoothing finished.
bool run_smoothing(int max_clearance_tests) {
    // Memo entries from before a change of the stamped cells would hold stale clearance tests.
    uint32_t stamp_hash = hash_stamped_cells();
    if (stamp_hash != smoothing.stamp_hash) {
        start_clearance_generation();
        smoothing.stamp_hash = stamp_hash;
    }

    // An iteration that starts within the budget runs to its end, so a call can exceed it.
    int first_test_count = clearance_test_count;
    while (smoothing.phase != SMOOTHING_FINISHED) {
        if (clearance_test_count - first_test_count >= max_clearance_tests)
            return false;
        if (smoothing.phase == SMOOTHING_DISPLACEMENTS)
            run_displacement_iteration();
        else
            run_widening_iteration();
    }
    return true;
}


// `get_clearance_test_count` returns the number of clearance tests of this thread so far.
int get_clearance_test_count(void) {
    return clearance_test_count;
}


// `smooth_route` runs the whole smoothing of the route of a squad in one call.
void smooth_route(Squad *squad, float max_radius, float half_width, float full_speed_min_radius,
                  int terrain_class) {
    begin_smoothing(squad, max_radius, half_width, full_speed_min_radius, terrain_class);
    run_smoothing(INT_MAX);
}
