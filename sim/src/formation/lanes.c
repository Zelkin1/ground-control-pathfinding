/* lanes.c lays out the lanes of formation following and reads them.
 *
 * Callers: formation.c calls `set_up_lanes` from `set_up_formation` when a squad gets its route,
 *   and reads the lanes with `get_lane_point` and `do_lanes_fit` while the squad moves. metrics.c
 *   reads them with `get_lane` and `get_lane_point` for the trace.
 *
 * Terms: Each member drives along its own lane, a line beside the squad route. A lane sample (a
 *   `Lane`) holds the lane values at one point of the route, and the samples are the lane spacing
 *   apart (`get_lane_spacing`). A slot x is the sideways offset of a slot from the anchor,
 *   `place_offset.x`. The sideways offset of a lane from the route is its lane x. The range of a
 *   sample is the span of lane x that the lanes of the outermost slots must stay in, so that
 *   their bodies keep clear of blocked ground. The shift of a sample (`Lane.shift`) is the
 *   sideways offset that all lanes of the sample share. It is not the shift of a squad around a
 *   meeting or a pass (`side_offset`, read with `get_side_shift`), which `get_lane_point` adds
 *   later.
 *
 * Layout: At each sample, `set_up_lanes` measures the free distance to the left and right of the
 *   route, across the direction of the formation. It fits the slot x values of the members into
 *   that width. It shifts the slot x values away from blocked ground on one side, or squeezes
 *   them together down to two radii of the widest member plus `SQUEEZED_GAP_METERS` apart. Where
 *   even that does not fit, the squad drives in single file down the middle of the passage.
 *
 * Order: Lanes never cross. The lane x of a lane is its slot x times a positive `scale` plus a
 *   `shift`, and all members share both values. So the lanes keep the left-to-right order of the
 *   slots everywhere. Where a lane comes closer than the radius of the widest member plus
 *   `LANE_CLAMP_MARGIN_METERS` to blocked ground, it is clamped, so neighboring lanes can merge
 *   but not swap.
 *
 * Folds: On the inside of sharp corners, the innermost lane can fold back on itself, which means
 *   that it moves backward from one sample to the next. There, all lanes are pulled in
 *   together.
 *
 * Spacing: The lane samples are `LANE_STEP_METERS` (2 m) apart on every route up to
 *   `MAX_FULL_LANE_METERS` (8 km), so that all routes get the same resolution. A longer route gets
 *   a wider spacing, in proportion to its length, so that its samples still fit into
 *   `LANE_MAX_SAMPLES`. The windows of the layout are distances, and `set_up_lanes` turns them
 *   into counts of samples at the spacing of the route. */
#include <stdlib.h>

#include "sim.h"
#include "formation.h"

#define LANE_STEP_METERS 2.0f           /* Lane samples are this far apart along the route. */
#define MAX_FULL_LANE_METERS 8000.0f    /* Longer routes get a wider lane spacing to fit. */
#define LANE_MAX_SAMPLES 4096           /* A squad holds this many: 8 km at 2 m, and 2 more. */
#define LANE_CLAMP_MARGIN_METERS 0.5f   /* Lane clamps keep bodies this far from blocked ground. */
#define LANE_SLOPE 0.3f                 /* Range ends shift at most this many m per m of route. */
#define SAME_COLUMN_METERS 0.5f         /* Slots closer than this sideways share a lane. */
#define SQUEEZED_GAP_METERS 3.0f        /* Squeezed lanes keep two radii plus this apart. */
#define NO_GAP_METERS 1e9f              /* This gap means that all slots share one column. */
#define ROOM_WINDOW_METERS 4.0f         /* The room at a sample spans this far each way. */
#define BELL_REACH_METERS 40.0f         /* The cosine bell of a shift reaches this far each way. */
#define AVERAGE_WINDOW_METERS 6.0f      /* Lanes are averaged over this far each way. */
#define FILE_WINDOW_METERS 8.0f         /* Single file starts this far early. */
#define FOLD_PASSES 3                   /* Fold removal goes over the route this many times. */
#define FOLD_ATTEMPTS 12                /* Lanes are pulled in at most this often per pair. */
#define FOLD_SHRINK_FACTOR 0.8f         /* Each pull scales the lane offsets by this. */
#define MIN_FORWARD_STEP_FRACTION 0.25f /* Outer lanes must move forward this part of a step. */

/* `lanes` holds `LANE_MAX_SAMPLES` samples for each squad, squad after squad in the order of
 * `squads`, so the samples of the squad at `squad_index` start at index
 * `squad_index * LANE_MAX_SAMPLES`. Sample k of a squad lies k lane spacings
 * (`get_lane_spacing`) along its route, and the squad uses `lane_count` of its samples. */
static Lane *lanes;

/* ------------------------------------------------------------------------------------------------
 * `get_lane_spacing` returns how far apart the lane samples of the squad lie. It reads only
 * `route_length`, which `set_up_formation` sets right before it lays out the lanes, so the
 * spacing stays that of the samples until the next layout. It is never below
 * `LANE_STEP_METERS`, also for a squad that never got lanes.
 * --------------------------------------------------------------------------------------------- */
static float get_lane_spacing(const Squad *squad) {
    float length_scale = fmaxf(1, squad->route_length / MAX_FULL_LANE_METERS);
    return LANE_STEP_METERS * length_scale;
}

/* ------------------------------------------------------------------------------------------------
 * `count_lane_samples` returns how many lane spacings make up `meters`, rounded to the nearest
 * whole number, but at least 1. The bell of `smooth_lane_shifts` divides by its count, and a
 * count can round to 0 when the spacing grows on very long routes.
 * --------------------------------------------------------------------------------------------- */
static int count_lane_samples(float meters, float lane_spacing) {
    int count = (int)roundf(meters / lane_spacing);
    if (count < 1)
        return 1;
    return count;
}

/* ------------------------------------------------------------------------------------------------
 * `get_squad_lanes` returns the first lane sample of the squad in `lanes`.
 * --------------------------------------------------------------------------------------------- */
static Lane *get_squad_lanes(const Squad *squad) {
    int first_sample_index = squad->squad_index * LANE_MAX_SAMPLES;
    return &lanes[first_sample_index];
}

/* ------------------------------------------------------------------------------------------------
 * `allocate_lanes` puts the samples on the heap. A static array this large pushes the block that
 * game.c reserves for the image of gc.exe past 0x400000, the fixed base of the image.
 * --------------------------------------------------------------------------------------------- */
void allocate_lanes(void) {
    lanes = malloc(sizeof(Lane) * MAX_SQUADS * LANE_MAX_SAMPLES);
}

/* ------------------------------------------------------------------------------------------------
 * `get_lane_x` returns the lane x of the slot whose sideways offset is `slot_x`. Every slot uses
 * the same `scale` and `shift`, and `scale` is positive, so the lanes keep the order of the
 * slots.
 * --------------------------------------------------------------------------------------------- */
static float get_lane_x(const Lane *lane, float slot_x) {
    return slot_x * lane->scale + lane->shift;
}

/* ------------------------------------------------------------------------------------------------
 * `interpolate` returns the value `fraction` of the way from `here` to `next`.
 * --------------------------------------------------------------------------------------------- */
static float interpolate(float here, float next, float fraction) {
    return here + (next - here) * fraction;
}

/* A `RouteSample` is the squad route at one lane sample: its point, the direction of the
 * formation there, and the direction to the right of that. */
typedef struct {
    V2 point;
    V2 forward;
    V2 right;
} RouteSample;

/* ------------------------------------------------------------------------------------------------
 * `smooth_lane_shifts` turns the sideways shifts of the lanes, and their single-file lines, into
 * smooth bumps instead of ramps with kinks.
 * Inputs: `raw_lanes` holds `sample_count` samples. For each sample, `min_outer_x` and
 *   `max_outer_x` hold the range that the lanes of the outermost slots, at `min_slot_x` and
 *   `max_slot_x`, must stay in.
 * Result: The shift of each sample carries over to the samples around it, times a weight (a
 *   cosine bell) that drops from 1 at the sample to 0 at `BELL_REACH_METERS` from it. The new
 *   shift at a sample is the largest carried shift to the right plus the largest carried shift to
 *   the left, which is 0 or less. It is clamped so that the outermost lanes stay in their range.
 *   `middle` changes in the same way, and it is clamped to the range itself.
 * Edge case: Where the range is empty, the sample keeps its shift or its middle.
 * --------------------------------------------------------------------------------------------- */
static void smooth_lane_shifts(Lane *raw_lanes, int sample_count, const float *min_outer_x,
                               const float *max_outer_x, float min_slot_x, float max_slot_x,
                               float lane_spacing) {
    /* Find the largest carried values on each side at each sample, before any sample changes.
     * Entry k of each array belongs to sample k. */
    static float shift_right[LANE_MAX_SAMPLES];
    static float shift_left[LANE_MAX_SAMPLES];
    static float middle_right[LANE_MAX_SAMPLES];
    static float middle_left[LANE_MAX_SAMPLES];
    int bell_sample_count = count_lane_samples(BELL_REACH_METERS, lane_spacing);

    /* Compute the weight of the bell once for each offset from a sample, because the weight
     * depends on nothing else. Entry k of `bells` belongs to the offset k - `bell_sample_count`,
     * in samples. */
    float bells[2 * bell_sample_count + 1];
    for (int offset = -bell_sample_count; offset <= bell_sample_count; offset++) {
        float angle = PI * offset / bell_sample_count;
        bells[offset + bell_sample_count] = 0.5f * (1 + cosf(angle));
    }

    /* Carry the shift and the middle of every sample in the window to the sample. */
    for (int sample_index = 0; sample_index < sample_count; sample_index++) {
        float best_right = 0;
        float best_left = 0;
        float best_middle_right = 0;
        float best_middle_left = 0;
        int window_start = sample_index - bell_sample_count;
        int window_end = sample_index + bell_sample_count;
        for (int window_index = window_start; window_index <= window_end; window_index++) {
            if (window_index < 0 || window_index >= sample_count)
                continue;
            int offset = window_index - sample_index;
            float bell = bells[offset + bell_sample_count];
            const Lane *raw_lane = &raw_lanes[window_index];
            float carried_shift = raw_lane->shift * bell;
            float carried_middle = raw_lane->middle * bell;
            best_right = fmaxf(best_right, carried_shift);
            best_left = fminf(best_left, carried_shift);
            best_middle_right = fmaxf(best_middle_right, carried_middle);
            best_middle_left = fminf(best_middle_left, carried_middle);
        }
        shift_right[sample_index] = best_right;
        shift_left[sample_index] = best_left;
        middle_right[sample_index] = best_middle_right;
        middle_left[sample_index] = best_middle_left;
    }

    /* Add the two sides, and clamp. */
    for (int sample_index = 0; sample_index < sample_count; sample_index++) {
        Lane *lane = &raw_lanes[sample_index];
        float min_x = min_outer_x[sample_index];
        float max_x = max_outer_x[sample_index];
        float min_shift = min_x - min_slot_x * lane->scale;
        float max_shift = max_x - max_slot_x * lane->scale;
        float shift = shift_right[sample_index] + shift_left[sample_index];
        float middle = middle_right[sample_index] + middle_left[sample_index];
        if (min_shift <= max_shift)
            lane->shift = clamp_float(shift, min_shift, max_shift);
        if (min_x <= max_x)
            lane->middle = clamp_float(middle, min_x, max_x);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `get_lane_step` returns how far the lane of the slot at `slot_x` moves forward from one sample
 * to the next, along the direction of the formation at the first sample. `route_pair` and
 * `lane_pair` each point to the two samples.
 * --------------------------------------------------------------------------------------------- */
static float get_lane_step(const RouteSample *route_pair, const Lane *lane_pair, float slot_x) {
    const RouteSample *here_route = &route_pair[0];
    const RouteSample *next_route = &route_pair[1];
    float lane_x = get_lane_x(&lane_pair[0], slot_x);
    float next_lane_x = get_lane_x(&lane_pair[1], slot_x);
    V2 lane_point = add(here_route->point, mul(here_route->right, lane_x));
    V2 next_lane_point = add(next_route->point, mul(next_route->right, next_lane_x));
    V2 step = sub(next_lane_point, lane_point);
    return dot(step, here_route->forward);
}

/* ------------------------------------------------------------------------------------------------
 * `unfold_lane_pair` pulls the lanes at two neighboring samples in toward the route until the
 * outermost lanes move forward enough from the first sample to the second.
 *
 *         <-------- b     the direction of the formation at b
 *                   ^
 *                   |
 *    L1 ----------- a     the direction of the formation at a points up
 *                   |
 *                   L2
 *
 * In this sketch, the direction of the formation turns sharply left from the route point at
 * sample a to the one at sample b. L1 and L2 are the points of a lane far to the left at a and
 * at b. L2 lies behind L1 along the direction of the formation at a, so this lane folds back.
 * Inputs: `route_pair` and `lane_pair` each point to the two samples.
 * Result: Each pull multiplies `scale` and `shift` of both lane samples by `FOLD_SHRINK_FACTOR`.
 *   The pulls stop once the lanes of the slots at `min_slot_x` and `max_slot_x` both move forward
 *   by at least `MIN_FORWARD_STEP_FRACTION` of `lane_spacing`, the distance between the two
 *   samples. Every lane between them then moves forward too, because a lane point is linear in
 *   the x of its slot.
 * Edge case: After `FOLD_ATTEMPTS` pulls, `unfold_lane_pair` stops even when the lanes still
 *   fold.
 * --------------------------------------------------------------------------------------------- */
static void unfold_lane_pair(const RouteSample *route_pair, Lane *lane_pair, float min_slot_x,
                             float max_slot_x, float lane_spacing) {
    float min_allowed_step = MIN_FORWARD_STEP_FRACTION * lane_spacing;
    for (int attempt_index = 0; attempt_index < FOLD_ATTEMPTS; attempt_index++) {
        /* Find the smaller forward step of the two outermost lanes. */
        float min_step = LARGE_VALUE;
        for (int edge_index = 0; edge_index <= 1; edge_index++) {
            bool is_right_edge = edge_index == 1;
            float slot_x = is_right_edge ? max_slot_x : min_slot_x;
            float step = get_lane_step(route_pair, lane_pair, slot_x);
            min_step = fminf(min_step, step);
        }
        if (min_step >= min_allowed_step)
            return;

        /* Pull both samples in. */
        for (int pair_index = 0; pair_index <= 1; pair_index++) {
            lane_pair[pair_index].scale *= FOLD_SHRINK_FACTOR;
            lane_pair[pair_index].shift *= FOLD_SHRINK_FACTOR;
        }
    }
}

/* ------------------------------------------------------------------------------------------------
 * `limit_range_change` narrows the range at sample `sample_index` so that it reaches at most
 * `max_change` past the range at sample `neighbor_index` on each side.
 * --------------------------------------------------------------------------------------------- */
static void limit_range_change(float *min_outer_x, float *max_outer_x, int sample_index,
                               int neighbor_index, float max_change) {
    float spread_min = min_outer_x[neighbor_index] - max_change;
    float spread_max = max_outer_x[neighbor_index] + max_change;
    min_outer_x[sample_index] = fmaxf(min_outer_x[sample_index], spread_min);
    max_outer_x[sample_index] = fminf(max_outer_x[sample_index], spread_max);
}

/* ------------------------------------------------------------------------------------------------
 * `set_up_lanes` samples the route at the lane spacing and measures the free ground on each
 * side. It turns the free ground into a range of lane x for the outermost lanes, and limits how
 * fast the range changes along the route. Then it fits the slots into the range, smooths and
 * averages the result, and removes folds on the inside of sharp corners.
 *
 * This sketch shows one lane sample, seen from above. Lane x runs across the direction
 * of the formation from the route point (+), with positive values to the right:
 *
 *                          ^ forward
 *                          |
 *   //////|<- left_free ---+-- right_free ->|//////   blocked ground
 *         |   |<-------- range -------->|   |
 *         |<->|                         |<->|
 *        margin                        margin
 *
 * The range (the span of lane x for the outermost lanes) goes from `min_outer_x` to
 * `max_outer_x`. Its margin from blocked ground is the radius of the widest member plus
 * `LANE_MARGIN_METERS`.
 * --------------------------------------------------------------------------------------------- */
void set_up_lanes(Squad *squad, float max_radius) {
    /* Find the sideways spread of the slots, and the smallest sideways gap between two slots
     * that do not share a lane. `min_scale` squeezes that gap to two radii plus
     * `SQUEEZED_GAP_METERS`. */
    int first_index = squad->first_unit_index;
    int end_index = first_index + squad->unit_count;
    float min_slot_x = LARGE_VALUE;
    float max_slot_x = -LARGE_VALUE;
    float min_slot_gap = NO_GAP_METERS;
    for (int unit_index = first_index; unit_index < end_index; unit_index++) {
        float slot_x = units[unit_index].place_offset.x;
        min_slot_x = fminf(min_slot_x, slot_x);
        max_slot_x = fmaxf(max_slot_x, slot_x);
        for (int other_index = unit_index + 1; other_index < end_index; other_index++) {
            float slot_gap = fabsf(slot_x - units[other_index].place_offset.x);
            if (slot_gap > SAME_COLUMN_METERS)
                min_slot_gap = fminf(min_slot_gap, slot_gap);
        }
    }
    float min_scale = 1;
    if (min_slot_gap < NO_GAP_METERS) {
        float squeezed_gap = 2 * max_radius + SQUEEZED_GAP_METERS;
        min_scale = fminf(1, squeezed_gap / min_slot_gap);
    }
    float center_slot_x = (min_slot_x + max_slot_x) / 2;
    float slot_span = max_slot_x - min_slot_x;

    /* Turn the windows into counts of samples at the lane spacing of this route. */
    float lane_spacing = get_lane_spacing(squad);
    int room_window_samples = count_lane_samples(ROOM_WINDOW_METERS, lane_spacing);
    int average_window_samples = count_lane_samples(AVERAGE_WINDOW_METERS, lane_spacing);
    int file_window_samples = count_lane_samples(FILE_WINDOW_METERS, lane_spacing);

    /* Sample the route, and measure the free distance on each side of it, across the direction
     * of the formation. The last sample lies past the end of the route, so that every distance on
     * the route lies between two samples. The lane spacing keeps the count within
     * `LANE_MAX_SAMPLES`. Entry k of each array of this function belongs to sample k. */
    int sample_count = (int)(squad->route_length / lane_spacing) + 2;
    squad->lane_count = sample_count;
    int mover_class = units[first_index].kind->mover_class;
    RouteSample route_samples[LANE_MAX_SAMPLES];
    float left_free[LANE_MAX_SAMPLES];
    float right_free[LANE_MAX_SAMPLES];
    for (int sample_index = 0; sample_index < sample_count; sample_index++) {
        RouteSample *sample = &route_samples[sample_index];
        V2 unused_direction;
        float distance = sample_index * lane_spacing;
        sample->forward = get_formation_direction(squad, distance);
        sample->point = get_route_point(squad, distance, &unused_direction);
        sample->right = get_right(sample->forward);
        V2 point = sample->point;
        V2 left = mul(sample->right, -1);
        float max_distance = LANE_MAX_DISTANCE_METERS;
        right_free[sample_index] = measure_free_distance(point, sample->right, mover_class,
                                                         max_distance);
        left_free[sample_index] = measure_free_distance(point, left, mover_class, max_distance);
    }

    /* Find the range at each sample from the smallest free distance on each side within
     * `ROOM_WINDOW_METERS` each way, less the radius of the widest member and
     * `LANE_MARGIN_METERS`. */
    float min_outer_x[LANE_MAX_SAMPLES];
    float max_outer_x[LANE_MAX_SAMPLES];
    for (int sample_index = 0; sample_index < sample_count; sample_index++) {
        float left_room = LANE_MAX_DISTANCE_METERS;
        float right_room = LANE_MAX_DISTANCE_METERS;
        int window_start = sample_index - room_window_samples;
        int window_end = sample_index + room_window_samples;
        for (int window_index = window_start; window_index <= window_end; window_index++) {
            if (window_index < 0 || window_index >= sample_count)
                continue;
            left_room = fminf(left_room, left_free[window_index]);
            right_room = fminf(right_room, right_free[window_index]);
        }
        min_outer_x[sample_index] = -left_room + max_radius + LANE_MARGIN_METERS;
        max_outer_x[sample_index] = right_room - max_radius - LANE_MARGIN_METERS;
    }

    /* Spread each narrowing of the range to the samples on both sides, so that the range changes
     * by at most `LANE_SLOPE` m per m along the route. A forward pass spreads it to later samples
     * and a backward pass to earlier ones. Lanes then start moving aside well before an obstacle
     * or a narrowing. An end of the range moves at most `max_change` from one sample to the
     * next. */
    float max_change = LANE_SLOPE * lane_spacing;
    for (int sample_index = 1; sample_index < sample_count; sample_index++) {
        int before_index = sample_index - 1;
        limit_range_change(min_outer_x, max_outer_x, sample_index, before_index, max_change);
    }
    for (int sample_index = sample_count - 2; sample_index >= 0; sample_index--) {
        int after_index = sample_index + 1;
        limit_range_change(min_outer_x, max_outer_x, sample_index, after_index, max_change);
    }

    /* Fit the slots into the range at each sample. Where they fit, shift them only as far as an
     * edge of the range pushes them. Where they do not fit, squeeze them around the middle of the
     * range, but not below `min_scale`. The squad fits side by side (`is_wide_enough`) only where
     * the squeeze needs no scale below `min_scale`. Where the range is empty, its middle is the
     * middle of the free ground at the sample. */
    Lane raw_lanes[LANE_MAX_SAMPLES];
    for (int sample_index = 0; sample_index < sample_count; sample_index++) {
        float min_x = min_outer_x[sample_index];
        float max_x = max_outer_x[sample_index];
        float range_width = max_x - min_x;
        Lane *lane = &raw_lanes[sample_index];
        if (min_x <= max_x)
            lane->middle = (min_x + max_x) / 2;
        else
            lane->middle = (right_free[sample_index] - left_free[sample_index]) / 2;
        if (slot_span <= range_width) {
            float smallest_shift = min_x - min_slot_x;
            float largest_shift = max_x - max_slot_x;
            float left_shift = fminf(0, largest_shift);
            lane->scale = 1;
            lane->shift = fmaxf(smallest_shift, left_shift);
            lane->is_wide_enough = true;
        } else {
            float squeeze_scale = 1;
            if (slot_span > 0)
                squeeze_scale = fmaxf(0, range_width) / slot_span;
            lane->is_wide_enough = squeeze_scale >= min_scale;
            lane->scale = fmaxf(squeeze_scale, min_scale);
            lane->shift = lane->middle - center_slot_x * lane->scale;
        }
    }
    if (is_smooth_on)
        smooth_lane_shifts(raw_lanes, sample_count, min_outer_x, max_outer_x, min_slot_x,
                           max_slot_x, lane_spacing);

    /* Average each sample over `AVERAGE_WINDOW_METERS` each way. A sample counts as wide enough
     * only when all samples within `FILE_WINDOW_METERS` each way are, so single file starts that
     * far before a narrow place and ends that far after it. The clamp range keeps the body of the
     * widest member `LANE_CLAMP_MARGIN_METERS` from blocked ground at the sample itself. */
    Lane *squad_lanes = get_squad_lanes(squad);
    for (int sample_index = 0; sample_index < sample_count; sample_index++) {
        float scale_sum = 0;
        float shift_sum = 0;
        float middle_sum = 0;
        int window_count = 0;
        bool is_wide_enough = true;
        int window_start = sample_index - file_window_samples;
        int window_end = sample_index + file_window_samples;
        for (int window_index = window_start; window_index <= window_end; window_index++) {
            if (window_index < 0 || window_index >= sample_count)
                continue;
            const Lane *raw_lane = &raw_lanes[window_index];
            is_wide_enough &= raw_lane->is_wide_enough;
            bool is_in_average = abs(window_index - sample_index) <= average_window_samples;
            if (!is_in_average)
                continue;
            scale_sum += raw_lane->scale;
            shift_sum += raw_lane->shift;
            middle_sum += raw_lane->middle;
            window_count++;
        }
        float min_lane_x = -left_free[sample_index] + max_radius + LANE_CLAMP_MARGIN_METERS;
        float max_lane_x = right_free[sample_index] - max_radius - LANE_CLAMP_MARGIN_METERS;
        squad_lanes[sample_index] = (Lane){
            .scale = scale_sum / window_count,
            .shift = shift_sum / window_count,
            .middle = middle_sum / window_count,
            .min_x = min_lane_x,
            .max_x = max_lane_x,
            .is_wide_enough = is_wide_enough,
        };
    }

    /* Remove folds on the inside of sharp corners. Pulling one sample in can make a new fold with
     * the sample before it, so this goes over the route `FOLD_PASSES` times. */
    for (int pass_index = 0; pass_index < FOLD_PASSES; pass_index++) {
        for (int sample_index = 0; sample_index + 1 < sample_count; sample_index++) {
            const RouteSample *route_pair = &route_samples[sample_index];
            Lane *lane_pair = &squad_lanes[sample_index];
            unfold_lane_pair(route_pair, lane_pair, min_slot_x, max_slot_x, lane_spacing);
        }
    }
}

/* ------------------------------------------------------------------------------------------------
 * `get_lane` interpolates the numbers linearly between the two samples around `distance`. It
 * takes whether the squad fits side by side (`is_wide_enough`) from the sample nearer to
 * `distance`, because a yes-or-no value cannot be interpolated.
 *
 * TODO: confirm that the memory of a squad without samples (`lane_count` 0) is zeroed or never
 * read for output. `allocate_lanes` takes `lanes` from `malloc`, which does not zero it, and no
 * code clears it. So the squad can read values of an earlier route of a squad at the same
 * index, or values that were never set. The trace reads the lanes of every squad in formation
 * mode with lanes, also of a squad whose move was denied before its first `set_up_formation`,
 * so it can show such values.
 * --------------------------------------------------------------------------------------------- */
Lane get_lane(const Squad *squad, float distance) {
    /* Find the sample at or before `distance` and the one after it. A squad whose move was denied
     * before its first `set_up_formation` has no samples, and the trace still reads its lanes.
     * Limit the position with `fminf` and then `fmaxf` instead of `clamp_float`, so that it is 0
     * for such a squad and not -1. Position -1 reads before the samples of the squad. */
    const Lane *squad_lanes = get_squad_lanes(squad);
    float last_sample_position = (float)(squad->lane_count - 1);
    float below_last = fminf(last_sample_position, distance / get_lane_spacing(squad));
    float sample_position = fmaxf(0, below_last);
    int sample_index = (int)sample_position;
    int next_index = sample_index + 1;
    if (next_index >= squad->lane_count)
        next_index = sample_index;
    float fraction = sample_position - sample_index;

    /* Interpolate between them. */
    const Lane *here = &squad_lanes[sample_index];
    const Lane *next = &squad_lanes[next_index];
    int nearer_index = fraction < 0.5f ? sample_index : next_index;
    return (Lane){
        .scale = interpolate(here->scale, next->scale, fraction),
        .shift = interpolate(here->shift, next->shift, fraction),
        .middle = interpolate(here->middle, next->middle, fraction),
        .min_x = interpolate(here->min_x, next->min_x, fraction),
        .max_x = interpolate(here->max_x, next->max_x, fraction),
        .is_wide_enough = squad_lanes[nearer_index].is_wide_enough,
    };
}

/* ------------------------------------------------------------------------------------------------
 * `do_lanes_fit` steps along the route by the spacing of the lane samples, and reads the
 * interpolated lane at each step.
 * --------------------------------------------------------------------------------------------- */
bool do_lanes_fit(const Squad *squad, float from_distance, float to_distance) {
    float lane_spacing = get_lane_spacing(squad);
    for (float distance = from_distance; distance <= to_distance; distance += lane_spacing) {
        Lane lane = get_lane(squad, distance);
        if (!lane.is_wide_enough)
            return false;
    }
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `get_lane_point` offsets the extended route point to the right by the lane x of the unit. The
 * lane x comes from the slot of the unit, or from `middle` in single file, plus the shift of the
 * squad for passing, and it is limited to the free ground.
 * --------------------------------------------------------------------------------------------- */
V2 get_lane_point(const Squad *squad, const Unit *unit, float distance) {
    V2 unused_direction;
    V2 route_point = get_extended_route_point(squad, distance, &unused_direction);
    if (!are_lanes_on)
        return route_point;

    /* Find the lane x, add the shift of the squad for passing another squad, and clamp it to the
     * free ground. Clamping keeps the order of the lanes, so neighboring lanes can merge but not
     * swap. Where the free ground is too narrow for a body and the margins, the lane runs down
     * its middle. */
    Lane lane = get_lane(squad, distance);
    float lane_x;
    if (squad->is_single_file)
        lane_x = lane.middle;
    else
        lane_x = get_lane_x(&lane, unit->place_offset.x);
    lane_x += get_side_shift(squad, distance);
    if (lane.min_x > lane.max_x)
        lane_x = (lane.min_x + lane.max_x) / 2;
    else
        lane_x = clamp_float(lane_x, lane.min_x, lane.max_x);

    /* Go that far to the right of the route, across the direction of the formation. */
    V2 forward = get_formation_direction(squad, distance);
    V2 right = get_right(forward);
    return add(route_point, mul(right, lane_x));
}
