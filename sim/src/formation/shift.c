/* shift.c moves the lanes of a squad sideways around a meeting or a pass, in the "coord" mode of
 * `--avoid`. coordination.c sets the shift of two squads that meet head-on with
 * `set_shift_target`, and overtaking.c sets the shift of a pass with `set_pass_shift`. Once per
 * tick, `coordinate_squads` then keeps each shift steady with `keep_shift_steady` and moves it
 * with `move_side_offset`. `get_lane_point` in lanes.c adds `get_side_shift` to every lane.
 *
 * Terms: The top comment of sim.h defines the shift (`side_offset`). In short, a shift moves all
 * lanes of a squad sideways over a stretch of its route, in meters and positive to the right.
 * formation.h defines the wanted shift (`side_offset_target`), the stretch of a shift and a pair
 * (two moving squads that `coordinate_squads` checks against each other) above the declarations
 * of this file. */
#include "sim.h"
#include "formation.h"

#define SIDE_SHIFT_SLOPE 0.3f            /* Lanes shift at most this many m per m driven. */
#define SHIFT_REACH_METERS 80.0f         /* The full shift reaches this far around the meeting. */
#define SHIFT_AFTER_METERS 10.0f         /* With `--shortreturn`, it ends this far past it. */
#define SHIFT_BEFORE_METERS 20.0f        /* With `--lateshift`, it starts this far before it. */
#define SWAY_MIN_SPEED_FRACTION 0.25f    /* Shifts move as if at this part of top speed or more. */
#define SWAY_SLOPE 0.15f                 /* Smooth shifts move at most this many m per m driven. */

bool has_meeting[MAX_SQUADS];
bool is_pass_shift[MAX_SQUADS];

/* ------------------------------------------------------------------------------------------------
 * `set_shift_stretch` sets `side_offset_target` of the squad to `target`, and the stretch of the
 * full shift to the route from `start` to `end`, unless another pair already set a larger target
 * in this tick.
 * --------------------------------------------------------------------------------------------- */
static void set_shift_stretch(Squad *squad, float start, float end, float target) {
    if (fabsf(target) <= fabsf(squad->side_offset_target))
        return;
    squad->side_offset_target = target;
    squad->side_shift_start = start;
    squad->side_shift_end = end;
}

/* ------------------------------------------------------------------------------------------------
 * `set_shift_target` puts the stretch around the meeting and passes it to `set_shift_stretch`:
 *
 *   start       meeting            end
 *     |<---------->x<-------------->|
 *      before_reach   after_reach
 *
 * Reaches: Both reaches are `SHIFT_REACH_METERS` unless a switch below shortens them.
 * `--lateshift on`: `before_reach` is only `SHIFT_BEFORE_METERS`, so that the squads move aside
 *   when they need to. With `--lateshift off`, and with the fade before the stretch, lanes start
 *   to move about 160 m (5 s for light tanks) before the squads meet.
 * `--shortreturn on`: `after_reach` is only `SHIFT_AFTER_METERS`, so that the squads go back to
 *   their own lanes soon after they pass each other. While they still overlap there,
 *   `pass_head_on` in coordination.c finds the meeting again, and its new stretch keeps them
 *   apart.
 * --------------------------------------------------------------------------------------------- */
void set_shift_target(Squad *squad, float meeting_distance, float target) {
    float before_reach = is_late_shift_on ? SHIFT_BEFORE_METERS : SHIFT_REACH_METERS;
    float after_reach = is_short_return_on ? SHIFT_AFTER_METERS : SHIFT_REACH_METERS;
    float start = meeting_distance - before_reach;
    float end = meeting_distance + after_reach;
    set_shift_stretch(squad, start, end, target);
}

/* ------------------------------------------------------------------------------------------------
 * `set_pass_shift` sets the shift with `set_shift_stretch`, and it also sets `has_meeting` and
 * `is_pass_shift` of the squad. With `--smoothsway on`, `coordinate_squads` reads both in
 * `keep_shift_steady`, so that the shift of a pass stays steady from tick to tick.
 * --------------------------------------------------------------------------------------------- */
void set_pass_shift(Squad *squad, float start, float end, float target) {
    set_shift_stretch(squad, start, end, target);
    has_meeting[squad->squad_index] = true;
    is_pass_shift[squad->squad_index] = true;
}

/* ------------------------------------------------------------------------------------------------
 * `move_side_offset` limits the step of the shift in one of two ways.
 * `--smoothsway off`: The shift moves by at most `SIDE_SHIFT_SLOPE` m per m that the squad
 *   drives in this tick at its top speed. The top speed and not the speed of the anchor sets
 *   the rate, so that a shift also fades out after an anchor that stopped at the end of the
 *   route. But a squad that is already on the stretch of the shift then swerves at once, which a
 *   slow-turning unit can only do by braking.
 * `--smoothsway on`: The speed of the shift, `side_offset_rate`, builds up and dies down within
 *   the acceleration `speed^2 / full_speed_radius`. A unit that drives at `speed` while its lane
 *   slides sideways with an acceleration `a` follows a path of the curvature `a / speed^2`, so
 *   that path bends no sharper than the members can drive at top speed. `speed` is the speed of
 *   the anchor, but at least `SWAY_MIN_SPEED_FRACTION` of the top speed, so that a slow squad
 *   still finishes its shift. The speed of the shift also stays within `SWAY_SLOPE` times
 *   `speed`, and within the speed from which that acceleration stops it at the target:
 *   `rate^2 = 2 * acceleration * remaining`. A squad without a full-speed radius, which never
 *   had a route, moves as with `--smoothsway off`.
 * --------------------------------------------------------------------------------------------- */
void move_side_offset(Squad *squad) {
    /* With `--smoothsway off`, or without a full-speed radius, step at the fixed rate. */
    float change = squad->side_offset_target - squad->side_offset;
    if (!is_smooth_sway_on || squad->full_speed_radius <= 0) {
        float max_step = SIDE_SHIFT_SLOPE * squad->kind->max_speed * TICK_SECONDS;
        squad->side_offset += clamp_float(change, -max_step, max_step);
        return;
    }

    /* Find the limits of the speed of the shift and of its change in this tick. */
    float min_speed = SWAY_MIN_SPEED_FRACTION * squad->kind->max_speed;
    float speed = fmaxf(squad->anchor_speed, min_speed);
    float max_acceleration = speed * speed / squad->full_speed_radius;
    float stopping_rate = sqrtf(2 * max_acceleration * fabsf(change));
    float max_rate = fminf(SWAY_SLOPE * speed, stopping_rate);
    float max_rate_step = max_acceleration * TICK_SECONDS;

    /* Steer the speed of the shift toward the wanted one, and move the shift. Stop on the target
     * when the step would pass it. */
    float wanted_rate = copysignf(max_rate, change);
    float rate_change = wanted_rate - squad->side_offset_rate;
    squad->side_offset_rate += clamp_float(rate_change, -max_rate_step, max_rate_step);
    float step = squad->side_offset_rate * TICK_SECONDS;
    bool is_toward_target = step * change > 0;
    if (is_toward_target && fabsf(step) >= fabsf(change)) {
        squad->side_offset = squad->side_offset_target;
        squad->side_offset_rate = 0;
        return;
    }
    squad->side_offset += step;
}

/* ------------------------------------------------------------------------------------------------
 * `get_side_shift` fades the shift in before its stretch and out after it, along the route, so
 * that lanes far from the meeting stay where they are:
 *
 *   side_offset             .------------.
 *                        .                  .
 *   0  ______________ .                        . ______________  distance
 *                      fade                fade
 *                         start         end
 *
 *   Between `side_shift_start` (start) and `side_shift_end` (end), the lanes shift by the full
 *   `side_offset`. Squads keep their last stretch while the shift fades out after the meeting.
 * The length of the fade is the shift divided by `SIDE_SHIFT_SLOPE`, so that a lane moves
 *   sideways by at most about that slope per m along the route. The fade is half a cosine wave,
 *   so the lane has no kinks where it starts and ends.
 * With `--smoothsway on`, the fade uses the gentler `SWAY_SLOPE`, and it is also long enough for
 *   every member to drive its bends at top speed. A fade of the shift `A` over the length `L` is
 *   `A / 2 * (1 + cos(PI * x / L))`. Its sharpest bends, at its two ends, have the curvature
 *   `A / 2 * (PI / L)^2`. That is at most `1 / full_speed_radius` when
 *   `L >= PI * sqrt(A * full_speed_radius / 2)`.
 * --------------------------------------------------------------------------------------------- */
float get_side_shift(const Squad *squad, float distance) {
    if (squad->side_offset == 0)
        return 0;

    /* Find the length of the fade. */
    float shift_meters = fabsf(squad->side_offset);
    float fade_meters = shift_meters / SIDE_SHIFT_SLOPE;
    if (is_smooth_sway_on) {
        float gentle_fade_meters = shift_meters / SWAY_SLOPE;
        float wide_fade_meters = PI * sqrtf(shift_meters * squad->full_speed_radius / 2);
        fade_meters = fmaxf(gentle_fade_meters, wide_fade_meters);
    }

    /* Find how far `distance` lies outside the stretch, and fade the shift over `fade_meters`. */
    float outside_meters = 0;
    if (distance < squad->side_shift_start)
        outside_meters = squad->side_shift_start - distance;
    if (distance > squad->side_shift_end)
        outside_meters = distance - squad->side_shift_end;
    float fade_fraction = clamp_float(outside_meters / fade_meters, 0, 1);
    float weight = 0.5f * (1 + cosf(PI * fade_fraction));
    return squad->side_offset * weight;
}

/* ------------------------------------------------------------------------------------------------
 * The meeting is found again in every tick, and its place and shift jump. Without
 * `keep_shift_steady`, the stretch moves by up to 20 m and the shift by up to 4 m from one tick
 * to the next, and in some ticks the meeting is missed. Each jump moves the lane points ahead of
 * the units sideways, and the bend limit makes them brake. A pass shift keeps its own target,
 * because the room ahead of a passing squad shrinks as it drives on, and a shift past the room
 * presses its lanes onto the kinks of the clamps.
 * --------------------------------------------------------------------------------------------- */
void keep_shift_steady(Squad *squad, float last_target, float last_start, float last_end) {
    /* Keep nothing without a last shift, or once the anchor reaches the end of the last
     * stretch. */
    bool is_before_last_end = squad->anchor_distance < last_end;
    if (last_target == 0 || !is_before_last_end)
        return;

    /* Let a shift to the other side, or a meeting without a shift, replace the last shift. */
    float target = squad->side_offset_target;
    bool is_meeting_missed = !has_meeting[squad->squad_index];
    bool is_same_side = target * last_target > 0;
    if (!is_meeting_missed && !is_same_side)
        return;

    /* Let a pass shift keep its own target, and only grow its stretch. */
    if (is_pass_shift[squad->squad_index]) {
        squad->side_shift_start = fminf(squad->side_shift_start, last_start);
        squad->side_shift_end = fmaxf(squad->side_shift_end, last_end);
        return;
    }

    /* Keep the larger of the two targets, and the last stretch. */
    if (fabsf(last_target) > fabsf(target))
        squad->side_offset_target = last_target;
    squad->side_shift_start = last_start;
    squad->side_shift_end = last_end;
}
