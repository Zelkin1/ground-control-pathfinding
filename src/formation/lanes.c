// Lanes of formation following (`--lanes`). A lane is the path of one slot beside the route. At
// lane cross-sections, `LANE_STEP_METERS` apart along the route, each squad stores how sideways
// slot positions map to sideways lane positions: `slot_x * scale + offset` in square formation,
// or `middle` for every squad member in column formation, within the lane clamp `[min_x, max_x]`.
// `get_lane_point` interpolates between the two nearest cross-sections.
//
// `set_up_lanes` plans the lanes over the whole route, so that they respond early to blocking
// terrain. It measures the free distance to each side of the route, keeps the outer lanes clear
// of the narrowest free distance nearby, fits the slots into that range, spreads each sideways
// move of the lanes as a smooth bump, and averages the result along the route. Where the lanes do
// not fit even when squeezed, the squad drives in column formation (`is_wide_enough` is false).
// Lanes can merge, but they never cross.
//
// Sideways positions are relative to the route point, positive to the right of the formation
// direction as seen facing along it. In local names, `cs` stands for a lane cross-section.
#include <stdlib.h>

#include "sim.h"
#include "formation.h"

#define LANE_STEP_METERS 2.0f                  // spacing of the lane cross-sections
#define MAX_FULL_LANE_METERS 8000.0f           // longer routes get a larger spacing
#define LANE_MAX_CROSS_SECTIONS 4096           // cross-sections per squad, 8 km at 2 m and 2 more
#define LANE_CLAMP_MARGIN_METERS 0.5f          // gap of the lane clamp to blocking terrain
#define LANE_SLOPE 0.3f                        // largest change of the free range per meter
#define SAME_LANE_METERS 0.5f                  // closer slots share one lane
#define NO_GAP_METERS 1e9f                     // slot gap of a squad with a single lane
#define FREE_WINDOW_METERS 4.0f                // reach of the narrowest free distance
#define BELL_REACH_METERS 40.0f                // half length of a smooth bump of the lanes
#define AVERAGE_WINDOW_METERS 6.0f             // reach of the average along the route
#define COLUMN_WINDOW_METERS 8.0f              // column formation extends this far on each side
#define FOLD_ITERATIONS 3                      // passes over the route against folds
#define FOLD_ATTEMPTS 12                       // most pulls per pair of cross-sections
#define FOLD_SHRINK_FACTOR 0.8f                // scale of the sideways offsets per pull
#define MIN_FORWARD_STEP_FRACTION 0.25f        // smallest forward step of a lane, per spacing

// Lane cross-sections of all squads: one block of `LANE_MAX_CROSS_SECTIONS` per squad, in the
// order of the squad indexes, and in each block, in the order along the route.
static LaneCrossSection *lane_cross_sections;


// Route point, formation direction and its right side at one lane cross-section.
typedef struct {
    V2 point;
    V2 forward;
    V2 right;
} RouteCrossSection;


// Sideways extent of the slots of a squad, and the smallest scale of `slot_x` that keeps squeezed
// lanes apart.
typedef struct {
    float min_x;
    float max_x;
    float min_scale;
} SlotSpread;


// `get_lane_spacing` returns the spacing of the lane cross-sections of a squad. On a route longer
// than `MAX_FULL_LANE_METERS`, the spacing grows with the route length, so that the route fits.
static float get_lane_spacing(const Squad *squad) {
    float length_scale = fmaxf(1, squad->route_length / MAX_FULL_LANE_METERS);
    return LANE_STEP_METERS * length_scale;
}


// `count_cross_sections` returns the number of cross-sections that span `meters` at
// `lane_spacing`, rounded to the nearest whole number, and at least 1.
static int count_cross_sections(float meters, float lane_spacing) {
    int count = (int)roundf(meters / lane_spacing);
    if (count < 1)
        return 1;
    return count;
}


// `get_squad_cross_sections` returns the first lane cross-section of the block of a squad.
static LaneCrossSection *get_squad_cross_sections(const Squad *squad) {
    int first_index = squad->squad_index * LANE_MAX_CROSS_SECTIONS;
    return &lane_cross_sections[first_index];
}


// `allocate_lanes` allocates the lane cross-sections of all squads.
void allocate_lanes(void) {
    lane_cross_sections = malloc(sizeof(LaneCrossSection) * MAX_SQUADS * LANE_MAX_CROSS_SECTIONS);
}


// `get_lane_x` returns the sideways lane position of the sideways slot position `slot_x` at a
// cross-section, in square formation.
static float get_lane_x(const LaneCrossSection *cross_section, float slot_x) {
    return slot_x * cross_section->scale + cross_section->offset;
}


// `interpolate` returns the value at `fraction` of the way from `here` to `next`.
static float interpolate(float here, float next, float fraction) {
    return here + (next - here) * fraction;
}


// `smooth_lane_offsets` spreads each sideways move of the lanes along the route as a smooth bump
// (`--smooth`). Each cross-section takes the largest offset to the right and the largest offset
// to the left within `BELL_REACH_METERS`, each weighted by a raised cosine of the distance.
static void smooth_lane_offsets(LaneCrossSection *raw_cross_sections, int count,
                                const float *min_outer_x, const float *max_outer_x,
                                float min_slot_x, float max_slot_x, float lane_spacing) {
    // `bells` holds the raised cosine from `-reach` to `reach` cross-sections, 1 in the middle.
    static float right_offsets[LANE_MAX_CROSS_SECTIONS];
    static float left_offsets[LANE_MAX_CROSS_SECTIONS];
    static float right_middles[LANE_MAX_CROSS_SECTIONS];
    static float left_middles[LANE_MAX_CROSS_SECTIONS];
    int reach = count_cross_sections(BELL_REACH_METERS, lane_spacing);
    float bells[2 * reach + 1];
    for (int relative_index = -reach; relative_index <= reach; relative_index++) {
        float angle = PI * relative_index / reach;
        bells[relative_index + reach] = 0.5f * (1 + cosf(angle));
    }

    // The offset and the middle each take their weighted extremes on both sides of zero.
    for (int cs_index = 0; cs_index < count; cs_index++) {
        float off_r = 0;        // largest weighted offset to the right
        float off_l = 0;        // largest weighted offset to the left
        float mid_r = 0;        // largest weighted middle to the right
        float mid_l = 0;        // largest weighted middle to the left
        int window_start = cs_index - reach;
        int window_end = cs_index + reach;
        for (int window_index = window_start; window_index <= window_end; window_index++) {
            if (window_index < 0 || window_index >= count)
                continue;
            int relative_index = window_index - cs_index;
            float bell = bells[relative_index + reach];
            const LaneCrossSection *raw = &raw_cross_sections[window_index];
            float offset = raw->offset * bell;
            float middle = raw->middle * bell;
            off_r = fmaxf(off_r, offset);
            off_l = fminf(off_l, offset);
            mid_r = fmaxf(mid_r, middle);
            mid_l = fminf(mid_l, middle);
        }
        right_offsets[cs_index] = off_r;
        left_offsets[cs_index] = off_l;
        right_middles[cs_index] = mid_r;
        left_middles[cs_index] = mid_l;
    }

    // The bumps stay inside the free range of the outer lanes, where that range is not empty.
    for (int cs_index = 0; cs_index < count; cs_index++) {
        LaneCrossSection *cs = &raw_cross_sections[cs_index];
        float min_x = min_outer_x[cs_index];
        float max_x = max_outer_x[cs_index];
        float min_offset = min_x - min_slot_x * cs->scale;
        float max_offset = max_x - max_slot_x * cs->scale;
        float offset = right_offsets[cs_index] + left_offsets[cs_index];
        float middle = right_middles[cs_index] + left_middles[cs_index];
        if (min_offset <= max_offset)
            cs->offset = clamp_float(offset, min_offset, max_offset);
        if (min_x <= max_x)
            cs->middle = clamp_float(middle, min_x, max_x);
    }
}


// `get_lane_step` returns how far the lane of `slot_x` moves forward from the first to the second
// cross-section of a pair, along the formation direction at the first one.
static float get_lane_step(const RouteCrossSection *route_pair,
                           const LaneCrossSection *cross_section_pair, float slot_x) {
    const RouteCrossSection *here = &route_pair[0];
    const RouteCrossSection *next = &route_pair[1];
    float x = get_lane_x(&cross_section_pair[0], slot_x);
    float x_next = get_lane_x(&cross_section_pair[1], slot_x);
    V2 point = add(here->point, mul(here->right, x));
    V2 point_next = add(next->point, mul(next->right, x_next));
    V2 step = sub(point_next, point);
    return dot(step, here->forward);
}


// `unfold_cross_section_pair` pulls the lanes of two consecutive cross-sections toward the route
// until every lane moves forward by at least `MIN_FORWARD_STEP_FRACTION` of the spacing, in at
// most `FOLD_ATTEMPTS` pulls. In a sharp bend, an inner lane could otherwise run backward.
static void unfold_cross_section_pair(const RouteCrossSection *route_pair,
                                      LaneCrossSection *cross_section_pair, float min_slot_x,
                                      float max_slot_x, float lane_spacing) {
    // The forward step is linear in `slot_x`, so the two outer lanes have the smallest step.
    float step_min = MIN_FORWARD_STEP_FRACTION * lane_spacing;        // smallest allowed step
    for (int attempt_index = 0; attempt_index < FOLD_ATTEMPTS; attempt_index++) {
        float step = LARGE_VALUE;                                     // smallest step of a lane
        for (int edge_index = 0; edge_index <= 1; edge_index++) {
            bool is_right_edge = edge_index == 1;
            float slot_x = is_right_edge ? max_slot_x : min_slot_x;
            float edge_step = get_lane_step(route_pair, cross_section_pair, slot_x);
            step = fminf(step, edge_step);
        }
        if (step >= step_min)
            return;

        // A pull scales the sideways positions of all lanes of both cross-sections.
        for (int pair_index = 0; pair_index <= 1; pair_index++) {
            cross_section_pair[pair_index].scale *= FOLD_SHRINK_FACTOR;
            cross_section_pair[pair_index].offset *= FOLD_SHRINK_FACTOR;
        }
    }
}


// `limit_range_change` keeps the free range at the cross-section `cross_section_index` from
// reaching more than `max_change` farther out than at `neighbor_index`, on each side.
static void limit_range_change(float *min_outer_x, float *max_outer_x, int cross_section_index,
                               int neighbor_index, float max_change) {
    float widened_min_x = min_outer_x[neighbor_index] - max_change;
    float widened_max_x = max_outer_x[neighbor_index] + max_change;
    min_outer_x[cross_section_index] = fmaxf(min_outer_x[cross_section_index], widened_min_x);
    max_outer_x[cross_section_index] = fminf(max_outer_x[cross_section_index], widened_max_x);
}


// `measure_slot_spread` measures the sideways extent of the slots of a squad, and the smallest
// scale that keeps squeezed lanes two radii of the widest unit plus the squeezed gap of the
// faction tuning apart.
static SlotSpread measure_slot_spread(const Squad *squad, float max_radius) {
    // Slots closer sideways than `SAME_LANE_METERS` share a lane, so their gap does not count.
    int first_index = squad->first_unit_index;
    int end_index = first_index + squad->unit_count;
    float x_min = LARGE_VALUE;
    float x_max = -LARGE_VALUE;
    float gap_min = NO_GAP_METERS;        // smallest gap between two lanes
    for (int unit_index = first_index; unit_index < end_index; unit_index++) {
        float x = units[unit_index].slot_position.x;
        x_min = fminf(x_min, x);
        x_max = fmaxf(x_max, x);
        for (int other_index = unit_index + 1; other_index < end_index; other_index++) {
            float gap = fabsf(x - units[other_index].slot_position.x);
            if (gap > SAME_LANE_METERS)
                gap_min = fminf(gap_min, gap);
        }
    }

    // A squad with a single lane needs no squeeze limit.
    float min_scale = 1;
    if (gap_min < NO_GAP_METERS) {
        float gap_meters = get_tuning(squad->unit_type)->squeezed_gap_meters;
        float squeezed = 2 * max_radius + gap_meters;        // squeezed lane gap
        min_scale = fminf(1, squeezed / gap_min);
    }
    return (SlotSpread){
        .min_x = x_min,
        .max_x = x_max,
        .min_scale = min_scale,
    };
}


// `get_free_distance_limit` returns how far the free distances of a squad are measured: at least
// `LANE_MAX_DISTANCE_METERS`, and far enough that the outer lanes fit in open terrain. A narrower
// limit would squeeze a wide formation, such as a line of large units, even in open terrain.
static float get_free_distance_limit(const SlotSpread *spread, float max_radius) {
    float outer_x = fmaxf(-spread->min_x, spread->max_x);        // farthest slot from the route
    float needed = outer_x + max_radius + LANE_MARGIN_METERS;
    return fmaxf(LANE_MAX_DISTANCE_METERS, needed);
}


// `sample_route_cross_sections` fills the route cross-sections of a squad, and the free distances
// from the route point to the left and to the right, up to `d_max`.
static void sample_route_cross_sections(const Squad *squad, int count, float lane_spacing,
                                        float d_max, RouteCrossSection *route_cross_sections,
                                        float *left_free, float *right_free) {
    int terrain_class = units[squad->first_unit_index].unit_type->terrain_class;
    for (int cs_index = 0; cs_index < count; cs_index++) {
        RouteCrossSection *route_cs = &route_cross_sections[cs_index];
        V2 unused;
        float d = cs_index * lane_spacing;        // route position
        route_cs->forward = get_formation_direction(squad, d);
        route_cs->point = get_route_point(squad, d, &unused);
        route_cs->right = get_right(route_cs->forward);
        V2 point = route_cs->point;
        V2 left = mul(route_cs->right, -1);
        right_free[cs_index] = measure_free_distance(point, route_cs->right, terrain_class, d_max);
        left_free[cs_index] = measure_free_distance(point, left, terrain_class, d_max);
    }
}


// `find_outer_ranges` computes the free range of the outer lanes at each cross-section: the
// sideways positions whose body of the widest unit plus `LANE_MARGIN_METERS` keeps clear of the
// narrowest free distance within `window` cross-sections on each side, up to `d_max`.
static void find_outer_ranges(int count, int window, float max_radius, float d_max,
                              const float *left_free, const float *right_free,
                              float *min_outer_x, float *max_outer_x) {
    for (int cs_index = 0; cs_index < count; cs_index++) {
        float left = d_max;         // narrowest free distance to the left
        float right = d_max;        // narrowest free distance to the right
        int window_start = cs_index - window;
        int window_end = cs_index + window;
        for (int window_index = window_start; window_index <= window_end; window_index++) {
            if (window_index < 0 || window_index >= count)
                continue;
            left = fminf(left, left_free[window_index]);
            right = fminf(right, right_free[window_index]);
        }
        min_outer_x[cs_index] = -left + max_radius + LANE_MARGIN_METERS;
        max_outer_x[cs_index] = right - max_radius - LANE_MARGIN_METERS;
    }
}


// `limit_range_changes` limits the sideways change of the free ranges to `LANE_SLOPE` per meter of
// route, in a forward and a backward pass. The free range then narrows gradually before and
// after a narrow part of the route.
static void limit_range_changes(int count, float lane_spacing, float *min_outer_x,
                                float *max_outer_x) {
    float max_change = LANE_SLOPE * lane_spacing;
    for (int cs_index = 1; cs_index < count; cs_index++) {
        int before_index = cs_index - 1;
        limit_range_change(min_outer_x, max_outer_x, cs_index, before_index, max_change);
    }
    for (int cs_index = count - 2; cs_index >= 0; cs_index--) {
        int after_index = cs_index + 1;
        limit_range_change(min_outer_x, max_outer_x, cs_index, after_index, max_change);
    }
}


// `fit_raw_cross_sections` fits the slots into the free range at each cross-section. Slots that
// fit keep their spacing, with the offset closest to zero that keeps them inside the range. Slots
// that do not fit are squeezed around the middle of the range, but no further than `min_scale`.
static void fit_raw_cross_sections(int count, const SlotSpread *spread, const float *min_outer_x,
                                   const float *max_outer_x, const float *left_free,
                                   const float *right_free, LaneCrossSection *raw_cross_sections) {
    float center = (spread->min_x + spread->max_x) / 2;        // middle of the slots
    float span = spread->max_x - spread->min_x;                // sideways extent of the slots
    for (int cs_index = 0; cs_index < count; cs_index++) {
        // An empty free range has the middle of the free distances as its middle.
        float min_x = min_outer_x[cs_index];
        float max_x = max_outer_x[cs_index];
        float width = max_x - min_x;
        LaneCrossSection *cs = &raw_cross_sections[cs_index];
        if (min_x <= max_x) {
            cs->middle = (min_x + max_x) / 2;
        } else {
            float right = right_free[cs_index];
            float left = left_free[cs_index];
            cs->middle = (right - left) / 2;
        }

        // `lowest` and `highest` are the offsets that put the left lane on `min_x` and the right
        // lane on `max_x`. A squeeze below `min_scale` makes the cross-section too narrow.
        if (span <= width) {
            float lowest = min_x - spread->min_x;
            float highest = max_x - spread->max_x;
            float left_offset = fminf(0, highest);
            cs->scale = 1;
            cs->offset = fmaxf(lowest, left_offset);
            cs->is_wide_enough = true;
        } else {
            float squeeze = 1;
            if (span > 0)
                squeeze = fmaxf(0, width) / span;
            cs->is_wide_enough = squeeze >= spread->min_scale;
            cs->scale = fmaxf(squeeze, spread->min_scale);
            cs->offset = cs->middle - center * cs->scale;
        }
    }
}


// `average_cross_sections` stores the lane cross-sections of a squad: the raw values averaged
// within `average_window` cross-sections on each side, and the lane clamp from the free
// distances. A cross-section is wide enough only when all raw ones within `column_window` are.
static void average_cross_sections(const LaneCrossSection *raw_cross_sections, int count,
                                   int average_window, int column_window, float max_radius,
                                   const float *left_free, const float *right_free,
                                   LaneCrossSection *squad_cross_sections) {
    for (int cs_index = 0; cs_index < count; cs_index++) {
        // The average reaches at most as far as `column_window`.
        float scale_sum = 0;
        float offset_sum = 0;
        float middle_sum = 0;
        int n = 0;        // number of averaged cross-sections
        bool is_wide_enough = true;
        int window_start = cs_index - column_window;
        int window_end = cs_index + column_window;
        for (int window_index = window_start; window_index <= window_end; window_index++) {
            if (window_index < 0 || window_index >= count)
                continue;
            const LaneCrossSection *raw = &raw_cross_sections[window_index];
            is_wide_enough &= raw->is_wide_enough;
            int relative_index = window_index - cs_index;
            bool is_in_average = abs(relative_index) <= average_window;
            if (!is_in_average)
                continue;
            scale_sum += raw->scale;
            offset_sum += raw->offset;
            middle_sum += raw->middle;
            n++;
        }

        // The lane clamp keeps the body of the widest unit `LANE_CLAMP_MARGIN_METERS` away from
        // blocking terrain at this cross-section only.
        float min_x = -left_free[cs_index] + max_radius + LANE_CLAMP_MARGIN_METERS;
        float max_x = right_free[cs_index] - max_radius - LANE_CLAMP_MARGIN_METERS;
        squad_cross_sections[cs_index] = (LaneCrossSection){
            .scale = scale_sum / n,
            .offset = offset_sum / n,
            .middle = middle_sum / n,
            .min_x = min_x,
            .max_x = max_x,
            .is_wide_enough = is_wide_enough,
        };
    }
}


// `unfold_lanes` runs `FOLD_ITERATIONS` passes of `unfold_cross_section_pair` over every pair of
// consecutive cross-sections of a squad.
static void unfold_lanes(const RouteCrossSection *route_cross_sections,
                         LaneCrossSection *squad_cross_sections, int count,
                         const SlotSpread *spread, float lane_spacing) {
    for (int iteration_index = 0; iteration_index < FOLD_ITERATIONS; iteration_index++) {
        for (int cs_index = 0; cs_index + 1 < count; cs_index++) {
            const RouteCrossSection *route_pair = &route_cross_sections[cs_index];
            LaneCrossSection *cs_pair = &squad_cross_sections[cs_index];
            unfold_cross_section_pair(route_pair, cs_pair, spread->min_x, spread->max_x,
                                      lane_spacing);
        }
    }
}


// `set_up_lanes` plans the lane cross-sections of a squad over its whole route. `max_radius` is
// the radius of the widest squad member, which the free ranges and the lane clamp keep clear.
void set_up_lanes(Squad *squad, float max_radius) {
    // The last cross-section lies at or past the route end.
    SlotSpread spread = measure_slot_spread(squad, max_radius);
    float spacing = get_lane_spacing(squad);
    int free_window = count_cross_sections(FREE_WINDOW_METERS, spacing);
    int average_window = count_cross_sections(AVERAGE_WINDOW_METERS, spacing);
    int column_window = count_cross_sections(COLUMN_WINDOW_METERS, spacing);
    int count = (int)(squad->route_length / spacing) + 2;
    squad->cross_section_count = count;

    // Free ranges of the outer lanes, from the free distances on both sides of the route.
    RouteCrossSection route_cross_sections[LANE_MAX_CROSS_SECTIONS];
    float left_free[LANE_MAX_CROSS_SECTIONS];
    float right_free[LANE_MAX_CROSS_SECTIONS];
    float min_outer_x[LANE_MAX_CROSS_SECTIONS];
    float max_outer_x[LANE_MAX_CROSS_SECTIONS];
    float d_max = get_free_distance_limit(&spread, max_radius);        // free distance limit
    sample_route_cross_sections(squad, count, spacing, d_max, route_cross_sections, left_free,
                                right_free);
    find_outer_ranges(count, free_window, max_radius, d_max, left_free, right_free, min_outer_x,
                      max_outer_x);
    limit_range_changes(count, spacing, min_outer_x, max_outer_x);

    // The slots fit into the free ranges first, and the later steps smooth the result.
    LaneCrossSection raw_cross_sections[LANE_MAX_CROSS_SECTIONS];
    fit_raw_cross_sections(count, &spread, min_outer_x, max_outer_x, left_free, right_free,
                           raw_cross_sections);
    if (is_smooth_on)
        smooth_lane_offsets(raw_cross_sections, count, min_outer_x, max_outer_x, spread.min_x,
                            spread.max_x, spacing);
    LaneCrossSection *squad_cross_sections = get_squad_cross_sections(squad);
    average_cross_sections(raw_cross_sections, count, average_window, column_window, max_radius,
                           left_free, right_free, squad_cross_sections);
    unfold_lanes(route_cross_sections, squad_cross_sections, count, &spread, spacing);
}


// `get_lane_cross_section` returns the lane cross-section of a squad at the route position
// `distance`, interpolated between the two nearest stored ones. `is_wide_enough` comes from the
// nearer one, and positions beyond the range read an end one.
LaneCrossSection get_lane_cross_section(const Squad *squad, float distance) {
    // `position` counts cross-sections from the route start, with a fraction.
    const LaneCrossSection *cross_sections = get_squad_cross_sections(squad);
    int count = squad->cross_section_count;
    float last = (float)(count - 1);
    float below_last = fminf(last, distance / get_lane_spacing(squad));
    float position = fmaxf(0, below_last);
    int cs_index = (int)position;
    int next_index = cs_index + 1;
    if (next_index >= count)
        next_index = cs_index;
    float fraction = position - cs_index;

    const LaneCrossSection *here = &cross_sections[cs_index];
    const LaneCrossSection *next = &cross_sections[next_index];
    int nearer_index = fraction < 0.5f ? cs_index : next_index;
    return (LaneCrossSection){
        .scale = interpolate(here->scale, next->scale, fraction),
        .offset = interpolate(here->offset, next->offset, fraction),
        .middle = interpolate(here->middle, next->middle, fraction),
        .min_x = interpolate(here->min_x, next->min_x, fraction),
        .max_x = interpolate(here->max_x, next->max_x, fraction),
        .is_wide_enough = cross_sections[nearer_index].is_wide_enough,
    };
}


// `do_lanes_fit` tells whether every lane cross-section of a squad from `from_distance` to
// `to_distance` is wide enough for square formation.
bool do_lanes_fit(const Squad *squad, float from_distance, float to_distance) {
    float spacing = get_lane_spacing(squad);
    for (float d = from_distance; d <= to_distance; d += spacing) {
        LaneCrossSection cs = get_lane_cross_section(squad, d);
        if (!cs.is_wide_enough)
            return false;
    }
    return true;
}


// `get_lane_point` returns the point of the lane of a squad member at the route position
// `distance`, including the lane shift. With `--lanes off`, the lane is the route itself.
V2 get_lane_point(const Squad *squad, const Unit *unit, float distance) {
    V2 unused;
    V2 route_point = get_extended_route_point(squad, distance, &unused);
    if (!are_lanes_on)
        return route_point;

    // The lane clamp also limits the shift. An empty clamp range puts the lane in its middle.
    LaneCrossSection cs = get_lane_cross_section(squad, distance);
    float x;        // sideways lane position
    if (squad->is_in_column)
        x = cs.middle;
    else
        x = get_lane_x(&cs, unit->slot_position.x);
    x += get_lane_shift(squad, distance);
    if (cs.min_x > cs.max_x)
        x = (cs.min_x + cs.max_x) / 2;
    else
        x = clamp_float(x, cs.min_x, cs.max_x);

    V2 forward = get_formation_direction(squad, distance);
    V2 right = get_right(forward);
    return add(route_point, mul(right, x));
}
