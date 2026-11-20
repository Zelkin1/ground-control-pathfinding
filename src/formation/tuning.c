// Faction tuning (`--factiontuning`). The constants of formation following were first set for the
// Crayven unit types, but the Order units are larger and faster, and the Order light tank brakes
// slowly (friction 30). So each faction gets its own values, found by parameter scans: each
// constant alone, then in combinations, over 300 seeds of every scenario in the coordination mode.
// The values and their measured effects are in NOTES.md, "Faction tuning".
#include "formation.h"

// The values from before the tuning, which every faction uses with `--factiontuning off`.
static const FactionTuning DEFAULT_TUNING = {
    .brake_look_seconds = 0.4f,
    .far_bend_look_seconds = 1.2f,
    .heading_time_constant_seconds = 0.1f,
    .pursuit_slot_meters = 4.0f,
    .pursuit_look_seconds = 0.8f,
    .orca_margin_meters = 0.5f,
    .squeezed_gap_meters = 3.0f,
    .column_gap_radii = 2.2f,
    .corner_cut_margin_meters = 8.0f,
    .lag_wait_meters = 8.0f,
};

// The tuned values, by faction.
static const FactionTuning FACTION_TUNING[FACTION_COUNT] = {
    [FACTION_CRAYVEN] = {
        .brake_look_seconds = 0.4f,
        .far_bend_look_seconds = 1.6f,
        .heading_time_constant_seconds = 0.05f,
        .pursuit_slot_meters = 4.0f,
        .pursuit_look_seconds = 0.8f,
        .orca_margin_meters = 1.0f,
        .squeezed_gap_meters = 3.0f,
        .column_gap_radii = 2.2f,
        .corner_cut_margin_meters = 8.0f,
        .lag_wait_meters = 16.0f,
    },
    // A longer brake look lets the slowly braking Order units stop before blocking terrain.
    [FACTION_ORDER] = {
        .brake_look_seconds = 1.6f,
        .far_bend_look_seconds = 1.2f,
        .heading_time_constant_seconds = 0.1f,
        .pursuit_slot_meters = 8.0f,
        .pursuit_look_seconds = 0.8f,
        .orca_margin_meters = 0.5f,
        .squeezed_gap_meters = 3.0f,
        .column_gap_radii = 2.2f,
        .corner_cut_margin_meters = 8.0f,
        .lag_wait_meters = 8.0f,
    },
    // The Phoenix units keep the untuned values: their tuned set scored better in the scans but
    // did not look better in the replay (NOTES.md, Rejected features).
    [FACTION_PHOENIX] = {
        .brake_look_seconds = 0.4f,
        .far_bend_look_seconds = 1.2f,
        .heading_time_constant_seconds = 0.1f,
        .pursuit_slot_meters = 4.0f,
        .pursuit_look_seconds = 0.8f,
        .orca_margin_meters = 0.5f,
        .squeezed_gap_meters = 3.0f,
        .column_gap_radii = 2.2f,
        .corner_cut_margin_meters = 8.0f,
        .lag_wait_meters = 8.0f,
    },
};


// `get_tuning` picks the row of the faction of `unit_type`, or the untuned row with the switch
// off, so that `--factiontuning off` keeps the output from before the tuning.
const FactionTuning *get_tuning(const UnitType *unit_type) {
    if (!is_faction_tuning_on)
        return &DEFAULT_TUNING;
    return &FACTION_TUNING[unit_type->faction];
}
