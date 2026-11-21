// Left click on empty ground (`--clickdeselect`). In the 3D view, each press of the select button
// (the left mouse button, inferred from its use) opens a selection box, and at the release a box
// that did not grow into a drag counts as a click. When no entity is under the cursor, the base
// game leaves the selection as it is, and with its option `leftclickmovement` on it also gives the
// selected squads a move order to the clicked point. The squads stay selected, and every later
// order goes to them. With the switch, such a click deselects every squad and moves none.
//
// A hook on the mouse handler of the view runs `prepare_ground_click` before the game handles the
// mouse and `finish_ground_click` after it. While the option is held at 0, the game looks for the
// entity nearest the cursor within 30 m (0x4af660, measured in 3D from the point of the ground
// under the cursor) and selects its squad when it is one of the player's. Units have radii of 1 to
// 6.5 m, so a click well beside a unit would still select it. A second hook keeps the entity of
// that search only when the cursor point lies within its body radius plus `PICK_MARGIN_METERS`.
// The squads are deselected only when no entity is left. The core installs both hooks at each
// load, and the loader removes them before the unload.
#include <stdbool.h>
#include <stdint.h>

#include "mod.h"
#include "game_object.h"
#include "sim.h"

#define THISCALL __attribute__((thiscall))

// Mouse handler of the 3D view: a virtual thiscall method without a name in gc.exe, with one
// argument, the order that the interface builds (`ret 4`). Its first instructions are push -1,
// push imm32 (7 bytes, decompilation/tools/gcdis.py func).
#define VIEW_MOUSE_ADDRESS 0x4d09f0
#define VIEW_MOUSE_STOLEN_BYTES 7
#define VIEW_HANDLER_INTERFACE 4            // pointer to the interface object (`Ingameinterface`)

// Fields of the interface object.
#define INTERFACE_CURSOR_ENTITY 0x9e4       // 16-bit id of the entity under the cursor
#define INTERFACE_IS_BOX_OPEN 0x10d04       // byte, not 0 while a selection box is open
#define INTERFACE_IS_SELECT_BOX 0x10d05     // byte, not 0 when the select button opened the box
#define INTERFACE_IS_BOX_DRAGGED 0x10d10    // byte, not 0 once the box grew into a drag

// Client entities, by id: a table of pointers in gc.exe. Ids from `CLIENT_ENTITY_COUNT` on have no
// entity; the game uses 600 for "no entity". A client entity keeps its shown position as x,
// height, y floats, and its type, whose body radius the formation layout of 0x4b1020 reads.
#define CLIENT_ENTITY_TABLE_ADDRESS 0x6e6320
#define CLIENT_ENTITY_COUNT 600
#define NO_ENTITY 600
#define ENTITY_SHOWN_POSITION 0x44
#define ENTITY_TYPE 4
#define ENTITY_TYPE_RADIUS 0x88

// Near-entity search of the view (thiscall: interface, float radius). Its first instructions are
// push -1, push imm32 (7 bytes). 0x4af810 (thiscall: interface, id) makes an entity the one under
// the cursor; with `NO_ENTITY`, it clears it and resets the hover state.
#define PICK_NEAREST_ADDRESS 0x4af660
#define PICK_NEAREST_STOLEN_BYTES 7
#define SET_CURSOR_ENTITY_ADDRESS 0x4af810
#define CURSOR_POINT_ADDRESS 0x6e7748       // point of the ground under the cursor: x, height, y
#define PICK_MARGIN_METERS 1.0f             // a click this far past the body still selects

#define LEFT_CLICK_MOVEMENT_ADDRESS 0x67773c    // game option `leftclickmovement`, 32 bits

// Game functions. The button tests read the mouse object and return 1 in the frame in which their
// button goes up.
#define SELECT_BUTTON_RELEASED_ADDRESS 0x4c2b20 // test of the select button (left)
#define ORDER_BUTTON_RELEASED_ADDRESS 0x4c2b80  // test of the order button (right)
#define SELECT_SQUADS_ADDRESS 0x4aee70          // `Ingameinterface::SelectSquads`
#define ALL_SQUADS (-1)                         // squad number of every squad with members

typedef void(THISCALL *ViewMouseFunction)(void *view_handler, void *order);
typedef int (*ButtonTestFunction)(void);
// `Ingameinterface::SelectSquads` selects (`is_selected` 1) or deselects (0) the squad with the
// number `squad_number`, or every squad with members for `ALL_SQUADS`.
typedef void(THISCALL *SelectSquadsFunction)(void *game_interface, int squad_number,
                                             int is_selected);
typedef void(THISCALL *PickNearestFunction)(void *game_interface, float radius);
typedef void(THISCALL *SetCursorEntityFunction)(void *game_interface, unsigned entity_id);

static ViewMouseFunction original_view_mouse;   // trampoline that runs the game handler
static PickNearestFunction original_pick_nearest;       // trampoline of the near-entity search
static bool is_ground_click_running;            // the game handler runs for a ground click

// Value that `leftclickmovement` had before `prepare_ground_click` set it to 0.
static int saved_left_click_movement;


// `get_left_click_movement` returns the game option `leftclickmovement`, which the mod writes.
static int *get_left_click_movement(void) {
    return (int *)LEFT_CLICK_MOVEMENT_ADDRESS;
}


// `get_cursor_entity` returns the client entity under the cursor of the interface object
// `game_interface`, or NULL without one.
static const void *get_cursor_entity(const void *game_interface) {
    int id = read_u16(game_interface, INTERFACE_CURSOR_ENTITY);
    if (id >= CLIENT_ENTITY_COUNT)
        return NULL;
    const void *const *table = (const void *const *)CLIENT_ENTITY_TABLE_ADDRESS;
    return table[id];
}


// `has_cursor_entity` tells whether an entity is under the cursor of the interface object
// `game_interface`.
static bool has_cursor_entity(const void *game_interface) {
    return get_cursor_entity(game_interface) != NULL;
}


// `is_cursor_on_body` tells whether the point of the ground under the cursor lies within the body
// radius of `entity` plus `PICK_MARGIN_METERS`, in 3D, as the near-entity search measures.
static bool is_cursor_on_body(const void *entity) {
    const void *cursor = (const void *)CURSOR_POINT_ADDRESS;
    float distance_squared = 0;
    for (int axis_offset = 0; axis_offset < 3 * (int)sizeof(float); axis_offset += sizeof(float)) {
        float entity_coordinate = read_float(entity, ENTITY_SHOWN_POSITION + axis_offset);
        float d = read_float(cursor, axis_offset) - entity_coordinate;        // offset on one axis
        distance_squared += d * d;
    }
    const void *entity_type = read_pointer(entity, ENTITY_TYPE);
    float reach = read_float(entity_type, ENTITY_TYPE_RADIUS) + PICK_MARGIN_METERS;
    return distance_squared <= reach * reach;
}


// `is_button_released` tells whether a mouse button goes up in this frame. The game closes a
// selection box at the release of either button.
static bool is_button_released(void) {
    ButtonTestFunction is_left_up = (ButtonTestFunction)SELECT_BUTTON_RELEASED_ADDRESS;
    ButtonTestFunction is_right_up = (ButtonTestFunction)ORDER_BUTTON_RELEASED_ADDRESS;
    return is_left_up() || is_right_up();
}


// `is_ground_click` tells whether a click of the select button ends in this frame with no entity
// under the cursor of the interface object `game_interface`.
static bool is_ground_click(const void *game_interface) {
    bool is_open = read_byte(game_interface, INTERFACE_IS_BOX_OPEN);
    bool is_sel = read_byte(game_interface, INTERFACE_IS_SELECT_BOX);         // select button box
    bool is_drag = read_byte(game_interface, INTERFACE_IS_BOX_DRAGGED);       // box is a drag
    bool is_click = is_open && is_sel && !is_drag;
    return is_click && is_button_released() && !has_cursor_entity(game_interface);
}


// `prepare_ground_click` returns whether the mouse handler `view_handler` is about to handle a left
// click on empty ground, and holds `leftclickmovement` at 0 for it.
static bool prepare_ground_click(void *view_handler) {
    const void *ui = read_pointer(view_handler, VIEW_HANDLER_INTERFACE);        // interface object
    if (!is_ground_click(ui))
        return false;

    // With the option at 0, the game searches for an entity near the cursor instead of moving the
    // selected squads to the clicked point.
    int *option = get_left_click_movement();
    saved_left_click_movement = *option;
    *option = 0;
    return true;
}


// `finish_ground_click` restores `leftclickmovement` after a click that `prepare_ground_click`
// found, and deselects every squad when the click selected nothing.
static void finish_ground_click(void *view_handler) {
    *get_left_click_movement() = saved_left_click_movement;

    // The handler leaves the box open when another branch took the click, such as the one of a
    // held order key; an entity under the cursor now is the one that the search near it found.
    void *ui = read_pointer(view_handler, VIEW_HANDLER_INTERFACE);        // interface object
    bool is_open = read_byte(ui, INTERFACE_IS_BOX_OPEN);
    if (is_open || has_cursor_entity(ui))
        return;
    SelectSquadsFunction select = (SelectSquadsFunction)SELECT_SQUADS_ADDRESS;
    select(ui, ALL_SQUADS, false);
    write_log("left click on empty ground: squads deselected");
}


// `hook_view_mouse` runs the game handler of the view between the two steps of a left click on
// empty ground.
static void THISCALL hook_view_mouse(void *view_handler, void *order) {
    bool is_click = prepare_ground_click(view_handler);
    is_ground_click_running = is_click;
    original_view_mouse(view_handler, order);
    is_ground_click_running = false;
    if (is_click)
        finish_ground_click(view_handler);
}


// `hook_pick_nearest` runs the near-entity search, and during a ground click drops the entity it
// found when the cursor point lies outside the body of that entity.
static void THISCALL hook_pick_nearest(void *game_interface, float radius) {
    original_pick_nearest(game_interface, radius);
    if (!is_ground_click_running)
        return;
    const void *entity = get_cursor_entity(game_interface);
    if (!entity || is_cursor_on_body(entity))
        return;
    SetCursorEntityFunction set_cursor_entity = (SetCursorEntityFunction)SET_CURSOR_ENTITY_ADDRESS;
    set_cursor_entity(game_interface, NO_ENTITY);
}


// `install_ground_click_hook` hooks the mouse handler of the view and the near-entity search with
// `install_hook` while the switch is on; without the switch, both game functions stay as they are.
void install_ground_click_hook(InstallHookFunction install_hook) {
    if (!is_click_deselect_on)
        return;
    original_view_mouse = install_hook(VIEW_MOUSE_ADDRESS, VIEW_MOUSE_STOLEN_BYTES,
                                       (const void *)hook_view_mouse);
    if (!original_view_mouse)
        write_log("hook of the view mouse handler failed: left clicks keep the game behavior");
    original_pick_nearest = install_hook(PICK_NEAREST_ADDRESS, PICK_NEAREST_STOLEN_BYTES,
                                         (const void *)hook_pick_nearest);
    if (!original_pick_nearest)
        write_log("hook of the near-entity search failed: a click within 30 m selects a unit");
}
