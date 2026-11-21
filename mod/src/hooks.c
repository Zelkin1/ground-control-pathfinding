// Hooks on eight game functions (docs/PATHFINDING.md). They hand game objects to the core of the mod
// (core_api.h) while one is loaded, and without a core the game runs as before:
// - `Squad::Update` 0x504950 runs once per squad and frame, for player and computer squads;
//   the bridge learns from it which squads exist.
// - `Mover::SetMove` 0x50e7e0 gives each member the route of its squad at a move order; the
//   bridge copies the route for the formation set-up.
// - *Tick* 0x50e5b0 moves one unit per frame; the bridge moves the members of squads in
//   formation following, and the original moves every other unit.
// - `PathFinder::LoadGrid` 0x4e8200 loads the grid of a map; the bridge then selects its
//   clearance fields.
// - `Squad::~Squad` 0x504510 runs when the game destroys a squad (the destructor of computer
//   squads, 0x502950, calls it too); the bridge drops the squad at once, so it never reads a
//   destroyed squad.
// - `CD3duContext::UpdateFrame` 0x4085c4 shows each finished frame (Flip or Blt); before it, the
//   hook loads or reloads the core (reload.c), and the core writes whether the mod is on
//   (overlay.c). It is a COM method of the D3DX utility library linked into gc.exe (stdcall, the
//   context first); its vtable entry is at 0x60f294.
// - `Ingameinterface::CalculateSquadFormationPoints` 0x4b1020 lays out one position per selected
//   squad for a move order of the player; the core may hand these positions out again
//   (group_order.c) before the preview shows them and the order uses them.
// - The constructor of the D3DX renderer 0x431cd0 (thiscall: window, windowed flag, D3DX device
//   index) runs in WinMain before the game creates its first screen; without a core, the hook
//   loads it there, so that the core can choose the size and mode of that first screen.
// Hooks added later live in the core, which installs them through `CoreHost` (core_api.h), so they
// change with a core reload.
// The stolen lengths end on instruction boundaries (decompilation/tools/gcdis.py func):
// `Squad::Update` push, push, mov, xor (6 bytes); *Tick*, `SetMove`, `Squad::~Squad` and
// `CalculateSquadFormationPoints` push -1, push imm32 (7 bytes); `LoadGrid` mov eax, fs:[0]
// (6 bytes); `UpdateFrame` mov eax, imm32 (5 bytes); the renderer constructor mov eax,
// [esp + 8], push esi (5 bytes). None of them is a relative jump or call.
#include <windows.h>

#include "mod.h"

#define THISCALL __attribute__((thiscall))
#define STDCALL __attribute__((stdcall))

#define SQUAD_UPDATE_ADDRESS 0x504950
#define SQUAD_UPDATE_STOLEN_BYTES 6
#define MOVER_TICK_ADDRESS 0x50e5b0
#define MOVER_TICK_STOLEN_BYTES 7
#define SET_MOVE_ADDRESS 0x50e7e0
#define SET_MOVE_STOLEN_BYTES 7
#define LOAD_GRID_ADDRESS 0x4e8200
#define LOAD_GRID_STOLEN_BYTES 6
#define UPDATE_FRAME_ADDRESS 0x4085c4
#define UPDATE_FRAME_STOLEN_BYTES 5
#define SQUAD_DESTRUCTOR_ADDRESS 0x504510
#define SQUAD_DESTRUCTOR_STOLEN_BYTES 7
#define GROUP_POSITIONS_ADDRESS 0x4b1020
#define GROUP_POSITIONS_STOLEN_BYTES 7
#define CREATE_RENDERER_ADDRESS 0x431cd0
#define CREATE_RENDERER_STOLEN_BYTES 5

typedef int(STDCALL *UpdateFrameFunction)(void *context, uint32_t flags);
typedef void(THISCALL *SquadDestructorFunction)(void *squad);
typedef int(THISCALL *SquadUpdateFunction)(void *squad);
typedef void(THISCALL *MoverTickFunction)(void *mover);
typedef void(THISCALL *SetMoveFunction)(void *mover, const void *route_list, float facing_x,
                                        float facing_y, void *squad);
typedef int(THISCALL *LoadGridFunction)(void *pathfinder, const char *map_name);
typedef void(THISCALL *GroupPositionsFunction)(void *game_interface);
typedef void *(THISCALL *CreateRendererFunction)(void *renderer, HWND window, int is_windowed,
                                                 int device_index);

static SquadUpdateFunction original_squad_update;
static MoverTickFunction original_mover_tick;
static SetMoveFunction original_set_move;
static LoadGridFunction original_load_grid;
static UpdateFrameFunction original_update_frame;
static SquadDestructorFunction original_squad_destructor;
static GroupPositionsFunction original_group_positions;
static CreateRendererFunction original_create_renderer;

// Thread of the frame hook, and whether the squad hook compared its own thread with it.
static DWORD frame_thread_id;
static bool is_thread_checked;


// `hook_update_frame` updates the core between two frames, lets it write the state of the mod,
// and then shows the frame with the original.
static int STDCALL hook_update_frame(void *context, uint32_t flags) {
    frame_thread_id = GetCurrentThreadId();
    update_core();
    const CoreApi *core = get_core();
    if (core)
        core->note_frame_shown(context);
    return original_update_frame(context, flags);
}


// `hook_create_renderer` loads the core before the game builds a D3DX renderer, unless one is
// loaded. A reload here could unload a core whose code is on the stack (the game also builds a
// renderer inside 0x403f20, which the core hooks), so reloads stay at the frame hook.
static void *THISCALL hook_create_renderer(void *renderer, HWND window, int is_windowed,
                                           int device_index) {
    if (!get_core())
        update_core();
    return original_create_renderer(renderer, window, is_windowed, device_index);
}


// `hook_squad_destructor` makes the core forget the squad before the game destroys it.
static void THISCALL hook_squad_destructor(void *squad) {
    const CoreApi *core = get_core();
    if (core)
        core->note_squad_destroyed(squad);
    original_squad_destructor(squad);
}


// `check_game_thread` logs once whether the game updates squads on the thread of the frame hook.
// On another thread, a reload of the core could unload it while a hook runs inside it.
static void check_game_thread(void) {
    if (is_thread_checked || !frame_thread_id)
        return;
    is_thread_checked = true;
    DWORD squad_thread_id = GetCurrentThreadId();
    if (squad_thread_id == frame_thread_id)
        write_log("squads and frames on thread %lu", squad_thread_id);
    else
        write_log("WARNING: squads on thread %lu, frames on thread %lu: a core reload is unsafe",
                  squad_thread_id, frame_thread_id);
}


// `hook_squad_update` records the squad, then runs the original update.
static int THISCALL hook_squad_update(void *squad) {
    check_game_thread();
    const CoreApi *core = get_core();
    if (core)
        core->note_squad_update(squad);
    return original_squad_update(squad);
}


// `hook_mover_tick` lets the core move formation members, and the original move the others.
static void THISCALL hook_mover_tick(void *mover) {
    const CoreApi *core = get_core();
    bool is_moved_by_core = core && core->tick_formation_mover(mover);
    if (!is_moved_by_core)
        original_mover_tick(mover);
}


// `hook_set_move` runs the original, which sets up the waypoints of the member, and then hands
// the route to the core. Every member of the squad gets the same route.
static void THISCALL hook_set_move(void *mover, const void *route_list, float facing_x,
                                   float facing_y, void *squad) {
    original_set_move(mover, route_list, facing_x, facing_y, squad);
    const CoreApi *core = get_core();
    if (core)
        core->note_route(squad, route_list);
}


// `hook_load_grid` loads the grid with the original, then passes the map name on.
static int THISCALL hook_load_grid(void *pathfinder, const char *map_name) {
    int result = original_load_grid(pathfinder, map_name);
    remember_map_name(map_name);
    const CoreApi *core = get_core();
    if (core)
        core->note_map_loaded(map_name);
    return result;
}


// `hook_group_positions` lays out the positions with the original, then lets the core hand them
// out again.
static void THISCALL hook_group_positions(void *game_interface) {
    original_group_positions(game_interface);
    const CoreApi *core = get_core();
    if (core)
        core->note_group_positions(game_interface);
}


// `install_game_hooks` installs the hooks in turn and logs the first one that fails.
bool install_game_hooks(void) {
    original_load_grid = install_hook(LOAD_GRID_ADDRESS, LOAD_GRID_STOLEN_BYTES,
                                      (const void *)hook_load_grid);
    if (!original_load_grid) {
        write_log("hook LoadGrid failed");
        return false;
    }
    original_set_move = install_hook(SET_MOVE_ADDRESS, SET_MOVE_STOLEN_BYTES,
                                     (const void *)hook_set_move);
    if (!original_set_move) {
        write_log("hook SetMove failed");
        return false;
    }
    original_squad_update = install_hook(SQUAD_UPDATE_ADDRESS, SQUAD_UPDATE_STOLEN_BYTES,
                                         (const void *)hook_squad_update);
    if (!original_squad_update) {
        write_log("hook Squad::Update failed");
        return false;
    }
    original_squad_destructor = install_hook(SQUAD_DESTRUCTOR_ADDRESS,
                                             SQUAD_DESTRUCTOR_STOLEN_BYTES,
                                             (const void *)hook_squad_destructor);
    if (!original_squad_destructor) {
        write_log("hook Squad destructor failed");
        return false;
    }
    original_update_frame = install_hook(UPDATE_FRAME_ADDRESS, UPDATE_FRAME_STOLEN_BYTES,
                                         (const void *)hook_update_frame);
    if (!original_update_frame) {
        write_log("hook UpdateFrame failed");
        return false;
    }
    original_group_positions = install_hook(GROUP_POSITIONS_ADDRESS, GROUP_POSITIONS_STOLEN_BYTES,
                                            (const void *)hook_group_positions);
    if (!original_group_positions) {
        write_log("hook CalculateSquadFormationPoints failed");
        return false;
    }
    original_mover_tick = install_hook(MOVER_TICK_ADDRESS, MOVER_TICK_STOLEN_BYTES,
                                       (const void *)hook_mover_tick);
    if (!original_mover_tick) {
        write_log("hook Tick failed");
        return false;
    }
    original_create_renderer = install_hook(CREATE_RENDERER_ADDRESS, CREATE_RENDERER_STOLEN_BYTES,
                                            (const void *)hook_create_renderer);
    if (!original_create_renderer) {
        write_log("hook of the renderer constructor failed");
        return false;
    }
    write_log("hooks installed");
    return true;
}
