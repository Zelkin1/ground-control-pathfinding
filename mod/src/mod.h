// Shared declarations of the game mod. The loader (wsock32.dll: proxy.c, mod.c, patch.c, hooks.c,
// reload.c) and the core (gcmod_core.dll: core.c, heap.c, config.c, bridge.c, overlay.c) each
// implement their part; core_api.h connects them.
#ifndef MOD_H
#define MOD_H

#include <stdbool.h>
#include <stdint.h>

#include "core_api.h"

// Both DLLs.
// `write_log` appends one formatted line to gcmod.log, next to gc.exe, and flushes it, so the
// line survives a crash of the game. It does nothing when the log could not be opened. Once the
// log reaches 1000 lines, the newest 900 replace it; a line longer than 1023 characters is cut.
void write_log(const char *format, ...);
// `get_game_folder_path` returns the folder of gc.exe, with a trailing backslash.
const char *get_game_folder_path(void);

// Loader.
// `start_mod` opens the log and installs the hooks. proxy.c calls it once, when gc.exe loads the
// DLL; the core loads later, at the first frames that the game shows.
void start_mod(void);
// `write_log_line` appends `line` and a line end to gcmod.log, and flushes it.
void write_log_line(const char *line);
// `install_hook` makes the game function at `address` jump to `hook` (patch.c). The first
// `stolen_length` bytes of the function (whole instructions, 5 to 16 bytes) move into a
// trampoline. It returns the trampoline, which runs the original function, or NULL on failure.
void *install_hook(uintptr_t address, int stolen_length, const void *hook);
// `install_game_hooks` hooks the game functions that the mod needs (hooks.c). It returns false
// when a hook fails; the movement hook comes last, so without it the game moves units as before.
bool install_game_hooks(void);
// `start_core_loader` sets the paths of the core file and of its copy, and starts watching the
// game folder for changes (reload.c).
void start_core_loader(void);
// `update_core` loads the core at its first call, and loads it again at the first call after the
// core file has changed and no other program has it open. It must run between game frames, on
// the thread of the game.
void update_core(void);
// `get_core` returns the entry points of the loaded core, or NULL without one.
const CoreApi *get_core(void);
// `remember_map_name` keeps the name of the map that the game loaded, so that a core loaded
// during a mission can select its clearance fields.
void remember_map_name(const char *map_name);

// Core.
// Settings of gcmod.ini (config.c) that are not switches of the simulator.
typedef struct {
    int toggle_key;           // virtual-key code of the key that turns the mod on or off
} ModConfig;

// `read_config` sets the switches (switches.c) and `config` from gcmod.ini in the game folder.
// Lines it cannot read go to the log. Without the file, everything keeps its default.
void read_config(ModConfig *config);

// Bridge between the game and the formation code (bridge.c). The hooks pass game objects as
// plain pointers.
// `start_bridge` prepares the formation code (it allocates about 13 MB of lane tables) and sets
// the key that turns the mod on or off during a mission. It must run before the first hook.
void start_bridge(int virtual_key);
// `stop_bridge` stops the members of every squad in formation following where they stand.
void stop_bridge(void);
// `note_squad_update` records that the game updates `squad_object` in this frame.
void note_squad_update(void *squad_object);
// `note_route` takes the route that the game hands to the members of `squad_object` at a move
// order (`route_list` is a list of the game), for the formation set-up in the next frame.
void note_route(void *squad_object, const void *route_list);
// `tick_formation_mover` moves `mover` for one frame when it belongs to a squad in formation
// following, and returns true then; otherwise the game moves it.
bool tick_formation_mover(void *mover);
// `note_map_loaded` selects the clearance fields of the map whose grid the game just loaded.
void note_map_loaded(const char *map_name);
// `get_mod_status` sets `is_on` to whether the mod is on (the toggle key switches it), and
// returns true while a mission runs.
bool get_mod_status(bool *is_on);
// `note_squad_destroyed` forgets `squad_object`, which the game is about to destroy.
void note_squad_destroyed(void *squad_object);

// `draw_status_text` writes `text` in small grey letters at the top right of the frame that the
// D3DX context `context` is about to show (overlay.c).
void draw_status_text(void *context, const char *text);
// `count_frame` counts one shown frame and returns the frame rate, averaged over half a second.
int count_frame(void);
// `release_status_font` deletes the font of the status text, if `draw_status_text` made one.
void release_status_font(void);

// `get_core_heap` returns the heap of the core (heap.c), or NULL before its first allocation.
void *get_core_heap(void);

#endif
