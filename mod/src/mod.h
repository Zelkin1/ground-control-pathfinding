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
// `remove_hook` restores the game function at `address` that `install_hook` patched, with the
// same `stolen_length`, and frees its `trampoline`. It returns false when the code pages cannot be
// opened for writing; the hook then stays, and so does the trampoline.
bool remove_hook(uintptr_t address, void *trampoline, int stolen_length);
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
    bool is_log_on;           // whether the core writes its lines to gcmod.log
    // Whether the video menu of the game also lists 1920x1080, 2560x1440 and 3840x2160
    // (`modern_resolution`).
    bool is_modern_resolution_on;
    // New size that the video menu applied last, in pixels (`resolution`, which the core writes);
    // 0 for a size of the game.
    int saved_resolution_width;
    int saved_resolution_height;
    // Factor by which the interface grows at the sizes that the mod gives the screen
    // (`ui_scale`); 0 when off.
    float ui_scale;
    // Factor on the camera turn per pixel of mouse movement while the middle button is down
    // (`middlerotate_sensitivity`, `--middlerotate`); 1 by default.
    float middle_rotate_sensitivity;
} ModConfig;

// `read_config` sets the switches (switches.c) and `config` from gcmod.ini in the game folder.
// Lines it cannot read go to the log. Without the file, everything keeps its default.
void read_config(ModConfig *config);
// `save_resolution_line` sets the line "resolution = `value`" in gcmod.ini and keeps the other
// lines. It returns false, and logs why, when it cannot replace the file.
bool save_resolution_line(const char *value);

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

// `draw_hud_text` writes `line_count` lines in small grey letters at the top right of the frame
// that the D3DX context `context` is about to show, one below the other (overlay.c). A line wider
// than 60 % of the frame is cut and ends in "...".
void draw_hud_text(void *context, const char *const *lines, int line_count);
// `count_frame` counts one shown frame and returns the frame rate, averaged over half a second.
int count_frame(void);
// `release_hud_font` deletes the font of the HUD text, if `draw_hud_text` made one.
void release_hud_font(void);

// `note_group_positions` hands out again the positions that the interface object `game_interface`
// laid out for the squads of a group order of the player, so that the squads keep their order
// from side to side (`--keeporder`, group_order.c). Without the switch, nothing changes.
void note_group_positions(void *game_interface);

// `install_ground_click_hook` makes a left click on empty ground in the 3D view deselect every
// squad (`--clickdeselect`, ground_click.c). It hooks the mouse handler of the view with
// `install_hook` (the service of the loader, which removes the hook before the core unloads), and
// does nothing without the switch. It must run after `read_config`.
void install_ground_click_hook(InstallHookFunction install_hook);

// `install_middle_rotate_hook` makes the mouse turn the camera while the middle button is down,
// and stops the screen borders from turning it (`--middlerotate`, middle_rotate.c), by a hook on
// the border rotation of the game through `install_hook`. The turn per pixel grows with the
// sensitivity in `config`. It does nothing without the switch, and must run after `read_config`.
void install_middle_rotate_hook(InstallHookFunction install_hook, const ModConfig *config);

// `install_resolution_hook` adds the new sizes that the display offers, up to the desktop size, to
// the video menu of the game and applies them (resolution.c), by hooks through `install_hook`, when
// `modern_resolution` is on. The size saved in `config` is the size of the first screen of the
// game with loader 0.4.0 (with an older loader, it comes at the first frames); each later change
// of size in the menu writes the `resolution` line of gcmod.ini.
void install_resolution_hook(InstallHookFunction install_hook, const ModConfig *config);
// `stop_resolution` must run before the core unloads. It sets the menu choice to 1024x768 during
// a new size, which the game code cannot read; the screen keeps its size.
void stop_resolution(void);
// `is_modern_screen_size` tells whether `width` by `height` is a new size of the video menu.
bool is_modern_screen_size(int width, int height);

// `install_ui_scale_hook` makes the menus, the in-game interface and the cursor `ui_scale` times
// larger at the sizes that the mod gives the screen (ui_scale.c), by hooks through `install_hook`
// and in the methods of the Direct3D device. It does nothing without `ui_scale`, and needs
// `modern_resolution`. It must run after `install_resolution_hook`.
void install_ui_scale_hook(InstallHookFunction install_hook, const ModConfig *config);
// `read_screen_size` sets `width` and `height` to the size of the D3DX context of `renderer`. The
// size fields of the renderer hold the smaller size of the game code while the scale acts.
void read_screen_size(const void *renderer, int *width, int *height);
// `stop_ui_scale` must run before the core unloads. It gives the game code the screen size again
// and restores the methods of the Direct3D device.
void stop_ui_scale(void);
// `note_ui_scale_frame` must run at each shown frame. At the first one after a core load during a
// modern size, it starts the scale, since the core reads the renderer only between frames.
void note_ui_scale_frame(void);

// `get_core_heap` returns the heap of the core (heap.c), or NULL before its first allocation.
void *get_core_heap(void);

#endif
