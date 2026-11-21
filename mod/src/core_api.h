// Interface between the two DLLs of the mod. The loader, wsock32.dll, stays loaded while the game
// runs: it holds the proxy, the code hooks and the log. The core, gcmod_core.dll, holds the
// configuration, the bridge, the HUD text and the formation code. The loader runs the core
// from a copy, so gcmod_core.dll in the game folder can be replaced while the game runs, and it
// loads the core again when that file changes (reload.c).
#ifndef CORE_API_H
#define CORE_API_H

#include <stdbool.h>
#include <stdint.h>

#define CORE_START_EXPORT "start_core"          // name of the export that starts the core

// `InstallHookFunction` makes the game function at `address` jump to `hook`, as `install_hook` in
// mod.h describes, and returns the trampoline that runs the original, or NULL on failure.
typedef void *(*InstallHookFunction)(uintptr_t address, int stolen_length, const void *hook);

// Services of the loader for the core. They stay valid while the game runs.
typedef struct {
    void (*write_log_line)(const char *line);  // appends one line to gcmod.log
    const char *game_folder_path;              // folder of gc.exe, with a trailing backslash
    // `install_hook` hooks a game function for the core. The loader removes every such hook after
    // `stop_core` and before it unloads the core, so a new hook needs only a new core. Loaders
    // before gcmod 0.3.0 pass a `CoreHost` without this field.
    InstallHookFunction install_hook;
} CoreHost;

// Entry points of the core. The hooks call the first six as the functions of the same names in
// mod.h describe.
typedef struct {
    void (*note_squad_update)(void *squad_object);
    void (*note_route)(void *squad_object, const void *route_list);
    bool (*tick_formation_mover)(void *mover);
    void (*note_map_loaded)(const char *map_name);
    void (*note_squad_destroyed)(void *squad_object);
    void (*note_frame_shown)(void *context);

    // `stop_core` hands every squad back to the game before the core unloads.
    void (*stop_core)(void);
    // `get_core_heap` returns the heap that holds all memory of the core (heap.c). The loader
    // destroys it after it unloads the core.
    void *(*get_core_heap)(void);
    // `note_group_positions` may hand the positions of a group order out again (mod.h). It comes
    // last, so that an older loader, which does not call it, still works with a newer core.
    void (*note_group_positions)(void *game_interface);
} CoreApi;

// `start_core`, the export of the core, reads gcmod.ini, prepares the core and returns its entry
// points. `host` must stay valid while the core is loaded.
typedef const CoreApi *(*StartCoreFunction)(const CoreHost *host);

#endif
