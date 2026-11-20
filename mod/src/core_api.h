// Interface between the two DLLs of the mod. The loader, wsock32.dll, stays loaded while the game
// runs: it holds the proxy, the code hooks and the log. The core, gcmod_core.dll, holds the
// configuration, the bridge, the status text and the formation code. The loader runs the core
// from a copy, so gcmod_core.dll in the game folder can be replaced while the game runs, and it
// loads the core again when that file changes (reload.c).
#ifndef CORE_API_H
#define CORE_API_H

#include <stdbool.h>

#define CORE_START_EXPORT "start_core"          // name of the export that starts the core

// Services of the loader for the core. They stay valid while the game runs.
typedef struct {
    void (*write_log_line)(const char *line);  // appends one line to gcmod.log
    const char *game_folder_path;              // folder of gc.exe, with a trailing backslash
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
} CoreApi;

// `start_core`, the export of the core, reads gcmod.ini, prepares the core and returns its entry
// points. `host` must stay valid while the core is loaded.
typedef const CoreApi *(*StartCoreFunction)(const CoreHost *host);

#endif
