// Loading of the core DLL, gcmod_core.dll, from the game folder. Windows keeps a loaded DLL file
// open, so the loader runs a copy of it from the temporary folder, and the file in the game folder
// can be replaced while the game runs. Windows signals `folder_change` when a file of the game
// folder changes, and `update_core` then reloads the core at the first frame at which the new
// file is complete:
//
//   old core stops its squads -> remove its hooks -> unload -> destroy its heap -> copy -> load
//   -> start new core (installs its hooks)
//
// A new core starts with no squad in formation following, its switches read again from
// gcmod.ini, and the clearance fields of the current map. All this runs between two game frames
// on the thread of the game, so no hook is inside the core while it changes. The core installs
// its own hooks through `CoreHost`, and the loader removes them before the unload, since a hook
// left in place would jump into the freed DLL.
#include <stdio.h>
#include <string.h>
#include <windows.h>

#include "mod.h"

#define CORE_FILE_NAME "gcmod_core.dll"                 // core file in the game folder
#define LOADED_CORE_FILE_NAME "gcmod_core_loaded.dll"   // its copy in the temporary folder
#define MAX_MAP_NAME_LENGTH 64
#define MAX_CORE_HOOKS 32                               // most hooks that one core installs

// A hook that the core installed, with what `remove_hook` needs to undo it.
typedef struct {
    uintptr_t address;
    void *trampoline;
    int stolen_length;
} CoreHook;

static char core_path[MAX_PATH];
static char loaded_core_path[MAX_PATH];
static CoreHost core_host;
static HMODULE core_module;
static const CoreApi *core;
static int load_count;
static char map_name[MAX_MAP_NAME_LENGTH];
static CoreHook core_hooks[MAX_CORE_HOOKS];     // hooks of the loaded core
static int core_hook_count;

// Change notification of the game folder, and whether a change since the last load could be a
// new core file. The log of the mod is in the same folder, so most changes are log lines.
static HANDLE folder_change = INVALID_HANDLE_VALUE;
static bool is_change_pending = true;

// Write time of the last core file that `load_core` tried; that file is not tried again.
static FILETIME tried_write_time;
static bool has_tried_write_time;


// `install_core_hook` installs a hook for the core and records it for `remove_core_hooks`.
static void *install_core_hook(uintptr_t address, int stolen_length, const void *hook) {
    if (core_hook_count == MAX_CORE_HOOKS) {
        write_log("core hook at 0x%x refused: the core already has %d hooks", (unsigned)address,
                  MAX_CORE_HOOKS);
        return NULL;
    }
    void *trampoline = install_hook(address, stolen_length, hook);
    if (!trampoline)
        return NULL;
    core_hooks[core_hook_count] = (CoreHook){
        .address = address,
        .trampoline = trampoline,
        .stolen_length = stolen_length,
    };
    core_hook_count++;
    return trampoline;
}


// `remove_core_hooks` removes every hook that the core installed. A hook that cannot be removed
// would jump into the core after the unload, so its address goes to the log.
static void remove_core_hooks(void) {
    for (int hook_index = 0; hook_index < core_hook_count; hook_index++) {
        const CoreHook *hook = &core_hooks[hook_index];
        if (!remove_hook(hook->address, hook->trampoline, hook->stolen_length))
            write_log("cannot remove the core hook at 0x%x", (unsigned)hook->address);
    }
    core_hook_count = 0;
}


// `start_core_loader` sets the paths and the services that the core gets.
void start_core_loader(void) {
    snprintf(core_path, sizeof core_path, "%s%s", get_game_folder_path(), CORE_FILE_NAME);
    char temporary_folder[MAX_PATH];
    DWORD folder_length = GetTempPathA(MAX_PATH, temporary_folder);
    if (folder_length == 0 || folder_length >= MAX_PATH)
        temporary_folder[0] = '\0';
    snprintf(loaded_core_path, sizeof loaded_core_path, "%s%s", temporary_folder,
             LOADED_CORE_FILE_NAME);
    core_host = (CoreHost){
        .write_log_line = write_log_line,
        .game_folder_path = get_game_folder_path(),
        .install_hook = install_core_hook,
    };

    // A new core file comes as a write into the old file, or as a new file of that name.
    DWORD change_filter = FILE_NOTIFY_CHANGE_LAST_WRITE | FILE_NOTIFY_CHANGE_FILE_NAME;
    folder_change = FindFirstChangeNotificationA(get_game_folder_path(), FALSE, change_filter);
    if (folder_change == INVALID_HANDLE_VALUE)
        write_log("no change notification for the game folder (error %lu): no core reloads",
                  GetLastError());
}


// `get_core` returns the entry points of the loaded core.
const CoreApi *get_core(void) {
    return core;
}


// `remember_map_name` copies the map name for later loads of the core.
void remember_map_name(const char *name) {
    snprintf(map_name, sizeof map_name, "%s", name);
}


// `read_write_time` reads the last write time of the core file without opening it. It returns
// false when the file does not exist.
static bool read_write_time(FILETIME *write_time) {
    WIN32_FILE_ATTRIBUTE_DATA attributes;
    if (!GetFileAttributesExA(core_path, GetFileExInfoStandard, &attributes))
        return false;
    *write_time = attributes.ftLastWriteTime;
    return true;
}


// `read_complete_write_time` reads the last write time of the core file once no other program
// has it open, so a copy into it is complete. It returns false while the file is missing or open.
static bool read_complete_write_time(FILETIME *write_time) {
    HANDLE file = CreateFileA(core_path, GENERIC_READ, 0, NULL, OPEN_EXISTING,
                              FILE_ATTRIBUTE_NORMAL, NULL);
    if (file == INVALID_HANDLE_VALUE)
        return false;
    bool is_read = GetFileTime(file, NULL, NULL, write_time);
    CloseHandle(file);
    return is_read;
}


// `is_tried_write_time` tells whether `write_time` is that of the core file tried last.
static bool is_tried_write_time(const FILETIME *write_time) {
    return has_tried_write_time && CompareFileTime(write_time, &tried_write_time) == 0;
}


// `unload_core` stops the core, removes its hooks, unloads it and frees all its memory. The heap
// goes last, because the C runtime of the core can still free memory while it unloads.
static void unload_core(void) {
    if (!core)
        return;
    HANDLE heap = core->get_core_heap();
    core->stop_core();
    remove_core_hooks();
    FreeLibrary(core_module);
    if (heap)
        HeapDestroy(heap);
    core = NULL;
    core_module = NULL;
    write_log("core unloaded");
}


// `load_core` copies the core file, loads the copy and starts it. A core loaded during a mission
// also gets the map name, as at a map load. On failure, the game moves every unit itself.
static void load_core(void) {
    if (!CopyFileA(core_path, loaded_core_path, FALSE)) {
        write_log("cannot copy %s to %s (error %lu)", core_path, loaded_core_path, GetLastError());
        return;
    }
    core_module = LoadLibraryA(loaded_core_path);
    if (!core_module) {
        write_log("cannot load %s (error %lu)", loaded_core_path, GetLastError());
        return;
    }
    StartCoreFunction start_core = (StartCoreFunction)GetProcAddress(core_module,
                                                                     CORE_START_EXPORT);
    if (!start_core) {
        write_log("%s has no %s", CORE_FILE_NAME, CORE_START_EXPORT);
        FreeLibrary(core_module);
        core_module = NULL;
        return;
    }
    load_count++;
    write_log("core load %d", load_count);
    core = start_core(&core_host);
    if (map_name[0])
        core->note_map_loaded(map_name);
}


// `update_core` looks at the core file after a change in the game folder. While a new file is
// still open in another program, it looks again at each frame. A file that failed to load is not
// tried again until it changes.
void update_core(void) {
    bool has_folder_changed = folder_change != INVALID_HANDLE_VALUE &&
                              WaitForSingleObject(folder_change, 0) == WAIT_OBJECT_0;
    if (has_folder_changed) {
        FindNextChangeNotification(folder_change);
        is_change_pending = true;
    }
    if (!is_change_pending)
        return;

    // The open for the complete check comes only after a new write time, so that a look caused
    // by a log line could never block a copy that starts at that moment.
    FILETIME write_time;
    if (!read_write_time(&write_time) || is_tried_write_time(&write_time)) {
        is_change_pending = false;
        return;
    }
    if (!read_complete_write_time(&write_time))
        return;
    is_change_pending = false;
    if (is_tried_write_time(&write_time))
        return;
    tried_write_time = write_time;
    has_tried_write_time = true;
    unload_core();
    load_core();
}
