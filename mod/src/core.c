// Entry of the core DLL, gcmod_core.dll. The loader (reload.c) calls `start_core` after each load
// and `stop_core` before each unload; between them, the hooks of the loader call the bridge
// through `CORE_API`. The core writes its log lines through the loader, which keeps gcmod.log
// open across reloads.
#include <stdarg.h>
#include <stdio.h>
#include <windows.h>

#include "mod.h"

#define LOG_LINE_SIZE 512                     // longest log line of the core
#define STATUS_TEXT_SIZE 64                   // room for the status text, frame rate and times

static const CoreHost *host;
static SYSTEMTIME core_load_time;             // local time of the last core load
static ModConfig mod_config = {
    .toggle_key = VK_F9,
};


// `write_log` formats one line and hands it to the loader.
void write_log(const char *format, ...) {
    if (!host)
        return;
    char line[LOG_LINE_SIZE];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(line, sizeof line, format, arguments);
    va_end(arguments);
    host->write_log_line(line);
}


// `get_game_folder_path` returns the folder of gc.exe that the loader found.
const char *get_game_folder_path(void) {
    return host->game_folder_path;
}


// `note_frame_shown` counts the frame and, during a mission, writes at the top right of it the
// state of the mod, the frame rate, the local time and the time of the last core load, which
// shows whether a new core is in use.
static void note_frame_shown(void *context) {
    int frame_rate = count_frame();
    bool is_on;
    if (!get_mod_status(&is_on))
        return;
    SYSTEMTIME now;
    GetLocalTime(&now);
    char text[STATUS_TEXT_SIZE];
    snprintf(text, sizeof text, "gcmod: %s   %d fps   %02d:%02d:%02d   updated %02d:%02d:%02d",
             is_on ? "on" : "off", frame_rate, now.wHour, now.wMinute, now.wSecond,
             core_load_time.wHour, core_load_time.wMinute, core_load_time.wSecond);
    draw_status_text(context, text);
}


// `stop_core` hands the squads back to the game and deletes the font of the status text, which
// does not live in the heap of the core.
static void stop_core(void) {
    stop_bridge();
    release_status_font();
}


static const CoreApi CORE_API = {
    .note_squad_update = note_squad_update,
    .note_route = note_route,
    .tick_formation_mover = tick_formation_mover,
    .note_map_loaded = note_map_loaded,
    .note_squad_destroyed = note_squad_destroyed,
    .note_frame_shown = note_frame_shown,
    .stop_core = stop_core,
    .get_core_heap = get_core_heap,
};


// `start_core` reads gcmod.ini and prepares the bridge, then returns the entry points.
__declspec(dllexport) const CoreApi *start_core(const CoreHost *core_host) {
    host = core_host;
    GetLocalTime(&core_load_time);
    read_config(&mod_config);
    write_log("toggle key: 0x%02x", mod_config.toggle_key);
    start_bridge(mod_config.toggle_key);
    return &CORE_API;
}
