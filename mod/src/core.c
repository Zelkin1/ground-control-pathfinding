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
#define KEY_TEXT_SIZE 64                      // room for the line that names the keys
#define HUD_KEY VK_F10                       // key that shows or hides the HUD
#define KEY_DOWN_BIT 0x8000                   // GetAsyncKeyState: the key is down now

static const CoreHost *host;
static SYSTEMTIME core_load_time;             // local time of the last core load
static ModConfig mod_config = {
    .toggle_key = VK_F9,
    .is_log_on = true,
    .middle_rotate_sensitivity = 1,
};
static bool is_hud_visible = true;
static bool was_hud_key_down;


// `write_log` formats one line and hands it to the loader, unless gcmod.ini turned the log off.
// The loader still writes its own few lines, such as each core load, since it never reads
// gcmod.ini.
void write_log(const char *format, ...) {
    if (!host || !mod_config.is_log_on)
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


// `update_hud_visibility` shows or hides the HUD when the HUD key goes down.
static void update_hud_visibility(void) {
    bool is_key_down = (GetAsyncKeyState(HUD_KEY) & KEY_DOWN_BIT) != 0;
    if (is_key_down && !was_hud_key_down)
        is_hud_visible = !is_hud_visible;
    was_hud_key_down = is_key_down;
}


// `note_frame_shown` counts the frame, lets the interface scale start after a core load and,
// during a mission, writes the HUD at the top right of it: the state of the mod, the frame rate,
// the local time and the time of the last core load (which shows whether a new core is in use),
// then the keys.
static void note_frame_shown(void *context) {
    int frame_rate = count_frame();
    note_ui_scale_frame();
    bool is_on;
    if (!get_mod_status(&is_on))
        return;
    update_hud_visibility();
    if (!is_hud_visible)
        return;

    // Both keys are function keys (config.c takes F1 to F12).
    SYSTEMTIME now;
    GetLocalTime(&now);
    char status_text[STATUS_TEXT_SIZE];
    snprintf(status_text, sizeof status_text,
             "gcmod: %s   %d fps   %02d:%02d:%02d   updated %02d:%02d:%02d", is_on ? "on" : "off",
             frame_rate, now.wHour, now.wMinute, now.wSecond, core_load_time.wHour,
             core_load_time.wMinute, core_load_time.wSecond);
    char key_text[KEY_TEXT_SIZE];
    int toggle_key_number = mod_config.toggle_key - VK_F1 + 1;
    int hud_key_number = HUD_KEY - VK_F1 + 1;
    snprintf(key_text, sizeof key_text, "F%d: mod on/off   F%d: show/hide this text",
             toggle_key_number, hud_key_number);
    const char *lines[] = {status_text, key_text};
    draw_hud_text(context, lines, sizeof lines / sizeof lines[0]);
}


// `stop_core` hands the squads, the menu choice of the screen size and the screen size of the
// game code back to the game, and deletes the font of the HUD text, which does not live in the
// heap of the core.
static void stop_core(void) {
    stop_bridge();
    stop_resolution();
    stop_ui_scale();
    release_hud_font();
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
    .note_group_positions = note_group_positions,
};


// `start_core` reads gcmod.ini, prepares the bridge and installs the hooks of the core, then
// returns the entry points.
__declspec(dllexport) const CoreApi *start_core(const CoreHost *core_host) {
    host = core_host;
    GetLocalTime(&core_load_time);
    read_config(&mod_config);
    write_log("toggle key: 0x%02x", mod_config.toggle_key);
    start_bridge(mod_config.toggle_key);
    install_ground_click_hook(host->install_hook);
    install_middle_rotate_hook(host->install_hook, &mod_config);
    install_resolution_hook(host->install_hook, &mod_config);
    install_ui_scale_hook(host->install_hook, &mod_config);
    return &CORE_API;
}
