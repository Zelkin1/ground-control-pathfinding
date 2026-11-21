// Start of the loader DLL and the log file of the mod. `start_mod` runs inside `DllMain`
// (proxy.c), before the code of gc.exe starts: it opens the log and patches the game functions
// (hooks.c), which take effect once a mission runs. The core of the mod loads later (reload.c),
// outside `DllMain`, and writes its lines through `write_log_line`.
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <windows.h>

#include "mod.h"

#define LOG_FILE_NAME "gcmod.log"
#define LOG_LINE_LIMIT 1000     // line count at which `rewrite_log` drops the oldest lines
#define LOG_KEPT_LINES 900      // newest lines that `rewrite_log` keeps
#define LOG_LINE_BYTES 1024     // longest line, with its terminating zero; longer lines are cut
#define MOD_VERSION "0.4.0"

static FILE *log_file;
static char log_path[MAX_PATH];
static int log_line_count;      // lines in the log file
// Ring of the newest `LOG_KEPT_LINES` lines; once it is full, the oldest one sits at
// `next_kept_index`, where the next line goes.
static char kept_lines[LOG_KEPT_LINES][LOG_LINE_BYTES];
static int next_kept_index;
static char game_folder_path[MAX_PATH];


// `find_game_folder` stores the folder of gc.exe in `game_folder_path`. The mod files sit next to
// gc.exe, whatever the working folder of the game is.
static void find_game_folder(void) {
    DWORD path_length = GetModuleFileNameA(NULL, game_folder_path, MAX_PATH);
    if (path_length == 0 || path_length >= MAX_PATH) {
        game_folder_path[0] = '\0';
        return;
    }
    char *last_separator = strrchr(game_folder_path, '\\');
    if (last_separator)
        last_separator[1] = '\0';
}


// `get_game_folder_path` returns the folder that `find_game_folder` stored.
const char *get_game_folder_path(void) {
    return game_folder_path;
}


// `rewrite_log` replaces the log with a line that marks the cut and the newest `LOG_KEPT_LINES`
// lines, oldest first. Rewriting only when the log reaches `LOG_LINE_LIMIT`, not at each line,
// keeps the cost to one rewrite per `LOG_LINE_LIMIT - LOG_KEPT_LINES` lines.
static void rewrite_log(void) {
    log_file = freopen(log_path, "w", log_file);
    if (!log_file)
        return;
    fputs("(older lines of the log removed)\n", log_file);
    for (int line_offset = 0; line_offset < LOG_KEPT_LINES; line_offset++) {
        int line_index = (next_kept_index + line_offset) % LOG_KEPT_LINES;
        fputs(kept_lines[line_index], log_file);
        fputc('\n', log_file);
    }
    fflush(log_file);
    log_line_count = LOG_KEPT_LINES + 1;
}


// `write_log` writes one line and flushes it at once. It also keeps the line in `kept_lines`
// for `rewrite_log`.
void write_log(const char *format, ...) {
    if (!log_file)
        return;
    char *line = kept_lines[next_kept_index];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(line, LOG_LINE_BYTES, format, arguments);
    va_end(arguments);
    next_kept_index = (next_kept_index + 1) % LOG_KEPT_LINES;

    // Write the line, and drop the oldest lines once the log is full.
    fputs(line, log_file);
    fputc('\n', log_file);
    fflush(log_file);
    log_line_count++;
    if (log_line_count >= LOG_LINE_LIMIT)
        rewrite_log();
}


// `write_log_line` writes `line` as it is, for the core, which formats its lines itself.
void write_log_line(const char *line) {
    write_log("%s", line);
}


// `start_mod` opens a new log next to gc.exe, then installs the hooks.
void start_mod(void) {
    find_game_folder();
    snprintf(log_path, sizeof log_path, "%s%s", game_folder_path, LOG_FILE_NAME);
    log_file = fopen(log_path, "w");
    write_log("gcmod %s loaded", MOD_VERSION);
    start_core_loader();
    install_game_hooks();
}
