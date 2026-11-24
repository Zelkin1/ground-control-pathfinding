// Reader of gcmod.ini, the configuration file of the mod in the game folder. Each line is
// "name = value"; text after '#' is a comment. A name is `toggle_key` (F1 to F12), `reverse_key`
// (alt, ctrl or shift), `log` (on or
// off), `modern_resolution` (on or off), `music_loop` (on or off), `disablehitslow` (on or off),
// `resolution` (off or a size
// such as 1920x1080, which
// the core itself writes), `ui_scale` (off or a factor above 1, such as
// 1.5), `middlerotate_sensitivity` (a factor above 0), `avoid` (the movement mode of the simulator option `--avoid`: form or coord), or a switch
// of the simulator without its leading "--" (docs/SWITCHES.md), with the value on or off.
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

#include "mod.h"
#include "sim.h"

#define CONFIG_FILE_NAME "gcmod.ini"
#define MAX_LINE_LENGTH 256
#define MAX_SWITCH_NAME_LENGTH 64
#define MAX_FUNCTION_KEY 12           // F12 is the last function key that `toggle_key` takes


// `trim_spaces` removes the spaces at both ends of `text` in place and returns its new start.
static char *trim_spaces(char *text) {
    while (isspace((unsigned char)*text))
        text++;
    char *end = text + strlen(text);
    while (end > text && isspace((unsigned char)end[-1]))
        end--;
    *end = '\0';
    return text;
}


// `read_function_key` returns the virtual-key code of "F1" to "F12" in `value`, or 0 for any
// other text.
static int read_function_key(const char *value) {
    if (toupper((unsigned char)value[0]) != 'F')
        return 0;
    int key_number = atoi(&value[1]);
    if (key_number < 1 || key_number > MAX_FUNCTION_KEY)
        return 0;
    return VK_F1 + key_number - 1;
}


// `read_modifier_key` returns the virtual-key code of "alt", "ctrl" or "shift" in `value`, or 0
// for any other text. Each code covers the left and the right key.
static int read_modifier_key(const char *value) {
    if (!strcmp(value, "alt"))
        return VK_MENU;
    if (!strcmp(value, "ctrl"))
        return VK_CONTROL;
    if (!strcmp(value, "shift"))
        return VK_SHIFT;
    return 0;
}


// `apply_resolution` reads the value of `resolution`: "off" or a size such as "1920x1080", into
// `config`.
static void apply_resolution(const char *value, ModConfig *config) {
    int width;
    int height;
    char extra;                       // a character after the size, which makes it unreadable
    bool is_size = sscanf(value, "%dx%d%c", &width, &height, &extra) == 2;
    config->saved_resolution_width = 0;
    config->saved_resolution_height = 0;
    if (is_size && width > 0 && height > 0) {
        config->saved_resolution_width = width;
        config->saved_resolution_height = height;
    } else if (strcmp(value, "off")) {
        write_log("gcmod.ini: resolution '%s' is not off or a size such as 1920x1080", value);
    }
}


// `apply_ui_scale` reads the value of `ui_scale`: "off" or a factor above 1, such as "1.5", into
// `config`.
static void apply_ui_scale(const char *value, ModConfig *config) {
    char *number_end;
    float factor = strtof(value, &number_end);
    bool is_number = number_end != value && *number_end == '\0';
    config->ui_scale = 0;
    if (is_number && factor > 1) {
        config->ui_scale = factor;
        write_log("ui_scale %.2f", factor);
    } else if (strcmp(value, "off")) {
        write_log("gcmod.ini: ui_scale '%s' is not off or a number above 1, such as 1.5", value);
    }
}


// `apply_middle_rotate_sensitivity` reads the value of `middlerotate_sensitivity`: a factor above
// 0, such as "1.5", into `config`.
static void apply_middle_rotate_sensitivity(const char *value, ModConfig *config) {
    char *number_end;
    float factor = strtof(value, &number_end);
    bool is_number = number_end != value && *number_end == '\0';
    if (is_number && factor > 0) {
        config->middle_rotate_sensitivity = factor;
        write_log("middlerotate_sensitivity %.2f", factor);
    } else {
        write_log("gcmod.ini: middlerotate_sensitivity '%s' is not a number above 0", value);
    }
}


// `apply_setting` applies one "name = value" pair and logs it, or logs why it cannot.
static void apply_setting(const char *name, const char *value, ModConfig *config) {
    if (!strcmp(name, "toggle_key")) {
        int key = read_function_key(value);
        if (key)
            config->toggle_key = key;
        else
            write_log("gcmod.ini: toggle_key '%s' is not F1 to F12", value);
        return;
    }
    if (!strcmp(name, "reverse_key")) {
        int key = read_modifier_key(value);
        if (key) {
            config->reverse_key = key;
            write_log("reverse_key %s", value);
        } else {
            write_log("gcmod.ini: reverse_key '%s' is not alt, ctrl or shift", value);
        }
        return;
    }
    if (!strcmp(name, "log")) {
        bool is_on = !strcmp(value, "on");
        bool is_off = !strcmp(value, "off");
        if (is_on || is_off)
            config->is_log_on = is_on;
        else
            write_log("gcmod.ini: log '%s' is not on or off", value);
        return;
    }
    if (!strcmp(name, "modern_resolution")) {
        bool is_on = !strcmp(value, "on");
        bool is_off = !strcmp(value, "off");
        if (is_on || is_off)
            config->is_modern_resolution_on = is_on;
        else
            write_log("gcmod.ini: modern_resolution '%s' is not on or off", value);
        return;
    }
    if (!strcmp(name, "disablehitslow")) {
        bool is_on = !strcmp(value, "on");
        bool is_off = !strcmp(value, "off");
        if (is_on || is_off) {
            config->is_hit_slowdown_disabled = is_on;
            write_log("disablehitslow %s", value);
        } else {
            write_log("gcmod.ini: disablehitslow '%s' is not on or off", value);
        }
        return;
    }
    if (!strcmp(name, "music_loop")) {
        bool is_on = !strcmp(value, "on");
        bool is_off = !strcmp(value, "off");
        if (is_on || is_off) {
            config->is_music_loop_on = is_on;
            write_log("music_loop %s", value);
        } else {
            write_log("gcmod.ini: music_loop '%s' is not on or off", value);
        }
        return;
    }
    if (!strcmp(name, "resolution")) {
        apply_resolution(value, config);
        return;
    }
    if (!strcmp(name, "ui_scale")) {
        apply_ui_scale(value, config);
        return;
    }
    if (!strcmp(name, "middlerotate_sensitivity")) {
        apply_middle_rotate_sensitivity(value, config);
        return;
    }
    if (!strcmp(name, "avoid")) {
        bool is_form = !strcmp(value, "form");
        bool is_coord = !strcmp(value, "coord");
        if (is_form || is_coord) {
            avoid_mode = is_coord ? AVOID_COORD : AVOID_FORM;
            write_log("avoid %s", value);
        } else {
            write_log("gcmod.ini: avoid '%s' is not form or coord", value);
        }
        return;
    }

    // The switch table keeps the names with their leading "--", as on the command line of the
    // simulator.
    char switch_name[MAX_SWITCH_NAME_LENGTH];
    snprintf(switch_name, sizeof switch_name, "--%s", name);
    bool *flag = find_on_off_switch(switch_name);
    bool is_on = !strcmp(value, "on");
    bool is_off = !strcmp(value, "off");
    if (!flag || !(is_on || is_off)) {
        write_log("gcmod.ini: cannot use '%s = %s'", name, value);
        return;
    }
    *flag = is_on;
    write_log("switch %s %s", name, value);
}


// `read_config` applies the settings of gcmod.ini line by line, in file order.
void read_config(ModConfig *config) {
    char config_path[MAX_PATH];
    snprintf(config_path, sizeof config_path, "%s%s", get_game_folder_path(), CONFIG_FILE_NAME);
    FILE *config_file = fopen(config_path, "r");
    if (!config_file) {
        write_log("no %s: default settings", CONFIG_FILE_NAME);
        return;
    }

    // Each line without a comment and with an '=' is one setting.
    char line[MAX_LINE_LENGTH];
    while (fgets(line, sizeof line, config_file)) {
        char *comment = strchr(line, '#');
        if (comment)
            *comment = '\0';
        char *separator = strchr(line, '=');
        if (!separator)
            continue;
        *separator = '\0';
        char *name = trim_spaces(line);
        char *value = trim_spaces(separator + 1);
        apply_setting(name, value, config);
    }
    fclose(config_file);
}


// `is_resolution_line` tells whether `line` of gcmod.ini sets `resolution`.
static bool is_resolution_line(const char *line) {
    char setting[MAX_LINE_LENGTH];
    snprintf(setting, sizeof setting, "%s", line);
    char *comment = strchr(setting, '#');
    if (comment)
        *comment = '\0';
    char *separator = strchr(setting, '=');
    if (!separator)
        return false;
    *separator = '\0';
    return !strcmp(trim_spaces(setting), "resolution");
}


// `save_resolution_line` writes gcmod.ini again with "resolution = `value`" in place of each line
// that sets `resolution`, or at its end without such a line, and keeps every other line.
bool save_resolution_line(const char *value) {
    char config_path[MAX_PATH];
    char new_path[MAX_PATH];
    snprintf(config_path, sizeof config_path, "%s%s", get_game_folder_path(), CONFIG_FILE_NAME);
    snprintf(new_path, sizeof new_path, "%s.new", config_path);
    FILE *config_file = fopen(config_path, "r");
    FILE *new_file = config_file ? fopen(new_path, "w") : NULL;
    if (!new_file) {
        if (config_file)
            fclose(config_file);
        write_log("resolution %s not saved: cannot write %s", value, CONFIG_FILE_NAME);
        return false;
    }

    // Copy the lines, with the new value in the line of the setting.
    char line[MAX_LINE_LENGTH];
    bool is_line_found = false;
    while (fgets(line, sizeof line, config_file)) {
        if (is_resolution_line(line)) {
            fprintf(new_file, "resolution = %s\n", value);
            is_line_found = true;
        } else {
            fputs(line, new_file);
        }
    }
    if (!is_line_found)
        fprintf(new_file, "resolution = %s\n", value);
    fclose(config_file);
    bool is_written = !ferror(new_file);
    is_written = fclose(new_file) == 0 && is_written;

    // The new file replaces the old one in one step, so a failed write could not leave gcmod.ini
    // cut short.
    if (!is_written || !MoveFileExA(new_path, config_path, MOVEFILE_REPLACE_EXISTING)) {
        DeleteFileA(new_path);
        write_log("resolution %s not saved: cannot replace %s", value, CONFIG_FILE_NAME);
        return false;
    }
    write_log("gcmod.ini: resolution = %s saved", value);
    return true;
}
