// Reader of gcmod.ini, the configuration file of the mod in the game folder. Each line is
// "name = value"; text after '#' is a comment. A name is `toggle_key` (F1 to F12), `avoid` (the
// movement mode of the simulator option `--avoid`: form or coord), or a switch of the simulator
// without its leading "--" (docs/SWITCHES.md), with the value on or off.
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
