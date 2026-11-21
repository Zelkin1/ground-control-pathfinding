// Modern screen sizes in the video menu (`modern_resolution` in gcmod.ini). The video menu of
// gc.exe offers 640x480, 800x600 and 1024x768 as size indexes 0 to 2 (index 3 is 1600x1200 of the
// command line option -1600, which the menu never lists). The mod adds 1920x1080, 2560x1440 and
// 3840x2160 as indexes 4 to 6 to that list, up to the desktop size, and applies them in place of
// the game. Once per pass of its main
// loop, the game compares the menu choice with what it applied last, and after a change it
// releases the D3DX context of its renderer and creates it again at the new size (0x403f20). The
// hook on that function does the same for the new indexes, which the game code cannot read.
//
// The game saves the width that it applied in data/gcvideo.dat, and at the next start it would
// end with the fatal error "Unsupported resolution" for a width that is not one of its own, before
// the core loads. So that field keeps 1024 (the start width), the core saves a new size in the
// `resolution` line of gcmod.ini, and it chooses that size again once the intro movies end.
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <windows.h>

#include "mod.h"
#include "game_object.h"

#define THISCALL __attribute__((thiscall))

// Function that applies the choice of the video menu (no arguments, returns a byte: 1 keeps the
// game running). The main loop of WinMain (0x404210) calls it while the renderer is active. Its
// first instruction is mov eax, fs:[0] (6 bytes, decompilation/tools/gcdis.py dis 403f20).
#define APPLY_VIDEO_MENU_ADDRESS 0x403f20
#define APPLY_VIDEO_MENU_STOLEN_BYTES 6

// Function that adds one entry to the list of sizes of the video menu (thiscall: list, label,
// size index; it copies the label). Only the builder of the video page (0x47e380) calls it, once
// per size. Its first instructions are push ebx, push ebp, mov ebx, ecx and push 0xc (6 bytes).
#define ADD_SIZE_ENTRY_ADDRESS 0x463200
#define ADD_SIZE_ENTRY_STOLEN_BYTES 6

// Function that appends a widget to a menu page, which shows each widget on a line of its own
// (thiscall: page, widget, flag; the builder of the video page passes the flag 0). Its first
// instructions are push ebx, push esi, push edi and mov esi, ecx (5 bytes).
#define ADD_PAGE_WIDGET_ADDRESS 0x467cc0
#define ADD_PAGE_WIDGET_STOLEN_BYTES 5

// A size list is a row of check boxes (0x60 bytes, constructor 0x462d80, thiscall: one byte, 0
// in the video page). `+0x34` points to the size index that the row shows checked and sets on a
// click; every row bound to the same index acts as one choice. Slot 4 of its vtable (0x4620c0,
// thiscall: left, top, right, bottom) sets its area, -1 for a size taken from its content.
// The game allocates widgets with its operator new 0x5e89be (cdecl: size), and a page deletes
// them when it closes.
#define GAME_NEW_ADDRESS 0x5e89be
#define CONSTRUCT_SIZE_ROW_ADDRESS 0x462d80
#define SIZE_ROW_BYTES 0x60
#define SIZE_ROW_INDEX_POINTER 0x34
#define SET_AREA_SLOT 4
#define AREA_FROM_CONTENT -1

// Pointers to game objects, at fixed addresses in gc.exe.
#define SETTINGS_POINTER_ADDRESS 0x6a8094   // settings block of the game
#define RENDERER_POINTER_ADDRESS 0x6a8090   // renderer of the game
#define APP_POINTER_ADDRESS 0x6997cc        // application object, whose states 0x4018a0 runs
#define MOUSE_POINTER_ADDRESS 0x69b694      // mouse object

// Fields of the settings block: the choice of the video menu, then what the game applied.
#define MENU_DEVICE_INDEX 0x7c4             // device in the device table
#define MENU_SIZE_INDEX 0x7c8               // size index
#define MENU_IS_FULLSCREEN 0x7cc            // byte
#define MENU_IS_32_BIT 0x818                // byte: 32-bit color, else 16-bit
#define DEVICE_TABLE 0x98c                  // `DEVICE_ENTRY_SIZE` bytes per device, from index 0
#define APPLIED_DEVICE_INDEX 0xd00
#define APPLIED_WIDTH 0xd08                 // the width that gcvideo.dat saves
#define APPLIED_IS_WINDOWED 0xd0c           // byte
#define APPLIED_IS_32_BIT 0xd0d             // byte

// Fields of one entry of the device table (0x402d90 fills it). The widest width of the menu is
// 1024 for a hardware device, unless the capability file of the game or -1600 sets another.
#define DEVICE_ENTRY_SIZE 0x10
#define DEVICE_MAX_WIDTH 0xc

// Size indexes. The game lists the new sizes only after its 1024x768 entry, which it adds only
// for a device whose widest width is above 800. 1024x768 is also the size of the start.
#define START_SIZE_INDEX 2
#define START_WIDTH 1024
#define START_HEIGHT 768
#define FIRST_MODERN_SIZE_INDEX 4
#define NO_SIZE_INDEX -1

// Renderer. The D3DX renderer (constructor 0x431cd0) has this vtable; the software renderer
// (0x51bfe0) has another and stays at the sizes of the game. Slot 1 (0x431e30) releases the D3DX
// context; slot 2 (0x431eb0, thiscall: width, height, flag) stores the size and creates the
// context again. The game passes the flag 1 for a change after the start, which skips the
// capability lookup.
#define D3DX_RENDERER_VTABLE 0x611588
#define RENDERER_WIDTH 0x1c
#define RENDERER_HEIGHT 0x20
#define RELEASE_CONTEXT_SLOT 1
#define SET_SIZE_SLOT 2
#define IS_SIZE_CHANGE 1

// Slot 2 of the D3DX renderer, which the game calls with the flag 0 at the start (0x401140,
// before the first screen) and after a change of device. Its first instructions are mov edx,
// [esp + 8] and mov eax, [esp + 4] (8 bytes).
#define SET_RENDERER_SIZE_ADDRESS 0x431eb0
#define SET_RENDERER_SIZE_STOLEN_BYTES 8

// The application object holds the Bink movie that it plays first, or NULL without one. Its
// states 0, 1 and 2 open the movies sierra, massive and intro, each with the state that follows
// it (1, 2 or 3) as the next state; state 7 plays the open movie and then goes to the next state.
// The movie draws with the DirectDraw surfaces of the D3DX context that existed when it opened,
// so a new context while it plays would leave it with released surfaces. When the player skips a
// movie, the game destroys it with 0x418460 (thiscall, no arguments) and its operator delete
// 0x5e89e7 (cdecl: pointer).
#define APP_MOVIE 0
#define APP_STATE 4
#define APP_NEXT_STATE 8
#define DESTROY_MOVIE_ADDRESS 0x418460
#define GAME_DELETE_ADDRESS 0x5e89e7

// 0x419330 (thiscall on the mouse object: left, top, right, bottom) sets the screen area in which
// the game moves its cursor.
#define SET_MOUSE_AREA_ADDRESS 0x419330

#define SIZE_TEXT_SIZE 16                   // room for "3840x2160"

typedef bool (*ApplyVideoMenuFunction)(void);
typedef void(THISCALL *DestroyMovieFunction)(void *movie);
typedef void (*GameDeleteFunction)(void *object);
typedef void(THISCALL *AddSizeEntryFunction)(void *size_list, const char *label, int size_index);
typedef void(THISCALL *AddPageWidgetFunction)(void *page, void *widget, int flag);
typedef void *(*GameNewFunction)(size_t byte_count);
typedef void *(THISCALL *ConstructSizeRowFunction)(void *size_row, int flag);
typedef void(THISCALL *SetAreaFunction)(void *widget, int left, int top, int right, int bottom);
typedef void(THISCALL *ReleaseContextFunction)(void *renderer);
typedef HRESULT(THISCALL *SetSizeFunction)(void *renderer, int width, int height, int flag);
typedef void(THISCALL *SetMouseAreaFunction)(void *mouse, int left, int top, int right,
                                             int bottom);

typedef struct {
    int width;
    int height;
    const char *label;                      // text of the entry, in the form of the game's own
} ModernSize;

// New sizes of the video menu; size index `FIRST_MODERN_SIZE_INDEX` + array index.
static const ModernSize MODERN_SIZES[] = {
    {
        .width = 1920,
        .height = 1080,
        .label = "1920x1080",
    },
    {
        .width = 2560,
        .height = 1440,
        .label = "2560x1440",
    },
    {
        .width = 3840,
        .height = 2160,
        .label = "3840x2160",
    },
};
#define MODERN_SIZE_COUNT (int)(sizeof MODERN_SIZES / sizeof MODERN_SIZES[0])

static ApplyVideoMenuFunction original_apply_video_menu;   // trampoline of 0x403f20
static SetSizeFunction original_set_renderer_size;         // trampoline of 0x431eb0
static AddSizeEntryFunction original_add_size_entry;       // trampoline of 0x463200
static AddPageWidgetFunction original_add_page_widget;     // trampoline of 0x467cc0
static void *game_size_row;           // size list of the game in the video page being built
static void *modern_size_row;         // row of the new sizes that goes on the line below it
static bool is_size_offered[MODERN_SIZE_COUNT];   // the display has a mode of that size
static int saved_size_index = NO_SIZE_INDEX;      // size of the `resolution` line of gcmod.ini
static bool is_restore_pending;       // the saved size still has to go into the menu choice
static int resized_size_index = NO_SIZE_INDEX;    // size index of the last change of the hook
static int resized_width;             // size of the renderer after that change
static int resized_height;
static int desktop_width;             // size of the Windows desktop
static int desktop_height;


// `get_modern_size` returns the new size of the size index `size_index`.
static const ModernSize *get_modern_size(int size_index) {
    return &MODERN_SIZES[size_index - FIRST_MODERN_SIZE_INDEX];
}


// `get_start_width` returns the width that the hook leaves in the applied field during a new
// size: 1024, but at most the widest width of the chosen device, as the game would apply it.
static int get_start_width(const void *settings) {
    int device_index = read_int(settings, MENU_DEVICE_INDEX);
    int device_offset = DEVICE_TABLE + device_index * DEVICE_ENTRY_SIZE;
    int max_width = read_int(settings, device_offset + DEVICE_MAX_WIDTH);
    return START_WIDTH < max_width ? START_WIDTH : max_width;
}


// `is_device_change` tells whether the menu chooses another device, or a change between window
// and full screen. The game then makes a new renderer, which the hook leaves to the game.
static bool is_device_change(const void *settings) {
    bool is_same_device =
        read_int(settings, MENU_DEVICE_INDEX) == read_int(settings, APPLIED_DEVICE_INDEX);
    bool is_fullscreen = read_byte(settings, MENU_IS_FULLSCREEN) != 0;
    bool is_windowed = read_byte(settings, APPLIED_IS_WINDOWED) != 0;
    return !is_same_device || is_fullscreen == is_windowed;
}


// `is_movie_playing` tells whether the application has one of its Bink movies of the start open.
static bool is_movie_playing(void) {
    const void *app = *(void **)APP_POINTER_ADDRESS;
    return app && read_pointer(app, APP_MOVIE);
}


// `save_size` writes the size of `size_index`, or "off" for `NO_SIZE_INDEX`, into the
// `resolution` line of gcmod.ini, unless the line already holds it.
static void save_size(int size_index) {
    if (size_index == saved_size_index)
        return;
    char size_text[SIZE_TEXT_SIZE] = "off";
    if (size_index != NO_SIZE_INDEX) {
        const ModernSize *size = get_modern_size(size_index);
        snprintf(size_text, sizeof size_text, "%dx%d", size->width, size->height);
    }
    if (save_resolution_line(size_text))
        saved_size_index = size_index;
}


// `run_game_change` runs the code of the game that applies the video menu. That code would read
// no width for a new size index, so it gets the start index in its place for that call.
static bool run_game_change(void *settings) {
    int size_index = read_int(settings, MENU_SIZE_INDEX);
    bool is_modern_size = size_index >= FIRST_MODERN_SIZE_INDEX;
    if (is_modern_size)
        write_int(settings, MENU_SIZE_INDEX, START_SIZE_INDEX);
    bool is_running = original_apply_video_menu();
    if (is_modern_size)
        write_int(settings, MENU_SIZE_INDEX, size_index);
    return is_running;
}


// `resize_renderer` creates the D3DX context of `renderer` again at the size of `size_index`, as
// the game does after a change in its video menu, and fits the cursor area to the new size. It
// returns false when Direct3D refused that size; the renderer then has the start size.
static bool resize_renderer(void *renderer, int size_index) {
    const ModernSize *size = get_modern_size(size_index);
    void *const *vtable = read_pointer(renderer, 0);
    ReleaseContextFunction release_context = (ReleaseContextFunction)vtable[RELEASE_CONTEXT_SLOT];
    SetSizeFunction set_size = (SetSizeFunction)vtable[SET_SIZE_SLOT];
    release_context(renderer);
    bool is_resized = SUCCEEDED(set_size(renderer, size->width, size->height, IS_SIZE_CHANGE));

    // The game shows the error text of a failed context and goes on without a Direct3D device,
    // which would crash its next frame, so the start size replaces the refused one.
    if (!is_resized) {
        write_log("resolution: Direct3D refused %s; back to %dx%d", size->label, START_WIDTH,
                  START_HEIGHT);
        release_context(renderer);
        set_size(renderer, START_WIDTH, START_HEIGHT, IS_SIZE_CHANGE);
    }

    // The screen size is read back, since a smaller result than the one asked for would
    // otherwise make the hook ask again at every pass. The cursor area takes the renderer size,
    // which is smaller than the screen under `ui_scale` (ui_scale.c).
    resized_size_index = size_index;
    read_screen_size(renderer, &resized_width, &resized_height);
    SetMouseAreaFunction set_mouse_area = (SetMouseAreaFunction)SET_MOUSE_AREA_ADDRESS;
    int layout_width = read_int(renderer, RENDERER_WIDTH);
    int layout_height = read_int(renderer, RENDERER_HEIGHT);
    set_mouse_area(*(void **)MOUSE_POINTER_ADDRESS, 0, 0, layout_width, layout_height);
    write_log("resolution: screen now %dx%d", resized_width, resized_height);
    return is_resized;
}


// `is_size_applied` tells whether `renderer` already has the size of `size_index`, or the result
// of the last change of the hook to that size.
static bool is_size_applied(const void *renderer, int size_index) {
    const ModernSize *size = get_modern_size(size_index);
    int width;
    int height;
    read_screen_size(renderer, &width, &height);
    bool is_target = width == size->width && height == size->height;
    bool is_result_of_index = size_index == resized_size_index;
    bool is_result = is_result_of_index && width == resized_width && height == resized_height;
    return is_target || is_result;
}


// `is_modern_size_current` tells whether `renderer` has the size of `size_index` and the color
// depth of the menu.
static bool is_modern_size_current(const void *settings, const void *renderer, int size_index) {
    uint8_t is_menu_32_bit = read_byte(settings, MENU_IS_32_BIT);
    bool is_same_color = is_menu_32_bit == read_byte(settings, APPLIED_IS_32_BIT);
    return is_same_color && is_size_applied(renderer, size_index);
}


// `apply_modern_size` makes `renderer` use the new size of `size_index` and the color depth of
// the menu, and saves the size in gcmod.ini.
static void apply_modern_size(void *settings, void *renderer, int size_index) {
    // The game would find no change in the applied width after the hook goes away, and
    // gcvideo.dat keeps a width that the game can start with.
    write_int(settings, APPLIED_WIDTH, get_start_width(settings));
    if (is_modern_size_current(settings, renderer, size_index))
        return;
    uint8_t is_menu_32_bit = read_byte(settings, MENU_IS_32_BIT);

    // The new context takes its color depth from the applied field. After a refused size, the
    // menu shows the start size, and the size leaves the menu until the next core load.
    write_byte(settings, APPLIED_IS_32_BIT, is_menu_32_bit);
    if (resize_renderer(renderer, size_index)) {
        save_size(size_index);
        return;
    }
    is_size_offered[size_index - FIRST_MODERN_SIZE_INDEX] = false;
    write_int(settings, MENU_SIZE_INDEX, START_SIZE_INDEX);
    write_int(settings, APPLIED_WIDTH, read_int(renderer, RENDERER_WIDTH));
    save_size(NO_SIZE_INDEX);
}


// `leave_modern_size` prepares the change from a new size back to a size of the game. The game
// keeps the renderer width in the applied field, so a renderer of another width has a new size,
// and a cleared field makes the game apply its own size again.
static void leave_modern_size(void *settings, const void *renderer) {
    int width;
    int height;
    read_screen_size(renderer, &width, &height);
    if (width != read_int(settings, APPLIED_WIDTH))
        write_int(settings, APPLIED_WIDTH, 0);
    save_size(NO_SIZE_INDEX);
}


// `hook_set_renderer_size` gives the D3DX renderer the saved new size when the game itself sets
// the size (flag 0), so that the first screen of the game already has it and the display mode
// never changes after it. The changes of the hook pass the flag 1.
static HRESULT THISCALL hook_set_renderer_size(void *renderer, int width, int height, int flag) {
    bool is_d3dx_renderer = read_pointer(renderer, 0) == (void *)D3DX_RENDERER_VTABLE;
    if (!is_d3dx_renderer || flag == IS_SIZE_CHANGE)
        return original_set_renderer_size(renderer, width, height, flag);
    if (saved_size_index == NO_SIZE_INDEX)
        return original_set_renderer_size(renderer, width, height, flag);
    const ModernSize *size = get_modern_size(saved_size_index);
    return original_set_renderer_size(renderer, size->width, size->height, flag);
}


// `restart_movie` closes the intro movie that plays, as the game does when the player skips it,
// and sets the application to open the same movie again at its next pass.
static void restart_movie(void) {
    void *app = *(void **)APP_POINTER_ADDRESS;
    void *movie = read_pointer(app, APP_MOVIE);
    DestroyMovieFunction destroy_movie = (DestroyMovieFunction)DESTROY_MOVIE_ADDRESS;
    GameDeleteFunction game_delete = (GameDeleteFunction)GAME_DELETE_ADDRESS;
    destroy_movie(movie);
    game_delete(movie);
    write_pointer(app, APP_MOVIE, NULL);
    write_int(app, APP_STATE, read_int(app, APP_NEXT_STATE) - 1);
    write_log("resolution: intro movie restarted at the new size");
}


// `hook_apply_video_menu` applies the new sizes of the video menu, and leaves the sizes of the
// game and device changes to the game. A change while an intro movie plays restarts that movie,
// so that the intro plays at the new size.
static bool hook_apply_video_menu(void) {
    void *settings = *(void **)SETTINGS_POINTER_ADDRESS;
    void *renderer = *(void **)RENDERER_POINTER_ADDRESS;

    // At the start, the game chose its size from gcvideo.dat; the saved new size replaces it.
    if (is_restore_pending) {
        write_int(settings, MENU_SIZE_INDEX, saved_size_index);
        is_restore_pending = false;
    }

    // A new size needs the D3DX renderer of the applied device.
    int size_index = read_int(settings, MENU_SIZE_INDEX);
    bool is_d3dx_renderer = read_pointer(renderer, 0) == (void *)D3DX_RENDERER_VTABLE;
    bool can_resize = is_d3dx_renderer && !is_device_change(settings);
    bool is_modern_size = size_index >= FIRST_MODERN_SIZE_INDEX;
    bool is_modern_change = is_modern_size && can_resize;

    // With a loader older than 0.4.0, the core loads only at the first frames, so the first movie
    // has already started at the size of the game; the hook leaves movies alone without a change.
    if (is_movie_playing()) {
        bool is_modern_pending =
            is_modern_change && !is_modern_size_current(settings, renderer, size_index);
        if (!is_modern_pending)
            return run_game_change(settings);
        restart_movie();
    }
    if (is_modern_change) {
        apply_modern_size(settings, renderer, size_index);
        return true;
    }
    if (!is_modern_size && can_resize)
        leave_modern_size(settings, renderer);
    return run_game_change(settings);
}


// `make_modern_size_row` makes a row of check boxes with the new sizes that the display offers,
// bound to the same size index as `game_size_list`, as the builder of the video page makes its
// own row. It returns NULL when no new size is offered.
static void *make_modern_size_row(const void *game_size_list) {
    bool is_any_offered = false;
    for (int modern_index = 0; modern_index < MODERN_SIZE_COUNT; modern_index++)
        is_any_offered = is_any_offered || is_size_offered[modern_index];
    if (!is_any_offered)
        return NULL;

    // Build the row in the memory of the game, which the page deletes with its own widgets.
    GameNewFunction game_new = (GameNewFunction)GAME_NEW_ADDRESS;
    ConstructSizeRowFunction construct_size_row =
        (ConstructSizeRowFunction)CONSTRUCT_SIZE_ROW_ADDRESS;
    void *size_row = construct_size_row(game_new(SIZE_ROW_BYTES), 0);
    void *size_index_pointer = read_pointer(game_size_list, SIZE_ROW_INDEX_POINTER);
    write_pointer(size_row, SIZE_ROW_INDEX_POINTER, size_index_pointer);
    void *const *vtable = read_pointer(size_row, 0);
    SetAreaFunction set_area = (SetAreaFunction)vtable[SET_AREA_SLOT];
    set_area(size_row, 0, 0, AREA_FROM_CONTENT, AREA_FROM_CONTENT);

    for (int modern_index = 0; modern_index < MODERN_SIZE_COUNT; modern_index++) {
        if (!is_size_offered[modern_index])
            continue;
        const char *modern_label = MODERN_SIZES[modern_index].label;
        original_add_size_entry(size_row, modern_label, FIRST_MODERN_SIZE_INDEX + modern_index);
    }
    return size_row;
}


// `hook_add_size_entry` prepares the row of the new sizes once the game has added its last size,
// 1024x768, to its own row; at most three sizes per row keep the rows inside the menu box.
static void THISCALL hook_add_size_entry(void *size_list, const char *label, int size_index) {
    original_add_size_entry(size_list, label, size_index);
    if (size_index != START_SIZE_INDEX)
        return;
    game_size_row = size_list;
    modern_size_row = make_modern_size_row(size_list);
}


// `hook_add_page_widget` puts the row of the new sizes on the line right below the size row of
// the game, so that the widgets after it move down one line.
static void THISCALL hook_add_page_widget(void *page, void *widget, int flag) {
    original_add_page_widget(page, widget, flag);
    if (widget != game_size_row)
        return;
    game_size_row = NULL;
    if (!modern_size_row)
        return;
    original_add_page_widget(page, modern_size_row, flag);
    modern_size_row = NULL;
}


// `is_display_mode` tells whether the display offers a mode of `width` by `height` pixels, at any
// color depth. Windows lists 32-bit modes only, and emulates 16-bit ones for DirectDraw.
static bool is_display_mode(int width, int height) {
    DEVMODEA mode = {
        .dmSize = sizeof mode,
    };
    for (DWORD mode_index = 0; EnumDisplaySettingsA(NULL, mode_index, &mode); mode_index++) {
        bool is_width = (int)mode.dmPelsWidth == width;
        if (is_width && (int)mode.dmPelsHeight == height)
            return true;
    }
    return false;
}


// `find_desktop_size` sets the size of the Windows desktop, and returns false when Windows gives
// none. It comes from the registry, since the current mode is the one of the game.
static bool find_desktop_size(void) {
    DEVMODEA desktop_mode = {
        .dmSize = sizeof desktop_mode,
    };
    if (!EnumDisplaySettingsA(NULL, ENUM_REGISTRY_SETTINGS, &desktop_mode)) {
        write_log("resolution: no desktop size found; the game keeps its own sizes");
        return false;
    }
    desktop_width = (int)desktop_mode.dmPelsWidth;
    desktop_height = (int)desktop_mode.dmPelsHeight;
    return true;
}


// `find_offered_sizes` records which new sizes the display offers up to the size of the desktop,
// logs them, and returns whether there is one. The display can also list modes above the size of
// its panel (scaled down by the graphics card), and the game failed to start Direct3D in such a
// mode (2560x1440 on a display whose widest mode is 2880x1620).
static bool find_offered_sizes(void) {
    bool is_any_offered = false;
    for (int modern_index = 0; modern_index < MODERN_SIZE_COUNT; modern_index++) {
        const ModernSize *size = &MODERN_SIZES[modern_index];
        bool is_display_size = is_display_mode(size->width, size->height);
        bool is_width_in_desktop = size->width <= desktop_width;
        bool is_in_desktop = is_width_in_desktop && size->height <= desktop_height;
        is_size_offered[modern_index] = is_display_size && is_in_desktop;
        is_any_offered = is_any_offered || is_size_offered[modern_index];
        const char *offer_text = "in the video menu";
        if (!is_in_desktop)
            offer_text = "larger than the desktop";
        else if (!is_display_size)
            offer_text = "not offered by the display";
        write_log("modern resolution %s: %s (desktop %dx%d)", size->label, offer_text,
                  desktop_width, desktop_height);
    }
    return is_any_offered;
}


// `find_saved_size` sets the size index of the `resolution` line of `config`, and plans its
// restore when it is a new size that the display offers.
static void find_saved_size(const ModConfig *config) {
    saved_size_index = NO_SIZE_INDEX;
    is_restore_pending = false;
    if (!config->saved_resolution_width)
        return;
    for (int modern_index = 0; modern_index < MODERN_SIZE_COUNT; modern_index++) {
        const ModernSize *size = &MODERN_SIZES[modern_index];
        bool is_width = size->width == config->saved_resolution_width;
        if (!is_width || size->height != config->saved_resolution_height)
            continue;
        if (!is_size_offered[modern_index])
            return;
        saved_size_index = FIRST_MODERN_SIZE_INDEX + modern_index;
        is_restore_pending = true;
        return;
    }
    write_log("gcmod.ini: resolution %dx%d is not a size of the video menu",
              config->saved_resolution_width, config->saved_resolution_height);
}


// `is_modern_screen_size` tells whether `width` by `height` is a new size of the video menu.
bool is_modern_screen_size(int width, int height) {
    for (int modern_index = 0; modern_index < MODERN_SIZE_COUNT; modern_index++) {
        const ModernSize *size = &MODERN_SIZES[modern_index];
        if (size->width == width && size->height == height)
            return true;
    }
    return false;
}


// `install_resolution_hook` hooks, with `install_hook`, the function that applies the video
// menu, the size function of the D3DX renderer, the function that appends widgets to menu pages
// and the function that fills the size list, when `modern_resolution` is on.
void install_resolution_hook(InstallHookFunction install_hook, const ModConfig *config) {
    if (!config->is_modern_resolution_on || !find_desktop_size() || !find_offered_sizes())
        return;
    find_saved_size(config);
    original_apply_video_menu = install_hook(APPLY_VIDEO_MENU_ADDRESS,
                                             APPLY_VIDEO_MENU_STOLEN_BYTES,
                                             (const void *)hook_apply_video_menu);
    if (!original_apply_video_menu) {
        write_log("hook of the video menu failed: the game keeps its own sizes");
        return;
    }

    // Without this hook, the game would still start at its own size and change once later.
    original_set_renderer_size = install_hook(SET_RENDERER_SIZE_ADDRESS,
                                              SET_RENDERER_SIZE_STOLEN_BYTES,
                                              (const void *)hook_set_renderer_size);
    if (!original_set_renderer_size)
        write_log("hook of the renderer size failed: the game starts at its own size");

    // The menu would hand the game a size index that only the hook above can apply, so it gets
    // the new row only after that hook is in place, and only when the row can go on the page.
    original_add_page_widget = install_hook(ADD_PAGE_WIDGET_ADDRESS, ADD_PAGE_WIDGET_STOLEN_BYTES,
                                            (const void *)hook_add_page_widget);
    if (!original_add_page_widget) {
        write_log("hook of the menu pages failed: the video menu shows only the sizes of the game");
        return;
    }
    original_add_size_entry = install_hook(ADD_SIZE_ENTRY_ADDRESS, ADD_SIZE_ENTRY_STOLEN_BYTES,
                                           (const void *)hook_add_size_entry);
    if (!original_add_size_entry)
        write_log("hook of the size list failed: the video menu shows only the sizes of the game");
}


// `stop_resolution` gives the game back a size index that its own code can read, in case the
// next core has no hook. The screen keeps its size until the next change in the video menu.
void stop_resolution(void) {
    if (!original_apply_video_menu)
        return;
    void *settings = *(void **)SETTINGS_POINTER_ADDRESS;
    if (read_int(settings, MENU_SIZE_INDEX) >= FIRST_MODERN_SIZE_INDEX)
        write_int(settings, MENU_SIZE_INDEX, START_SIZE_INDEX);
}
