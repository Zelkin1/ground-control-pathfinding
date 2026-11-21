// Scale of the 2D interface at the modern screen sizes (`ui_scale` in gcmod.ini). The menus of
// gc.exe are one 640x480 layout in the middle of the screen, and the in-game interface sits at the
// screen edges with sizes in pixels, so both look small at 1920x1080. The core therefore gives the
// game code a smaller screen, the game screen: the screen size divided by the scale. This acts
// only at the sizes that the mod gives the screen (resolution.c). The game lays out its menus and
// interface, moves its cursor and picks units in the game screen. Only what reaches Direct3D and
// DirectDraw goes back to screen pixels: the vertices that the game gives in screen positions,
// the viewports and rectangles to clear, the copied images and the Bink movie frames. The game
// transforms its 3D world into screen positions itself, so the world too is drawn with float
// positions on every pixel of the screen.
//
//   game code, game screen           Direct3D, screen
//   +--------------+                 +---------------------+
//   |  1280 x 720  |   x 1.5  --->   |     1920 x 1080     |
//   +--------------+                 +---------------------+
//
// The renderer holds the game screen size in the fields that the game reads, except while it
// creates its D3DX context, which needs the screen size.
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <d3d.h>

#include "mod.h"
#include "game_object.h"

#define THISCALL __attribute__((thiscall))

// Slot 3 of the D3DX renderer (thiscall: renderer, two arguments that it passes on, `ret 8`)
// creates the D3DX context at the renderer size and returns an HRESULT. Slot 2 calls it after a
// size change, and 0x432b80 after a lost device. Its first instruction is mov eax, fs:[0]
// (6 bytes, decompilation/tools/gcdis.py dis 431ed0).
#define CREATE_CONTEXT_ADDRESS 0x431ed0
#define CREATE_CONTEXT_STOLEN_BYTES 6

// Slot 36 of the D3DX renderer (thiscall: renderer, image item, `ret 4`) copies an image with
// IDirectDrawSurface7::BltFast to the render surface, at the size of its source. The 2D pass
// (slot 35, 0x434630) calls it, in place of a textured quad, for textures whose byte +0x20 is set.
// Its first instructions are sub esp, 0x10, push esi and push edi (5 bytes).
#define BLIT_IMAGE_ADDRESS 0x434870
#define BLIT_IMAGE_STOLEN_BYTES 5

// 0x4189c0 (thiscall: movie, returns a byte) shows the next frame of a Bink movie: it copies the
// frame into the surface of the movie, then copies that surface with IDirectDrawSurface7::Blt
// to the target surface of the movie, in a rectangle centered in the renderer size (or over the
// whole target when a game option asks for that). Its first instructions are sub esp, 0x20,
// push esi and mov esi, ecx (6 bytes).
#define SHOW_MOVIE_FRAME_ADDRESS 0x4189c0
#define SHOW_MOVIE_FRAME_STOLEN_BYTES 6
#define MOVIE_TARGET_SURFACE 4              // the render surface of the context at the movie open

// Pointers to game objects, at fixed addresses in gc.exe.
#define RENDERER_POINTER_ADDRESS 0x6a8090   // renderer of the game
#define SETTINGS_POINTER_ADDRESS 0x6a8094   // settings block of the game
#define MOUSE_POINTER_ADDRESS 0x69b694      // mouse object

// Fields of the D3DX renderer (vtable 0x611588, docs/PATHFINDING.md "Screen resolution").
#define D3DX_RENDERER_VTABLE 0x611588
#define RENDERER_WIDTH 0x1c                 // width that the getter 0x4294b0 returns
#define RENDERER_HEIGHT 0x20                // height that the getter 0x430140 returns
#define RENDERER_DEVICE 0x40                // IDirect3DDevice7
#define RENDERER_CONTEXT 0x48               // D3DX context

// Top left corner of the 640x480 menu layout on the screen: two floats of the settings block that
// the menu pages add to each position of dialogs/*.txt. Slot 3 sets them to the middle of the
// renderer size (0x432351).
#define MENU_ORIGIN_X 0x82c
#define MENU_ORIGIN_Y 0x830
#define MENU_WIDTH 640
#define MENU_HEIGHT 480

// 0x419330 (thiscall on the mouse object: left, top, right, bottom) sets the area in which the
// game moves its cursor.
#define SET_MOUSE_AREA_ADDRESS 0x419330

// Image item of the blit (0x20 bytes): texture, then the screen position and size, then the
// position of the image in its source surface. The texture holds at +0x3c an object whose +0x3c
// is that surface.
#define IMAGE_TEXTURE 0
#define IMAGE_X 4
#define IMAGE_Y 8
#define IMAGE_WIDTH 0xc
#define IMAGE_HEIGHT 0x10
#define IMAGE_SOURCE_X 0x14
#define IMAGE_SOURCE_Y 0x18
#define TEXTURE_SURFACE_OWNER 0x3c
#define OWNER_SURFACE 0x3c

// The blit asks the D3DX context for its target surface through vtable offset 0x20, with the
// argument 0 (0x43487f).
#define CONTEXT_TARGET_SURFACE_SLOT 8
#define TARGET_SURFACE_INDEX 0

// A vertex with a screen position (`D3DFVF_XYZRHW`) starts with the floats x and y.
#define VERTEX_X 0
#define VERTEX_Y 4

#define MAX_CLEAR_RECTS 16                  // the game clears one rectangle at a time

typedef HRESULT(THISCALL *CreateContextFunction)(void *renderer, int first_argument,
                                                 int second_argument);
typedef void(THISCALL *BlitImageFunction)(void *renderer, const void *image);
typedef bool(THISCALL *ShowMovieFrameFunction)(void *movie);
typedef HRESULT(WINAPI *SurfaceBltFunction)(IDirectDrawSurface7 *surface, RECT *target_area,
                                            IDirectDrawSurface7 *source, RECT *source_area,
                                            DWORD flags, DDBLTFX *effects);
typedef void(THISCALL *SetMouseAreaFunction)(void *mouse, int left, int top, int right,
                                             int bottom);
typedef IDirectDrawSurface7 *(WINAPI *GetTargetSurfaceFunction)(void *context, DWORD index);
typedef HRESULT(WINAPI *DrawPrimitiveFunction)(IDirect3DDevice7 *device,
                                               D3DPRIMITIVETYPE primitive_type, DWORD fvf,
                                               void *vertices, DWORD vertex_count, DWORD flags);
typedef HRESULT(WINAPI *DrawIndexedPrimitiveFunction)(IDirect3DDevice7 *device,
                                                      D3DPRIMITIVETYPE primitive_type, DWORD fvf,
                                                      void *vertices, DWORD vertex_count,
                                                      WORD *indices, DWORD index_count,
                                                      DWORD flags);
typedef HRESULT(WINAPI *SetViewportFunction)(IDirect3DDevice7 *device, D3DVIEWPORT7 *viewport);
typedef HRESULT(WINAPI *ClearFunction)(IDirect3DDevice7 *device, DWORD rect_count, D3DRECT *rects,
                                       DWORD flags, D3DCOLOR color, D3DVALUE z, DWORD stencil);

static float configured_scale;              // `ui_scale` of gcmod.ini
static CreateContextFunction original_create_context;   // trampoline of 0x431ed0
static BlitImageFunction original_blit_image;           // trampoline of 0x434870
static ShowMovieFrameFunction original_show_movie_frame;   // trampoline of 0x4189c0
static SurfaceBltFunction original_movie_blt;   // Blt of the movie target, during a movie frame
static bool is_scaled;                      // the renderer holds the game screen size
static bool is_load_check_pending;          // the first frame after the core load is still due
static int screen_width;                    // size of the D3DX context
static int screen_height;
static int game_width;                      // size that the game code uses
static int game_height;
static float x_factor;                      // screen pixels per game screen pixel
static float y_factor;
static uint8_t *scaled_vertices;            // copy of the vertices of the last scaled draw
static size_t scaled_vertices_size;

// Device methods that the core replaces in the vtable of the Direct3D device, and their originals.
static IDirect3DDevice7Vtbl *patched_vtable;
static DrawPrimitiveFunction original_draw_primitive;
static DrawIndexedPrimitiveFunction original_draw_indexed_primitive;
static SetViewportFunction original_set_viewport;
static ClearFunction original_clear;
static void keep_device_hooks(void);


// `scale_x` returns the screen column of the game screen column `x`.
static LONG scale_x(LONG x) {
    return MulDiv(x, screen_width, game_width);
}


// `scale_y` returns the screen row of the game screen row `y`.
static LONG scale_y(LONG y) {
    return MulDiv(y, screen_height, game_height);
}


// `is_in_game_screen` tells whether the rectangle from (`left`, `top`) to (`right`, `bottom`)
// lies inside the game screen. The D3DX context sets its own viewport and clears in screen pixels;
// those rectangles are larger, and they stay as they are.
static bool is_in_game_screen(LONG left, LONG top, LONG right, LONG bottom) {
    bool is_x_inside = left >= 0 && right <= game_width;
    return is_x_inside && top >= 0 && bottom <= game_height;
}


// `get_vertex_size` returns the size in bytes of one vertex of the flexible vertex format `fvf`.
static int get_vertex_size(DWORD fvf) {
    // The position is 3 floats, then the reciprocal of w (RHW) or 1 to 5 blend weights; the table
    // goes by the position bits shifted right by 1.
    static const int POSITION_SIZES[] = {0, 12, 16, 16, 20, 24, 28, 32};
    int size = POSITION_SIZES[(fvf & D3DFVF_POSITION_MASK) >> 1];
    if (fvf & D3DFVF_NORMAL)
        size += 3 * sizeof(float);
    if (fvf & D3DFVF_RESERVED1)
        size += sizeof(DWORD);
    if (fvf & D3DFVF_DIFFUSE)
        size += sizeof(D3DCOLOR);
    if (fvf & D3DFVF_SPECULAR)
        size += sizeof(D3DCOLOR);

    // Each texture stage has 2 bits for the size of its coordinates, from bit 16 on, in the order
    // of `D3DFVF_TEXTUREFORMAT2`, 3, 4 and 1.
    static const int COORDINATE_SIZES[] = {8, 12, 16, 4};
    int texture_count = (fvf & D3DFVF_TEXCOUNT_MASK) >> D3DFVF_TEXCOUNT_SHIFT;
    for (int stage = 0; stage < texture_count; stage++) {
        int format = (fvf >> (16 + 2 * stage)) & 3;
        size += COORDINATE_SIZES[format];
    }
    return size;
}


// `scale_screen_vertices` returns a copy of the `vertex_count` vertices at `vertices` with their
// screen positions in screen pixels, or NULL when the draw needs no change: no game screen, or a
// format `fvf` without screen positions, which Direct3D places through the viewport.
static void *scale_screen_vertices(DWORD fvf, const void *vertices, DWORD vertex_count) {
    bool is_screen_position = (fvf & D3DFVF_POSITION_MASK) == D3DFVF_XYZRHW;
    if (!is_scaled || !is_screen_position || !vertices)
        return NULL;
    int vertex_size = get_vertex_size(fvf);
    size_t vertices_size = (size_t)vertex_size * vertex_count;
    if (vertices_size > scaled_vertices_size) {
        uint8_t *larger_copy = realloc(scaled_vertices, vertices_size);
        if (!larger_copy)
            return NULL;
        scaled_vertices = larger_copy;
        scaled_vertices_size = vertices_size;
    }

    // The copy keeps the vertices one after another, `vertex_size` bytes each.
    memcpy(scaled_vertices, vertices, vertices_size);
    for (DWORD vertex_index = 0; vertex_index < vertex_count; vertex_index++) {
        uint8_t *vertex = &scaled_vertices[vertex_index * vertex_size];
        write_float(vertex, VERTEX_X, read_float(vertex, VERTEX_X) * x_factor);
        write_float(vertex, VERTEX_Y, read_float(vertex, VERTEX_Y) * y_factor);
    }
    return scaled_vertices;
}


// `hook_draw_primitive` draws with the screen positions of the vertices in screen pixels.
static HRESULT WINAPI hook_draw_primitive(IDirect3DDevice7 *device,
                                          D3DPRIMITIVETYPE primitive_type, DWORD fvf,
                                          void *vertices, DWORD vertex_count, DWORD flags) {
    void *screen_vertices = scale_screen_vertices(fvf, vertices, vertex_count);
    if (screen_vertices)
        vertices = screen_vertices;
    HRESULT result =
        original_draw_primitive(device, primitive_type, fvf, vertices, vertex_count, flags);
    keep_device_hooks();
    return result;
}


// `hook_draw_indexed_primitive` draws with the screen positions of the vertices in screen pixels.
static HRESULT WINAPI hook_draw_indexed_primitive(IDirect3DDevice7 *device,
                                                  D3DPRIMITIVETYPE primitive_type, DWORD fvf,
                                                  void *vertices, DWORD vertex_count,
                                                  WORD *indices, DWORD index_count, DWORD flags) {
    void *screen_vertices = scale_screen_vertices(fvf, vertices, vertex_count);
    if (screen_vertices)
        vertices = screen_vertices;
    HRESULT result = original_draw_indexed_primitive(device, primitive_type, fvf, vertices,
                                                     vertex_count, indices, index_count, flags);
    keep_device_hooks();
    return result;
}


// `hook_set_viewport` sets a viewport of the game screen in screen pixels.
static HRESULT WINAPI hook_set_viewport(IDirect3DDevice7 *device, D3DVIEWPORT7 *viewport) {
    D3DVIEWPORT7 screen_viewport;
    D3DVIEWPORT7 *used_viewport = viewport;
    if (is_scaled && viewport) {
        LONG left = (LONG)viewport->dwX;
        LONG top = (LONG)viewport->dwY;
        LONG right = left + (LONG)viewport->dwWidth;
        LONG bottom = top + (LONG)viewport->dwHeight;

        // Both edges go to screen pixels, so that views side by side still meet.
        if (is_in_game_screen(left, top, right, bottom)) {
            screen_viewport = *viewport;
            screen_viewport.dwX = (DWORD)scale_x(left);
            screen_viewport.dwY = (DWORD)scale_y(top);
            screen_viewport.dwWidth = (DWORD)(scale_x(right) - scale_x(left));
            screen_viewport.dwHeight = (DWORD)(scale_y(bottom) - scale_y(top));
            used_viewport = &screen_viewport;
        }
    }
    HRESULT result = original_set_viewport(device, used_viewport);
    keep_device_hooks();
    return result;
}


// `hook_clear` clears the rectangles of the game screen in screen pixels.
static HRESULT WINAPI hook_clear(IDirect3DDevice7 *device, DWORD rect_count, D3DRECT *rects,
                                 DWORD flags, D3DCOLOR color, D3DVALUE z, DWORD stencil) {
    // The game clears at the start of each frame, so a method that Direct3D replaced outside the
    // hooks gets its hook back here.
    keep_device_hooks();
    bool has_rects = rects && rect_count > 0;
    if (!is_scaled || !has_rects || rect_count > MAX_CLEAR_RECTS)
        return original_clear(device, rect_count, rects, flags, color, z, stencil);
    D3DRECT screen_rects[MAX_CLEAR_RECTS];
    for (DWORD rect_index = 0; rect_index < rect_count; rect_index++) {
        const D3DRECT *rect = &rects[rect_index];
        D3DRECT *screen_rect = &screen_rects[rect_index];
        *screen_rect = *rect;
        if (!is_in_game_screen(rect->x1, rect->y1, rect->x2, rect->y2))
            continue;
        screen_rect->x1 = scale_x(rect->x1);
        screen_rect->y1 = scale_y(rect->y1);
        screen_rect->x2 = scale_x(rect->x2);
        screen_rect->y2 = scale_y(rect->y2);
    }
    return original_clear(device, rect_count, screen_rects, flags, color, z, stencil);
}


// `write_vtable` replaces the methods of `vtable` with those of `methods` (a whole vtable), and
// returns false when the memory of the vtable cannot be opened for writing.
static bool write_vtable(IDirect3DDevice7Vtbl *vtable, const IDirect3DDevice7Vtbl *methods) {
    DWORD old_protection;
    if (!VirtualProtect(vtable, sizeof *vtable, PAGE_READWRITE, &old_protection))
        return false;
    *vtable = *methods;
    VirtualProtect(vtable, sizeof *vtable, old_protection, &old_protection);
    return true;
}


// `keep_device_hooks` puts the hooks back where Direct3D has replaced a hooked method of the
// patched vtable by another one of its own, which the hook then calls as the original. In game,
// the runtime replaced `DrawPrimitive` right after the first draw, which ended the scaling; most
// likely it binds the code of that method at its first call (TODO: confirm).
static void keep_device_hooks(void) {
    if (!patched_vtable)
        return;
    IDirect3DDevice7Vtbl methods = *patched_vtable;
    bool is_draw_replaced = methods.DrawPrimitive != hook_draw_primitive;
    bool is_indexed_draw_replaced = methods.DrawIndexedPrimitive != hook_draw_indexed_primitive;
    bool is_viewport_replaced = methods.SetViewport != hook_set_viewport;
    bool is_clear_replaced = methods.Clear != hook_clear;
    bool is_any_draw_replaced = is_draw_replaced || is_indexed_draw_replaced;
    if (!is_any_draw_replaced && !is_viewport_replaced && !is_clear_replaced)
        return;

    // Take each replacement as the new original.
    if (is_draw_replaced)
        original_draw_primitive = methods.DrawPrimitive;
    if (is_indexed_draw_replaced)
        original_draw_indexed_primitive = methods.DrawIndexedPrimitive;
    if (is_viewport_replaced)
        original_set_viewport = methods.SetViewport;
    if (is_clear_replaced)
        original_clear = methods.Clear;
    methods.DrawPrimitive = hook_draw_primitive;
    methods.DrawIndexedPrimitive = hook_draw_indexed_primitive;
    methods.SetViewport = hook_set_viewport;
    methods.Clear = hook_clear;
    write_vtable(patched_vtable, &methods);
}


// `unpatch_device` puts the original methods back into the vtable that `patch_device` changed.
static void unpatch_device(void) {
    if (!patched_vtable)
        return;
    IDirect3DDevice7Vtbl methods = *patched_vtable;
    methods.DrawPrimitive = original_draw_primitive;
    methods.DrawIndexedPrimitive = original_draw_indexed_primitive;
    methods.SetViewport = original_set_viewport;
    methods.Clear = original_clear;
    if (!write_vtable(patched_vtable, &methods)) {
        write_log("ui scale: cannot restore the methods of the Direct3D device");
        return;
    }
    patched_vtable = NULL;
}


// `patch_device` makes `device` draw, set viewports and clear through the hooks above. All devices
// of one type share their vtable, so a device that the game creates again keeps the hooks.
static void patch_device(IDirect3DDevice7 *device) {
    if (device->lpVtbl == patched_vtable)
        return;
    unpatch_device();
    IDirect3DDevice7Vtbl methods = *device->lpVtbl;
    original_draw_primitive = methods.DrawPrimitive;
    original_draw_indexed_primitive = methods.DrawIndexedPrimitive;
    original_set_viewport = methods.SetViewport;
    original_clear = methods.Clear;
    methods.DrawPrimitive = hook_draw_primitive;
    methods.DrawIndexedPrimitive = hook_draw_indexed_primitive;
    methods.SetViewport = hook_set_viewport;
    methods.Clear = hook_clear;
    if (!write_vtable(device->lpVtbl, &methods)) {
        write_log("ui scale: cannot change the methods of the Direct3D device");
        return;
    }
    patched_vtable = device->lpVtbl;
}


// `set_layout_size` gives `renderer` the size `width` by `height` that the game code reads, and
// places the menu layout and the cursor area in it, as the game does after a size change.
static void set_layout_size(void *renderer, int width, int height) {
    write_int(renderer, RENDERER_WIDTH, width);
    write_int(renderer, RENDERER_HEIGHT, height);
    void *settings = *(void **)SETTINGS_POINTER_ADDRESS;
    write_float(settings, MENU_ORIGIN_X, (float)(width / 2 - MENU_WIDTH / 2));
    write_float(settings, MENU_ORIGIN_Y, (float)(height / 2 - MENU_HEIGHT / 2));
    void *mouse = *(void **)MOUSE_POINTER_ADDRESS;
    SetMouseAreaFunction set_mouse_area = (SetMouseAreaFunction)SET_MOUSE_AREA_ADDRESS;
    if (mouse)
        set_mouse_area(mouse, 0, 0, width, height);
}


// `start_game_screen` gives the game code the game screen of `renderer`, whose D3DX context has
// just been created at a modern size, and hooks the Direct3D device.
static void start_game_screen(void *renderer) {
    screen_width = read_int(renderer, RENDERER_WIDTH);
    screen_height = read_int(renderer, RENDERER_HEIGHT);

    // The game screen must hold the 640x480 menu layout.
    float scale = configured_scale;
    float width_limit = (float)screen_width / MENU_WIDTH;
    float height_limit = (float)screen_height / MENU_HEIGHT;
    float scale_limit = fminf(width_limit, height_limit);
    if (scale > scale_limit) {
        write_log("ui scale: %.2f would leave less than 640x480 at %dx%d; %.2f instead", scale,
                  screen_width, screen_height, scale_limit);
        scale = scale_limit;
    }

    // The game screen has whole pixels, so each axis gets the factor of its rounded size.
    game_width = (int)lroundf(screen_width / scale);
    game_height = (int)lroundf(screen_height / scale);
    x_factor = (float)screen_width / game_width;
    y_factor = (float)screen_height / game_height;
    patch_device(read_pointer(renderer, RENDERER_DEVICE));
    if (!patched_vtable)
        return;
    set_layout_size(renderer, game_width, game_height);
    is_scaled = true;
    write_log("ui scale: screen %dx%d, game screen %dx%d (scale %.2f)", screen_width,
              screen_height, game_width, game_height, scale);
}


// `hook_create_context` creates the D3DX context of `renderer` at the screen size, and gives the
// game code the game screen again when that size is a modern one.
static HRESULT THISCALL hook_create_context(void *renderer, int first_argument,
                                            int second_argument) {
    // After a lost device, the renderer creates its context again at the size it holds, which is
    // then the game screen. After a size change, slot 2 has just written the new screen size.
    bool is_game_width = read_int(renderer, RENDERER_WIDTH) == game_width;
    bool is_game_screen = is_game_width && read_int(renderer, RENDERER_HEIGHT) == game_height;
    if (is_scaled && is_game_screen) {
        write_int(renderer, RENDERER_WIDTH, screen_width);
        write_int(renderer, RENDERER_HEIGHT, screen_height);
    }
    is_scaled = false;

    HRESULT result = original_create_context(renderer, first_argument, second_argument);
    int width = read_int(renderer, RENDERER_WIDTH);
    int height = read_int(renderer, RENDERER_HEIGHT);
    if (SUCCEEDED(result) && is_modern_screen_size(width, height))
        start_game_screen(renderer);
    return result;
}


// `hook_blit_image` copies the image item `image` as the game does, but stretched from the game
// screen to the screen, since BltFast cannot change the size of an image.
static void THISCALL hook_blit_image(void *renderer, const void *image) {
    if (!is_scaled) {
        original_blit_image(renderer, image);
        return;
    }

    // Target and source as the game finds them.
    void *context = read_pointer(renderer, RENDERER_CONTEXT);
    void *const *context_vtable = read_pointer(context, 0);
    GetTargetSurfaceFunction get_target_surface =
        (GetTargetSurfaceFunction)context_vtable[CONTEXT_TARGET_SURFACE_SLOT];
    IDirectDrawSurface7 *target = get_target_surface(context, TARGET_SURFACE_INDEX);
    const void *texture = read_pointer(image, IMAGE_TEXTURE);
    const void *surface_owner = read_pointer(texture, TEXTURE_SURFACE_OWNER);
    IDirectDrawSurface7 *source = read_pointer(surface_owner, OWNER_SURFACE);

    // The game cuts the source rectangle at the last row of its screen (0x4348b9).
    RECT source_area = {
        .left = read_int(image, IMAGE_SOURCE_X),
        .top = read_int(image, IMAGE_SOURCE_Y),
        .right = read_int(image, IMAGE_SOURCE_X) + read_int(image, IMAGE_WIDTH),
        .bottom = read_int(image, IMAGE_SOURCE_Y) + read_int(image, IMAGE_HEIGHT),
    };
    if (source_area.bottom > game_height - 1)
        source_area.bottom = game_height - 1;
    LONG left = read_int(image, IMAGE_X);
    LONG top = read_int(image, IMAGE_Y);
    RECT target_area = {
        .left = scale_x(left),
        .top = scale_y(top),
        .right = scale_x(left + source_area.right - source_area.left),
        .bottom = scale_y(top + source_area.bottom - source_area.top),
    };
    HRESULT result =
        target->lpVtbl->Blt(target, &target_area, source, &source_area, DDBLT_WAIT, NULL);
    if (FAILED(result))
        write_log("ui scale: image copy failed (0x%08lx)", (unsigned long)result);
}


// `hook_movie_blt` copies a movie frame into the rectangle `target_area` of the game screen,
// converted to screen pixels. A NULL rectangle (the whole target) stays as it is.
static HRESULT WINAPI hook_movie_blt(IDirectDrawSurface7 *surface, RECT *target_area,
                                     IDirectDrawSurface7 *source, RECT *source_area,
                                     DWORD flags, DDBLTFX *effects) {
    RECT screen_area;
    bool is_game_area = target_area && is_in_game_screen(target_area->left, target_area->top,
                                                         target_area->right, target_area->bottom);
    if (is_game_area) {
        screen_area.left = scale_x(target_area->left);
        screen_area.top = scale_y(target_area->top);
        screen_area.right = scale_x(target_area->right);
        screen_area.bottom = scale_y(target_area->bottom);
        target_area = &screen_area;
    }
    return original_movie_blt(surface, target_area, source, source_area, flags, effects);
}


// `write_method` writes `method` into the vtable entry `entry`, and returns false when the memory
// of the vtable cannot be opened for writing.
static bool write_method(void **entry, void *method) {
    DWORD old_protection;
    if (!VirtualProtect(entry, sizeof *entry, PAGE_READWRITE, &old_protection))
        return false;
    *entry = method;
    VirtualProtect(entry, sizeof *entry, old_protection, &old_protection);
    return true;
}


// `hook_show_movie_frame` shows the next frame of `movie` stretched from the game screen to the
// screen. The game centers the frame with the renderer size, which is the game screen, so the
// Blt of the target surface goes through `hook_movie_blt` for the time of that one frame.
static bool THISCALL hook_show_movie_frame(void *movie) {
    if (!is_scaled)
        return original_show_movie_frame(movie);
    IDirectDrawSurface7 *target = read_pointer(movie, MOVIE_TARGET_SURFACE);
    void **blt_entry = (void **)&target->lpVtbl->Blt;
    original_movie_blt = (SurfaceBltFunction)*blt_entry;
    if (!write_method(blt_entry, (void *)hook_movie_blt))
        return original_show_movie_frame(movie);
    bool result = original_show_movie_frame(movie);
    write_method(blt_entry, (void *)original_movie_blt);
    return result;
}


// `is_modern_screen_renderer` tells whether `renderer` is the D3DX renderer at a modern size.
static bool is_modern_screen_renderer(const void *renderer) {
    if (!renderer || read_pointer(renderer, 0) != (void *)D3DX_RENDERER_VTABLE)
        return false;
    int width = read_int(renderer, RENDERER_WIDTH);
    return is_modern_screen_size(width, read_int(renderer, RENDERER_HEIGHT));
}


// `install_ui_scale_hook` hooks the creation of the D3DX context and the image copy of the
// renderer with `install_hook`, when `ui_scale` is set.
void install_ui_scale_hook(InstallHookFunction install_hook, const ModConfig *config) {
    if (!config->ui_scale)
        return;
    if (!config->is_modern_resolution_on) {
        write_log("ui scale: needs modern_resolution on; the interface keeps its size");
        return;
    }
    configured_scale = config->ui_scale;
    original_create_context = install_hook(CREATE_CONTEXT_ADDRESS, CREATE_CONTEXT_STOLEN_BYTES,
                                           (const void *)hook_create_context);
    if (!original_create_context) {
        write_log("hook of the D3DX context failed: the interface keeps its size");
        return;
    }
    original_blit_image = install_hook(BLIT_IMAGE_ADDRESS, BLIT_IMAGE_STOLEN_BYTES,
                                       (const void *)hook_blit_image);
    if (!original_blit_image)
        write_log("hook of the image copy failed: copied images keep their size");
    original_show_movie_frame = install_hook(SHOW_MOVIE_FRAME_ADDRESS,
                                             SHOW_MOVIE_FRAME_STOLEN_BYTES,
                                             (const void *)hook_show_movie_frame);
    if (!original_show_movie_frame)
        write_log("hook of the movie frames failed: movies keep their size");

    // A core that loads while the screen has a modern size scales the interface at its first
    // shown frame (`note_ui_scale_frame`).
    is_load_check_pending = true;
}


// `note_ui_scale_frame` starts the scale once after a core load during a modern size. The core
// reads the renderer pointer of the game only here, between frames: loader 0.4.0 can load the
// core inside the renderer constructor of a device change, while that pointer still holds the
// deleted renderer.
void note_ui_scale_frame(void) {
    if (!is_load_check_pending)
        return;
    is_load_check_pending = false;
    void *renderer = *(void **)RENDERER_POINTER_ADDRESS;
    if (!is_scaled && is_modern_screen_renderer(renderer))
        start_game_screen(renderer);
}


// `read_screen_size` sets `width` and `height` to the size of the D3DX context of `renderer`.
void read_screen_size(const void *renderer, int *width, int *height) {
    if (is_scaled) {
        *width = screen_width;
        *height = screen_height;
        return;
    }
    *width = read_int(renderer, RENDERER_WIDTH);
    *height = read_int(renderer, RENDERER_HEIGHT);
}


// `stop_ui_scale` gives the game code the screen size again and restores the Direct3D device, so
// that a next core without the scale leaves no part of the screen unused.
void stop_ui_scale(void) {
    void *renderer = *(void **)RENDERER_POINTER_ADDRESS;
    if (is_scaled && renderer)
        set_layout_size(renderer, screen_width, screen_height);
    is_scaled = false;
    unpatch_device();
}
