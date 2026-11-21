// On-screen text of the mod (the HUD). `count_frame` measures the frame rate from the frames
// shown, and `draw_hud_text` writes lines of text in small grey letters at the top right of the
// finished frame. Like `CD3duContext::DrawDebugText` (0x407d4d) of the D3DX
// utility library in gc.exe, it draws with GDI on a device context (DC) of the render buffer of the
// D3DX context, but with its own font and position.
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <windows.h>

#include "mod.h"

#define CONTEXT_RENDER_BUFFER 0x94          // IDirectDrawSurface7 pointer of the D3DX context
#define SURFACE_GET_DC_SLOT 17              // IDirectDrawSurface7::GetDC (vtable offset 0x44)
#define SURFACE_RELEASE_DC_SLOT 26          // IDirectDrawSurface7::ReleaseDC (vtable offset 0x68)
#define STATUS_FONT_HEIGHT 12               // letter height in pixels
#define STATUS_MARGIN 4                     // gap to the top and right edges, in pixels
#define STATUS_GREY RGB(150, 150, 150)
#define HUD_WIDTH_FRACTION 0.6              // share of the frame width that a line may cover
#define FRAME_RATE_WINDOW_SECONDS 0.5       // time over which `count_frame` averages

typedef HRESULT(STDMETHODCALLTYPE *GetDcFunction)(void *surface, HDC *dc);
typedef HRESULT(STDMETHODCALLTYPE *ReleaseDcFunction)(void *surface, HDC dc);

static HFONT status_font;
static LARGE_INTEGER window_start;          // performance counter at the start of the window
static int window_frame_count;              // frames shown since then
static int frame_rate;                      // frames per second over the last full window


// `count_frame` counts one shown frame and returns the frame rate over the last full window of
// `FRAME_RATE_WINDOW_SECONDS`, or 0 before the first window ends.
int count_frame(void) {
    LARGE_INTEGER now;
    LARGE_INTEGER frequency;
    QueryPerformanceCounter(&now);
    QueryPerformanceFrequency(&frequency);
    if (window_start.QuadPart == 0)
        window_start = now;
    window_frame_count++;
    double ticks = (double)(now.QuadPart - window_start.QuadPart);
    double elapsed_seconds = ticks / (double)frequency.QuadPart;
    if (elapsed_seconds >= FRAME_RATE_WINDOW_SECONDS) {
        frame_rate = (int)lround(window_frame_count / elapsed_seconds);
        window_frame_count = 0;
        window_start = now;
    }
    return frame_rate;
}


// `get_surface_method` returns the method in slot `slot_index` of the vtable of the COM object
// `surface`.
static void *get_surface_method(void *surface, int slot_index) {
    void **vtable;
    memcpy(&vtable, surface, sizeof vtable);
    return vtable[slot_index];
}


// `draw_hud_text` creates the font on first use and draws the lines right-aligned, one below the
// other, from the top. Without a render buffer, or when the buffer gives no DC, the frame stays as
// it is.
void draw_hud_text(void *context, const char *const *lines, int line_count) {
    void *surface;
    memcpy(&surface, &((uint8_t *)context)[CONTEXT_RENDER_BUFFER], sizeof surface);
    if (!surface)
        return;
    GetDcFunction get_dc = (GetDcFunction)get_surface_method(surface, SURFACE_GET_DC_SLOT);
    ReleaseDcFunction release_dc =
        (ReleaseDcFunction)get_surface_method(surface, SURFACE_RELEASE_DC_SLOT);
    HDC dc;
    if (FAILED(get_dc(surface, &dc)))
        return;

    // A negative height asks for the letter height without the inner spacing of the font.
    if (!status_font) {
        status_font = CreateFontA(-STATUS_FONT_HEIGHT, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                                  DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                                  NONANTIALIASED_QUALITY, DEFAULT_PITCH | FF_SWISS, "Arial");
    }
    HGDIOBJ old_font = SelectObject(dc, status_font);
    SetTextColor(dc, STATUS_GREY);
    SetBkMode(dc, TRANSPARENT);
    TEXTMETRICA metrics;
    GetTextMetricsA(dc, &metrics);

    // Each line gets a row as high as the font. A line wider than the row would end in "...",
    // so that long log lines do not cover the whole frame.
    RECT frame_area;
    GetClipBox(dc, &frame_area);
    int frame_width = frame_area.right - frame_area.left;
    RECT row = {
        .left = frame_area.right - (int)(HUD_WIDTH_FRACTION * frame_width),
        .top = frame_area.top + STATUS_MARGIN,
        .right = frame_area.right - STATUS_MARGIN,
        .bottom = frame_area.top + STATUS_MARGIN + metrics.tmHeight,
    };
    UINT format = DT_RIGHT | DT_TOP | DT_SINGLELINE | DT_END_ELLIPSIS | DT_NOPREFIX;
    for (int line_index = 0; line_index < line_count; line_index++) {
        DrawTextA(dc, lines[line_index], -1, &row, format);
        row.top += metrics.tmHeight;
        row.bottom += metrics.tmHeight;
    }
    SelectObject(dc, old_font);
    release_dc(surface, dc);
}


// `release_hud_font` deletes the font, so that a reload of the core leaves no GDI object.
void release_hud_font(void) {
    if (!status_font)
        return;
    DeleteObject(status_font);
    status_font = NULL;
}
