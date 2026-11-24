// Camera rotation with the middle mouse button (`--middlerotate`). The base game turns the camera
// while the cursor touches a border of the screen: 0x4b6a80, which the input handler of the
// interface runs each frame while no selection box is open, changes the yaw at the left and right
// borders and the pitch at the top and bottom borders, at the rotation rate of the game options.
// The game reads only two mouse buttons, so the middle button has no use in the base game.
//
// A hook replaces 0x4b6a80, so the borders no longer turn the camera. It reads the middle button
// from the DirectInput mouse of the game, and while the button is down, it turns the cursor
// movement of each frame into yaw and pitch changes, with the functions and the tutorial locks of
// the border code, and puts the cursor back at the point where the button went down. Moving the
// mouse to the left turns the camera as the left border did, and moving it up turns it as the top
// border did.
#include <stdbool.h>
#include <stdint.h>

#include "mod.h"
#include "game_object.h"
#include "sim.h"

#define THISCALL __attribute__((thiscall))
#define STDCALL __attribute__((stdcall))

// Border rotation: a thiscall method of the interface object (`Ingameinterface`) without
// arguments. Its first instructions are push -1, push imm32 (7 bytes,
// decompilation/tools/gcdis.py dis).
#define BORDER_ROTATION_ADDRESS 0x4b6a80
#define BORDER_ROTATION_STOLEN_BYTES 7

// The camera is a member of the interface object. 0x4c23b0 adds an angle to its yaw (`+0x668`)
// and 0x4c2330 to its pitch (`+0x664`); both keep the angle in [0, 2 pi), and the pitch function
// also keeps the camera from tilting past straight down or straight up.
#define INTERFACE_CAMERA 0x1c4
#define TURN_YAW_ADDRESS 0x4c23b0
#define TURN_PITCH_ADDRESS 0x4c2330

// Settings of the border rotation. Both rates come from one game option (0x4ac240) and are 2 rad/s
// by default. The tutorial scripts lock and unlock each axis (`EnableYRotation`, `EnableXRotation`).
#define YAW_RATE_ADDRESS 0x6e6138           // float, radians per second
#define PITCH_RATE_ADDRESS 0x6e613c         // float, radians per second
#define IS_YAW_UNLOCKED_ADDRESS 0x67d0c6    // byte, not 0 while the yaw may change
#define IS_PITCH_UNLOCKED_ADDRESS 0x67d0c5  // byte, not 0 while the pitch may change

// At the sensitivity 1, a pixel of cursor movement turns the camera as far as a border turns it in
// this time, so the rotation option of the game scales the turn too. At the default rate, a full
// turn then takes about 1750 pixels.
#define ROTATION_SECONDS_PER_PIXEL 0.0018f

// Mouse of the game. The mouse object keeps the cursor position in pixels, which its update
// (0x418fc0) moves by the DirectInput motion times the mouse sensitivity of the game options and
// keeps inside the screen. 0x4c2aa0 (stdcall: x, y) moves the cursor to a point.
#define MOUSE_OBJECT_POINTER_ADDRESS 0x69b694
#define MOUSE_CURSOR_X 0x28                 // 32-bit integer
#define MOUSE_CURSOR_Y 0x2c                 // 32-bit integer
#define SET_CURSOR_ADDRESS 0x4c2aa0

// DirectInput device of the mouse (IDirectInputDevice7A), with the data format DIMOUSESTATE2: the
// game reads its state through slot `+0x24` (GetDeviceState) with that size. The game handles only
// the buffered events of buttons 0 and 1, so the state of button 2, the middle button, is read here.
#define MOUSE_DEVICE_POINTER_ADDRESS 0x69b690
#define GET_DEVICE_STATE_SLOT 0x24          // byte offset of GetDeviceState in the vtable
#define MIDDLE_BUTTON_INDEX 2
#define BUTTON_DOWN_BIT 0x80
#define MOUSE_BUTTON_COUNT 8

// Layout of DIMOUSESTATE2: the motion on each axis since the last read, then one byte per button.
typedef struct {
    int32_t x_motion;
    int32_t y_motion;
    int32_t wheel_motion;
    uint8_t buttons[MOUSE_BUTTON_COUNT];
} MouseState;

typedef void(THISCALL *TurnCameraFunction)(void *camera, float angle);
typedef void(STDCALL *SetCursorFunction)(int x, int y);
typedef long(STDCALL *GetDeviceStateFunction)(void *device, unsigned long size, void *state);

static float sensitivity;                   // `middlerotate_sensitivity` of gcmod.ini
static bool is_rotating;                    // the middle button was down in the last call
static int held_cursor_x;                   // cursor position where the middle button went down
static int held_cursor_y;


// `is_middle_button_down` reads the state of the middle button from the mouse device of the game.
// A device that has not acquired the mouse, such as while the game window is in the background,
// reports no button down.
static bool is_middle_button_down(void) {
    void *device = *(void **)MOUSE_DEVICE_POINTER_ADDRESS;
    if (!device)
        return false;
    const void *vtable = read_pointer(device, 0);
    GetDeviceStateFunction get_state = read_pointer(vtable, GET_DEVICE_STATE_SLOT);
    MouseState state;
    if (get_state(device, sizeof state, &state) != 0)
        return false;
    return (state.buttons[MIDDLE_BUTTON_INDEX] & BUTTON_DOWN_BIT) != 0;
}


// `get_camera` returns the camera, a member of the interface object `game_interface`.
static void *get_camera(void *game_interface) {
    return &((uint8_t *)game_interface)[INTERFACE_CAMERA];
}


// `turn_camera` turns the camera of `game_interface` for a cursor movement of `x_pixels` to the
// right and `y_pixels` down, in the direction in which the border on that side turns it.
static void turn_camera(void *game_interface, int x_pixels, int y_pixels) {
    void *camera = get_camera(game_interface);
    float seconds_per_pixel = ROTATION_SECONDS_PER_PIXEL * sensitivity;
    bool is_yaw_unlocked = read_byte((const void *)IS_YAW_UNLOCKED_ADDRESS, 0);
    if (x_pixels != 0 && is_yaw_unlocked) {
        float rate = read_float((const void *)YAW_RATE_ADDRESS, 0);       // yaw rate of a border
        TurnCameraFunction turn_yaw = (TurnCameraFunction)TURN_YAW_ADDRESS;
        turn_yaw(camera, x_pixels * rate * seconds_per_pixel);
    }
    bool is_pitch_unlocked = read_byte((const void *)IS_PITCH_UNLOCKED_ADDRESS, 0);
    if (y_pixels != 0 && is_pitch_unlocked) {
        float rate = read_float((const void *)PITCH_RATE_ADDRESS, 0);     // pitch rate of a border
        TurnCameraFunction turn_pitch = (TurnCameraFunction)TURN_PITCH_ADDRESS;
        turn_pitch(camera, y_pixels * rate * seconds_per_pixel);
    }
}


// `hook_border_rotation` turns the camera by the cursor movement while the middle button is down,
// in place of the border rotation of the game.
static void THISCALL hook_border_rotation(void *game_interface) {
    if (!is_middle_button_down()) {
        is_rotating = false;
        return;
    }

    // The first frame with the button down only records the cursor position.
    const void *mouse = *(const void **)MOUSE_OBJECT_POINTER_ADDRESS;
    int cursor_x = read_int(mouse, MOUSE_CURSOR_X);
    int cursor_y = read_int(mouse, MOUSE_CURSOR_Y);
    if (!is_rotating) {
        is_rotating = true;
        held_cursor_x = cursor_x;
        held_cursor_y = cursor_y;
        return;
    }

    // Without the move back, the cursor would stop at the edge of the screen, and further mouse
    // movement toward that edge would turn nothing.
    turn_camera(game_interface, cursor_x - held_cursor_x, cursor_y - held_cursor_y);
    SetCursorFunction set_cursor = (SetCursorFunction)SET_CURSOR_ADDRESS;
    set_cursor(held_cursor_x, held_cursor_y);
}


// `install_middle_rotate_hook` hooks the border rotation with `install_hook` while the switch is
// on; without the switch, the game function stays as it is. The hook never runs the original, so
// the trampoline only shows whether the hook went in.
void install_middle_rotate_hook(InstallHookFunction install_hook, const ModConfig *config) {
    if (!is_middle_rotate_on)
        return;
    sensitivity = config->middle_rotate_sensitivity;
    is_rotating = false;
    void *trampoline = install_hook(BORDER_ROTATION_ADDRESS, BORDER_ROTATION_STOLEN_BYTES,
                                    (const void *)hook_border_rotation);
    if (!trampoline)
        write_log("hook of the border rotation failed: the middle button does not turn the camera");
}
