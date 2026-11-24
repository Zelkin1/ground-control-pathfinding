// Hit slowdown of the game (`disablehitslow`). Each projectile that ends near a unit adds time to the
// slow timer of the unit, and while that timer runs, the speed effects function 0x4fb940 (thiscall
// entity, once per unit and frame) lowers the top speed of its mover (times 0.5 for Crayven, 0.9
// for the Order, 0.75 for Phoenix; not for aircraft) and the turn rate of its turret aimer (times
// 0.7).
//
// With `disablehitslow = on`, a hook on that function lets the original see a run-out timer, so it
// applies neither slowdown, and then sets the timer to the value that the countdown of the original
// would have left. Any other reader of the timer still sees it count down as in the game.
#include "mod.h"
#include "game_object.h"

#define THISCALL __attribute__((thiscall))

#define SPEED_EFFECTS_ADDRESS 0x4fb940
#define SPEED_EFFECTS_STOLEN_BYTES 6        // push esi; mov esi, ecx; mov eax, [esi + 0x58]
#define ENTITY_SLOW_SECONDS 0x84            // float, slow timer of the entity
#define GAME_FRAME_SECONDS_ADDRESS 0x82f5cc         // float, time step that the countdown takes
#define RUN_OUT_SLOW_SECONDS -1.0f          // timer value at which the original applies no slowdown

typedef void(THISCALL *SpeedEffectsFunction)(void *entity);

static SpeedEffectsFunction original_speed_effects;


// `hook_speed_effects` runs the speed effects of the game without the hit slowdown.
static void THISCALL hook_speed_effects(void *entity) {
    // The original counts down by one frame and resets a negative timer to 0, as here.
    float slow_seconds = read_float(entity, ENTITY_SLOW_SECONDS);
    write_float(entity, ENTITY_SLOW_SECONDS, RUN_OUT_SLOW_SECONDS);
    original_speed_effects(entity);
    float frame_seconds = *(const float *)GAME_FRAME_SECONDS_ADDRESS;
    float left_seconds = slow_seconds - frame_seconds;
    write_float(entity, ENTITY_SLOW_SECONDS, left_seconds < 0 ? 0 : left_seconds);
}


// `install_hit_slowdown_hook` hooks the speed effects of the game with `install_hook` when
// gcmod.ini turns the hit slowdown off.
void install_hit_slowdown_hook(InstallHookFunction install_hook, const ModConfig *config) {
    if (!config->is_hit_slowdown_disabled)
        return;
    original_speed_effects = install_hook(SPEED_EFFECTS_ADDRESS, SPEED_EFFECTS_STOLEN_BYTES,
                                          (const void *)hook_speed_effects);
    if (!original_speed_effects)
        write_log("hook of the speed effects failed: hit units still slow down");
}
