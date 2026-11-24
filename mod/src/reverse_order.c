// Backward move orders of the player (`--reverse`). The server of the game stores each move order
// with `Squad::OrderMove` 0x50ab30 before it searches the route. A hook on it tells the bridge
// whether the reverse key (gcmod.ini `reverse_key`, Alt by default) is down at that moment; the
// formation set-up of the order then lets the squad drive backward when its destination lies
// behind it (shared/formation/formation.c `decide_reverse`).
//
// A queued order (a chain of move orders) starts only when the squad reaches the order before it,
// so the key state at that time says nothing about it. A second hook, on `Squad::QueueMove`
// 0x505110, therefore records the key state when the player queues the order, by the queue item
// of the order; the queue later hands that item to `Squad::OrderMove`.
#include <windows.h>

#include "mod.h"
#include "game_object.h"
#include "sim.h"

#define THISCALL __attribute__((thiscall))

// thiscall (squad, destination point, facing x, facing y, quiet flag), `ret 0x10`. Its callers
// are `ParseMove` 0x4f3a60 (move order of a player, facing 0, 0), `ParseFaceMove` 0x4f3ca0 (move
// order with a facing) and the queue 0x506aee (`PopQueuedOrder`, quiet 1), which passes the
// queue item itself as the destination point: the point is the first field of the item.
#define ORDER_MOVE_ADDRESS 0x50ab30
#define ORDER_MOVE_STOLEN_BYTES 6           // mov eax, [esp + 4]; mov edx, [eax]
// thiscall (squad, point, facing), `ret 8`: appends a queue item of order type 6 (a searched move)
// and returns true, or false when it refuses the order. Its callers are `ParseStackMove` 0x4f3b20
// and `ParseStackFaceMove` 0x4f3d60, the queued move orders of a player.
#define QUEUE_MOVE_ADDRESS 0x505110
#define QUEUE_MOVE_STOLEN_BYTES 6           // push ebx; push ebp; push esi; push edi; mov edi, ecx
// Queue of a squad: list node pointer of its last item; a node holds the item pointer first.
#define SQUAD_QUEUE_TAIL 0x40
#define NODE_ITEM 0x00
#define KEY_DOWN_BIT 0x8000                 // bit of `GetAsyncKeyState` while a key is down
#define MAX_REVERSE_ITEMS 256               // queued backward orders that the mod keeps track of

typedef void(THISCALL *OrderMoveFunction)(void *squad_object, const void *destination,
                                          float facing_x, float facing_y, uint8_t is_quiet);
typedef uint8_t(THISCALL *QueueMoveFunction)(void *squad_object, const void *point,
                                             const void *facing);

static OrderMoveFunction original_order_move;
static QueueMoveFunction original_queue_move;
static int reverse_key;
// Queue items of the queued orders that the player gave with the reverse key down, oldest first.
// Every item of order type 6 comes from `Squad::QueueMove`, which forgets an item address there
// before a new item can take it.
static const void *reverse_items[MAX_REVERSE_ITEMS];
static int reverse_item_count;


// `is_reverse_key_down` tells whether the reverse key is down now.
static bool is_reverse_key_down(void) {
    return (GetAsyncKeyState(reverse_key) & KEY_DOWN_BIT) != 0;
}


// `take_reverse_item` removes `item` from `reverse_items` and tells whether it was there.
static bool take_reverse_item(const void *item) {
    for (int item_index = 0; item_index < reverse_item_count; item_index++) {
        if (reverse_items[item_index] != item)
            continue;
        int later_count = reverse_item_count - item_index - 1;
        memmove(&reverse_items[item_index], &reverse_items[item_index + 1],
                (size_t)later_count * sizeof reverse_items[0]);
        reverse_item_count--;
        return true;
    }
    return false;
}


// `add_reverse_item` appends `item` to `reverse_items`; when the list is full, the oldest entry
// gives way, since its order has most likely started or been cleared long ago.
static void add_reverse_item(const void *item) {
    if (reverse_item_count == MAX_REVERSE_ITEMS)
        take_reverse_item(reverse_items[0]);
    reverse_items[reverse_item_count] = item;
    reverse_item_count++;
}


// `hook_order_move` stores the order with the original, then tells the bridge whether it asks for
// a backward move: for a queued order, whether the key was down when the player queued it.
static void THISCALL hook_order_move(void *squad_object, const void *destination, float facing_x,
                                     float facing_y, uint8_t is_quiet) {
    original_order_move(squad_object, destination, facing_x, facing_y, is_quiet);
    bool is_reverse_order = is_reverse_key_down();
    if (is_quiet)
        is_reverse_order = take_reverse_item(destination);
    note_move_order(squad_object, is_reverse_order);
}


// `hook_queue_move` queues the order with the original, and records its new queue item when the
// reverse key is down.
static uint8_t THISCALL hook_queue_move(void *squad_object, const void *point, const void *facing) {
    uint8_t is_queued = original_queue_move(squad_object, point, facing);
    if (!is_queued)
        return is_queued;
    const void *tail = read_pointer(squad_object, SQUAD_QUEUE_TAIL);
    const void *item = read_pointer(tail, NODE_ITEM);
    take_reverse_item(item);
    if (is_reverse_key_down())
        add_reverse_item(item);
    return is_queued;
}


// `install_reverse_order_hook` hooks `Squad::OrderMove` and `Squad::QueueMove` with
// `install_hook` while the switch is on.
void install_reverse_order_hook(InstallHookFunction install_hook, const ModConfig *config) {
    if (!is_reverse_on)
        return;
    reverse_key = config->reverse_key;
    reverse_item_count = 0;
    original_order_move = install_hook(ORDER_MOVE_ADDRESS, ORDER_MOVE_STOLEN_BYTES,
                                       (const void *)hook_order_move);
    if (!original_order_move)
        write_log("hook of the move order failed: no backward moves");
    original_queue_move = install_hook(QUEUE_MOVE_ADDRESS, QUEUE_MOVE_STOLEN_BYTES,
                                       (const void *)hook_queue_move);
    if (!original_queue_move)
        write_log("hook of the queued move order failed: queued orders drive forward");
}
