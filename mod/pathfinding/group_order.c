// Positions of squads that get one move order together (`--keeporder`). When the player orders
// several selected squads, `Ingameinterface::CalculateSquadFormationPoints` (0x4b1020) lays out
// one position per squad, in rows of up to 4 across the facing of the order, and the game hands
// the `i`-th position to the `i`-th selected squad. The selection order says nothing about where
// the squads stand, so squads could cross each other on the way. `note_group_positions` hands the
// same positions out again by place: the front squads get the front row, and in each row the
// squads keep their order from side to side.
//
//        facing of the order
//              ^
//      A'  B'  C'          A, B and C stand in this order across the facing, so they get
//                          A', B' and C', also in this order, whatever order the player
//    A     B      C        selected them in.
//
// The order preview and the order that the interface sends to the game both read the list of
// positions after this function, in the order of the selected squads.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mod.h"
#include "game_object.h"
#include "sim.h"

// Fields of the interface object (`Ingameinterface`). Points are x, y pairs of floats.
#define INTERFACE_SQUAD_LIST 0x1a4          // first node of the list of the squads of the player
#define INTERFACE_ORDER_POINT 0x10cb8       // order point, the center of the first row
#define INTERFACE_FACING 0x10cc8            // facing of the order
#define INTERFACE_POSITION_LIST 0x10cd0     // first node of the list of positions
#define INTERFACE_POSITION_COUNT 0x10cdc    // number of positions in that list

// List nodes: the squad list holds a squad pointer, then the next node; the position list holds
// a point, then the next node.
#define SQUAD_NODE_NEXT 4
#define POSITION_NODE_NEXT 8
#define POINT_Y 4                           // y follows x in a point

// Fields of a squad of the interface. Its member list starts with the node of the first member,
// which starts with the 16-bit id of that member in the table of client entities (0x4c6930).
#define SQUAD_IS_SELECTED 0x34              // byte, not 0 while the squad is selected
#define SQUAD_MEMBER_LIST 0x2c              // pointer to the member list
#define MEMBER_LIST_COUNT 8                 // number of members in the member list

// Client entities, by id: a table of pointers in gc.exe. A client entity keeps its shown position
// as x, height, y (`CL_Entity::Interpolate` 0x4bbe10 updates it).
#define CLIENT_ENTITY_TABLE_ADDRESS 0x6e6320
#define CLIENT_ENTITY_COUNT 600
#define ENTITY_SHOWN_X 0x44
#define ENTITY_SHOWN_Y 0x4c

#define MAX_GROUP_SQUADS 64                 // larger groups keep the order of the game
#define ROW_TOLERANCE_METERS 0.5f           // positions of one row lie this close along the facing

// Place of a squad or of a position: `across` is the distance to the right of the order point,
// seen facing along the facing of the order, and `along` the distance ahead of it.
typedef struct {
    float across;
    float along;
    int index;                              // index of the squad or position in its list
} GroupPlace;

// Assignment that `note_group_positions` logged last: position index by squad index.
static int logged_position_indexes[MAX_GROUP_SQUADS];
static int logged_count;


// `read_point` reads the x, y pair of floats at `offset` in the game object `object`.
static V2 read_point(const void *object, int offset) {
    float x = read_float(object, offset);
    float y = read_float(object, offset + POINT_Y);
    return v2(x, y);
}


// `get_client_entity` returns the client entity with the id `entity_id`, or NULL without one.
static const void *get_client_entity(int entity_id) {
    if (entity_id >= CLIENT_ENTITY_COUNT)
        return NULL;
    const void *const *table = (const void *const *)CLIENT_ENTITY_TABLE_ADDRESS;
    return table[entity_id];
}


// `find_squad_point` sets `point` to the shown position of the first member of the squad, which
// stands for the place of the whole squad. It returns false for a squad without members.
static bool find_squad_point(const void *squad, V2 *point) {
    const void *member_list = read_pointer(squad, SQUAD_MEMBER_LIST);
    if (read_int(member_list, MEMBER_LIST_COUNT) == 0)
        return false;
    const void *first_node = read_pointer(member_list, 0);
    const void *entity = get_client_entity(read_u16(first_node, 0));
    if (!entity)
        return false;
    *point = v2(read_float(entity, ENTITY_SHOWN_X), read_float(entity, ENTITY_SHOWN_Y));
    return true;
}


// `measure_place` returns the place of `point` relative to `origin`, for the unit vector `facing`.
static GroupPlace measure_place(V2 point, V2 origin, V2 facing, int index) {
    V2 right = v2(facing.y, -facing.x);
    V2 offset = sub(point, origin);
    return (GroupPlace){
        .across = dot(offset, right),
        .along = dot(offset, facing),
        .index = index,
    };
}


// `compare_front_first` orders places from the front to the back, and then from left to right.
static int compare_front_first(const void *first, const void *second) {
    const GroupPlace *a = first;
    const GroupPlace *b = second;
    if (a->along != b->along)
        return a->along > b->along ? -1 : 1;
    if (a->across != b->across)
        return a->across < b->across ? -1 : 1;
    return 0;
}


// `compare_left_first` orders places from left to right.
static int compare_left_first(const void *first, const void *second) {
    const GroupPlace *a = first;
    const GroupPlace *b = second;
    if (a->across != b->across)
        return a->across < b->across ? -1 : 1;
    return 0;
}


// `match_places` pairs squads and positions row by row, and writes the position index of each
// squad into `position_indexes`, by squad index. Both arrays of places get sorted.
static void match_places(GroupPlace *squad_places, GroupPlace *position_places, int count,
                         int *position_indexes) {
    qsort(squad_places, (size_t)count, sizeof(GroupPlace), compare_front_first);
    qsort(position_places, (size_t)count, sizeof(GroupPlace), compare_front_first);

    // A row is a run of positions at the same distance along the facing. It takes as many squads,
    // the frontmost ones left, and both sides sort from left to right before they pair up.
    int row_start = 0;
    while (row_start < count) {
        int row_end = row_start + 1;
        float row_along = position_places[row_start].along;
        while (row_end < count &&
               fabsf(position_places[row_end].along - row_along) < ROW_TOLERANCE_METERS)
            row_end++;
        size_t row_size = (size_t)(row_end - row_start);
        qsort(&squad_places[row_start], row_size, sizeof(GroupPlace), compare_left_first);
        qsort(&position_places[row_start], row_size, sizeof(GroupPlace), compare_left_first);
        for (int place_index = row_start; place_index < row_end; place_index++) {
            int squad_index = squad_places[place_index].index;
            position_indexes[squad_index] = position_places[place_index].index;
        }
        row_start = row_end;
    }
}


// `log_new_assignment` logs the assignment when it differs from the order of the game and from
// the one logged last; the preview computes the positions again in many frames.
static void log_new_assignment(const int *position_indexes, int count) {
    // An assignment that the game would make too, or that the log already shows, adds nothing.
    bool is_game_order = true;
    bool is_logged = count == logged_count;
    for (int squad_index = 0; squad_index < count; squad_index++) {
        int position_index = position_indexes[squad_index];
        is_game_order = is_game_order && position_index == squad_index;
        is_logged = is_logged && position_index == logged_position_indexes[squad_index];
    }
    if (is_game_order || is_logged)
        return;

    // The line lists the position index of each selected squad, in selection order.
    char text[MAX_GROUP_SQUADS * 4];
    int length = 0;
    for (int squad_index = 0; squad_index < count; squad_index++) {
        length += snprintf(&text[length], sizeof text - (size_t)length, " %d",
                           position_indexes[squad_index]);
        logged_position_indexes[squad_index] = position_indexes[squad_index];
    }
    logged_count = count;
    write_log("group order: %d squads, position of each selected squad:%s", count, text);
}


// `note_group_positions` hands the positions of the interface object `game_interface` out again by
// the places of the selected squads. Without a usable place for every squad, it keeps the order of
// the game.
void note_group_positions(void *game_interface) {
    int count = read_int(game_interface, INTERFACE_POSITION_COUNT);
    if (!is_keep_order_on || count < 2 || count > MAX_GROUP_SQUADS)
        return;
    V2 origin = read_point(game_interface, INTERFACE_ORDER_POINT);
    V2 facing = read_point(game_interface, INTERFACE_FACING);
    float facing_length = len(facing);
    if (facing_length == 0)
        return;
    facing = mul(facing, 1 / facing_length);

    // Places of the selected squads, in the order in which they take the positions.
    GroupPlace squad_places[MAX_GROUP_SQUADS];
    int squad_count = 0;
    const void *squad_node = read_pointer(game_interface, INTERFACE_SQUAD_LIST);
    for (; squad_node; squad_node = read_pointer(squad_node, SQUAD_NODE_NEXT)) {
        const void *squad = read_pointer(squad_node, 0);
        if (!read_byte(squad, SQUAD_IS_SELECTED))
            continue;
        V2 point;
        if (squad_count == count || !find_squad_point(squad, &point))
            return;
        squad_places[squad_count] = measure_place(point, origin, facing, squad_count);
        squad_count++;
    }
    if (squad_count != count)
        return;

    // Places of the positions; the list nodes stay in their order.
    void *position_nodes[MAX_GROUP_SQUADS];
    V2 positions[MAX_GROUP_SQUADS];
    GroupPlace position_places[MAX_GROUP_SQUADS];
    void *position_node = read_pointer(game_interface, INTERFACE_POSITION_LIST);
    for (int position_index = 0; position_index < count; position_index++) {
        if (!position_node)
            return;
        position_nodes[position_index] = position_node;
        positions[position_index] = read_point(position_node, 0);
        V2 position = positions[position_index];
        position_places[position_index] = measure_place(position, origin, facing, position_index);
        position_node = read_pointer(position_node, POSITION_NODE_NEXT);
    }

    // The node of each squad gets the position of its match.
    int position_indexes[MAX_GROUP_SQUADS];
    match_places(squad_places, position_places, count, position_indexes);
    for (int squad_index = 0; squad_index < count; squad_index++) {
        V2 position = positions[position_indexes[squad_index]];
        write_float(position_nodes[squad_index], 0, position.x);
        write_float(position_nodes[squad_index], POINT_Y, position.y);
    }
    log_new_assignment(position_indexes, count);
}
