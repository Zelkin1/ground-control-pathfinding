// Fields of the game objects of the server side that more than one file of the core reads: the
// entity (a unit of the game), its entity type, its mover and its squad. The bridge (bridge.c)
// and the run-over kills (crush.c) use them. Offsets come from docs/PATHFINDING.md.
#ifndef GAME_ENTITY_H
#define GAME_ENTITY_H

#include "game_object.h"

#define THISCALL __attribute__((thiscall))

// Entity. The map plane is (x, z) of the game, which is (x, y) in the formation code.
#define ENTITY_X 0x0c                       // float
#define ENTITY_Y 0x14                       // float, z of the game
#define ENTITY_TYPE 0x54                    // pointer to the entity type
#define ENTITY_MOVER 0x58                   // pointer to the mover, or NULL
#define ENTITY_ACTIVE 0x88                  // byte; Update and Steer skip an inactive unit
// Pointer to the APC that carries the unit, or NULL. *Tick* sets it when a unit that drives to
// an APC comes within twice the APC radius (0x50f634), so the unit boards it.
#define ENTITY_CARRIER 0xac

// Entity type, from the ENT file of the unit (`EntityData::LoadMe` 0x4212c0).
#define TYPE_RADIUS 0x88                    // float

// Mover (the movement state of a unit).
#define MOVER_VELOCITY_X 0x08               // float
#define MOVER_VELOCITY_Y 0x0c               // float
#define MOVER_ENTITY 0x74                   // pointer to the entity

// Squad. The member vector holds entity pointers from `begin` to `end`.
#define SQUAD_MEMBER_VECTOR 0x14            // pointer to the member vector
#define VECTOR_BEGIN 0x00
#define VECTOR_END 0x0c

// Team of a squad, a byte from 0 to 9, which `Squad::LogAlliedAndEnemyStats` 0x5099a0 passes to
// the alliance test. thiscall (team byte, other team byte): whether the two teams are allies; a
// team is its own ally, and the table of allied teams depends on the game mode (global 0x67faa4).
#define SQUAD_TEAM 0x18
#define IS_ALLIED_TEAM_ADDRESS 0x4e6840

typedef uint8_t(THISCALL *IsAlliedTeamFunction)(const void *team, const void *other_team);


// `get_members` returns the member vector of a game squad and its length in `member_count`.
static inline void **get_members(const void *squad_object, int *member_count) {
    const void *vector = read_pointer(squad_object, SQUAD_MEMBER_VECTOR);
    void **begin = read_pointer(vector, VECTOR_BEGIN);
    void **end = read_pointer(vector, VECTOR_END);
    *member_count = (int)(end - begin);
    return begin;
}


// `get_squad_team` returns the address of the team byte of a game squad, which the alliance test
// of the game reads.
static inline const void *get_squad_team(const void *squad_object) {
    return &((const uint8_t *)squad_object)[SQUAD_TEAM];
}

#endif
