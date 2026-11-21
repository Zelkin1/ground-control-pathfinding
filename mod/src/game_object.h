// Reads and writes of the fields of game objects. The mod has no C types for the objects of
// gc.exe, so it names each field by its byte offset (docs/PATHFINDING.md), and these functions
// copy the bytes, which works at any alignment.
#ifndef GAME_OBJECT_H
#define GAME_OBJECT_H

#include <stdint.h>
#include <string.h>


// `read_float` reads the float at `offset` in the game object `object`.
static inline float read_float(const void *object, int offset) {
    float value;
    memcpy(&value, &((const uint8_t *)object)[offset], sizeof value);
    return value;
}


// `write_float` writes the float at `offset` in the game object `object`.
static inline void write_float(void *object, int offset, float value) {
    memcpy(&((uint8_t *)object)[offset], &value, sizeof value);
}


// `read_pointer` reads the pointer at `offset` in the game object `object`.
static inline void *read_pointer(const void *object, int offset) {
    void *value;
    memcpy(&value, &((const uint8_t *)object)[offset], sizeof value);
    return value;
}


// `write_pointer` writes the pointer at `offset` in the game object `object`.
static inline void write_pointer(void *object, int offset, const void *value) {
    memcpy(&((uint8_t *)object)[offset], &value, sizeof value);
}


// `read_byte` reads the byte at `offset` in the game object `object`.
static inline uint8_t read_byte(const void *object, int offset) {
    return ((const uint8_t *)object)[offset];
}


// `write_byte` writes the byte at `offset` in the game object `object`.
static inline void write_byte(void *object, int offset, uint8_t value) {
    ((uint8_t *)object)[offset] = value;
}


// `read_u16` reads the unsigned 16-bit number at `offset` in the game object `object`.
static inline uint16_t read_u16(const void *object, int offset) {
    uint16_t value;
    memcpy(&value, &((const uint8_t *)object)[offset], sizeof value);
    return value;
}


// `read_int` reads the 32-bit integer at `offset` in the game object `object`.
static inline int read_int(const void *object, int offset) {
    int value;
    memcpy(&value, &((const uint8_t *)object)[offset], sizeof value);
    return value;
}


// `write_int` writes the 32-bit integer at `offset` in the game object `object`.
static inline void write_int(void *object, int offset, int value) {
    memcpy(&((uint8_t *)object)[offset], &value, sizeof value);
}

#endif
