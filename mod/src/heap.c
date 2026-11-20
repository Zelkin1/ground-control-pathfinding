// Memory of the core. The core and the formation code allocate their tables (lanes, clearance
// fields, smoothing buffers) with `malloc` and never free most of them. These definitions replace
// `malloc` and its relatives in the core with a heap of its own, which the loader destroys after it
// unloads the core (reload.c). Without it, each reload would leave about 15 MB behind in the
// 32-bit address space of the game. The C runtime functions inside msvcrt.dll keep their own heap.
#include <stddef.h>
#include <windows.h>

#include "mod.h"

static HANDLE core_heap;


// `get_heap` creates the heap at the first allocation, which can come from the start-up code of
// the C runtime, before `start_core`.
static HANDLE get_heap(void) {
    if (!core_heap)
        core_heap = HeapCreate(0, 0, 0);
    return core_heap;
}


// `get_core_heap` returns the heap, or NULL before the first allocation.
void *get_core_heap(void) {
    return core_heap;
}


// `malloc` allocates from the heap of the core. A size of 0 gives a block of 1 byte, so that the
// result is a pointer that `free` takes.
void *malloc(size_t size) {
    return HeapAlloc(get_heap(), 0, size ? size : 1);
}


// `calloc` allocates a zeroed block from the heap of the core.
void *calloc(size_t count, size_t size) {
    size_t byte_count = count * size;
    return HeapAlloc(get_heap(), HEAP_ZERO_MEMORY, byte_count ? byte_count : 1);
}


// `realloc` resizes a block of the heap of the core, as the C function does for NULL and size 0.
void *realloc(void *block, size_t size) {
    if (!block)
        return malloc(size);
    if (size == 0) {
        HeapFree(get_heap(), 0, block);
        return NULL;
    }
    return HeapReAlloc(get_heap(), 0, block, size);
}


// `free` returns a block to the heap of the core.
void free(void *block) {
    if (block)
        HeapFree(get_heap(), 0, block);
}
