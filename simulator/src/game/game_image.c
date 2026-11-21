// Mapping of gc.exe into the simulator (docs/PATHFINDING.md). `gc_load` maps the image of gc.exe at
// its base address 0x400000, inside the reserved space `game_image_space` of sim.exe, so that the
// simulator can call the base game path search, the raycast and the formation layout directly
// (game.c). sim.exe is linked at 0x10000 without address space layout randomization, so that this
// space covers the base address (see build.sh). The game mod runs inside gc.exe and needs none of
// this.
//
// Before the first call, `gc_load` patches the image: `operator new` and `delete` jump to
// `malloc` and `free`, the logger returns at once, the one-time guard of Expand is set, and the
// static initializer of the path finder runs. Without that initializer, the inverse cell size
// would stay zero, every position would fall into cell 0, and no blocking terrain would exist.
#include "game.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

#define GC_BASE 0x400000u                                       // image base of gc.exe
#define GC_LOGGER ((uint8_t *)0x4e13c0)                         // logger entry
#define GC_EXPAND_GUARD ((uint8_t *)0x82f4e8)                   // one-time guard of Expand
#define EXPAND_GUARD_BIT 1                                      // bit that marks the guard done
#define GC_STATIC_INITIALIZER ((void (*)(void))0x4e8130)        // sets the inverse cell size

#define DOS_NT_HEADER_OFFSET 0x3c                               // PE: offset of the NT headers
#define NT_SECTION_COUNT 6                                      // PE: number of sections
#define NT_OPTIONAL_HEADER_SIZE 20                              // PE: optional header size
#define NT_OPTIONAL_HEADER 24                                   // PE: optional header start
#define OPTIONAL_IMAGE_BASE 28                                  // PE: image base
#define OPTIONAL_IMAGE_SIZE 56                                  // PE: size of the image
#define OPTIONAL_HEADERS_SIZE 60                                // PE: size of all headers
#define SECTION_HEADER_SIZE 40                                  // PE: size of a section header
#define SECTION_VIRTUAL_ADDRESS 12                              // PE: relative section address
#define SECTION_RAW_SIZE 16                                     // PE: size of the section data
#define SECTION_RAW_OFFSET 20                                   // PE: file offset of the data

#define X86_JMP_REL32 0xe9                                      // opcode of a relative jump
#define X86_JMP_OFFSET_INDEX 1                                  // byte of its 32-bit offset
#define X86_JMP_REL32_SIZE 5                                    // length of the jump
#define X86_RET 0xc3                                            // opcode of a return

#define GAME_IMAGE_SPACE_BYTES 0xd00000                         // space for the gc.exe image

// Space for the image of gc.exe, which `map_image` checks to cover its base address.
static uint8_t game_image_space[GAME_IMAGE_SPACE_BYTES] __attribute__((aligned(4096)));


// `allocate_game_memory` replaces `operator new` of the game. A request for zero bytes gets one
// byte, because `operator new` must return a pointer that is not NULL.
static void *allocate_game_memory(unsigned byte_count) {
    return malloc(byte_count ? byte_count : 1);
}


// `free_game_memory` replaces `operator delete` of the game.
static void free_game_memory(void *pointer) {
    free(pointer);
}


// `write_jump` writes a relative jump to `target` at the code address `address`. The offset of
// the jump counts from the end of the jump instruction.
static void write_jump(uint32_t address, const void *target) {
    uint8_t *code = (uint8_t *)address;
    uint32_t next_instruction = address + X86_JMP_REL32_SIZE;
    int32_t offset = (int32_t)((uint32_t)target - next_instruction);
    code[0] = X86_JMP_REL32;
    *(int32_t *)&code[X86_JMP_OFFSET_INDEX] = offset;
}


// `read_u16` reads a 16-bit value at `offset` in `bytes`.
static uint16_t read_u16(const uint8_t *bytes, uint32_t offset) {
    return *(const uint16_t *)&bytes[offset];
}


// `read_u32` reads a 32-bit value at `offset` in `bytes`.
static uint32_t read_u32(const uint8_t *bytes, uint32_t offset) {
    return *(const uint32_t *)&bytes[offset];
}


// `get_image_address` returns the address of the relative address `relative_address` of the
// mapped gc.exe image.
static void *get_image_address(uint32_t relative_address) {
    return (void *)(GC_BASE + relative_address);
}


// `map_image` copies the headers and the sections of the gc.exe file in `file_bytes` to their
// addresses at the base address. It returns the image size, or 0 when the image does not have
// that base or does not fit into `game_image_space`.
static uint32_t map_image(const uint8_t *file_bytes) {
    // The NT headers start at the offset that the DOS header holds.
    uint32_t nt_offset = read_u32(file_bytes, DOS_NT_HEADER_OFFSET);
    const uint8_t *nt_header = &file_bytes[nt_offset];
    uint16_t section_count = read_u16(nt_header, NT_SECTION_COUNT);
    uint16_t optional_size = read_u16(nt_header, NT_OPTIONAL_HEADER_SIZE);
    const uint8_t *optional_header = &nt_header[NT_OPTIONAL_HEADER];
    uint32_t image_base = read_u32(optional_header, OPTIONAL_IMAGE_BASE);
    uint32_t image_size = read_u32(optional_header, OPTIONAL_IMAGE_SIZE);
    uint32_t headers_size = read_u32(optional_header, OPTIONAL_HEADERS_SIZE);

    // The image must fit at its own base, because its code holds absolute addresses.
    uint32_t reserved_start = (uint32_t)game_image_space;
    uint32_t reserved_end = reserved_start + sizeof game_image_space;
    bool is_at_base = image_base == GC_BASE;
    bool is_covered = reserved_start <= GC_BASE && reserved_end >= GC_BASE + image_size;
    if (!is_at_base || !is_covered) {
        fprintf(stderr, "cannot map gc.exe: base %08x size %08x, reserved %08x..%08x\n",
                image_base, image_size, reserved_start, reserved_end);
        return 0;
    }

    // The space becomes executable and writable, so that the code runs and can be patched.
    DWORD old_protection;
    void *image_start = get_image_address(0);
    VirtualProtect(image_start, image_size, PAGE_EXECUTE_READWRITE, &old_protection);
    memcpy(image_start, file_bytes, headers_size);

    // The section table follows the optional header.
    const uint8_t *section_table = &optional_header[optional_size];
    for (int section_index = 0; section_index < section_count; section_index++) {
        const uint8_t *section = &section_table[section_index * SECTION_HEADER_SIZE];
        uint32_t virtual_address = read_u32(section, SECTION_VIRTUAL_ADDRESS);
        uint32_t raw_size = read_u32(section, SECTION_RAW_SIZE);
        uint32_t raw_offset = read_u32(section, SECTION_RAW_OFFSET);
        memcpy(get_image_address(virtual_address), &file_bytes[raw_offset], raw_size);
    }
    return image_size;
}


// `gc_load` maps gc.exe from `exe_path` and prepares its code for calls from the simulator. It
// returns false when the file cannot be read or mapped.
bool gc_load(const char *exe_path) {
    FILE *file = fopen(exe_path, "rb");
    if (!file) {
        fprintf(stderr, "cannot open %s\n", exe_path);
        return false;
    }
    fseek(file, 0, SEEK_END);
    long size = ftell(file);        // file size
    fseek(file, 0, SEEK_SET);
    uint8_t *bytes = malloc(size);
    if (fread(bytes, 1, size, file) != (size_t)size) {
        fclose(file);
        return false;
    }
    fclose(file);

    uint32_t image_size = map_image(bytes);
    free(bytes);
    if (image_size == 0)
        return false;

    // The patches and the initializer come before the first call into the game.
    write_jump(GC_OPERATOR_NEW, (const void *)allocate_game_memory);
    write_jump(GC_OPERATOR_DELETE, (const void *)free_game_memory);
    *GC_LOGGER = X86_RET;
    *GC_EXPAND_GUARD |= EXPAND_GUARD_BIT;
    GC_STATIC_INITIALIZER();
    FlushInstructionCache(GetCurrentProcess(), get_image_address(0), image_size);
    return true;
}
