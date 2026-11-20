// Code hooks in gc.exe. gc.exe has no relocations, so its functions sit at fixed addresses
// (docs/PATHFINDING.md). A hook overwrites the first instructions of a game function with a jump to
// the hook, and copies them into a trampoline that runs them and jumps back to the rest of the
// function, so the hook can still call the original:
//
//   game function:  jmp hook | nop ... | rest of the function
//   trampoline:     first instructions of the function | jmp rest of the function
//
// The copied instructions must not depend on their address (no relative jumps or calls), and
// `stolen_length` must end on an instruction boundary; the caller checks both in a disassembly.
#include <stdint.h>
#include <string.h>
#include <windows.h>

#include "mod.h"

#define JUMP_OPCODE 0xe9                  // x86 jump with a 32-bit offset from the next instruction
#define JUMP_SIZE 5                       // size of that jump
#define NOP_OPCODE 0x90                   // x86 instruction that does nothing
#define MAX_STOLEN_BYTES 16               // most instruction bytes that a hook moves
#define TRAMPOLINE_BYTES (MAX_STOLEN_BYTES + JUMP_SIZE)


// `write_jump` writes a jump to `target` at `code`.
static void write_jump(uint8_t *code, const void *target) {
    uintptr_t next_instruction = (uintptr_t)&code[JUMP_SIZE];
    int32_t offset = (int32_t)((uintptr_t)target - next_instruction);
    code[0] = JUMP_OPCODE;
    memcpy(&code[1], &offset, sizeof offset);
}


// `install_hook` builds the trampoline, then patches the game function to jump to `hook`.
void *install_hook(uintptr_t address, int stolen_length, const void *hook) {
    if (stolen_length < JUMP_SIZE || stolen_length > MAX_STOLEN_BYTES)
        return NULL;
    uint8_t *function_code = (uint8_t *)address;
    DWORD allocation_type = MEM_COMMIT | MEM_RESERVE;
    uint8_t *trampoline = VirtualAlloc(NULL, TRAMPOLINE_BYTES, allocation_type,
                                       PAGE_EXECUTE_READWRITE);
    if (!trampoline)
        return NULL;
    memcpy(trampoline, function_code, (size_t)stolen_length);
    write_jump(&trampoline[stolen_length], &function_code[stolen_length]);

    // The code pages of gc.exe are read-only, so the patch opens them for writing first.
    DWORD old_protection;
    if (!VirtualProtect(function_code, (SIZE_T)stolen_length, PAGE_EXECUTE_READWRITE,
                        &old_protection))
        return NULL;
    write_jump(function_code, hook);
    memset(&function_code[JUMP_SIZE], NOP_OPCODE, (size_t)(stolen_length - JUMP_SIZE));
    VirtualProtect(function_code, (SIZE_T)stolen_length, old_protection, &old_protection);
    FlushInstructionCache(GetCurrentProcess(), function_code, (SIZE_T)stolen_length);
    return trampoline;
}
