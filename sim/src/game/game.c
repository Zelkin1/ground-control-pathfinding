/* game.c maps the real gc.exe 1.0.1.1 into this process and wraps the game functions that the
 * simulator calls, which game.h documents.
 *
 * Addresses:
 *   gc.exe has no relocations and always loads at 0x400000, so each address below is a fixed
 *   address of a function or a variable in that exe. They come from disassembly and the
 *   decompile, and re/PATHFINDING.md describes each function.
 *
 * Start-up:
 *   The exe runs without its start-up code. This file copies its headers and sections to their
 *   addresses, but it does not connect the imports of the exe to Windows DLLs and does not start
 *   the C runtime of the exe. `gc_load` then patches the few things that the search and the
 *   formation layout need from that start-up.
 *
 * Calls:
 *   The member functions of the game take `this` in the ECX register (thiscall), so the
 *   function pointer types below use that convention. */
#include "game.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

/* The game is MSVC6 C++, and its member functions take `this` in the ECX register. */
#define THISCALL __attribute__((thiscall))

#define GC_BASE 0x400000u                 /* The image of gc.exe always starts at this address. */
#define GC_PATHFINDER ((void *)0x6eb470)  /* This is the one global PathFinder of the game. */
#define PATHFINDER_LAST_RAY_SIDE 0x00     /* The last ray side is at this PathFinder offset. */
#define PATHFINDER_GRID 0x0c              /* The grid starts at this PathFinder offset. */
#define GC_OPERATOR_NEW 0x5e89be          /* `operator new` of the game starts at this address. */
#define GC_OPERATOR_DELETE 0x5e89e7       /* `operator delete` of the game is at this address. */
#define GC_LOGGER ((uint8_t *)0x4e13c0)   /* The debug logger of the game is at this address. */
#define GC_EXPAND_GUARD ((uint8_t *)0x82f4e8) /* This byte guards the `atexit` call of Expand. */
#define EXPAND_GUARD_BIT 1                /* Expand sets this bit when it calls `atexit`. */
#define GC_STATIC_INITIALIZER ((void (*)(void))0x4e8130) /* It sets the inverse cell size. */
#define GC_INVERSE_CELL_SIZE ((const float *)0x82f4ec)   /* This float holds 1 / cell size. */

/* These are the byte offsets of the PE header fields that `map_image` reads. PE is the file
 * format of Windows executables. `DOS_` offsets count from the start of the file, `NT_` offsets
 * from the NT headers, `OPTIONAL_` offsets from the optional header and `SECTION_` offsets from a
 * section header. The NT headers start with the 4-byte signature "PE\0\0" and the 20-byte file
 * header, and the optional header follows them. */
#define DOS_NT_HEADER_OFFSET 0x3c  /* `e_lfanew` of the DOS header is the NT headers offset. */
#define NT_SECTION_COUNT 6         /* `NumberOfSections` is the number of section headers. */
#define NT_OPTIONAL_HEADER_SIZE 20 /* `SizeOfOptionalHeader` is the size of the optional header. */
#define NT_OPTIONAL_HEADER 24      /* The optional header starts at this offset. */
#define OPTIONAL_IMAGE_BASE 28     /* `ImageBase` is the address that the image is built for. */
#define OPTIONAL_IMAGE_SIZE 56     /* `SizeOfImage` is the size of the image in memory. */
#define OPTIONAL_HEADERS_SIZE 60   /* `SizeOfHeaders` is the size of all headers in the file. */
#define SECTION_HEADER_SIZE 40     /* Each section header takes this many bytes. */
#define SECTION_VIRTUAL_ADDRESS 12 /* `VirtualAddress` is the offset of the section. */
#define SECTION_RAW_SIZE 16        /* `SizeOfRawData` is the size of the file data. */
#define SECTION_RAW_OFFSET 20      /* `PointerToRawData` is the file offset of that data. */

#define X86_JMP_REL32 0xe9     /* This opcode jumps by the 32-bit offset that follows it. */
#define X86_JMP_OFFSET_INDEX 1 /* The offset of the jump starts at this byte of the instruction. */
#define X86_JMP_REL32_SIZE 5   /* The jump instruction takes this many bytes. */
#define X86_RET 0xc3           /* This opcode returns and leaves the arguments on the stack. */

#define CELLS_PER_BYTE 4                               /* Each grid byte holds this many cells. */
#define BITS_PER_CELL 2                                /* Each cell takes this many bits. */
#define GRID_ROW_BYTES (GC_GRID_SIZE / CELLS_PER_BYTE) /* A grid row takes this many bytes. */

/* These are byte offsets of PathRequest fields, from the decompile of PathFinder::Update. */
#define REQUEST_IS_FOUND 0x38     /* This byte is set when the search reached its target. */
#define REQUEST_IS_SEARCHING 0x3a /* This byte is set when the search can go on. */

/* `GcNode` and `GcList` are the doubly linked list of points of the game, which GetPath and
 * CalculateFormationPoints fill. A node takes 0x10 bytes. */
typedef struct GcNode {
    float x;
    float y;
    struct GcNode *next;
    struct GcNode *previous;
} GcNode;

typedef struct {
    GcNode *head;
    GcNode *iterator;
    GcNode *tail;
    int count;
} GcList;

/* `EMPTY_LIST` is a list with no nodes, for a game function to fill. */
static const GcList EMPTY_LIST = {
    .head = NULL,
    .iterator = NULL,
    .tail = NULL,
    .count = 0,
};

/* These are the types of the game functions below. CalculateFormationPoints is cdecl, and the
 * others are member functions. */
typedef void(THISCALL *RequestFunction)(GcRequest *);
typedef void(THISCALL *StartFunction)(GcRequest *, float, float, float, float, int, float, float);
typedef int(THISCALL *UpdateFunction)(void *, GcRequest *, int);
typedef void(THISCALL *GetPathFunction)(GcRequest *, GcList *);
typedef int(THISCALL *RaycastFunction)(void *, float, float, float, float, int);
typedef char (*FormationFunction)(float, float, float, float, GcList *, int, float, int, int);

/* These are the game functions that game.h wraps. The first two are the constructor and the
 * destructor of PathRequest, and re/PATHFINDING.md describes the others under these names. */
#define REQUEST_CONSTRUCTOR ((RequestFunction)0x4eb800) /* PathRequest constructor. */
#define REQUEST_DESTRUCTOR ((RequestFunction)0x4eb820)  /* PathRequest destructor: frees memory. */
#define START_SEARCH_QUIET ((StartFunction)0x4eb8d0)    /* StartSearchQuiet: forward search. */
#define START_SEARCH ((StartFunction)0x4eb9a0)          /* StartSearch: backward search. */
#define PATHFINDER_UPDATE ((UpdateFunction)0x4e8760)    /* PathFinder::Update: runs expansions. */
#define GET_PATH ((GetPathFunction)0x4ebbb0)            /* GetPath: lists a found route. */
#define RAYCAST ((RaycastFunction)0x4e8400)             /* Raycast: casts a ray in the grid. */
/* SquadFormation::CalculateFormationPoints lays out the slots of a squad. */
#define CALCULATE_FORMATION_POINTS ((FormationFunction)0x4e7be0)

/* The image of the game must sit at 0x400000..0xc52000. This program is linked at 0x10000, and
 * `game_image_space` is a zero-filled array in its .bss that starts below 0x400000.
 * `map_image` checks that the array covers the image. Large static arrays elsewhere can move the
 * start of the array past 0x400000, so the rest of the simulator keeps them on the heap. */
#define GAME_IMAGE_SPACE_BYTES 0xd00000 /* This size makes the array reach past 0xc52000. */
static uint8_t game_image_space[GAME_IMAGE_SPACE_BYTES] __attribute__((aligned(4096)));

/* ------------------------------------------------------------------------------------------------
 * `allocate_game_memory` takes the place of `operator new` of the game. The code of the game
 * jumps into it, so it is cdecl like the original function.
 * `operator new` must return a usable pointer even for 0 bytes, but `malloc(0)` can return NULL,
 * so 0 bytes become 1.
 * --------------------------------------------------------------------------------------------- */
static void *allocate_game_memory(unsigned byte_count) {
    return malloc(byte_count ? byte_count : 1);
}

/* ------------------------------------------------------------------------------------------------
 * `free_game_memory` takes the place of `operator delete` of the game. The code of the game
 * jumps into it, so it is cdecl like the original function.
 * --------------------------------------------------------------------------------------------- */
static void free_game_memory(void *pointer) {
    free(pointer);
}

/* ------------------------------------------------------------------------------------------------
 * `write_jump` writes a jump to `target` at `address` in the code of the game. The offset of
 * the jump counts from the end of the jump instruction.
 * --------------------------------------------------------------------------------------------- */
static void write_jump(uint32_t address, const void *target) {
    uint8_t *code = (uint8_t *)address;
    uint32_t next_instruction = address + X86_JMP_REL32_SIZE;
    int32_t jump_offset = (int32_t)((uint32_t)target - next_instruction);
    code[0] = X86_JMP_REL32;
    *(int32_t *)&code[X86_JMP_OFFSET_INDEX] = jump_offset;
}

/* ------------------------------------------------------------------------------------------------
 * `read_u16` reads the 16-bit little-endian value at `offset` bytes from `bytes`. The x86 CPU
 * allows reads at any alignment.
 * --------------------------------------------------------------------------------------------- */
static uint16_t read_u16(const uint8_t *bytes, uint32_t offset) {
    return *(const uint16_t *)&bytes[offset];
}

/* ------------------------------------------------------------------------------------------------
 * `read_u32` reads the 32-bit little-endian value at `offset` bytes from `bytes`.
 * --------------------------------------------------------------------------------------------- */
static uint32_t read_u32(const uint8_t *bytes, uint32_t offset) {
    return *(const uint32_t *)&bytes[offset];
}

/* ------------------------------------------------------------------------------------------------
 * `get_image_address` returns where the byte at `relative_address` of the mapped image is. PE
 * headers give positions in the image relative to its start, `GC_BASE`.
 * --------------------------------------------------------------------------------------------- */
static void *get_image_address(uint32_t relative_address) {
    return (void *)(GC_BASE + relative_address);
}

/* ------------------------------------------------------------------------------------------------
 * `get_pathfinder_field` returns the address of the field of PathFinder at `offset` bytes from
 * its start. re/PATHFINDING.md lists the fields.
 * --------------------------------------------------------------------------------------------- */
static uint8_t *get_pathfinder_field(uint32_t offset) {
    uint8_t *pathfinder = GC_PATHFINDER;
    return &pathfinder[offset];
}

/* ------------------------------------------------------------------------------------------------
 * `get_game_grid` returns the passability grid of the game. The grid is `GC_GRID_SIZE` rows of
 * `GRID_ROW_BYTES` bytes, row 0 first. Each byte holds 4 cells, and the cell with the lowest
 * `cell_x` uses the highest 2 bits.
 * --------------------------------------------------------------------------------------------- */
static uint8_t *get_game_grid(void) {
    return get_pathfinder_field(PATHFINDER_GRID);
}

/* `thread_grid` points to the grid copy that the cell reads of this thread use, or is NULL while
 * they use the grid of the game. Each thread has its own (`_Thread_local`). */
static _Thread_local uint8_t *thread_grid;

/* ------------------------------------------------------------------------------------------------
 * `get_grid` returns the grid that the cells of this thread are read from and written to: the grid
 * copy of `gc_use_grid`, or the grid of the game. Only the main thread writes cells, and it always
 * uses the grid of the game.
 * --------------------------------------------------------------------------------------------- */
static uint8_t *get_grid(void) {
    if (thread_grid)
        return thread_grid;
    return get_game_grid();
}

/* ------------------------------------------------------------------------------------------------
 * `map_image` copies the headers and sections of gc.exe from `file_bytes`, the whole file, to
 * their addresses in the image, and makes the image executable.
 * Result: It returns the image size.
 * Edge case: It returns 0 after an error message when the image is not built for `GC_BASE` or
 *   does not fit into `game_image_space`.
 * --------------------------------------------------------------------------------------------- */
static uint32_t map_image(const uint8_t *file_bytes) {
    /* Read the PE headers. */
    uint32_t nt_header_offset = read_u32(file_bytes, DOS_NT_HEADER_OFFSET);
    const uint8_t *nt_header = &file_bytes[nt_header_offset];
    uint16_t section_count = read_u16(nt_header, NT_SECTION_COUNT);
    uint16_t optional_header_size = read_u16(nt_header, NT_OPTIONAL_HEADER_SIZE);
    const uint8_t *optional_header = &nt_header[NT_OPTIONAL_HEADER];
    uint32_t image_base = read_u32(optional_header, OPTIONAL_IMAGE_BASE);
    uint32_t image_size = read_u32(optional_header, OPTIONAL_IMAGE_SIZE);
    uint32_t headers_size = read_u32(optional_header, OPTIONAL_HEADERS_SIZE);

    /* Make sure that the image is built for `GC_BASE` and that `game_image_space` covers it. */
    uint32_t reserved_start = (uint32_t)game_image_space;
    uint32_t reserved_end = reserved_start + sizeof game_image_space;
    bool is_at_base = image_base == GC_BASE;
    bool is_covered = reserved_start <= GC_BASE && reserved_end >= GC_BASE + image_size;
    if (!is_at_base || !is_covered) {
        fprintf(stderr, "cannot map gc.exe: base %08x size %08x, reserved %08x..%08x\n",
                image_base, image_size, reserved_start, reserved_end);
        return 0;
    }

    /* Make the image executable, because the code of the game runs from it, and copy the
     * headers. */
    DWORD old_protection;
    void *image_start = get_image_address(0);
    VirtualProtect(image_start, image_size, PAGE_EXECUTE_READWRITE, &old_protection);
    memcpy(image_start, file_bytes, headers_size);

    /* Copy the file data of each section. The section headers follow the optional header as an
     * array of `SECTION_HEADER_SIZE`-byte records. The rest of a section, past its file data,
     * stays zero as the Windows loader leaves it, because `game_image_space` starts
     * zero-filled. */
    const uint8_t *section_table = &optional_header[optional_header_size];
    for (int section_index = 0; section_index < section_count; section_index++) {
        const uint8_t *section = &section_table[section_index * SECTION_HEADER_SIZE];
        uint32_t virtual_address = read_u32(section, SECTION_VIRTUAL_ADDRESS);
        uint32_t raw_size = read_u32(section, SECTION_RAW_SIZE);
        uint32_t raw_offset = read_u32(section, SECTION_RAW_OFFSET);
        memcpy(get_image_address(virtual_address), &file_bytes[raw_offset], raw_size);
    }
    return image_size;
}

/* ------------------------------------------------------------------------------------------------
 * `gc_load` reads the whole exe into memory, maps it with `map_image`, and then patches the
 * start-up work that the game needs but that does not run here. The steps below explain each
 * patch.
 * --------------------------------------------------------------------------------------------- */
bool gc_load(const char *exe_path) {
    /* Read the whole file. */
    FILE *file = fopen(exe_path, "rb");
    if (!file) {
        fprintf(stderr, "cannot open %s\n", exe_path);
        return false;
    }
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    uint8_t *file_bytes = malloc(file_size);
    if (fread(file_bytes, 1, file_size, file) != (size_t)file_size) {
        fclose(file);
        return false;
    }
    fclose(file);

    /* Map the image. */
    uint32_t image_size = map_image(file_bytes);
    free(file_bytes);
    if (image_size == 0)
        return false;

    /* Send `operator new` and `operator delete` of the game to the C library, because the heap
     * of the game needs its C runtime. */
    write_jump(GC_OPERATOR_NEW, (const void *)allocate_game_memory);
    write_jump(GC_OPERATOR_DELETE, (const void *)free_game_memory);

    /* Make the debug logger return at once. CalculateFormationPoints calls it when it places a
     * different number of slots than asked. The logger writes into the log object of the game,
     * which the start-up code of the game creates (0x4e0b30) and stores at 0x6eb41c, so the
     * object does not exist here. The logger is cdecl, so its callers remove the arguments, and
     * a lone `ret` is enough. */
    *GC_LOGGER = X86_RET;

    /* Expand (0x4e8920) registers a function with `atexit` of the game (0x53eeeb) the first time
     * it runs, and sets `EXPAND_GUARD_BIT` so that it does this only once. Set the bit in
     * advance, so that Expand never calls into the C runtime of the game. */
    *GC_EXPAND_GUARD |= EXPAND_GUARD_BIT;

    /* Run the static initializer, as the game does at start-up. It sets the inverse cell size at
     * 0x82f4ec (1 / 6.25) that the raycast, the cell lookup and the search read. Then flush the
     * instruction cache of the processor, because this function wrote the code of the game. */
    GC_STATIC_INITIALIZER();
    FlushInstructionCache(GetCurrentProcess(), get_image_address(0), image_size);
    return true;
}

/* ------------------------------------------------------------------------------------------------
 * `gc_is_outside_grid` compares both cell coordinates with the bounds of the grid.
 * --------------------------------------------------------------------------------------------- */
bool gc_is_outside_grid(int cell_x, int cell_y) {
    bool is_negative = cell_x < 0 || cell_y < 0;
    bool is_too_large = cell_x >= GC_GRID_SIZE || cell_y >= GC_GRID_SIZE;
    return is_negative || is_too_large;
}

/* ------------------------------------------------------------------------------------------------
 * `get_cell_shift` returns how far the 2 bits of cell `cell_x` are shifted up in their byte.
 * --------------------------------------------------------------------------------------------- */
static int get_cell_shift(int cell_x) {
    int index_in_byte = cell_x % CELLS_PER_BYTE;
    return (CELLS_PER_BYTE - 1 - index_in_byte) * BITS_PER_CELL;
}

/* ------------------------------------------------------------------------------------------------
 * `get_cell_byte` returns the grid byte that holds cell (`cell_x`, `cell_y`).
 * --------------------------------------------------------------------------------------------- */
static uint8_t *get_cell_byte(int cell_x, int cell_y) {
    uint8_t *grid = get_grid();
    int byte_index = cell_y * GRID_ROW_BYTES + cell_x / CELLS_PER_BYTE;
    return &grid[byte_index];
}

/* ------------------------------------------------------------------------------------------------
 * `gc_set_cell` writes into the grid of the game directly, with a read-modify-write of the byte
 * that holds the cell.
 * --------------------------------------------------------------------------------------------- */
void gc_set_cell(int cell_x, int cell_y, int value) {
    if (gc_is_outside_grid(cell_x, cell_y))
        return;

    /* Replace the 2 bits of the cell and keep the other cells of its byte. */
    uint8_t *cell_byte = get_cell_byte(cell_x, cell_y);
    int shift = get_cell_shift(cell_x);
    int cell_mask = GC_CELL_VALUE_MASK << shift;
    int other_cells = *cell_byte & ~cell_mask;
    int new_bits = (value & GC_CELL_VALUE_MASK) << shift;
    *cell_byte = (uint8_t)(other_cells | new_bits);
}

/* ------------------------------------------------------------------------------------------------
 * `gc_get_cell` reads the grid of the game directly: it shifts the 2 bits of the cell down out
 * of their byte.
 * --------------------------------------------------------------------------------------------- */
int gc_get_cell(int cell_x, int cell_y) {
    if (gc_is_outside_grid(cell_x, cell_y))
        return GC_CELL_CLIFF;
    uint8_t cell_byte = *get_cell_byte(cell_x, cell_y);
    int shift = get_cell_shift(cell_x);
    return (cell_byte >> shift) & GC_CELL_VALUE_MASK;
}

/* ------------------------------------------------------------------------------------------------
 * `gc_clear_grid` zeroes all bytes of the grid of the game.
 * --------------------------------------------------------------------------------------------- */
void gc_clear_grid(void) {
    memset(get_game_grid(), 0, GC_GRID_BYTES);
}

/* ------------------------------------------------------------------------------------------------
 * `gc_load_slp` does the same as PathFinder::LoadGrid of the game (0x4e8200). It copies the rows
 * in reverse order, so the last row of the file becomes grid row 0.
 * --------------------------------------------------------------------------------------------- */
void gc_load_slp(const uint8_t *slp_bytes) {
    uint8_t *grid = get_game_grid();
    for (int row_index = 0; row_index < GC_GRID_SIZE; row_index++) {
        int file_row_index = GC_GRID_SIZE - 1 - row_index;
        uint8_t *grid_row = &grid[row_index * GRID_ROW_BYTES];
        const uint8_t *file_row = &slp_bytes[file_row_index * GRID_ROW_BYTES];
        memcpy(grid_row, file_row, GRID_ROW_BYTES);
    }
}

/* ------------------------------------------------------------------------------------------------
 * `gc_copy_grid` copies the grid bytes as they lie in the game.
 * --------------------------------------------------------------------------------------------- */
void gc_copy_grid(uint8_t *grid_copy) {
    memcpy(grid_copy, get_game_grid(), GC_GRID_BYTES);
}

/* ------------------------------------------------------------------------------------------------
 * `gc_use_grid` sets `thread_grid`. The copy is only read, but `get_grid` also serves
 * `gc_set_cell`, so `thread_grid` drops the const.
 * --------------------------------------------------------------------------------------------- */
void gc_use_grid(const uint8_t *grid_copy) {
    thread_grid = (uint8_t *)grid_copy;
}

/* ------------------------------------------------------------------------------------------------
 * The constructor of the game clears only the lists of the request. So `gc_init_request` first
 * zeroes the whole request, so that its flags, such as `REQUEST_IS_FOUND`, start at 0.
 * --------------------------------------------------------------------------------------------- */
void gc_init_request(GcRequest *request) {
    memset(request, 0, sizeof *request);
    REQUEST_CONSTRUCTOR(request);
}

/* ------------------------------------------------------------------------------------------------
 * `gc_free_request` calls the destructor of PathRequest, which frees the lists of the request.
 * --------------------------------------------------------------------------------------------- */
void gc_free_request(GcRequest *request) {
    REQUEST_DESTRUCTOR(request);
}

/* ------------------------------------------------------------------------------------------------
 * The game has one function for each search direction, with the same arguments. StartSearchQuiet
 * is the forward search and StartSearch is the backward one, so `gc_start_search` picks one of
 * them.
 * --------------------------------------------------------------------------------------------- */
void gc_start_search(GcRequest *request, bool is_quiet, float from_x, float from_y, float to_x,
                     float to_y, int mover_class, float clearance, float tolerance) {
    StartFunction start_search = is_quiet ? START_SEARCH_QUIET : START_SEARCH;
    start_search(request, from_x, from_y, to_x, to_y, mover_class, clearance, tolerance);
}

/* ------------------------------------------------------------------------------------------------
 * Each expansion takes the best point that the search found but did not expand yet. It adds the
 * target when a straight line reaches it, or else corner points around the obstacle in the way.
 * `gc_update_search` ignores the return value of PathFinder::Update. It reads the state of the
 * search from two flag bytes of the request: `REQUEST_IS_FOUND` means found, and otherwise
 * `REQUEST_IS_SEARCHING` means that the search can go on. The limit of 500 expansions in total
 * is part of the game code. TODO: confirm which flag it clears when it ends the search.
 * --------------------------------------------------------------------------------------------- */
int gc_update_search(GcRequest *request, int expansion_budget) {
    PATHFINDER_UPDATE(GC_PATHFINDER, request, expansion_budget);
    if (request->bytes[REQUEST_IS_FOUND])
        return GC_SEARCH_FOUND;
    return request->bytes[REQUEST_IS_SEARCHING] ? GC_SEARCH_RUNNING : GC_SEARCH_FAILED;
}

/* ------------------------------------------------------------------------------------------------
 * `copy_and_free_list` copies the points of a game list into `points`, as x, y pairs, and frees
 * every node of the list. The nodes come from `operator new` of the game, which `gc_load` sends
 * to `allocate_game_memory`, so `free` releases them.
 * Result: It returns how many points it copied.
 * Edge case: It copies only the first `max_points` points, but frees the other nodes too.
 * --------------------------------------------------------------------------------------------- */
static int copy_and_free_list(const GcList *list, float *points, int max_points) {
    int point_count = 0;
    GcNode *node = list->head;
    while (node) {
        GcNode *next = node->next;
        if (point_count < max_points) {
            int x_index = 2 * point_count;
            points[x_index] = node->x;
            points[x_index + 1] = node->y;
            point_count++;
        }
        free(node);
        node = next;
    }
    return point_count;
}

/* ------------------------------------------------------------------------------------------------
 * GetPath fills a list of nodes, so `gc_get_path` gives it an empty list and then copies the
 * points out with `copy_and_free_list`.
 * --------------------------------------------------------------------------------------------- */
int gc_get_path(GcRequest *request, float *points, int max_points) {
    GcList list = EMPTY_LIST;
    GET_PATH(request, &list);
    return copy_and_free_list(&list, points, max_points);
}

/* ------------------------------------------------------------------------------------------------
 * `gc_cast_ray` calls Raycast of the game on the global PathFinder, which owns the grid.
 * --------------------------------------------------------------------------------------------- */
int gc_cast_ray(float from_x, float from_y, float to_x, float to_y, int mover_class) {
    return RAYCAST(GC_PATHFINDER, from_x, from_y, to_x, to_y, mover_class);
}

/* ------------------------------------------------------------------------------------------------
 * Raycast stores the side of its last step in the first byte of the PathFinder, at
 * `PATHFINDER_LAST_RAY_SIDE`. `gc_get_last_ray_side` reads that byte.
 * --------------------------------------------------------------------------------------------- */
int gc_get_last_ray_side(void) {
    return *get_pathfinder_field(PATHFINDER_LAST_RAY_SIDE);
}

/* ------------------------------------------------------------------------------------------------
 * `gc_get_cell_at` does in C what CellAt of the game (0x4e82e0) does, without calling it. CellAt
 * is slow: it converts each coordinate with `__ftol` of the game (0x53f8d0), which reloads the x87
 * control word twice per call, and the route set-up of formation following reads millions of
 * cells.
 * CellAt returns `GC_CELL_OUTSIDE` unless both coordinates lie strictly between 0 and the width
 * of the grid. Inside, it multiplies each coordinate by the inverse cell size on the x87 unit,
 * truncates the product, and reads the 2 bits of that cell. The product of two floats is exact
 * in a double, as it is on the x87 unit, so truncating the double product gives the same cell. A
 * check of both functions over 22 million points, many of them a few float steps from cell
 * edges, found the same value everywhere.
 * --------------------------------------------------------------------------------------------- */
int gc_get_cell_at(float x, float y) {
    int cell_x;
    int cell_y;
    return gc_read_cell_at(x, y, &cell_x, &cell_y);
}

/* ------------------------------------------------------------------------------------------------
 * `gc_read_cell_at` does the work of `gc_get_cell_at`. Inside the grid, the truncated products
 * lie from 0 to `GC_GRID_SIZE - 1`, because both coordinates are below the width of the grid and
 * the inverse cell size of the game is a little below 1 / 6.25.
 * --------------------------------------------------------------------------------------------- */
int gc_read_cell_at(float x, float y, int *cell_x, int *cell_y) {
    float grid_meters = GC_GRID_SIZE * GC_CELL_METERS;
    bool is_x_inside = x > 0 && x < grid_meters;
    bool is_y_inside = y > 0 && y < grid_meters;
    if (!is_x_inside || !is_y_inside)
        return GC_CELL_OUTSIDE;
    double inverse_cell_size = *GC_INVERSE_CELL_SIZE;
    *cell_x = (int)(x * inverse_cell_size);
    *cell_y = (int)(y * inverse_cell_size);
    uint8_t cell_byte = *get_cell_byte(*cell_x, *cell_y);
    return (cell_byte >> get_cell_shift(*cell_x)) & GC_CELL_VALUE_MASK;
}

/* ------------------------------------------------------------------------------------------------
 * The game fills a list of nodes instead of an array. `gc_place_formation` copies the list into
 * `slots` and frees it, whatever the result of the game, so that no node leaks. The count check
 * catches a game that placed fewer slots or more slots than `slot_count`, even when it reports
 * success.
 * --------------------------------------------------------------------------------------------- */
bool gc_place_formation(float anchor_x, float anchor_y, float direction_x, float direction_y,
                        int slot_count, float unit_radius, int mover_class, int formation_type,
                        float *slots) {
    GcList list = EMPTY_LIST;
    bool is_placed = CALCULATE_FORMATION_POINTS(anchor_x, anchor_y, direction_x, direction_y,
                                                &list, slot_count, unit_radius, mover_class,
                                                formation_type);
    int placed_count = copy_and_free_list(&list, slots, slot_count);
    return is_placed && placed_count == slot_count;
}
