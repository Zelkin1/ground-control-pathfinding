"""This tool writes a copy of gc.exe with the parameter patches of PathFix phase 1.

PathFix is the mod that changes the path search of the game, and its phase 1 is a set of
parameter patches. Each patch replaces a constant inside an instruction of the code of gc.exe
1.0.1.1, the Steam "Anthology" version. The tool never writes to its input. Before it changes
anything, it checks that every patch site holds the original bytes or the patched bytes, so it
refuses an exe of another version instead of corrupting it.

Usage: `HELP_TEXT` holds the examples, and `python -I tools/pathfix/patch_gc.py --help` prints it.
"""
import argparse
import hashlib
import os
import struct
import sys
from pathlib import Path

# `parse_options` prints this text as the description of `--help`. A backslash at the end of a
# line joins it to the next line in the printed text.
HELP_TEXT = """\
PathFix phase 1: parameter patches for Ground Control 1.0.1.1 (Steam "Anthology" gc.exe).

Never modifies the input. Writes a patched copy (default: build/gc.exe next to this repo).
Every patch checks the original bytes first, so a different exe version is refused \
instead of corrupted.

  python -I tools/pathfix/patch_gc.py                      # default PathFix settings
  python -I tools/pathfix/patch_gc.py --list               # show what each patch does
  python -I tools/pathfix/patch_gc.py --heuristic 1.0      # optimal (non-greedy) A*
  python -I tools/pathfix/patch_gc.py --vanilla            # all original values \
(sanity check: output == input)

See docs/PATHFINDING.md for what each address is.
"""

TOOLS_DIR = Path(__file__).resolve().parent.parent  # This file is in the tools/pathfix folder.
PROJECT_ROOT = TOOLS_DIR.parent
# This is the game folder of a default Steam install. The tool only reads the input.
GAME_DIR = Path(r"C:\Program Files (x86)\Steam\steamapps\common\Ground Control Anthology")
DEFAULT_INPUT_PATH = GAME_DIR / "gc.exe"
DEFAULT_OUTPUT_PATH = PROJECT_ROOT / "build" / "gc.exe"

# `KNOWN_SHA256` is the hash of the Steam gc.exe 1.0.1.1, for which the patch sites below were
# found. A resolution changer changes 4 header bytes of gc.exe. Such a copy has another hash but
# the same code, so `main` only warns and relies on the byte checks.
KNOWN_SHA256 = "4f5db3501f7f1fd611c98fca00db66cfb690e04e4e810da8883575c3b83a2797"

# The .text section of gc.exe (its code) starts at this virtual address and this file offset, as
# its section table says. gc.exe always loads at its fixed base 0x400000 and has no relocations,
# so these values never change. `tools/re/gcdis.py` maps every section in the same way.
TEXT_VIRTUAL_ADDRESS = 0x401000  # This is the base 0x400000 plus the offset 0x1000 of the section.
TEXT_FILE_OFFSET = 0x200  # The bytes of the section start at this offset in the file.

# The path search of the game is A* (a best-first search). It ranks each point that it reaches by
# f = g + weight * h, where g is the length of the path to the point and h is the straight-line
# distance from the point to the target. The weight of the game, 1.5, makes the search greedy, so
# the route that it finds is not always the shortest. The heuristic patch points the `fmul` of the
# search at another float constant instead of changing the 1.5, because other code reads that 1.5
# too. Each address holds a 4-byte float constant with the value of its key, in the read-only
# data (.rdata) of gc.exe.
WEIGHT_CONSTANT_ADDRESSES = {
    1.0: 0x60D5D0,
    1.1: 0x6134B8,
    1.25: 0x6132C4,
    1.3: 0x6120F0,
    1.5: 0x6116C4,
}

# These are the patch sites in gc.exe 1.0.1.1 and the values that the game has there.
# tools/re/gcdis.py found them, and docs/PATHFINDING.md describes the functions. An expansion is one
# step of the path search: it takes the best point found so far and adds new points from it.
# Apart from the search cap and the heuristic, each site holds the expansion budget that a caller
# pushes as an argument of a path search. The budget is an immediate (a constant stored inside
# the instruction) after the one-byte opcode of `push imm8` (6A) or `push imm32` (68).
PUSH_OPCODE_SIZE = 1

# `PathFinder::Update` (0x4e8760) gives up on a search after 500 expansions in total. Two
# `cmp word ptr [edi + 0x3c], imm16` instructions compare the expansion counter at
# `PathRequest+0x3c` with that cap. Unlike at the other sites, these addresses are the starts of
# the instructions, and the patch bytes include the 4 bytes before the immediate, so the byte
# check also proves that the instruction is this compare. TODO: confirm why only this patch
# checks the bytes before the immediate.
SEARCH_CAP_ADDRESSES = (0x4E88CA, 0x4E88DA)
SEARCH_CAP_COMPARE = bytes.fromhex("66817f3c")  # Bytes of the compare before its imm16.
ORIGINAL_SEARCH_CAP = 500

# `Squad::CalculatingMoveOrderLogic` (0x50a6a0) runs up to this many expansions of the route
# search of a squad per tick, with a `push imm8`.
SQUAD_BUDGET_ADDRESS = 0x50A6A5
ORIGINAL_SQUAD_BUDGET = 60

# `Mover::SetMove` (0x50e7e0) gives this budget, with a `push imm32`, to the search of each member
# from the member to the route of its squad.
SETMOVE_BUDGET_ADDRESS = 0x50E969
ORIGINAL_SETMOVE_BUDGET = 200

# `Mover::Update` (0x50eec0) pushes these budgets for its searches with `Mover::FindPath`
# (0x50eb60), which re-path a unit, for example when its current waypoint is out of sight. Each
# entry is the address of a `push imm8` and the budget that it pushes.
UNIT_REPATH_SITES = ((0x50F044, 0x20), (0x50F052, 0x64), (0x50F6EE, 0x28), (0x50F7FA, 0x14),
                     (0x50F894, 0x14), (0x50F95F, 0x14), (0x50F9F6, 0x14), (0x50FA3D, 0x14),
                     (0x50FD90, 0x32))

# The first two sites are in `Squad::MoveDirect` (0x505200): its forward search and its backward
# retry each run up to 500 expansions in one call, not spread over ticks. All AI moves use
# `MoveDirect`, and so do queued orders, formation changes, scripts, attack moves and evasion.
# The third site is in 0x50cca0, which also runs up to 500 expansions in one call. Its only direct
# caller in the Ghidra decompile (tools/re/ghidra_decompile.py) is `SV_Session::ParseDropship`
# (0x4f4e50). TODO: confirm whether
# that third search is an AI query. Each site is a `push imm32`.
AI_BUDGET_ADDRESSES = (0x505465, 0x5054CE, 0x50CF31)
ORIGINAL_AI_BUDGET = 500

# `PathFinder::Expand` (0x4e8920) adds a new point in two places, for a clear line and for a
# blocked line that ends near the target. Both multiply the h of the point by the weight with an
# `fmul dword ptr [weight address]`. These addresses are the starts of the two instructions.
HEURISTIC_ADDRESSES = (0x4E8AC8, 0x4E8C1B)
FMUL_OPCODE_SIZE = 2  # The two bytes D8 0D come before the 4-byte address of the weight.
ORIGINAL_HEURISTIC = 1.5
VANILLA_UNIT_BUDGET = 0  # With 0, every re-path site keeps its own budget, which is larger.

# These are the default settings of PathFix.
DEFAULT_SEARCH_CAP = 3000
DEFAULT_SQUAD_BUDGET = 127
DEFAULT_SETMOVE_BUDGET = 1000
DEFAULT_UNIT_BUDGET = 127
DEFAULT_AI_BUDGET = 1500
DEFAULT_HEURISTIC = 1.25

MAX_IMM8 = 127  # A `push imm8` sign-extends its byte, so larger values turn negative.
MAX_IMM16 = 32767  # The expansion counter at `PathRequest+0x3c` is a signed 16-bit number.


def encode_imm8(value):
    """`encode_imm8` returns `value` as the one-byte immediate of a `push imm8`.

    Edge case: It stops the program when `value` is not in 1..`MAX_IMM8`.
    """
    if not 1 <= value <= MAX_IMM8:
        raise SystemExit(f"value {value} must be 1..{MAX_IMM8} (encoded as push imm8)")
    return bytes([value])


def encode_imm16(value):
    """`encode_imm16` returns `value` as a little-endian 16-bit immediate.

    Edge case: It stops the program when `value` is not in 1..`MAX_IMM16`, the range of the
      signed expansion counter.
    """
    if not 1 <= value <= MAX_IMM16:
        raise SystemExit(f"value {value} must be 1..{MAX_IMM16} (signed 16-bit counter)")
    return struct.pack("<H", value)


def encode_imm32(value):
    """`encode_imm32` returns `value` as a little-endian 32-bit immediate."""
    return struct.pack("<I", value)


def get_operand_address(instruction_address, opcode_size):
    """`get_operand_address` returns the address of the operand bytes of the instruction at
    `instruction_address`. They follow the `opcode_size` bytes of opcode of the instruction."""
    return instruction_address + opcode_size


def describe_expansions(expansion_count, original_count):
    """`describe_expansions` returns the part of a patch description that gives the new and the
    original number of expansions."""
    return f"{expansion_count} expansions (orig {original_count})"


def build_patches(args):
    """`build_patches` makes the list of patches for the settings in `args`.

    Inputs: `args` holds `search_cap`, `squad_budget`, `setmove_budget`, `unit_budget`,
      `ai_budget` and `heuristic`.
    Result: It returns one `(name, virtual address, original bytes, new bytes, description)`
      tuple for each patch site. The bytes start at the virtual address.
    Edge case: It stops the program when a setting does not fit its immediate, or when
      `heuristic` has no constant in `WEIGHT_CONSTANT_ADDRESSES`.
    """
    patches = []

    # Raise the hard cap on expansions per search at both compares, so that fewer moves end in
    # "Movement denied".
    original_compare = SEARCH_CAP_COMPARE + encode_imm16(ORIGINAL_SEARCH_CAP)
    new_compare = SEARCH_CAP_COMPARE + encode_imm16(args.search_cap)
    cap_change = describe_expansions(args.search_cap, ORIGINAL_SEARCH_CAP)
    description = f"PathFinder::Update: give up after {cap_change} -> fewer 'Movement denied'"
    for virtual_address in SEARCH_CAP_ADDRESSES:
        patch = ("search_cap", virtual_address, original_compare, new_compare, description)
        patches.append(patch)

    # Give the route search of a squad more expansions per tick, so that squads react sooner.
    operand_address = get_operand_address(SQUAD_BUDGET_ADDRESS, PUSH_OPCODE_SIZE)
    original_bytes = encode_imm8(ORIGINAL_SQUAD_BUDGET)
    new_bytes = encode_imm8(args.squad_budget)
    budget_change = f"{args.squad_budget} expansions per tick (orig {ORIGINAL_SQUAD_BUDGET})"
    description = f"Squad::CalculatingMoveOrderLogic: {budget_change} -> squads react sooner"
    patches.append(("squad_budget", operand_address, original_bytes, new_bytes, description))

    # Give the search of each member to the route of its squad more expansions. In the printed
    # description, "formation slot" is the final parking position of the member.
    operand_address = get_operand_address(SETMOVE_BUDGET_ADDRESS, PUSH_OPCODE_SIZE)
    original_bytes = encode_imm32(ORIGINAL_SETMOVE_BUDGET)
    new_bytes = encode_imm32(args.setmove_budget)
    budget_change = describe_expansions(args.setmove_budget, ORIGINAL_SETMOVE_BUDGET)
    description = f"Mover::SetMove: initial per-unit path to formation slot, {budget_change}"
    patch = ("unit_setmove_budget", operand_address, original_bytes, new_bytes, description)
    patches.append(patch)

    # Raise each re-path budget to `unit_budget`. A site keeps its own budget when that is larger.
    for instruction_address, original_budget in UNIT_REPATH_SITES:
        budget = max(original_budget, args.unit_budget)
        operand_address = get_operand_address(instruction_address, PUSH_OPCODE_SIZE)
        original_bytes = encode_imm8(original_budget)
        new_bytes = encode_imm8(budget)
        budget_change = describe_expansions(budget, original_budget)
        site_name = f"Mover::Update re-path when blocked @{instruction_address:#x}"
        description = f"{site_name}: {budget_change}"
        patch = ("unit_repath_budget", operand_address, original_bytes, new_bytes, description)
        patches.append(patch)

    # Give the searches that run in one call more expansions.
    original_bytes = encode_imm32(ORIGINAL_AI_BUDGET)
    new_bytes = encode_imm32(args.ai_budget)
    budget_change = describe_expansions(args.ai_budget, ORIGINAL_AI_BUDGET)
    for instruction_address in AI_BUDGET_ADDRESSES:
        operand_address = get_operand_address(instruction_address, PUSH_OPCODE_SIZE)
        description = f"AI path query @{instruction_address:#x}: {budget_change}"
        patches.append(("ai_budget", operand_address, original_bytes, new_bytes, description))

    # Point both `fmul` instructions of the search at the float constant for the new weight.
    if args.heuristic not in WEIGHT_CONSTANT_ADDRESSES:
        raise SystemExit(f"--heuristic must be one of {sorted(WEIGHT_CONSTANT_ADDRESSES)}")
    original_bytes = encode_imm32(WEIGHT_CONSTANT_ADDRESSES[ORIGINAL_HEURISTIC])
    new_bytes = encode_imm32(WEIGHT_CONSTANT_ADDRESSES[args.heuristic])
    weight_change = f"{args.heuristic} (orig {ORIGINAL_HEURISTIC}, greedy)"
    description = f"PathFinder::Expand: heuristic weight {weight_change}"
    for instruction_address in HEURISTIC_ADDRESSES:
        operand_address = get_operand_address(instruction_address, FMUL_OPCODE_SIZE)
        patches.append(("heuristic", operand_address, original_bytes, new_bytes, description))
    return patches


def convert_to_file_offset(virtual_address):
    """`convert_to_file_offset` returns the file offset of `virtual_address`, which must be in
    the .text section of gc.exe."""
    return virtual_address - TEXT_VIRTUAL_ADDRESS + TEXT_FILE_OFFSET


def read_site(exe_bytes, virtual_address, byte_count):
    """`read_site` returns the `byte_count` bytes of `exe_bytes` that start at `virtual_address`.

    Edge case: The result is shorter when `exe_bytes` ends before the last of those bytes.
    """
    file_offset = convert_to_file_offset(virtual_address)
    return bytes(exe_bytes[file_offset : file_offset + byte_count])


def write_site(exe_bytes, virtual_address, new_bytes):
    """`write_site` replaces the bytes of `exe_bytes` that start at `virtual_address` with
    `new_bytes`."""
    file_offset = convert_to_file_offset(virtual_address)
    exe_bytes[file_offset : file_offset + len(new_bytes)] = new_bytes


def find_mismatches(exe_bytes, patches):
    """`find_mismatches` checks that each patch site in `exe_bytes` holds its original bytes or its
    new bytes, so that a site that is already patched is fine.

    Result: It returns one line of text for each site that holds other bytes.
    """
    mismatches = []
    for name, virtual_address, original_bytes, new_bytes, description in patches:
        current_bytes = read_site(exe_bytes, virtual_address, len(original_bytes))
        if current_bytes not in (original_bytes, new_bytes):
            site = f"{virtual_address:08x} {name}"
            expected_hex = original_bytes.hex()
            found_hex = current_bytes.hex()
            mismatches.append(f"{site}: expected {expected_hex} found {found_hex}")
    return mismatches


def parse_options():
    """`parse_options` reads the command line.

    Result: It returns the options. With `--vanilla`, every setting holds the value that the
      game itself uses.
    """
    # Read the options.
    help_format = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(description=HELP_TEXT, formatter_class=help_format)
    parser.add_argument("--input", default=DEFAULT_INPUT_PATH)
    parser.add_argument("--output", default=DEFAULT_OUTPUT_PATH)
    parser.add_argument("--search-cap", type=int, default=DEFAULT_SEARCH_CAP)
    parser.add_argument("--squad-budget", type=int, default=DEFAULT_SQUAD_BUDGET)
    parser.add_argument("--setmove-budget", type=int, default=DEFAULT_SETMOVE_BUDGET)
    parser.add_argument("--unit-budget", type=int, default=DEFAULT_UNIT_BUDGET)
    parser.add_argument("--ai-budget", type=int, default=DEFAULT_AI_BUDGET)
    parser.add_argument("--heuristic", type=float, default=DEFAULT_HEURISTIC)
    parser.add_argument("--vanilla", action="store_true", help="use all original values")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    # With `--vanilla`, replace every setting with the value of the game itself.
    if args.vanilla:
        args.search_cap = ORIGINAL_SEARCH_CAP
        args.squad_budget = ORIGINAL_SQUAD_BUDGET
        args.setmove_budget = ORIGINAL_SETMOVE_BUDGET
        args.unit_budget = VANILLA_UNIT_BUDGET
        args.ai_budget = ORIGINAL_AI_BUDGET
        args.heuristic = ORIGINAL_HEURISTIC
    return args


def main():
    """`main` prints the patches with `--list`, or else writes a patched copy of the input."""
    # With `--list`, only print the patches.
    args = parse_options()
    patches = build_patches(args)
    if args.list:
        for name, virtual_address, original_bytes, new_bytes, description in patches:
            site = f"{virtual_address:08x} {name:20s}"
            change = f"{original_bytes.hex():>12s} -> {new_bytes.hex():12s}"
            print(f"{site} {change} {description}")
        return

    # Read the input. Another exe version is still patched when every site holds the expected
    # bytes.
    with open(args.input, "rb") as input_file:
        exe_bytes = bytearray(input_file.read())
    input_sha256 = hashlib.sha256(exe_bytes).hexdigest()
    if input_sha256 != KNOWN_SHA256:
        warning = f"warning: unknown gc.exe (sha256 {input_sha256}); relying on byte checks"
        print(warning, file=sys.stderr)

    # Check every site before changing anything.
    mismatches = find_mismatches(exe_bytes, patches)
    if mismatches:
        raise SystemExit("refusing to patch, bytes differ:\n  " + "\n  ".join(mismatches))

    # Patch the copy in memory.
    for name, virtual_address, original_bytes, new_bytes, description in patches:
        write_site(exe_bytes, virtual_address, new_bytes)
        change = f"{original_bytes.hex():>8s} -> {new_bytes.hex():8s}"
        print(f"  {virtual_address:08x} {change}  {description}")

    # Write the copy, never over the input.
    if os.path.abspath(args.output) == os.path.abspath(args.input):
        message = "output must differ from input; copy the result into the game folder yourself"
        raise SystemExit(message)
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "wb") as output_file:
        output_file.write(exe_bytes)
    output_sha256 = hashlib.sha256(exe_bytes).hexdigest()
    print(f"wrote {args.output}  sha256 {output_sha256}")


if __name__ == "__main__":
    main()
