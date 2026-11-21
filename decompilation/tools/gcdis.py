"""This tool helps to reverse-engineer gc.exe, Ground Control 1.0.1.1, built with MSVC6.

This tool found the addresses in docs/PATHFINDING.md. It reads gc.exe from the Steam folder, or
from the path in the `GC_EXE` environment variable, and it never writes to it. Addresses are virtual addresses (VAs):
the addresses of the bytes in the loaded program. gc.exe has no relocations (the table that lets
Windows load a program at another address), so it always loads at 0x400000 and a VA never
changes.

Usage:
  python -I decompilation/tools/gcdis.py str <regex>
  python -I decompilation/tools/gcdis.py xref <va>
  python -I decompilation/tools/gcdis.py dis <va> [count]
  python -I decompilation/tools/gcdis.py func <va>
  python -I decompilation/tools/gcdis.py calls <va>
  python -I decompilation/tools/gcdis.py names

Give VAs in hex. `str` lists the strings that contain a match for `regex`, with their VAs
and the code that references them (xrefs). `xref` lists the instructions that hold `va` as an
imm32 (a 32-bit value inside the instruction) or a disp32 (a 32-bit address in a memory operand).
`dis` disassembles `count` instructions from `va`, or 60 without `count`. `func` disassembles the
whole function that holds `va`. `calls` lists the direct calls to the function at `va`. `names`
names functions after the `Class::Method` names in the debug strings that they use.
"""
import bisect
import os
import re
import struct
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[2] / "tools"  # the tools folder at the project root
PYLIB_DIR = TOOLS_DIR / "pylib"  # The disassembler, capstone, is bundled here.
sys.path.insert(0, str(PYLIB_DIR))
sys.path.insert(0, str(TOOLS_DIR))  # The shared readers, binary_files.py, are in this folder.
import capstone  # noqa: E402
from binary_files import read_i32, read_pe_layout, read_u32  # noqa: E402

DEFAULT_EXE_PATH = r"C:\Program Files (x86)\Steam\steamapps\common\Ground Control Anthology\gc.exe"
EXE_PATH = os.environ.get("GC_EXE", DEFAULT_EXE_PATH)  # `GC_EXE` replaces the default path.
IMAGE_BASE = 0x400000  # This is the VA where gc.exe loads, its image base.

# The manual of Intel for the x86 instruction set gives these facts.
MAX_INSTRUCTION_BYTES = 15  # The longest x86 instruction has this many bytes.
ADDRESS_SIZE = 4  # An address in 32-bit code has this many bytes.
CALL_OPCODE = 0xE8  # This opcode is a call with a 32-bit displacement from the next instruction.
CALL_OPCODE_SIZE = 1  # The displacement follows this one-byte opcode.
CALL_INSTRUCTION_SIZE = 5  # The opcode and the displacement have this many bytes.
MAX_BYTES_BEFORE_ADDRESS = 7  # Without prefixes, an imm32 or disp32 starts at most this far in.
PATH_END_MNEMONICS = ("ret", "retn", "int3", "hlt")  # These instructions end a code path.

DEFAULT_INSTRUCTION_COUNT = 60  # `dis` decodes this many instructions without a count.
MAX_FUNCTION_INSTRUCTIONS = 6000  # `read_function` starts no new path past this many.
MAX_SWITCH_CASES = 256  # `read_switch_targets` reads at most this many entries.
MAX_FUNCTION_SIZE = 0x10000  # A switch target lies less than this past the function start.

RDATA_START = 0x60D000  # The section table of gc.exe starts .rdata here, above the code.
MAX_STRING_PREVIEW_BYTES = 80  # Only strings shorter than this get a comment.
# A string is a run of 4 or more printable ASCII characters, tabs and newlines.
STRING_PATTERN = re.compile(rb"[\x20-\x7e\t\n]{4,}")  # It finds such runs.
ADDRESS_PATTERN = re.compile(r"0x([0-9a-f]{6,8})")  # Hex values of 6 to 8 digits can be VAs.
NAME_PATTERN = r"[A-Za-z_][A-Za-z0-9_]*"  # It matches a C++ name.
METHOD_NAME_PATTERN = rf"{NAME_PATTERN}::~?{NAME_PATTERN}"  # It matches `Class::Method` names.


class Image:
    """`Image` is gc.exe as read from disk, with its sections and a disassembler.

    Each section is a tuple `(name, start VA, virtual size, raw offset, raw size)`.
    """

    def __init__(self, path=EXE_PATH):
        self.file_bytes = open(path, "rb").read()

        # Read the sections from the PE headers. The VA of a section is the image base plus its
        # RVA.
        self.sections = []
        for header in read_pe_layout(self.file_bytes).section_headers:
            name, virtual_size, relative_address, raw_size, raw_offset = header
            section_name = name.rstrip(b"\0").decode()
            section_start = IMAGE_BASE + relative_address
            self.sections.append((section_name, section_start, virtual_size, raw_offset, raw_size))
        self.text_section = self.find_section(".text")

        # Set up a 32-bit x86 disassembler.
        self.disassembler = capstone.Cs(capstone.CS_ARCH_X86, capstone.CS_MODE_32)
        self.disassembler.detail = False

    def find_section(self, name):
        """`find_section` returns the section called `name`."""
        return next(section for section in self.sections if section[0] == name)

    def read_section_bytes(self, section):
        """`read_section_bytes` returns the bytes of `section` that are stored in the file."""
        _, _, _, raw_offset, raw_size = section
        return self.file_bytes[raw_offset : raw_offset + raw_size]

    def convert_to_file_offset(self, virtual_address):
        """`convert_to_file_offset` returns the file offset of `virtual_address`.

        Edge case: It returns None when the VA is outside every section, or when it has no bytes
          in the file, such as in the zero-filled end of .data.
        """
        for _, section_start, virtual_size, raw_offset, raw_size in self.sections:
            section_end = section_start + max(virtual_size, raw_size)
            if section_start <= virtual_address < section_end:
                offset_in_section = virtual_address - section_start
                if offset_in_section < raw_size:
                    return raw_offset + offset_in_section
                return None
        return None

    def convert_to_virtual_address(self, file_offset):
        """`convert_to_virtual_address` returns the VA of `file_offset`.

        Edge case: It returns None when the offset is outside every section.
        """
        for _, section_start, virtual_size, raw_offset, raw_size in self.sections:
            if raw_offset <= file_offset < raw_offset + raw_size:
                return section_start + file_offset - raw_offset
        return None

    def read(self, virtual_address, byte_count):
        """`read` returns `byte_count` bytes of the file from a VA.

        Edge case: It returns fewer bytes at the end of the file.
        """
        file_offset = self.convert_to_file_offset(virtual_address)
        return self.file_bytes[file_offset : file_offset + byte_count]

    def read_c_string(self, virtual_address):
        """`read_c_string` returns the string at a VA, which ends with a NUL byte."""
        file_offset = self.convert_to_file_offset(virtual_address)
        string_end = self.file_bytes.index(b"\0", file_offset)
        return self.file_bytes[file_offset:string_end].decode("latin1")

    def read_text_section(self):
        """`read_text_section` returns `(start VA, bytes)` of the code section, .text."""
        _, section_start, _, _, _ = self.text_section
        return section_start, self.read_section_bytes(self.text_section)

    def decode_instruction(self, code_bytes, address):
        """`decode_instruction` returns the instruction at the start of `code_bytes`.

        Inputs: `address` is the VA of `code_bytes`.
        Edge case: It returns None when the bytes are not a valid instruction.
        """
        return next(self.disassembler.disasm(code_bytes, address), None)

    def find_instruction_around(self, code, text_start, address_start):
        """`find_instruction_around` finds the instruction that holds the 4 bytes at an offset.

        Inputs: `code` is the bytes of .text, which starts at the VA `text_start`, and
          `address_start` is the offset in `code` of the 4 bytes.
        Result: `find_instruction_around` tries the instruction starts from 1 to
          `MAX_BYTES_BEFORE_ADDRESS` bytes before `address_start`. It returns the first
          instruction that ends at or after the last of the 4 bytes.
        Edge case: It returns None when no start gives such an instruction.
        """
        address_end = address_start + ADDRESS_SIZE
        for bytes_before in range(1, MAX_BYTES_BEFORE_ADDRESS + 1):
            instruction_start = address_start - bytes_before
            if instruction_start < 0:
                continue
            window_end = instruction_start + MAX_INSTRUCTION_BYTES
            instruction_address = text_start + instruction_start
            instruction_bytes = code[instruction_start:window_end]
            instruction = self.decode_instruction(instruction_bytes, instruction_address)
            if instruction is None:
                continue
            instruction_end = instruction_start + instruction.size
            if instruction_end >= address_end:
                return instruction
        return None

    def find_xrefs(self, target_address):
        """`find_xrefs` returns the instructions in .text that hold `target_address`.

        The instructions hold the address as an imm32 or a disp32. A raw scan finds the 4 bytes
        of the address, and disassembly from a few bytes earlier confirms an instruction that
        holds them. The first decode that fits wins, so a hit can be a mis-decode inside the
        right function.
        """
        text_start, code = self.read_text_section()
        address_bytes = struct.pack("<I", target_address)
        referencing_instructions = []
        for match in re.finditer(re.escape(address_bytes), code):
            instruction = self.find_instruction_around(code, text_start, match.start())
            if instruction is not None:
                referencing_instructions.append(instruction)
        return referencing_instructions

    def disassemble(self, virtual_address, instruction_count=DEFAULT_INSTRUCTION_COUNT):
        """`disassemble` returns up to `instruction_count` instructions from a VA.

        It decodes them in a straight line, without following jumps.
        """
        instructions = []
        code_bytes = self.read(virtual_address, instruction_count * MAX_INSTRUCTION_BYTES)
        for instruction in self.disassembler.disasm(code_bytes, virtual_address):
            instructions.append(instruction)
            if len(instructions) >= instruction_count:
                break
        return instructions

    def find_direct_calls(self):
        """`find_direct_calls` yields `(call address, target address)` for each E8 byte in .text.

        `find_direct_calls` reads each E8 byte as a direct call, except in the last
        `CALL_INSTRUCTION_SIZE` bytes of .text. Some of these bytes are inside other
        instructions, so callers filter the targets.
        """
        text_start, code = self.read_text_section()
        for call_offset in range(len(code) - CALL_INSTRUCTION_SIZE):
            if code[call_offset] == CALL_OPCODE:
                displacement = read_i32(code, call_offset + CALL_OPCODE_SIZE)
                call_address = text_start + call_offset
                yield call_address, call_address + CALL_INSTRUCTION_SIZE + displacement

    _function_starts = None  # `list_function_starts` fills this on its first call.

    def list_function_starts(self):
        """`list_function_starts` returns the known function starts, sorted.

        They are the targets in .text of all direct calls, and the aligned 32-bit values in
        .rdata that point into .text, such as the entries of vtables (the tables of the
        virtual methods of a class). The scan takes most of a second, so `list_function_starts`
        builds the list once and then keeps it.
        """
        if self._function_starts is None:
            text_start, code = self.read_text_section()
            text_end = text_start + len(code)
            targets = set()

            # Add the targets of direct calls.
            for _, target in self.find_direct_calls():
                if text_start <= target < text_end:
                    targets.add(target)

            # Add the values in .rdata.
            rdata_bytes = self.read_section_bytes(self.find_section(".rdata"))
            for word_offset in range(0, len(rdata_bytes) - ADDRESS_SIZE + 1, ADDRESS_SIZE):
                target = read_u32(rdata_bytes, word_offset)
                if text_start <= target < text_end:
                    targets.add(target)

            self._function_starts = sorted(targets)
        return self._function_starts

    def find_function_start(self, virtual_address):
        """`find_function_start` returns the last known function start at or before a VA.

        Edge case: It returns the VA itself when no known function starts at or before it.
        """
        starts = self.list_function_starts()
        start_index = bisect.bisect_right(starts, virtual_address) - 1
        return starts[start_index] if start_index >= 0 else virtual_address

    def read_function(self, start, max_instructions=MAX_FUNCTION_INSTRUCTIONS):
        """`read_function` disassembles the function at `start` by following its jumps.

        `read_function` decodes a path in a straight line from each address that the code can
        reach, so data between pieces of code is skipped. A path ends at an instruction in
        `PATH_END_MNEMONICS`, at an unconditional `jmp`, at an address that is already decoded,
        or at bytes that are not a valid instruction. Each jump adds the targets that
        `find_jump_targets` gives as new paths.
        Result: It returns `(start, instructions in address order)`.
        Edge case: It starts no new path once it has `max_instructions` instructions.
        """
        # Decode one path at a time, from the last pending address.
        instructions_by_address = {}
        pending_addresses = [start]
        function_starts = self.list_function_starts()
        while pending_addresses and len(instructions_by_address) < max_instructions:
            address = pending_addresses.pop()
            while address not in instructions_by_address:
                code_bytes = self.read(address, MAX_INSTRUCTION_BYTES)
                instruction = self.decode_instruction(code_bytes, address)
                if instruction is None:
                    break
                instructions_by_address[address] = instruction
                mnemonic = instruction.mnemonic
                if mnemonic in PATH_END_MNEMONICS:
                    break
                if mnemonic.startswith("j"):
                    jump_targets = self.find_jump_targets(instruction, start, function_starts)
                    pending_addresses.extend(jump_targets)
                    if mnemonic == "jmp":
                        break
                address += instruction.size

        # Sort the instructions by address. The addresses are the keys of a dict, so they all
        # differ and the sort never compares two instructions.
        instructions = [instruction for _, instruction in sorted(instructions_by_address.items())]
        return start, instructions

    def find_jump_targets(self, instruction, function_start, function_starts):
        """`find_jump_targets` returns the addresses in the function that a jump can lead to.

        Inputs: `function_starts` is the list from `list_function_starts`.
        Result: For a direct jump, the list holds its target. It is empty when the target is the
          start of another known function, because that jump is a tail call (a call that ends
          the function). For a jump through a switch table, the list holds the entries that
          `read_switch_targets` gives. For other jumps, the list is empty.
        """
        operands = instruction.op_str

        # Follow a direct jump, unless it is a tail call. Capstone writes its operand as a hex
        # number.
        if operands.startswith("0x"):
            target = int(operands, 16)
            is_tail_call = target in function_starts and target != function_start
            return [] if is_tail_call else [target]

        # Find the table of a jump through a switch table. Capstone writes its operand as
        # `dword ptr [register*4 + table]`.
        if "*4 +" not in operands or "dword ptr" not in operands:
            return []
        table_match = re.search(r"\+ (0x[0-9a-f]+)\]", operands)
        if not table_match:
            return []
        table_address = int(table_match.group(1), 16)
        return self.read_switch_targets(table_address, function_start)

    def read_switch_targets(self, table_address, function_start):
        """`read_switch_targets` returns the entries of the switch table at `table_address`.

        Result: The list ends before the first entry that points before `function_start` or
          `MAX_FUNCTION_SIZE` bytes or more past it, and it holds at most `MAX_SWITCH_CASES`
          entries.
        """
        targets = []
        function_end = function_start + MAX_FUNCTION_SIZE
        for case_index in range(MAX_SWITCH_CASES):
            entry_address = table_address + ADDRESS_SIZE * case_index
            target = read_u32(self.read(entry_address, ADDRESS_SIZE), 0)
            is_inside_function = function_start <= target < function_end
            if not is_inside_function:
                break
            targets.append(target)
        return targets

    def find_callers(self, target_address):
        """`find_callers` returns the addresses of the direct calls to `target_address`."""
        call_sites = []
        for call_site, call_target in self.find_direct_calls():
            if call_target == target_address:
                call_sites.append(call_site)
        return call_sites

    def find_strings(self, pattern):
        """`find_strings` yields `(VA, text)` for each printable string with a match for `pattern`.

        `find_strings` searches all sections, and `pattern` is a regex.
        """
        compiled_pattern = re.compile(pattern)
        for section in self.sections:
            _, section_start, _, _, _ = section
            section_bytes = self.read_section_bytes(section)
            for match in STRING_PATTERN.finditer(section_bytes):
                text = match.group().decode("latin1")
                if compiled_pattern.search(text):
                    yield section_start + match.start(), text


def read_data_string(image, virtual_address):
    """`read_data_string` returns the printable string at a VA in a data section, or None.

    Edge case: It returns None when the VA is below `RDATA_START`, which is code, or has no bytes
      in the file. It also returns None when the string does not end within
      `MAX_STRING_PREVIEW_BYTES` bytes, or when `STRING_PATTERN` does not match all of it.
    """
    # Find the bytes of the VA in the file.
    file_offset = image.convert_to_file_offset(virtual_address)
    if file_offset is None or virtual_address < RDATA_START:
        return None

    # Take the bytes before the first NUL byte in `preview_bytes`, when they form a string.
    preview_bytes = image.file_bytes[file_offset : file_offset + MAX_STRING_PREVIEW_BYTES]
    string_end = preview_bytes.find(b"\0")
    string_bytes = preview_bytes[:string_end]
    if string_end >= 0 and STRING_PATTERN.fullmatch(string_bytes):
        return string_bytes.decode()
    return None


def format_instruction(instruction, image=None):
    """`format_instruction` returns one line for `instruction`.

    With `image`, each operand that points at a string in a data section gets the string as a
    comment.
    """
    line = f"{instruction.address:08x}  {instruction.mnemonic:7s} {instruction.op_str}"
    if image:
        for hex_digits in ADDRESS_PATTERN.findall(instruction.op_str):
            string = read_data_string(image, int(hex_digits, 16))
            if string is not None:
                line += f'    ; "{string}"'
    return line


def print_string_references(image, pattern):
    """`print_string_references` prints the strings with a match for `pattern`, and their xrefs.

    Each line holds the VA of a string, the string, and the addresses of the instructions that
    reference it.
    """
    for virtual_address, text in image.find_strings(pattern):
        references = image.find_xrefs(virtual_address)
        reference_list = ", ".join(f"{reference.address:08x}" for reference in references)
        print(f"{virtual_address:08x} {text!r}  <- {reference_list}")


def print_references(image, target_address):
    """`print_references` prints the instructions that hold `target_address`, and their functions.

    An instruction holds the address as an imm32 or a disp32. Each line ends with the start of
    the function that holds the instruction.
    """
    for reference in image.find_xrefs(target_address):
        function_start = image.find_function_start(reference.address)
        print(format_instruction(reference, image), "   [func", hex(function_start) + "]")


def print_disassembly(image, virtual_address, instruction_count):
    """`print_disassembly` prints `instruction_count` instructions decoded from a VA."""
    for instruction in image.disassemble(virtual_address, instruction_count):
        print(format_instruction(instruction, image))


def print_function(image, virtual_address):
    """`print_function` prints the whole function that holds a VA.

    A line "..." marks each gap, where an instruction does not start right after the previous
    one.
    """
    # Decode the function.
    function_start = image.find_function_start(virtual_address)
    start, instructions = image.read_function(function_start)

    # Print a header line, then the instructions in address order.
    print(f"; function @ {start:08x}, {len(instructions)} insns")
    previous_end = None
    for instruction in instructions:
        if previous_end is not None and instruction.address != previous_end:
            print("        ...")
        print(format_instruction(instruction, image))
        previous_end = instruction.address + instruction.size


def print_callers(image, target_address):
    """`print_callers` prints the direct calls to `target_address` and the starts of the
    functions that hold them."""
    for call_site in image.find_callers(target_address):
        print(f"{call_site:08x}  [func {image.find_function_start(call_site):08x}]")


def print_function_names(image):
    """`print_function_names` names functions after the debug strings that they use.

    Result: `print_function_names` prints each function that uses a string with a
      `Class::Method` name, with the first such name of each of those strings.
    """
    # Collect the method names of the strings that each function uses.
    names_by_function = {}
    for virtual_address, text in image.find_strings(METHOD_NAME_PATTERN):
        method_name = re.search(METHOD_NAME_PATTERN, text).group()
        for reference in image.find_xrefs(virtual_address):
            function_start = image.find_function_start(reference.address)
            names_by_function.setdefault(function_start, set()).add(method_name)

    # Print them in address order.
    for function_start in sorted(names_by_function):
        method_names = sorted(names_by_function[function_start])
        print(f"{function_start:08x} {' | '.join(method_names)}")


def main(argv):
    """`main` runs the command in `argv`, as the module docstring describes."""
    # Read gc.exe.
    image = Image()

    # Run the command.
    command = argv[1]
    if command == "str":
        print_string_references(image, argv[2])
    elif command == "xref":
        print_references(image, int(argv[2], 16))
    elif command == "dis":
        instruction_count = DEFAULT_INSTRUCTION_COUNT
        if len(argv) > 3:
            instruction_count = int(argv[3])
        print_disassembly(image, int(argv[2], 16), instruction_count)
    elif command == "func":
        print_function(image, int(argv[2], 16))
    elif command == "names":
        print_function_names(image)
    elif command == "calls":
        print_callers(image, int(argv[2], 16))


if __name__ == "__main__":
    main(sys.argv)
