"""This tool reads the Windows minidumps of gc.exe for crash analysis, with no dependencies.

A minidump is the file that Windows writes when a program crashes. It holds the loaded modules,
the threads, the exception and some of the memory of the program. Windows writes the crash dumps of
gc.exe to %LOCALAPPDATA%\\CrashDumps. This tool shows which DLLs were really loaded and from
where, the exception and the likely call chain, and it searches the captured memory.

Usage:
  python -I decompilation/tools/minidump.py <file.dmp>
  python -I decompilation/tools/minidump.py <file.dmp> strings <regex>
  python -I decompilation/tools/minidump.py <file.dmp> read <va> <n>

Without a mode, the tool prints the modules, the exception and a scan of the stack of the thread
that crashed. `strings` prints the ASCII and UTF-16 strings in the captured memory that contain a
match for `regex`. `read` prints a hex dump of `n` bytes of captured memory at the virtual
address `va`, which is in hex.
"""
import re
import struct
import sys
from pathlib import Path

# The shared readers are in the tools folder at the project root.
TOOLS_DIR = Path(__file__).resolve().parents[2] / "tools"
sys.path.insert(0, str(TOOLS_DIR))
from binary_files import read_u32, read_u64  # noqa: E402

# The `MINIDUMP_*` structures of Microsoft (minidumpapiset.h in the Windows SDK) define the
# minidump format. A header points to a directory of streams, and each directory entry gives the
# type, the size and the file offset of one stream. Each stream holds one kind of data. The
# thread, the module and the memory lists start with a u32 (an unsigned 32-bit number) entry
# count, and a location is a u32 size and a u32 file offset.
MINIDUMP_SIGNATURE = b"MDMP"  # A minidump starts with these bytes.
HEADER_FORMAT = "<4sIII"  # It reads the signature, version, stream count and directory offset.
DIRECTORY_ENTRY_FORMAT = "<III"  # It reads the stream type, size and file offset.
DIRECTORY_ENTRY_SIZE = 12  # `MINIDUMP_DIRECTORY` has this many bytes.
THREAD_LIST_STREAM = 3  # `ThreadListStream` holds the threads.
MODULE_LIST_STREAM = 4  # `ModuleListStream` holds the loaded modules.
MEMORY_LIST_STREAM = 5  # `MemoryListStream` holds memory ranges in smaller dumps.
EXCEPTION_STREAM = 6  # `ExceptionStream` holds the exception.
MEMORY64_LIST_STREAM = 9  # `Memory64ListStream` holds the memory ranges in full-memory dumps.
LIST_COUNT_SIZE = 4  # The u32 entry count of a list has this many bytes.
LOCATION_FORMAT = "<II"  # It reads a `MINIDUMP_LOCATION_DESCRIPTOR`.

# A memory descriptor gives the u64 start address of a captured range and its size. In the memory
# list, the size is a u32 and a u32 file offset follows. The memory64 list starts with a u64 range
# count and the u64 file offset of the bytes of the first range. Its sizes are u64, and the bytes
# of all ranges follow each other.
MEMORY64_LIST_HEADER_FORMAT = "<QQ"  # It reads the range count and the offset of the bytes.
MEMORY64_LIST_HEADER_SIZE = 16  # The memory64 list header has this many bytes.
MEMORY64_DESCRIPTOR_FORMAT = "<QQ"  # It reads the start address and the size.
MEMORY_DESCRIPTOR_FORMAT = "<QII"  # It reads the start address, the size and the file offset.
MEMORY_DESCRIPTOR_SIZE = 16  # Both kinds of memory descriptor have this many bytes.

# A module entry starts with its u64 base address, then its u32 size, checksum, timestamp and the
# file offset of its path, a `MINIDUMP_STRING`. That string is a u32 byte count, then UTF-16 text.
MODULE_ENTRY_FORMAT = "<QIIII"  # It reads the start of a `MINIDUMP_MODULE`.
MODULE_ENTRY_SIZE = 108  # `MINIDUMP_MODULE` has this many bytes.
STRING_LENGTH_SIZE = 4  # The byte count of a `MINIDUMP_STRING` has this many bytes.

# A thread entry starts with the u32 thread id. The stack is a `MINIDUMP_MEMORY_DESCRIPTOR`, and
# the context is the location of the saved registers.
THREAD_ENTRY_SIZE = 48  # `MINIDUMP_THREAD` has this many bytes.
THREAD_STACK_OFFSET = 24  # `Stack` starts this far into a thread entry.
THREAD_CONTEXT_OFFSET = 40  # `ThreadContext` starts this far into a thread entry.

# The exception stream starts with the u32 id of the thread that raised the exception. These
# offsets are from the start of `MINIDUMP_EXCEPTION_STREAM`.
EXCEPTION_CODE_OFFSET = 8  # The u32 `ExceptionCode` starts here.
EXCEPTION_ADDRESS_OFFSET = 24  # The u64 `ExceptionAddress` starts here.
PARAMETER_COUNT_OFFSET = 32  # The u32 `NumberParameters` starts here.
EXCEPTION_PARAMETERS_OFFSET = 40  # The u64 parameters, `ExceptionInformation`, start here.
EXCEPTION_PARAMETER_COUNT = 15  # `EXCEPTION_MAXIMUM_PARAMETERS` is the size of that array.
PARAMETERS_FORMAT = f"<{EXCEPTION_PARAMETER_COUNT}Q"  # It reads the whole array.
EXCEPTION_CONTEXT_OFFSET = 160  # `ThreadContext`, a location, follows the parameters here.

POINTER_SIZE = 4  # gc.exe is a 32-bit program, so its pointers have this many bytes.
MAX_STACK_CANDIDATES = 60  # The stack scan prints at most this many words.

# A string in memory is 5 or more printable ASCII characters, or the same in UTF-16.
ASCII_STRING_PATTERN = re.compile(rb"[\x20-\x7e]{5,}")  # It finds ASCII strings.
UTF16_STRING_PATTERN = re.compile(rb"(?:[\x20-\x7e]\x00){5,}")  # It finds UTF-16 strings.
STRING_KINDS = (  # Each kind of string has a pattern, an encoding and a label for the output.
    (ASCII_STRING_PATTERN, "ascii", ""),
    (UTF16_STRING_PATTERN, "utf-16-le", "(u16) "),
)


class Dump:
    """`Dump` is a minidump file with its streams and the memory ranges that it captured."""

    def __init__(self, path):
        self.dump_bytes = open(path, "rb").read()

        # Read the stream directory. When a type has several streams, the tool uses the first.
        signature, _, stream_count, directory_offset = self.unpack(HEADER_FORMAT, 0)
        assert signature == MINIDUMP_SIGNATURE, "not a minidump"
        self.streams = {}  # This maps each stream type to the (size, file offset) of the stream.
        for stream_index in range(stream_count):
            entry_offset = directory_offset + DIRECTORY_ENTRY_SIZE * stream_index
            stream_type, size, stream_offset = self.unpack(DIRECTORY_ENTRY_FORMAT, entry_offset)
            self.streams.setdefault(stream_type, (size, stream_offset))

        # Read the ranges of full-memory dumps, whose bytes follow each other from `data_offset`.
        self.ranges = []  # Each range is (start address, size, file offset).
        if MEMORY64_LIST_STREAM in self.streams:
            _, stream_offset = self.streams[MEMORY64_LIST_STREAM]
            range_count, data_offset = self.unpack(MEMORY64_LIST_HEADER_FORMAT, stream_offset)
            first_descriptor_offset = stream_offset + MEMORY64_LIST_HEADER_SIZE
            file_offset = data_offset
            for range_index in range(range_count):
                descriptor_offset = first_descriptor_offset + MEMORY_DESCRIPTOR_SIZE * range_index
                range_address, size = self.unpack(MEMORY64_DESCRIPTOR_FORMAT, descriptor_offset)
                self.ranges.append((range_address, size, file_offset))
                file_offset += size

        # Read the ranges of smaller dumps, each with its own file offset.
        memory_list_offsets = self.list_entry_offsets(MEMORY_LIST_STREAM, MEMORY_DESCRIPTOR_SIZE)
        for descriptor_offset in memory_list_offsets:
            memory_range = self.unpack(MEMORY_DESCRIPTOR_FORMAT, descriptor_offset)
            self.ranges.append(memory_range)

    def unpack(self, struct_format, offset):
        """`unpack` returns the values that `struct_format` reads at `offset` in the file."""
        return struct.unpack_from(struct_format, self.dump_bytes, offset)

    def list_entry_offsets(self, stream_type, entry_size):
        """`list_entry_offsets` returns the file offsets of the entries of a list stream.

        Inputs: `stream_type` is the type of a thread, module or memory list, and `entry_size` is
          the size of its entries.
        Edge case: It returns [] when the dump has no stream of that type.
        """
        if stream_type not in self.streams:
            return []
        _, stream_offset = self.streams[stream_type]
        entry_count = read_u32(self.dump_bytes, stream_offset)
        first_entry_offset = stream_offset + LIST_COUNT_SIZE
        entries_end = first_entry_offset + entry_size * entry_count
        return list(range(first_entry_offset, entries_end, entry_size))

    def read_string(self, string_offset):
        """`read_string` returns the text of the `MINIDUMP_STRING` at `string_offset`."""
        text_byte_count = read_u32(self.dump_bytes, string_offset)
        text_start = string_offset + STRING_LENGTH_SIZE
        text_bytes = self.dump_bytes[text_start : text_start + text_byte_count]
        return text_bytes.decode("utf-16-le", "replace")

    def list_modules(self):
        """`list_modules` returns `(base address, size, timestamp, path)` for every module.

        Edge case: It returns [] when the dump has no module list.
        """
        module_entries = []
        for entry_offset in self.list_entry_offsets(MODULE_LIST_STREAM, MODULE_ENTRY_SIZE):
            entry_fields = self.unpack(MODULE_ENTRY_FORMAT, entry_offset)
            base_address, size, _, timestamp, name_offset = entry_fields
            module_entries.append((base_address, size, timestamp, self.read_string(name_offset)))
        return module_entries

    def read(self, virtual_address, byte_count):
        """`read` returns `byte_count` bytes of memory at `virtual_address`.

        Edge case: It returns None when no single captured range holds all of those bytes.
        """
        request_end = virtual_address + byte_count
        for range_address, size, file_offset in self.ranges:
            range_end = range_address + size
            if range_address <= virtual_address and request_end <= range_end:
                start = file_offset + virtual_address - range_address
                return self.dump_bytes[start : start + byte_count]
        return None

    def read_exception(self):
        """`read_exception` returns the exception that the dump recorded.

        Result: The tuple is `(thread id, exception code, exception address, parameters, context
          size, context file offset)`. The context holds the registers of the thread.
        Edge case: It returns None when the dump has no exception stream.
        """
        if EXCEPTION_STREAM not in self.streams:
            return None
        _, stream_offset = self.streams[EXCEPTION_STREAM]

        # Read the thread, the code and the address of the exception.
        thread_id = read_u32(self.dump_bytes, stream_offset)
        exception_code = read_u32(self.dump_bytes, stream_offset + EXCEPTION_CODE_OFFSET)
        exception_address = read_u64(self.dump_bytes, stream_offset + EXCEPTION_ADDRESS_OFFSET)

        # Read the parameters that the exception uses, and the location of the context.
        parameter_count = read_u32(self.dump_bytes, stream_offset + PARAMETER_COUNT_OFFSET)
        parameters_offset = stream_offset + EXCEPTION_PARAMETERS_OFFSET
        all_parameters = self.unpack(PARAMETERS_FORMAT, parameters_offset)
        parameters = all_parameters[:parameter_count]
        context_location_offset = stream_offset + EXCEPTION_CONTEXT_OFFSET
        context_location = self.unpack(LOCATION_FORMAT, context_location_offset)
        return (thread_id, exception_code, exception_address, parameters, *context_location)

    def list_threads(self):
        """`list_threads` returns a dict from each thread id to the stack and context of that
        thread.

        Result: Each value is `(stack address, stack size, stack file offset, context size,
          context file offset)`. The dict is empty when the dump has no thread list.
        """
        threads_by_id = {}
        for entry_offset in self.list_entry_offsets(THREAD_LIST_STREAM, THREAD_ENTRY_SIZE):
            thread_id = read_u32(self.dump_bytes, entry_offset)
            stack = self.unpack(MEMORY_DESCRIPTOR_FORMAT, entry_offset + THREAD_STACK_OFFSET)
            context = self.unpack(LOCATION_FORMAT, entry_offset + THREAD_CONTEXT_OFFSET)
            threads_by_id[thread_id] = stack + context
        return threads_by_id


def find_owner_module(modules, virtual_address):
    """`find_owner_module` returns "file.dll+0x<offset>" for the module that holds an address.

    Inputs: `modules` is the list from `Dump.list_modules`.
    Edge case: It returns None when no module holds `virtual_address`.
    """
    for base_address, size, _, module_path in modules:
        if base_address <= virtual_address < base_address + size:
            file_name = module_path.split("\\")[-1]
            return f"{file_name}+{virtual_address - base_address:#x}"
    return None


def print_strings(dump, pattern_text):
    """`print_strings` prints the strings in captured memory in which `pattern_text` finds a match.

    `pattern_text` is a regex, and it is tested on the ASCII form of each string. The ASCII
    strings of each range come first, then its UTF-16 strings.
    """
    pattern = re.compile(pattern_text.encode())
    for range_address, size, file_offset in dump.ranges:
        range_bytes = dump.dump_bytes[file_offset : file_offset + size]
        for string_pattern, encoding, label in STRING_KINDS:
            for match in string_pattern.finditer(range_bytes):
                text = match.group().decode(encoding)
                if pattern.search(text.encode()):
                    print(f"{range_address + match.start():08x} {label}{text}")


def print_stack_scan(dump, modules, thread):
    """`print_stack_scan` prints the words of the stack of a thread that point into a module.

    Without symbols, these words are the candidates for return addresses, so they hint at the
    call chain. `print_stack_scan` prints at most `MAX_STACK_CANDIDATES` of them.
    Inputs: `thread` is a value from `Dump.list_threads`.
    """
    # Read the captured bytes of the stack.
    stack_address, stack_size, stack_offset, _, _ = thread
    stack_bytes = dump.dump_bytes[stack_offset : stack_offset + stack_size]

    # Print the words that point into a module.
    print(f"stack {stack_address:#x}+{stack_size:#x}; return-address candidates:")
    shown_count = 0
    for word_offset in range(0, len(stack_bytes) - POINTER_SIZE + 1, POINTER_SIZE):
        value = read_u32(stack_bytes, word_offset)
        location = find_owner_module(modules, value)
        if location and shown_count < MAX_STACK_CANDIDATES:
            print(f"  [{stack_address + word_offset:08x}] {value:08x} {location}")
            shown_count += 1


def print_summary(dump, modules):
    """`print_summary` prints the streams, the modules, the exception and a stack scan.

    The stack scan is of the thread that raised the exception.
    """
    # Print the streams and the modules.
    print(f"streams: {sorted(dump.streams)}  memory ranges: {len(dump.ranges)}")
    print("modules:")
    for base_address, size, timestamp, module_path in modules:
        print(f"  {base_address:08x} {size:8x} {timestamp:08x} {module_path}")

    # Print the exception, and scan the stack of the thread that raised it.
    exception = dump.read_exception()
    if exception:
        thread_id, exception_code, exception_address, parameters, _, _ = exception
        owner_module = find_owner_module(modules, exception_address)
        parameter_texts = [hex(parameter) for parameter in parameters]
        code_text = f"exception {exception_code:#010x} at {exception_address:#x} ({owner_module})"
        print(code_text, f"thread {thread_id} params {parameter_texts}")
        thread = dump.list_threads().get(thread_id)
        if thread:
            print_stack_scan(dump, modules, thread)


def main():
    """`main` runs the mode given on the command line, as the module docstring describes."""
    # Read the dump and its modules.
    dump = Dump(sys.argv[1])
    modules = dump.list_modules()

    # Run the mode.
    mode = sys.argv[2] if len(sys.argv) > 2 else None
    if mode == "strings":
        print_strings(dump, sys.argv[3])
    elif mode == "read":
        virtual_address = int(sys.argv[3], 16)
        byte_count = int(sys.argv[4])
        memory_bytes = dump.read(virtual_address, byte_count)
        if memory_bytes:
            print(memory_bytes.hex(" "))
        else:
            print("not captured")
    else:
        print_summary(dump, modules)


if __name__ == "__main__":
    main()
