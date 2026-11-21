"""This module reads the binary files that the tools take apart: little-endian numbers, and the
headers of PE files.

PE is the format of Windows .exe and .dll files. decompilation/tools/fix_pe_header.py,
decompilation/tools/gcdis.py and mod/tools/gen_proxy.py read PE headers with this module, and
tools/sdf.py and decompilation/tools/minidump.py read numbers with it. A tool imports it only after it puts the tools folder on `sys.path`, because `python -I`
does not put the folder of the script there.
"""
import struct
from collections import namedtuple

# The PE format specification of Microsoft defines these byte offsets. The DOS header at the
# start of the file holds the file offset of the NT headers, which are the "PE\0\0" signature, the
# COFF header and the optional header. The table of section headers follows the optional header.
# A section header starts with the name (8 bytes), then the virtual size (the size in memory),
# the relative virtual address (RVA: the address minus the address where Windows loads the file),
# the raw size and the raw offset (the size and the file offset of the bytes of the section in
# the file), each a u32 (an unsigned 32-bit number). Memory past the raw size of a section is
# zero-filled.
NT_HEADER_POINTER_OFFSET = 0x3C  # `e_lfanew` in the DOS header holds the offset of the NT headers.
PE_SIGNATURE = b"PE\0\0"  # The NT headers start with these bytes.
SECTION_COUNT_OFFSET = 6  # The u16 `NumberOfSections` is this far into the NT headers.
OPTIONAL_HEADER_SIZE_OFFSET = 20  # The u16 `SizeOfOptionalHeader` is this far into them.
OPTIONAL_HEADER_OFFSET = 24  # The signature and the 20-byte COFF header come before it.
SECTION_HEADER_SIZE = 40  # Each section header has this many bytes.
SECTION_HEADER_FORMAT = "<8sIIII"  # It reads the name, virtual size, RVA, raw size, raw offset.

# `PeLayout` gives the file offsets of the headers of a PE file and its section headers. Each
# entry of `section_headers` is the tuple `(name, virtual size, RVA, raw size, raw offset)`, in
# the order of the section table, and `name` is the 8 raw bytes of the name.
PeLayout = namedtuple("PeLayout", [
    "nt_header_offset",
    "section_count",
    "optional_header_offset",
    "optional_header_size",
    "section_table_offset",
    "section_headers",
])


def read_u16(data, offset):
    """`read_u16` returns the little-endian unsigned 16-bit number at `offset` in `data`."""
    return struct.unpack_from("<H", data, offset)[0]


def read_u32(data, offset):
    """`read_u32` returns the little-endian unsigned 32-bit number at `offset` in `data`."""
    return struct.unpack_from("<I", data, offset)[0]


def read_i32(data, offset):
    """`read_i32` returns the little-endian signed 32-bit number at `offset` in `data`."""
    return struct.unpack_from("<i", data, offset)[0]


def read_u64(data, offset):
    """`read_u64` returns the little-endian unsigned 64-bit number at `offset` in `data`."""
    return struct.unpack_from("<Q", data, offset)[0]


def read_pe_layout(file_bytes):
    """`read_pe_layout` finds the headers and the section headers of the PE file in `file_bytes`.

    Result: It returns a `PeLayout`.
    Edge case: It raises AssertionError, with the offset of the NT headers in hex, when the NT
      headers do not start with the PE signature.
    """
    # Find the NT headers and check their signature.
    nt_header_offset = read_u32(file_bytes, NT_HEADER_POINTER_OFFSET)
    signature_end = nt_header_offset + len(PE_SIGNATURE)
    assert file_bytes[nt_header_offset:signature_end] == PE_SIGNATURE, hex(nt_header_offset)

    # Find the optional header and the section table that follows it.
    section_count = read_u16(file_bytes, nt_header_offset + SECTION_COUNT_OFFSET)
    optional_header_size = read_u16(file_bytes, nt_header_offset + OPTIONAL_HEADER_SIZE_OFFSET)
    optional_header_offset = nt_header_offset + OPTIONAL_HEADER_OFFSET
    section_table_offset = optional_header_offset + optional_header_size

    # Read the section headers.
    section_headers = []
    for section_index in range(section_count):
        header_offset = section_table_offset + SECTION_HEADER_SIZE * section_index
        section_header = struct.unpack_from(SECTION_HEADER_FORMAT, file_bytes, header_offset)
        section_headers.append(section_header)
    return PeLayout(
        nt_header_offset=nt_header_offset,
        section_count=section_count,
        optional_header_offset=optional_header_offset,
        optional_header_size=optional_header_size,
        section_table_offset=section_table_offset,
        section_headers=section_headers,
    )
