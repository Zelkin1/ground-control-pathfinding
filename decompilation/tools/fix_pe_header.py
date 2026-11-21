"""This script writes a copy of a PE file, such as gc.exe, with its NT headers moved to 0x40.

PE is the format of Windows .exe and .dll files. The NT headers are the PE signature, the COFF
header and the optional header, and binary_files.py describes them. In gc.exe, the NT headers
overlap the DOS header at the start of the file, and some PE readers reject that. In the copy,
the NT headers start right after a minimal DOS header, and decompilation/tools/ghidra_decompile.py
works on that copy. Only the first `SizeOfHeaders` bytes change, so the sections and their
addresses stay the same. The script prints the header values that it found, then the section table.

Usage: python -I decompilation/tools/fix_pe_header.py <input.exe> <output.exe>
"""
import struct
import sys
from pathlib import Path

# The shared readers are in the tools folder at the project root.
TOOLS_DIR = Path(__file__).resolve().parents[2] / "tools"
sys.path.insert(0, str(TOOLS_DIR))
from binary_files import (  # noqa: E402
    NT_HEADER_POINTER_OFFSET,
    SECTION_HEADER_SIZE,
    read_pe_layout,
    read_u32,
)

# The PE format specification of Microsoft also defines these offsets.
DOS_HEADER_SIZE = 0x40  # The DOS header has this many bytes.
DOS_SIGNATURE = b"MZ"  # A DOS header starts with these bytes.
SIZE_OF_HEADERS_OFFSET = 60  # The u32 `SizeOfHeaders` is this far into the optional header.
NEW_NT_HEADER_OFFSET = DOS_HEADER_SIZE  # The moved NT headers start right after the DOS header.

source_path = Path(sys.argv[1])
output_path = Path(sys.argv[2])
exe_bytes = bytearray(source_path.read_bytes())

# Find the NT headers, the section headers and the end of the headers.
layout = read_pe_layout(exe_bytes)
nt_header_offset = layout.nt_header_offset
section_count = layout.section_count
optional_header_size = layout.optional_header_size
headers_end = layout.section_table_offset + SECTION_HEADER_SIZE * section_count
size_of_headers_position = layout.optional_header_offset + SIZE_OF_HEADERS_OFFSET
size_of_headers = read_u32(exe_bytes, size_of_headers_position)
section_headers = layout.section_headers

# Find where the bytes of the first section start, and print the header values. Sections with no
# bytes in the file, such as zero-filled data, do not count.
first_raw_offset = min(raw_offset for _, _, _, raw_size, raw_offset in section_headers if raw_size)
nt_header_text = f"lfanew={nt_header_offset:#x} nsec={section_count}"
size_text = f"optsize={optional_header_size:#x} hdr_end={headers_end:#x}"
layout_text = f"SizeOfHeaders={size_of_headers:#x} first_raw={first_raw_offset:#x}"
print(nt_header_text, size_text, layout_text)

# Check that the moved headers end before the bytes of the first section, and within
# `SizeOfHeaders`.
nt_headers = bytes(exe_bytes[nt_header_offset:headers_end])
new_headers_end = NEW_NT_HEADER_OFFSET + len(nt_headers)
assert new_headers_end <= first_raw_offset and new_headers_end <= size_of_headers, "no room"

# Write a minimal DOS header, the moved NT headers, and zeros up to `SizeOfHeaders`.
fixed_bytes = bytearray(exe_bytes)
dos_header = bytearray(DOS_HEADER_SIZE)
dos_header[0:len(DOS_SIGNATURE)] = DOS_SIGNATURE
struct.pack_into("<I", dos_header, NT_HEADER_POINTER_OFFSET, NEW_NT_HEADER_OFFSET)
fixed_bytes[0:DOS_HEADER_SIZE] = dos_header
fixed_bytes[NEW_NT_HEADER_OFFSET:new_headers_end] = nt_headers
for byte_offset in range(new_headers_end, size_of_headers):
    fixed_bytes[byte_offset] = 0
output_path.write_bytes(fixed_bytes)

# List the sections.
for name, virtual_size, relative_address, raw_size, raw_offset in section_headers:
    section_name = name.rstrip(b"\0").decode(errors="replace")
    address_text = f"va={relative_address:#x} vsize={virtual_size:#x}"
    file_text = f"raw={raw_offset:#x} rawsize={raw_size:#x}"
    print(f"  {section_name:8}", address_text, file_text)
