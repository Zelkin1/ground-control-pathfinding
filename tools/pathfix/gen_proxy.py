"""`gen_proxy` writes the export layer of a proxy DLL from a 32-bit system DLL.

A proxy DLL has the name of a system DLL and is in the folder of gc.exe, so gc.exe loads it in
place of the system DLL. It passes every call on to the real system DLL. This tool reads the
exports of %SystemRoot%\\SysWOW64\\<dll>.dll and writes three files into `OUTPUT_DIR`
(`src/generated`):
- `proxy_exports.S` has one assembly stub for each export. Stub i jumps to the address in entry i
  of `g_proxy_procs`, an array that the C code of the proxy defines.
- `proxy.def` exports each stub under the name and the ordinal (the number of the export) of the
  real export, so the ordinals stay the same.
- `proxy_names.h` holds the name of the DLL and `g_proxy_exports`, the name and ordinal of each
  export in stub order.
The `DllMain` of the proxy must fill `g_proxy_procs` from the real system DLL when the proxy
loads, in the order of `g_proxy_exports`. Forwarders (exports that point to a function in
another DLL) resolve through `GetProcAddress` like the other exports.

Usage: python -I tools/pathfix/gen_proxy.py [dll name, default wsock32]
  python -I tools/pathfix/gen_proxy.py wsock32

The default is wsock32 because gc.exe gets compatibility shims (AcGenral, AcLayers) that load
winmm.dll and other DLLs from System32 before Windows resolves the imports of the game itself, so
a winmm proxy never loads. Windows resolves the WSOCK32 import of gc.exe after the shims (see
re/PATHFINDING.md, "Loading a mod DLL").
"""
import os
import struct
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent.parent  # This file is in the tools/pathfix folder.
sys.path.insert(0, str(TOOLS_DIR))  # The shared readers, binary_files.py, are in the tools folder.
from binary_files import read_pe_layout, read_u16, read_u32  # noqa: E402

PROJECT_ROOT = TOOLS_DIR.parent
OUTPUT_DIR = PROJECT_ROOT / "src" / "generated"
DEFAULT_DLL_NAME = "wsock32"  # The module docstring says why the proxy takes this name.
DEFAULT_WINDOWS_DIR = r"C:\Windows"  # This is used when the SystemRoot variable is not set.
POINTER_SIZE = 4  # `g_proxy_procs` holds 32-bit pointers, because gc.exe is a 32-bit program.

# binary_files.py describes the PE headers, and the PE format specification of Microsoft also
# defines these offsets. Addresses in these headers are relative virtual addresses (RVAs), which
# count from the address where Windows loads the DLL.
PE32_MAGIC = 0x10B  # The optional header of a 32-bit file starts with this number.
# In a 32-bit file, the data directories start this far into the optional header. The first one
# holds the RVA and the size of the export directory.
EXPORT_DIRECTORY_POINTER_OFFSET = 96
EXPORT_TABLES_OFFSET = 16  # The export table fields lie this far into the export directory.
# These fields are the ordinal base, the counts of functions and names, and the RVAs of the
# function table, the name table and the ordinal table.
EXPORT_TABLES_FORMAT = "<IIIIII"
ADDRESS_SIZE = 4  # The function table and the name table hold 32-bit RVAs.
ORDINAL_SIZE = 2  # Each entry of the ordinal table is a 16-bit index into the function table.


def read_exports(path):
    """`read_exports` lists the exports of the 32-bit DLL at `path`.

    The export directory points to three tables. The function table has one slot for each
    ordinal, from the ordinal base on, and each slot holds the RVA of a function, or 0 when the
    slot is unused. Entry i of the name table holds the RVA of a name, and entry i of the ordinal
    table holds the index of the function table slot with that name.
    Result: It returns one `(name, ordinal)` pair for each used slot of the function table, in
      slot order. `name` is None for an export that has only an ordinal.
    Edge cases: It raises AssertionError when the file is not a PE file or not a 32-bit DLL, and
      ValueError with the RVA in hex when no section holds an RVA that it reads.
    """
    with open(path, "rb") as dll_file:
        dll_bytes = dll_file.read()

    # Find the optional header, the export directory and the section headers.
    layout = read_pe_layout(dll_bytes)
    optional_header_offset = layout.optional_header_offset
    assert read_u16(dll_bytes, optional_header_offset) == PE32_MAGIC, "expected a 32-bit DLL"
    export_pointer_offset = optional_header_offset + EXPORT_DIRECTORY_POINTER_OFFSET
    export_directory_address = read_u32(dll_bytes, export_pointer_offset)
    sections = layout.section_headers

    def convert_to_file_offset(relative_address):
        """`convert_to_file_offset` returns the file offset of the RVA `relative_address`.

        A section covers the larger of its size in memory and its size in the file.
        Edge case: It raises ValueError with the RVA in hex when no section holds it.
        """
        for _, virtual_size, section_start, raw_size, raw_offset in sections:
            section_end = section_start + max(virtual_size, raw_size)
            if section_start <= relative_address < section_end:
                return raw_offset + relative_address - section_start
        raise ValueError(hex(relative_address))

    def read_c_string(relative_address):
        """`read_c_string` reads the NUL-terminated string at the RVA `relative_address`."""
        file_offset = convert_to_file_offset(relative_address)
        string_end = dll_bytes.index(b"\0", file_offset)
        return dll_bytes[file_offset:string_end].decode()

    def read_table_u32(table_address, entry_index):
        """`read_table_u32` reads entry `entry_index` of the RVA table at `table_address`."""
        table_offset = convert_to_file_offset(table_address)
        return read_u32(dll_bytes, table_offset + ADDRESS_SIZE * entry_index)

    def read_table_u16(table_address, entry_index):
        """`read_table_u16` reads entry `entry_index` of the ordinal table at `table_address`."""
        table_offset = convert_to_file_offset(table_address)
        return read_u16(dll_bytes, table_offset + ORDINAL_SIZE * entry_index)

    # Read where the export tables are.
    export_directory_offset = convert_to_file_offset(export_directory_address)
    tables_offset = export_directory_offset + EXPORT_TABLES_OFFSET
    export_tables = struct.unpack_from(EXPORT_TABLES_FORMAT, dll_bytes, tables_offset)
    ordinal_base, function_count, name_count, *table_addresses = export_tables
    function_table_address, name_table_address, ordinal_table_address = table_addresses

    # Name each slot of the function table that has a name.
    names_by_index = {}
    for name_index in range(name_count):
        name_address = read_table_u32(name_table_address, name_index)
        export_name = read_c_string(name_address)
        function_index = read_table_u16(ordinal_table_address, name_index)
        names_by_index[function_index] = export_name

    # List the used slots of the function table.
    export_entries = []
    for function_index in range(function_count):
        function_address = read_table_u32(function_table_address, function_index)
        if function_address == 0:
            continue
        ordinal = ordinal_base + function_index
        export_entries.append((names_by_index.get(function_index), ordinal))
    return export_entries


def write_assembly(output_dir, export_count):
    """`write_assembly` writes `proxy_exports.S` into `output_dir`, with one stub for each export.

    Stub i has the global label `_pfx_exp_<i>` and jumps to the address in entry i of
    `g_proxy_procs`. The leading underscores are the 32-bit C name decoration of `g_proxy_procs`
    and `pfx_exp_<i>`.
    """
    assembly_path = output_dir / "proxy_exports.S"
    with open(assembly_path, "w", encoding="utf-8", newline="\n") as output_file:
        output_file.write("# generated by tools/pathfix/gen_proxy.py - do not edit\n\t.text\n")
        for export_index in range(export_count):
            slot_offset = POINTER_SIZE * export_index
            output_file.write(f"\t.globl _pfx_exp_{export_index}\n_pfx_exp_{export_index}:\n")
            output_file.write(f"\tjmp *_g_proxy_procs+{slot_offset}\n")


def write_module_definition(output_dir, dll_name, export_entries):
    """`write_module_definition` writes `proxy.def` into `output_dir`. This module definition file
    tells the linker what the proxy exports.

    Stub i is exported under the name and ordinal of export i. An export that has only an ordinal
    gets the name `pfx_ordinal_<ordinal>`, and NONAME keeps that name out of the export table of
    the proxy.
    """
    definition_path = output_dir / "proxy.def"
    with open(definition_path, "w", encoding="utf-8", newline="\n") as output_file:
        output_file.write("; generated by tools/pathfix/gen_proxy.py - do not edit\n")
        output_file.write(f"LIBRARY {dll_name}.dll\nEXPORTS\n")
        for export_index, (export_name, ordinal) in enumerate(export_entries):
            stub_name = f"pfx_exp_{export_index}"
            if export_name:
                output_file.write(f"\t{export_name}={stub_name} @{ordinal}\n")
            else:
                output_file.write(f"\tpfx_ordinal_{ordinal}={stub_name} @{ordinal} NONAME\n")


def write_header(output_dir, dll_name, export_entries):
    """`write_header` writes `proxy_names.h` into `output_dir`, for the C code of the proxy.

    The header defines `PROXY_DLL`, the file name of the DLL as a wide string,
    `PROXY_NUM_EXPORTS`, and `g_proxy_exports`, the name and ordinal of each export in stub
    order. The name is 0 for an export that has only an ordinal.
    """
    header_path = output_dir / "proxy_names.h"
    with open(header_path, "w", encoding="utf-8", newline="\n") as output_file:
        output_file.write("/* generated by tools/pathfix/gen_proxy.py - do not edit */\n")
        output_file.write(f'#define PROXY_DLL L"{dll_name}.dll"\n')
        output_file.write(f"#define PROXY_NUM_EXPORTS {len(export_entries)}\n")
        entry_type = "struct { const char *name; unsigned short ordinal; }"
        output_file.write(f"static const {entry_type} g_proxy_exports[] = {{\n")
        for export_name, ordinal in export_entries:
            quoted_name = f'"{export_name}"' if export_name else "0"
            output_file.write(f"\t{{{quoted_name}, {ordinal}}},\n")
        output_file.write("};\n")


def main():
    """`main` writes the three generated files for one DLL into `OUTPUT_DIR`.

    Inputs: The first command-line argument is the name of the DLL without `.dll`. Without it,
      the DLL is `DEFAULT_DLL_NAME`.
    """
    # Read the exports of the 32-bit system DLL, because gc.exe is a 32-bit program. 64-bit
    # Windows keeps its 32-bit system DLLs in SysWOW64.
    dll_name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DLL_NAME
    windows_dir = Path(os.environ.get("SystemRoot", DEFAULT_WINDOWS_DIR))
    dll_path = windows_dir / "SysWOW64" / (dll_name + ".dll")
    export_entries = read_exports(dll_path)
    export_count = len(export_entries)

    # Write the generated files.
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    write_assembly(OUTPUT_DIR, export_count)
    write_module_definition(OUTPUT_DIR, dll_name, export_entries)
    write_header(OUTPUT_DIR, dll_name, export_entries)

    # Print the counts.
    named_count = sum(1 for export_name, _ in export_entries if export_name)
    print(f"{dll_name}.dll: {export_count} exports ({named_count} named) -> {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
