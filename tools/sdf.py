"""This tool lists and extracts the files in the .sdf archives of Ground Control.

The game keeps its data in .sdf archives. The simulator reads the maps (`*.SLP` in `Missions.sdf`)
and the unit values (`ENTFILES/*.ENT` in `Data0.sdf`, `DATA1.sdf` and `DATA2.sdf`) from copies
that this tool extracts. The tool reads the archives in the same way as the code in gc.exe, and
docs/PATHFINDING.md describes the format in the section "Archive format (.sdf)".

Usage:
  python -I tools/sdf.py list    <archive.sdf>
  python -I tools/sdf.py extract <archive.sdf> <out_dir> [name_regex]

`list` prints the size, the offset and the path of each file in the archive. `extract` writes
the files below `out_dir`. With `name_regex`, it writes only the files whose path contains a
match for `name_regex`, and the match ignores upper and lower case.
"""
import re
import struct
import sys
import zlib
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent  # The shared readers are in this folder.
sys.path.insert(0, str(TOOLS_DIR))
from binary_files import read_u32  # noqa: E402

# An archive starts with a u32 (an unsigned 32-bit number) called the magic, which `Archive::Open`
# (0x410dc0) in gc.exe reads. The low 24 bits of the magic are the bytes "RYS", and its top byte
# selects the mode. The low 6 bits of the mode are the version of the archive. Each mode maps to a
# set of flags in the game, which say whether file data is encrypted, compressed or plain.
ARCHIVE_MAGIC = 0x535952  # The low 24 bits of the magic are this value, "RYS".
MAGIC_MASK = 0xFFFFFF  # This mask keeps the low 24 bits of the magic.
MODE_SHIFT = 24  # The mode is the top byte of the magic.
VERSION_MASK = 0x3F  # This mask keeps the version, the low 6 bits of the mode.
FLAGS_BY_MODE = {0x07: 0x40, 0x87: 0x80, 0xC7: 0x88, 0x47: 0x08}  # The flag 0x40 means plain data.
ENCRYPTED_FLAG = 0x08  # With this flag, file data is encrypted.
COMPRESSED_FLAG = 0x80  # With this flag, file data is a u32 plain size, then a zlib stream.
LAST_SHORT_HEADER_VERSION = 5  # Later versions store the size of the archive after the magic.
SHORT_HEADER_SIZE = 4  # The header of older versions holds only the magic.
LONG_HEADER_SIZE = 8  # The header of later versions holds the magic and the size of the archive.

# A directory block starts with one u32 offset per entry, counted from the start of the block. An
# entry is a u32 size, a u32 offset from the start of the archive, a u8 flags byte, and then the
# name, which ends with a NUL byte. For a subdirectory, the size is its entry count.
ENTRY_FORMAT = "<IIB"  # This reads the size, the offset and the flags of an entry.
ENTRY_NAME_OFFSET = 9  # The name starts after the size, the offset and the flags.
DIRECTORY_ENTRY_FLAG = 1  # With this flag, the entry is a subdirectory.

# `Decrypt` (0x410b80) in gc.exe decrypts with a rolling XOR over 32-bit words: it XORs each word
# with a key, and then it changes the key. A salt is a fixed number that is mixed into the key.
KEY_SALT_BY_VERSION = {5: 0x7E183C5A, 6: 0x7E183C5A, 7: 0x6D3AE31B}  # These are the salts.
WORD_SIZE = 4  # A word, a u32, has this many bytes.
WORD_BITS = 32  # A word has this many bits.
WORD_MASK = 0xFFFFFFFF  # This mask keeps the low 32 bits of a number.
ROTATION_MASK = WORD_BITS - 1  # The key turns by its low 5 bits, so by 0 to 31 bits.


def decrypt(data, version):
    """`decrypt` returns `data` decrypted as one block, as `Decrypt` (0x410b80) in gc.exe does it.

    The key starts from the length of the block, so callers must decrypt a block as a whole, with
    the length that the game reads it with. For a version in `KEY_SALT_BY_VERSION`, the key starts
    as the bitwise NOT of the length, XORed with the salt of the version. For other versions, the
    key starts as the version number. After each word, the key turns right by its low 5 bits, and
    then `decrypt` XORs it with the number of bytes from the start of that word to the end of
    the block.
    Edge case: The bytes of a trailing partial word stay as they are.
    """
    # Start the key from the length of the block.
    block_length = len(data)
    if version in KEY_SALT_BY_VERSION:
        key = (~block_length & WORD_MASK) ^ KEY_SALT_BY_VERSION[version]
    else:
        key = version

    # XOR each whole word with the key, then turn the key and mix in the bytes left.
    decrypted = bytearray(data)
    remaining_bytes = block_length
    word_offset = 0
    while remaining_bytes >= WORD_SIZE:
        word = read_u32(decrypted, word_offset)
        struct.pack_into("<I", decrypted, word_offset, word ^ key)
        rotation_bits = key & ROTATION_MASK
        rotated_key = ((key >> rotation_bits) | (key << (WORD_BITS - rotation_bits))) & WORD_MASK
        key = rotated_key ^ remaining_bytes
        remaining_bytes -= WORD_SIZE
        word_offset += WORD_SIZE
    return bytes(decrypted)


class Archive:
    """`Archive` is an open .sdf archive.

    Edge case: The constructor raises `ValueError` when the low 24 bits of the first u32 of the
      file are not "RYS".
    """

    def __init__(self, path):
        # Read the magic. Unlike the directory, it is not encrypted.
        self.file = open(path, "rb")
        magic = read_u32(self.file.read(WORD_SIZE), 0)
        if magic & MAGIC_MASK != ARCHIVE_MAGIC:
            raise ValueError(f"{path}: not an .sdf archive")

        # Find the version, the flags and the start of the root directory.
        mode = magic >> MODE_SHIFT
        self.version = mode & VERSION_MASK
        self.flags = FLAGS_BY_MODE[mode]
        has_long_header = self.version > LAST_SHORT_HEADER_VERSION
        self.directory_offset = LONG_HEADER_SIZE if has_long_header else SHORT_HEADER_SIZE

    def read(self, byte_count):
        """`read` returns the next `byte_count` bytes of the directory, decrypted as one block."""
        return decrypt(self.file.read(byte_count), self.version)

    def read_word(self):
        """`read_word` returns the next u32 of the directory, decrypted as a block of its own."""
        return read_u32(self.read(WORD_SIZE), 0)

    def list_entries(self, prefix=""):
        """`list_entries` yields `(path, size, offset)` for every file in the archive.

        Inputs: `prefix` goes in front of every path.
        """
        # The root directory is a u32 entry count, then a block like that of a subdirectory.
        self.file.seek(self.directory_offset)
        entry_count = self.read_word()
        yield from self._walk(entry_count, prefix)

    def _walk(self, entry_count, prefix):
        """`_walk` yields the files of the block at the file position, then of its subdirectories.

        Inputs: `entry_count` is the number of entries in the block, and `prefix` is the path of
          the directory, which goes in front of each name.
        """
        # Read the block, which follows its u32 size.
        block_size = self.read_word()
        block = self.read(block_size)

        # Yield the files, and keep the subdirectories for later.
        subdirectories = []
        for entry_index in range(entry_count):
            record_offset = read_u32(block, WORD_SIZE * entry_index)
            entry_fields = struct.unpack_from(ENTRY_FORMAT, block, record_offset)
            entry_size, entry_offset, entry_flags = entry_fields
            name_start = record_offset + ENTRY_NAME_OFFSET
            name_end = block.index(b"\0", name_start)
            name = block[name_start:name_end].decode("latin1")
            entry_path = prefix + name
            if entry_flags & DIRECTORY_ENTRY_FLAG:
                subdirectories.append((entry_path + "/", entry_size, entry_offset))
            else:
                yield entry_path, entry_size, entry_offset

        # Walk each subdirectory, whose u32 block size is at the offset of its entry.
        for subdirectory_prefix, subdirectory_entry_count, subdirectory_offset in subdirectories:
            self.file.seek(subdirectory_offset)
            yield from self._walk(subdirectory_entry_count, subdirectory_prefix)

    def read_entry_data(self, size, offset):
        """`read_entry_data` returns the bytes of a file, decrypted and unpacked as the flags say.

        Inputs: `size` and `offset` are the values that `list_entries` gives for the file.
        """
        # Read the stored bytes, and decrypt them as one block.
        self.file.seek(offset)
        entry_data = self.file.read(size)
        if self.flags & ENCRYPTED_FLAG:
            entry_data = decrypt(entry_data, self.version)

        # Unpack the zlib stream that follows the u32 plain size.
        if self.flags & COMPRESSED_FLAG:
            plain_size = read_u32(entry_data, 0)
            entry_data = zlib.decompress(entry_data[WORD_SIZE:])
            unpacked_size = len(entry_data)
            assert unpacked_size == plain_size, (unpacked_size, plain_size)
        return entry_data


def print_listing(archive, entries):
    """`print_listing` prints the size, the offset and the path of each entry, then a summary.

    Inputs: `entries` is the list of `(path, size, offset)` from `Archive.list_entries`.
    """
    for entry_path, size, offset in entries:
        print(f"{size:10d} {offset:#010x} {entry_path}")
    print(f"{len(entries)} files, version {archive.version}, flags {archive.flags:#x}")


def extract_files(archive, entries, output_dir, name_pattern):
    """`extract_files` writes entries to files in the folders of the archive below `output_dir`.

    Inputs: `entries` is the list from `Archive.list_entries`. When `name_pattern`, a compiled
      regex, is not None, `extract_files` writes only the entries whose path contains a match for
      it.
    """
    extracted_count = 0
    for entry_path, size, offset in entries:
        if name_pattern and not name_pattern.search(entry_path):
            continue
        path_parts = entry_path.replace("\\", "/").split("/")
        target_path = output_dir.joinpath(*path_parts)
        target_path.parent.mkdir(parents=True, exist_ok=True)
        target_path.write_bytes(archive.read_entry_data(size, offset))
        extracted_count += 1
    print(f"extracted {extracted_count} files to {output_dir}")


def main(argv):
    """`main` runs the command in `argv`, as the module docstring describes."""
    # Read the directory of the archive.
    command = argv[1]
    archive_path = argv[2]
    archive = Archive(archive_path)
    entries = list(archive.list_entries())

    # Run the command.
    if command == "list":
        print_listing(archive, entries)
    elif command == "extract":
        output_dir = Path(argv[3])
        name_pattern = None
        if len(argv) > 4:
            name_pattern = re.compile(argv[4], re.I)
        extract_files(archive, entries, output_dir, name_pattern)


if __name__ == "__main__":
    main(sys.argv)
