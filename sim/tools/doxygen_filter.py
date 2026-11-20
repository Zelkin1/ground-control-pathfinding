"""This script turns the plain C comments of the simulator into Doxygen comments while Doxygen
reads a source file, without changing the file. Doxygen runs it as its input filter
(`INPUT_FILTER`): it passes the path of one source file, and the script prints the filtered text.

Line numbers: The output has exactly the lines of the input, so the line numbers that Doxygen
shows stay right.

Rules:
- The first comment of the file becomes the file documentation: `/*` becomes `/**`, and `@file`
  goes before its closing `*/`.
- A comment that starts at column 0 and ends on the line right before code documents that code:
  `/*` becomes `/**`. The separator lines of a framed comment (a line of dashes) lose their
  dashes.
- In a .c file, the comment above a public (not `static`) definition gets `@details`: the
  header holds the usage of that function, whose first sentence must stay the brief, and the
  .c comment adds how it works.
- A comment that follows code on the same line documents that struct field, constant or enum
  value: `/*` becomes `/**<`.
- All other comments stay plain, so Doxygen skips them: step comments inside functions, comments
  inside structs that cover a group of fields, and section comments followed by a blank line.

Usage: python -I sim/tools/doxygen_filter.py FILE (sim/tools/run_doxygen.py sets it up)
"""
import re
import sys
from pathlib import Path

DASH_RUN = re.compile(r"-{20,}")  # A separator line of a framed comment holds such a run.


def find_comment_start(line):
    """`find_comment_start` returns the column where a block comment starts on `line`, outside of
    strings, or -1."""
    is_in_string = False
    for column, character in enumerate(line):
        if character == '"' and (column == 0 or line[column - 1] != "\\"):
            is_in_string = not is_in_string
        elif not is_in_string and line.startswith("/*", column):
            return column
    return -1


def find_comment_end(lines, start_index, start_column):
    """`find_comment_end` returns the index of the line where the comment that starts at the given
    place ends."""
    end_column = lines[start_index].find("*/", start_column + 2)
    if end_column >= 0:
        return start_index
    for line_index in range(start_index + 1, len(lines)):
        if "*/" in lines[line_index]:
            return line_index
    return len(lines) - 1


def is_code_line(line):
    """`is_code_line` returns True when `line` holds code, that is, it is neither blank nor a
    comment start."""
    stripped = line.strip()
    return bool(stripped) and not stripped.startswith(("/*", "//", "*"))


def drop_separator(line):
    """`drop_separator` returns a framed separator line without its run of dashes."""
    return DASH_RUN.sub("", line).rstrip() if DASH_RUN.search(line) else line


def filter_lines(lines, is_source_file):
    """`filter_lines` returns `lines` with their comments changed by the rules at the top of this
    file.

    Inputs: `is_source_file` is True for a .c file and False for a header.
    """
    output = list(lines)
    has_file_comment = False
    line_index = 0
    while line_index < len(lines):
        line = lines[line_index]
        start_column = find_comment_start(line)
        if start_column < 0:
            line_index += 1
            continue
        end_index = find_comment_end(lines, line_index, start_column)
        before = line[:start_column]

        # Document a struct field or a constant with the comment that follows it on its line.
        if before.strip():
            output[line_index] = before + "/**<" + line[start_column + 2:]
            line_index = end_index + 1
            continue

        # Leave indented comments plain: they are step or group comments.
        is_at_column_zero = start_column == 0
        next_line = lines[end_index + 1] if end_index + 1 < len(lines) else ""
        is_file_comment = not has_file_comment and is_at_column_zero
        is_above_code = is_at_column_zero and is_code_line(next_line)
        if is_file_comment or is_above_code:
            is_public_definition = is_source_file and not next_line.startswith("static")
            opener = "/** @details" if is_above_code and is_public_definition else "/**"
            if is_file_comment:
                opener = "/**"
            output[line_index] = drop_separator(opener + line[2:])
            for inner_index in range(line_index + 1, end_index + 1):
                output[inner_index] = drop_separator(lines[inner_index])
            if is_file_comment:
                closing = output[end_index]
                close_column = closing.rfind("*/")
                output[end_index] = closing[:close_column] + " @file " + closing[close_column:]
            has_file_comment = True
        line_index = end_index + 1
    return output


source_path = Path(sys.argv[1])
source = source_path.read_text(encoding="utf-8")
filtered = filter_lines(source.split("\n"), source_path.suffix == ".c")
sys.stdout.write("\n".join(filtered))
