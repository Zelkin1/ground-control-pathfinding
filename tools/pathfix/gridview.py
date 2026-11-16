"""This tool draws a grid that PathFix dumped, with the searches of a PathFix log, into a PNG.

PathFix, the mod DLL that hooked the path search of the game, could dump the grid of the game to
a file (pathfix_mapN.grid) and log every search at `LogLevel=2`. The picture shows where
searches run next to ground that blocks them.

Usage: python -I tools/pathfix/gridview.py GRID LOG OUT.png [x0 y0 x1 y1]
The optional x0 y0 x1 y1, in cells, crop the picture to x from x0 to x1 and y from y0 to y1. Like
world y, cell y counts up from the bottom of the map.

Cells with the values 0, 1, 2 and 3 are white, light gray, dark gray and black. Each search is a
straight line from its start to its destination: red for class 0, blue for class 1 and green for
class 2. The tool also prints, for each class, how many free cells are 1, 2, or 3 or more cells
away from ground that the class must not use.
"""
import re
import sys

import numpy as np
from PIL import Image, ImageDraw

# The dump is a copy of the grid of the game, which is at 0x6eb47c in gc.exe (see
# re/PATHFINDING.md). It holds 1024 x 1024 cells of 2 bits, 4 cells per byte, with the row y = 0
# first. The cell size must match the cell size of the game, or the logged world positions land
# on the wrong cells.
GRID_WIDTH_CELLS = 1024  # The grid is square, with this many cells on each side.
CELL_SIZE_WORLD_UNITS = 6.25  # Each cell is this wide.
CELLS_PER_BYTE = 4  # Each byte holds this many cells.
BITS_PER_CELL = 2  # Each cell has this many bits.
CELL_VALUE_COUNT = 4  # A cell holds one of this many values, 0 to 3.
CELL_VALUE_MASK = CELL_VALUE_COUNT - 1  # This mask keeps the bits of one cell.
CELL_SHADES = (255, 200, 120, 0)  # These are the gray levels of the cell values 0 to 3.

# A cell blocks a mover when its value is above the class of the mover. Ground units have the
# classes 0 to 2. `CLASS_COLORS` holds the RGB color for the searches of each of these classes.
MOVER_CLASS_COUNT = 3  # The wall statistics cover this many classes, from 0.
CLASS_COLORS = {0: (220, 30, 30), 1: (30, 60, 230), 2: (20, 160, 40)}
OTHER_CLASS_COLOR = (0, 0, 0)  # Searches of other classes are black.
MAX_WALL_DISTANCE_CELLS = 15  # The distance to a wall stops growing at this many cells.
NEIGHBORHOOD_WIDTH_CELLS = 3  # The neighbors of a cell fill a 3 x 3 square around it.
OUTPUT_SIZE_PIXELS = 1024  # A crop is scaled up by a whole number to about this size.

# PathFix logged each search on one line, and logstats.py reads the same lines. The pattern
# captures the mover class, then the start and the destination in world units.
POINT_PATTERN = r"\(([\d.-]+),([\d.-]+)\)"  # A world position is logged as "(x,y)".
SEARCH_PATTERN = re.compile(rf"path cls=(\d) \w clr=[\d.]+ {POINT_PATTERN}->{POINT_PATTERN}")


def load_grid(path):
    """`load_grid` returns the dumped grid as a 1024 x 1024 array of cell values.

    Result: The array is indexed `[y, x]`, in the same layout as the grid of the game.
    """
    # Read the packed bytes, one row of cells after another.
    grid_bytes = open(path, "rb").read()
    packed_bytes = np.frombuffer(grid_bytes, dtype=np.uint8)
    bytes_per_row = GRID_WIDTH_CELLS // CELLS_PER_BYTE
    packed_rows = packed_bytes.reshape(GRID_WIDTH_CELLS, bytes_per_row)

    # Unpack the cells. The first cell of a byte is in its highest bits.
    cells = np.zeros((GRID_WIDTH_CELLS, GRID_WIDTH_CELLS), dtype=np.uint8)
    for cell_index_in_byte in range(CELLS_PER_BYTE):
        shift_bits = BITS_PER_CELL * (CELLS_PER_BYTE - 1 - cell_index_in_byte)
        cell_values = (packed_rows >> shift_bits) & CELL_VALUE_MASK
        cells[:, cell_index_in_byte::CELLS_PER_BYTE] = cell_values
    return cells


def find_neighbor_minimum(wall_distances):
    """`find_neighbor_minimum` returns the lowest value of `wall_distances` around each cell.

    Result: For each cell, the value is the lowest one in the 3 x 3 square around the cell, the
      cell itself included.
    Edge case: Cells outside the grid count as 0, which marks blocked ground.
    """
    # Pad the grid with one cell of 0 on each side, then take the lowest of the 9 shifted copies.
    padded = np.pad(wall_distances, 1, constant_values=0)
    shifted_copies = []
    for row_offset in range(NEIGHBORHOOD_WIDTH_CELLS):
        for column_offset in range(NEIGHBORHOOD_WIDTH_CELLS):
            row_end = row_offset + GRID_WIDTH_CELLS
            column_end = column_offset + GRID_WIDTH_CELLS
            shifted_copies.append(padded[row_offset:row_end, column_offset:column_end])
    return np.minimum.reduce(shifted_copies)


def compute_wall_distances(cells, mover_class):
    """`compute_wall_distances` returns, for each cell, how far it is from blocked ground.

    The distance is the Chebyshev distance: the number of steps in any of the 8 directions from
    the cell to the closest cell that blocks `mover_class`. It is 0 on blocked cells, it stops
    growing at `MAX_WALL_DISTANCE_CELLS`, and ground outside the grid counts as blocked.
    """
    is_blocked = cells > mover_class
    wall_distances = np.where(is_blocked, 0, MAX_WALL_DISTANCE_CELLS).astype(np.int16)
    for _ in range(MAX_WALL_DISTANCE_CELLS):
        # Lower each cell to 1 more than its lowest neighbor, and keep blocked cells at 0.
        neighbor_minimum = find_neighbor_minimum(wall_distances)
        wall_distances = np.minimum(wall_distances, neighbor_minimum + 1)
        wall_distances[is_blocked] = 0
    return wall_distances


def read_searches(log_path):
    """`read_searches` returns the searches in the log.

    Result: Each search is a tuple of floats: the mover class, the start x and y, and the
      destination x and y.
    """
    searches = []
    for line in open(log_path, encoding="latin1"):
        match = SEARCH_PATTERN.search(line)
        if match:
            search_values = map(float, match.groups())
            searches.append(tuple(search_values))
    return searches


# -------------------------------------------------------------------------------------------------
# World y counts up from the bottom of the map, and picture rows count down from the top:
#
#   world, in cells            picture, in pixels
#   y=1024 +-------+           row 0    +-------+
#          |       |                    |       |
#   y=0    +-------+           row 1024 +-------+
#          x=0     x=1024               x=0     x=1024
# -------------------------------------------------------------------------------------------------
def convert_world_to_pixel(world_x, world_y):
    """`convert_world_to_pixel` returns the pixel of a world position in the picture of the map.

    The picture has its first row at the top, so world y points up in it.
    """
    pixel_x = world_x / CELL_SIZE_WORLD_UNITS
    pixel_y = GRID_WIDTH_CELLS - world_y / CELL_SIZE_WORLD_UNITS
    return pixel_x, pixel_y


def draw_map(cells, searches):
    """`draw_map` returns a picture of the whole map, one pixel per cell.

    Result: The cells are gray, and each search is a straight line from its start to its
      destination.
    """
    # Shade the cells. The first row of the picture is at the top, so flip the rows to put y = 0
    # at the bottom.
    cell_shades = np.array(CELL_SHADES, dtype=np.uint8)[cells]
    rgb_pixels = np.stack([cell_shades, cell_shades, cell_shades], axis=-1)
    flipped_pixels = rgb_pixels[::-1].copy()
    image = Image.fromarray(flipped_pixels)

    # Draw the searches.
    draw = ImageDraw.Draw(image)
    for logged_class, start_x, start_y, destination_x, destination_y in searches:
        color = CLASS_COLORS.get(int(logged_class), OTHER_CLASS_COLOR)
        start_pixel = convert_world_to_pixel(start_x, start_y)
        destination_pixel = convert_world_to_pixel(destination_x, destination_y)
        draw.line([start_pixel, destination_pixel], fill=color, width=1)
    return image


def print_wall_statistics(cells):
    """`print_wall_statistics` counts the free cells that are 1, 2, or at least 3 cells from walls.

    `print_wall_statistics` prints the counts in one line for each mover class. For a class, a
    wall is ground that the class must not use.
    """
    for mover_class in range(MOVER_CLASS_COUNT):
        wall_distances = compute_wall_distances(cells, mover_class)
        is_free = wall_distances > 0
        free_count = int(is_free.sum())
        one_cell_count = int((wall_distances == 1).sum())
        two_cells_count = int((wall_distances == 2).sum())
        three_or_more_count = int((wall_distances >= 3).sum())
        free_text = f"class {mover_class}: free cells {free_count},"
        adjacent_text = f"of which next to a wall (d=1) {one_cell_count},"
        print(free_text, adjacent_text, f"d=2 {two_cells_count}, d>=3 {three_or_more_count}")


def main():
    """`main` draws the grid and the log named on the command line into a PNG."""
    # Read the command line. Without a crop, the picture shows the whole grid.
    grid_path, log_path, output_path = sys.argv[1:4]
    crop = [0, 0, GRID_WIDTH_CELLS, GRID_WIDTH_CELLS]
    if len(sys.argv) >= 8:
        crop = list(map(int, sys.argv[4:8]))

    # Load the grid and count its cell values.
    cells = load_grid(grid_path)
    value_counts = {value: int((cells == value).sum()) for value in range(CELL_VALUE_COUNT)}
    print("cell values:", value_counts)

    # Draw the logged searches on the map, and print how far free cells are from walls.
    searches = read_searches(log_path)
    image = draw_map(cells, searches)
    print_wall_statistics(cells)

    # Crop, scale up by a whole number and save. The crop is in cells with y up, and the picture
    # has its first row at the top.
    left, bottom, right, top = crop
    image = image.crop((left, GRID_WIDTH_CELLS - top, right, GRID_WIDTH_CELLS - bottom))
    crop_size_cells = max(right - left, top - bottom)
    scale = max(1, OUTPUT_SIZE_PIXELS // crop_size_cells)
    image = image.resize((image.width * scale, image.height * scale), Image.NEAREST)
    image.save(output_path)
    print("wrote", output_path, image.size, "searches drawn:", len(searches))


if __name__ == "__main__":
    main()
