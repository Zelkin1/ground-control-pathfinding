"""This script checks a simulator trace for lanes that cross.

In formation following, each member of a squad drives along its own lane beside the route, and
two lanes of one squad must never cross. `build/sim.exe --trace FILE.json` writes each lane into
the trace as a list of points. Each point is the list [x, y], or null where the lanes do not fit
side by side and the squad drives in column formation, which leaves a gap in the lane. The script
tests every pair of straight pieces of two lanes of a squad, which this file calls segments. It
prints one line for each squad that has lanes. If any lanes cross, it exits with status 1.

Arrays: NumPy tests all segment pairs of two lanes at once. A point array holds one point for
each segment. Its row 0 holds the x coordinates and its row 1 the y coordinates, in meters. In
`count_lane_crossings`, axis 1 lists the segments of the first lane and axis 2 the segments of
the second lane, so each test gives one result for each pair of segments.

Usage: python -I sim/tools/check_lanes.py build/sim_trace.json
"""
import itertools
import json
import sys
from pathlib import Path

import numpy as np

EXIT_SUCCESS = 0  # The script exits with this status when no lanes cross.
EXIT_FAILURE = 1  # The script exits with this status when any lanes cross.


def iterate_segments(lane):
    """`iterate_segments` yields the segments of `lane` as (start, end) pairs of points.

    Edge case: It leaves out each segment that has a null point at either end.
    """
    for start, end in zip(lane, lane[1:]):
        if start is not None and end is not None:
            yield start, end


def split_segment_ends(lane_segments):
    """`split_segment_ends` puts the start points and the end points of the segments of a lane into
    two point arrays.

    Result: It returns the array of start points and the array of end points. Each has the shape
      (2, segment count).
    """
    # Lay the segments out as `segment_array[segment_index, end_index, coordinate_index]`. The
    # reshape also gives this shape to a lane without segments.
    segment_array = np.array(lane_segments, dtype=float).reshape(-1, 2, 2)
    starts = segment_array[:, 0, :].T
    ends = segment_array[:, 1, :].T
    return starts, ends


def are_ranges_apart(first_a, first_b, second_a, second_b):
    """`are_ranges_apart` tells for each pair of ranges of numbers whether they have no number in
    common.

    Inputs: The first ranges run between `first_a` and `first_b`, and the second ranges between
      `second_a` and `second_b`. Either end of a range can be the larger one. The four arrays
      must broadcast together.
    Result: It returns an array of booleans in the broadcast shape.
    """
    is_first_below = np.maximum(first_a, first_b) < np.minimum(second_a, second_b)
    is_second_below = np.maximum(second_a, second_b) < np.minimum(first_a, first_b)
    return is_first_below | is_second_below


# -------------------------------------------------------------------------------------------------
#   These two segments have boxes that are apart on the x axis:
#
#   +-------+
#   |      /|   +-------+
#   |    /  |   |\      |
#   |  /    |   |  \    |
#   |/      |   |    \  |
#   +-------+   |      \|
#               +-------+
# -------------------------------------------------------------------------------------------------
def are_boxes_apart(first_start, first_end, second_start, second_end):
    """`are_boxes_apart` tells for each pair of segments whether their bounding boxes do not
    overlap.

    The bounding box of a segment is the smallest rectangle with sides along the x and y axes that
    holds the segment. When the boxes are apart, the segments cannot cross.
    Inputs: The four arguments are point arrays that broadcast together.
    Result: It returns an array of booleans in the broadcast shape.
    """
    are_x_apart = are_ranges_apart(first_start[0], first_end[0], second_start[0], second_end[0])
    are_y_apart = are_ranges_apart(first_start[1], first_end[1], second_start[1], second_end[1])
    return are_x_apart | are_y_apart


# -------------------------------------------------------------------------------------------------
#   The segments cross, because the two ends of each segment lie on opposite sides of the line
#   through the other segment:
#
#                first_start
#                     \
#   second_start ------\------ second_end
#                       \
#                     first_end
# -------------------------------------------------------------------------------------------------
def do_segments_cross(first_start, first_end, second_start, second_end):
    """`do_segments_cross` tells for each pair of segments whether the two segments cross.

    Inputs: The four arguments are point arrays that broadcast together.
    Result: It returns an array of booleans in the broadcast shape. A value is True when each
      segment has its two ends on opposite sides of the line through the other segment.
    Edge case: An end that lies exactly on the line of the other segment is on neither side, so
      segments that only touch do not cross.
    """
    def compute_side(line_start, line_end, point):
        """`compute_side` tells on which side of the line through `line_start` and `line_end` the
        `point` lies.

        Result: It returns the cross product of the direction of the line and the direction from
          `line_start` to `point`. The value is positive on one side of the line, negative on the
          other side and 0 on the line.
        """
        direction_x = line_end[0] - line_start[0]
        direction_y = line_end[1] - line_start[1]
        to_point_x = point[0] - line_start[0]
        to_point_y = point[1] - line_start[1]
        return direction_x * to_point_y - direction_y * to_point_x

    # Find on which side of the line of the other segment each end lies.
    first_start_side = compute_side(second_start, second_end, first_start)
    first_end_side = compute_side(second_start, second_end, first_end)
    second_start_side = compute_side(first_start, first_end, second_start)
    second_end_side = compute_side(first_start, first_end, second_end)

    # Two ends on opposite sides give a negative product.
    are_first_ends_opposite = first_start_side * first_end_side < 0
    are_second_ends_opposite = second_start_side * second_end_side < 0
    return are_first_ends_opposite & are_second_ends_opposite


def count_lane_crossings(segments_per_lane):
    """`count_lane_crossings` counts the crossings between segments of different lanes.

    Inputs: `segments_per_lane` holds a list of segments for each lane of one squad.
    """
    crossing_count = 0
    for first_lane_segments, second_lane_segments in itertools.combinations(segments_per_lane, 2):
        # Put the segments of the first lane along axis 1 and those of the second lane along
        # axis 2.
        first_starts, first_ends = split_segment_ends(first_lane_segments)
        second_starts, second_ends = split_segment_ends(second_lane_segments)
        first_start = first_starts[:, :, np.newaxis]
        first_end = first_ends[:, :, np.newaxis]
        second_start = second_starts[:, np.newaxis, :]
        second_end = second_ends[:, np.newaxis, :]

        # Count the pairs that cross and whose boxes overlap. Segments whose boxes are apart
        # cannot cross. So the box test changes the count only where rounding makes the side
        # test report a crossing that is not there.
        are_apart = are_boxes_apart(first_start, first_end, second_start, second_end)
        do_cross = do_segments_cross(first_start, first_end, second_start, second_end)
        crossing_count += int(np.count_nonzero(do_cross & ~are_apart))
    return crossing_count


def main(trace_path):
    """`main` checks the trace at `trace_path`, prints one line per squad with lanes, and exits."""
    # Read the trace.
    trace_text = Path(trace_path).read_text(encoding="utf-8")
    trace = json.loads(trace_text)

    # Check every squad that has lanes. Only the moving squads of formation-following runs with
    # lanes turned on have them.
    crossing_total = 0
    for run in trace["runs"]:
        for squad_index, squad in enumerate(run["squads"]):
            lanes = squad.get("lanes")
            if not lanes:
                continue
            segments_per_lane = [list(iterate_segments(lane)) for lane in lanes]
            crossing_count = count_lane_crossings(segments_per_lane)
            segment_count = sum(len(lane_segments) for lane_segments in segments_per_lane)
            lane_count = len(lanes)
            squad_text = f"{run['name']:<9} squad {squad_index}: {lane_count} lanes"
            print(f"{squad_text}, {segment_count} segments, {crossing_count} crossings")
            crossing_total += crossing_count

    # Fail when any lanes cross.
    sys.exit(EXIT_FAILURE if crossing_total else EXIT_SUCCESS)


if __name__ == "__main__":
    main(sys.argv[1])
