"""This script summarizes a log that PathFix wrote at `LogLevel=2`.

PathFix, the mod DLL that hooked the path search of the game, logged every search and every
formation change at `LogLevel=2`. The summary shows the totals, the destinations with the most
searches and the busiest seconds. Many searches to the same destination show a unit that
searches again and again, in a loop. The patterns below must match the lines that PathFix wrote,
and gridview.py reads the same search lines.

Usage: python -I tools/pathfix/logstats.py <log> [x y radius]
With x, y and radius, the script also lists the searches that start within `radius` meters of
(x, y).
"""
import collections
import re
import sys

# Each line starts with a timestamp, the time of day as "hh:mm:ss.mmm".
# - A search line then holds the terrain class, a letter for the type of search, the corner gap
#   (logged as "clr"), the start and the destination. The terrain class is the number that says
#   which cells block the unit. The corner gap is the gap that the search keeps from the corners
#   of blocking terrain. Last come the number of route corners, the route length, the number of
#   expansions (the nodes that the A* search took from its open list), the search time and any
#   flags, such as "cached".
# - A formation line holds the member count, the formation type, the terrain class, the position,
#   the radius and the number of final parking positions that moved (logged as "slots"). The
#   final parking positions are the positions that the formation layout of the game gives.
TIME_PATTERN = r"(\d\d):(\d\d):(\d\d)\.(\d+)"  # It captures the four parts of a timestamp.
POINT_PATTERN = r"\(([\d.-]+),([\d.-]+)\)"  # A world position is logged as "(x,y)".
SEARCH_REQUEST_PATTERN = rf"path cls=(\d) (\w) clr=([\d.]+) {POINT_PATTERN}->{POINT_PATTERN}"
SEARCH_RESULT_PATTERN = r"pts=(\d+) len=(\d+) exp=(\d+) ([\d.]+)ms(.*)"
SEARCH_LINE_PATTERN = rf"^{TIME_PATTERN} {SEARCH_REQUEST_PATTERN} {SEARCH_RESULT_PATTERN}$"
SEARCH_PATTERN = re.compile(SEARCH_LINE_PATTERN)
FORMATION_SQUAD_PATTERN = r"formation n=(\d+) type=(\d+) cls=(\d)"
FORMATION_RESULT_PATTERN = rf"at {POINT_PATTERN} radius=([\d.]+): (\d+) slots moved"
FORMATION_LINE_PATTERN = rf"^{TIME_PATTERN} {FORMATION_SQUAD_PATTERN} {FORMATION_RESULT_PATTERN}"
FORMATION_PATTERN = re.compile(FORMATION_LINE_PATTERN)
TIMESTAMP_LENGTH = 12  # A timestamp has this many characters.
SECONDS_PER_MINUTE = 60
SECONDS_PER_HOUR = 3600
MINUTES_PER_HOUR = 60
HOURS_PER_DAY = 24
MILLISECONDS_PER_SECOND = 1000
DESTINATION_DECIMALS = 1  # Destinations that are the same to this many decimals count as one.

# The summary lists this many entries of each type.
TOP_CORNER_GAP_COUNT = 12
TOP_RADIUS_COUNT = 10
TOP_DESTINATION_COUNT = 15
BUSIEST_SECOND_COUNT = 8


def convert_to_seconds(hours, minutes, seconds, milliseconds):
    """`convert_to_seconds` returns the time of day in seconds from the digits of a timestamp."""
    hour_seconds = int(hours) * SECONDS_PER_HOUR
    minute_seconds = int(minutes) * SECONDS_PER_MINUTE
    whole_seconds = hour_seconds + minute_seconds + int(seconds)
    return whole_seconds + int(milliseconds) / MILLISECONDS_PER_SECOND


def parse_search(match):
    """`parse_search` returns a search line that `SEARCH_PATTERN` matched as a dict.

    Result: The keys name the logged fields in full words. `time_seconds` is the time of day in
      seconds, `mode` is the letter for the type of search, `corner_gap` is the logged "clr",
      and `route_length`, `expansion_count` and `duration_milliseconds` are as logged. `flags`
      is the rest of the line without the spaces around it.
    """
    hours, minutes, seconds, milliseconds, *search_fields = match.groups()
    terrain_class, mode, corner_gap, *route_fields = search_fields
    start_x, start_y, destination_x, destination_y, *result_fields = route_fields
    point_count, route_length, expansion_count, duration_milliseconds, flags = result_fields
    return {
        "time_seconds": convert_to_seconds(hours, minutes, seconds, milliseconds),
        "terrain_class": int(terrain_class),
        "mode": mode,
        "corner_gap": float(corner_gap),
        "start_x": float(start_x),
        "start_y": float(start_y),
        "destination_x": float(destination_x),
        "destination_y": float(destination_y),
        "point_count": int(point_count),
        "route_length": int(route_length),
        "expansion_count": int(expansion_count),
        "duration_milliseconds": float(duration_milliseconds),
        "flags": flags.strip(),
    }


def parse_formation(match):
    """`parse_formation` returns a formation line that `FORMATION_PATTERN` matched as a dict.

    Result: The keys name the logged fields in full words, and `time_seconds` is the time of day
      in seconds.
    """
    hours, minutes, seconds, milliseconds, *formation_fields = match.groups()
    member_count, formation_type, terrain_class, x, y, radius, moved_count = formation_fields
    return {
        "time_seconds": convert_to_seconds(hours, minutes, seconds, milliseconds),
        "member_count": int(member_count),
        "formation_type": int(formation_type),
        "terrain_class": int(terrain_class),
        "x": float(x),
        "y": float(y),
        "radius": float(radius),
        "moved_count": int(moved_count),
    }


def format_time_of_day(time_seconds):
    """`format_time_of_day` returns "hh:mm:ss" for a whole number of seconds."""
    hours = time_seconds // SECONDS_PER_HOUR % HOURS_PER_DAY
    minutes = time_seconds // SECONDS_PER_MINUTE % MINUTES_PER_HOUR
    seconds = time_seconds % SECONDS_PER_MINUTE
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def format_search(search):
    """`format_search` returns one line with the fields of `search`, a dict from `parse_search`."""
    class_text = f"{search['time_seconds']:.3f} cls={search['terrain_class']} {search['mode']}"
    start_text = f"({search['start_x']},{search['start_y']})"
    destination_text = f"({search['destination_x']},{search['destination_y']})"
    route_text = f"clr={search['corner_gap']} {start_text}->{destination_text}"
    count_text = f"pts={search['point_count']} len={search['route_length']}"
    result_text = f"{count_text} exp={search['expansion_count']}"
    return f"  {class_text} {route_text} {result_text} {search['flags']}"


def format_destination(destination, destination_searches):
    """`format_destination` returns one line about the searches to one destination.

    Inputs: `destination` is `(terrain class, x, y)`, and `destination_searches` holds the dicts
      from `parse_search` of the searches to it, in log order.
    Result: The line gives the number of searches, the seconds from the first to the last, the
      number of starts that differ when rounded to whole meters, the corner gap of the first
      search, and the shortest and the longest route length.
    """
    # Measure the searches.
    terrain_class, destination_x, destination_y = destination
    first_search = destination_searches[0]
    last_search = destination_searches[-1]
    span_seconds = last_search["time_seconds"] - first_search["time_seconds"]
    rounded_starts = set()
    for search in destination_searches:
        rounded_starts.add((round(search["start_x"]), round(search["start_y"])))
    route_lengths = [search["route_length"] for search in destination_searches]
    shortest_length = min(route_lengths)
    longest_length = max(route_lengths)

    # Build the line.
    destination_text = f"  cls={terrain_class} dest=({destination_x},{destination_y})"
    count_text = f"{len(destination_searches)} searches over {span_seconds:.1f}s,"
    start_text = f"{len(rounded_starts)} distinct starts,"
    length_text = f"clr={first_search['corner_gap']} lens {shortest_length}..{longest_length}"
    return " ".join([destination_text, count_text, start_text, length_text])


# Read the searches, the formations and the times of the map loads from the log.
searches = []
formations = []
map_load_times = []
for line in open(sys.argv[1], encoding="latin1"):
    line = line.strip()
    match = SEARCH_PATTERN.match(line)
    if match:
        searches.append(parse_search(match))
        continue
    match = FORMATION_PATTERN.match(line)
    if match:
        formations.append(parse_formation(match))
        continue
    if "map grid loaded" in line:
        map_load_times.append(line[:TIMESTAMP_LENGTH])

# Print the totals.
print("maps loaded at", map_load_times)
print("searches", len(searches), "formations", len(formations))
mode_keys = [(search["terrain_class"], search["mode"]) for search in searches]
mode_counts = collections.Counter(mode_keys)
print("modes", mode_counts)
corner_gap_keys = [(search["terrain_class"], search["corner_gap"]) for search in searches]
corner_gap_counts = collections.Counter(corner_gap_keys)
print("clearances", corner_gap_counts.most_common(TOP_CORNER_GAP_COUNT))
print("flags", collections.Counter(search["flags"] for search in searches))

# Print the most formation changes in one second, the number of changes that moved final parking
# positions, and the radii by terrain class.
formation_seconds = [int(formation["time_seconds"]) for formation in formations]
formations_per_second = collections.Counter(formation_seconds)
max_formations_per_second = 0
if formations_per_second:
    max_formations_per_second = max(formations_per_second.values())
moved_formation_count = sum(1 for formation in formations if formation["moved_count"])
print("formations: max per second", max_formations_per_second, " moved>0:", moved_formation_count)
radius_keys = [(formation["terrain_class"], formation["radius"]) for formation in formations]
radius_counts = collections.Counter(radius_keys)
print("formation radii by class", radius_counts.most_common(TOP_RADIUS_COUNT))

# Group the searches by terrain class and destination.
searches_by_destination = collections.defaultdict(list)
for search in searches:
    rounded_x = round(search["destination_x"], DESTINATION_DECIMALS)
    rounded_y = round(search["destination_y"], DESTINATION_DECIMALS)
    destination = (search["terrain_class"], rounded_x, rounded_y)
    searches_by_destination[destination].append(search)

# Print the destinations with the most searches.
print("\ndestinations with the most searches:")
destination_items = searches_by_destination.items()
sorted_destinations = sorted(destination_items, key=lambda item: -len(item[1]))
for destination, destination_searches in sorted_destinations[:TOP_DESTINATION_COUNT]:
    print(format_destination(destination, destination_searches))

# Print the seconds with the most searches.
search_seconds = [int(search["time_seconds"]) for search in searches]
searches_per_second = collections.Counter(search_seconds)
busiest_seconds = []
for second, search_count in searches_per_second.most_common(BUSIEST_SECOND_COUNT):
    busiest_seconds.append((format_time_of_day(second), search_count))
print("\nbusiest seconds:", busiest_seconds)

# List the searches that start within `radius` of the given point.
if len(sys.argv) > 2:
    center_x, center_y, radius = map(float, sys.argv[2:5])
    print(f"\nsearches starting within {radius} of ({center_x},{center_y}):")
    radius_squared = radius * radius
    for search in searches:
        offset_x = search["start_x"] - center_x
        offset_y = search["start_y"] - center_y
        distance_squared = offset_x ** 2 + offset_y ** 2
        if distance_squared <= radius_squared:
            print(format_search(search))
