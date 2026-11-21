"""This script builds the replay page from a simulator trace.

Usage: python -I simulator/replay/make_replay.py [--leave-out <names>] <trace.json> <out.html>
           [<extra.json> <suffix> <note> <mode>]...

`build/sim.exe --trace <trace.json>` writes the trace. `replay_template.html` links its styles in
`replay.css` and its script in the five files of `SCRIPT_NAMES`. This script copies each file into
the template in place of the tag that links it, and it puts the trace into the script of the page.
It saves the result as one page that a browser can open on its own.

Each group of four extra arguments adds the runs of another trace, for example one written with
`--seed 99 ridge`. Each of its runs gets `suffix` added to its name after a space, so that the page
shows it on a tab of its own, and `note` added to its description after a space. Its runs also
take `mode` as their mode, so that a trace written with other switches, such as
`--avoid coord --widebends off`, can fill a mode button of its own. An empty argument keeps the
names, the descriptions or the modes as they are. The page splits its address at "-", so `suffix`
and `mode` must not hold one.

`--leave-out` drops every run of the scenarios `names`, separated by commas, from all traces, so
that a page with many runs can stay under the size limit of one published file.

An output path that ends with ".json" gets the merged trace alone, as a data file of a faction tab
that the page loads from beside itself (`FACTIONS` in replay.js).
"""
import json
import sys
from pathlib import Path

REPLAY_FOLDER = Path(__file__).parent  # The template, styles and script sit beside this file.
TEMPLATE_PATH = REPLAY_FOLDER / "replay_template.html"
STYLE_PATH = REPLAY_FOLDER / "replay.css"
# The scripts share one global scope, so they must stay in the order of the tags of the template.
SCRIPT_NAMES = [
    "replay_trace.js",
    "replay_map.js",
    "replay_strip.js",
    "replay_side.js",
    "replay.js",
]
STYLE_LINK = '<link rel="stylesheet" href="replay.css">'  # The styles replace this tag.
TRACE_PLACEHOLDER = "/*TRACE_JSON*/null"  # The trace replaces this text in replay_trace.js.
EXTRA_GROUP_SIZE = 4  # Each extra trace comes with its path, suffix, note and mode.
LEAVE_OUT_OPTION = "--leave-out"  # Followed by the names of the scenarios to leave out.

# Read the optional scenarios to leave out, then the paths.
arguments = sys.argv[1:]
left_out_names = set()
if arguments[0] == LEAVE_OUT_OPTION:
    left_out_names = set(arguments[1].split(","))
    arguments = arguments[2:]
trace_path = Path(arguments[0])
output_path = arguments[1]

# Read the trace. Add the runs of each extra trace, renamed, after its own runs. All traces must
# use the same time between frames, because the page reads it once for all runs.
trace = json.loads(trace_path.read_text(encoding="utf-8"))
extra_arguments = arguments[2:]
for group_start in range(0, len(extra_arguments), EXTRA_GROUP_SIZE):
    group = extra_arguments[group_start:group_start + EXTRA_GROUP_SIZE]
    extra_path, suffix, note, mode = group
    extra_text = Path(extra_path).read_text(encoding="utf-8")
    extra_trace = json.loads(extra_text)
    if extra_trace["frame_dt"] != trace["frame_dt"]:
        sys.exit(f"{extra_path} has another time between frames")
    for extra_run in extra_trace["runs"]:
        if suffix:
            extra_run["name"] = f"{extra_run['name']} {suffix}"
        if note:
            extra_run["desc"] = f"{extra_run['desc']} {note}"
        if mode:
            extra_run["mode"] = mode
        trace["runs"].append(extra_run)

# A run name starts with the name of its scenario, which holds no space.
trace["runs"] = [run for run in trace["runs"] if run["name"].split()[0] not in left_out_names]

# A data file of a faction tab holds only the merged trace.
if output_path.endswith(".json"):
    Path(output_path).write_text(json.dumps(trace, separators=(",", ":")), encoding="utf-8")
    print(f"wrote {output_path}")
    sys.exit()

# Read the template and the styles.
template = TEMPLATE_PATH.read_text(encoding="utf-8")
style = STYLE_PATH.read_text(encoding="utf-8")

# Write the trace as compact JSON. Change each "</" into "<\/", which JavaScript reads as the same
# text, so that no "</script>" in the trace can end the script of the page early.
trace_json = json.dumps(trace, separators=(",", ":")).replace("</", "<\\/")

# Put the styles and each script into the template in place of their tags. All files end with a
# line break, so each closing tag starts a new line.
page = template.replace(STYLE_LINK, f"<style>\n{style}</style>")
for script_name in SCRIPT_NAMES:
    script = (REPLAY_FOLDER / script_name).read_text(encoding="utf-8")
    script_link = f'<script src="{script_name}"></script>'
    page = page.replace(script_link, f"<script>\n{script}</script>")

# Put the trace into the script and save the page.
page = page.replace(TRACE_PLACEHOLDER, trace_json)
Path(output_path).write_text(page, encoding="utf-8")
print(f"wrote {output_path}")
