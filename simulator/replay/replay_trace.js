// This file holds the trace of the replay page and the functions that read it. It is the first
// script of the page, so the other scripts (replay_map.js, replay_strip.js, replay_side.js and
// replay.js) use its names. replay_template.html loads it, and make_replay.py copies it
// into a script element of the saved page, so that the page is one file.

// Trace: make_replay.py replaces the comment and the `null` after `TRACE =` on the
// next line with the trace, so that line must keep its text. The trace is the JSON that
// `build/sim.exe --trace` writes (simulator/src/sim.c and simulator/src/metrics.c). It has
// `frame_dt`, the time in seconds between two frames, and `runs`. Each run has these keys:
// - `name` is the scenario, `mode` is the movement mode, `desc` describes the scenario, and
//   `cell` is the size of a map cell in meters.
// - `units` holds `squad`, `r`, the radius in meters, and `v`, the top speed in meters per
//   second, for each unit. Traces from older builds have no `v`.
// - `frames` holds the frames, laid out as the comment above `FRAME_TIME_INDEX` describes.
// - `squads` holds, for each squad, `type` (the unit type), `moving` (0 for a parked squad),
//   `route`, `parking` (the final parking positions, where the members stop at the destination),
//   `lanes` and the numbers whose keys are in `TABLE_ROWS` (replay_side.js).
// - `grid` holds the cells of the map around the area that the run uses, as `drawCells`
//   (replay_map.js) describes.
const TRACE = /*TRACE_JSON*/null;

// These match the data-mode of the buttons. "original" (the original game mode) and "coord" are
// `MODE_NAME` values in simulator/src/sim.c, and "new" adds the switches of
// `NEW_FEATURE_SWITCHES` in build_replay.sh to coord.
const MODES = ["original", "coord", "new"];
const DEFAULT_MODE = "original"; // A run or a page address without a mode means this mode.

// A frame is a flat array that `write_trace_frame` in simulator/src/metrics.c writes. It holds the
// time in seconds, then `NUMBERS_PER_UNIT` numbers for each unit, in the order of `run.units`.
// `UNIT_FIELD` gives the index of each number among them. The first three numbers are x and y in
// meters, with y pointing up, and the heading in radians, which points along (sin heading,
// cos heading). Then come the x and y of the point that the unit drives to, which is its current
// base game waypoint, and the state bits of the unit.
const FRAME_TIME_INDEX = 0;
const FIRST_UNIT_INDEX = 1;
const NUMBERS_PER_UNIT = 6;
const UNIT_FIELD = { x: 0, y: 1, heading: 2, waypointX: 3, waypointY: 4, bits: 5 };

// `write_trace_frame` packs the flags of a unit into these state bits. An avoidance turn is a turn
// of the base game avoidance (AvoidUnits) at full lock, the sharpest turn of the unit, around a
// unit in the way. The ORCA avoidance (optimal reciprocal collision avoidance) of formation
// following changes the velocities of units so that they do not touch.
const STOPPED_BIT = 1; // The unit counts as stopped.
const WAITING_BIT = 2; // The unit waits 2 s for a faster unit.
const AVOIDANCE_TURN_BIT = 4; // The unit makes an avoidance turn.
const TURNING_ON_SPOT_BIT = 8; // The unit turns on the spot.
const CONTACT_BIT = 16; // The unit touches another unit.
const ORCA_AVOIDING_BIT = 32; // The ORCA avoidance changes the speed or direction of the unit.

// ------------------------------------------------------------------------------------------------
// `readFrameTime`, `readUnitNumber`, `readUnitX`, `readUnitY` and `readUnitBits` read the frames
// of the trace. `field` is one of the values in `UNIT_FIELD`.
// ------------------------------------------------------------------------------------------------
const readFrameTime = (frame) => frame[FRAME_TIME_INDEX];

function readUnitNumber(frame, unitIndex, field) {
  const numberIndex = FIRST_UNIT_INDEX + unitIndex * NUMBERS_PER_UNIT + field;
  return frame[numberIndex];
}

const readUnitX = (frame, unitIndex) => readUnitNumber(frame, unitIndex, UNIT_FIELD.x);
const readUnitY = (frame, unitIndex) => readUnitNumber(frame, unitIndex, UNIT_FIELD.y);
const readUnitBits = (frame, unitIndex) => readUnitNumber(frame, unitIndex, UNIT_FIELD.bits);

// ------------------------------------------------------------------------------------------------
// `getDuration` returns the length of the shown run in seconds, which is the time of its last
// frame.
// ------------------------------------------------------------------------------------------------
const getDuration = () => readFrameTime(run.frames[run.frames.length - 1]);

// ------------------------------------------------------------------------------------------------
// `clamp` returns `value` limited to the range from `min` to `max`.
// ------------------------------------------------------------------------------------------------
function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

// ------------------------------------------------------------------------------------------------
// `findFramePair` finds the frames just before and just after the moment `timeSeconds`. It takes
// frame k to be at about k times `TRACE.frame_dt`.
// Result: It returns `before` and `after`, the two frames, `beforeIndex`, the index of `before`,
//   and `blend`, from 0 to 1, which tells how far the moment is from `before` toward `after`.
// Edge case: At the last frame and after it, `before` and `after` are both the last frame.
// ------------------------------------------------------------------------------------------------
function findFramePair(timeSeconds) {
  // Find the frame at or before the moment, and the frame after it.
  const frames = run.frames;
  const frameStepSeconds = TRACE.frame_dt;
  const lastFrameIndex = frames.length - 1;
  const unclampedFrameIndex = Math.floor(timeSeconds / frameStepSeconds);
  const frameIndex = clamp(unclampedFrameIndex, 0, lastFrameIndex);
  const nextFrameIndex = Math.min(lastFrameIndex, frameIndex + 1);

  // Find how far the moment is between the two frames.
  const sinceFrameSeconds = timeSeconds - readFrameTime(frames[frameIndex]);
  const blend = clamp(sinceFrameSeconds / frameStepSeconds, 0, 1);
  const before = frames[frameIndex];
  const after = frames[nextFrameIndex];
  return { before, after, blend, beforeIndex: frameIndex };
}

// ------------------------------------------------------------------------------------------------
// `interpolateUnitState` returns the state of a unit at the moment of `framePair`, which
// `findFramePair` returns.
// Result: It returns `x`, `y`, `heading`, `waypointX`, `waypointY` and `bits`. The position and
//   the heading are blended between the two frames.
// Edge case: The waypoint and the state bits change in steps, so they are not blended.
//   `interpolateUnitState` takes the waypoint from the earlier frame and the bits from the frame
//   nearer to the moment.
//   TODO: confirm why the waypoint and the bits are taken from different frames.
// ------------------------------------------------------------------------------------------------
function interpolateUnitState(framePair, unitIndex) {
  const { before, after, blend } = framePair;
  const blendUnitNumber = (field) => {
    const beforeValue = readUnitNumber(before, unitIndex, field);
    const afterValue = readUnitNumber(after, unitIndex, field);
    return beforeValue + (afterValue - beforeValue) * blend;
  };

  // Turn the heading the shorter way around.
  const headingBefore = readUnitNumber(before, unitIndex, UNIT_FIELD.heading);
  let headingChange = readUnitNumber(after, unitIndex, UNIT_FIELD.heading) - headingBefore;
  if (headingChange > Math.PI)
    headingChange -= 2 * Math.PI;
  if (headingChange < -Math.PI)
    headingChange += 2 * Math.PI;

  // Blend the position and the heading, and take the waypoint and the bits as they are.
  const nearerFrame = blend < 0.5 ? before : after;
  return {
    x: blendUnitNumber(UNIT_FIELD.x),
    y: blendUnitNumber(UNIT_FIELD.y),
    heading: headingBefore + headingChange * blend,
    waypointX: readUnitNumber(before, unitIndex, UNIT_FIELD.waypointX),
    waypointY: readUnitNumber(before, unitIndex, UNIT_FIELD.waypointY),
    bits: readUnitBits(nearerFrame, unitIndex),
  };
}

// ------------------------------------------------------------------------------------------------
// `findRun` returns the run of the trace with the scenario `name` and the mode `modeName`, or
// undefined when the trace has no such run. A run without a mode counts as a run in
// `DEFAULT_MODE`.
// ------------------------------------------------------------------------------------------------
function findRun(name, modeName) {
  return TRACE.runs.find((candidate) => {
    const candidateMode = candidate.mode || DEFAULT_MODE;
    return candidate.name === name && candidateMode === modeName;
  });
}
