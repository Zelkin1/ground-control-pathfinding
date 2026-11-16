// This file draws the strip under the time slider of the replay page: the bars of touching units
// and avoidance, and the speed share of each squad. It is the third script of the page. It uses
// the trace readers of replay_trace.js, the canvas helpers of replay_map.js and the state of
// replay.js (`run` and `timeSeconds`), and `draw` in replay.js calls `drawStrip`.
// replay_template.html loads it, and sim/replay/make_replay.py copies it into a script element of
// the saved page, so that the page is one file.

const stripCanvas = getElement("strip");

// These values size the strip under the time slider.
const FALLBACK_STRIP_WIDTH_PIXELS = 600; // The strip has this width while it has no layout width.
const STRIP_HEIGHT_PIXELS = 64; // This matches the height attribute of #strip.
const SPEED_LINE_WIDTH_PIXELS = 1.5; // The speed line of each squad on the strip is this wide.
const STRIP_TOP_GAP_PIXELS = 4; // The highest bar leaves this much room at the top.
const MIN_STRIP_BAR_HEIGHT_PIXELS = 2;
const MIN_STRIP_BAR_WIDTH_PIXELS = 1;
const TIME_CURSOR_WIDTH_PIXELS = 2;

// ------------------------------------------------------------------------------------------------
// `countUnitsWithBit` returns how many units have the state bit `bit` in `frame`.
// ------------------------------------------------------------------------------------------------
function countUnitsWithBit(frame, bit) {
  let count = 0;
  for (let unitIndex = 0; unitIndex < run.units.length; unitIndex++) {
    if (readUnitBits(frame, unitIndex) & bit)
      count++;
  }
  return count;
}

// ------------------------------------------------------------------------------------------------
// `drawStripSegment` draws one part of a strip bar in `color`, `segmentHeight` high, with its
// bottom at `bottomY`. A part higher than 0 is at least `MIN_STRIP_BAR_HEIGHT_PIXELS` high, so
// that a single unit shows.
// ------------------------------------------------------------------------------------------------
function drawStripSegment(context, barX, barWidth, bottomY, segmentHeight, color) {
  if (segmentHeight <= 0)
    return;
  const drawnHeight = Math.max(MIN_STRIP_BAR_HEIGHT_PIXELS, segmentHeight);
  context.fillStyle = color;
  context.fillRect(barX, bottomY - drawnHeight, barWidth, drawnHeight);
}

// ------------------------------------------------------------------------------------------------
// `findSpeedShares` returns, for each frame, the mean over the members of squad `squadIndex` of
// their speed as a share of their top speed `v`, the speed of the unit kind in a straight line
// without turns or obstacles. The speed of a unit at a frame is the distance that it moves
// from the frame before to the frame after, divided by the time between those frames. The first
// and the last frame use the one step that they have.
// ------------------------------------------------------------------------------------------------
function findSpeedShares(squadIndex) {
  // List the indexes of the members of the squad in `run.units`.
  const frames = run.frames;
  const memberIndexes = [];
  run.units.forEach((unit, unitIndex) => {
    if (unit.squad === squadIndex)
      memberIndexes.push(unitIndex);
  });

  // Average the speed shares of the members at each frame.
  return frames.map((frame, frameIndex) => {
    const before = frames[Math.max(0, frameIndex - 1)];
    const after = frames[Math.min(frames.length - 1, frameIndex + 1)];
    const stepSeconds = readFrameTime(after) - readFrameTime(before);
    if (stepSeconds <= 0)
      return 0;
    let shareSum = 0;
    for (const unitIndex of memberIndexes) {
      const moveX = readUnitX(after, unitIndex) - readUnitX(before, unitIndex);
      const moveY = readUnitY(after, unitIndex) - readUnitY(before, unitIndex);
      const speed = Math.hypot(moveX, moveY) / stepSeconds;
      shareSum += speed / run.units[unitIndex].v;
    }
    return shareSum / memberIndexes.length;
  });
}

// `speedLinesByRun` keeps, for each run, the speed shares of its moving squads, so that the strip
// works them out only once. Each entry holds the index of the squad and its shares.
const speedLinesByRun = new Map();

// ------------------------------------------------------------------------------------------------
// `getSpeedLines` returns the speed lines of the moving squads of the shown run.
// Edge case: A trace without `v` gives no lines.
// ------------------------------------------------------------------------------------------------
function getSpeedLines() {
  if (!speedLinesByRun.has(run)) {
    const hasTopSpeeds = run.units.every((unit) => unit.v > 0);
    const lines = [];
    run.squads.forEach((squad, squadIndex) => {
      if (hasTopSpeeds && squad.moving)
        lines.push({ squadIndex, shares: findSpeedShares(squadIndex) });
    });
    speedLinesByRun.set(run, lines);
  }
  return speedLinesByRun.get(run);
}

// ------------------------------------------------------------------------------------------------
// `drawSpeedLines` draws a line for each moving squad, in the color of the squad, of the mean
// speed of its members as a share of their top speed: 0 at the bottom of the strip and 1 at the
// height of the highest bar. A share above 1, from collisions, is drawn at 1.
// ------------------------------------------------------------------------------------------------
function drawSpeedLines(context, width, height) {
  const frameCount = run.frames.length;
  const topY = STRIP_TOP_GAP_PIXELS;
  for (const line of getSpeedLines()) {
    setLineStyle(context, getSquadColor(line.squadIndex), SPEED_LINE_WIDTH_PIXELS);
    context.beginPath();
    line.shares.forEach((share, frameIndex) => {
      // Put the point of each frame at the middle of its slice of the strip width.
      const lineX = ((frameIndex + 0.5) / frameCount) * width;
      const lineY = height - Math.min(share, 1) * (height - topY);
      if (frameIndex === 0)
        context.moveTo(lineX, lineY);
      else
        context.lineTo(lineX, lineY);
    });
    context.stroke();
  }
}

// ------------------------------------------------------------------------------------------------
// `drawStrip` draws the strip under the time slider and a line at the current time. It has a bar
// for each frame: at the bottom, in the contact color, the number of units that touch another
// unit, and on top of it, in the adjust color, the number of units whose speed or direction the
// avoidance changes. Over the bars, `drawSpeedLines` draws the speed of each moving squad.
// ------------------------------------------------------------------------------------------------
function drawStrip() {
  // Size and clear the strip.
  const width = stripCanvas.clientWidth || FALLBACK_STRIP_WIDTH_PIXELS;
  const height = STRIP_HEIGHT_PIXELS;
  const context = sizeCanvas(stripCanvas, width, height);
  context.clearRect(0, 0, width, height);

  // Count both kinds of units in each frame, and find the highest total.
  const frames = run.frames;
  const contactCounts = frames.map((frame) => countUnitsWithBit(frame, CONTACT_BIT));
  const adjustCounts = frames.map((frame) => countUnitsWithBit(frame, ADJUSTING_BIT));
  let peakCount = 1;
  contactCounts.forEach((contactCount, frameIndex) => {
    peakCount = Math.max(peakCount, contactCount + adjustCounts[frameIndex]);
  });

  // Draw the bars, scaled to the highest total.
  const pixelsPerUnit = (height - STRIP_TOP_GAP_PIXELS) / peakCount;
  const barWidth = Math.max(MIN_STRIP_BAR_WIDTH_PIXELS, width / frames.length);
  contactCounts.forEach((contactCount, frameIndex) => {
    const barX = (frameIndex / frames.length) * width;
    const contactHeight = contactCount * pixelsPerUnit;
    const adjustHeight = adjustCounts[frameIndex] * pixelsPerUnit;
    drawStripSegment(context, barX, barWidth, height, contactHeight, colors.contact);
    const adjustBottom = height - contactHeight;
    drawStripSegment(context, barX, barWidth, adjustBottom, adjustHeight, colors.adjust);
  });
  drawSpeedLines(context, width, height);

  // Draw the current time.
  context.fillStyle = colors.ink;
  const cursorX = (timeSeconds / getDuration()) * width - TIME_CURSOR_WIDTH_PIXELS / 2;
  context.fillRect(cursorX, 0, TIME_CURSOR_WIDTH_PIXELS, height);
}
