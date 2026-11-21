// This file draws the map of the replay page: the view, the canvas helpers, the static layer
// (grid, cells, routes, lanes, final parking positions, whole paths and scale bar), and the units
// with their trails and marks. It is the second script of the page. It uses the trace readers of
// replay_trace.js and the state of replay.js (`run`), and replay_strip.js, replay_side.js and
// replay.js use its helpers. replay_template.html loads it, and make_replay.py copies
// it into a script element of the saved page, so that the page is one file.

const getElement = (id) => document.getElementById(id);
const mapCanvas = getElement("map");
const mapContext = mapCanvas.getContext("2d");

const SQUAD_COLOR_COUNT = 3; // Squads take the colors --squad-0 to --squad-2 in turn.

// These values set the map view.
// The view adds this margin around all units, routes and final parking positions.
const VIEW_PADDING_METERS = 30;
const FALLBACK_MAP_WIDTH_PIXELS = 900; // The map has this width while #map has no layout width.
const MIN_MAP_ASPECT = 0.45; // The height of the map is at least this part of its width.
const MAX_MAP_ASPECT = 0.85; // The height of the map is at most this part of its width.

// These values style the static layer: the grid, the cells, the routes, the lanes, the final
// parking positions, the whole paths and the scale bar.
const GRID_LINE_WIDTH_PIXELS = 1;
const MAJOR_GRID_LINE_EVERY = 8; // Lines at multiples of this cell index are darker major lines.
const MIN_CELL_PIXELS_FOR_MINOR_LINES = 4; // With smaller cells, only the major lines are drawn.
const PIXEL_CENTER_OFFSET = 0.5; // A 1-pixel line through pixel centers stays sharp.
const DIGIT_ZERO_CODE = 48; // The trace stores each cell value as a digit. This is the code of 0.
const SLOPE_CELL = 1; // This cell value marks a gentle slope.
const HILL_CELL = 2; // This cell value marks hills, which tanks cannot enter but infantry can.
const CLIFF_CELL = 3; // This cell value marks cliffs, which no unit can enter.
const CELL_OVERLAP_PIXELS = 1; // Cells are drawn this much larger, so that no seams show.
const ROUTE_ALPHA = 0.7;
const ROUTE_LINE_WIDTH_PIXELS = 1.5;
const ROUTE_DASH_PIXELS = [6, 5];
const LANE_ALPHA = 0.55;
const LANE_LINE_WIDTH_PIXELS = 1;
const PARKING_LINE_WIDTH_PIXELS = 1.2;
const PARKING_CROSS_HALF_SIZE_PIXELS = 4;
const WHOLE_PATH_ALPHA = 0.22;
const WHOLE_PATH_LINE_WIDTH_PIXELS = 1;
const SCALE_BAR_METERS = 50;
const SCALE_BAR_LEFT_PIXELS = 12;
const SCALE_BAR_BOTTOM_PIXELS = 14;
const SCALE_BAR_TICK_PIXELS = 4; // The end ticks reach this far above and below the bar.
const SCALE_BAR_LINE_WIDTH_PIXELS = 2;
const SCALE_LABEL_GAP_PIXELS = 6; // The label starts this far right of the end of the bar.
const SCALE_LABEL_DROP_PIXELS = 6; // The bottom of the label is this far below the bar.
const SCALE_LABEL_FONT_PIXELS = 12;

// These values style the units, their trails and the marks for their state bits.
const TRAIL_ALPHA = 0.75;
const TRAIL_LINE_WIDTH_PIXELS = 2;
const WAYPOINT_LINE_ALPHA = 0.45;
const WAYPOINT_LINE_WIDTH_PIXELS = 1;
const MIN_UNIT_RADIUS_PIXELS = 3;
const MIN_HEADING_LINE_WIDTH_PIXELS = 1.5;
const HEADING_LINE_WIDTH_FRACTION = 0.28; // The heading line is this part of the radius wide.
const HEADING_LINE_LENGTH_FRACTION = 0.95; // The heading line is this part of the radius long.
const CONTACT_RING_GAP_PIXELS = 3; // The contact ring is this far outside the unit.
const CONTACT_RING_WIDTH_PIXELS = 2.5;
// The marks for turning on the spot, avoidance turns and ORCA avoidance are this far outside the
// unit.
const OUTER_RING_GAP_PIXELS = 6;
const ON_SPOT_RING_WIDTH_PIXELS = 1.2;
const ON_SPOT_RING_DASH_PIXELS = [2, 3];
const AVOIDANCE_TURN_ARC_WIDTH_PIXELS = 2;
const AVOIDANCE_TURN_ARC_HALF_ANGLE_RADIANS = 0.7;
const ORCA_RING_WIDTH_PIXELS = 1.2;
const ORCA_RING_ALPHA = 0.7;
const PAUSE_GAP_PIXELS = 4; // The pause sign is this far right of the unit and this far above it.
const MIN_PAUSE_HEIGHT_PIXELS = 3;
const PAUSE_HEIGHT_FRACTION = 0.5; // The pause sign is this part of the radius of the unit high.
const PAUSE_BAR_WIDTH_FRACTION = 0.35; // Each bar of the sign is this part of its height wide.
const PAUSE_SECOND_BAR_FRACTION = 0.6; // The two bars start this part of the height apart.

let view = null; // This holds the size and scale of the map, from `computeView`.
let staticLayer = null; // `buildStaticLayer` draws into this off-screen canvas.
let colors = {}; // `readColors` copies the theme colors into this object.

// `readColors` copies these custom properties of :root, named without their "--".
const THEME_COLOR_NAMES = [
  "map", "grid", "grid-major", "slope", "rock", "rough", "ink", "muted", "panel", "contact",
  "orca", "squad-0", "squad-1", "squad-2",
];

// ------------------------------------------------------------------------------------------------
// The canvas cannot use var(), so `readColors` copies the theme colors from the custom
// properties on :root into `colors`. The page calls it again when the theme changes.
// ------------------------------------------------------------------------------------------------
function readColors() {
  const rootStyle = getComputedStyle(document.documentElement);
  for (const colorName of THEME_COLOR_NAMES)
    colors[colorName] = rootStyle.getPropertyValue("--" + colorName).trim();
}

const getSquadColor = (squadIndex) => colors["squad-" + (squadIndex % SQUAD_COLOR_COUNT)];

// ------------------------------------------------------------------------------------------------
// `computeView` fits every unit position, route point and final parking position of the shown run
// into the map.
// Result: It returns the `width` and `height` of the map in pixels, `pixelsPerMeter`, and
//   `centerX` and `centerY`, the world point in meters at the center of the map.
// ------------------------------------------------------------------------------------------------
function computeView() {
  // Find the box around every unit position, route point and final parking position.
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  const growBounds = (x, y) => {
    minX = Math.min(minX, x);
    minY = Math.min(minY, y);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  };
  for (const frame of run.frames) {
    for (let unitIndex = 0; unitIndex < run.units.length; unitIndex++)
      growBounds(readUnitX(frame, unitIndex), readUnitY(frame, unitIndex));
  }
  for (const squad of run.squads) {
    squad.route.forEach((point) => growBounds(point[0], point[1]));
    squad.parking.forEach((point) => growBounds(point[0], point[1]));
  }

  // Add free space around the box.
  minX -= VIEW_PADDING_METERS;
  minY -= VIEW_PADDING_METERS;
  maxX += VIEW_PADDING_METERS;
  maxY += VIEW_PADDING_METERS;
  const worldWidthMeters = maxX - minX;
  const worldHeightMeters = maxY - minY;

  // Fit the box into the map, with the same scale on both axes.
  const width = mapCanvas.clientWidth || FALLBACK_MAP_WIDTH_PIXELS;
  const worldAspect = worldHeightMeters / worldWidthMeters;
  const aspect = clamp(worldAspect, MIN_MAP_ASPECT, MAX_MAP_ASPECT);
  const height = Math.round(width * aspect);
  const pixelsPerMeter = Math.min(width / worldWidthMeters, height / worldHeightMeters);
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  return { width, height, pixelsPerMeter, centerX, centerY };
}

// ------------------------------------------------------------------------------------------------
// `getScreenX` and `getScreenY` give the map pixel of a world position in meters. World y points
// up, and screen y points down. They put the world point (`view.centerX`, `view.centerY`) at the
// center of the map:
//
//   world, in meters                     map, in pixels
//   y                                    (0, 0) +-----------> x
//   ^                                           |
//   |    + (centerX, centerY)        -->        |    + (width / 2, height / 2)
//   |                                           |
//   +-----------> x                             v y
// ------------------------------------------------------------------------------------------------
const getScreenX = (worldX) => view.width / 2 + (worldX - view.centerX) * view.pixelsPerMeter;
const getScreenY = (worldY) => view.height / 2 - (worldY - view.centerY) * view.pixelsPerMeter;

// ------------------------------------------------------------------------------------------------
// The canvas helpers below draw in page pixels. A page pixel can cover several screen pixels.
// ------------------------------------------------------------------------------------------------
const getPixelRatio = () => window.devicePixelRatio || 1;
const snapToPixelCenter = (screenCoordinate) => Math.round(screenCoordinate) + PIXEL_CENTER_OFFSET;

// ------------------------------------------------------------------------------------------------
// `applyPixelRatio` makes `context` draw in page pixels on a canvas that has the pixel
// density of the screen.
// ------------------------------------------------------------------------------------------------
function applyPixelRatio(context) {
  const pixelRatio = getPixelRatio();
  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
}

// ------------------------------------------------------------------------------------------------
// `setLineStyle` sets the color, the opacity when `alpha` is given, and the width of the lines
// that `context` draws next.
// ------------------------------------------------------------------------------------------------
function setLineStyle(context, color, lineWidthPixels, alpha) {
  context.strokeStyle = color;
  if (alpha !== undefined)
    context.globalAlpha = alpha;
  context.lineWidth = lineWidthPixels;
}

// ------------------------------------------------------------------------------------------------
// `strokeLine`, `strokeCircle` and `fillCircle` each draw one shape as a new path, in pixels,
// with the current style.
// ------------------------------------------------------------------------------------------------
function strokeLine(context, fromX, fromY, toX, toY) {
  context.beginPath();
  context.moveTo(fromX, fromY);
  context.lineTo(toX, toY);
  context.stroke();
}

function strokeCircle(context, centerX, centerY, radiusPixels) {
  context.beginPath();
  context.arc(centerX, centerY, radiusPixels, 0, 2 * Math.PI);
  context.stroke();
}

function fillCircle(context, centerX, centerY, radiusPixels) {
  context.beginPath();
  context.arc(centerX, centerY, radiusPixels, 0, 2 * Math.PI);
  context.fill();
}

// ------------------------------------------------------------------------------------------------
// `addPathPoint` adds a world point to the current path. When `isFirstPoint` is set, the point
// starts a new part of the path. Otherwise `addPathPoint` adds a line to the point.
// ------------------------------------------------------------------------------------------------
function addPathPoint(context, isFirstPoint, worldX, worldY) {
  const screenX = getScreenX(worldX);
  const screenY = getScreenY(worldY);
  if (isFirstPoint) {
    context.moveTo(screenX, screenY);
  } else {
    context.lineTo(screenX, screenY);
  }
}

// ------------------------------------------------------------------------------------------------
// `addUnitPathPoints` adds the positions of a unit from the first frame up to `lastFrameIndex` to
// the current path.
// ------------------------------------------------------------------------------------------------
function addUnitPathPoints(context, unitIndex, lastFrameIndex) {
  for (let frameIndex = 0; frameIndex <= lastFrameIndex; frameIndex++) {
    const frame = run.frames[frameIndex];
    const worldX = readUnitX(frame, unitIndex);
    const worldY = readUnitY(frame, unitIndex);
    addPathPoint(context, frameIndex === 0, worldX, worldY);
  }
}

// ------------------------------------------------------------------------------------------------
// `drawParkingCross` draws the mark of a final parking position, a small cross on the pixel
// `pixelX`, `pixelY`.
// ------------------------------------------------------------------------------------------------
function drawParkingCross(context, pixelX, pixelY) {
  context.beginPath();
  context.moveTo(pixelX - PARKING_CROSS_HALF_SIZE_PIXELS, pixelY);
  context.lineTo(pixelX + PARKING_CROSS_HALF_SIZE_PIXELS, pixelY);
  context.moveTo(pixelX, pixelY - PARKING_CROSS_HALF_SIZE_PIXELS);
  context.lineTo(pixelX, pixelY + PARKING_CROSS_HALF_SIZE_PIXELS);
  context.stroke();
}

// ------------------------------------------------------------------------------------------------
// `sizeCanvas` gives `canvas` a size of `width` by `height` page pixels at the pixel
// density of the screen, so that it stays sharp.
// Result: It returns the context of the canvas, set up to draw in page pixels.
// ------------------------------------------------------------------------------------------------
function sizeCanvas(canvas, width, height) {
  const pixelRatio = getPixelRatio();
  canvas.width = Math.round(width * pixelRatio);
  canvas.height = Math.round(height * pixelRatio);
  canvas.style.height = height + "px";
  const context = canvas.getContext("2d");
  applyPixelRatio(context);
  return context;
}

// ------------------------------------------------------------------------------------------------
// `buildStaticLayer` sizes the map and sets `view`. Then it draws everything that does not move
// into the off-screen canvas `staticLayer`, which `draw` copies under the units in every frame.
// The ids that start with "l-" are the layer checkboxes of the page.
// ------------------------------------------------------------------------------------------------
function buildStaticLayer() {
  // Size the map, and make an off-screen canvas of the same size.
  view = computeView();
  sizeCanvas(mapCanvas, view.width, view.height);
  staticLayer = document.createElement("canvas");
  staticLayer.width = mapCanvas.width;
  staticLayer.height = mapCanvas.height;
  const context = staticLayer.getContext("2d");
  applyPixelRatio(context);

  // Fill the ground, then draw the grid and the cells.
  context.fillStyle = colors.map;
  context.fillRect(0, 0, view.width, view.height);
  if (getElement("l-grid").checked)
    drawGridLines(context);
  drawCells(context);

  // Draw the routes, the lanes and the final parking positions.
  if (getElement("l-route").checked)
    drawRoutes(context);
  if (getElement("l-lanes").checked)
    drawLanes(context);
  drawParkingPositions(context);

  // Draw the whole paths of the units, and the scale bar.
  if (getElement("l-ghost").checked)
    drawWholePaths(context);
  drawScaleBar(context);
}

// ------------------------------------------------------------------------------------------------
// `drawGridLines` draws the grid lines on the cell edges. When the cells are small on the map, it
// draws only the major lines.
// ------------------------------------------------------------------------------------------------
function drawGridLines(context) {
  // Find the world area that the map shows.
  const cellMeters = run.cell;
  const halfVisibleWidthMeters = view.width / 2 / view.pixelsPerMeter;
  const halfVisibleHeightMeters = view.height / 2 / view.pixelsPerMeter;
  const visibleMinX = view.centerX - halfVisibleWidthMeters;
  const visibleMaxX = view.centerX + halfVisibleWidthMeters;
  const visibleMinY = view.centerY - halfVisibleHeightMeters;
  const visibleMaxY = view.centerY + halfVisibleHeightMeters;
  const areMinorLinesHidden = view.pixelsPerMeter * cellMeters < MIN_CELL_PIXELS_FOR_MINOR_LINES;
  context.lineWidth = GRID_LINE_WIDTH_PIXELS;

  // Draw the vertical lines, then the horizontal lines.
  strokeGridLines(context, visibleMinX, visibleMaxX, areMinorLinesHidden, (edgeX) => {
    const lineX = snapToPixelCenter(getScreenX(edgeX));
    strokeLine(context, lineX, 0, lineX, view.height);
  });
  strokeGridLines(context, visibleMinY, visibleMaxY, areMinorLinesHidden, (edgeY) => {
    const lineY = snapToPixelCenter(getScreenY(edgeY));
    strokeLine(context, 0, lineY, view.width, lineY);
  });
}

// ------------------------------------------------------------------------------------------------
// `strokeGridLines` draws the grid lines across one axis, on each cell edge from `visibleMin` to
// `visibleMax` in meters.
// Inputs: `strokeLineAt` draws one line at the world coordinate that it gets.
// Edge case: When `areMinorLinesHidden` is set, `strokeGridLines` draws only the major lines.
// ------------------------------------------------------------------------------------------------
function strokeGridLines(context, visibleMin, visibleMax, areMinorLinesHidden, strokeLineAt) {
  const cellMeters = run.cell;
  const firstEdgeIndex = Math.floor(visibleMin / cellMeters);
  for (let edgeIndex = firstEdgeIndex; edgeIndex * cellMeters <= visibleMax; edgeIndex++) {
    const isMajorLine = edgeIndex % MAJOR_GRID_LINE_EVERY === 0;
    context.strokeStyle = isMajorLine ? colors["grid-major"] : colors.grid;
    if (areMinorLinesHidden && !isMajorLine)
      continue;
    const edgeMeters = edgeIndex * cellMeters;
    strokeLineAt(edgeMeters);
  }
}

// ------------------------------------------------------------------------------------------------
// `drawCells` fills the cells of the map itself that are not open ground (value 0) with the colors
// of their values: slopes (1), hills (2) and cliffs (3). `write_trace_grid` in simulator/src/metrics.c
// writes `run.grid` for the cells around the area that the run uses. `x0` and `y0` are its first
// column and row in the grid of the game, `w` and `h` are its size in cells, and `cells` has one
// digit for each cell, row by row from `y0` upwards.
// Edge case: A trace can have no grid, and then `drawCells` draws nothing.
// ------------------------------------------------------------------------------------------------
function drawCells(context) {
  const grid = run.grid;
  if (!grid)
    return;

  // Fill each cell that is not open ground. World y points up, so the top edge of a cell is the
  // bottom edge of the row above it.
  const cellMeters = run.cell;
  const cellColors = {
    [SLOPE_CELL]: colors.slope,
    [HILL_CELL]: colors.rough,
    [CLIFF_CELL]: colors.rock,
  };
  const cellPixels = Math.ceil(cellMeters * view.pixelsPerMeter) + CELL_OVERLAP_PIXELS;
  for (let rowIndex = 0; rowIndex < grid.h; rowIndex++) {
    for (let columnIndex = 0; columnIndex < grid.w; columnIndex++) {
      const cellIndex = rowIndex * grid.w + columnIndex;
      const cellValue = grid.cells.charCodeAt(cellIndex) - DIGIT_ZERO_CODE;
      if (!cellValue)
        continue;
      context.fillStyle = cellColors[cellValue];
      const leftEdgeMeters = (grid.x0 + columnIndex) * cellMeters;
      const topEdgeMeters = (grid.y0 + rowIndex + 1) * cellMeters;
      const cellLeft = Math.floor(getScreenX(leftEdgeMeters));
      const cellTop = Math.floor(getScreenY(topEdgeMeters));
      context.fillRect(cellLeft, cellTop, cellPixels, cellPixels);
    }
  }
}

// ------------------------------------------------------------------------------------------------
// `drawRoutes` draws the route of each squad as a dashed line. In formation following, the route
// is shortened, and the anchor of the squad follows it. The anchor is a point that moves along the
// route at the speed of the squad, and the slots of the members move with it. The slot of a member
// is the point where the member belongs in the formation at that moment.
// ------------------------------------------------------------------------------------------------
function drawRoutes(context) {
  run.squads.forEach((squad, squadIndex) => {
    if (squad.route.length < 2)
      return;
    setLineStyle(context, getSquadColor(squadIndex), ROUTE_LINE_WIDTH_PIXELS, ROUTE_ALPHA);
    context.setLineDash(ROUTE_DASH_PIXELS);
    context.beginPath();
    squad.route.forEach((point, pointIndex) => {
      addPathPoint(context, pointIndex === 0, point[0], point[1]);
    });
    context.stroke();
    context.setLineDash([]);
    context.globalAlpha = 1;
  });
}

// ------------------------------------------------------------------------------------------------
// `drawLanes` draws the lane of each member (the path of its slot beside the route, which the
// member drives along). Only formation-following runs have lanes. `report_squad` in
// simulator/src/metrics.c writes a lane point every 4 m along the route. Where the lanes do not fit side
// by side, the squad drives in column formation, and the trace has null in place of the point,
// which leaves a gap in the line.
// ------------------------------------------------------------------------------------------------
function drawLanes(context) {
  run.squads.forEach((squad, squadIndex) => {
    setLineStyle(context, getSquadColor(squadIndex), LANE_LINE_WIDTH_PIXELS, LANE_ALPHA);
    for (const lane of squad.lanes || []) {
      // Lift the pen at each null point, so that the line has a gap there.
      context.beginPath();
      let isPenDown = false;
      for (const point of lane) {
        if (!point) {
          isPenDown = false;
          continue;
        }
        addPathPoint(context, !isPenDown, point[0], point[1]);
        isPenDown = true;
      }
      context.stroke();
    }
    context.globalAlpha = 1;
  });
}

// ------------------------------------------------------------------------------------------------
// `drawParkingPositions` draws a cross on the final parking position of each member, where the
// member stops at the destination.
// ------------------------------------------------------------------------------------------------
function drawParkingPositions(context) {
  run.squads.forEach((squad, squadIndex) => {
    setLineStyle(context, getSquadColor(squadIndex), PARKING_LINE_WIDTH_PIXELS);
    for (const [x, y] of squad.parking)
      drawParkingCross(context, getScreenX(x), getScreenY(y));
  });
}

// ------------------------------------------------------------------------------------------------
// `drawWholePaths` draws the path of every unit over the whole run.
// ------------------------------------------------------------------------------------------------
function drawWholePaths(context) {
  context.lineWidth = WHOLE_PATH_LINE_WIDTH_PIXELS;
  context.globalAlpha = WHOLE_PATH_ALPHA;
  const lastFrameIndex = run.frames.length - 1;
  for (let unitIndex = 0; unitIndex < run.units.length; unitIndex++) {
    context.strokeStyle = getSquadColor(run.units[unitIndex].squad);
    context.beginPath();
    addUnitPathPoints(context, unitIndex, lastFrameIndex);
    context.stroke();
  }
  context.globalAlpha = 1;
}

// ------------------------------------------------------------------------------------------------
// `drawScaleBar` draws a bar `SCALE_BAR_METERS` long, with end ticks and a label, at the bottom
// left of the map.
// ------------------------------------------------------------------------------------------------
function drawScaleBar(context) {
  // Draw the bar and its end ticks.
  const barPixels = SCALE_BAR_METERS * view.pixelsPerMeter;
  const barX = SCALE_BAR_LEFT_PIXELS;
  const barEndX = barX + barPixels;
  const barY = view.height - SCALE_BAR_BOTTOM_PIXELS;
  const tickTop = barY - SCALE_BAR_TICK_PIXELS;
  const tickBottom = barY + SCALE_BAR_TICK_PIXELS;
  context.strokeStyle = colors.ink;
  context.fillStyle = colors.ink;
  context.lineWidth = SCALE_BAR_LINE_WIDTH_PIXELS;
  context.beginPath();
  context.moveTo(barX, barY);
  context.lineTo(barEndX, barY);
  context.moveTo(barX, tickTop);
  context.lineTo(barX, tickBottom);
  context.moveTo(barEndX, tickTop);
  context.lineTo(barEndX, tickBottom);
  context.stroke();

  // Draw the label.
  const dataFont = getComputedStyle(document.body).getPropertyValue("--font-data");
  context.font = SCALE_LABEL_FONT_PIXELS + "px " + dataFont;
  context.textBaseline = "bottom";
  const labelX = barEndX + SCALE_LABEL_GAP_PIXELS;
  const labelBottom = barY + SCALE_LABEL_DROP_PIXELS;
  context.fillText(SCALE_BAR_METERS + " m", labelX, labelBottom);
}

// ------------------------------------------------------------------------------------------------
// `drawTrails` draws the path of each unit from the start up to the current moment.
// ------------------------------------------------------------------------------------------------
function drawTrails(framePair) {
  mapContext.lineWidth = TRAIL_LINE_WIDTH_PIXELS;
  for (let unitIndex = 0; unitIndex < run.units.length; unitIndex++) {
    mapContext.strokeStyle = getSquadColor(run.units[unitIndex].squad);
    mapContext.globalAlpha = TRAIL_ALPHA;
    mapContext.beginPath();
    addUnitPathPoints(mapContext, unitIndex, framePair.beforeIndex);
    const unitState = interpolateUnitState(framePair, unitIndex);
    addPathPoint(mapContext, false, unitState.x, unitState.y);
    mapContext.stroke();
  }
  mapContext.globalAlpha = 1;
}

// ------------------------------------------------------------------------------------------------
// `drawUnit` draws one unit to scale, with a line for its heading and the marks for its state
// bits. When the layer "Where each unit drives" is on and the unit has not stopped, it also draws
// a line to the waypoint of the unit.
// ------------------------------------------------------------------------------------------------
function drawUnit(framePair, unitIndex) {
  // Find where the unit is on the map and how large it is. It is never smaller than
  // `MIN_UNIT_RADIUS_PIXELS`, so that it stays visible.
  const unitState = interpolateUnitState(framePair, unitIndex);
  const unitInfo = run.units[unitIndex];
  const color = getSquadColor(unitInfo.squad);
  const pixelX = getScreenX(unitState.x);
  const pixelY = getScreenY(unitState.y);
  const radiusPixels = Math.max(MIN_UNIT_RADIUS_PIXELS, unitInfo.r * view.pixelsPerMeter);

  // Draw the line to the waypoint.
  if (getElement("l-wp").checked && !(unitState.bits & STOPPED_BIT)) {
    setLineStyle(mapContext, color, WAYPOINT_LINE_WIDTH_PIXELS, WAYPOINT_LINE_ALPHA);
    const waypointPixelX = getScreenX(unitState.waypointX);
    const waypointPixelY = getScreenY(unitState.waypointY);
    strokeLine(mapContext, pixelX, pixelY, waypointPixelX, waypointPixelY);
    mapContext.globalAlpha = 1;
  }

  // Draw the body.
  mapContext.fillStyle = color;
  fillCircle(mapContext, pixelX, pixelY, radiusPixels);

  // Draw the heading line. The heading points along (sin heading, cos heading) in world
  // coordinates, and screen y points down.
  const headingX = Math.sin(unitState.heading);
  const headingY = Math.cos(unitState.heading);
  const scaledLineWidth = radiusPixels * HEADING_LINE_WIDTH_FRACTION;
  const headingLineWidth = Math.max(MIN_HEADING_LINE_WIDTH_PIXELS, scaledLineWidth);
  setLineStyle(mapContext, colors.panel, headingLineWidth);
  const headingEndX = pixelX + headingX * radiusPixels * HEADING_LINE_LENGTH_FRACTION;
  const headingEndY = pixelY - headingY * radiusPixels * HEADING_LINE_LENGTH_FRACTION;
  strokeLine(mapContext, pixelX, pixelY, headingEndX, headingEndY);

  // Draw the marks for the state bits.
  drawMarks(mapContext, pixelX, pixelY, radiusPixels, unitState.bits, headingX, headingY);
}

// ------------------------------------------------------------------------------------------------
// `drawMarks` draws a mark around a unit for each of its state bits in `bits`.
// Inputs: `headingX` and `headingY` give the heading of the unit in world coordinates, with y up.
// ------------------------------------------------------------------------------------------------
function drawMarks(context, pixelX, pixelY, radiusPixels, bits, headingX, headingY) {
  const outerRingRadiusPixels = radiusPixels + OUTER_RING_GAP_PIXELS;

  // Draw a ring when the unit touches another unit.
  if (bits & CONTACT_BIT) {
    setLineStyle(context, colors.contact, CONTACT_RING_WIDTH_PIXELS);
    strokeCircle(context, pixelX, pixelY, radiusPixels + CONTACT_RING_GAP_PIXELS);
  }

  // Draw a dashed ring when the unit turns on the spot.
  if (bits & TURNING_ON_SPOT_BIT) {
    setLineStyle(context, colors.ink, ON_SPOT_RING_WIDTH_PIXELS);
    context.setLineDash(ON_SPOT_RING_DASH_PIXELS);
    strokeCircle(context, pixelX, pixelY, outerRingRadiusPixels);
    context.setLineDash([]);
  }

  // Draw a pause sign, two bars above the unit and to its right, when the unit waits.
  if (bits & WAITING_BIT) {
    context.fillStyle = colors.ink;
    const pauseHeight = Math.max(MIN_PAUSE_HEIGHT_PIXELS, radiusPixels * PAUSE_HEIGHT_FRACTION);
    const pauseLeft = pixelX + radiusPixels + PAUSE_GAP_PIXELS;
    const pauseTop = pixelY - radiusPixels - PAUSE_GAP_PIXELS - pauseHeight;
    const pauseBarWidth = pauseHeight * PAUSE_BAR_WIDTH_FRACTION;
    const secondBarLeft = pauseLeft + pauseHeight * PAUSE_SECOND_BAR_FRACTION;
    context.fillRect(pauseLeft, pauseTop, pauseBarWidth, pauseHeight);
    context.fillRect(secondBarLeft, pauseTop, pauseBarWidth, pauseHeight);
  }

  // Draw an arc in front of the unit when it makes an avoidance turn. The angle is measured on the
  // screen, where y points down.
  if (bits & AVOIDANCE_TURN_BIT) {
    setLineStyle(context, colors.ink, AVOIDANCE_TURN_ARC_WIDTH_PIXELS);
    const headingAngle = Math.atan2(-headingY, headingX);
    const arcStart = headingAngle - AVOIDANCE_TURN_ARC_HALF_ANGLE_RADIANS;
    const arcEnd = headingAngle + AVOIDANCE_TURN_ARC_HALF_ANGLE_RADIANS;
    context.beginPath();
    context.arc(pixelX, pixelY, outerRingRadiusPixels, arcStart, arcEnd);
    context.stroke();
  }

  // Draw a faint ring when the ORCA avoidance changes the speed or direction of the unit, in the
  // color of the ORCA avoidance bars of the strip.
  if (bits & ORCA_AVOIDING_BIT) {
    setLineStyle(context, colors.orca, ORCA_RING_WIDTH_PIXELS);
    context.globalAlpha = ORCA_RING_ALPHA;
    strokeCircle(context, pixelX, pixelY, outerRingRadiusPixels);
    context.globalAlpha = 1;
  }
}
