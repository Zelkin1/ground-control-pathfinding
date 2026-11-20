// This file fills the side panel of the replay page: the number formatters, the rows of the
// comparison table, the card of each squad and the legend. It is the fourth script of the page. It
// uses the trace readers of replay_trace.js, the canvas helpers of replay_map.js and the state of
// replay.js (`run` and `mode`), and `selectRun` in replay.js calls `renderSide`.
// replay_template.html loads it, and sim/replay/make_replay.py copies it into a script element of
// the saved page, so that the page is one file.

// The trace names the unit types after the unit files of the game, and the page shows these names.
const UNIT_TYPE_LABEL = {
  cc_mediumtank: "Medium tanks",
  cc_lighttank: "Light tanks",
  cc_heavytank: "Heavy tanks",
  cc_marine: "Marines",
};
// The mode columns of the table have these heads.
const MODE_LABEL = {
  original: "Base game",
  coord: "Mod",
  new: "New features",
};

// These values size the legend icons.
const ICON_WIDTH_PIXELS = 30;
const ICON_HEIGHT_PIXELS = 22;
const ICON_UNIT_RADIUS_PIXELS = 5;
const ICON_UNIT_OFFSET_PIXELS = 3; // The unit sits this far left of the center, so its mark fits.
const ICON_SQUARE_PIXELS = 16;
const ICON_LINE_MARGIN_PIXELS = 2; // The lines of the icons end this far from both icon edges.
// The two lanes of the lane icon are this far above and below the center.
const ICON_LANE_OFFSET_PIXELS = 4;
const ICON_LANE_GAP_HALF_WIDTH_PIXELS = 3; // The gap in the lower lane is twice this wide.
const ICON_ROUTE_DASH_PIXELS = [5, 4];

const NO_VALUE_TEXT = "—"; // The comparison table shows this dash where it has no value.

// ------------------------------------------------------------------------------------------------
// `formatNumber` shows `value` with `decimals` decimals.
// Edge case: A negative value means that there is no value, and then `formatNumber` returns
//   `NO_VALUE_TEXT`.
// ------------------------------------------------------------------------------------------------
function formatNumber(value, decimals = 1) {
  if (value < 0)
    return NO_VALUE_TEXT;
  return value.toFixed(decimals);
}

// ------------------------------------------------------------------------------------------------
// `formatSeconds` and `formatPercent` show a value with one decimal or none, followed by its unit.
// ------------------------------------------------------------------------------------------------
function formatSeconds(value) {
  return formatNumber(value) + " s";
}

function formatPercent(value) {
  return formatNumber(value, 0) + "%";
}

// ------------------------------------------------------------------------------------------------
// `formatStops` shows the stops per unit with two decimals.
// ------------------------------------------------------------------------------------------------
function formatStops(value) {
  return value.toFixed(2);
}

// ------------------------------------------------------------------------------------------------
// `formatArrival` shows the arrival time, or "never" for a squad that never arrived, whose
// arrival time is negative.
// ------------------------------------------------------------------------------------------------
function formatArrival(value) {
  if (value < 0)
    return "never";
  return formatSeconds(value);
}

// ------------------------------------------------------------------------------------------------
// `allowMissing` returns a function that shows a value with `formatValue`, or `NO_VALUE_TEXT`
// when the trace has no such value.
// ------------------------------------------------------------------------------------------------
function allowMissing(formatValue) {
  return (value) => {
    if (value === undefined)
      return NO_VALUE_TEXT;
    return formatValue(value);
  };
}

// `TABLE_ROWS` holds the rows of the comparison table. Each row has its label, the key of the
// number of the squad in the trace, a function that shows the number, and a function that tells
// whether the table marks the number as bad, in the contact color. `report_squad` in
// sim/src/metrics.c writes these keys and more, which the table leaves out. A negative number
// means that there is no value, so `arrive` is -1 when the squad never arrived. The trace can
// leave out `top_speed`. Stops per unit get two decimals and no dash for a negative value.
// `top_speed` is the share of the travel time of the members at 95% of their top speed or more,
// in percent.
// TODO: confirm why stops per unit are shown in a different way from the other numbers.
const STOPS_PER_UNIT_LIMIT = 0.5; // The table marks this many stops per unit or more as bad.
const SLOW_SECONDS_LIMIT = 2; // The table marks more slow time per unit than this as bad.
const STUCK_SECONDS_LIMIT = 0.5; // The table marks more stuck time per unit than this as bad.
const OWN_BUMPS_LIMIT = 2; // The table marks more bumps in the squad than this as bad.
const TABLE_ROWS = [
  ["Arrived after", "arrive", formatArrival, (value) => value < 0],
  ["Stops per unit", "stops", formatStops, (value) => value >= STOPS_PER_UNIT_LIMIT],
  ["Slow time per unit", "slow", formatSeconds, (value) => value > SLOW_SECONDS_LIMIT],
  ["Stuck time per unit", "stuck", formatSeconds, (value) => value > STUCK_SECONDS_LIMIT],
  ["Time at top speed", "top_speed", allowMissing(formatPercent), () => false],
  ["Bumps in squad", "bumps_own", String, (value) => value > OWN_BUMPS_LIMIT],
  ["Bumps with others", "bumps_other", String, (value) => value > 0],
];

// The card of a parked squad shows this note in place of the table.
const PARKED_SQUAD_TEXT = "Parked. In formation following, moving squads search their routes "
  + "around it, and its units step aside only a little.";
// The card of a squad whose units board an APC shows this note in place of the table.
const BOARDING_SQUAD_TEXT = "Boards the APC: its units walk straight at it with the movement of "
  + "the game, and each one rides inside it from within two APC radii on.";

// `LEGEND_ITEMS` holds the rows of the legend. Each key picks the icon that `drawLegendIcon`
// draws, and each text is the label of the row.
const LEGEND_ITEMS = [
  ["unit", "Unit to scale, line shows its heading"],
  ["contact", "Touching another unit"],
  ["onSpot", "Turning on the spot"],
  ["waiting", "Waiting 2 s for a faster unit"],
  ["avoidanceTurn", "Avoidance turn at full lock"],
  ["orca", "ORCA avoidance changing speed or direction"],
  ["speed", "Time strip line: squad speed / top speed"],
  ["parking", "Final parking position"],
  ["route", "Route (shortened in formation following)"],
  ["lanes", "Lanes of the members in formation following (gaps: column formation)"],
  ["rough", "Hills: tanks cannot enter, infantry can"],
  ["rock", "Cliffs: no unit can enter"],
];
// The icons of these legend keys show the mark of a state bit.
const LEGEND_MARK_BITS = {
  contact: CONTACT_BIT,
  onSpot: TURNING_ON_SPOT_BIT,
  waiting: WAITING_BIT,
  avoidanceTurn: AVOIDANCE_TURN_BIT,
  orca: ORCA_AVOIDING_BIT,
};

// ------------------------------------------------------------------------------------------------
// `renderSide` fills the side panel with a card for each squad, which compares the modes, and
// then with the legend.
// ------------------------------------------------------------------------------------------------
function renderSide() {
  const side = getElement("side");
  side.textContent = "";
  const availableModes = MODES.filter((modeName) => findRun(run.name, modeName));
  run.squads.forEach((squad, squadIndex) => {
    const card = buildSquadCard(squad, squadIndex, availableModes);
    side.append(card);
  });
  side.append(buildLegend());
}

// ------------------------------------------------------------------------------------------------
// `buildSquadCard` builds the card for one squad of the shown run.
// Result: For a moving squad, the card has a table with the numbers of the squad in every mode of
//   `availableModes`. For a parked squad, it has a note instead.
// ------------------------------------------------------------------------------------------------
function buildSquadCard(squad, squadIndex, availableModes) {
  // Make the title from the unit type and the number of units.
  const unitCount = run.units.filter((unit) => unit.squad === squadIndex).length;
  const card = document.createElement("article");
  card.className = "squad";
  const unitTypeLabel = UNIT_TYPE_LABEL[squad.type] || squad.type;
  const title = unitTypeLabel + " × " + unitCount;
  const squadColor = getSquadColor(squadIndex);
  const swatchHtml = `<span class="swatch" style="background:${squadColor}"></span>`;
  const headingHtml = `<h2>${swatchHtml}${title}</h2>`;
  if (!squad.moving) {
    const note = squad.boarding ? BOARDING_SQUAD_TEXT : PARKED_SQUAD_TEXT;
    card.innerHTML = headingHtml + `<p class="verdict">${note}</p>`;
    return card;
  }

  // Make the table, with a column for each mode, and highlight the column of the shown mode.
  const runPerMode = availableModes.map((modeName) => findRun(run.name, modeName));
  const squadPerMode = runPerMode.map((modeRun) => modeRun.squads[squadIndex]);
  const buildHeaderCell = (modeName) => `<th scope="col">${MODE_LABEL[modeName]}</th>`;
  const headerCells = availableModes.map(buildHeaderCell).join("");
  const bodyRows = TABLE_ROWS.map(([label, key, formatValue, isBad]) => {
    const valueCells = squadPerMode.map((modeSquad, modeIndex) => {
      const value = modeSquad[key];
      const currentClass = availableModes[modeIndex] === mode ? "cur" : "";
      const badClass = isBad(value) ? "bad" : "";
      return `<td class="${currentClass} ${badClass}">${formatValue(value)}</td>`;
    });
    return `<tr><td>${label}</td>${valueCells.join("")}</tr>`;
  });
  const headHtml = `<thead><tr><th scope="col">Squad</th>${headerCells}</tr></thead>`;
  const bodyHtml = `<tbody>${bodyRows.join("")}</tbody>`;
  card.innerHTML = headingHtml + `<table class="cmp">${headHtml}${bodyHtml}</table>`;
  return card;
}

// ------------------------------------------------------------------------------------------------
// `buildLegend` builds the legend, with a small icon and a text for each item of `LEGEND_ITEMS`.
// ------------------------------------------------------------------------------------------------
function buildLegend() {
  const legend = document.createElement("div");
  legend.className = "legend";
  legend.innerHTML = `<h3>On the map</h3>`;

  // Add a row with an icon and a text for each item.
  for (const [key, text] of LEGEND_ITEMS) {
    const row = document.createElement("div");
    const iconCanvas = document.createElement("canvas");
    const context = sizeCanvas(iconCanvas, ICON_WIDTH_PIXELS, ICON_HEIGHT_PIXELS);
    iconCanvas.style.width = ICON_WIDTH_PIXELS + "px";
    drawLegendIcon(context, key);
    row.append(iconCanvas, document.createTextNode(text));
    legend.append(row);
  }
  return legend;
}

// ------------------------------------------------------------------------------------------------
// `drawLegendIcon` draws the icon for one key of `LEGEND_ITEMS`, in the color of the first squad,
// in the same way as the map draws the thing that the key names.
// ------------------------------------------------------------------------------------------------
function drawLegendIcon(context, key) {
  const centerX = ICON_WIDTH_PIXELS / 2;
  const centerY = ICON_HEIGHT_PIXELS / 2;
  const lineLeft = ICON_LINE_MARGIN_PIXELS;
  const lineRight = ICON_WIDTH_PIXELS - ICON_LINE_MARGIN_PIXELS;
  const firstSquadColor = getSquadColor(0);
  context.fillStyle = firstSquadColor;
  context.strokeStyle = firstSquadColor;
  if (key === "rough" || key === "rock") {
    // Fill a square with the cell color, whose name is the key.
    context.fillStyle = colors[key];
    const squareLeft = centerX - ICON_SQUARE_PIXELS / 2;
    const squareTop = centerY - ICON_SQUARE_PIXELS / 2;
    context.fillRect(squareLeft, squareTop, ICON_SQUARE_PIXELS, ICON_SQUARE_PIXELS);
  } else if (key === "parking") {
    context.lineWidth = PARKING_LINE_WIDTH_PIXELS;
    drawParkingCross(context, centerX, centerY);
  } else if (key === "route") {
    context.lineWidth = ROUTE_LINE_WIDTH_PIXELS;
    context.setLineDash(ICON_ROUTE_DASH_PIXELS);
    strokeLine(context, lineLeft, centerY, lineRight, centerY);
  } else if (key === "speed") {
    // Draw a line that rises, dips and rises again, as a speed line on the strip does. It uses
    // the offsets of the lane icon.
    context.lineWidth = SPEED_LINE_WIDTH_PIXELS;
    context.beginPath();
    context.moveTo(lineLeft, centerY + ICON_LANE_OFFSET_PIXELS);
    context.lineTo(centerX - ICON_LANE_GAP_HALF_WIDTH_PIXELS, centerY - ICON_LANE_OFFSET_PIXELS);
    context.lineTo(centerX + ICON_LANE_GAP_HALF_WIDTH_PIXELS, centerY);
    context.lineTo(lineRight, centerY - ICON_LANE_OFFSET_PIXELS);
    context.stroke();
  } else if (key === "lanes") {
    // Draw two lanes. The lower one has a gap, as where the squad drives in column formation.
    const upperLaneY = centerY - ICON_LANE_OFFSET_PIXELS;
    const lowerLaneY = centerY + ICON_LANE_OFFSET_PIXELS;
    context.lineWidth = LANE_LINE_WIDTH_PIXELS;
    context.globalAlpha = LANE_ALPHA;
    context.beginPath();
    context.moveTo(lineLeft, upperLaneY);
    context.lineTo(lineRight, upperLaneY);
    context.moveTo(lineLeft, lowerLaneY);
    context.lineTo(centerX - ICON_LANE_GAP_HALF_WIDTH_PIXELS, lowerLaneY);
    context.moveTo(centerX + ICON_LANE_GAP_HALF_WIDTH_PIXELS, lowerLaneY);
    context.lineTo(lineRight, lowerLaneY);
    context.stroke();
    context.globalAlpha = 1;
  } else {
    // Draw a unit that heads right, along (1, 0), with the mark for the state bit of the key.
    const unitX = centerX - ICON_UNIT_OFFSET_PIXELS;
    fillCircle(context, unitX, centerY, ICON_UNIT_RADIUS_PIXELS);
    setLineStyle(context, colors.panel, MIN_HEADING_LINE_WIDTH_PIXELS);
    strokeLine(context, unitX, centerY, unitX + ICON_UNIT_RADIUS_PIXELS, centerY);
    if (key !== "unit")
      drawMarks(context, unitX, centerY, ICON_UNIT_RADIUS_PIXELS, LEGEND_MARK_BITS[key], 1, 0);
  }
}
