// This file holds the state of the replay page, the choice of the shown run, the playback and the
// start of the page. replay_template.html loads it, and make_replay.py copies it into a
// script element of the saved page, so that the page is one file.

// This script plays one run of the trace on the map, draws the strip under the time slider, and
// fills the side panel. The strip shows the units that touch another unit and the units whose
// speed or direction the ORCA avoidance changes. The trace is the JSON that
// `build/sim.exe --trace` writes (simulator/src/sim.c and simulator/src/metrics.c).
//
// Terms:
// - Formation following is the movement of the "coord" runs. In it, the anchor of each squad is a
//   point that moves along the route at the speed of the squad, and the formation is built around
//   it. Each member has a slot, a point that moves with the anchor. Each member drives along its
//   own lane, the path of its slot beside the route.
// - The ORCA avoidance (optimal reciprocal collision avoidance) is the code of formation following
//   that changes the speed or direction of a unit so that it does not touch other units.
// - The final parking position of a unit is where it stops at the destination.
//
// Files: The page has one script for each part, and the scripts share one global scope. This file
// is the last one, because the start of the page at its end needs the names of the others. The
// order is:
// - replay_trace.js: the trace, the frame layout, and the functions that read the frames.
// - replay_map.js: the map view, the canvas helpers, the static layer, and the units and marks.
// - replay_strip.js: the speed shares of the squads and the strip under the time slider.
// - replay_side.js: the formatters, the table rows, the squad cards and the legend.
// - replay.js: the state, the choice of the run, the playback and `initialize`.

const DEFAULT_SCENARIO = "cross"; // The page shows this scenario when its address names none.
// Factions of the faction tabs. The trace of the page holds the runs of the first faction, the
// scenarios as written. Each other faction loads its runs, the same scenarios and seeds with its
// own unit types, from its data file beside the page the first time that its tab is selected.
const FACTIONS = [
  { name: "crayven", label: "Crayven", file: null },
  { name: "order", label: "Order of the New Dawn", file: "faction_order.json" },
  { name: "phoenix", label: "Phoenix", file: "faction_phoenix.json" },
];
// The page address puts a faction other than the first before the run, as in "order:ridge-coord".
const FACTION_SEPARATOR = ":";

// These values control the playback time.
const MILLISECONDS_PER_SECOND = 1000;
const SCRUB_STEPS = 1000; // This matches the max attribute of the #scrub slider.
const END_TOLERANCE_SECONDS = 0.01; // A press of play this close to the end starts again at 0.
const ARROW_KEY_STEP_SECONDS = 1;

// The play button shows one of these two icons. Each is a 14 px SVG image in the text color.
const BUTTON_ICON_START = '<svg width="14" height="14" viewBox="0 0 14 14" aria-hidden="true">';
const BUTTON_ICON_END = "</svg>";
const PAUSE_LEFT_BAR_SVG = '<rect x="2" y="1" width="3.5" height="12" fill="currentColor"/>';
const PAUSE_RIGHT_BAR_SVG = '<rect x="8.5" y="1" width="3.5" height="12" fill="currentColor"/>';
const PLAY_TRIANGLE_SVG = '<path d="M3 1 L13 7 L3 13 Z" fill="currentColor"/>';
const PAUSE_BARS_SVG = PAUSE_LEFT_BAR_SVG + PAUSE_RIGHT_BAR_SVG;
const PAUSE_ICON_HTML = BUTTON_ICON_START + PAUSE_BARS_SVG + BUTTON_ICON_END;
const PLAY_ICON_HTML = BUTTON_ICON_START + PLAY_TRIANGLE_SVG + BUTTON_ICON_END;

let mode = DEFAULT_MODE; // This is the mode of the shown run.
let run = null; // This is the run of the trace that the page shows.
let factionName = FACTIONS[0].name; // This is the faction of the runs in `TRACE.runs`.
const factionRuns = { [FACTIONS[0].name]: TRACE.runs }; // These are the loaded runs, by faction.
let timeSeconds = 0; // This is the moment that the map shows.
let isPlaying = false;
let lastStepMilliseconds = 0; // The last playback step ran at this `performance.now()` time.

// ------------------------------------------------------------------------------------------------
// `draw` draws the current moment: the static layer, then the trails, the units and their marks,
// the clock and the strip.
// ------------------------------------------------------------------------------------------------
function draw() {
  // Copy the static layer pixel for pixel, then draw in page pixels again.
  mapContext.setTransform(1, 0, 0, 1, 0, 0);
  mapContext.drawImage(staticLayer, 0, 0);
  applyPixelRatio(mapContext);

  // Draw the trails and the units.
  const framePair = findFramePair(timeSeconds);
  if (getElement("l-trails").checked)
    drawTrails(framePair);
  for (let unitIndex = 0; unitIndex < run.units.length; unitIndex++)
    drawUnit(framePair, unitIndex);

  // Update the clock and the time slider, and draw the strip.
  const durationSeconds = getDuration();
  const clockText = timeSeconds.toFixed(1) + " s / " + durationSeconds.toFixed(1) + " s";
  getElement("clock").textContent = "t = " + clockText;
  getElement("scrub").value = Math.round((timeSeconds / durationSeconds) * SCRUB_STEPS);
  drawStrip();
}

// ------------------------------------------------------------------------------------------------
// `readHash` returns the part of the page address after "#".
// The browser escapes a space in the address as "%20", and run names can hold spaces.
// ------------------------------------------------------------------------------------------------
const readHash = () => decodeURIComponent(location.hash.slice(1));

// ------------------------------------------------------------------------------------------------
// `selectRun` shows the run that `hash` names. "scenario" names the run of a scenario in
// `DEFAULT_MODE`, the original game mode, and "scenario-mode" names its run in another mode.
// Edge case: When the trace has no such run, `selectRun` shows the run of the scenario in
//   `DEFAULT_MODE`, or else the first run of the trace.
// ------------------------------------------------------------------------------------------------
function selectRun(hash) {
  // Find the run.
  const [name, modeName] = hash.split("-");
  const hasRequestedRun = MODES.includes(modeName) && findRun(name, modeName) !== undefined;
  const requestedMode = hasRequestedRun ? modeName : DEFAULT_MODE;
  run = findRun(name, requestedMode) || TRACE.runs[0];
  mode = run.mode || DEFAULT_MODE;

  // Update the scenario tabs, the mode buttons and the description.
  for (const button of getElement("tabs").querySelectorAll("button"))
    button.setAttribute("aria-selected", String(button.dataset.name === run.name));
  for (const button of getElement("modes").children) {
    button.setAttribute("aria-checked", String(button.dataset.mode === mode));
    button.disabled = !findRun(run.name, button.dataset.mode);
  }
  getElement("desc").textContent = run.desc;

  // Start again from the beginning.
  setPlaying(false);
  timeSeconds = 0;
  buildStaticLayer();
  renderSide();
  draw();
}

// ------------------------------------------------------------------------------------------------
// `showAddress` shows the faction and the run that the page address `hash` names, after it loads
// the runs of that faction when they are not loaded yet.
// Edge case: When the data file cannot load, for example on a page opened as a local file, the
//   description says so and the page keeps its runs.
// ------------------------------------------------------------------------------------------------
async function showAddress(hash) {
  // Split the faction from the run. An address without a known faction names the first faction.
  const separatorIndex = hash.indexOf(FACTION_SEPARATOR);
  const namedFaction = separatorIndex < 0 ? "" : hash.slice(0, separatorIndex);
  const runHash = separatorIndex < 0 ? hash : hash.slice(separatorIndex + 1);
  const faction = FACTIONS.find((entry) => entry.name === namedFaction) || FACTIONS[0];

  // Load the runs of the faction once.
  if (!(faction.name in factionRuns)) {
    try {
      const response = await fetch(faction.file);
      factionRuns[faction.name] = (await response.json()).runs;
    } catch (error) {
      getElement("desc").textContent = "The runs of " + faction.label + " could not load.";
      return;
    }
  }

  // Show the runs of the faction, and mark its tab.
  factionName = faction.name;
  TRACE.runs = factionRuns[faction.name];
  for (const button of getElement("faction-tabs").querySelectorAll("button"))
    button.setAttribute("aria-selected", String(button.dataset.faction === faction.name));
  selectRun(runHash || DEFAULT_SCENARIO);
}

// ------------------------------------------------------------------------------------------------
// `setPlaying` starts or stops the playback, and shows the matching icon on the play button.
// Edge case: A start close to the end of the run starts again at 0.
// ------------------------------------------------------------------------------------------------
function setPlaying(shouldPlay) {
  // Show the icon of what the button does next.
  isPlaying = shouldPlay;
  const playButton = getElement("play");
  playButton.setAttribute("aria-label", shouldPlay ? "Pause" : "Play");
  playButton.innerHTML = shouldPlay ? PAUSE_ICON_HTML : PLAY_ICON_HTML;

  // Ask for the first playback step.
  if (shouldPlay) {
    if (timeSeconds >= getDuration() - END_TOLERANCE_SECONDS)
      timeSeconds = 0;
    lastStepMilliseconds = performance.now();
    requestAnimationFrame(advancePlayback);
  }
}

// ------------------------------------------------------------------------------------------------
// `advancePlayback` makes one playback step. requestAnimationFrame calls it with the time
// `nowMilliseconds`. It moves the moment on by the real time since the last step times the
// playback speed, and it stops the playback at the end of the run.
// ------------------------------------------------------------------------------------------------
function advancePlayback(nowMilliseconds) {
  // Do nothing when the playback stopped since the last step.
  if (!isPlaying)
    return;

  // Move the moment on.
  const elapsedSeconds = (nowMilliseconds - lastStepMilliseconds) / MILLISECONDS_PER_SECOND;
  timeSeconds += elapsedSeconds * Number(getElement("speed").value);
  lastStepMilliseconds = nowMilliseconds;

  // Stop at the end, or ask for the next step.
  const durationSeconds = getDuration();
  if (timeSeconds >= durationSeconds) {
    timeSeconds = durationSeconds;
    draw();
    setPlaying(false);
    return;
  }
  draw();
  requestAnimationFrame(advancePlayback);
}

// ------------------------------------------------------------------------------------------------
// `initialize` builds the faction and scenario tabs, connects every control, and shows the run
// that the page address names.
// ------------------------------------------------------------------------------------------------
function initialize() {
  readColors();

  // Make the faction tabs, the scenario tabs and the mode buttons change the page address. The
  // hashchange handler below then shows the run.
  const buildHash = (name, modeName, faction) => {
    const runHash = modeName === DEFAULT_MODE ? name : name + "-" + modeName;
    if (faction === FACTIONS[0].name)
      return runHash;
    return faction + FACTION_SEPARATOR + runHash;
  };
  const factionRow = document.createElement("div");
  factionRow.className = "tab-row";
  factionRow.setAttribute("role", "presentation");
  for (const faction of FACTIONS) {
    const button = document.createElement("button");
    button.type = "button";
    button.setAttribute("role", "tab");
    button.dataset.faction = faction.name;
    button.textContent = faction.label;
    button.addEventListener("click", () => {
      location.hash = buildHash(run.name, mode, faction.name);
    });
    factionRow.append(button);
  }
  getElement("faction-tabs").append(factionRow);
  const appendTabRow = (names) => {
    const row = document.createElement("div");
    row.className = "tab-row";
    row.setAttribute("role", "presentation");
    for (const name of names) {
      const button = document.createElement("button");
      button.type = "button";
      button.setAttribute("role", "tab");
      button.dataset.name = name;
      button.textContent = name;
      button.addEventListener("click", () => {
        location.hash = buildHash(name, mode, factionName);
      });
      row.append(button);
    }
    getElement("tabs").append(row);
  };

  // Put the tabs of the scenarios at seed 0 in the first row, and the tabs of chosen seeds in a
  // second row. The run of a chosen seed has a name with a suffix after a space, which
  // make_replay.py adds, and the name of a scenario itself has no space.
  const runNames = [...new Set(TRACE.runs.map((traceRun) => traceRun.name))];
  const isChosenSeed = (name) => name.includes(" ");
  const seedNames = runNames.filter(isChosenSeed);
  appendTabRow(runNames.filter((runName) => !isChosenSeed(runName)));
  if (seedNames.length > 0)
    appendTabRow(seedNames);
  for (const button of getElement("modes").children) {
    button.addEventListener("click", () => {
      location.hash = buildHash(run.name, button.dataset.mode, factionName);
    });
  }

  // Make the play button play or pause, and make the time slider and a click on the strip jump to
  // a moment. `jumpToFraction` stops the playback and shows the moment at `fraction` of the run.
  getElement("play").addEventListener("click", () => setPlaying(!isPlaying));
  const jumpToFraction = (fraction) => {
    setPlaying(false);
    timeSeconds = fraction * getDuration();
    draw();
  };
  getElement("scrub").addEventListener("input", (event) => {
    jumpToFraction(event.target.value / SCRUB_STEPS);
  });
  stripCanvas.addEventListener("click", (event) => {
    const stripBounds = stripCanvas.getBoundingClientRect();
    jumpToFraction((event.clientX - stripBounds.left) / stripBounds.width);
  });

  // Redraw when a layer checkbox changes. Trails and waypoint lines are drawn in every frame, and
  // the other layers are in the static layer.
  for (const id of ["l-trails", "l-wp"])
    getElement(id).addEventListener("change", draw);
  for (const id of ["l-ghost", "l-route", "l-lanes", "l-grid"]) {
    getElement(id).addEventListener("change", () => {
      buildStaticLayer();
      draw();
    });
  }

  // Make space play or pause, and make the arrow keys step through time. Leave a key that is
  // typed into a control to that control.
  document.addEventListener("keydown", (event) => {
    if (event.target.closest("input, select, button"))
      return;
    if (event.key === " ") {
      event.preventDefault();
      setPlaying(!isPlaying);
    }
    if (event.key === "ArrowRight") {
      timeSeconds = Math.min(getDuration(), timeSeconds + ARROW_KEY_STEP_SECONDS);
      draw();
    }
    if (event.key === "ArrowLeft") {
      timeSeconds = Math.max(0, timeSeconds - ARROW_KEY_STEP_SECONDS);
      draw();
    }
  });

  // Show the named run when the page address changes. Rebuild the map when the window size or the
  // color theme changes. A resize waits for the next animation frame, so that a burst of resize
  // events causes one rebuild. The theme follows the system theme and the data-theme attribute of
  // the root element.
  window.addEventListener("hashchange", () => showAddress(readHash()));
  let pendingFrameId = 0;
  window.addEventListener("resize", () => {
    cancelAnimationFrame(pendingFrameId);
    pendingFrameId = requestAnimationFrame(() => {
      buildStaticLayer();
      draw();
    });
  });
  const applyTheme = () => {
    readColors();
    buildStaticLayer();
    renderSide();
    draw();
  };
  window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", applyTheme);
  const themeObserver = new MutationObserver(applyTheme);
  const themeObserverOptions = { attributes: true, attributeFilter: ["data-theme"] };
  themeObserver.observe(document.documentElement, themeObserverOptions);

  // Show the first run.
  showAddress(readHash());
}

// Wait for the web fonts before the first drawing, because the canvas text uses them and does not
// redraw when they arrive.
if (document.fonts && document.fonts.ready) {
  document.fonts.ready.then(initialize);
} else {
  initialize();
}
