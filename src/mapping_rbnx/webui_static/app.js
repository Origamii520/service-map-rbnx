// Bootstrap 5.3 dark theme. The page is served as a fragment with no <html>
// tag of its own, so the attribute is set here rather than in the markup.
document.documentElement.setAttribute("data-bs-theme", "dark");

function setStatus(t) {
  document.getElementById("status").textContent = t;
}
// ── interactive canvas map: pan (drag) / zoom (wheel) / grid / pose / click-pose
// ── scene-style world-centered canvas (proven model from scene webui) ──
// fit() pins the canvas backing-store resolution to its CSS display size,
// so pointer coords map 1:1 — this is what kept the click coords honest.
let STALE = true;
let CANDIDATES = [];
let CANDIDATE_MAP = null;
let SUBMITTING_POSE = false;
let CURTRUST = null;
let OPENED_CANDIDATE_RUN = null;
let CANDIDATE_SIGNATURE = "";
let POLL_SEQUENCE = 0;
let POLL_APPLIED = 0;

function localizationProgress() {
  const loc = CURLOC || {};
  if (STALE) return {key: "offline", title: "Connecting to localization", detail: "Waiting for live status. Updates resume automatically when connected."};
  const progress = {
    idle: ["Waiting for initial localization", "Load a saved map in localization mode to begin."],
    restarting: ["Restarting localization", "Finishing the previous operation before starting a new search. Previous results have been discarded."],
    searching: ["Searching the map for the initial pose", "Keep the robot still during the initial scan match. A clear match is accepted automatically; otherwise up to five candidates will appear."],
    choosing: ["Confirm the robot position", "Choose a numbered position and heading, or move the robot manually to help the particle filter converge."],
    verifying: ["Initial pose found · verifying", "Checking the live laser against the map before activating localization."],
    starting: ["Activating localization", "Starting the map engine at the verified pose and checking its live output."],
    ready: ["Localization ready", "The red marker shows the robot position. See laser verification for live confidence."],
    failed: ["Localization failed", "Review the details and restart localization, or supply an initial pose on the map."],
  };
  const phase = loc.phase || "idle";
  const [title, detail] = progress[phase] || progress.idle;
  return {key: phase, title: phase === "choosing" && CANDIDATES.length ? title + " · " + CANDIDATES.length + " candidates" : title,
    detail: phase === "choosing" && loc.filter_state === "stopped" ? loc.detail : detail,
    busy: ["restarting", "searching", "verifying", "starting"].includes(phase),
    since: (loc.phase_started_at || loc.started_at || 0) * 1000,
    action: phase === "choosing" && CANDIDATES.length > 0};
}

function renderLocalizationProgress() {
  const panel = document.getElementById("loc-progress");
  if (!panel) return;
  panel.hidden = CURENGINE !== "slam_toolbox" || CURMODE !== "localization";
  if (panel.hidden) return;
  const p = localizationProgress();
  panel.dataset.state = p.key;
  document.getElementById("loc-progress-title").textContent = p.title;
  document.getElementById("loc-progress-detail").textContent = p.detail;
  document.getElementById("loc-progress-spinner").hidden = !p.busy;
  document.getElementById("loc-progress-time").textContent = p.since && ["restarting", "searching", "choosing", "verifying", "starting"].includes(p.key)
    ? "Elapsed " + Math.max(0, Math.floor((Date.now() - p.since) / 1000)) + " s" : "";
  document.getElementById("loc-progress-action").hidden = !p.action;
  const filter = document.getElementById("candidate-filter-status");
  if (filter) filter.textContent = p.key === "choosing"
    ? ({starting: "Particle filter is starting. You can already choose a candidate.", running: "Particle filter is active. You can choose a candidate or move the robot manually to help it converge.", stopped: "Particle filter has stopped. Choose an initial pose or restart localization."}[CURLOC.filter_state] || "") : "";
}
setInterval(renderLocalizationProgress, 1000);

// The trust line answers "is my localization right?" from the map rather than
// from the filter's own confidence, which is why it can disagree with the
// spread shown beside it — and when it does, it is the one to believe.
function applyTrust() {
  const el = document.getElementById("trust");
  if (!el) return;
  const t = CURTRUST || {};
  if (!t.verdict) {
    el.classList.remove("on");
    return;
  }
  el.classList.add("on");
  el.classList.remove("ok", "suspect", "stale", "unknown");
  el.classList.add(t.verdict);
  // The median of the recent readings, not the latest one: a single scan
  // dips whenever someone walks past the robot.
  const pct = t.fit != null ? Math.round(t.fit * 100) + "%" : "—";
  const gate = Math.round((t.threshold ?? 0) * 100) + "%";
  const icon = document.getElementById("trusticon");
  const text = document.getElementById("trusttext");
  if (t.verdict == "ok") {
    icon.textContent = "\u2713";
    text.innerHTML = "<b>Localization checks out</b> — " + pct +
      " of the laser lands on mapped walls (needs " + gate + ")";
  } else if (t.verdict == "stale") {
    icon.textContent = "!";
    text.textContent = "The pose has not passed live verification. See localization progress below.";
  } else if (t.verdict == "suspect") {
    icon.textContent = "!";
    text.innerHTML = t.withdrawn
      ? "<b>Map frame withdrawn</b> — only " + pct +
        " of the laser lands on mapped walls. Navigation and recording are " +
        "stopped until this is settled; relocalizing."
      : "<b>Localization does not match the map</b> — only " + pct +
        " of the laser lands on mapped walls (needs " + gate +
        "). The map frame is withdrawn if this holds.";
  } else {
    icon.textContent = "?";
    text.innerHTML = "<b>Localization unverified</b> — " + (t.detail || "no evidence either way");
  }
  el.title = t.detail || "";
}

// The live geometry lives in its own chip: it is the one string on the page
// that changes every poll, and sharing a chip with status made the whole row
// (and the toolbar above it) shuffle a few pixels each second.
function setPose(t) {
  const el = document.getElementById("posechip");
  if (el) el.textContent = t;
}
const cv = document.getElementById("mapcv"),
  cx = cv.getContext("2d");
function fit() {
  if (cv.width != cv.clientWidth) cv.width = cv.clientWidth;
  if (cv.height != cv.clientHeight) cv.height = cv.clientHeight;
}
window.addEventListener("resize", () => {
  fit();
  draw();
});
fit();
let MI = null,
  mapImg = null;
let center = [0, 0],
  pxPerM = 40,
  userMoved = false, // world center + zoom
  AUTOFITTED = false;
function w2p(x, y) {
  return [cv.width / 2 + (x - center[0]) * pxPerM, cv.height / 2 - (y - center[1]) * pxPerM];
}
function p2w(sx, sy) {
  return [center[0] + (sx - cv.width / 2) / pxPerM, center[1] - (sy - cv.height / 2) / pxPerM];
}
function reloadMapImg() {
  let i = new Image();
  i.onload = () => {
    mapImg = i;
    draw();
  };
  i.onerror = () => {};
  i.src = "/api/map.png?" + Date.now();
}
function fitView() {
  if (!MI) return;
  userMoved = false;
  fit();
  let wM = MI.width * MI.resolution,
    hM = MI.height * MI.resolution;
  // Fit means the whole map is on screen, so it centres on the map, not on the
  // robot: centring on a robot standing at one corner pushed the rest of the
  // map off the far edge at exactly the zoom that was supposed to reveal it.
  center = [MI.origin_x + wM / 2, MI.origin_y + hM / 2];
  // Leave room for the toolbar and status strip along the top.
  pxPerM = Math.min(cv.width / wM, (cv.height - 150) / hM) * 0.92;
  draw();
}
function draw() {
  fit();
  cx.clearRect(0, 0, cv.width, cv.height);
  if (!MI) {
    cx.fillStyle = "#5a6172";
    cx.font = "13px system-ui";
    cx.fillText("no map yet", 16, 24);
    return;
  }
  // Fit once, when the first map arrives: the canvas is now the whole window,
  // so a default zoom leaves a real map as a postage stamp in the middle.
  if (!AUTOFITTED && MI.width) {
    AUTOFITTED = true;
    fitView();
    return;
  }
  if (!userMoved && MI.pose && (CURENGINE !== "slam_toolbox" || MI.localization?.state === "converged")) center = [MI.pose.x, MI.pose.y];
  // occupancy underlay — map.png is already y-flipped (row0 = world max-y),
  // so place top-left at world (origin_x, origin_y+hMeters) and grow down.
  if (mapImg && MI.resolution > 0) {
    let wM = MI.width * MI.resolution,
      hM = MI.height * MI.resolution;
    let tl = w2p(MI.origin_x, MI.origin_y + hM);
    cx.imageSmoothingEnabled = false;
    cx.drawImage(mapImg, tl[0], tl[1], wM * pxPerM, hM * pxPerM);
  }
  // 1 m grid aligned to world
  cx.strokeStyle = "rgba(90,130,200,0.18)";
  cx.lineWidth = 1;
  let step = pxPerM,
    ox = (cv.width / 2 - center[0] * pxPerM) % step,
    oy = (cv.height / 2 + center[1] * pxPerM) % step;
  cx.beginPath();
  for (let x = ox; x < cv.width; x += step) {
    cx.moveTo(x, 0);
    cx.lineTo(x, cv.height);
  }
  for (let y = oy; y < cv.height; y += step) {
    cx.moveTo(0, y);
    cx.lineTo(cv.width, y);
  }
  cx.stroke();
  // range overlay: what the robot sees right now, in map coordinates. If these
  // returns do not sit on the walls of the underlay, localization is off — which
  // is the whole reason for drawing them.
  if (RANGE.cloud && RANGE.cloud.pts && RANGE.cloud.pts.length) {
    cx.fillStyle = "rgba(80,180,230,0.45)";
    for (const q of RANGE.cloud.pts) {
      let p = w2p(q[0], q[1]);
      cx.fillRect(p[0] - 1, p[1] - 1, 2, 2);
    }
  }
  if (RANGE.scan && RANGE.scan.pts && RANGE.scan.pts.length) {
    cx.fillStyle = "#39d353";
    for (const q of RANGE.scan.pts) {
      let p = w2p(q[0], q[1]);
      cx.fillRect(p[0] - 1.5, p[1] - 1.5, 3, 3);
    }
  }
  // Alternatives are shown only when the matcher cannot pick one confidently.
  for (const candidate of CANDIDATES) {
    const p = w2p(candidate.x, candidate.y);
    const hue = candidate.rank === 1 ? "#ffb454" : "#9ecaff";
    cx.save();
    cx.translate(p[0], p[1]);
    cx.rotate(-candidate.theta);
    cx.fillStyle = hue;
    cx.strokeStyle = "#10151d";
    cx.lineWidth = 2;
    cx.beginPath();
    cx.moveTo(17, 0);
    cx.lineTo(-10, -8);
    cx.lineTo(-7, 0);
    cx.lineTo(-10, 8);
    cx.closePath();
    cx.fill();
    cx.stroke();
    cx.restore();
    cx.fillStyle = "#10151d";
    cx.fillRect(p[0] + 16, p[1] - 11, 46, 17);
    cx.fillStyle = hue;
    cx.font = "bold 11px system-ui";
    cx.fillText("#" + candidate.rank + " " + Math.round(candidate.score * 100) + "%", p[0] + 19, p[1] + 1);
  }
  // pose-estimate arrow being dragged. Long enough to be unambiguous is drawn
  // in amber; too short is drawn in grey with the reason, because releasing
  // there does nothing and the operator has to know that before letting go.
  if (poseDrag) {
    let a = w2p(poseDrag.x0, poseDrag.y0),
      b = w2p(poseDrag.x1, poseDrag.y1);
    let dx = b[0] - a[0],
      dy = b[1] - a[1],
      len = Math.hypot(dx, dy),
      ok = len >= MIN_POSE_DRAG_PX;
    let col = ok ? "#ffb454" : "#6b7280";
    cx.setLineDash([4, 4]);
    cx.strokeStyle = col + "88";
    cx.lineWidth = 1;
    cx.beginPath();
    cx.arc(a[0], a[1], MIN_POSE_DRAG_PX, 0, 7);
    cx.stroke();
    cx.setLineDash([]);
    cx.strokeStyle = col;
    cx.fillStyle = col;
    cx.lineWidth = 2.5;
    cx.beginPath();
    cx.arc(a[0], a[1], 4, 0, 7);
    cx.fill();
    if (len > 2) {
      let ux = dx / len,
        uy = dy / len,
        hx = b[0] - ux * 12,
        hy = b[1] - uy * 12,
        w = 7;
      cx.beginPath();
      cx.moveTo(a[0], a[1]);
      cx.lineTo(hx, hy);
      cx.stroke();
      cx.beginPath();
      cx.moveTo(b[0], b[1]);
      cx.lineTo(hx - uy * w, hy + ux * w);
      cx.lineTo(hx + uy * w, hy - ux * w);
      cx.closePath();
      cx.fill();
    }
    let th = (Math.atan2(poseDrag.y1 - poseDrag.y0, poseDrag.x1 - poseDrag.x0) * 180) / Math.PI;
    cx.font = "12px system-ui";
    cx.fillStyle = ok ? "#ffd9a0" : "#9aa3b2";
    cx.fillText(
      ok ? th.toFixed(0) + "\u00b0" : "drag further to set a heading",
      b[0] + 10,
      b[1] - 8,
    );
  }
  // live pose marker
  if (MI.pose && (CURENGINE !== "slam_toolbox" || MI.localization?.state === "converged")) {
    let p = w2p(MI.pose.x, MI.pose.y),
      yaw = MI.pose.theta;
    cx.fillStyle = "#e63b3b";
    cx.strokeStyle = "#e63b3b";
    cx.lineWidth = 2;
    cx.beginPath();
    cx.arc(p[0], p[1], 5, 0, 7);
    cx.fill();
    cx.beginPath();
    cx.moveTo(p[0], p[1]);
    cx.lineTo(p[0] + 18 * Math.cos(yaw), p[1] - 18 * Math.sin(yaw));
    cx.stroke();
  }
}
setInterval(reloadMapImg, 2000);
reloadMapImg();
// interaction — fit() makes internal==display, so (clientX-rect.left) is canvas px
function pt(e) {
  let r = cv.getBoundingClientRect();
  return [e.clientX - r.left, e.clientY - r.top];
}
let drag = null,
  moved = 0,
  POSEMODE = false,
  poseDrag = null;
cv.addEventListener("mousedown", (e) => {
  if (POSEMODE && MI) {
    let w = p2w(...pt(e));
    poseDrag = { x0: w[0], y0: w[1], x1: w[0], y1: w[1] };
    draw();
    return;
  }
  drag = pt(e);
  moved = 0;
});
window.addEventListener("mouseup", () => {
  if (poseDrag) {
    finishPose();
    return;
  }
  drag = null;
});
window.addEventListener("mousemove", (e) => {
  if (poseDrag) {
    let w = p2w(...pt(e));
    poseDrag.x1 = w[0];
    poseDrag.y1 = w[1];
    draw();
    return;
  }
  if (!drag) return;
  let p = pt(e);
  center[0] -= (p[0] - drag[0]) / pxPerM;
  center[1] += (p[1] - drag[1]) / pxPerM;
  moved += Math.abs(p[0] - drag[0]) + Math.abs(p[1] - drag[1]);
  userMoved = true;
  drag = p;
  draw();
});
cv.addEventListener(
  "wheel",
  (e) => {
    e.preventDefault();
    let p = pt(e),
      wp = p2w(p[0], p[1]);
    pxPerM *= e.deltaY < 0 ? 1.15 : 1 / 1.15;
    center[0] = wp[0] - (p[0] - cv.width / 2) / pxPerM;
    center[1] = wp[1] + (p[1] - cv.height / 2) / pxPerM;
    userMoved = true;
    draw();
  },
  { passive: false },
);
cv.addEventListener("dblclick", () => fitView());
// Pose estimate is armed explicitly and set by dragging: press where the robot
// is, drag the way it faces, release. A heading matters as much as a position —
// seeding the right spot facing backwards makes relocalization fail the same
// way a wrong spot does.
// A drag shorter than this cannot express a heading anyone meant: the angle
// swings wildly over a few pixels, and a pose seeded facing the wrong way
// fails to relocalize exactly like a pose in the wrong place.
const MIN_POSE_DRAG_PX = 45;
function setPoseArmed(on) {
  POSEMODE = on;
  poseDrag = null;
  let b = document.getElementById("btn-pose");
  if (b) b.classList.toggle("active", on);
  let h = document.getElementById("posehint");
  if (h) h.classList.toggle("on", on);
  cv.style.cursor = on ? "crosshair" : "grab";
  draw();
}
function togglePose() {
  setPoseArmed(!POSEMODE);
  setStatus(
    POSEMODE ? "press where the robot is, then drag the way it faces" : "pose estimate cancelled",
  );
}
document.addEventListener("keydown", (e) => {
  if (e.key == "Escape" && POSEMODE) {
    setPoseArmed(false);
    setStatus("pose estimate cancelled");
  }
});
async function finishPose() {
  let d = poseDrag;
  poseDrag = null;
  let dx = d.x1 - d.x0,
    dy = d.y1 - d.y0;
  if (Math.hypot(dx, dy) * pxPerM < MIN_POSE_DRAG_PX) {
    // Stay armed: the operator meant to set a pose and only fell short, and
    // dropping out of the mode would make them hunt for the button again.
    draw();
    setStatus("too short to read a heading — press and drag further, or Esc to cancel");
    return;
  }
  setPoseArmed(false);
  let th = Math.atan2(dy, dx);
  if (
    !(await askConfirm(
      "Seed pose estimate?",
      "Position (" +
        d.x0.toFixed(2) +
        ", " +
        d.y0.toFixed(2) +
        "), heading " +
        ((th * 180) / Math.PI).toFixed(0) +
        "\u00b0." +
        "\n\n" +
        "RTAB-Map relocalizes from this guess by scan matching. Watch the green scan returns: " +
        "once they line up with the walls of the map, the estimate has converged.",
      { yes: "Seed pose" },
    ))
  )
    return;
  setStatus("seeding pose…");
  let r = await (
    await fetch("/api/pose_estimate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x: d.x0, y: d.y0, theta: th }),
    })
  ).json();
  setStatus(r.detail || "seeded");
}
async function poll() {
  try {
    const request = ++POLL_SEQUENCE;
    let s = await (await fetch("/api/state")).json();
    if (request < POLL_APPLIED) return;
    POLL_APPLIED = request;
    MI = s;
    if (s.mode) CURMODE = s.mode;
    CURENGINE = s.engine || "";
    CURLOCALIZER = s.localizer || "";
    CURLOC = s.localization || null;
    CURTRUST = s.trust || null;
    CURMAP = s.map_id || "";
    const candidateTool = document.getElementById("tool-candidates");
    if (candidateTool) candidateTool.hidden = s.engine !== "slam_toolbox";
    syncCandidates(s);
    applyMode();
    applyTrust();
    setPose(
      s.has_map
        ? "map " +
            s.width +
            "×" +
            s.height +
            " @" +
            s.resolution +
            "m  pose=" +
            (s.pose
              ? "(" +
                s.pose.x.toFixed(2) +
                ", " +
                s.pose.y.toFixed(2) +
                ", " +
                s.pose.theta.toFixed(2) +
                ")"
              : "—") +
            (s.dist_from_seed != null ? "  Δseed=" + s.dist_from_seed + "m" : "") +
            (s.navigation_grid ? " · grid " + (s.navigation_grid.error || s.navigation_grid.revision) : "")
        : "no map yet",
    );
    if (STALE) {
      STALE = false;
      setStatus("live");
    }
    draw();
    renderLocalizationProgress();
  } catch (e) {
    STALE = true;
    setStatus("disconnected");
    renderLocalizationProgress();
  }
}
setInterval(poll, 1000);
poll();
async function loadLib() {
  let m = await (await fetch("/api/maps")).json();
  let el = document.getElementById("lib");
  el.innerHTML = "";
  if (!m.length) {
    el.innerHTML = '<div class="hint">no saved maps yet</div>';
    return;
  }
  for (const x of m) {
    let d = document.createElement("div");
    d.className = "mapitem";
    d.innerHTML = `<img src="/api/maps/${x.map_id}/preview.png?${Date.now()}" alt="">
   <div class="mi"><b title="${x.map_id}">${x.map_id}</b><div title="${x.detail || ""}">${(x.db_size / 1e6).toFixed(1)} MB · ${x.engine || "?"}${x.has_db ? "" : " · no map data"}${x.loadable_here ? "" : " · other backend"}</div></div>
   <div class="acts"><button class="btn sm"${x.loadable_here ? "" : ` disabled title="${x.detail || "cannot be loaded here"}"`} onclick="doLoad('${x.map_id}')">Load</button>
   <button class="btn sm danger" onclick="doDelete('${x.map_id}')">Del</button></div>`;
    if (x.has_preview) {
      const edit = document.createElement("a");
      edit.className = "btn sm";
      edit.href = "/editor?map_id=" + encodeURIComponent(x.map_id);
      edit.textContent = "Edit grid";
      d.querySelector(".acts").appendChild(edit);
    }
    el.appendChild(d);
  }
}
setInterval(() => {
  if (paneOpen("libpanel")) loadLib();
}, 5000);

// The map is the page; everything else is a panel raised over it from the
// toolbar. Opening one refreshes it immediately rather than waiting for the
// next poll, and closing it stops that poll.

function syncCandidates(state) {
  const loc = state.localization || {};
  const sameMap = state.engine === "slam_toolbox" && loc.map_id === state.map_id && loc.generation === state.generation;
  const poses = sameMap && loc.phase === "choosing" && Array.isArray(loc.candidates) ? loc.candidates : [];
  const signature = JSON.stringify([loc.run_id, loc.phase, poses]);
  CANDIDATES = poses;
  CANDIDATE_MAP = poses.length ? {map_id: state.map_id, generation: state.generation, run_id: loc.run_id} : null;
  document.getElementById("candidate-status").textContent = loc.detail || "No candidate positions for the current localization run.";
  if (signature !== CANDIDATE_SIGNATURE) {
    CANDIDATE_SIGNATURE = signature;
    const list = document.getElementById("candidate-list");
    list.replaceChildren();
    for (const pose of poses) {
      const item = document.createElement("div");
      item.className = "candidate-item";
      const label = document.createElement("span");
      label.textContent = `#${pose.rank} · ${Math.round(pose.score * 100)}% · (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}) · ${(pose.theta * 180 / Math.PI).toFixed(0)}°`;
      const button = document.createElement("button");
      button.className = "btn sm";
      button.textContent = "Choose this pose";
      button.addEventListener("click", () => chooseCandidate(pose));
      item.append(label, button);
      list.appendChild(item);
    }
  }
  if (poses.length && OPENED_CANDIDATE_RUN !== loc.run_id) {
    OPENED_CANDIDATE_RUN = loc.run_id;
    if (!paneOpen("candidatepanel")) togglePanel("candidatepanel");
  }
}

async function chooseCandidate(pose) {
  if (!CANDIDATE_MAP || SUBMITTING_POSE) return;
  const expected = {...CANDIDATE_MAP};
  const confirmed = await askConfirm(
    "Use pose candidate #" + pose.rank + "?",
    `Check that the robot is near (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}) facing ${(pose.theta * 180 / Math.PI).toFixed(0)}°. The live laser will verify this pose.`,
    {yes: "Use initial pose"},
  );
  if (!confirmed) return;
  SUBMITTING_POSE = true;
  try {
    const result = await (await fetch("/api/pose_estimate", {
      method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify({x: pose.x, y: pose.y, theta: pose.theta, ...expected}),
    })).json();
    if (!result.ok) setStatus(result.detail || "Could not submit pose.");
    await poll();
  } catch (e) {
    setStatus("Could not submit pose: " + e.message);
  } finally {
    SUBMITTING_POSE = false;
  }
}

function togglePanel(id) {
  const dock = document.getElementById("dock");
  const pane = document.getElementById(id);
  const already = pane.classList.contains("on") && dock.classList.contains("on");
  for (const p of document.querySelectorAll(".pane")) p.classList.remove("on");
  for (const t of document.querySelectorAll(".tab")) t.classList.remove("on");
  for (const b of ["tool-save", "tool-library", "tool-candidates", "tool-log"]) {
    const el = document.getElementById(b);
    if (el) el.classList.remove("active");
  }
  if (already) {
    dock.classList.remove("on");
    return;
  }
  dock.classList.add("on");
  pane.classList.add("on");
  const tab = document.getElementById("tab-" + id);
  if (tab) tab.classList.add("on");
  const tool = { savepanel: "tool-save", libpanel: "tool-library", candidatepanel: "tool-candidates", logpanel: "tool-log" }[id];
  if (tool) document.getElementById(tool).classList.add("active");
  if (id == "libpanel") loadLib();
  if (id == "logpanel") loadLog();
  if (id == "savepanel") document.getElementById("saveid").focus();
}

function closeDock() {
  document.getElementById("dock").classList.remove("on");
  for (const p of document.querySelectorAll(".pane")) p.classList.remove("on");
  for (const t of document.querySelectorAll(".tab")) t.classList.remove("on");
  for (const b of ["tool-save", "tool-library", "tool-candidates", "tool-log"]) {
    const el = document.getElementById(b);
    if (el) el.classList.remove("active");
  }
}

function paneOpen(id) {
  const el = document.getElementById(id);
  return !!el && el.classList.contains("on");
}

const KCOL = {
  save: "#5bd66f",
  load: "#5aa9ff",
  switch: "#d6a85b",
  pose: "#d65b9a",
  info: "#8b93a3",
};
async function loadLog() {
  try {
    let L = await (await fetch("/api/log")).json();
    let box = document.getElementById("logbox");
    // An empty box reads as a broken panel, so say which it is: the log holds
    // this session's actions and starts empty after every boot.
    if (!L.length) {
      box.innerHTML =
        '<div class="hint">nothing yet — saves, loads, mode switches and pose ' +
        "seeds from this session appear here</div>";
      return;
    }
    let atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 20;
    box.innerHTML = L.map((e) => {
      let t = new Date(e.t * 1000).toLocaleTimeString();
      let c = KCOL[e.kind] || "#8b93a3";
      return `<div><span class="t">${t}</span> <b style="color:${c}">${e.kind}</b> ${e.msg.replace(/</g, "&lt;")}</div>`;
    }).join("");
    if (atBottom) box.scrollTop = box.scrollHeight;
  } catch (e) {}
}
setInterval(() => {
  if (paneOpen("logpanel")) loadLog();
}, 1500);
// A robot that has been carried is the case this exists for: its pose is wrong
// and nothing in the running system knows it, because a scan matcher tracking
// frame to frame follows the robot into the wrong place quite happily.
async function doRelocalize() {
  if (
    !(await askConfirm(
      "Find the robot again?",
      "The map frame is withdrawn while this runs, so anything navigating or " +
        "recording in it stops until the new pose has been checked. The robot " +
        "does not need to be driven.",
      { yes: "Relocalize" },
    ))
  )
    return;
  await runExclusive("Relocalizing", "matching what the laser sees against the map", async () => {
    const r = await (await fetch("/api/relocalize", { method: "POST" })).json();
    setStatus(r.detail || (r.ok ? "relocalizing" : "relocalize failed"));
    await poll();
  });
}

async function doSave() {
  let id = document.getElementById("saveid").value.trim();
  if (!id) {
    setStatus("enter a map_id first");
    return;
  }
  let r = await runExclusive(
    "Saving map “" + id + "”…",
    "RTAB-Map is paused while its database is flushed and copied, then the occupancy preview is rendered. Do not drive the robot until this finishes.",
    async () =>
      await (
        await fetch("/api/save", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ map_id: id }),
        })
      ).json(),
  );
  if (r) {
    setStatus(r.detail || "saved");
    loadLib();
    if (r.ok) document.getElementById("saveid").value = "";
  }
}
async function doLoad(id) {
  if (
    !(await askConfirm(
      "Load map " + id + " in localization mode?",
      "This replaces the live SLAM session. Anything mapped since the last Save is discarded — " +
        "save the current map first if you still need it.",
      { danger: true, yes: "Load " + id },
    ))
  )
    return;
  let r = await runExclusive(
    "Loading map “" + id + "”…",
    "The saved database is copied to a runtime file, RTAB-Map switches onto it, and the map is republished. Watch the green scan afterwards to see whether it relocalized.",
    async () =>
      await (
        await fetch("/api/load", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ map_id: id, mode: "localization" }),
        })
      ).json(),
  );
  if (r) setStatus(r.detail || "loaded");
  if (r && r.ok && /global localization/i.test(r.detail || "")) {
    setStatus("relocalizing — particles scattered over the map; drive to converge");
  }
}
async function doSwitch(mode) {
  // Switching a running RTAB-Map from localization to mapping is the one
  // transition that can destroy work: it resumes building from wherever the
  // robot currently believes it is, so if relocalization has not converged on
  // the loaded map it opens a new session instead of extending the old one.
  if (mode == "mapping" && CURMODE == "localization") {
    let what = CURMAP ? 'map "' + CURMAP + '"' : "the loaded map";
    if (
      !(await askConfirm(
        "Switch to mapping while localized on " + what + "?",
        "Expect " +
          what +
          " to disappear from the live view.\n\n" +
          "Why: entering localization started a new RTAB-Map session id. RTAB-Map only links " +
          "consecutive nodes that share a session id, so everything built after this switch " +
          "forms a separate graph component, and the published map is assembled from the " +
          "component the robot is currently in. " +
          what +
          " is still in the database — it comes " +
          "back the moment a loop closure ties the two sessions together, and the saved copy on " +
          "disk is never touched.\n\n" +
          "So this is only safe if RTAB-Map can relocalize where you are standing. To build a " +
          "genuinely new map, restart the service with map_mode: mapping instead.",
        { danger: true, yes: "Switch to mapping" },
      ))
    )
      return;
  }
  let r = await runExclusive(
    "Switching to " + mode + "…",
    "Asking RTAB-Map to change mode. The map itself is not touched.",
    async () =>
      await (
        await fetch("/api/switch_mode", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ mode: mode }),
        })
      ).json(),
  );
  if (r && r.ok) CURMODE = mode;
  applyMode();
  if (r) setStatus(r.detail || "mode " + mode);
} // poll() re-reads the real mode a second later
async function doReset() {
  if (
    !(await askConfirm(
      "Clear the live map and rebuild from scratch?",
      "The new map origin becomes the current robot position, so the rebuilt map will NOT " +
        "line up with the old frame — anything recorded against the previous map goes stale. " +
        "Saved maps on disk are not affected.",
      { danger: true, yes: "Clear live map" },
    ))
  )
    return;
  let r = await runExclusive(
    "Clearing the live map…",
    "RTAB-Map is resetting and resumes mapping from the current pose. Everything recorded against the old frame is now stale.",
    async () =>
      await (
        await fetch("/api/reset", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: "{}",
        })
      ).json(),
  );
  if (r) setStatus(r.detail || "reset");
}
async function doDelete(id) {
  if (
    !(await askConfirm(
      "Delete saved map " + id + "?",
      "The saved database and its previews are removed from disk. This cannot be undone.",
      { danger: true, yes: "Delete " + id },
    ))
  )
    return;
  let r = await runExclusive(
    "Deleting “" + id + "”…",
    "Removing the saved database and its previews from disk.",
    async () =>
      await (
        await fetch("/api/delete", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ map_id: id }),
        })
      ).json(),
  );
  if (r) {
    setStatus(r.detail || "deleted");
    loadLib();
  }
}
let CURMODE = null,
  CURENGINE = "",
  CURLOC = null,
  CURLOCALIZER = "",
  CURMAP = "",
  RANGE = {},
  BUSY = false;
// Save and load take tens of seconds on a real map: rtabmap is paused, the
// database is flushed and copied, previews are rendered. A second operation
// issued in the middle of that acts on a half-published map, so the page holds
// everything back behind one overlay until the first one answers.
function busyOn(title, body) {
  BUSY = true;
  document.getElementById("busytitle").textContent = title;
  document.getElementById("busybody").textContent = body || "";
  document.getElementById("busy").classList.add("on");
}
function busyOff() {
  BUSY = false;
  document.getElementById("busy").classList.remove("on");
}
async function runExclusive(title, body, fn) {
  if (BUSY) return null;
  busyOn(title, body);
  try {
    return await fn();
  } catch (e) {
    setStatus("failed: " + e);
    return null;
  } finally {
    busyOff();
  }
}
async function pollRange() {
  try {
    let r = await (await fetch("/api/range")).json();
    RANGE = r;
    let el = document.getElementById("rangebadge");
    if (!el) return;
    let bits = [];
    for (const k of ["scan", "cloud"]) {
      if (!r[k]) continue;
      let n = r[k].pts ? r[k].pts.length : 0;
      // Name the reason, not just the absence: "no data on /scan" and "no
      // transform to map" are opposite faults behind the same empty overlay.
      bits.push(k + " " + (n ? n + " pts" : r[k].why || "waiting"));
    }
    el.textContent = bits.length ? bits.join(" · ") : "no lidar capability bound";
    draw();
  } catch (e) {}
}
setInterval(pollRange, 500);
pollRange();
let MODALRESOLVE = null;
function askConfirm(title, body, opts) {
  opts = opts || {};
  let m = document.getElementById("modal");
  document.getElementById("modaltitle").textContent = title;
  document.getElementById("modalbody").textContent = body;
  document.getElementById("modalyes").textContent = opts.yes || "Continue";
  document.getElementById("modalbox").className = opts.danger ? "danger" : "";
  // An explanation has nothing to cancel, so it gets one button.
  document.getElementById("modalno").style.display = opts.info ? "none" : "";
  m.classList.add("on");
  return new Promise((res) => {
    MODALRESOLVE = res;
  });
}
function closeModal(v) {
  document.getElementById("modal").classList.remove("on");
  if (MODALRESOLVE) {
    let r = MODALRESOLVE;
    MODALRESOLVE = null;
    r(v);
  }
}
document.getElementById("modalyes").onclick = () => closeModal(true);
document.getElementById("modalno").onclick = () => closeModal(false);
document.getElementById("modal").onclick = (e) => {
  if (e.target.id == "modal") closeModal(false);
};
document.addEventListener("keydown", (e) => {
  if (e.key == "Escape" && MODALRESOLVE) closeModal(false);
});
function applyMode() {
  document.getElementById("btn-reloc").hidden = CURENGINE !== "slam_toolbox";
  let mp = document.getElementById("btn-mapping"),
    lo = document.getElementById("btn-localization");
  let bdg = document.getElementById("modebadge");
  if (bdg) bdg.textContent = CURMODE ? "mode: " + CURMODE : "mode: —";
  // Saved maps belong to the engine that built them, so the running backend is
  // named on screen next to the mode; the localizer is appended when one is on.
  // Relocalization is the one thing on this page that is in flight rather than
  // simply true or false: after a load with no pose the filter is spread over
  // the whole map and the arrow is a guess. Say so, and show the spread
  // shrinking, instead of leaving the operator to wonder.
  let lb = document.getElementById("locbadge");
  if (lb) {
    let st = (CURLOC || {}).state || "off";
    lb.classList.toggle("d-none", st == "off");
    lb.classList.toggle("loc-converging", st == "converging" || st == "waiting");
    lb.classList.toggle("loc-converged", st == "converged");
    lb.classList.toggle("loc-failed", st == "failed");
    if (st == "waiting") {
      lb.textContent = "Waiting for initial localization";
      lb.title = "The localizer has been asked to relocalize but has not published an estimate yet.";
    } else if (st == "converging") {
      lb.textContent = "Localizing · see progress";
      lb.title =
        "Particles are still spread over the map (position ±" +
        (CURLOC.position_stddev_m ?? 0).toFixed(2) +
        " m, heading ±" +
        (((CURLOC.yaw_stddev_rad ?? 0) * 180) / Math.PI).toFixed(0) +
        "°). Drive the robot to help it converge — the pose shown is not settled yet.";
    } else if (st == "failed") {
      lb.textContent = "Localization failed";
      lb.title = (CURLOC || {}).detail || "The localizer could not find the robot on this map.";
    } else if (st == "converged") {
      lb.textContent = "Localized";
      lb.title = "The pose is confirmed. See laser verification for live localization confidence.";
    }
  }
  let eb = document.getElementById("enginebadge");
  if (eb)
    eb.textContent =
      "backend: " +
      (CURENGINE || "—") +
      (CURLOCALIZER && CURLOCALIZER != "none" ? " + " + CURLOCALIZER : "");
  let sw = document.getElementById("modeswitch");
  if (sw) sw.classList.toggle("loc", CURMODE == "localization");
  if (mp && lo) {
    mp.classList.toggle("on", CURMODE == "mapping");
    lo.classList.toggle("on", CURMODE == "localization");
  }
  let w = document.getElementById("modewarn");
  if (!w) return;
  w.classList.toggle("on", CURMODE == "localization");
  if (CURMODE == "localization") {
    document.getElementById("modewarntext").textContent =
      "Localized on " +
      (CURMAP ? '"' + CURMAP + '"' : "a loaded map") +
      " — switching to mapping drops it from the live view.";
  }
}

// The reason is a paragraph, and a paragraph in the panel pushes everything
// below it off screen. It lives behind the Why? button instead.
function explainModeRisk() {
  let what = CURMAP ? 'map "' + CURMAP + '"' : "the loaded map";
  askConfirm(
    "Why does " + what + " disappear?",
    "Loading a map put RTAB-Map into localization, which starts a new session id. RTAB-Map only " +
      "links consecutive nodes that share a session id, so anything built after a switch back to " +
      "mapping forms a separate graph component — and the published map is assembled from the " +
      "component the robot is currently in.\n\n" +
      "Nothing is deleted. " +
      what.charAt(0).toUpperCase() +
      what.slice(1) +
      " returns as soon as a loop closure ties the two sessions together, and the copy on disk is " +
      "never touched. To build a genuinely new map, restart the service with map_mode: mapping.",
    { info: true, yes: "Got it" },
  );
}
