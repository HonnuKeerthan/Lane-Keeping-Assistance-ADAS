/**
 * LKA Simulation – Frontend Controller
 * =====================================
 * Handles:
 *  - LKA Master ON/OFF toggle switch (backend synchronization)
 *  - Road geometry selection (Straight, Gentle L/R, Sharp L/R, S-Curve)
 *  - Video uploads & sample video selection
 *  - Start / Stop / Reset Vehicle / Full Reset
 *  - Real-time MJPEG feed display & HUD badges
 *  - Live status polling & card telemetry updates
 *  - Dynamic Chart.js telemetry (Deviation & Steering Correction Angle)
 */

"use strict";

// ─────────────────────────────────────────────
//  DOM References
// ─────────────────────────────────────────────

// Navigation & Header
const headerModeBadge     = document.getElementById("headerModeBadge");
const headerModeText      = document.getElementById("headerModeText");

// Left Panel Controls
const lkaSwitch           = document.getElementById("lkaSwitch");
const lkaToggleStatus     = document.getElementById("lkaToggleStatus");
const ltcModeText         = document.getElementById("ltcModeText");
const ltcHint             = document.getElementById("ltcHint");

const currentRoadSubbadge = document.getElementById("currentRoadSubbadge");
const roadGrid            = document.getElementById("roadGrid");
const roadButtons         = document.querySelectorAll(".road-btn");

const btnStart            = document.getElementById("btnStart");
const btnStop             = document.getElementById("btnStop");
const btnResetVehicle     = document.getElementById("btnResetVehicle");
const btnReset            = document.getElementById("btnReset");

// Video details & upload
const videoFileInput      = document.getElementById("videoFileInput");
const dropZone            = document.getElementById("dropZone");
const sampleListEl        = document.getElementById("sampleList");
const selectedFileDisplay = document.getElementById("selectedFileDisplay");
const selectedFileNameEl  = document.getElementById("selectedFileName");
const sfClearBtn          = document.getElementById("sfClearBtn");

// System info
const sysMode             = document.getElementById("sysMode");
const sysStatus           = document.getElementById("sysStatus");
const sysFps              = document.getElementById("sysFps");
const sysCurvature        = document.getElementById("sysCurvature");

// Video Feed & HUD Badges
const videoContainer      = document.getElementById("videoContainer");
const videoFeed           = document.getElementById("videoFeed");
const videoPlaceholder    = document.getElementById("videoPlaceholder");
const badgeLKA            = document.getElementById("badgeLKA");
const badgeLDW            = document.getElementById("badgeLDW");
const badgeRoad           = document.getElementById("badgeRoad");
const badgeFPS            = document.getElementById("badgeFPS");
const badgeMode           = document.getElementById("badgeMode");

// Right Panel Telemetry
const lkaIndicator        = document.getElementById("lkaIndicator");
const lkaPulse            = document.getElementById("lkaPulse");
const lkaLabel            = document.getElementById("lkaLabel");
const lkaSublabel         = document.getElementById("lkaSublabel");

const cardLaneStatus      = document.getElementById("cardLaneStatus");
const valLaneStatus       = document.getElementById("valLaneStatus");

const cardSteering        = document.getElementById("cardSteering");
const valSteering         = document.getElementById("valSteering");

const cardSteerAngle      = document.getElementById("cardSteerAngle");
const valSteerAngle       = document.getElementById("valSteerAngle");

const cardDeviation       = document.getElementById("cardDeviation");
const valDeviation        = document.getElementById("valDeviation");
const devBarFill          = document.getElementById("devBarFill");

const cardRoadType        = document.getElementById("cardRoadType");
const valRoadType         = document.getElementById("valRoadType");

const cardLDW             = document.getElementById("cardLDW");
const valLDW              = document.getElementById("valLDW");

// Steering Visualiser
const svCar               = document.getElementById("svCar");
const svArrow             = document.getElementById("svArrow");
const svLabel             = document.getElementById("svLabel");


// ─────────────────────────────────────────────
//  Application State
// ─────────────────────────────────────────────

let currentRoadType       = "straight";
let lkaEnabled            = true;
let selectedVideoPath     = "";
let isRunning             = false;
let statusPollTimer       = null;

// Ring Buffers for Chart.js (60 points)
const MAX_POINTS          = 60;
const timeLabels          = [];
const deviationData       = [];
const steeringCorrData    = [];   // exact steering correction angle in degrees


// ─────────────────────────────────────────────
//  Utility Helpers
// ─────────────────────────────────────────────

function showToast(msg, durationMs = 3000) {
  const toast = document.getElementById("toast");
  toast.textContent = msg;
  toast.style.display = "block";
  clearTimeout(toast._timer);
  toast._timer = setTimeout(() => { toast.style.display = "none"; }, durationMs);
}

function push(arr, val) {
  arr.push(val);
  if (arr.length > MAX_POINTS) arr.shift();
}

function timeLabel() {
  const d = new Date();
  return `${d.getMinutes().toString().padStart(2, "0")}:${d.getSeconds().toString().padStart(2, "0")}`;
}


// ─────────────────────────────────────────────
//  LKA Master Toggle Control
// ─────────────────────────────────────────────

lkaSwitch.addEventListener("change", async () => {
  lkaEnabled = lkaSwitch.checked;
  updateLKAUI(lkaEnabled);

  try {
    const res = await fetch("/set_lka", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled: lkaEnabled }),
    });
    const data = await res.json();
    if (data.success) {
      showToast(lkaEnabled ? "✓ LKA Enabled (Active Assistance)" : "⚠ LKA Disabled (Manual Simulation)");
      if (data.status) updateDashboard(data.status);
    }
  } catch (err) {
    showToast(`Network error: ${err.message}`, 4000);
  }
});

function updateLKAUI(enabled) {
  if (enabled) {
    headerModeBadge.className = "header-mode-badge";
    headerModeText.textContent = "LKA: ACTIVE · ASSISTED";

    lkaToggleStatus.textContent = "ACTIVE";
    lkaToggleStatus.className   = "ltc-status active";
    ltcModeText.textContent     = "LKA ASSISTED";
    ltcHint.textContent         = "Automatic corrective steering enabled";

    badgeLKA.textContent       = "LKA: ACTIVE";
    badgeLKA.className         = "vid-badge vid-badge-tl active";
    badgeMode.textContent      = "MODE: LKA ASSISTED";

    lkaIndicator.className     = "lka-indicator active";
    lkaPulse.className         = "lka-pulse active";
    lkaLabel.textContent       = "ACTIVE";
    lkaSublabel.textContent    = "ASSISTED";
  } else {
    headerModeBadge.className  = "header-mode-badge disabled";
    headerModeText.textContent = "LKA: DISABLED · MANUAL CONTROL";

    lkaToggleStatus.textContent = "DISABLED";
    lkaToggleStatus.className   = "ltc-status disabled";
    ltcModeText.textContent     = "MANUAL CONTROL";
    ltcHint.textContent         = "MANUAL CONTROL — SIMULATION ONLY. Drift allowed.";

    badgeLKA.textContent       = "LKA: DISABLED";
    badgeLKA.className         = "vid-badge vid-badge-tl disabled";
    badgeMode.textContent      = "MODE: MANUAL CONTROL";

    lkaIndicator.className     = "lka-indicator disabled";
    lkaPulse.className         = "lka-pulse disabled";
    lkaLabel.textContent       = "DISABLED";
    lkaSublabel.textContent    = "MANUAL";
  }
}


// ─────────────────────────────────────────────
//  Road Geometry Selection
// ─────────────────────────────────────────────

roadButtons.forEach(btn => {
  btn.addEventListener("click", async () => {
    const road = btn.dataset.road;
    selectRoadType(road, true);
  });
});

async function selectRoadType(road, notifyBackend = true) {
  currentRoadType = road;

  // Update button active state
  roadButtons.forEach(b => {
    if (b.dataset.road === road) {
      b.classList.add("active");
      currentRoadSubbadge.textContent = b.querySelector(".rb-name").textContent;
    } else {
      b.classList.remove("active");
    }
  });

  // If a video was previously selected, clear it so curved road runs
  if (selectedVideoPath) {
    clearSelectedVideo();
  }

  if (notifyBackend) {
    try {
      const res = await fetch("/set_road", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ road_type: road }),
      });
      const data = await res.json();
      if (data.success) {
        showToast(`Road set to: ${data.status?.road_name || road}`);
        if (data.status) updateDashboard(data.status);
      }
    } catch (err) {
      showToast(`Network error: ${err.message}`, 4000);
    }
  }
}


// ─────────────────────────────────────────────
//  Action Controls: Start / Stop / Reset Vehicle
// ─────────────────────────────────────────────

btnStart.addEventListener("click", async () => {
  const payload = {
    road_type:  currentRoadType,
    video_path: selectedVideoPath || "",
  };

  try {
    const res = await fetch("/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (data.success) {
      startFeed();
      showToast("▶ Simulation started");
    } else {
      showToast(`✗ ${data.message}`, 5000);
    }
  } catch (err) {
    showToast(`✗ ${err.message}`, 5000);
  }
});

btnStop.addEventListener("click", async () => {
  try {
    await fetch("/stop", { method: "POST" });
    stopFeed();
    showToast("■ Simulation stopped");
  } catch (err) {
    showToast(`✗ ${err.message}`, 5000);
  }
});

btnResetVehicle.addEventListener("click", async () => {
  try {
    const res = await fetch("/reset_vehicle", { method: "POST" });
    const data = await res.json();
    if (data.success) {
      showToast("↺ Vehicle re-centered in lane.");
      if (data.status) updateDashboard(data.status);
    }
  } catch (err) {
    showToast(`✗ ${err.message}`, 4000);
  }
});

btnReset.addEventListener("click", async () => {
  try {
    await fetch("/reset", { method: "POST" });
    stopFeed();
    clearSelectedVideo();
    selectRoadType("straight", false);
    resetCards();
    resetCharts();
    showToast("⟲ System fully reset");
  } catch (err) {
    showToast(`✗ ${err.message}`, 5000);
  }
});


// ─────────────────────────────────────────────
//  Video Upload & Sample Selection
// ─────────────────────────────────────────────

videoFileInput.addEventListener("change", async () => {
  const file = videoFileInput.files[0];
  if (!file) return;
  await uploadFile(file);
});

dropZone.addEventListener("dragover", e => { e.preventDefault(); dropZone.classList.add("drag-over"); });
dropZone.addEventListener("dragleave", () => dropZone.classList.remove("drag-over"));
dropZone.addEventListener("drop", async e => {
  e.preventDefault();
  dropZone.classList.remove("drag-over");
  const file = e.dataTransfer.files[0];
  if (file) await uploadFile(file);
});
dropZone.addEventListener("click", () => videoFileInput.click());

async function uploadFile(file) {
  showToast("Uploading video…");
  const formData = new FormData();
  formData.append("video", file);

  try {
    const res  = await fetch("/upload", { method: "POST", body: formData });
    const data = await res.json();
    if (data.success) {
      setSelectedVideo(data.video_path, data.filename);
      showToast(`✓ Uploaded: ${data.filename}`);
    } else {
      showToast(`✗ Upload failed: ${data.message}`, 5000);
    }
  } catch (err) {
    showToast(`✗ Network error: ${err.message}`, 5000);
  }
}

async function loadSampleList() {
  try {
    const res  = await fetch("/sample_videos");
    const data = await res.json();
    sampleListEl.innerHTML = "";

    if (!data.videos || data.videos.length === 0) {
      sampleListEl.innerHTML = '<div class="loading-samples">No samples found.</div>';
      return;
    }

    data.videos.forEach(v => {
      const btn = document.createElement("button");
      btn.className = "sample-btn";
      btn.innerHTML = `<span class="sb-icon">🎬</span> ${v.name}`;
      btn.dataset.path = v.path;
      btn.dataset.name = v.name;
      btn.addEventListener("click", () => selectSample(btn, v.path, v.name));
      sampleListEl.appendChild(btn);
    });
  } catch (err) {
    sampleListEl.innerHTML = '<div class="loading-samples">Could not load samples.</div>';
  }
}

async function selectSample(btn, path, name) {
  document.querySelectorAll(".sample-btn").forEach(b => b.classList.remove("active"));
  btn.classList.add("active");

  try {
    const res  = await fetch(`/use_sample?name=${encodeURIComponent(name)}`);
    const data = await res.json();
    if (data.success) {
      setSelectedVideo(data.video_path, data.filename);
      showToast(`✓ Selected clip: ${data.filename}`);
    } else {
      showToast(`✗ ${data.message}`, 5000);
    }
  } catch (err) {
    showToast(`✗ ${err.message}`, 5000);
  }
}

function setSelectedVideo(path, name) {
  selectedVideoPath = path;
  selectedFileDisplay.style.display = "flex";
  selectedFileNameEl.textContent    = name;
  sysMode.textContent               = "Recorded Video";
  // Un-highlight road buttons to indicate video override
  roadButtons.forEach(b => b.classList.remove("active"));
  currentRoadSubbadge.textContent = "Video Mode";
}

function clearSelectedVideo() {
  selectedVideoPath = "";
  selectedFileDisplay.style.display = "none";
  selectedFileNameEl.textContent    = "—";
  sysMode.textContent               = "Curved Simulator";
  document.querySelectorAll(".sample-btn").forEach(b => b.classList.remove("active"));
}

if (sfClearBtn) {
  sfClearBtn.addEventListener("click", () => {
    clearSelectedVideo();
    selectRoadType(currentRoadType, true);
    showToast("Reverted to Curved Road Simulator.");
  });
}


// ─────────────────────────────────────────────
//  MJPEG Stream Control & Status Polling
// ─────────────────────────────────────────────

function startFeed() {
  isRunning = true;
  videoFeed.src = `/video_feed?t=${Date.now()}`;
  videoFeed.style.display = "block";
  videoPlaceholder.style.display = "none";

  btnStart.disabled = true;
  btnStop.disabled  = false;

  startStatusPolling();
  animatePipeline();
}

function stopFeed() {
  isRunning = false;
  videoFeed.src = "";
  videoFeed.style.display = "none";
  videoPlaceholder.style.display = "flex";

  btnStart.disabled = false;
  btnStop.disabled  = true;

  stopStatusPolling();
}

function startStatusPolling() {
  stopStatusPolling();
  statusPollTimer = setInterval(pollStatus, 250);   // ~4 polls/sec
}

function stopStatusPolling() {
  if (statusPollTimer) {
    clearInterval(statusPollTimer);
    statusPollTimer = null;
  }
}

async function pollStatus() {
  try {
    const res  = await fetch("/status");
    const data = await res.json();
    updateDashboard(data);
  } catch (_) { /* ignore transient network blips */ }
}


// ─────────────────────────────────────────────
//  Telemetry Dashboard Update
// ─────────────────────────────────────────────

function updateDashboard(d) {
  const isLkaOn  = typeof d.lka_enabled === "boolean" ? d.lka_enabled : true;
  const lkaStat  = d.lka_status  || (isLkaOn ? "ACTIVE" : "DISABLED");
  const laneStat = d.lane_status || "---";
  const steering = d.steering    || "---";
  const dev      = typeof d.deviation === "number" ? d.deviation : 0.0;
  const steerAng = typeof d.correction_val === "number" ? d.correction_val : 0.0;
  const roadName = d.road_name   || "Straight Road";
  const curv     = typeof d.curvature === "number" ? d.curvature : 0.0;
  const isLdw    = Boolean(d.ldw_active || d.ldw_warning || Math.abs(dev) >= 0.75);
  const fps      = d.fps         || 0;
  const mode     = d.mode        || (isLkaOn ? "LKA ASSISTED" : "MANUAL");

  // Keep toggle synced if backend changes
  if (lkaSwitch.checked !== isLkaOn) {
    lkaSwitch.checked = isLkaOn;
  }
  updateLKAUI(isLkaOn);

  // ── Overlay Badges ─────────────────────────
  badgeLKA.textContent  = `LKA: ${lkaStat}`;
  badgeRoad.textContent = roadName;
  badgeFPS.textContent  = `FPS: ${fps}`;
  badgeMode.textContent = `MODE: ${mode}`;

  // Lane Departure Warning Overlay Banner
  badgeLDW.style.display = isLdw ? "block" : "none";

  // ── System Info ────────────────────────────
  sysStatus.textContent    = d.running ? "RUNNING" : "STOPPED";
  sysFps.textContent       = fps ? `${fps} fps` : "—";
  sysCurvature.textContent = curv.toFixed(4);
  if (d.simulation_mode === "video") {
    sysMode.textContent = "Recorded Video";
  } else {
    sysMode.textContent = "Curved Simulator";
  }

  // ── Lane Status Card ───────────────────────
  valLaneStatus.textContent = laneStat;
  valLaneStatus.className   = "sc-value";
  cardLaneStatus.className  = "status-card";

  if (isLdw) {
    valLaneStatus.classList.add("danger");
    cardLaneStatus.classList.add("danger");
  } else if (laneStat === "CENTERED") {
    valLaneStatus.classList.add("centered");
    cardLaneStatus.classList.add("good");
  } else if (laneStat.startsWith("DRIFTING")) {
    valLaneStatus.classList.add("drifting");
    cardLaneStatus.classList.add("alert");
  } else {
    valLaneStatus.classList.add("no-lane");
    cardLaneStatus.classList.add("danger");
  }

  // ── Steering Decision Card ─────────────────
  valSteering.className   = "sc-value steering-value";
  cardSteering.className  = "status-card";

  if (!isLkaOn) {
    valSteering.textContent = "MANUAL CONTROL";
    valSteering.classList.add("manual");
    cardSteering.classList.add("manual");
  } else {
    valSteering.textContent = steering;
    if (steering === "KEEP STRAIGHT") {
      valSteering.classList.add("straight");
      cardSteering.classList.add("good");
    } else if (steering === "STEER LEFT") {
      valSteering.classList.add("steer-l");
      cardSteering.classList.add("alert");
    } else if (steering === "STEER RIGHT") {
      valSteering.classList.add("steer-r");
      cardSteering.classList.add("alert");
    }
  }

  // ── Steering Correction Angle Card ─────────
  if (!isLkaOn) {
    valSteerAngle.textContent = "0.0° [OFF]";
    valSteerAngle.style.color = "var(--text-muted)";
  } else {
    valSteerAngle.textContent = `${steerAng >= 0 ? "+" : ""}${steerAng.toFixed(1)}°`;
    valSteerAngle.style.color = Math.abs(steerAng) > 1.2 ? "var(--accent)" : "var(--green)";
  }

  // ── Lane Deviation Card & Bar ──────────────
  valDeviation.textContent = `${dev >= 0 ? "+" : ""}${dev.toFixed(3)} m`;
  updateDeviationBar(dev, isLdw);

  // ── Road Geometry Card ─────────────────────
  valRoadType.textContent = `${roadName}`;

  // ── Lane Departure Warning Card ────────────
  cardLDW.className = "status-card";
  if (isLdw) {
    valLDW.textContent = "⚠️ WARNING TRIGGERED";
    valLDW.style.color = "#ef5350";
    cardLDW.classList.add("danger");
  } else if (Math.abs(dev) > 0.35) {
    valLDW.textContent = "APPROACHING BOUNDARY";
    valLDW.style.color = "var(--orange)";
    cardLDW.classList.add("alert");
  } else {
    valLDW.textContent = "SAFE IN LANE";
    valLDW.style.color = "var(--green)";
    cardLDW.classList.add("good");
  }

  // ── Steering Visualiser ────────────────────
  updateSteeringViz(isLkaOn, steering, steerAng, dev);

  // ── Charts Update ──────────────────────────
  // IMPORTANT: When LKA is OFF, steering correction MUST strictly remain 0.0!
  const activeCorrection = isLkaOn ? steerAng : 0.0;

  push(timeLabels,       timeLabel());
  push(deviationData,    dev);
  push(steeringCorrData, activeCorrection);
  updateCharts();
}

function updateDeviationBar(dev, isLdw) {
  // Scale ±1.75m to ±50% width
  const pct = Math.min(Math.abs(dev) / 1.75, 1) * 50;
  devBarFill.style.width = `${pct}%`;
  devBarFill.style.left  = dev < 0 ? `${50 - pct}%` : "50%";

  if (isLdw) {
    devBarFill.style.background = "#ef5350";
  } else if (Math.abs(dev) > 0.3) {
    devBarFill.style.background = "var(--orange)";
  } else {
    devBarFill.style.background = "var(--green)";
  }
}

function updateSteeringViz(isLkaOn, steering, steerAng, dev) {
  // Car horizontal position on track
  const maxShift = 55;
  const shift    = Math.max(-maxShift, Math.min(maxShift, (dev / 1.75) * maxShift));
  svCar.style.left = `calc(50% + ${shift}px)`;

  if (!isLkaOn) {
    // When LKA is OFF, do not show automatic correction arrow!
    svArrow.textContent = "·";
    svArrow.style.color = "var(--text-muted)";
    svLabel.textContent = "MANUAL CONTROL";
    svLabel.style.color = "var(--text-muted)";
  } else {
    if (steering === "STEER LEFT") {
      svArrow.textContent = "⬅";
      svArrow.style.color = "#42a5f5";
    } else if (steering === "STEER RIGHT") {
      svArrow.textContent = "➡";
      svArrow.style.color = "#ef5350";
    } else {
      svArrow.textContent = "⬆";
      svArrow.style.color = "var(--green)";
    }
    svLabel.textContent = `${steering} (${steerAng >= 0 ? "+" : ""}${steerAng.toFixed(1)}°)`;
    svLabel.style.color = "var(--text-primary)";
  }
}

function resetCards() {
  valLaneStatus.textContent = "CENTERED";
  valSteering.textContent   = "KEEP STRAIGHT";
  valSteerAngle.textContent = "0.0°";
  valDeviation.textContent  = "0.00 m";
  valRoadType.textContent   = "Straight Road";
  valLDW.textContent        = "SAFE";
  valLDW.style.color        = "var(--green)";
  cardLDW.className         = "status-card good";
  badgeLDW.style.display    = "none";
  devBarFill.style.width    = "0";
  svCar.style.left          = "50%";
  svArrow.textContent       = "⬆";
  svLabel.textContent       = "KEEP STRAIGHT";
  sysStatus.textContent     = "IDLE";
  sysFps.textContent        = "—";
  sysCurvature.textContent  = "0.0000";
}


// ─────────────────────────────────────────────
//  Mini Pipeline Animation
// ─────────────────────────────────────────────

let pipeIdx = 0;
let pipeTimer = null;

function animatePipeline() {
  const steps = document.querySelectorAll(".mp-step");
  if (!steps.length) return;
  if (pipeTimer) clearInterval(pipeTimer);
  pipeTimer = setInterval(() => {
    if (!isRunning) { clearInterval(pipeTimer); return; }
    steps.forEach(s => s.classList.remove("active"));
    steps[pipeIdx % steps.length].classList.add("active");
    pipeIdx++;
  }, 350);
}


// ─────────────────────────────────────────────
//  Chart.js Graphs Setup
// ─────────────────────────────────────────────

const chartDefaults = {
  responsive: true,
  maintainAspectRatio: false,
  animation: { duration: 0 },
  plugins: {
    legend: { display: false },
    tooltip: { enabled: true },
  },
  scales: {
    x: {
      ticks: { color: "#546e7a", font: { size: 9 }, maxTicksLimit: 8, maxRotation: 0 },
      grid:  { color: "rgba(255,255,255,0.04)" },
    },
    y: {
      ticks: { color: "#546e7a", font: { size: 9 } },
      grid:  { color: "rgba(255,255,255,0.05)" },
    },
  },
};

// 1. Lane Deviation Chart (with ±0.75m threshold bands)
const devCtx = document.getElementById("deviationChart").getContext("2d");
const deviationChart = new Chart(devCtx, {
  type: "line",
  data: {
    labels: timeLabels,
    datasets: [{
      label:           "Deviation",
      data:            deviationData,
      borderColor:     "#00d4ff",
      backgroundColor: "rgba(0, 212, 255, 0.08)",
      borderWidth:     2,
      pointRadius:     0,
      fill:            true,
      tension:         0.3,
    }],
  },
  options: {
    ...chartDefaults,
    scales: {
      ...chartDefaults.scales,
      y: {
        ...chartDefaults.scales.y,
        min: -1.8,
        max: 1.8,
        title: { display: true, text: "Metres (m)", color: "#546e7a", font: { size: 9 } },
      },
    },
  },
});

// 2. Steering Correction Angle Chart (Degrees)
const steerCtx = document.getElementById("steeringChart").getContext("2d");
const steeringChart = new Chart(steerCtx, {
  type: "line",
  data: {
    labels: timeLabels,
    datasets: [{
      label:           "Correction Angle",
      data:            steeringCorrData,
      borderColor:     "#00e676",
      backgroundColor: "rgba(0, 230, 118, 0.08)",
      borderWidth:     2,
      pointRadius:     0,
      fill:            true,
      tension:         0.3,
    }],
  },
  options: {
    ...chartDefaults,
    scales: {
      ...chartDefaults.scales,
      y: {
        ...chartDefaults.scales.y,
        min: -20,
        max: 20,
        title: { display: true, text: "Correction (°)", color: "#546e7a", font: { size: 9 } },
      },
    },
  },
});

function updateCharts() {
  deviationChart.data.labels   = [...timeLabels];
  deviationChart.data.datasets[0].data = [...deviationData];

  // Adjust deviation border color if in LDW state
  const lastDev = deviationData[deviationData.length - 1] || 0;
  if (Math.abs(lastDev) >= 0.75) {
    deviationChart.data.datasets[0].borderColor = "#ef5350";
    deviationChart.data.datasets[0].backgroundColor = "rgba(239, 83, 80, 0.15)";
  } else {
    deviationChart.data.datasets[0].borderColor = "#00d4ff";
    deviationChart.data.datasets[0].backgroundColor = "rgba(0, 212, 255, 0.08)";
  }

  steeringChart.data.labels    = [...timeLabels];
  steeringChart.data.datasets[0].data = [...steeringCorrData];

  // If LKA is disabled, color chart neutral grey
  if (!lkaEnabled) {
    steeringChart.data.datasets[0].borderColor = "#78909c";
    steeringChart.data.datasets[0].backgroundColor = "rgba(120, 144, 156, 0.05)";
  } else {
    steeringChart.data.datasets[0].borderColor = "#00e676";
    steeringChart.data.datasets[0].backgroundColor = "rgba(0, 230, 118, 0.08)";
  }

  deviationChart.update("none");
  steeringChart.update("none");
}

function resetCharts() {
  timeLabels.length       = 0;
  deviationData.length    = 0;
  steeringCorrData.length = 0;
  updateCharts();
}


// ─────────────────────────────────────────────
//  Initialization
// ─────────────────────────────────────────────

document.addEventListener("DOMContentLoaded", () => {
  loadSampleList();
  resetCards();
  updateLKAUI(true);
  selectRoadType("straight", false);
  btnStart.disabled = false;
});
