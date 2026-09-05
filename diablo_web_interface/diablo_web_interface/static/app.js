/* Browser client for the Diablo Nav2 web node. */

const $ = (id) => document.getElementById(id);

let stateSocket = null;
let topicSocket = null;
let reconnectTimer = null;
let topicReconnectTimer = null;
let config = {};
let topicCatalog = [];
let selectedTopics = [];
let mapData = makePreviewMap();
let robotPose = { x: 0.62, y: 0.42, theta: 0.18, source: "local preview /map" };
let wheelRobotPose = { x: 0.62, y: 0.42, theta: 0.18, source: "local preview /odometry/filtered" };
let navPath = makePreviewPath();
let lidarScan = makePreviewScan();
let localCostmap = makePreviewCostmap(mapData, 2);
let globalCostmap = makePreviewCostmap(mapData, 3);
let goalDraft = null;
let initialPoseDraft = null;
let activeMapTool = "goal";
let stations = [];
let pressedKeys = new Set();
let manualActive = false;
let hardwareReady = false;
let lastConnectionLog = 0;
let lastNavigationLog = "";

function makePreviewMap() {
  const width = 72;
  const height = 46;
  const data = Array.from({ length: width * height }, (_, index) => {
    const x = index % width;
    const y = Math.floor(index / width);
    const border = x === 0 || y === 0 || x === width - 1 || y === height - 1;
    const blockA = x > 20 && x < 27 && y > 9 && y < 36;
    const blockB = x > 39 && x < 58 && y > 27 && y < 33;
    const blockC = x > 60 && x < 66 && y > 8 && y < 21;
    return border || blockA || blockB || blockC ? 100 : 0;
  });
  return {
    frame_id: "map",
    resolution: 0.05,
    width,
    height,
    origin: { x: -1, y: -1, yaw: 0 },
    data,
  };
}

function makePreviewCostmap(grid, radius) {
  const data = grid.data.map((value, index) => {
    if (value >= 65) return 100;
    const x = index % grid.width;
    const y = Math.floor(index / grid.width);
    for (let dy = -radius; dy <= radius; dy += 1) {
      for (let dx = -radius; dx <= radius; dx += 1) {
        const distance = Math.hypot(dx, dy);
        const neighbor = grid.data[(y + dy) * grid.width + (x + dx)];
        if (distance <= radius && neighbor !== undefined && neighbor >= 65) {
          return Math.max(24, Math.round(95 - distance * 18));
        }
      }
    }
    return 0;
  });
  return { ...grid, data };
}

function makePreviewPath() {
  return {
    frame_id: "map",
    poses: Array.from({ length: 20 }, (_, index) => ({
      x: 0.45 + index * 0.11,
      y: 0.34 + Math.sin(index / 4) * 0.15,
    })),
  };
}

function makePreviewScan() {
  const ranges = Array.from({ length: 144 }, (_, index) => index > 62 && index < 82 ? 1.15 : 2.35 + Math.sin(index / 8) * 0.2);
  return {
    frame_id: "laser",
    angle_min: -Math.PI,
    angle_increment: (Math.PI * 2) / ranges.length,
    range_min: 0.12,
    range_max: 8,
    ranges,
  };
}

const canvas = $("nav-canvas");
const context = canvas.getContext("2d");

function formatNumber(value, digits = 2) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "—";
}

function sendStateCommand(payload, fallbackPath = null) {
  if (stateSocket && stateSocket.readyState === WebSocket.OPEN) {
    stateSocket.send(JSON.stringify(payload));
    return;
  }
  if (fallbackPath) {
    fetch(fallbackPath, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    }).catch((error) => addLog(`Command failed: ${error}`, "error"));
  }
}

function addLog(message, kind = "info") {
  const log = $("event-log");
  const line = document.createElement("div");
  line.className = `log-line ${kind}`;
  const time = document.createElement("time");
  time.textContent = new Date().toLocaleTimeString([], { hour12: false });
  const text = document.createElement("span");
  text.textContent = message;
  line.append(time, text);
  log.appendChild(line);
  while (log.children.length > 120) log.removeChild(log.firstChild);
  log.scrollTop = log.scrollHeight;
}

function setConnection(online) {
  const chip = $("connection-dot").parentElement;
  chip.classList.toggle("online", online);
  $("connection-text").textContent = online ? "ROS CONNECTED" : "DISCONNECTED";
}

function setNav2Ready(ready) {
  const chip = $("nav2-dot").parentElement;
  chip.classList.toggle("online", Boolean(ready));
  $("nav2-text").textContent = ready ? "NAV2 READY" : "NAV2 OFFLINE";
}

function setMode(mode) {
  const value = String(mode || "manual").toUpperCase();
  $("mode-text").textContent = value;
  $("mode-text").style.color = value === "AUTO" ? "var(--blue)" : "var(--amber)";
}

function connectStateSocket() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  stateSocket = new WebSocket(`${scheme}://${window.location.host}/ws`);
  stateSocket.onopen = () => {
    setConnection(true);
    if (Date.now() - lastConnectionLog > 10000) {
      addLog("Connected to Diablo ROS bridge.", "success");
      lastConnectionLog = Date.now();
    }
  };
  stateSocket.onmessage = (event) => {
    try {
      const packet = JSON.parse(event.data);
      if (packet.type === "state") handleState(packet);
      if (packet.type === "goal_pose_ack") {
        if (packet.accepted) addLog("Nav2 goal submitted.", "success");
        else addLog(packet.message || "Nav2 rejected the goal.", "error");
      }
      if (packet.type === "goal_cancel_ack") addLog("Navigation cancel requested.", "warn");
      if (["start_hardware_ack", "start_localization_ack", "start_navigation_ack", "start_mapping_ack", "start_lidar_ack"].includes(packet.type)) {
        addLog(packet.message || "Startup command acknowledged.", packet.requested === false ? "warn" : "success");
      }
      if (packet.type === "error") addLog(packet.detail || "WebSocket error", "error");
    } catch (error) {
      addLog(`Invalid state packet: ${error}`, "error");
    }
  };
  stateSocket.onerror = () => setConnection(false);
  stateSocket.onclose = () => {
    setConnection(false);
    if (!reconnectTimer) {
      reconnectTimer = setInterval(() => {
        if (!stateSocket || stateSocket.readyState === WebSocket.CLOSED) {
          clearInterval(reconnectTimer);
          reconnectTimer = null;
          connectStateSocket();
        }
      }, 2000);
    }
  };
}

function handleState(packet) {
  setNav2Ready(packet.nav2_ready);
  setMode(packet.control_mode);
  const hasWheelPose = Object.prototype.hasOwnProperty.call(packet, "wheel_pose");
  if (hasWheelPose) wheelRobotPose = packet.wheel_pose;
  if (packet.pose !== undefined) {
    robotPose = packet.pose;
    updatePoseUi();
  } else if (hasWheelPose) updatePoseUi();
  if (packet.telemetry) updateTelemetry(packet.telemetry);
  if (packet.nav_goal) updateNavigationStatus(packet.nav_goal);
  if (packet.hardware) updateHardwareUi(packet.hardware);
  if (Object.prototype.hasOwnProperty.call(packet, "map")) {
    mapData = packet.map;
    $("map-overlay").classList.toggle("has-map", Boolean(mapData));
    $("map-overlay").textContent = mapData ? "" : "Menunggu /map…";
  }
  if (Object.prototype.hasOwnProperty.call(packet, "local_costmap")) localCostmap = packet.local_costmap;
  if (Object.prototype.hasOwnProperty.call(packet, "global_costmap")) globalCostmap = packet.global_costmap;
  updateCostmapUi();
  if (Object.prototype.hasOwnProperty.call(packet, "path")) navPath = packet.path;
  if (Object.prototype.hasOwnProperty.call(packet, "scan")) lidarScan = packet.scan;
  drawNavigation();
}

function updatePoseUi() {
  const previewPose = wheelRobotPose;
  if (previewPose) {
    $("preview-x-position").textContent = formatNumber(previewPose.x);
    $("preview-y-position").textContent = formatNumber(previewPose.y);
    $("preview-heading").textContent = formatNumber(Number(previewPose.theta) * 180 / Math.PI, 3);
  } else {
    $("preview-x-position").textContent = "—";
    $("preview-y-position").textContent = "—";
    $("preview-heading").textContent = "—";
  }
  $("nav-robot-x").textContent = robotPose ? formatNumber(robotPose.x) : "—";
  $("nav-robot-y").textContent = robotPose ? formatNumber(robotPose.y) : "—";
  $("nav-robot-heading").textContent = robotPose ? formatNumber(Number(robotPose.theta) * 180 / Math.PI, 1) : "—";
  $("position-metric").textContent = robotPose ? `${formatNumber(robotPose.x)} , ${formatNumber(robotPose.y)}` : "—";
  $("position-source").textContent = robotPose ? robotPose.source || "unknown" : "waiting for TF / odometry/filtered";
}

function updateHardwareUi(hardware) {
  const ready = Boolean(hardware && hardware.ready);
  const starting = Boolean(hardware && hardware.starting);
  hardwareReady = ready;
  $("hardware-status-preview").textContent = ready ? "READY" : starting ? "STARTING" : "IDLE";
  $("hardware-message-preview").textContent = hardware?.message || "Start hardware before using Drive Control.";
  $("hardware-status-preview").style.color = ready ? "var(--green)" : starting ? "var(--amber)" : "var(--muted)";
  const components = Object.fromEntries((hardware?.components || []).map((item) => [item.id, item]));
  [
    ["hardware-diablo-preview", "diablo", "DIABLO ROS2"],
    ["hardware-lidar-preview", "lidar", "LIDAR"],
    ["hardware-dxl-preview", "dynamixel", "DYNAMIXEL U2D2"],
  ].forEach(([id, key, label]) => {
    const item = components[key];
    $(id).textContent = `● ${label} · ${(item?.state || "offline").replace("_", " ").toUpperCase()}`;
  });
  const button = $("start-hardware");
  button.disabled = ready || starting;
  button.textContent = ready ? "HARDWARE READY" : starting ? "STARTING…" : "START HARDWARE";
  document.querySelectorAll(".key, #stand-up, #stand-down, #height-input, #pitch-input, #roll-input").forEach((control) => { control.disabled = !ready; });
}

function updateCostmapUi() {
  $("global-costmap-status").textContent = globalCostmap ? `${globalCostmap.width}×${globalCostmap.height} RECEIVING` : "WAITING";
  $("local-costmap-status").textContent = localCostmap ? `${localCostmap.width}×${localCostmap.height} RECEIVING` : "WAITING";
  $("show-global-costmap").disabled = !globalCostmap;
  $("show-local-costmap").disabled = !localCostmap;
}

function updateTelemetry(telemetry) {
  const battery = telemetry.battery;
  if (battery) {
    $("battery-percent").textContent = battery.percentage == null ? "—" : `${formatNumber(battery.percentage, 0)}%`;
    $("battery-voltage").textContent = `${formatNumber(battery.voltage)} V`;
  }
  const body = telemetry.body_state;
  if (body) {
    $("robot-mode").textContent = `MODE ${body.robot_mode}`;
    $("body-error").textContent = body.error ? `error 0x${body.error.toString(16)}` : "no error";
  }
  const imu = telemetry.imu;
  if (imu) {
    $("imu-yaw").textContent = `${formatNumber(imu.yaw)} rad`;
    $("imu-rpy").textContent = `roll ${formatNumber(imu.roll)} · pitch ${formatNumber(imu.pitch)}`;
  }
}

function updateNavigationStatus(status) {
  const label = String(status.state || "idle").toUpperCase();
  const distance = status.distance_remaining == null ? "" : ` · ${formatNumber(status.distance_remaining)} m`;
  $("nav-state").textContent = `${label} · ${status.message || ""}${distance}`;
  const logKey = `${label}:${status.seq || 0}:${status.message || ""}`;
  if (logKey === lastNavigationLog) return;
  lastNavigationLog = logKey;
  const history = $("navigation-history");
  if (history.querySelector(".event-empty")) history.innerHTML = "";
  const line = document.createElement("div");
  line.className = "log-line";
  const time = document.createElement("time");
  time.textContent = new Date().toLocaleTimeString([], { hour12: false });
  const message = document.createElement("span");
  message.textContent = `${label} · ${status.message || ""}`;
  line.append(time, message);
  history.appendChild(line);
  while (history.children.length > 12) history.removeChild(history.firstChild);
  if (label === "SUCCEEDED") addLog("Nav2 goal reached.", "success");
  if (label === "ABORTED" || label === "ERROR" || label === "REJECTED") addLog(status.message || "Navigation failed.", "error");
}

async function loadConfig() {
  try {
    const response = await fetch("/api/config");
    config = await response.json();
    $("manual-topic").textContent = config.manual_cmd_topic || "—";
    $("odom-topic").textContent = `${config.odom_topic || "—"} / ${config.base_frame || "—"}`;
    $("scan-topic").textContent = config.scan_topic || "—";
  } catch (error) {
    addLog(`Config unavailable: ${error}`, "warn");
  }
}

async function loadMaps() {
  try {
    const response = await fetch("/api/maps");
    const names = await response.json();
    const select = $("map-select");
    (Array.isArray(names) ? names : []).forEach((name) => {
      const option = document.createElement("option");
      option.value = name;
      option.textContent = name;
      select.appendChild(option);
    });
  } catch {
    // The static preview remains usable without the FastAPI map catalog.
  }
}

async function loadTopics() {
  try {
    const response = await fetch("/api/topics");
    const result = await response.json();
    topicCatalog = result.topics || [];
    renderTopicOptions();
  } catch (error) {
    addLog(`Topic list unavailable: ${error}`, "warn");
  }
}

function renderTopicOptions() {
  const filter = $("topic-filter").value.toLowerCase();
  const select = $("topic-select");
  const previous = select.value;
  select.innerHTML = "";
  topicCatalog
    .filter((item) => !filter || item.name.toLowerCase().includes(filter))
    .forEach((item) => {
      const option = document.createElement("option");
      option.value = item.name;
      option.textContent = `${item.name}  ·  ${(item.types || []).join(", ")}`;
      select.appendChild(option);
    });
  if ([...select.options].some((option) => option.value === previous)) select.value = previous;
}

function connectTopicSocket() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  topicSocket = new WebSocket(`${scheme}://${window.location.host}/ws/topics`);
  topicSocket.onopen = () => sendTopicSubscription();
  topicSocket.onmessage = (event) => {
    try {
      const packet = JSON.parse(event.data);
      if (packet.type === "topic") renderTopicCard(packet);
      if (packet.type === "error") addLog(packet.detail || "Topic echo error", "error");
    } catch (error) {
      addLog(`Invalid topic packet: ${error}`, "error");
    }
  };
  topicSocket.onclose = () => {
    if (!topicReconnectTimer) {
      topicReconnectTimer = setTimeout(() => {
        topicReconnectTimer = null;
        connectTopicSocket();
      }, 2500);
    }
  };
}

function sendTopicSubscription() {
  if (!topicSocket || topicSocket.readyState !== WebSocket.OPEN) return;
  topicSocket.send(JSON.stringify({ type: "subscribe", topics: selectedTopics }));
  renderTopicCardsPlaceholder();
}

function renderTopicCardsPlaceholder() {
  const container = $("topic-cards");
  if (!selectedTopics.length) {
    container.innerHTML = '<div class="empty-state">Pilih topic untuk mulai echo.</div>';
    return;
  }
  container.innerHTML = "";
  selectedTopics.forEach((topic, index) => {
    const card = document.createElement("article");
    card.className = "topic-card";
    card.dataset.slot = String(index + 1);
    card.innerHTML = `<div class="topic-card-header"><strong></strong><small>waiting</small></div><div class="topic-meta"></div><pre>{}</pre>`;
    card.querySelector("strong").textContent = topic;
    container.appendChild(card);
  });
}

function renderTopicCard(packet) {
  const card = $("topic-cards").querySelector(`[data-slot="${packet.slot}"]`);
  if (!card) return;
  card.querySelector("strong").textContent = packet.topic;
  card.querySelector("small").textContent = `${packet.count} msg`;
  card.querySelector(".topic-meta").textContent = `${packet.msg_type} · ${packet.stamp}`;
  card.querySelector("pre").textContent = JSON.stringify(packet.data, null, 2);
}

function clearTopics() {
  selectedTopics = [];
  if (topicSocket && topicSocket.readyState === WebSocket.OPEN) {
    topicSocket.send(JSON.stringify({ type: "clear" }));
  }
  renderTopicCardsPlaceholder();
}

function setupTabs() {
  document.querySelectorAll(".tab").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".tab").forEach((item) => item.classList.remove("active"));
      document.querySelectorAll(".tab-panel").forEach((item) => item.classList.remove("active"));
      button.classList.add("active");
      $(`tab-${button.dataset.tab}`).classList.add("active");
      if (button.dataset.tab === "navigation") resizeCanvas();
    });
  });
}

function setupPreviewPanels() {
  document.querySelectorAll(".preview-collapse").forEach((button) => {
    button.addEventListener("click", () => {
      const panel = button.closest(".preview-collapsible");
      if (!panel) return;
      const collapsed = panel.classList.toggle("collapsed");
      panel.querySelectorAll(".preview-collapsible-body").forEach((body) => { body.style.display = collapsed ? "none" : ""; });
      button.textContent = collapsed ? "+" : "−";
    });
  });
  $("preview-sidebar-toggle").addEventListener("click", () => {
    const collapsed = document.querySelector(".dashboard").classList.toggle("preview-sidebar-collapsed");
    $("preview-sidebar-toggle").textContent = collapsed ? "›" : "‹";
  });
}

function setupTeleop() {
  const forward = $("forward-input");
  const turn = $("turn-input");
  const roll = $("roll-input");
  const height = $("height-input");
  const pitch = $("pitch-input");

  function updateControlLabels() {
    $("forward-output").value = Number(forward.value).toFixed(2);
    $("turn-output").value = Number(turn.value).toFixed(2);
    $("roll-output").value = Number(roll.value).toFixed(2);
    $("height-output").value = Number(height.value).toFixed(2);
    $("pitch-output").value = Number(pitch.value).toFixed(2);
    $("forward-value").textContent = Number(forward.value).toFixed(2);
    $("turn-value").textContent = Number(turn.value).toFixed(2);
  }

  function commandFromInputs() {
    let commandForward = Number(forward.value);
    let commandTurn = Number(turn.value);
    let commandRoll = Number(roll.value);
    if (pressedKeys.has("w") || pressedKeys.has("s")) commandForward = pressedKeys.has("w") ? 1 : -1;
    if (pressedKeys.has("a") || pressedKeys.has("d")) commandTurn = pressedKeys.has("a") ? 1 : -1;
    if (pressedKeys.has("q") || pressedKeys.has("e")) commandRoll = pressedKeys.has("e") ? 0.1 : -0.1;
    return {
      type: "manual",
      forward: commandForward,
      left: commandTurn,
      roll: commandRoll,
      up: Number(height.value),
      pitch: Number(pitch.value),
    };
  }

  window.sendCurrentManual = () => { if (hardwareReady) sendStateCommand(commandFromInputs(), "/api/teleop"); };
  [forward, turn, roll, height, pitch].forEach((input) => input.addEventListener("input", () => {
    updateControlLabels();
    manualActive = true;
    sendCurrentManual();
  }));
  updateControlLabels();

  document.querySelectorAll(".key[data-key]").forEach((button) => {
    const key = button.dataset.key;
    const press = (event) => {
      event.preventDefault();
      if (!hardwareReady) { addLog("Start Hardware first; Drive Control is locked.", "warn"); return; }
      pressedKeys.add(key);
      button.classList.add("active");
      manualActive = true;
      sendCurrentManual();
    };
    const release = (event) => {
      event.preventDefault();
      pressedKeys.delete(key);
      button.classList.remove("active");
      if (!pressedKeys.size) {
        manualActive = false;
        sendStateCommand({ type: "stop" }, "/api/control/stop");
      }
    };
    button.addEventListener("pointerdown", press);
    button.addEventListener("pointerup", release);
    button.addEventListener("pointerleave", release);
  });

  window.addEventListener("keydown", (event) => {
    const key = event.key.toLowerCase();
    if (!["w", "a", "s", "d", "q", "e"].includes(key)) return;
    if (!hardwareReady) { addLog("Start Hardware first; Drive Control is locked.", "warn"); return; }
    event.preventDefault();
    pressedKeys.add(key);
    const button = document.querySelector(`.key[data-key="${key}"]`);
    if (button) button.classList.add("active");
    manualActive = true;
  });
  window.addEventListener("keyup", (event) => {
    const key = event.key.toLowerCase();
    if (!pressedKeys.has(key)) return;
    pressedKeys.delete(key);
    const button = document.querySelector(`.key[data-key="${key}"]`);
    if (button) button.classList.remove("active");
    if (!pressedKeys.size) {
      manualActive = false;
      sendStateCommand({ type: "stop" }, "/api/control/stop");
    }
  });
  window.setInterval(() => {
    if (manualActive) sendCurrentManual();
  }, 50);

  $("release-joystick").addEventListener("click", () => {
    forward.value = "0";
    turn.value = "0";
    roll.value = "0";
    pressedKeys.clear();
    manualActive = false;
    updateControlLabels();
    sendStateCommand({ type: "stop" }, "/api/control/stop");
  });
  $("stand-up").addEventListener("click", () => {
    if (!hardwareReady) return;
    sendStateCommand({ type: "stand", stand: true }, "/api/teleop/stand");
    addLog("Stand-up command sent.", "success");
  });
  $("stand-down").addEventListener("click", () => {
    if (!hardwareReady) return;
    sendStateCommand({ type: "stand", stand: false }, "/api/teleop/stand");
    addLog("Stand-down command sent.", "warn");
  });
  $("reset-odom").addEventListener("click", () => {
    sendStateCommand({ type: "reset_odom" }, "/api/odom/reset");
    addLog("Reset odom requested.", "success");
  });
  $("reset-encoder").addEventListener("click", () => {
    sendStateCommand({ type: "reset_encoder" }, "/api/encoder/reset");
    addLog("Reset encoder reference requested.", "success");
  });
  $("start-lidar").addEventListener("click", () => {
    sendStateCommand({ type: "start_lidar" }, "/api/sensors/lidar/start");
    addLog("Start LiDAR requested.", "info");
  });
  $("emergency-stop").addEventListener("click", () => {
    pressedKeys.clear();
    manualActive = false;
    sendStateCommand({ type: "stop" }, "/api/control/stop");
    addLog("STOP command sent.", "error");
  });
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * ratio));
  canvas.height = Math.max(1, Math.floor(rect.height * ratio));
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  drawNavigation();
}

function mapView() {
  if (!mapData || !mapData.width || !mapData.height) return null;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const cell = Math.min((width - 24) / mapData.width, (height - 24) / mapData.height);
  return {
    cell,
    offsetX: (width - mapData.width * cell) / 2,
    offsetY: (height - mapData.height * cell) / 2,
  };
}

function worldToCanvas(x, y, view = mapView()) {
  if (!view || !mapData) return null;
  const origin = mapData.origin || { x: 0, y: 0, yaw: 0 };
  const dx = Number(x) - origin.x;
  const dy = Number(y) - origin.y;
  const angle = origin.yaw || 0;
  const gx = (Math.cos(angle) * dx + Math.sin(angle) * dy) / mapData.resolution;
  const gy = (-Math.sin(angle) * dx + Math.cos(angle) * dy) / mapData.resolution;
  return [view.offsetX + gx * view.cell, view.offsetY + (mapData.height - gy) * view.cell];
}

function canvasToWorld(x, y, view = mapView()) {
  if (!view || !mapData) return null;
  const gx = (x - view.offsetX) / view.cell;
  const gy = mapData.height - (y - view.offsetY) / view.cell;
  const localX = gx * mapData.resolution;
  const localY = gy * mapData.resolution;
  const origin = mapData.origin || { x: 0, y: 0, yaw: 0 };
  const angle = origin.yaw || 0;
  return {
    x: origin.x + Math.cos(angle) * localX - Math.sin(angle) * localY,
    y: origin.y + Math.sin(angle) * localX + Math.cos(angle) * localY,
  };
}

function drawNavigation() {
  if (!canvas || !context) return;
  const overlay = $("map-overlay");
  if (overlay) {
    overlay.classList.toggle("has-map", Boolean(mapData));
    overlay.textContent = mapData ? "" : "Menunggu /map…";
  }
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#f8fbfd";
  context.fillRect(0, 0, width, height);
  const view = mapView();
  if (!view) return;

  for (let row = 0; row < mapData.height; row += 1) {
    for (let column = 0; column < mapData.width; column += 1) {
      const value = mapData.data[row * mapData.width + column];
      context.fillStyle = value < 0 ? "#e2e9ee" : value >= 65 ? "#42596a" : "#f8fbfd";
      context.fillRect(view.offsetX + column * view.cell, view.offsetY + (mapData.height - row - 1) * view.cell, Math.ceil(view.cell + .2), Math.ceil(view.cell + .2));
    }
  }

  drawCostmap(globalCostmap, view, "global");
  drawCostmap(localCostmap, view, "local");

  if (navPath && navPath.poses) {
    context.beginPath();
    context.strokeStyle = "#6ca8ff";
    context.lineWidth = 2;
    navPath.poses.forEach((point, index) => {
      const position = worldToCanvas(point.x, point.y, view);
      if (!position) return;
      if (index === 0) context.moveTo(position[0], position[1]);
      else context.lineTo(position[0], position[1]);
    });
    context.stroke();
  }

  if ($("show-lidar").checked && lidarScan && robotPose) drawLidar(view);
  if (initialPoseDraft) drawMarker(initialPoseDraft.x, initialPoseDraft.y, initialPoseDraft.theta, "#2c83a9", view, true);
  if (goalDraft) drawMarker(goalDraft.x, goalDraft.y, goalDraft.theta, "#c55300", view, true);
  if (robotPose) drawMarker(robotPose.x, robotPose.y, robotPose.theta, "#4f925c", view, false);
}

function drawCostmap(costmap, view, layer) {
  if (!costmap) return;
  const showLayer = $(layer === "global" ? "show-global-costmap" : "show-local-costmap");
  if (showLayer && !showLayer.checked) return;
  const showInflation = $("show-inflation");
  for (let row = 0; row < costmap.height; row += 1) {
    for (let column = 0; column < costmap.width; column += 1) {
      const value = Number(costmap.data[row * costmap.width + column]);
      if (!Number.isFinite(value) || value < 1) continue;
      if (value < 90 && showInflation && !showInflation.checked) continue;
      const origin = costmap.origin || { x: 0, y: 0, yaw: 0 };
      const angle = origin.yaw || 0;
      const worldX = origin.x + Math.cos(angle) * column * costmap.resolution - Math.sin(angle) * row * costmap.resolution;
      const worldY = origin.y + Math.sin(angle) * column * costmap.resolution + Math.cos(angle) * row * costmap.resolution;
      const point = worldToCanvas(worldX, worldY, view);
      if (!point) continue;
      const inflated = value < 90;
      context.fillStyle = layer === "global"
        ? `rgba(47,120,174,${inflated ? .23 : .55})`
        : `rgba(197,83,0,${inflated ? .23 : .55})`;
      const size = Math.max(1, view.cell * costmap.resolution / mapData.resolution + .5);
      context.fillRect(point[0], point[1] - size, size, size);
    }
  }
}

function drawLidar(view) {
  const start = worldToCanvas(robotPose.x, robotPose.y, view);
  if (!start) return;
  context.fillStyle = "rgba(77, 214, 209, .38)";
  const scale = view.cell / mapData.resolution;
  lidarScan.ranges.forEach((range, index) => {
    if (range == null || range < lidarScan.range_min || range > lidarScan.range_max) return;
    const angle = robotPose.theta + lidarScan.angle_min + index * lidarScan.angle_increment;
    const point = worldToCanvas(robotPose.x + range * Math.cos(angle), robotPose.y + range * Math.sin(angle), view);
    if (point) context.fillRect(point[0] - 1, point[1] - 1, Math.max(2, scale * .035), Math.max(2, scale * .035));
  });
}

function drawMarker(x, y, theta, color, view, ring) {
  const point = worldToCanvas(x, y, view);
  if (!point) return;
  context.save();
  context.translate(point[0], point[1]);
  context.rotate(-theta);
  context.fillStyle = color;
  context.strokeStyle = color;
  context.lineWidth = 2;
  if (ring) {
    context.beginPath();
    context.arc(0, 0, 8, 0, Math.PI * 2);
    context.stroke();
  }
  context.beginPath();
  context.moveTo(10, 0);
  context.lineTo(-7, -6);
  context.lineTo(-5, 6);
  context.closePath();
  context.fill();
  context.restore();
}

function setupNavigation() {
  const number = (id) => Number($(id).value);
  const readPose = (prefix) => ({
    x: number(`${prefix}-x`),
    y: number(`${prefix}-y`),
    theta: number(`${prefix}-heading`) * Math.PI / 180,
  });
  const validPose = (pose) => [pose.x, pose.y, pose.theta].every(Number.isFinite);

  document.querySelectorAll("[data-tool]").forEach((button) => {
    button.addEventListener("click", () => {
      activeMapTool = button.dataset.tool;
      document.querySelectorAll("[data-tool]").forEach((item) => item.classList.toggle("active", item.dataset.tool === activeMapTool));
      addLog(`${activeMapTool.toUpperCase()} map tool active.`, "info");
    });
  });
  canvas.addEventListener("pointerdown", (event) => {
    if (!mapData) return;
    const rect = canvas.getBoundingClientRect();
    const point = canvasToWorld(event.clientX - rect.left, event.clientY - rect.top);
    if (!point) return;
    const prefix = activeMapTool === "initial" ? "initial" : activeMapTool === "station" ? "station" : "goal";
    $(`${prefix}-x`).value = point.x.toFixed(2);
    $(`${prefix}-y`).value = point.y.toFixed(2);
    if (activeMapTool === "goal") goalDraft = readPose("goal");
    if (activeMapTool === "initial") initialPoseDraft = readPose("initial");
    addLog(`${activeMapTool.toUpperCase()} selected at ${formatNumber(point.x)}, ${formatNumber(point.y)}.`, "info");
    drawNavigation();
  });
  $("send-goal").addEventListener("click", () => {
    goalDraft = readPose("goal");
    if (!validPose(goalDraft)) return;
    sendStateCommand({ type: "goal_pose", ...goalDraft }, "/api/goal/nav2");
    setMode("AUTO");
  });
  $("apply-initial").addEventListener("click", () => {
    initialPoseDraft = readPose("initial");
    if (!validPose(initialPoseDraft)) return;
    sendStateCommand({ type: "initial_pose", ...initialPoseDraft }, "/api/localization/initialpose");
    addLog("Initial pose published.", "success");
  });
  $("cancel-goal").addEventListener("click", () => {
    sendStateCommand({ type: "cancel_goal" }, "/api/goal/cancel");
    setMode("MANUAL");
  });
  ["show-lidar", "show-path", "show-global-costmap", "show-local-costmap", "show-inflation"].forEach((id) => $(id).addEventListener("change", drawNavigation));
  $("map-select").addEventListener("change", (event) => addLog(`Map ${event.target.value} selected; restart map_server to apply.`, "info"));
  $("start-hardware").addEventListener("click", () => { sendStateCommand({ type: "start_hardware" }, "/api/hardware/start"); addLog("Hardware startup requested.", "info"); });
  $("start-localization").addEventListener("click", () => { sendStateCommand({ type: "start_localization" }, "/api/navigation/start-localization"); addLog("Localization startup requested.", "info"); });
  $("start-navigation").addEventListener("click", () => { sendStateCommand({ type: "start_navigation" }, "/api/navigation/start"); addLog("Nav2 startup requested.", "info"); });
  $("start-mapping").addEventListener("click", () => { sendStateCommand({ type: "start_mapping" }, "/api/mapping/start"); addLog("Mapping startup requested.", "info"); });
  $("save-station").addEventListener("click", () => {
    const name = $("station-name").value.trim();
    const pose = readPose("station");
    if (!name || !validPose(pose)) return;
    stations = [...stations.filter((item) => item.name !== name), { name, ...pose }];
    try { localStorage.setItem("diablo-hmi-stations", JSON.stringify(stations)); } catch { /* optional */ }
    renderPreviewStations();
    addLog(`Station ${name} saved locally.`, "success");
  });
  try { stations = JSON.parse(localStorage.getItem("diablo-hmi-stations") || "[]"); } catch { stations = []; }
  renderPreviewStations();
  window.addEventListener("resize", resizeCanvas);
  resizeCanvas();
}

function renderPreviewStations() {
  const list = $("station-list-preview");
  if (!stations.length) { list.textContent = "Belum ada station."; return; }
  list.innerHTML = "";
  stations.forEach((station) => {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = `${station.name} · ${formatNumber(station.x)}, ${formatNumber(station.y)}`;
    button.addEventListener("click", () => {
      $("goal-x").value = formatNumber(station.x);
      $("goal-y").value = formatNumber(station.y);
      $("goal-heading").value = formatNumber(station.theta * 180 / Math.PI, 1);
      goalDraft = readPreviewPose("goal");
      activeMapTool = "goal";
      drawNavigation();
    });
    list.appendChild(button);
  });
}

function readPreviewPose(prefix) {
  return {
    x: Number($(`${prefix}-x`).value),
    y: Number($(`${prefix}-y`).value),
    theta: Number($(`${prefix}-heading`).value) * Math.PI / 180,
  };
}

function setupTopics() {
  $("topic-filter").addEventListener("input", renderTopicOptions);
  $("add-topic").addEventListener("click", () => {
    const selected = $("topic-select").value;
    if (!selected || selectedTopics.includes(selected)) return;
    if (selectedTopics.length >= 4) {
      addLog("Topic echo limit is four topics.", "warn");
      return;
    }
    selectedTopics.push(selected);
    sendTopicSubscription();
  });
  $("clear-topics").addEventListener("click", clearTopics);
  $("clear-log").addEventListener("click", () => { $("event-log").innerHTML = ""; });
}

document.addEventListener("DOMContentLoaded", () => {
  setupTabs();
  setupPreviewPanels();
  updateHardwareUi({ ready: false, starting: false, message: "Preview mode — press START HARDWARE when connected to the robot", components: [
    { id: "diablo", state: "offline" },
    { id: "lidar", state: "not_configured" },
    { id: "dynamixel", state: "offline" },
  ] });
  updatePoseUi();
  updateCostmapUi();
  setupTeleop();
  setupNavigation();
  setupTopics();
  loadConfig();
  loadMaps();
  loadTopics();
  connectStateSocket();
  connectTopicSocket();
});
