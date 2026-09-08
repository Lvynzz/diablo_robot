(() => {
  "use strict";

  const $ = (id) => document.getElementById(id);
  let socket = null;
  let retryTimer = null;
  let state = {
    pose: null,
    map: null,
    previewMap: null,
    selectedMap: null,
    hardware: { ready: false, mapping_ready: false, all_ready: false, starting: false, components: [] },
    processes: {},
    joints: [],
    mapping: { active: false, state: "idle", message: "" },
  };
  const keys = new Set();
  let teleopTimer = null;
  let topicSocket = null;
  let pendingStop = null;
  let jointMetaSignature = "";
  let poseTool = null;
  let mapPointerStart = null;

  const launchDefinitions = {
    hardware: { label: "HARDWARE", start: { type: "start_hardware" }, stop: { type: "stop_hardware" }, startPath: "/api/hardware/start", stopPath: "/api/hardware/stop" },
    localization: { label: "LOCALIZATION", start: { type: "start_localization" }, stop: { type: "stop_localization" }, startPath: "/api/navigation/start-localization", stopPath: "/api/navigation/stop-localization" },
    navigation: { label: "NAVIGATION", start: { type: "start_navigation" }, stop: { type: "stop_navigation" }, startPath: "/api/navigation/start", stopPath: "/api/navigation/stop" },
    mapping: { label: "MAPPING", start: { type: "start_mapping" }, stop: { type: "stop_mapping" }, startPath: "/api/mapping/start", stopPath: "/api/mapping/stop" },
  };

  function log(message, kind = "info") {
    const row = document.createElement("div");
    row.className = `log-row ${kind}`;
    row.innerHTML = `<time>${new Date().toLocaleTimeString([], { hour12: false })}</time><span></span>`;
    row.querySelector("span").textContent = message;
    $("event-log").prepend(row);
    while ($("event-log").children.length > 8) $("event-log").lastElementChild.remove();
  }

  function command(payload, fallbackPath) {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(payload));
      return Promise.resolve(true);
    }
    if (!fallbackPath) return Promise.resolve(false);
    return fetch(fallbackPath, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload) })
      .then((response) => response.ok)
      .catch(() => false);
  }

  function processActive(name) {
    if (name === "mapping" && state.mapping?.active) return true;
    return Boolean(state.processes?.[name]?.active);
  }

  function componentActive(name) {
    if (name === "hardware") {
      const hardware = state.hardware || {};
      return Boolean(hardware.ready || hardware.starting || hardware.all_ready);
    }
    return processActive(name);
  }

  function componentStatus(name) {
    if (name === "hardware") {
      const hardware = state.hardware || {};
      if (hardware.all_ready) return "ALL READY";
      if (hardware.ready) return "READY";
      if (hardware.starting) return "STARTING";
      return "OFFLINE";
    }
    if (name === "mapping" && state.mapping?.active) return "RUNNING";
    return String(state.processes?.[name]?.state || "idle").replaceAll("_", " ").toUpperCase();
  }

  function closeConfirmation() {
    pendingStop = null;
    $("confirm-modal").hidden = true;
  }

  function openConfirmation(name) {
    const definition = launchDefinitions[name];
    if (!definition) return;
    pendingStop = name;
    const dependencies = name === "hardware"
      ? ["mapping", "navigation", "localization"].filter(componentActive).map((item) => launchDefinitions[item].label)
      : [];
    $("confirm-title").textContent = `OFF ${definition.label}?`;
    $("confirm-message").textContent = dependencies.length
      ? `Hardware OFF akan menghentikan ${dependencies.join(", ")} lebih dulu, lalu mengirim command stop.`
      : `Tekan IYA untuk menghentikan ${definition.label}. Tekan TIDAK untuk membatalkan.`;
    $("confirm-modal").hidden = false;
  }

  function requestLaunch(name) {
    const definition = launchDefinitions[name];
    if (!definition) return;
    if (componentActive(name)) {
      openConfirmation(name);
      return;
    }
    command(definition.start, definition.startPath).then((accepted) => {
      log(`${definition.label} startup requested.`, accepted ? "success" : "error");
    });
  }

  function stopConfirmedComponent() {
    const name = pendingStop;
    const definition = launchDefinitions[name];
    closeConfirmation();
    if (!definition) return;
    stopTeleop();
    command(definition.stop, definition.stopPath).then((accepted) => {
      log(`${definition.label} stop requested.`, accepted ? "warn" : "error");
    });
  }

  function connect() {
    if (socket && socket.readyState <= WebSocket.OPEN) return;
    const scheme = window.location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(`${scheme}://${window.location.host}/ws`);
    socket.onopen = () => { $("connection-dot").className = "online"; $("connection-text").textContent = "CONNECTED"; log("Connected to FastAPI / ROS bridge.", "success"); };
    socket.onmessage = (event) => {
      let packet;
      try { packet = JSON.parse(event.data); } catch { return; }
      if (packet.type === "state") { state = { ...state, ...packet }; render(); return; }
      if (packet.type === "error") { log(packet.detail || "WebSocket error", "error"); return; }
      if (packet.type === "save_map_ack") { log(packet.message || (packet.saved ? "Map saved." : "Map was not saved."), packet.saved ? "success" : "warn"); return; }
      if (typeof packet.type === "string" && packet.type.endsWith("_ack")) log(packet.message || "Command acknowledged.", packet.requested === false || packet.saved === false ? "warn" : "success");
    };
    socket.onerror = () => { $("connection-dot").className = "error"; };
    socket.onclose = () => {
      $("connection-dot").className = "error";
      $("connection-text").textContent = "DISCONNECTED";
      socket = null;
      if (retryTimer === null) retryTimer = window.setTimeout(() => { retryTimer = null; connect(); }, 2000);
    };
  }

  function render() {
    const hardware = state.hardware || {};
    const mapping = state.mapping || {};
    const allReady = Boolean(hardware.all_ready);
    const mappingReady = Boolean(hardware.mapping_ready);
    const active = Boolean(mapping.active || processActive("mapping"));
    $("mapping-dot").className = active ? "online" : "";
    $("mapping-text").textContent = active ? "MAPPING ACTIVE" : "MAPPING IDLE";
    $("mapping-state").textContent = String(mapping.state || "idle").toUpperCase();
    $("mapping-status").textContent = String(mapping.state || "idle").toUpperCase();
    $("mapping-message").textContent = mapping.message || "Mapping belum dimulai.";
    $("hardware-status").textContent = mappingReady ? "MAPPING READY" : hardware.ready ? "TELEOP READY" : hardware.starting ? "STARTING" : "LOCKED";
    $("hardware-status").className = mappingReady || hardware.ready ? "ready" : hardware.starting ? "starting" : "";
    $("hardware-message").textContent = hardware.message || "Press ON HARDWARE to start hardware.";
    const components = Object.fromEntries((hardware.components || []).map((item) => [item.id, item]));
    [["diablo", "component-diablo", "DIABLO"], ["lidar", "component-lidar", "LIDAR"], ["dynamixel", "component-dynamixel", "DYNAMIXEL ARM"]].forEach(([id, element, label]) => {
      const item = components[id];
      $(element).textContent = `● ${label} · ${item ? item.state.replace("_", " ").toUpperCase() : "OFFLINE"}`;
      $(element).className = item && item.state === "ready" ? "ready" : item && item.state === "error" ? "error" : "";
    });
    const hardwareActive = componentActive("hardware");
    const hardwareButton = $("start-hardware");
    hardwareButton.disabled = false;
    hardwareButton.classList.toggle("is-active", hardwareActive);
    hardwareButton.textContent = hardwareActive ? `OFF HARDWARE · ${componentStatus("hardware")}` : "ON HARDWARE";
    ["localization", "navigation", "mapping"].forEach((name) => {
      const button = $(`launch-${name}`);
      if (!button) return;
      const running = componentActive(name);
      const definition = launchDefinitions[name];
      button.classList.toggle("is-active", running);
      button.querySelector("span").textContent = running ? `OFF ${definition.label}` : `ON ${definition.label}`;
      button.querySelector("b").textContent = componentStatus(name);
      const configured = state.processes?.[name]?.state !== "not_configured";
      button.disabled = !running && (!configured || (name === "mapping" && !mappingReady));
    });
    $("start-mapping").disabled = !mappingReady || active;
    $("stop-mapping").disabled = !active;
    $("save-map").disabled = !active;
    const teleopReady = Boolean(hardware.ready);
    $("teleop-lock").textContent = teleopReady ? "TELEOP UNLOCKED · MAPPING OPTIONAL" : "START HARDWARE TO UNLOCK TELEOP";
    $("teleop-lock").className = teleopReady ? "unlocked" : "";
    const pose = state.pose;
    $("pose-x").textContent = pose ? Number(pose.x).toFixed(3) : "—";
    $("pose-y").textContent = pose ? Number(pose.y).toFixed(3) : "—";
    $("pose-theta").textContent = pose ? (Number(pose.theta) * 180 / Math.PI).toFixed(1) : "—";
    const mapPose = pose && ["amcl", "map", "amcl_initial"].includes(String(pose.source || ""));
    ["pose-x-source", "pose-y-source"].forEach((id) => { const element = $(id); if (element) element.textContent = mapPose ? "METERS · MAP / AMCL" : "METERS · ODOM"; });
    const thetaSource = $("pose-theta-source");
    if (thetaSource) thetaSource.textContent = mapPose ? "DEGREES · MAP / AMCL" : "DEGREES · ODOM";
    const grid = state.selectedMap || state.previewMap || state.map;
    $("map-empty").style.display = grid ? "none" : "flex";
    $("map-meta").textContent = grid ? `${grid.width} × ${grid.height} · ${Number(grid.resolution).toFixed(3)} m · ${state.selectedMap ? "SELECTED MAP" : state.previewMap ? "PREVIEW" : grid.frame_id || "map"}` : "Menunggu /map";
    const sourceStatus = $("map-source-status");
    const lidarLayer = $("layer-lidar");
    if (sourceStatus) sourceStatus.textContent = `LIDAR: ${lidarLayer?.checked ? (state.scan ? "LIVE" : "WAITING") : "OFF"} · ${state.selectedMap ? `SELECTED: ${state.selectedMap.name || "MAP"}` : state.previewMap ? `PREVIEW: ${state.previewMap.name || "MAP"}` : "/map → OccupancyGrid"}`;
    $("map-select-apply").disabled = !state.previewMap;
    drawMap();
    renderJoints();
  }

  function renderJoints() {
    const container = $("joint-sliders");
    if (!container) return;
    const joints = Array.isArray(state.joints) ? state.joints : [];
    const signature = joints.map((joint) => `${joint.id}:${joint.name}:${joint.min}:${joint.max}`).join("|");
    if (signature !== jointMetaSignature) {
      jointMetaSignature = signature;
      container.innerHTML = "";
      if (!joints.length) {
        container.innerHTML = '<p class="joint-empty">Menunggu /joint_states. Slider tidak mengirim command saat halaman dibuka.</p>';
      } else {
        joints.forEach((joint) => {
          const row = document.createElement("label");
          row.className = "joint-slider-row";
          const heading = document.createElement("span");
          heading.className = "joint-slider-heading";
          heading.innerHTML = `<b>ID ${joint.id}</b><strong></strong><em></em>`;
          heading.querySelector("strong").textContent = joint.label;
          const input = document.createElement("input");
          input.type = "range";
          input.min = joint.min;
          input.max = joint.max;
          input.step = "0.01";
          input.value = String((Number(joint.min) + Number(joint.max)) / 2);
          input.dataset.lastSent = "";
          input.dataset.jointId = String(joint.id);
          const detail = document.createElement("small");
          detail.textContent = `${joint.name} · WAITING FOR FEEDBACK`;
          const updateValue = () => { heading.querySelector("em").textContent = `${Number(input.value).toFixed(3)} rad`; };
          const commit = () => {
            if (input.disabled || input.dataset.lastSent === input.value) return;
            input.dataset.lastSent = input.value;
            const id = Number(input.dataset.jointId);
            command({ type: "joint_position", id, position: Number(input.value) }, `/api/joints/${id}/position`).then((accepted) => {
              log(accepted ? `Dynamixel ID ${id} target sent.` : `Dynamixel ID ${id} command failed.`, accepted ? "success" : "error");
            });
          };
          input.addEventListener("input", updateValue);
          input.addEventListener("pointerup", commit);
          input.addEventListener("touchend", commit);
          input.addEventListener("keyup", commit);
          row.append(heading, input, detail);
          container.appendChild(row);
          updateValue();
        });
      }
    }
    const hardwareReady = Boolean(state.hardware?.all_ready);
    joints.forEach((joint) => {
      const input = container.querySelector(`input[data-joint-id="${joint.id}"]`);
      if (!input) return;
      const available = Boolean(joint.available);
      input.disabled = !hardwareReady || !available;
      const detail = input.parentElement?.querySelector("small");
      const optionalHand = [4, 5, 9, 10].includes(Number(joint.id));
      if (detail) detail.textContent = `${joint.name} · ${available ? "READY" : optionalHand && hardwareReady ? "OPTIONAL HAND SKIPPED" : "WAITING FOR FEEDBACK"}`;
    });
    $("joint-status").textContent = hardwareReady ? "JOINTS READY" : "START HARDWARE + ARM FEEDBACK";
  }

  function drawMap() {
    const canvas = $("map-canvas");
    const grid = state.selectedMap || state.previewMap || state.map;
    if (!canvas || !grid) return;
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.floor(rect.width * ratio));
    canvas.height = Math.max(1, Math.floor(rect.height * ratio));
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.fillStyle = "#f8fbfd";
    ctx.fillRect(0, 0, rect.width, rect.height);
    const cell = Math.min((rect.width - 28) / grid.width, (rect.height - 28) / grid.height);
    const ox = (rect.width - grid.width * cell) / 2;
    const oy = (rect.height - grid.height * cell) / 2;
    const sample = Math.max(1, Math.ceil(Math.sqrt((grid.width * grid.height) / 180000)));
    for (let row = 0; row < grid.height; row += sample) {
      for (let col = 0; col < grid.width; col += sample) {
        const value = grid.data[row * grid.width + col] ?? -1;
        ctx.fillStyle = value < 0 ? "#dce6ec" : value >= 65 ? "#405765" : "#f8fbfd";
        ctx.fillRect(ox + col * cell, oy + (grid.height - row - sample) * cell, Math.ceil(cell * sample + .3), Math.ceil(cell * sample + .3));
      }
    }
    const origin = grid.origin || { x: 0, y: 0, yaw: 0 };
    const toCanvas = (x, y) => {
      const dx = x - origin.x, dy = y - origin.y, angle = origin.yaw || 0;
      const gx = (Math.cos(angle) * dx + Math.sin(angle) * dy) / grid.resolution;
      const gy = (-Math.sin(angle) * dx + Math.cos(angle) * dy) / grid.resolution;
      return [ox + gx * cell, oy + (grid.height - gy) * cell];
    };
    const checked = (id, fallback) => {
      const element = $(id);
      return element ? element.checked : fallback;
    };
    const drawCostmap = (costmap, layer) => {
      if (!costmap) return;
      const overlaySample = Math.max(1, Math.ceil(Math.sqrt((costmap.width * costmap.height) / 65000)));
      const costOrigin = costmap.origin || { x: 0, y: 0, yaw: 0 };
      const costAngle = costOrigin.yaw || 0;
      for (let row = 0; row < costmap.height; row += overlaySample) {
        for (let col = 0; col < costmap.width; col += overlaySample) {
          const value = costmap.data[row * costmap.width + col] ?? -1;
          if (value < 1) continue;
          const worldX = costOrigin.x + Math.cos(costAngle) * col * costmap.resolution - Math.sin(costAngle) * row * costmap.resolution;
          const worldY = costOrigin.y + Math.sin(costAngle) * col * costmap.resolution + Math.cos(costAngle) * row * costmap.resolution;
          const [x, y] = toCanvas(worldX, worldY);
          const lethal = value >= 90;
          ctx.fillStyle = layer === "global"
            ? `rgba(47,120,174,${lethal ? .55 : .23})`
            : `rgba(197,83,0,${lethal ? .55 : .23})`;
          const sizeValue = Math.max(1, cell * costmap.resolution / grid.resolution * overlaySample + .5);
          ctx.fillRect(x, y - sizeValue, sizeValue, sizeValue);
        }
      }
    };
    if (checked("layer-global-costmap", true)) drawCostmap(state.global_costmap, "global");
    if (checked("layer-local-costmap", true)) drawCostmap(state.local_costmap, "local");
    if (checked("layer-lidar", false) && state.scan && state.pose) {
      ctx.fillStyle = "rgba(36,126,164,.62)";
      const sensorX = Number(state.scan.sensor_x) || 0;
      const sensorY = Number(state.scan.sensor_y) || 0;
      const sensorTheta = Number(state.scan.sensor_theta) || 0;
      const laserX = state.pose.x + Math.cos(state.pose.theta) * sensorX - Math.sin(state.pose.theta) * sensorY;
      const laserY = state.pose.y + Math.sin(state.pose.theta) * sensorX + Math.cos(state.pose.theta) * sensorY;
      state.scan.ranges.forEach((range, index) => {
        if (range === null || range < state.scan.range_min || range > state.scan.range_max) return;
        const angle = state.pose.theta + sensorTheta + state.scan.angle_min + index * state.scan.angle_increment;
        const [x, y] = toCanvas(laserX + range * Math.cos(angle), laserY + range * Math.sin(angle));
        ctx.fillRect(x - 1, y - 1, 2.5, 2.5);
      });
    }
    if (checked("layer-robot", true) && state.pose) {
      const [x, y] = toCanvas(state.pose.x, state.pose.y);
      ctx.save(); ctx.translate(x, y); ctx.rotate(-state.pose.theta);
      ctx.fillStyle = "#4f925c"; ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(13, 0); ctx.lineTo(-9, -7); ctx.lineTo(-6, 0); ctx.lineTo(-9, 7); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
    }
    const marker = (pose, color) => {
      if (!pose) return;
      const [x, y] = toCanvas(pose.x, pose.y);
      ctx.save(); ctx.translate(x, y); ctx.rotate(-pose.theta); ctx.fillStyle = color; ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(0, 0, 8, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(12, 0); ctx.lineTo(-7, -5); ctx.lineTo(-5, 0); ctx.lineTo(-7, 5); ctx.closePath(); ctx.fill(); ctx.restore();
    };
    marker(state.initialPose, "#2c83a9"); marker(state.goalPose, "#c55300");
  }

  function isUnlocked() { return Boolean(state.hardware?.ready); }
  function sendMotion() {
    if (!isUnlocked()) return;
    const forwardSpeed = Number($("forward-speed").value), turnSpeed = Number($("turn-speed").value);
    const forward = (keys.has("w") ? forwardSpeed : 0) - (keys.has("s") ? forwardSpeed : 0);
    const left = (keys.has("a") ? turnSpeed : 0) - (keys.has("d") ? turnSpeed : 0);
    if (!forward && !left) return command({ type: "stop" }, "/api/control/stop");
    return command({ type: "manual", forward, left, roll: 0, up: 1, pitch: 0 }, "/api/teleop");
  }
  function stopTeleop() {
    keys.clear();
    document.querySelectorAll(".keypad button[data-key]").forEach((button) => button.classList.remove("active"));
    if (teleopTimer !== null) window.clearInterval(teleopTimer);
    teleopTimer = null;
    command({ type: "stop" }, "/api/control/stop");
  }
  function pressKey(key) {
    if (!isUnlocked()) return;
    keys.add(key);
    document.querySelector(`.keypad button[data-key="${key}"]`)?.classList.add("active");
    sendMotion();
    if (teleopTimer === null) teleopTimer = window.setInterval(sendMotion, 100);
  }

  function poseValues(kind) {
    const prefix = kind === "initial" ? "initial" : "goal";
    return {
      x: Number($(`${prefix}-x`).value),
      y: Number($(`${prefix}-y`).value),
      theta: Number($(`${prefix}-theta`).value) * Math.PI / 180,
    };
  }

  function setPoseValues(kind, pose) {
    const prefix = kind === "initial" ? "initial" : "goal";
    $(`${prefix}-x`).value = Number(pose.x).toFixed(2);
    $(`${prefix}-y`).value = Number(pose.y).toFixed(2);
    $(`${prefix}-theta`).value = (Number(pose.theta) * 180 / Math.PI).toFixed(1);
    state[`${kind}Pose`] = { x: Number(pose.x), y: Number(pose.y), theta: Number(pose.theta) };
    render();
  }

  function mapPoint(event) {
    const grid = state.selectedMap || state.previewMap || state.map;
    const canvas = $("map-canvas");
    if (!grid || !canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const cell = Math.min((rect.width - 28) / Math.max(1, grid.width), (rect.height - 28) / Math.max(1, grid.height));
    const ox = (rect.width - grid.width * cell) / 2;
    const oy = (rect.height - grid.height * cell) / 2;
    const gx = (event.clientX - rect.left - ox) / cell;
    const gy = grid.height - (event.clientY - rect.top - oy) / cell;
    const origin = grid.origin || { x: 0, y: 0, yaw: 0 };
    const localX = gx * grid.resolution, localY = gy * grid.resolution, angle = origin.yaw || 0;
    return { x: origin.x + Math.cos(angle) * localX - Math.sin(angle) * localY, y: origin.y + Math.sin(angle) * localX + Math.cos(angle) * localY };
  }

  function initMapTools() {
    const select = $("map-select");
    fetch("/api/maps").then((response) => response.ok ? response.json() : Promise.reject(new Error("map catalog unavailable"))).then((items) => {
      (Array.isArray(items) ? items : []).forEach((item) => { const name = typeof item === "string" ? item : item.name; if (!name) return; const option = document.createElement("option"); option.value = name; option.textContent = name; select.appendChild(option); });
    }).catch(() => {});
    select.addEventListener("change", () => {
      const name = select.value;
      if (!name) { state.previewMap = null; render(); return; }
      fetch(`/api/maps/${encodeURIComponent(name)}`).then((response) => response.ok ? response.json() : Promise.reject(new Error("preview unavailable"))).then((map) => { state.previewMap = map; state.selectedMap = null; render(); log(`Preview map ${name} dimuat. Tekan SELECT MAP untuk menerapkan.`, "info"); }).catch((error) => log(`Preview map gagal: ${error.message}`, "warn"));
    });
    $("map-select-apply").addEventListener("click", () => { if (!state.previewMap) return; state.selectedMap = state.previewMap; render(); log(`Map ${state.selectedMap.name || select.value} dipilih untuk LIVE /MAP.`, "success"); });
    [["initial", "initial-pick"], ["goal", "goal-pick"]].forEach(([kind, id]) => $(id).addEventListener("click", () => { poseTool = poseTool === kind ? null : kind; $("initial-tool").classList.toggle("active", poseTool === "initial"); $("goal-tool").classList.toggle("active", poseTool === "goal"); $("map-canvas").classList.toggle("map-interactive", Boolean(poseTool)); }));
    $("map-canvas").addEventListener("pointerdown", (event) => { if (!poseTool) return; event.currentTarget.setPointerCapture(event.pointerId); mapPointerStart = mapPoint(event); if (mapPointerStart) setPoseValues(poseTool, { ...mapPointerStart, theta: 0 }); });
    $("map-canvas").addEventListener("pointermove", (event) => { if (!poseTool || !mapPointerStart) return; const point = mapPoint(event); if (!point) return; const distance = Math.hypot(point.x - mapPointerStart.x, point.y - mapPointerStart.y); const theta = distance > 0.03 ? Math.atan2(point.y - mapPointerStart.y, point.x - mapPointerStart.x) : 0; setPoseValues(poseTool, { x: mapPointerStart.x, y: mapPointerStart.y, theta }); });
    $("map-canvas").addEventListener("pointerup", () => { mapPointerStart = null; });
    $("map-canvas").addEventListener("pointercancel", () => { mapPointerStart = null; });
    ["layer-robot", "layer-lidar", "layer-local-costmap", "layer-global-costmap"].forEach((id) => $(id)?.addEventListener("change", render));
    $("initial-send").addEventListener("click", () => { const pose = poseValues("initial"); command({ type: "initial_pose", x: pose.x, y: pose.y, theta: pose.theta }, "/api/localization/initialpose").then((accepted) => log(accepted ? "Initial pose dikirim ke AMCL." : "Initial pose gagal dikirim.", accepted ? "success" : "warn")); });
    $("goal-send").addEventListener("click", () => { const pose = poseValues("goal"); command({ type: "goal_pose", x: pose.x, y: pose.y, theta: pose.theta }, "/api/goal/nav2").then((accepted) => log(accepted ? "Goal pose dikirim ke Nav2." : "Goal pose gagal dikirim.", accepted ? "success" : "warn")); });
    ["initial", "goal"].forEach((kind) => ["x", "y", "theta"].forEach((field) => $(`${kind}-${field}`).addEventListener("input", () => { const pose = poseValues(kind); if ([pose.x, pose.y, pose.theta].every(Number.isFinite)) { state[`${kind}Pose`] = { ...pose }; drawMap(); } })));
  }

  function initTeleop() {
    window.addEventListener("keydown", (event) => {
      if (["INPUT", "TEXTAREA"].includes(document.activeElement?.tagName)) return;
      const key = event.key.toLowerCase();
      if (!["w", "a", "s", "d"].includes(key)) return;
      event.preventDefault(); pressKey(key);
    });
    window.addEventListener("keyup", (event) => {
      const key = event.key.toLowerCase();
      if (!["w", "a", "s", "d"].includes(key)) return;
      keys.delete(key); document.querySelector(`.keypad button[data-key="${key}"]`)?.classList.remove("active");
      if (!keys.size) stopTeleop();
    });
    window.addEventListener("blur", stopTeleop);
    document.querySelectorAll(".keypad button[data-key]").forEach((button) => {
      button.addEventListener("pointerdown", (event) => { event.preventDefault(); pressKey(button.dataset.key); });
      button.addEventListener("pointerup", stopTeleop); button.addEventListener("pointerleave", stopTeleop); button.addEventListener("pointercancel", stopTeleop);
    });
    $("teleop-stop").addEventListener("click", stopTeleop);
    $("forward-speed").addEventListener("input", () => $("forward-value").textContent = Number($("forward-speed").value).toFixed(2));
    $("turn-speed").addEventListener("input", () => $("turn-value").textContent = Number($("turn-speed").value).toFixed(2));
  }

  function initControls() {
    $("stop-button").addEventListener("click", () => { stopTeleop(); command({ type: "stop" }, "/api/control/stop"); log("STOP command sent.", "warn"); });
    $("start-hardware").addEventListener("click", () => requestLaunch("hardware"));
    $("reset-position").addEventListener("click", () => { command({ type: "reset_position" }, "/api/odom/reset-position").then((accepted) => log("Reset X/Y position requested.", accepted ? "success" : "warn")); });
    $("reset-orientation").addEventListener("click", () => { command({ type: "reset_orientation" }, "/api/odom/reset-orientation").then((accepted) => log("Reset heading requested.", accepted ? "success" : "warn")); });
    $("launch-localization").addEventListener("click", () => requestLaunch("localization"));
    $("launch-navigation").addEventListener("click", () => requestLaunch("navigation"));
    $("launch-mapping").addEventListener("click", () => requestLaunch("mapping"));
    $("start-mapping").addEventListener("click", () => requestLaunch("mapping"));
    $("stop-mapping").addEventListener("click", () => { stopTeleop(); command({ type: "stop_mapping" }, "/api/mapping/stop"); log("Mapping stop requested.", "warn"); });
    $("save-map").addEventListener("click", () => { const name = $("map-name").value.trim(); if (!name) { log("Masukkan nama map terlebih dahulu.", "warn"); return; } command({ type: "save_map", name }, "/api/mapping/save"); });
    $("confirm-no").addEventListener("click", closeConfirmation);
    $("confirm-yes").addEventListener("click", stopConfirmedComponent);
    $("confirm-modal").addEventListener("click", (event) => { if (event.target === $("confirm-modal")) closeConfirmation(); });
    window.addEventListener("keydown", (event) => { if (event.key === "Escape" && !$("confirm-modal").hidden) closeConfirmation(); });
    window.addEventListener("resize", drawMap);
    initMapTools();
  }

  function initTabs() {
    document.querySelectorAll(".tab").forEach((button) => button.addEventListener("click", () => {
      const tab = button.dataset.tab;
      document.querySelectorAll(".tab").forEach((item) => item.classList.toggle("active", item === button));
      document.querySelectorAll(".tab-panel").forEach((panel) => panel.classList.toggle("active", panel.id === `tab-${tab}`));
      if (tab === "topics") loadTopics();
    }));
  }

  function loadTopics() {
    fetch("/api/topics").then((response) => response.json()).then((payload) => {
      const select = $("topic-select"); select.innerHTML = "";
      (payload.topics || []).forEach((item) => { const option = document.createElement("option"); option.value = item.name; option.textContent = `${item.name} · ${(item.types || []).join(", ")}`; select.appendChild(option); });
    }).catch(() => {});
  }

  function initTopics() {
    $("topic-add").addEventListener("click", () => {
      const topic = $("topic-select").value; if (!topic) return;
      if (!topicSocket || topicSocket.readyState !== WebSocket.OPEN) {
        const scheme = window.location.protocol === "https:" ? "wss" : "ws";
        topicSocket = new WebSocket(`${scheme}://${window.location.host}/ws/topics`);
        topicSocket.onopen = () => topicSocket.send(JSON.stringify({ type: "subscribe", topics: [topic] }));
        topicSocket.onmessage = (event) => { const packet = JSON.parse(event.data); if (packet.type === "topic") $("topic-cards").innerHTML = `<pre></pre>`; if (packet.data) $("topic-cards").querySelector("pre").textContent = JSON.stringify(packet.data, null, 2); };
      } else topicSocket.send(JSON.stringify({ type: "subscribe", topics: [topic] }));
    });
    $("topic-clear").addEventListener("click", () => { topicSocket?.send(JSON.stringify({ type: "clear" })); $("topic-cards").textContent = "Topic echo stopped."; });
  }

  function loadSettings() {
    fetch("/api/config").then((response) => response.json()).then((config) => {
      $("settings-card").innerHTML = `<p><b>MANUAL</b> <code>${config.manual_cmd_topic}</code></p><p><b>ODOMETRY</b> <code>${config.odom_topic}</code></p><p><b>SCAN</b> <code>${config.scan_topic}</code></p><p><b>MAP SAVE DIR</b> <code>${config.maps_dir}</code></p>`;
    }).catch(() => { $("settings-card").textContent = "Configuration unavailable."; });
  }

  initTabs(); initControls(); initTeleop(); initTopics(); loadSettings(); connect();
  render();
})();
