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
    navigation_readiness: null,
    command_pipeline: {},
    map_status: null,
    footprint: null,
    path: null,
  };
  const keys = new Set();
  let teleopTimer = null;
  let topicSocket = null;
  let pendingStop = null;
  let jointMetaSignature = "";
  let poseTool = null;
  let mapPointerStart = null;
  let mapPanStart = null;
  const MAP_MIN_ZOOM = 0.5;
  const MAP_MAX_ZOOM = 8;
  // Keep the camera independent from the ROS source.  Navigation replaces a
  // selected/preview OccupancyGrid with the live /map; refitting on every
  // packet made the map appear to jump or zoom when Nav2 was enabled.
  const mapView = {
    zoom: 1,
    panX: 0,
    panY: 0,
    geometrySignature: "",
  };
  // Cache the expensive occupancy/costmap layer.  Pose, LiDAR, footprint,
  // local-window and plan overlays are drawn on top of this image each
  // update, so a 20 Hz pose stream no longer walks every map cell.
  const mapRenderCache = {
    grid: null,
    globalCostmap: null,
    localCostmap: null,
    navigationActive: false,
    showGlobal: true,
    showLocal: true,
    zoom: 1,
    panX: 0,
    panY: 0,
    width: 0,
    height: 0,
    logicalWidth: 0,
    logicalHeight: 0,
    ratio: 1,
    image: null,
  };

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

  function displayRobotPose() {
    // Do not keep showing a stale map-frame pose after AMCL/Nav2 is stopped;
    // the filtered odometry stream is the live source in manual/mapping mode.
    const mapPoseOwned = processActive("navigation") || processActive("localization");
    return mapPoseOwned ? (state.pose || state.wheel_pose) : (state.wheel_pose || state.pose);
  }

  function componentActive(name) {
    if (name === "hardware") {
      const hardware = state.hardware || {};
      const components = Array.isArray(hardware.components) ? hardware.components : [];
      const componentProcessActive = components.some((item) => !["offline", "not_configured"].includes(String(item.state || "")));
      // Keep the OFF action available even when one sensor timed out.  Any
      // ready/waiting component or dependent launch means there is something
      // that must receive the stop command.
      return Boolean(
        hardware.ready || hardware.starting || hardware.all_ready ||
        componentProcessActive || processActive("mapping") ||
        processActive("navigation") || processActive("localization")
      );
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
    if (name === "localization" && state.processes?.localization?.owner === "navigation") {
      return "AMCL VIA NAV2";
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
      const embeddedLocalization = name === "localization" && state.processes?.localization?.owner === "navigation";
      button.classList.toggle("is-active", running);
      button.querySelector("span").textContent = embeddedLocalization ? "AMCL VIA NAV2" : running ? `OFF ${definition.label}` : `ON ${definition.label}`;
      button.querySelector("b").textContent = componentStatus(name);
      const configured = state.processes?.[name]?.state !== "not_configured";
      button.disabled = embeddedLocalization || (!running && (!configured || (name === "mapping" && !mappingReady)));
    });
    $("start-mapping").disabled = !mappingReady || active;
    $("stop-mapping").disabled = !active;
    $("save-map").disabled = !active;
    const teleopReady = Boolean(hardware.ready);
    $("teleop-lock").textContent = teleopReady ? "TELEOP UNLOCKED · MAPPING OPTIONAL" : "START HARDWARE TO UNLOCK TELEOP";
    $("teleop-lock").className = teleopReady ? "unlocked" : "";
    // Match the AMR navigation renderer: AMCL/map pose has priority, while
    // wheel odometry keeps the robot visible before AMCL publishes.
    const pose = displayRobotPose();
    $("pose-x").textContent = pose ? Number(pose.x).toFixed(3) : "—";
    $("pose-y").textContent = pose ? Number(pose.y).toFixed(3) : "—";
    $("pose-theta").textContent = pose ? (Number(pose.theta) * 180 / Math.PI).toFixed(1) : "—";
    const mapPose = pose && ["amcl", "map", "amcl_initial"].includes(String(pose.source || ""));
    ["pose-x-source", "pose-y-source"].forEach((id) => { const element = $(id); if (element) element.textContent = mapPose ? "METERS · MAP / AMCL" : "METERS · ODOM"; });
    const thetaSource = $("pose-theta-source");
    if (thetaSource) thetaSource.textContent = mapPose ? "DEGREES · MAP / AMCL" : "DEGREES · ODOM";
    // While Nav2 is running, draw its live /map. Costmaps are generated from
    // that map; a stale browser preview would make the overlay look misaligned.
    const navigationActive = Boolean(state.processes?.navigation?.active);
    const grid = currentMapGrid();
    const gridReady = validMapGrid(grid);
    $("map-empty").style.display = gridReady ? "none" : "flex";
    $("map-meta").textContent = gridReady ? `${grid.width} × ${grid.height} · ${Number(grid.resolution).toFixed(3)} m · ${state.selectedMap ? "SELECTED MAP" : state.previewMap ? "PREVIEW" : grid.frame_id || "map"}` : "Menunggu /map";
    const sourceStatus = $("map-source-status");
    const lidarLayer = $("layer-lidar");
    const usingLiveMap = navigationActive && grid === state.map;
    const selectedMapName = state.map_status?.selected_map || state.selectedMap?.name || state.previewMap?.name || "";
    if (sourceStatus) sourceStatus.textContent = `${usingLiveMap ? "LIVE NAV2 /MAP" : navigationActive ? "SELECTED MAP · WAITING /MAP" : "/map → OccupancyGrid"} · GLOBAL: ${state.global_costmap ? "LIVE" : "WAITING"} · LOCAL: ${state.local_costmap ? "LIVE" : "WAITING"} · PLAN: ${navigationActive && state.path?.poses?.length ? "LIVE" : "WAITING"} · LIDAR: ${lidarLayer?.checked ? (state.scan ? "LIVE" : "WAITING") : "OFF"} · ${selectedMapName ? `SELECTED: ${selectedMapName}` : "NO MAP SELECTED"}`;
    $("map-select-apply").disabled = !state.previewMap;
    drawMap();
    renderJoints();
    renderNavigationReadiness();
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

  function renderNavigationReadiness() {
    const panel = $("nav-readiness");
    const stateLabel = $("nav-readiness-state");
    const detail = $("nav-readiness-detail");
    const pipelineLabel = $("nav-readiness-pipeline");
    const normalButton = $("goal-send");
    const forceButton = $("goal-force");
    if (!panel || !stateLabel || !detail) return;
    const readiness = state.navigation_readiness;
    if (!readiness) {
      panel.dataset.ready = "false";
      stateLabel.textContent = "NAV2 CHECK: MENUNGGU";
      detail.textContent = "Menunggu status Navigation.";
      if (pipelineLabel) pipelineLabel.textContent = "PIPELINE: —";
      if (normalButton) normalButton.disabled = true;
      if (forceButton) { forceButton.hidden = true; forceButton.disabled = true; }
      return;
    }
    const ready = Boolean(readiness.ready);
    const blockers = Array.isArray(readiness.blockers) ? readiness.blockers : [];
    panel.dataset.ready = ready ? "true" : "false";
    stateLabel.textContent = ready ? "NAV2 CHECK: READY" : "NAV2 CHECK: BLOCKED";
    detail.textContent = ready
      ? "Hardware, scan, odom, AMCL/TF, costmap, Collision Monitor dan action server aktif."
      : (blockers.join(" · ") || readiness.message || "Prerequisite belum lengkap.");
    if (pipelineLabel) {
      const pipeline = readiness.pipeline || {};
      const names = ["cmd_vel_nav", "cmd_vel_smoothed", "motion_cmd_nav", "motion_cmd_mux"];
      pipelineLabel.textContent = `PIPELINE: ${names.map((name) => {
        const item = pipeline[name] || {};
        return `${name.replace("motion_cmd_", "MOTION ").replace("cmd_vel_", "VEL ")} ${item.recent ? (item.nonzero ? "LIVE" : "ZERO") : "—"}`;
      }).join(" → ")}`;
    }
    if (normalButton) normalButton.disabled = !ready;
    if (forceButton) {
      const forceAllowed = Boolean(readiness.force_allowed);
      forceButton.hidden = ready;
      forceButton.disabled = !forceAllowed;
      forceButton.title = forceAllowed
        ? "Kirim goal tanpa gate readiness (mode developer)."
        : "Start Navigation sampai action server tersedia.";
    }
  }

  function currentMapGrid() {
    const navigationActive = Boolean(state.processes?.navigation?.active);
    const selected = state.selectedMap || state.previewMap;
    if (!navigationActive) return selected || state.map;
    if (!validMapGrid(state.map)) return selected || state.map;
    // A previous Navigation process may have left a valid but different
    // /map (commonly the bundled 20x20 empty map) in the browser cache.  Keep
    // the selected map visible until the newly launched map_server publishes
    // the same geometry, instead of flashing a blank/tiny canvas.
    if (selected && validMapGrid(selected)
        && mapGeometrySignature(state.map) !== mapGeometrySignature(selected)) {
      return selected;
    }
    return state.map;
  }

  function validMapGrid(grid) {
    if (!grid) return false;
    const width = Number(grid.width);
    const height = Number(grid.height);
    const resolution = Number(grid.resolution);
    return Number.isFinite(width) && width > 0
      && Number.isFinite(height) && height > 0
      && Number.isFinite(resolution) && resolution > 0;
  }

  function mapGeometrySignature(grid) {
    const origin = grid.origin || {};
    return [
      Number(grid.width),
      Number(grid.height),
      Number(grid.resolution),
      Number(origin.x || 0).toFixed(5),
      Number(origin.y || 0).toFixed(5),
      Number(origin.yaw || 0).toFixed(5),
    ].join(":");
  }

  function prepareMapView(grid) {
    if (!validMapGrid(grid)) return;
    const signature = mapGeometrySignature(grid);
    if (!mapView.geometrySignature) {
      mapView.geometrySignature = signature;
    } else if (mapView.geometrySignature !== signature) {
      // A genuinely different map should start at a useful fit.  A preview
      // and its live Nav2 /map normally have identical geometry, so switching
      // between them keeps the user's current pan/zoom unchanged.
      mapView.zoom = 1;
      mapView.panX = 0;
      mapView.panY = 0;
      mapView.geometrySignature = signature;
    }
    mapView.zoom = Math.max(MAP_MIN_ZOOM, Math.min(MAP_MAX_ZOOM, Number(mapView.zoom) || 1));
  }

  function mapViewport(grid, rect) {
    prepareMapView(grid);
    const width = Number(grid.width);
    const height = Number(grid.height);
    const fitCell = Math.min(
      (rect.width - 28) / width,
      (rect.height - 28) / height,
    );
    const cell = Math.max(0.01, fitCell * mapView.zoom);
    return {
      width,
      height,
      resolution: Number(grid.resolution),
      origin: grid.origin || { x: 0, y: 0, yaw: 0 },
      angle: Number(grid.origin?.yaw || 0),
      cell,
      ox: (rect.width - width * cell) / 2 + mapView.panX,
      oy: (rect.height - height * cell) / 2 + mapView.panY,
    };
  }

  function worldToCanvas(grid, viewport, x, y) {
    const origin = viewport.origin;
    const cosine = Math.cos(viewport.angle);
    const sine = Math.sin(viewport.angle);
    const dx = Number(x) - Number(origin.x || 0);
    const dy = Number(y) - Number(origin.y || 0);
    const gx = (cosine * dx + sine * dy) / viewport.resolution;
    const gy = (-sine * dx + cosine * dy) / viewport.resolution;
    return [
      viewport.ox + gx * viewport.cell,
      viewport.oy + (viewport.height - gy) * viewport.cell,
    ];
  }

  function canvasToWorld(grid, viewport, x, y) {
    const gx = (Number(x) - viewport.ox) / viewport.cell;
    const gy = viewport.height - (Number(y) - viewport.oy) / viewport.cell;
    const cosine = Math.cos(viewport.angle);
    const sine = Math.sin(viewport.angle);
    const localX = (cosine * gx - sine * gy) * viewport.resolution;
    const localY = (sine * gx + cosine * gy) * viewport.resolution;
    return {
      x: Number(viewport.origin.x || 0) + localX,
      y: Number(viewport.origin.y || 0) + localY,
    };
  }

  function updateMapZoomLabel() {
    const label = $("map-zoom-level");
    if (label) label.textContent = `${Math.round(mapView.zoom * 100)}%`;
  }

  function zoomMapAt(factor, clientX, clientY) {
    const grid = currentMapGrid();
    const canvas = $("map-canvas");
    if (!validMapGrid(grid) || !canvas) return;
    const rect = canvas.getBoundingClientRect();
    const viewport = mapViewport(grid, rect);
    const anchorX = Number(clientX) - rect.left;
    const anchorY = Number(clientY) - rect.top;
    const anchorWorld = canvasToWorld(grid, viewport, anchorX, anchorY);
    const nextZoom = Math.max(
      MAP_MIN_ZOOM,
      Math.min(MAP_MAX_ZOOM, mapView.zoom * Number(factor)),
    );
    if (nextZoom === mapView.zoom) return;
    mapView.zoom = nextZoom;
    const nextViewport = mapViewport(grid, rect);
    const projected = worldToCanvas(grid, nextViewport, anchorWorld.x, anchorWorld.y);
    // Compensate the pan so the point under the cursor stays under the
    // cursor, which makes wheel/pinch zoom feel natural.
    mapView.panX += anchorX - projected[0];
    mapView.panY += anchorY - projected[1];
    updateMapZoomLabel();
    drawMap();
  }

  function resetMapView() {
    mapView.zoom = 1;
    mapView.panX = 0;
    mapView.panY = 0;
    updateMapZoomLabel();
    drawMap();
  }

  function drawMap() {
    const canvas = $("map-canvas");
    const navigationActive = Boolean(state.processes?.navigation?.active);
    const grid = currentMapGrid();
    if (!canvas || !validMapGrid(grid)) return;
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    const pixelWidth = Math.max(1, Math.floor(rect.width * ratio));
    const pixelHeight = Math.max(1, Math.floor(rect.height * ratio));
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
      mapRenderCache.image = null;
    }
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const viewport = mapViewport(grid, rect);
    const { cell } = viewport;
    const checked = (id, fallback) => {
      const element = $(id);
      return element ? element.checked : fallback;
    };
    const showGlobal = navigationActive && checked("layer-global-costmap", true);
    const showLocal = navigationActive && checked("layer-local-costmap", true);
    const cacheMatches = Boolean(
      mapRenderCache.image
      && mapRenderCache.grid === grid
      && mapRenderCache.globalCostmap === state.global_costmap
      && mapRenderCache.localCostmap === state.local_costmap
      && mapRenderCache.navigationActive === navigationActive
      && mapRenderCache.showGlobal === showGlobal
      && mapRenderCache.showLocal === showLocal
      && mapRenderCache.zoom === mapView.zoom
      && mapRenderCache.panX === mapView.panX
      && mapRenderCache.panY === mapView.panY
      && mapRenderCache.width === canvas.width
      && mapRenderCache.height === canvas.height
      && mapRenderCache.logicalWidth === rect.width
      && mapRenderCache.logicalHeight === rect.height
      && mapRenderCache.ratio === ratio,
    );
    if (cacheMatches) {
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.putImageData(mapRenderCache.image, 0, 0);
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    } else {
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      ctx.fillStyle = "#f8fbfd";
      ctx.fillRect(0, 0, rect.width, rect.height);
    }
    const origin = viewport.origin;
    const originAngle = viewport.angle;
    const toCanvas = (x, y) => worldToCanvas(grid, viewport, x, y);
    const worldFromGrid = (gridOrigin, angle, col, row, resolution) => {
      const cosine = Math.cos(angle), sine = Math.sin(angle);
      return [
        Number(gridOrigin.x || 0) + cosine * col * resolution - sine * row * resolution,
        Number(gridOrigin.y || 0) + sine * col * resolution + cosine * row * resolution,
      ];
    };
    const worldFromMeters = (gridOrigin, angle, localX, localY) => {
      const cosine = Math.cos(angle), sine = Math.sin(angle);
      return [
        Number(gridOrigin.x || 0) + cosine * localX - sine * localY,
        Number(gridOrigin.y || 0) + sine * localX + cosine * localY,
      ];
    };
    const drawWorldCell = (gridOrigin, angle, col, row, width, height, resolution) => {
      const corners = [
        worldFromGrid(gridOrigin, angle, col, row, resolution),
        worldFromGrid(gridOrigin, angle, col + width, row, resolution),
        worldFromGrid(gridOrigin, angle, col + width, row + height, resolution),
        worldFromGrid(gridOrigin, angle, col, row + height, resolution),
      ].map(([x, y]) => toCanvas(x, y));
      ctx.beginPath();
      ctx.moveTo(corners[0][0], corners[0][1]);
      corners.slice(1).forEach(([x, y]) => ctx.lineTo(x, y));
      ctx.closePath();
      ctx.fill();
    };
    const sample = Math.max(1, Math.ceil(Math.sqrt((grid.width * grid.height) / 90000)));
    if (!cacheMatches) {
      for (let row = 0; row < grid.height; row += sample) {
        for (let col = 0; col < grid.width; col += sample) {
          const value = grid.data[row * grid.width + col] ?? -1;
          ctx.fillStyle = value < 0 ? "#dce6ec" : value >= 65 ? "#405765" : "#f8fbfd";
          drawWorldCell(
            origin,
            originAngle,
            col,
            row,
            Math.min(sample, grid.width - col),
            Math.min(sample, grid.height - row),
            Number(grid.resolution || 1),
          );
        }
      }
    }
    const costColor = (value, layer) => {
      const numeric = Number(value);
      const normalized = Math.max(0, Math.min(1, numeric > 100 ? numeric / 254 : numeric / 100));
      const stops = normalized < 0.5
        ? [[104, 195, 236], [246, 218, 83], normalized * 2]
        : [[246, 218, 83], [219, 49, 52], (normalized - 0.5) * 2];
      const red = Math.round(stops[0][0] + (stops[1][0] - stops[0][0]) * stops[2]);
      const green = Math.round(stops[0][1] + (stops[1][1] - stops[0][1]) * stops[2]);
      const blue = Math.round(stops[0][2] + (stops[1][2] - stops[0][2]) * stops[2]);
      const alpha = layer === "global" ? 0.16 + 0.68 * normalized : 0.10 + 0.40 * normalized;
      return `rgba(${red},${green},${blue},${alpha.toFixed(3)})`;
    };
    const drawCostmap = (costmap, layer) => {
      if (!costmap) return;
      // The backend transforms odom-frame local costmaps into map coordinates.
      // Avoid drawing a misleading offset overlay while that TF is unavailable.
      if (costmap.transform_ok === false && costmap.frame_id !== grid.frame_id) return;
      const overlaySample = Math.max(1, Math.ceil(Math.sqrt((costmap.width * costmap.height) / 30000)));
      const costOrigin = costmap.origin || { x: 0, y: 0, yaw: 0 };
      const costAngle = costOrigin.yaw || 0;
      for (let row = 0; row < costmap.height; row += overlaySample) {
        for (let col = 0; col < costmap.width; col += overlaySample) {
          const value = costmap.data[row * costmap.width + col] ?? -1;
          if (value < 1 || value === 255) continue;
          ctx.fillStyle = costColor(value, layer);
          drawWorldCell(
            costOrigin,
            costAngle,
            col,
            row,
            Math.min(overlaySample, costmap.width - col),
            Math.min(overlaySample, costmap.height - row),
            Number(costmap.resolution || 1),
          );
        }
      }
    };
    if (!cacheMatches) {
      if (showGlobal) drawCostmap(state.global_costmap, "global");
      if (showLocal) drawCostmap(state.local_costmap, "local");
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      mapRenderCache.grid = grid;
      mapRenderCache.globalCostmap = state.global_costmap;
      mapRenderCache.localCostmap = state.local_costmap;
      mapRenderCache.navigationActive = navigationActive;
      mapRenderCache.showGlobal = showGlobal;
      mapRenderCache.showLocal = showLocal;
      mapRenderCache.zoom = mapView.zoom;
      mapRenderCache.panX = mapView.panX;
      mapRenderCache.panY = mapView.panY;
      mapRenderCache.width = canvas.width;
      mapRenderCache.height = canvas.height;
      mapRenderCache.logicalWidth = rect.width;
      mapRenderCache.logicalHeight = rect.height;
      mapRenderCache.ratio = ratio;
      mapRenderCache.image = ctx.getImageData(0, 0, canvas.width, canvas.height);
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    }
    const displayPose = displayRobotPose();
    const drawWindow = (costmap) => {
      let corners;
      let label;
      if (costmap && !(costmap.transform_ok === false && costmap.frame_id !== grid.frame_id)) {
        const windowOrigin = costmap.origin || { x: 0, y: 0, yaw: 0 };
        const angle = Number(windowOrigin.yaw || 0);
        const width = Number(costmap.width || 0) * Number(costmap.resolution || 0);
        const height = Number(costmap.height || 0) * Number(costmap.resolution || 0);
        corners = [
          worldFromMeters(windowOrigin, angle, 0, 0),
          worldFromMeters(windowOrigin, angle, width, 0),
          worldFromMeters(windowOrigin, angle, width, height),
          worldFromMeters(windowOrigin, angle, 0, height),
        ];
        label = `LOCAL ${width.toFixed(1)}×${height.toFixed(1)} m`;
      } else if (displayPose) {
        const fallback = state.navigation_readiness?.local_costmap_window || {};
        const width = Number(fallback.width) || 4;
        const height = Number(fallback.height) || 4;
        corners = [
          [displayPose.x - width / 2, displayPose.y - height / 2],
          [displayPose.x + width / 2, displayPose.y - height / 2],
          [displayPose.x + width / 2, displayPose.y + height / 2],
          [displayPose.x - width / 2, displayPose.y + height / 2],
        ];
        label = `LOCAL WINDOW ${width.toFixed(1)}×${height.toFixed(1)} m · WAITING`;
      }
      if (!corners) return;
      const canvasCorners = corners.map(([x, y]) => toCanvas(x, y));
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(canvasCorners[0][0], canvasCorners[0][1]);
      canvasCorners.slice(1).forEach(([x, y]) => ctx.lineTo(x, y));
      ctx.closePath();
      ctx.strokeStyle = "rgba(30,137,166,.9)";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "rgba(30,112,142,.95)";
      ctx.font = "700 8px monospace";
      ctx.fillText(label, canvasCorners[0][0] + 4, canvasCorners[0][1] - 5);
      ctx.restore();
    };
    if (navigationActive && checked("layer-local-costmap", true)) drawWindow(state.local_costmap);
    const drawGlobalPlan = (path) => {
      if (!path || !Array.isArray(path.poses) || path.poses.length < 2) return;
      // The backend converts non-map plans to the map frame.  If TF was not
      // available, leave the path hidden instead of drawing it at a wrong
      // offset on the occupancy grid.
      if (path.transform_ok === false && path.frame_id !== grid.frame_id) return;
      const points = path.poses
        .map((point) => toCanvas(Number(point.x), Number(point.y)))
        .filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y));
      if (points.length < 2) return;
      ctx.save();
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      // A light outline keeps the planner route visible over both the dark
      // occupancy map and the costmap gradients.
      ctx.beginPath();
      ctx.moveTo(points[0][0], points[0][1]);
      points.slice(1).forEach(([x, y]) => ctx.lineTo(x, y));
      ctx.strokeStyle = "rgba(255,255,255,.92)";
      ctx.lineWidth = 5;
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(points[0][0], points[0][1]);
      points.slice(1).forEach(([x, y]) => ctx.lineTo(x, y));
      ctx.strokeStyle = "#b3339b";
      ctx.lineWidth = 2.5;
      ctx.stroke();
      ctx.fillStyle = "#b3339b";
      ctx.beginPath();
      ctx.arc(points[points.length - 1][0], points[points.length - 1][1], 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    };
    if (navigationActive && checked("layer-global-plan", true)) drawGlobalPlan(state.path);
    if (checked("layer-lidar", false) && state.scan && displayPose) {
      ctx.fillStyle = "rgba(36,126,164,.62)";
      const sensorX = Number(state.scan.sensor_x) || 0;
      const sensorY = Number(state.scan.sensor_y) || 0;
      const sensorTheta = Number(state.scan.sensor_theta) || 0;
      const laserX = displayPose.x + Math.cos(displayPose.theta) * sensorX - Math.sin(displayPose.theta) * sensorY;
      const laserY = displayPose.y + Math.sin(displayPose.theta) * sensorX + Math.cos(displayPose.theta) * sensorY;
      state.scan.ranges.forEach((range, index) => {
        if (range === null || range < state.scan.range_min || range > state.scan.range_max) return;
        const angle = displayPose.theta + sensorTheta + state.scan.angle_min + index * state.scan.angle_increment;
        const [x, y] = toCanvas(laserX + range * Math.cos(angle), laserY + range * Math.sin(angle));
        ctx.fillRect(x - 1, y - 1, 2.5, 2.5);
      });
    }
    if (checked("layer-robot", true) && displayPose) {
      const [x, y] = toCanvas(displayPose.x, displayPose.y);
      let footprintPoints = null;
      const footprint = state.footprint;
      if (footprint && footprint.transform_ok && Array.isArray(footprint.points) && footprint.points.length >= 3) {
        footprintPoints = footprint.points.map((point) => [Number(point.x), Number(point.y)]);
      }
      if (!footprintPoints) {
        const fallback = [[0.30, 0.20], [0.30, -0.20], [-0.30, -0.20], [-0.30, 0.20]];
        const cosine = Math.cos(displayPose.theta), sine = Math.sin(displayPose.theta);
        footprintPoints = fallback.map(([px, py]) => [
          displayPose.x + cosine * px - sine * py,
          displayPose.y + sine * px + cosine * py,
        ]);
      }
      const footprintCanvas = footprintPoints.map(([px, py]) => toCanvas(px, py));
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(footprintCanvas[0][0], footprintCanvas[0][1]);
      footprintCanvas.slice(1).forEach(([px, py]) => ctx.lineTo(px, py));
      ctx.closePath();
      ctx.fillStyle = "rgba(61,145,85,.28)";
      ctx.strokeStyle = "#2f7b45";
      ctx.lineWidth = 2;
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(-displayPose.theta);
      const arrowSize = Math.max(9, Math.min(22, cell * 0.35 / Number(grid.resolution || 1)));
      ctx.fillStyle = "#4f925c";
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(arrowSize, 0);
      ctx.lineTo(-arrowSize * .70, -arrowSize * .48);
      ctx.lineTo(-arrowSize * .48, 0);
      ctx.lineTo(-arrowSize * .70, arrowSize * .48);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
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
    const grid = currentMapGrid();
    const canvas = $("map-canvas");
    if (!validMapGrid(grid) || !canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const viewport = mapViewport(grid, rect);
    // Invert exactly the same rotated/panned/zoomed transform used by
    // drawMap(), so pose clicks remain correct after the user moves the view.
    return canvasToWorld(
      grid,
      viewport,
      event.clientX - rect.left,
      event.clientY - rect.top,
    );
  }

  function submitGoal(force = false) {
    const pose = poseValues("goal");
    if (![pose.x, pose.y, pose.theta].every(Number.isFinite)) {
      log("Goal memiliki koordinat non-finite.", "warn");
      return Promise.resolve(false);
    }
    return fetch("/api/goal/nav2", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x: pose.x, y: pose.y, theta: pose.theta, force }),
    }).then(async (response) => {
      let payload = {};
      try { payload = await response.json(); } catch { /* empty response */ }
      if (!response.ok) {
        const detail = typeof payload.detail === "string" ? payload.detail : payload.detail?.message;
        throw new Error(detail || "Nav2 goal ditolak");
      }
      return payload;
    }).then((payload) => {
      log(payload.message || (force ? "Force goal dikirim." : "Goal pose dikirim ke Nav2."), "success");
      return payload.accepted !== false;
    }).catch((error) => {
      log(error.message || "Goal pose gagal dikirim.", force ? "warn" : "error");
      return false;
    });
  }

  function initMapTools() {
    const select = $("map-select");
    const loadPreview = (name, selected = false) => fetch(`/api/maps/${encodeURIComponent(name)}`)
      .then((response) => response.ok ? response.json() : Promise.reject(new Error("preview unavailable")))
      .then((map) => {
        if (selected) state.selectedMap = map;
        else { state.previewMap = map; state.selectedMap = null; }
        render();
        return map;
      });
    fetch("/api/maps")
      .then((response) => response.ok ? response.json() : Promise.reject(new Error("map catalog unavailable")))
      .then((items) => {
        (Array.isArray(items) ? items : []).forEach((item) => { const name = typeof item === "string" ? item : item.name; if (!name) return; const option = document.createElement("option"); option.value = name; option.textContent = name; select.appendChild(option); });
        return fetch("/api/maps/selected").then((response) => response.ok ? response.json() : null);
      })
      .then((payload) => {
        const name = payload && payload.map_name;
        if (!name) return;
        select.value = name.endsWith(".pgm") ? name : `${name.replace(/\.yaml$/i, "")}.pgm`;
        return loadPreview(select.value, true);
      })
      .catch(() => {});
    select.addEventListener("change", () => {
      const name = select.value;
      if (!name) { state.previewMap = null; render(); return; }
      loadPreview(name).then(() => log(`Preview map ${name} dimuat. Tekan SELECT MAP untuk menerapkan.`, "info")).catch((error) => log(`Preview map gagal: ${error.message}`, "warn"));
    });
    $("map-select-apply").addEventListener("click", () => {
      if (!state.previewMap) return;
      const name = state.previewMap.name || select.value;
      fetch("/api/maps/select", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ map_name: name }) })
        .then((response) => response.json().then((payload) => ({ response, payload })))
        .then(({ response, payload }) => {
          if (!response.ok) throw new Error(payload.detail || "select map gagal");
          state.selectedMap = state.previewMap;
          render();
          log(payload.message || `Map ${name} dipilih untuk localization.`, "success");
        })
        .catch((error) => log(`Map selection gagal: ${error.message}`, "warn"));
    });
    [["initial", "initial-pick"], ["goal", "goal-pick"]].forEach(([kind, id]) => $(id).addEventListener("click", () => {
      poseTool = poseTool === kind ? null : kind;
      mapPointerStart = null;
      mapPanStart = null;
      $("initial-tool").classList.toggle("active", poseTool === "initial");
      $("goal-tool").classList.toggle("active", poseTool === "goal");
      $("map-canvas").classList.toggle("map-interactive", Boolean(poseTool));
    }));
    const canvas = $("map-canvas");
    canvas.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      canvas.setPointerCapture(event.pointerId);
      if (poseTool) {
        mapPanStart = null;
        mapPointerStart = mapPoint(event);
        if (mapPointerStart) setPoseValues(poseTool, { ...mapPointerStart, theta: 0 });
        return;
      }
      mapPointerStart = null;
      mapPanStart = {
        pointerId: event.pointerId,
        x: event.clientX,
        y: event.clientY,
        panX: mapView.panX,
        panY: mapView.panY,
      };
      canvas.classList.add("map-panning");
    });
    canvas.addEventListener("pointermove", (event) => {
      if (poseTool && mapPointerStart) {
        const point = mapPoint(event);
        if (!point) return;
        const distance = Math.hypot(point.x - mapPointerStart.x, point.y - mapPointerStart.y);
        const theta = distance > 0.03
          ? Math.atan2(point.y - mapPointerStart.y, point.x - mapPointerStart.x)
          : 0;
        setPoseValues(poseTool, { x: mapPointerStart.x, y: mapPointerStart.y, theta });
        return;
      }
      if (!mapPanStart || mapPanStart.pointerId !== event.pointerId) return;
      mapView.panX = mapPanStart.panX + event.clientX - mapPanStart.x;
      mapView.panY = mapPanStart.panY + event.clientY - mapPanStart.y;
      drawMap();
    });
    const finishPointer = (event) => {
      if (mapPanStart && (!event || mapPanStart.pointerId === event.pointerId)) {
        mapPanStart = null;
        canvas.classList.remove("map-panning");
      }
      mapPointerStart = null;
    };
    canvas.addEventListener("pointerup", finishPointer);
    canvas.addEventListener("pointercancel", finishPointer);
    canvas.addEventListener("lostpointercapture", finishPointer);
    canvas.addEventListener("wheel", (event) => {
      if (!validMapGrid(currentMapGrid())) return;
      event.preventDefault();
      zoomMapAt(event.deltaY < 0 ? 1.15 : 1 / 1.15, event.clientX, event.clientY);
    }, { passive: false });
    $("map-zoom-in")?.addEventListener("click", () => {
      const rect = canvas.getBoundingClientRect();
      zoomMapAt(1.25, rect.left + rect.width / 2, rect.top + rect.height / 2);
    });
    $("map-zoom-out")?.addEventListener("click", () => {
      const rect = canvas.getBoundingClientRect();
      zoomMapAt(1 / 1.25, rect.left + rect.width / 2, rect.top + rect.height / 2);
    });
    $("map-zoom-reset")?.addEventListener("click", resetMapView);
    updateMapZoomLabel();
    ["layer-robot", "layer-lidar", "layer-local-costmap", "layer-global-costmap", "layer-global-plan"].forEach((id) => $(id)?.addEventListener("change", render));
    $("initial-send").addEventListener("click", () => {
      const pose = poseValues("initial");
      command({ type: "initial_pose", x: pose.x, y: pose.y, theta: pose.theta }, "/api/localization/initialpose").then((accepted) => {
        if (accepted) { state.initialPose = null; render(); }
        log(accepted ? "Initial pose dikirim ke AMCL; marker digantikan panah robot." : "Initial pose gagal dikirim.", accepted ? "success" : "warn");
      });
    });
    $("goal-send").addEventListener("click", () => submitGoal(false));
    $("goal-force").addEventListener("click", () => {
      if (!window.confirm("FORCE NAV2 GOAL mengabaikan gate readiness. Lanjutkan untuk diagnosis?")) return;
      submitGoal(true);
    });
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
