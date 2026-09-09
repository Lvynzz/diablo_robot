import { useCallback, useEffect, useRef, useState, type PointerEvent } from "react";
import { Icon } from "./Icon";
import { EmptyState, Panel, StatCard } from "./Panel";
import { LaunchToggleButton } from "./LaunchControls";
import type {
  DiabloState,
  EventEntry,
  HardwareStatus,
  OccupancyGrid,
  PanelKey,
  Pose,
  SocketCommand,
} from "../types";

function fmt(value: number | null | undefined, digits = 2) {
  return value === null || value === undefined || !Number.isFinite(value)
    ? "—"
    : value.toFixed(digits);
}

function degrees(value: number | null | undefined) {
  return value === null || value === undefined || !Number.isFinite(value)
    ? "—"
    : ((value * 180) / Math.PI).toFixed(1);
}

function componentLabel(value: HardwareStatus["components"][number]) {
  return value.state === "not_configured"
    ? "NOT CONFIGURED"
    : value.state.replace("_", " ").toUpperCase();
}

interface PoseDraft {
  x: string;
  y: string;
  heading: string;
}

const emptyPoseDraft: PoseDraft = { x: "0.00", y: "0.00", heading: "0.0" };

function poseFromDraft(draft: PoseDraft, source: string): Pose | null {
  const x = Number(draft.x);
  const y = Number(draft.y);
  const theta = (Number(draft.heading) * Math.PI) / 180;
  return Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(theta)
    ? { x, y, theta, source }
    : null;
}

function PoseDraftFields({ draft, onChange }: { draft: PoseDraft; onChange: (field: keyof PoseDraft, value: string) => void }) {
  return <div className="mapping-pose-fields">
    {(["x", "y", "heading"] as const).map((field) => <label key={field}><span>{field === "heading" ? "HEADING (DEG)" : field.toUpperCase() + " (M)"}</span><input type="number" step="0.01" value={draft[field]} onChange={(event) => onChange(field, event.target.value)} /></label>)}
  </div>;
}

interface OccupancyCanvasProps {
  grid: OccupancyGrid | null;
  pose: Pose | null;
  scan: DiabloState["scan"];
  globalCostmap: OccupancyGrid | null;
  localCostmap: OccupancyGrid | null;
  showRobot: boolean;
  showLidar: boolean;
  showGlobalCostmap: boolean;
  showLocalCostmap: boolean;
  initialPose: Pose | null;
  goalPose: Pose | null;
  onPick: (pose: Pose) => void;
  interactive: boolean;
}

function OccupancyCanvas({ grid, pose, scan, globalCostmap, localCostmap, showRobot, showLidar, showGlobalCostmap, showLocalCostmap, initialPose, goalPose, onPick, interactive }: OccupancyCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 1, height: 1 });
  const pointerStart = useRef<{ x: number; y: number } | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const update = () => {
      const rect = canvas.getBoundingClientRect();
      setSize({ width: Math.max(1, rect.width), height: Math.max(1, rect.height) });
    };
    update();
    const observer = new ResizeObserver(update);
    observer.observe(canvas);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !grid) return;
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.floor(size.width * ratio);
    canvas.height = Math.floor(size.height * ratio);
    const context = canvas.getContext("2d");
    if (!context) return;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, size.width, size.height);
    context.fillStyle = "#f8fbfd";
    context.fillRect(0, 0, size.width, size.height);

    const cell = Math.min(
      (size.width - 28) / Math.max(1, grid.width),
      (size.height - 28) / Math.max(1, grid.height),
    );
    const offsetX = (size.width - grid.width * cell) / 2;
    const offsetY = (size.height - grid.height * cell) / 2;
    const sample = Math.max(1, Math.ceil(Math.sqrt((grid.width * grid.height) / 180000)));

    for (let row = 0; row < grid.height; row += sample) {
      for (let column = 0; column < grid.width; column += sample) {
        const value = grid.data[row * grid.width + column] ?? -1;
        context.fillStyle = value < 0 ? "#dce6ec" : value >= 65 ? "#405765" : "#f8fbfd";
        context.fillRect(
          offsetX + column * cell,
          offsetY + (grid.height - row - sample) * cell,
          Math.ceil(cell * sample + 0.3),
          Math.ceil(cell * sample + 0.3),
        );
      }
    }

    const origin = grid.origin || { x: 0, y: 0, yaw: 0 };
    const toCanvas = (x: number, y: number) => {
      const dx = x - origin.x;
      const dy = y - origin.y;
      const angle = origin.yaw || 0;
      const gx = (Math.cos(angle) * dx + Math.sin(angle) * dy) / grid.resolution;
      const gy = (-Math.sin(angle) * dx + Math.cos(angle) * dy) / grid.resolution;
      return [offsetX + gx * cell, offsetY + (grid.height - gy) * cell] as const;
    };

    const drawCostmap = (costmap: OccupancyGrid | null, layer: "global" | "local") => {
      if (!costmap) return;
      if (costmap.transform_ok === false && costmap.frame_id !== grid.frame_id) return;
      const overlaySample = Math.max(1, Math.ceil(Math.sqrt((costmap.width * costmap.height) / 65000)));
      const costmapOrigin = costmap.origin || { x: 0, y: 0, yaw: 0 };
      const angle = costmapOrigin.yaw || 0;
      for (let row = 0; row < costmap.height; row += overlaySample) {
        for (let column = 0; column < costmap.width; column += overlaySample) {
          const value = costmap.data[row * costmap.width + column] ?? -1;
          if (value < 1) continue;
          const worldX = costmapOrigin.x + Math.cos(angle) * column * costmap.resolution - Math.sin(angle) * row * costmap.resolution;
          const worldY = costmapOrigin.y + Math.sin(angle) * column * costmap.resolution + Math.cos(angle) * row * costmap.resolution;
          const [x, y] = toCanvas(worldX, worldY);
          const lethal = value >= 90;
          context.fillStyle = layer === "global"
            ? `rgba(47,120,174,${lethal ? 0.55 : 0.23})`
            : `rgba(197,83,0,${lethal ? 0.55 : 0.23})`;
          const sizeValue = Math.max(1, cell * costmap.resolution / grid.resolution * overlaySample + 0.5);
          context.fillRect(x, y - sizeValue, sizeValue, sizeValue);
        }
      }
    };

    if (showGlobalCostmap) drawCostmap(globalCostmap, "global");
    if (showLocalCostmap) drawCostmap(localCostmap, "local");

    if (showLidar && scan && pose) {
      context.fillStyle = "rgba(36, 126, 164, .62)";
      const sensorX = scan.sensor_x || 0;
      const sensorY = scan.sensor_y || 0;
      const sensorTheta = scan.sensor_theta || 0;
      const laserX = pose.x + Math.cos(pose.theta) * sensorX - Math.sin(pose.theta) * sensorY;
      const laserY = pose.y + Math.sin(pose.theta) * sensorX + Math.cos(pose.theta) * sensorY;
      scan.ranges.forEach((range, index) => {
        if (range === null || range < scan.range_min || range > scan.range_max) return;
        const angle = pose.theta + sensorTheta + scan.angle_min + index * scan.angle_increment;
        const [x, y] = toCanvas(
          laserX + range * Math.cos(angle),
          laserY + range * Math.sin(angle),
        );
        context.fillRect(x - 1, y - 1, 2.5, 2.5);
      });
    }

    if (showRobot && pose) {
      const [x, y] = toCanvas(pose.x, pose.y);
      context.save();
      context.translate(x, y);
      context.rotate(-pose.theta);
      context.fillStyle = "#4f925c";
      context.strokeStyle = "#ffffff";
      context.lineWidth = 2;
      context.beginPath();
      context.moveTo(13, 0);
      context.lineTo(-9, -7);
      context.lineTo(-6, 0);
      context.lineTo(-9, 7);
      context.closePath();
      context.fill();
      context.stroke();
      context.restore();
    }
    const marker = (markerPose: Pose, color: string) => {
      const [x, y] = toCanvas(markerPose.x, markerPose.y);
      context.save();
      context.translate(x, y);
      context.rotate(-markerPose.theta);
      context.fillStyle = color;
      context.strokeStyle = "#ffffff";
      context.lineWidth = 2;
      context.beginPath();
      context.arc(0, 0, 8, 0, Math.PI * 2);
      context.stroke();
      context.beginPath();
      context.moveTo(12, 0);
      context.lineTo(-7, -5);
      context.lineTo(-5, 0);
      context.lineTo(-7, 5);
      context.closePath();
      context.fill();
      context.restore();
    };
    if (initialPose) marker(initialPose, "#2c83a9");
    if (goalPose) marker(goalPose, "#c55300");
  }, [globalCostmap, grid, goalPose, initialPose, localCostmap, pose, scan, showGlobalCostmap, showLidar, showLocalCostmap, showRobot, size]);

  const worldFromPointer = (event: PointerEvent<HTMLCanvasElement>) => {
    if (!grid) return null;
    const rect = event.currentTarget.getBoundingClientRect();
    const cell = Math.min((rect.width - 28) / Math.max(1, grid.width), (rect.height - 28) / Math.max(1, grid.height));
    const offsetX = (rect.width - grid.width * cell) / 2;
    const offsetY = (rect.height - grid.height * cell) / 2;
    const gx = (event.clientX - rect.left - offsetX) / cell;
    const gy = grid.height - (event.clientY - rect.top - offsetY) / cell;
    const origin = grid.origin || { x: 0, y: 0, yaw: 0 };
    const localX = gx * grid.resolution;
    const localY = gy * grid.resolution;
    return {
      x: origin.x + Math.cos(origin.yaw) * localX - Math.sin(origin.yaw) * localY,
      y: origin.y + Math.sin(origin.yaw) * localX + Math.cos(origin.yaw) * localY,
    };
  };

  const pick = (event: PointerEvent<HTMLCanvasElement>, final = false) => {
    const point = worldFromPointer(event);
    if (!point || !interactive) return;
    const start = pointerStart.current || point;
    const distance = Math.hypot(point.x - start.x, point.y - start.y);
    const theta = distance > 0.03 ? Math.atan2(point.y - start.y, point.x - start.x) : 0;
    onPick({ x: start.x, y: start.y, theta, source: "map click" });
    if (final) pointerStart.current = null;
  };

  const onPointerDown = (event: PointerEvent<HTMLCanvasElement>) => {
    if (!interactive || !grid) return;
    event.currentTarget.setPointerCapture(event.pointerId);
    pointerStart.current = worldFromPointer(event);
    pick(event);
  };
  const onPointerMove = (event: PointerEvent<HTMLCanvasElement>) => {
    if (pointerStart.current) pick(event);
  };
  const onPointerUp = (event: PointerEvent<HTMLCanvasElement>) => {
    if (pointerStart.current) pick(event, true);
  };

  if (!grid) {
    return <EmptyState title="Menunggu occupancy grid" detail="Nyalakan hardware, lalu mulai mapping untuk mengisi /map." />;
  }
  return <canvas ref={canvasRef} className={`mapping-canvas ${interactive ? "map-interactive" : ""}`} aria-label="Live occupancy grid map" onPointerDown={onPointerDown} onPointerMove={onPointerMove} onPointerUp={onPointerUp} onPointerCancel={() => { pointerStart.current = null; }} />;
}

interface MappingViewProps {
  state: DiabloState;
  hardware: HardwareStatus;
  panels: Record<PanelKey, boolean>;
  sendCommand: (command: SocketCommand) => Promise<boolean>;
  onEvent: (message: string, kind?: EventEntry["kind"]) => void;
}

export function MappingView({ state, hardware, panels, sendCommand, onEvent }: MappingViewProps) {
  const [mapName, setMapName] = useState("");
  const [forwardSpeed, setForwardSpeed] = useState(0.12);
  const [turnSpeed, setTurnSpeed] = useState(0.35);
  const [activeKeys, setActiveKeys] = useState<Set<string>>(new Set());
  const keysRef = useRef<Set<string>>(new Set());
  const timerRef = useRef<number | null>(null);
  const [mapChoices, setMapChoices] = useState<string[]>([]);
  const [mapChoice, setMapChoice] = useState("");
  const [previewMap, setPreviewMap] = useState<OccupancyGrid | null>(null);
  const [selectedMap, setSelectedMap] = useState<OccupancyGrid | null>(null);
  const [mapLoading, setMapLoading] = useState(false);
  const [poseTool, setPoseTool] = useState<"initial" | "goal" | null>(null);
  const [initialDraft, setInitialDraft] = useState<PoseDraft>({ ...emptyPoseDraft });
  const [goalDraft, setGoalDraft] = useState<PoseDraft>({ ...emptyPoseDraft, x: "1.00", y: "0.50" });
  const [showRobot, setShowRobot] = useState(true);
  const [showLidar, setShowLidar] = useState(false);
  const [showLocalCostmap, setShowLocalCostmap] = useState(true);
  const [showGlobalCostmap, setShowGlobalCostmap] = useState(true);
  const [initialPoseApplied, setInitialPoseApplied] = useState(false);

  const mappingActive = state.mapping.active;
  const mappingHardwareReady = hardware.mapping_ready;
  // Teleoperation only needs the Diablo motor feedback.  LiDAR and SLAM are
  // independent, so the robot can be driven immediately after hardware start.
  const teleopReady = hardware.ready;
  const displayMap = selectedMap || previewMap || state.map;
  const displayedPose = state.pose || state.wheel_pose;
  const initialPose = poseFromDraft(initialDraft, "initial pose draft");
  const goalPose = poseFromDraft(goalDraft, "goal pose draft");

  useEffect(() => {
    let active = true;
    fetch("/api/maps").then((response) => response.ok ? response.json() : Promise.reject(new Error("map catalog unavailable"))).then((value: unknown) => {
      if (!active || !Array.isArray(value)) return;
      const names = value.map((item) => typeof item === "string" ? item : String((item as { name?: unknown }).name || "")).filter(Boolean);
      setMapChoices(names);
      return fetch("/api/maps/selected").then((response) => response.ok ? response.json() : null);
    }).then((payload: unknown) => {
      if (!active || !payload || typeof payload !== "object") return;
      const selectedName = String((payload as { map_name?: unknown }).map_name || "");
      if (!selectedName) return;
      setMapChoice(selectedName.endsWith(".pgm") ? selectedName : `${selectedName.replace(/\.yaml$/i, "")}.pgm`);
      return fetch(`/api/maps/${encodeURIComponent(selectedName)}`).then((response) => response.ok ? response.json() : null).then((map) => {
        if (active && map) setSelectedMap(map as OccupancyGrid);
      });
    }).catch(() => { if (active) setMapChoices([]); });
    return () => { active = false; };
  }, []);

  const chooseMap = async (name: string) => {
    setMapChoice(name);
    if (!name) { setPreviewMap(null); setSelectedMap(null); return; }
    setSelectedMap(null);
    setMapLoading(true);
    try {
      const response = await fetch(`/api/maps/${encodeURIComponent(name)}`);
      if (!response.ok) throw new Error("Map preview unavailable");
      setPreviewMap(await response.json() as OccupancyGrid);
      onEvent(`Preview map ${name} dimuat. Tekan SELECT MAP jika ingin menggunakannya.`, "info");
    } catch (error) {
      onEvent(`Preview map gagal: ${error instanceof Error ? error.message : "unknown error"}`, "warn");
    } finally { setMapLoading(false); }
  };

  const applyMap = async () => {
    if (!previewMap) { onEvent("Pilih map PGM terlebih dahulu.", "warn"); return; }
    const name = previewMap.name || mapChoice;
    try {
      const response = await fetch("/api/maps/select", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ map_name: name }),
      });
      const payload = await response.json().catch(() => ({}));
      if (!response.ok) throw new Error(String(payload.detail || "select map gagal"));
      setSelectedMap(previewMap);
      onEvent(String(payload.message || `Map ${name} dipilih untuk localization.`), "success");
    } catch (error) {
      onEvent(`Map selection gagal: ${error instanceof Error ? error.message : "unknown error"}`, "warn");
    }
  };

  const updatePose = (which: "initial" | "goal", field: keyof PoseDraft, value: string) => {
    const setter = which === "initial" ? setInitialDraft : setGoalDraft;
    if (which === "initial") setInitialPoseApplied(false);
    setter((previous) => ({ ...previous, [field]: value }));
  };

  const pickPose = (picked: Pose) => {
    if (!poseTool) return;
    const setter = poseTool === "initial" ? setInitialDraft : setGoalDraft;
    if (poseTool === "initial") setInitialPoseApplied(false);
    setter({ x: picked.x.toFixed(2), y: picked.y.toFixed(2), heading: (picked.theta * 180 / Math.PI).toFixed(1) });
  };

  const publishPose = async (which: "initial" | "goal") => {
    const pose = which === "initial" ? initialPose : goalPose;
    if (!pose) { onEvent("Isi X, Y, dan heading yang valid terlebih dahulu.", "warn"); return; }
    const accepted = await sendCommand(which === "initial" ? { type: "initial_pose", x: pose.x, y: pose.y, theta: pose.theta } : { type: "goal_pose", x: pose.x, y: pose.y, theta: pose.theta });
    if (accepted && which === "initial") setInitialPoseApplied(true);
    onEvent(accepted ? (which === "initial" ? "Initial pose dikirim ke AMCL." : "Goal pose dikirim ke Nav2.") : "Perintah pose gagal dikirim.", accepted ? "success" : "error");
  };

  const sendMotion = useCallback(() => {
    if (!teleopReady) return;
    const keys = keysRef.current;
    const forward = (keys.has("w") ? forwardSpeed : 0) - (keys.has("s") ? forwardSpeed : 0);
    const left = (keys.has("a") ? turnSpeed : 0) - (keys.has("d") ? turnSpeed : 0);
    if (forward === 0 && left === 0) {
      void sendCommand({ type: "stop" });
      return;
    }
    void sendCommand({
      type: "manual",
      forward,
      left,
      roll: 0,
      up: 1,
      pitch: 0,
    });
  }, [forwardSpeed, sendCommand, teleopReady, turnSpeed]);

  const stopTeleop = useCallback(() => {
    keysRef.current.clear();
    setActiveKeys(new Set());
    if (timerRef.current !== null) {
      window.clearInterval(timerRef.current);
      timerRef.current = null;
    }
    void sendCommand({ type: "stop" });
  }, [sendCommand]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null;
      if (target?.tagName === "INPUT" || target?.tagName === "TEXTAREA" || target?.isContentEditable) return;
      const key = event.key.toLowerCase();
      if (!["w", "a", "s", "d"].includes(key)) return;
      event.preventDefault();
      keysRef.current.add(key);
      setActiveKeys(new Set(keysRef.current));
      if (timerRef.current === null) {
        sendMotion();
        timerRef.current = window.setInterval(sendMotion, 100);
      }
    };
    const onKeyUp = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      if (!["w", "a", "s", "d"].includes(key)) return;
      keysRef.current.delete(key);
      setActiveKeys(new Set(keysRef.current));
      if (keysRef.current.size === 0) stopTeleop();
    };
    const onBlur = () => stopTeleop();
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onBlur);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
      stopTeleop();
    };
  }, [sendMotion, stopTeleop]);

  useEffect(() => {
    if (!teleopReady) stopTeleop();
  }, [stopTeleop, teleopReady]);

  const requestHardware = async () => {
    const accepted = await sendCommand({ type: "start_hardware" });
    onEvent(accepted ? "Hardware startup requested: Diablo, LiDAR and optional Dynamixel arm." : "Hardware startup request failed.", accepted ? "success" : "error");
  };

  const requestMapping = async () => {
    if (!mappingHardwareReady) {
      onEvent("Mapping dikunci: tunggu feedback motor Diablo dan LiDAR READY.", "warn");
      return;
    }
    const accepted = await sendCommand({ type: "start_mapping" });
    onEvent(accepted ? "SLAM Toolbox mapping startup requested." : "Mapping tidak dapat dimulai.", accepted ? "success" : "error");
  };

  const saveMap = async () => {
    const name = mapName.trim();
    if (!name) {
      onEvent("Masukkan nama map terlebih dahulu.", "warn");
      return;
    }
    const accepted = await sendCommand({ type: "save_map", name });
    onEvent(accepted ? `Permintaan simpan map '${name}' dikirim.` : "Map tidak dapat disimpan.", accepted ? "success" : "error");
  };

  const mappingState = state.mapping.state.toUpperCase();

  return (
    <div className="view-stack mapping-view">
      {panels.map && <>
        <div className="mapping-pose-strip">
          <StatCard label="ROBOT X" value={fmt(displayedPose?.x)} unit={state.pose?.source === "amcl" || state.pose?.source === "map" || state.pose?.source === "amcl_initial" ? "METERS · AMCL / MAP" : "METERS · ODOM"} tone="green" />
          <StatCard label="ROBOT Y" value={fmt(displayedPose?.y)} unit={state.pose?.source === "amcl" || state.pose?.source === "map" || state.pose?.source === "amcl_initial" ? "METERS · AMCL / MAP" : "METERS · ODOM"} tone="blue" />
          <StatCard label="HEADING θ" value={degrees(displayedPose?.theta)} unit={state.pose?.source === "amcl" || state.pose?.source === "map" || state.pose?.source === "amcl_initial" ? "DEGREES · AMCL / MAP" : "DEGREES · ODOM"} tone="orange" />
          <div className="mapping-reset-actions"><span>POSE RESET</span><button type="button" onClick={() => void sendCommand({ type: "reset_position" })}>RESET X/Y</button><button type="button" onClick={() => void sendCommand({ type: "reset_orientation" })}>RESET HEADING</button></div>
        </div>
        <Panel title="Live Occupancy Grid" eyebrow="SLAM TOOLBOX // /MAP" accent="blue" actions={<div className="mapping-map-actions"><label className="map-choice"><span>SELECT MAP</span><select value={mapChoice} onChange={(event) => void chooseMap(event.target.value)} aria-label="Select PGM map"><option value="">LIVE /MAP</option>{mapChoices.map((name) => <option key={name} value={name}>{name}</option>)}</select></label><button className="panel-icon-action" type="button" disabled={!previewMap || mapLoading} onClick={applyMap}>{mapLoading ? "…" : "SELECT"}</button><span className="panel-chip">FRAME: {displayMap?.frame_id || "—"}</span></div>}>
          <div className="mapping-map-layout">
            <div className="mapping-map-stage">
              <OccupancyCanvas grid={displayMap} pose={displayedPose} scan={state.scan} globalCostmap={state.global_costmap} localCostmap={state.local_costmap} showRobot={showRobot} showLidar={showLidar} showGlobalCostmap={showGlobalCostmap} showLocalCostmap={showLocalCostmap} initialPose={initialPoseApplied ? null : initialPose} goalPose={goalPose} onPick={pickPose} interactive={poseTool !== null} />
              <div className="map-legend"><span><i className="legend-dot green" /> Diablo</span><span><i className="legend-dot cyan" /> LiDAR</span><span><i className="legend-dot blue" /> Init</span><span><i className="legend-dot orange" /> Goal</span></div>
            </div>
            <div className="mapping-map-tools">
              <div className={`mapping-pose-tool init ${poseTool === "initial" ? "active" : ""}`}><div className="pose-tool-heading"><span>SET INIT POSE</span><button type="button" onClick={() => setPoseTool(poseTool === "initial" ? null : "initial")}>{poseTool === "initial" ? "MAP ACTIVE" : "PICK MAP"}</button></div><p>Lokalisasi AMCL · drag di map untuk arah.</p><PoseDraftFields draft={initialDraft} onChange={(field, value) => updatePose("initial", field, value)} /><button className="primary-action" type="button" onClick={() => void publishPose("initial")}>SET INITIAL POSE</button></div>
              <div className={`mapping-pose-tool goal ${poseTool === "goal" ? "active" : ""}`}><div className="pose-tool-heading"><span>SET GOAL POSE</span><button type="button" onClick={() => setPoseTool(poseTool === "goal" ? null : "goal")}>{poseTool === "goal" ? "MAP ACTIVE" : "PICK MAP"}</button></div><p>Goal Nav2 · drag di map untuk arah.</p><PoseDraftFields draft={goalDraft} onChange={(field, value) => updatePose("goal", field, value)} /><button className="primary-action" type="button" onClick={() => void publishPose("goal")}>SEND NAV2 GOAL</button></div>
              <div className="map-instructions"><Icon name="target" size={15} /><span>{poseTool ? `Klik-drag map untuk memilih ${poseTool === "initial" ? "initial pose" : "goal pose"}.` : "Pilih PICK MAP pada panel pose."}</span></div>
            </div>
          </div>
          <div className="map-layer-bar"><span>RESOLUTION: {displayMap ? `${fmt(displayMap.resolution, 3)} m` : "—"}</span><span>SIZE: {displayMap ? `${displayMap.width} × ${displayMap.height}` : "—"}</span><label><input type="checkbox" checked={showRobot} onChange={(event) => setShowRobot(event.target.checked)} /> ROBOT</label><label><input type="checkbox" checked={showLidar} onChange={(event) => setShowLidar(event.target.checked)} /> LIDAR SCAN</label><label><input type="checkbox" checked={showLocalCostmap} disabled={!state.local_costmap} onChange={(event) => setShowLocalCostmap(event.target.checked)} /> LOCAL COSTMAP</label><label><input type="checkbox" checked={showGlobalCostmap} disabled={!state.global_costmap} onChange={(event) => setShowGlobalCostmap(event.target.checked)} /> GLOBAL COSTMAP</label><span className="map-source-status">LIDAR: {showLidar ? state.scan ? "LIVE" : "WAITING" : "OFF"} · {selectedMap ? `SELECTED: ${selectedMap.name || mapChoice}` : previewMap ? `PREVIEW: ${previewMap.name || mapChoice}` : "/map → OccupancyGrid"}</span></div>
        </Panel>
      </>}

      {panels.controls && <Panel title="Mapping Controls" eyebrow="HARDWARE // SLAM // TELEOP" accent="cyan">
        <div className="hardware-status-card mapping-hardware-card">
          <div><span>HARDWARE GATE</span><strong className={mappingHardwareReady || hardware.ready ? "hardware-status-ready" : hardware.starting ? "hardware-status-starting" : "hardware-status-idle"}>{mappingHardwareReady ? "MAPPING READY" : hardware.ready ? "TELEOP READY" : hardware.starting ? "STARTING" : "LOCKED"}</strong></div>
          <p>{hardware.message}</p>
          <div className="hardware-components">
            {hardware.components.map((component) => <div className={`hardware-component state-${component.state}`} key={component.id}><i /><span>{component.label}</span><b>{componentLabel(component)}</b></div>)}
          </div>
          <LaunchToggleButton component="hardware" state={state} hardware={hardware} sendCommand={sendCommand} onEvent={onEvent} compact />
        </div>

        <div className="mapping-control-grid">
          <div className="mapping-action-card">
            <div className="mapping-action-heading"><span>OCCUPANCY GRID MAPPING</span><strong className={`status-${state.mapping.active ? "ready" : state.mapping.state === "error" ? "error" : "idle"}`}>{mappingState}</strong></div>
            <p>{state.mapping.message}</p>
            <LaunchToggleButton component="mapping" state={state} hardware={hardware} sendCommand={sendCommand} onEvent={onEvent} compact />
            <div className="mapping-action-buttons legacy-mapping-buttons"><button className="primary-action" type="button" disabled={!mappingHardwareReady || state.mapping.active} onClick={() => void requestMapping()}>START MAPPING</button><button className="danger-action" type="button" disabled={!state.mapping.active} onClick={() => void sendCommand({ type: "stop_mapping" })}>STOP MAPPING</button></div>
          </div>
          <div className="mapping-action-card save-map-card">
            <div className="mapping-action-heading"><span>SAVE MAP</span><strong>PGM + YAML</strong></div>
            <p>File disimpan ke <code>diablo_bringup/map</code>.</p>
            <div className="save-map-row"><input aria-label="Map name" value={mapName} onChange={(event) => setMapName(event.target.value)} placeholder="museum_hall_01" maxLength={64} /><button className="primary-action" type="button" disabled={!state.mapping.active} onClick={() => void saveMap()}>SAVE</button></div>
          </div>
        </div>
      </Panel>}

      {panels.controls && <Panel title="Teleoperasi Mapping" eyebrow="W A S D // HOLD TO MOVE" accent="orange">
        <div className="mapping-teleop-layout">
          <div className="mapping-keypad" aria-label="Mapping teleoperation">
            <button className={activeKeys.has("w") ? "active" : ""} type="button" onPointerDown={() => { keysRef.current.add("w"); setActiveKeys(new Set(keysRef.current)); sendMotion(); }} onPointerUp={stopTeleop} onPointerLeave={stopTeleop}>W<span>MAJU</span></button>
            <button className={activeKeys.has("a") ? "active" : ""} type="button" onPointerDown={() => { keysRef.current.add("a"); setActiveKeys(new Set(keysRef.current)); sendMotion(); }} onPointerUp={stopTeleop} onPointerLeave={stopTeleop}>A<span>KIRI</span></button>
            <button className="stop" type="button" onClick={stopTeleop}>STOP</button>
            <button className={activeKeys.has("d") ? "active" : ""} type="button" onPointerDown={() => { keysRef.current.add("d"); setActiveKeys(new Set(keysRef.current)); sendMotion(); }} onPointerUp={stopTeleop} onPointerLeave={stopTeleop}>D<span>KANAN</span></button>
            <button className={activeKeys.has("s") ? "active" : ""} type="button" onPointerDown={() => { keysRef.current.add("s"); setActiveKeys(new Set(keysRef.current)); sendMotion(); }} onPointerUp={stopTeleop} onPointerLeave={stopTeleop}>S<span>MUNDUR</span></button>
          </div>
          <div className="mapping-teleop-settings">
            <label><span>FORWARD SPEED</span><b>{forwardSpeed.toFixed(2)} m/s</b><input type="range" min="0.05" max="0.35" step="0.01" value={forwardSpeed} onChange={(event) => setForwardSpeed(Number(event.target.value))} /></label>
            <label><span>TURN SPEED</span><b>{turnSpeed.toFixed(2)} command</b><input type="range" min="0.10" max="0.70" step="0.01" value={turnSpeed} onChange={(event) => setTurnSpeed(Number(event.target.value))} /></label>
            <div className={`teleop-lock ${teleopReady ? "unlocked" : ""}`}>{teleopReady ? "TELEOP UNLOCKED · MAPPING OPTIONAL" : "START HARDWARE TO UNLOCK TELEOP"}</div>
          </div>
        </div>
      </Panel>}

      {panels.log && <Panel title="Mapping Log" eyebrow="SESSION STATUS" accent="slate"><div className="mapping-session-note">Mapping memakai SLAM Toolbox dengan occupancy grid default. Simpan setelah area selesai dipindai; file `.pgm` dan `.yaml` dibuat bersamaan.</div></Panel>}
    </div>
  );
}
