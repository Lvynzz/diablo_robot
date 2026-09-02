import { useEffect, useRef, useState, type ChangeEvent, type PointerEvent } from "react";
import { Icon } from "./Icon";
import { EmptyState, Panel, StatCard } from "./Panel";
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

function fmtDegrees(value: number | null | undefined) {
  return value === null || value === undefined || !Number.isFinite(value)
    ? "—"
    : ((value * 180) / Math.PI).toFixed(1);
}

function radians(value: string) {
  const degrees = Number(value);
  return Number.isFinite(degrees) ? (degrees * Math.PI) / 180 : null;
}

interface PoseDraft {
  x: string;
  y: string;
  heading: string;
}

const emptyDraft: PoseDraft = { x: "0.00", y: "0.00", heading: "0.0" };

function poseFromDraft(draft: PoseDraft, source: string): Pose | null {
  const x = Number(draft.x);
  const y = Number(draft.y);
  const theta = radians(draft.heading);
  if (!Number.isFinite(x) || !Number.isFinite(y) || theta === null) return null;
  return { x, y, theta, source };
}

function draftFromPose(pose: Pose | null): PoseDraft {
  return pose
    ? { x: pose.x.toFixed(2), y: pose.y.toFixed(2), heading: ((pose.theta * 180) / Math.PI).toFixed(1) }
    : { ...emptyDraft };
}

function updateDraft(draft: PoseDraft, field: keyof PoseDraft, value: string): PoseDraft {
  return { ...draft, [field]: value };
}

function PoseFields({ draft, onChange }: { draft: PoseDraft; onChange: (field: keyof PoseDraft, value: string) => void }) {
  const fields: Array<[keyof PoseDraft, string, string]> = [
    ["x", "X (M)", "0.00"],
    ["y", "Y (M)", "0.00"],
    ["heading", "θ (DEG)", "0.0"],
  ];
  return (
    <div className="pose-coordinate-fields">
      {fields.map(([field, label, placeholder]) => (
        <label key={field}>
          <span>{label}</span>
          <input
            type="number"
            inputMode="decimal"
            step="0.01"
            value={draft[field]}
            placeholder={placeholder}
            onChange={(event) => onChange(field, event.target.value)}
          />
        </label>
      ))}
    </div>
  );
}

function componentStateLabel(state: HardwareStatus["components"][number]["state"]) {
  return state === "not_configured" ? "NOT CONFIGURED" : state.replace("_", " ").toUpperCase();
}

interface MapCanvasProps {
  grid: OccupancyGrid | null;
  pose: Pose | null;
  initialPose: Pose | null;
  path: DiabloState["path"];
  scan: DiabloState["scan"];
  goal: Pose | null;
  globalCostmap: OccupancyGrid | null;
  localCostmap: OccupancyGrid | null;
  showLidar: boolean;
  showPath: boolean;
  showGlobalCostmap: boolean;
  showLocalCostmap: boolean;
  showInflationLayer: boolean;
  onPick: (pose: Pose) => void;
}

function MapCanvas({
  grid,
  pose,
  initialPose,
  path,
  scan,
  goal,
  globalCostmap,
  localCostmap,
  showLidar,
  showPath,
  showGlobalCostmap,
  showLocalCostmap,
  showInflationLayer,
  onPick,
}: MapCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 1, height: 1 });

  useEffect(() => {
    const element = canvasRef.current;
    if (!element) return;
    const update = () => {
      const rect = element.getBoundingClientRect();
      setSize({ width: Math.max(1, rect.width), height: Math.max(1, rect.height) });
    };
    update();
    const observer = new ResizeObserver(update);
    observer.observe(element);
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

    const cell = Math.min((size.width - 28) / grid.width, (size.height - 28) / grid.height);
    const offsetX = (size.width - grid.width * cell) / 2;
    const offsetY = (size.height - grid.height * cell) / 2;
    const sample = Math.max(1, Math.ceil(Math.sqrt((grid.width * grid.height) / 150000)));

    for (let row = 0; row < grid.height; row += sample) {
      for (let column = 0; column < grid.width; column += sample) {
        const value = grid.data[row * grid.width + column] ?? -1;
        context.fillStyle = value < 0 ? "#e2e9ee" : value >= 65 ? "#42596a" : "#f8fbfd";
        context.fillRect(
          offsetX + column * cell,
          offsetY + (grid.height - row - sample) * cell,
          Math.ceil(cell * sample + 0.25),
          Math.ceil(cell * sample + 0.25),
        );
      }
    }

    const toCanvas = (x: number, y: number) => {
      const origin = grid.origin || { x: 0, y: 0, yaw: 0 };
      const dx = x - origin.x;
      const dy = y - origin.y;
      const angle = origin.yaw || 0;
      const gx = (Math.cos(angle) * dx + Math.sin(angle) * dy) / grid.resolution;
      const gy = (-Math.sin(angle) * dx + Math.cos(angle) * dy) / grid.resolution;
      return [offsetX + gx * cell, offsetY + (grid.height - gy) * cell];
    };

    const drawCostmap = (costmap: OccupancyGrid | null, layer: "global" | "local") => {
      if (!costmap) return;
      const overlaySample = Math.max(1, Math.ceil(Math.sqrt((costmap.width * costmap.height) / 65000)));
      for (let row = 0; row < costmap.height; row += overlaySample) {
        for (let column = 0; column < costmap.width; column += overlaySample) {
          const value = costmap.data[row * costmap.width + column] ?? -1;
          if (value < 1) continue;
          const inflated = value < 90;
          if (inflated && !showInflationLayer) continue;
          const origin = costmap.origin || { x: 0, y: 0, yaw: 0 };
          const angle = origin.yaw || 0;
          const worldX = origin.x + Math.cos(angle) * column * costmap.resolution - Math.sin(angle) * row * costmap.resolution;
          const worldY = origin.y + Math.sin(angle) * column * costmap.resolution + Math.cos(angle) * row * costmap.resolution;
          const [x, y] = toCanvas(worldX, worldY);
          const alpha = inflated ? 0.23 : 0.55;
          context.fillStyle = layer === "global"
            ? `rgba(47,120,174,${alpha})`
            : `rgba(197,83,0,${alpha})`;
          const sizeValue = Math.max(1, cell * costmap.resolution / grid.resolution * overlaySample + 0.5);
          context.fillRect(x, y - sizeValue, sizeValue, sizeValue);
        }
      }
    };

    if (showGlobalCostmap) drawCostmap(globalCostmap, "global");
    if (showLocalCostmap) drawCostmap(localCostmap, "local");

    if (showPath && path?.poses.length) {
      context.beginPath();
      context.strokeStyle = "#2f78ae";
      context.lineWidth = 2.5;
      path.poses.forEach((point, index) => {
        const [x, y] = toCanvas(point.x, point.y);
        if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
      });
      context.stroke();
    }

    if (showLidar && scan && pose) {
      context.fillStyle = "rgba(44,131,169,.62)";
      scan.ranges.forEach((range, index) => {
        if (range === null || range < scan.range_min || range > scan.range_max) return;
        const angle = pose.theta + scan.angle_min + index * scan.angle_increment;
        const [x, y] = toCanvas(pose.x + range * Math.cos(angle), pose.y + range * Math.sin(angle));
        context.fillRect(x - 1, y - 1, 2.5, 2.5);
      });
    }

    const marker = (markerPose: Pose, color: string, ring: boolean, sizeValue = 10) => {
      const [x, y] = toCanvas(markerPose.x, markerPose.y);
      context.save();
      context.translate(x, y);
      context.rotate(-markerPose.theta);
      context.fillStyle = color;
      context.strokeStyle = color;
      context.lineWidth = 2;
      if (ring) {
        context.beginPath();
        context.arc(0, 0, sizeValue, 0, Math.PI * 2);
        context.stroke();
      }
      context.beginPath();
      context.moveTo(sizeValue + 3, 0);
      context.lineTo(-sizeValue + 1, -sizeValue * .65);
      context.lineTo(-sizeValue + 2, sizeValue * .65);
      context.closePath();
      context.fill();
      context.restore();
    };
    if (initialPose) marker(initialPose, "#2c83a9", true, 8);
    if (goal) marker(goal, "#c55300", true, 10);
    if (pose) marker(pose, "#4f925c", false, 9);
  }, [globalCostmap, grid, goal, initialPose, localCostmap, path, pose, scan, showGlobalCostmap, showInflationLayer, showLidar, showLocalCostmap, showPath, size]);

  const onPointerDown = (event: PointerEvent<HTMLCanvasElement>) => {
    if (!grid) return;
    const rect = event.currentTarget.getBoundingClientRect();
    const cell = Math.min((rect.width - 28) / grid.width, (rect.height - 28) / grid.height);
    const offsetX = (rect.width - grid.width * cell) / 2;
    const offsetY = (rect.height - grid.height * cell) / 2;
    const gx = (event.clientX - rect.left - offsetX) / cell;
    const gy = grid.height - (event.clientY - rect.top - offsetY) / cell;
    const origin = grid.origin || { x: 0, y: 0, yaw: 0 };
    const localX = gx * grid.resolution;
    const localY = gy * grid.resolution;
    onPick({
      x: origin.x + Math.cos(origin.yaw) * localX - Math.sin(origin.yaw) * localY,
      y: origin.y + Math.sin(origin.yaw) * localX + Math.cos(origin.yaw) * localY,
      theta: 0,
      source: "map click",
    });
  };

  return grid ? <canvas ref={canvasRef} className="nav-map-canvas" onPointerDown={onPointerDown} /> : <EmptyState title="Map data unavailable" detail="Start map_server or SLAM Toolbox to populate this view." />;
}

interface Station {
  name: string;
  x: number;
  y: number;
  theta: number;
}

interface NavigationViewProps {
  state: DiabloState;
  hardware: HardwareStatus;
  panels: Record<PanelKey, boolean>;
  sendCommand: (command: SocketCommand) => Promise<boolean>;
  events: EventEntry[];
  onEvent: (message: string, kind?: EventEntry["kind"]) => void;
}

function loadStations(): Station[] {
  if (typeof window === "undefined") return [];
  try {
    const value = JSON.parse(window.localStorage.getItem("diablo-hmi-stations") || "[]") as unknown;
    if (!Array.isArray(value)) return [];
    return value.filter((item): item is Station => Boolean(item) && typeof item === "object" && typeof (item as Station).name === "string" && Number.isFinite((item as Station).x) && Number.isFinite((item as Station).y) && Number.isFinite((item as Station).theta));
  } catch {
    return [];
  }
}

function statusTone(state: string) {
  if (state === "ready" || state === "succeeded") return "ready";
  if (state === "error" || state === "aborted") return "error";
  if (state === "starting" || state === "waiting" || state === "navigating") return "starting";
  return "idle";
}

export function NavigationView({ state, hardware, panels, sendCommand, events, onEvent }: NavigationViewProps) {
  const [goalDraft, setGoalDraft] = useState<PoseDraft>({ ...emptyDraft, x: "1.20", y: "0.60" });
  const [initialDraft, setInitialDraft] = useState<PoseDraft>(emptyDraft);
  const [stationDraft, setStationDraft] = useState<PoseDraft>({ ...emptyDraft, heading: "0.0" });
  const [stationName, setStationName] = useState("");
  const [stations, setStations] = useState<Station[]>(loadStations);
  const [tool, setTool] = useState<"goal" | "initial" | "station">("goal");
  const [showLidar, setShowLidar] = useState(true);
  const [showPath, setShowPath] = useState(true);
  const [showGlobalCostmap, setShowGlobalCostmap] = useState(true);
  const [showLocalCostmap, setShowLocalCostmap] = useState(true);
  const [showInflationLayer, setShowInflationLayer] = useState(true);
  const [mapChoices, setMapChoices] = useState<string[]>([]);
  const [selectedMap, setSelectedMap] = useState("");
  const [mapMessage, setMapMessage] = useState("Live /map topic");

  const mapGrid = state.map || state.global_costmap || state.local_costmap;
  const goal = poseFromDraft(goalDraft, "nav goal draft");
  const initialPose = poseFromDraft(initialDraft, "initial pose draft");
  const stationPose = poseFromDraft(stationDraft, "station draft");

  useEffect(() => {
    let active = true;
    fetch("/api/maps")
      .then((response) => response.ok ? response.json() : Promise.reject(new Error("map catalog unavailable")))
      .then((value: unknown) => {
        if (!active || !Array.isArray(value)) return;
        const names = value.map((item) => typeof item === "string" ? item : String((item as { name?: unknown }).name || "")).filter(Boolean);
        setMapChoices(names);
        if (names[0]) setSelectedMap(names[0]);
      })
      .catch(() => {
        if (!active) return;
        setMapChoices(["empty.yaml"]);
        setSelectedMap("empty.yaml");
      });
    return () => { active = false; };
  }, []);

  useEffect(() => {
    try {
      window.localStorage.setItem("diablo-hmi-stations", JSON.stringify(stations));
    } catch {
      // Station persistence is optional in locked-down browser sessions.
    }
  }, [stations]);

  const updateGoal = (field: keyof PoseDraft, value: string) => setGoalDraft((previous) => updateDraft(previous, field, value));
  const updateInitial = (field: keyof PoseDraft, value: string) => setInitialDraft((previous) => updateDraft(previous, field, value));
  const updateStation = (field: keyof PoseDraft, value: string) => setStationDraft((previous) => updateDraft(previous, field, value));

  const pickPoint = (picked: Pose) => {
    const target = tool === "goal" ? goalDraft : tool === "initial" ? initialDraft : stationDraft;
    const next = { x: picked.x.toFixed(2), y: picked.y.toFixed(2), heading: target.heading };
    if (tool === "goal") setGoalDraft(next);
    if (tool === "initial") setInitialDraft(next);
    if (tool === "station") setStationDraft(next);
    onEvent(`${tool === "goal" ? "Goal" : tool === "initial" ? "Initial pose" : "Station"} point selected at ${fmt(picked.x)}, ${fmt(picked.y)}.`, "info");
  };

  const sendGoal = async () => {
    if (!goal) {
      onEvent("Fill valid goal X, Y and heading coordinates first.", "warn");
      return;
    }
    const accepted = await sendCommand({ type: "goal_pose", x: goal.x, y: goal.y, theta: goal.theta });
    onEvent(accepted ? "Nav2 goal submitted." : "Nav2 goal could not be submitted.", accepted ? "success" : "error");
  };

  const setInitial = async () => {
    if (!initialPose) {
      onEvent("Fill valid initial pose X, Y and heading coordinates first.", "warn");
      return;
    }
    const accepted = await sendCommand({ type: "initial_pose", x: initialPose.x, y: initialPose.y, theta: initialPose.theta });
    onEvent(accepted ? "Initial pose published to AMCL." : "Initial pose could not be published.", accepted ? "success" : "error");
  };

  const addStation = () => {
    const name = stationName.trim();
    if (!name || !stationPose) {
      onEvent("Enter a station name and valid coordinates first.", "warn");
      return;
    }
    setStations((previous) => [...previous.filter((item) => item.name !== name), { name, x: stationPose.x, y: stationPose.y, theta: stationPose.theta }]);
    setStationName("");
    onEvent(`Station ${name} saved locally.`, "success");
  };

  const useStation = (station: Station) => {
    setGoalDraft(draftFromPose({ x: station.x, y: station.y, theta: station.theta }));
    setTool("goal");
    onEvent(`Station ${station.name} loaded as Nav2 goal draft.`, "info");
  };

  const runStartup = async (command: SocketCommand, successMessage: string) => {
    const accepted = await sendCommand(command);
    onEvent(accepted ? successMessage : `${successMessage} Command is not configured or backend is unavailable.`, accepted ? "success" : "warn");
  };

  const chooseMap = (event: ChangeEvent<HTMLSelectElement>) => {
    const name = event.target.value;
    setSelectedMap(name);
    setMapMessage(`${name} selected · restart map_server to apply`);
    onEvent(`Map ${name} selected in the HMI.`, "info");
  };

  const navEvents = events.filter((event) => /hardware|localization|amcl|nav2|navigation|mapping|goal|pose|costmap/i.test(event.message)).slice(-8).reverse();

  return (
    <div className="view-stack navigation-view">
      {panels.map && <Panel title="Navigation Map" eyebrow="NAV2 // PGM OCCUPANCY GRID" accent="blue" actions={<div className="navigation-map-actions"><span className="panel-chip">FRAME: {mapGrid?.frame_id || "—"}</span><label className="map-choice"><span>CHOOSE MAP</span><select value={selectedMap} onChange={chooseMap} aria-label="Choose map"><option value="">LIVE /MAP</option>{mapChoices.map((name) => <option key={name} value={name}>{name}</option>)}</select></label></div>}>
        <div className="map-layout navigation-map-layout">
          <div className="map-stage"><MapCanvas grid={mapGrid} pose={state.pose} initialPose={initialPose} path={state.path} scan={state.scan} goal={goal} globalCostmap={state.global_costmap} localCostmap={state.local_costmap} showLidar={showLidar} showPath={showPath} showGlobalCostmap={showGlobalCostmap} showLocalCostmap={showLocalCostmap} showInflationLayer={showInflationLayer} onPick={pickPoint} /><div className="map-legend"><span><i className="legend-dot green" /> Diablo</span><span><i className="legend-dot cyan" /> Init pose</span><span><i className="legend-dot orange" /> Goal</span><span><i className="legend-line blue" /> Nav2 path</span></div></div>
          <div className="map-readouts"><StatCard label="ROBOT X" value={fmt(state.pose?.x)} unit="METERS · MAP POSE" tone="green" /><StatCard label="ROBOT Y" value={fmt(state.pose?.y)} unit="METERS · MAP POSE" tone="blue" /><StatCard label="HEADING θ" value={fmtDegrees(state.pose?.theta)} unit="DEGREES" tone="orange" /><div className="map-instructions"><Icon name="target" size={17} /><span>Choose a tool below, then click the map to place an initial pose, goal, or station.</span></div></div>
        </div>
        <div className="map-layer-bar"><label><input type="checkbox" checked={showLidar} onChange={(event) => setShowLidar(event.target.checked)} /> LiDAR</label><label><input type="checkbox" checked={showPath} onChange={(event) => setShowPath(event.target.checked)} /> NAV2 PATH</label><span className="map-source-status">{mapMessage}</span></div>
      </Panel>}

      {panels.poses && <Panel title="Pose & Stations" eyebrow="INITIAL LOCALIZATION // GOAL REGISTRY" accent="cyan">
        <div className="robot-status-strip"><span>ROBOT STATUS</span><strong className={`status-${statusTone(state.nav_goal.state)}`}>{state.nav_goal.state.toUpperCase()}</strong></div>
        <div className="pose-station-grid">
          <div className="pose-editor initial-editor"><div className="pose-editor-heading"><span>SET INIT POSE</span><button className={tool === "initial" ? "tool-active" : ""} type="button" onClick={() => setTool("initial")}><Icon name="pin" size={12} /> {tool === "initial" ? "MAP TOOL ACTIVE" : "SET INIT POSE"}</button></div><PoseFields draft={initialDraft} onChange={updateInitial} /><div className="pose-editor-actions"><button className="primary-action" type="button" onClick={() => void setInitial()}>APPLY INITIAL</button><button type="button" onClick={() => setTool("initial")}>PICK FROM MAP</button></div></div>
          <div className="pose-editor goal-editor"><div className="pose-editor-heading"><span>SET GOAL POSE</span><button className={tool === "goal" ? "tool-active" : ""} type="button" onClick={() => setTool("goal")}><Icon name="target" size={12} /> {tool === "goal" ? "MAP TOOL ACTIVE" : "SET GOAL POSE"}</button></div><PoseFields draft={goalDraft} onChange={updateGoal} /><div className="pose-editor-actions"><button className="primary-action" type="button" onClick={() => void sendGoal()}>SEND GOAL</button><button type="button" onClick={() => setTool("goal")}>PICK FROM MAP</button></div></div>
          <div className="station-editor"><div className="pose-editor-heading"><span>STATION REGISTRY</span><small>LOCAL BROWSER STORAGE</small></div><label className="station-name"><span>NAME</span><input value={stationName} onChange={(event) => setStationName(event.target.value)} placeholder="station_01" /></label><PoseFields draft={stationDraft} onChange={updateStation} /><div className="pose-editor-actions"><button className="primary-action" type="button" onClick={addStation}><Icon name="plus" size={12} /> SAVE STATION</button><button type="button" onClick={() => setTool("station")}>{tool === "station" ? "MAP TOOL ACTIVE" : "PICK FROM MAP"}</button></div><div className="station-list">{stations.length ? stations.map((station) => <div className="station-row" key={station.name}><button type="button" onClick={() => useStation(station)}><Icon name="pin" size={12} /><span>{station.name}</span><small>{station.x.toFixed(2)}, {station.y.toFixed(2)}</small></button></div>) : <span>Belum ada station.</span>}</div></div>
        </div>
      </Panel>}

      {panels.controls && <Panel title="Navigation Controls" eyebrow="ROS LAUNCH CONTROL" accent="orange" actions={<span className={`panel-chip ${hardware.ready ? "hardware-ready-chip" : "hardware-locked-chip"}`}><i className={`dot ${hardware.ready ? "green" : "amber"}`} /> {hardware.ready ? "HARDWARE READY" : hardware.starting ? "STARTING" : "IDLE"}</span>}>
        <div className="hardware-status-card"><div><span>HARDWARE GATE</span><strong className={`hardware-status-${hardware.ready ? "ready" : hardware.starting ? "starting" : "idle"}`}>{hardware.ready ? "READY" : hardware.starting ? "STARTING" : "IDLE"}</strong></div><p>{hardware.message}</p><div className="hardware-components">{hardware.components.map((component) => <span className={`hardware-component state-${component.state}`} key={component.id}><i />{component.label}<b>{componentStateLabel(component.state)}</b></span>)}</div></div>
        <div className="nav-start-grid">
          <div className="nav-start-card hardware-start-card"><div className="nav-start-card-heading"><span>HARDWARE</span><b>DIABLO + LIDAR + U2D2</b></div><small>Start Diablo ROS2 driver, configured LiDAR and Dynamixel U2D2 launch.</small><button className="primary-action" disabled={hardware.ready || hardware.starting} type="button" onClick={() => void runStartup({ type: "start_hardware" }, "Hardware startup requested.")}>{hardware.starting ? "STARTING HARDWARE…" : hardware.ready ? "HARDWARE READY" : "START HARDWARE"}</button></div>
          <div className="nav-start-card"><div className="nav-start-card-heading"><span>LOCALIZATION</span><b>AMCL</b></div><small>Start the configured AMCL/localization command.</small><button type="button" onClick={() => void runStartup({ type: "start_localization" }, "Localization startup requested.")}>START AMCL LOCAL</button></div>
          <div className="nav-start-card"><div className="nav-start-card-heading"><span>NAVIGATION</span><b>NAV2</b></div><small>Start the configured Nav2 navigation stack.</small><button type="button" onClick={() => void runStartup({ type: "start_navigation" }, "Nav2 startup requested.")}>START NAV2 (PURE)</button></div>
          <div className="nav-start-card"><div className="nav-start-card-heading"><span>MAPPING</span><b>SLAM TOOLBOX</b></div><small>Start the configured mapping command for a new map.</small><button type="button" onClick={() => void runStartup({ type: "start_mapping" }, "Mapping startup requested.")}>START NEW MAP</button></div>
        </div>
        <div className="nav-goal-status"><span>NAV2 ACTION STATUS</span><strong>{state.nav_goal.state.toUpperCase()}</strong><small>{state.nav_goal.message}{state.nav_goal.distance_remaining !== null ? ` · ${fmt(state.nav_goal.distance_remaining)} m remaining` : ""}</small><button className="danger-action" type="button" onClick={() => void runStartup({ type: "cancel_goal" }, "Navigation cancel requested.")}>CANCEL GOAL</button></div>
      </Panel>}

      {panels.costmaps && <Panel title="Costmap Layers" eyebrow="NAV2 // LIVE COSTMAP OVERLAY" accent="red" actions={<span className="panel-chip">MAP LAYERS</span>}>
        <div className="costmap-layer-grid"><label className="layer-toggle"><input type="checkbox" checked={showGlobalCostmap} disabled={!state.global_costmap} onChange={(event) => setShowGlobalCostmap(event.target.checked)} /><span className="layer-swatch global" /><b>GLOBAL COSTMAP</b><small className={state.global_costmap ? "data-ready" : "data-waiting"}>{state.global_costmap ? `${state.global_costmap.width}×${state.global_costmap.height} RECEIVING` : "WAITING FOR /GLOBAL_COSTMAP/COSTMAP"}</small></label><label className="layer-toggle"><input type="checkbox" checked={showLocalCostmap} disabled={!state.local_costmap} onChange={(event) => setShowLocalCostmap(event.target.checked)} /><span className="layer-swatch local" /><b>LOCAL COSTMAP</b><small className={state.local_costmap ? "data-ready" : "data-waiting"}>{state.local_costmap ? `${state.local_costmap.width}×${state.local_costmap.height} RECEIVING` : "WAITING FOR /LOCAL_COSTMAP/COSTMAP"}</small></label><label className="layer-toggle"><input type="checkbox" checked={showInflationLayer} onChange={(event) => setShowInflationLayer(event.target.checked)} /><span className="layer-swatch inflation" /><b>INFLATION LAYER</b><small>Show non-lethal obstacle cost values</small></label></div>
      </Panel>}

      {panels.log && <Panel title="Navigation Log History" eyebrow="LOCALIZATION // NAV2 FEEDBACK" accent="slate"><div className="nav-log-list">{navEvents.map((event) => <div className={`event-row ${event.kind}`} key={event.id}><time>{event.time}</time><span>{event.message}</span></div>)}{!navEvents.length && <div className="event-empty">No localization or Nav2 events yet.</div>}</div></Panel>}
    </div>
  );
}
