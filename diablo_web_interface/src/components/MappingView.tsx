import { useCallback, useEffect, useRef, useState } from "react";
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

interface OccupancyCanvasProps {
  grid: OccupancyGrid | null;
  pose: Pose | null;
  scan: DiabloState["scan"];
}

function OccupancyCanvas({ grid, pose, scan }: OccupancyCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 1, height: 1 });

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

    if (scan && pose) {
      context.fillStyle = "rgba(36, 126, 164, .62)";
      scan.ranges.forEach((range, index) => {
        if (range === null || range < scan.range_min || range > scan.range_max) return;
        const angle = pose.theta + scan.angle_min + index * scan.angle_increment;
        const [x, y] = toCanvas(
          pose.x + range * Math.cos(angle),
          pose.y + range * Math.sin(angle),
        );
        context.fillRect(x - 1, y - 1, 2.5, 2.5);
      });
    }

    if (pose) {
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
  }, [grid, pose, scan, size]);

  if (!grid) {
    return <EmptyState title="Menunggu occupancy grid" detail="Nyalakan hardware, lalu mulai mapping untuk mengisi /map." />;
  }
  return <canvas ref={canvasRef} className="mapping-canvas" aria-label="Live occupancy grid map" />;
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

  const mappingActive = state.mapping.active;
  const allHardwareReady = hardware.all_ready;

  const sendMotion = useCallback(() => {
    if (!mappingActive || !allHardwareReady) return;
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
  }, [allHardwareReady, forwardSpeed, mappingActive, sendCommand, turnSpeed]);

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
    if (!mappingActive || !allHardwareReady) stopTeleop();
  }, [allHardwareReady, mappingActive, stopTeleop]);

  const requestHardware = async () => {
    const accepted = await sendCommand({ type: "start_hardware" });
    onEvent(accepted ? "Hardware startup requested: Diablo, LiDAR and Dynamixel." : "Hardware startup request failed.", accepted ? "success" : "error");
  };

  const requestMapping = async () => {
    if (!allHardwareReady) {
      onEvent("Mapping dikunci: tunggu feedback motor, LiDAR dan Dynamixel READY.", "warn");
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
      {panels.map && <Panel title="Live Occupancy Grid" eyebrow="SLAM TOOLBOX // /MAP" accent="blue" actions={<span className="panel-chip">FRAME: {state.map?.frame_id || "—"}</span>}>
        <div className="mapping-map-layout">
          <div className="mapping-map-stage">
            <OccupancyCanvas grid={state.map} pose={state.pose} scan={state.scan} />
            <div className="map-legend"><span><i className="legend-dot green" /> Diablo</span><span><i className="legend-dot cyan" /> LiDAR</span><span><i className="legend-dot slate" /> Unknown</span></div>
          </div>
          <div className="map-readouts">
            <StatCard label="ROBOT X" value={fmt(state.pose?.x)} unit="METERS · ODOM" tone="green" />
            <StatCard label="ROBOT Y" value={fmt(state.pose?.y)} unit="METERS · ODOM" tone="blue" />
            <StatCard label="HEADING θ" value={degrees(state.pose?.theta)} unit="DEGREES" tone="orange" />
            <div className="map-instructions"><Icon name="target" size={17} /><span>Gerakkan robot perlahan dengan W/A/S/D. Grid diperbarui dari topic /map.</span></div>
          </div>
        </div>
        <div className="map-layer-bar"><span>RESOLUTION: {state.map ? `${fmt(state.map.resolution, 3)} m` : "—"}</span><span>SIZE: {state.map ? `${state.map.width} × ${state.map.height}` : "—"}</span><span className="map-source-status">/map → OccupancyGrid</span></div>
      </Panel>}

      {panels.controls && <Panel title="Mapping Controls" eyebrow="HARDWARE // SLAM // TELEOP" accent="cyan">
        <div className="hardware-status-card mapping-hardware-card">
          <div><span>HARDWARE GATE</span><strong className={hardware.all_ready ? "hardware-status-ready" : hardware.starting ? "hardware-status-starting" : "hardware-status-idle"}>{hardware.all_ready ? "ALL READY" : hardware.starting ? "STARTING" : "LOCKED"}</strong></div>
          <p>{hardware.message}</p>
          <div className="hardware-components">
            {hardware.components.map((component) => <div className={`hardware-component state-${component.state}`} key={component.id}><i /><span>{component.label}</span><b>{componentLabel(component)}</b></div>)}
          </div>
          <button className="primary-action mapping-wide-button" type="button" disabled={hardware.all_ready || hardware.starting} onClick={() => void requestHardware()}>{hardware.all_ready ? "HARDWARE READY" : hardware.starting ? "STARTING…" : "ON HARDWARE"}</button>
        </div>

        <div className="mapping-control-grid">
          <div className="mapping-action-card">
            <div className="mapping-action-heading"><span>OCCUPANCY GRID MAPPING</span><strong className={`status-${state.mapping.active ? "ready" : state.mapping.state === "error" ? "error" : "idle"}`}>{mappingState}</strong></div>
            <p>{state.mapping.message}</p>
            <div className="mapping-action-buttons"><button className="primary-action" type="button" disabled={!hardware.all_ready || state.mapping.active} onClick={() => void requestMapping()}>START MAPPING</button><button className="danger-action" type="button" disabled={!state.mapping.active} onClick={() => void sendCommand({ type: "stop_mapping" })}>STOP MAPPING</button></div>
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
            <div className={`teleop-lock ${mappingActive && allHardwareReady ? "unlocked" : ""}`}>{mappingActive && allHardwareReady ? "TELEOP UNLOCKED" : "START HARDWARE + MAPPING TO UNLOCK"}</div>
          </div>
        </div>
      </Panel>}

      {panels.log && <Panel title="Mapping Log" eyebrow="SESSION STATUS" accent="slate"><div className="mapping-session-note">Mapping memakai SLAM Toolbox dengan occupancy grid default. Simpan setelah area selesai dipindai; file `.pgm` dan `.yaml` dibuat bersamaan.</div></Panel>}
    </div>
  );
}
