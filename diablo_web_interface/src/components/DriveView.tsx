import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Icon } from "./Icon";
import { Panel, StatCard } from "./Panel";
import type {
  DiabloState,
  EventEntry,
  MotionCommand,
  PanelKey,
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
    : ((value * 180) / Math.PI).toFixed(3);
}

type KeybindId =
  | "forward"
  | "reverse"
  | "left"
  | "right"
  | "rollLeft"
  | "rollRight"
  | "rollLevel"
  | "stand"
  | "crawl"
  | "heightLow"
  | "heightMid"
  | "heightHigh"
  | "heightZero"
  | "pitchUp"
  | "pitchLevel"
  | "pitchDown"
  | "heightMode"
  | "heightDirect"
  | "pitchMode"
  | "pitchDirect"
  | "jump"
  | "danceOn"
  | "danceOff";

interface Keybind {
  id: KeybindId;
  key: string;
  label: string;
  detail: string;
  kind: "hold" | "action";
}

const DEFAULT_KEYBINDS: Keybind[] = [
  { id: "forward", key: "w", label: "FORWARD", detail: "Move forward", kind: "hold" },
  { id: "reverse", key: "s", label: "REVERSE", detail: "Move backward", kind: "hold" },
  { id: "left", key: "a", label: "TURN LEFT", detail: "Rotate left", kind: "hold" },
  { id: "right", key: "d", label: "TURN RIGHT", detail: "Rotate right", kind: "hold" },
  { id: "rollLeft", key: "q", label: "ROLL LEFT", detail: "Tilt left", kind: "hold" },
  { id: "rollRight", key: "e", label: "ROLL RIGHT", detail: "Tilt right", kind: "hold" },
  { id: "rollLevel", key: "r", label: "LEVEL ROLL", detail: "Set roll to zero", kind: "action" },
  { id: "stand", key: "z", label: "STANDING MODE", detail: "Stand up", kind: "action" },
  { id: "crawl", key: "x", label: "CRAWLING MODE", detail: "Fold down", kind: "action" },
  { id: "heightLow", key: "h", label: "HEIGHT LOW", detail: "Minimum standing height", kind: "action" },
  { id: "heightMid", key: "k", label: "HEIGHT MID", detail: "Medium standing height", kind: "action" },
  { id: "heightHigh", key: "j", label: "HEIGHT HIGH", detail: "Maximum standing height", kind: "action" },
  { id: "heightZero", key: "l", label: "HEIGHT ZERO", detail: "Set height command to zero", kind: "action" },
  { id: "pitchUp", key: "u", label: "LOOK UP", detail: "Body pitch up", kind: "action" },
  { id: "pitchLevel", key: "i", label: "LEVEL PITCH", detail: "Set pitch to zero", kind: "action" },
  { id: "pitchDown", key: "o", label: "LOOK DOWN", detail: "Body pitch down", kind: "action" },
  { id: "heightMode", key: "v", label: "HEIGHT POSITION", detail: "Enable height control", kind: "action" },
  { id: "heightDirect", key: "b", label: "HEIGHT DIRECT", detail: "Disable height control", kind: "action" },
  { id: "pitchMode", key: "n", label: "PITCH POSITION", detail: "Enable pitch control", kind: "action" },
  { id: "pitchDirect", key: "m", label: "PITCH DIRECT", detail: "Disable pitch control", kind: "action" },
  { id: "jump", key: "c", label: "JUMP MODE", detail: "Advanced action", kind: "action" },
  { id: "danceOn", key: "f", label: "MOONWALK ON", detail: "Dance mode", kind: "action" },
  { id: "danceOff", key: "g", label: "MOONWALK OFF", detail: "Stop dance mode", kind: "action" },
];

const HOLD_IDS = new Set<KeybindId>([
  "forward",
  "reverse",
  "left",
  "right",
  "rollLeft",
  "rollRight",
]);

const QUICK_ACTION_IDS = new Set<KeybindId>([
  "rollLevel",
  "stand",
  "crawl",
  "heightLow",
  "heightMid",
  "heightHigh",
  "pitchLevel",
]);

function loadKeybinds(): Keybind[] {
  if (typeof window === "undefined") return DEFAULT_KEYBINDS.map((item) => ({ ...item }));
  try {
    const stored = JSON.parse(window.localStorage.getItem("diablo-hmi-keybinds") || "null") as unknown;
    if (!Array.isArray(stored)) return DEFAULT_KEYBINDS.map((item) => ({ ...item }));
    const records = stored as Array<{ id?: unknown; key?: unknown }>;
    const keys = new Map(records.map((item) => [String(item.id), String(item.key || "").toLowerCase()]));
    return DEFAULT_KEYBINDS.map((item) => ({ ...item, key: keys.get(item.id) || item.key }));
  } catch {
    return DEFAULT_KEYBINDS.map((item) => ({ ...item }));
  }
}

function normalizeKey(value: string) {
  if (value === " ") return "space";
  return value.toLowerCase();
}

function HoldButton({
  label,
  hotkey,
  onPress,
  onRelease,
  className = "",
  disabled = false,
}: {
  label: string;
  hotkey: string;
  onPress: () => void;
  onRelease: () => void;
  className?: string;
  disabled?: boolean;
}) {
  return (
    <button
      className={`drive-hold-button ${className}`}
      type="button"
      disabled={disabled}
      onPointerDown={(event) => {
        event.preventDefault();
        onPress();
      }}
      onPointerUp={(event) => {
        event.preventDefault();
        onRelease();
      }}
      onPointerLeave={onRelease}
      onPointerCancel={onRelease}
    >
      <strong>{label}</strong>
      <kbd>{hotkey.toUpperCase()}</kbd>
    </button>
  );
}

function DirectionPad({
  keyFor,
  onCommand,
  onStop,
  activeKeys,
  enabled,
}: {
  keyFor: (id: KeybindId) => string;
  onCommand: (id: KeybindId, active: boolean) => void;
  onStop: () => void;
  activeKeys: Set<KeybindId>;
  enabled: boolean;
}) {
  return (
    <div className="direction-zone">
      <div className="direction-pad" aria-label="Diablo direction controls">
        <div />
        <HoldButton label="FORWARD" hotkey={keyFor("forward")} onPress={() => onCommand("forward", true)} onRelease={() => onCommand("forward", false)} disabled={!enabled} className={`forward ${activeKeys.has("forward") ? "active" : ""}`} />
        <div />
        <HoldButton label="LEFT" hotkey={keyFor("left")} onPress={() => onCommand("left", true)} onRelease={() => onCommand("left", false)} disabled={!enabled} className={`turn ${activeKeys.has("left") ? "active" : ""}`} />
        <button className="pad-stop" type="button" onClick={onStop}>
          <Icon name="stop" size={14} /> STOP
        </button>
        <HoldButton label="RIGHT" hotkey={keyFor("right")} onPress={() => onCommand("right", true)} onRelease={() => onCommand("right", false)} disabled={!enabled} className={`turn ${activeKeys.has("right") ? "active" : ""}`} />
        <div />
        <HoldButton label="REVERSE" hotkey={keyFor("reverse")} onPress={() => onCommand("reverse", true)} onRelease={() => onCommand("reverse", false)} disabled={!enabled} className={`reverse ${activeKeys.has("reverse") ? "active" : ""}`} />
        <div />
      </div>
      <div className="drive-key-status">
        <span>HOLD KEYS</span>
        {["forward", "reverse", "left", "right", "rollLeft", "rollRight"].map((id) => (
          <kbd className={activeKeys.has(id as KeybindId) ? "active" : ""} key={id}>{keyFor(id as KeybindId).toUpperCase()}</kbd>
        ))}
      </div>
    </div>
  );
}

function WheelTelemetry({ motors }: { motors: DiabloState["telemetry"]["motors"] }) {
  return (
    <div className="wheel-telemetry">
      <div className="wheel-row wheel-head"><span>CHANNEL</span><span>POSITION</span><span>REV</span><span>VELOCITY</span></div>
      <div className="wheel-row"><span>LEFT WHEEL</span><b>{fmt(motors?.left_wheel.position)}</b><b>{motors ? motors.left_wheel.revolutions : "—"}</b><b>{fmt(motors?.left_wheel.velocity)} rad/s</b></div>
      <div className="wheel-row"><span>RIGHT WHEEL</span><b>{fmt(motors?.right_wheel.position)}</b><b>{motors ? motors.right_wheel.revolutions : "—"}</b><b>{fmt(motors?.right_wheel.velocity)} rad/s</b></div>
      <div className="wheel-height"><span>LEG LENGTH</span><b>{fmt(motors?.left_leg_length, 3)} / {fmt(motors?.right_leg_length, 3)} m</b></div>
    </div>
  );
}

function TrajectoryMiniMap({ state }: { state: DiabloState }) {
  const map = state.map;
  const trajectory = state.wheel_trajectory || [];
  const current = state.wheel_pose;
  const worldWidth = map ? map.width * map.resolution : 4;
  const worldHeight = map ? map.height * map.resolution : 3;
  const origin = map?.origin || { x: -1, y: -1, yaw: 0 };
  const point = (x: number, y: number) => {
    const px = ((x - origin.x) / worldWidth) * 420;
    const py = 210 - ((y - origin.y) / worldHeight) * 210;
    return `${px.toFixed(1)},${py.toFixed(1)}`;
  };
  const currentPoint = current ? point(current.x, current.y).split(",") : null;

  return (
    <div className="trajectory-graphic">
      <svg viewBox="0 0 420 210" role="img" aria-label="Wheel odometry trajectory map">
        <defs><pattern id="wheel-odom-grid" width="21" height="21" patternUnits="userSpaceOnUse"><path d="M 21 0 L 0 0 0 21" fill="none" stroke="currentColor" strokeOpacity=".12" strokeWidth=".6" /></pattern></defs>
        <rect width="420" height="210" fill="url(#wheel-odom-grid)" />
        <line x1="210" y1="0" x2="210" y2="210" className="axis-line" />
        <line x1="0" y1="105" x2="420" y2="105" className="axis-line" />
        {trajectory.length > 1 && <polyline points={trajectory.map((item) => point(item.x, item.y)).join(" ")} className="trajectory-line" />}
        {currentPoint && <circle cx={currentPoint[0]} cy={currentPoint[1]} r="5" className="trajectory-robot" />}
      </svg>
      <div className="trajectory-legend"><span><i className="legend-line blue" /> Fused wheel/IMU odometry</span><span><i className="legend-dot cyan" /> Current robot</span></div>
      <small className="trajectory-source">SOURCE: {current?.source || "WAITING FOR /ODOMETRY/FILTERED"}</small>
    </div>
  );
}

interface DriveViewProps {
  state: DiabloState;
  hardwareReady: boolean;
  panels: Record<PanelKey, boolean>;
  sendCommand: (command: SocketCommand) => Promise<boolean>;
  onEvent: (message: string, kind?: EventEntry["kind"]) => void;
}

export function DriveView({ state, hardwareReady, panels, sendCommand, onEvent }: DriveViewProps) {
  const [maxSpeed, setMaxSpeed] = useState(0.5);
  const [turnRate, setTurnRate] = useState(1.0);
  const [up, setUp] = useState(1.0);
  const [pitch, setPitch] = useState(0.0);
  const [activeKeys, setActiveKeys] = useState<Set<KeybindId>>(new Set());
  const [keybinds, setKeybinds] = useState<Keybind[]>(loadKeybinds);
  const [assigning, setAssigning] = useState<KeybindId | null>(null);
  const keysRef = useRef<Set<KeybindId>>(new Set());
  const activeRef = useRef(false);

  const bindingMap = useMemo(() => new Map(keybinds.map((item) => [item.id, item.key])), [keybinds]);
  const keyFor = useCallback((id: KeybindId) => bindingMap.get(id) || "?", [bindingMap]);

  useEffect(() => {
    try {
      window.localStorage.setItem("diablo-hmi-keybinds", JSON.stringify(keybinds.map(({ id, key }) => ({ id, key }))));
    } catch {
      // Keybind persistence is optional in locked-down browser sessions.
    }
  }, [keybinds]);

  const manualCommand = useCallback((overrides: Partial<Omit<MotionCommand, "type">> = {}): MotionCommand => ({
    type: "manual",
    forward: 0,
    left: 0,
    roll: 0,
    up,
    pitch,
    ...overrides,
  }), [pitch, up]);

  const commandFromActiveKeys = useCallback(() => {
    const keys = keysRef.current;
    return manualCommand({
      forward: keys.has("forward") ? maxSpeed : keys.has("reverse") ? -maxSpeed : 0,
      left: keys.has("left") ? turnRate : keys.has("right") ? -turnRate : 0,
      roll: keys.has("rollLeft") ? -0.1 : keys.has("rollRight") ? 0.1 : 0,
    });
  }, [manualCommand, maxSpeed, turnRate]);

  const sendCurrent = useCallback(() => {
    if (!hardwareReady) return;
    void sendCommand(commandFromActiveKeys());
  }, [commandFromActiveKeys, hardwareReady, sendCommand]);

  const releaseAll = useCallback(() => {
    keysRef.current.clear();
    setActiveKeys(new Set());
    activeRef.current = false;
    void sendCommand({ type: "stop" });
  }, [sendCommand]);

  useEffect(() => {
    if (!hardwareReady && keysRef.current.size) releaseAll();
  }, [hardwareReady, releaseAll]);

  const executeAction = useCallback(async (id: KeybindId) => {
    if (!hardwareReady) {
      onEvent("Start Hardware first; manual motion is locked.", "warn");
      return;
    }
    let command: SocketCommand | null = null;
    let message = "";
    switch (id) {
      case "rollLevel": command = manualCommand({ roll: 0 }); message = "Roll level command sent."; break;
      case "stand": command = { type: "stand", stand: true }; message = "Standing mode command sent."; break;
      case "crawl": command = { type: "stand", stand: false }; message = "Crawling mode command sent."; break;
      case "heightLow": command = manualCommand({ mode_mark: true, height_ctrl_mode: true, up: -0.5 }); message = "Minimum standing height selected."; break;
      case "heightMid": command = manualCommand({ mode_mark: true, height_ctrl_mode: true, up: 0.5 }); message = "Medium standing height selected."; break;
      case "heightHigh": command = manualCommand({ mode_mark: true, height_ctrl_mode: true, up: 1.0 }); message = "Maximum standing height selected."; break;
      case "heightZero": command = manualCommand({ up: 0.0 }); message = "Height command set to zero."; break;
      case "pitchUp": command = manualCommand({ pitch: 0.5 }); message = "Look-up command sent."; break;
      case "pitchLevel": command = manualCommand({ pitch: 0.0 }); message = "Pitch level command sent."; break;
      case "pitchDown": command = manualCommand({ pitch: -0.5 }); message = "Look-down command sent."; break;
      case "heightMode": command = manualCommand({ mode_mark: true, height_ctrl_mode: true }); message = "Height position mode enabled."; break;
      case "heightDirect": command = manualCommand({ mode_mark: true, height_ctrl_mode: false }); message = "Height direct mode enabled."; break;
      case "pitchMode": command = manualCommand({ mode_mark: true, pitch_ctrl_mode: true }); message = "Pitch position mode enabled."; break;
      case "pitchDirect": command = manualCommand({ mode_mark: true, pitch_ctrl_mode: false }); message = "Pitch direct mode enabled."; break;
      case "jump": command = manualCommand({ mode_mark: true, jump_mode: true }); message = "Jump mode command sent."; break;
      case "danceOn": command = manualCommand({ mode_mark: true, split_mode: true }); message = "Moonwalk mode enabled."; break;
      case "danceOff": command = manualCommand({ mode_mark: true, split_mode: false }); message = "Moonwalk mode disabled."; break;
      default: break;
    }
    if (!command) return;
    const accepted = await sendCommand(command);
    onEvent(accepted ? message : `${message} Backend unavailable.`, accepted ? "success" : "error");
  }, [hardwareReady, manualCommand, onEvent, sendCommand]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      const key = normalizeKey(event.key);
      if (assigning) {
        if (key === "escape") {
          setAssigning(null);
          return;
        }
        const candidate = key === " " ? "space" : key;
        if (!candidate) return;
        event.preventDefault();
        setKeybinds((previous) => previous.map((item) => item.id === assigning ? { ...item, key: candidate } : item));
        setAssigning(null);
        onEvent(`Keybind assigned: ${candidate.toUpperCase()}.`, "success");
        return;
      }
      if (["INPUT", "SELECT", "TEXTAREA"].includes((event.target as HTMLElement)?.tagName || "")) return;
      const binding = keybinds.find((item) => item.key === key);
      if (!binding) return;
      event.preventDefault();
      if (binding.kind === "hold") {
        if (!hardwareReady) {
          onEvent("Start Hardware first; manual motion is locked.", "warn");
          return;
        }
        if (keysRef.current.has(binding.id)) return;
        keysRef.current.add(binding.id);
        setActiveKeys(new Set(keysRef.current));
        activeRef.current = true;
        sendCurrent();
      } else if (!event.repeat) {
        void executeAction(binding.id);
      }
    };
    const onKeyUp = (event: KeyboardEvent) => {
      const key = normalizeKey(event.key);
      const binding = keybinds.find((item) => item.key === key);
      if (!binding || !HOLD_IDS.has(binding.id)) return;
      if (!keysRef.current.delete(binding.id)) return;
      setActiveKeys(new Set(keysRef.current));
      if (!keysRef.current.size) releaseAll(); else sendCurrent();
    };
    const onWindowBlur = () => releaseAll();
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onWindowBlur);
    const heartbeat = window.setInterval(() => {
      if (activeRef.current) sendCurrent();
    }, 50);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onWindowBlur);
      window.clearInterval(heartbeat);
    };
  }, [assigning, executeAction, hardwareReady, keybinds, onEvent, releaseAll, sendCurrent]);

  const onPadCommand = (id: KeybindId, active: boolean) => {
    if (active && !hardwareReady) {
      onEvent("Start Hardware first; manual motion is locked.", "warn");
      return;
    }
    if (id === "forward" && !active && !keysRef.current.size) {
      releaseAll();
      return;
    }
    if (active) {
      keysRef.current.add(id);
      activeRef.current = true;
      sendCurrent();
    } else {
      keysRef.current.delete(id);
      if (!keysRef.current.size) releaseAll(); else sendCurrent();
    }
    setActiveKeys(new Set(keysRef.current));
  };

  const updateBody = (field: "up" | "pitch", value: number) => {
    if (field === "up") setUp(value); else setPitch(value);
    if (hardwareReady) {
      void sendCommand(manualCommand(field === "up" ? { up: value } : { pitch: value }));
    }
  };

  const quickRequest = async (command: SocketCommand, message: string) => {
    const accepted = await sendCommand(command);
    onEvent(accepted ? message : `${message} Backend unavailable.`, accepted ? "success" : "warn");
  };

  const wheelPose = state.wheel_pose;
  const actionBindings = keybinds.filter((item) => QUICK_ACTION_IDS.has(item.id));

  return (
    <div className="view-stack drive-view">
      <div className="position-summary">
        <StatCard label="X POSITION" value={fmt(wheelPose?.x)} unit="METERS · FILTERED ODOM" tone="green" />
        <StatCard label="Y POSITION" value={fmt(wheelPose?.y)} unit="METERS · FILTERED ODOM" tone="blue" />
        <StatCard label="HEADING θ" value={fmtDegrees(wheelPose?.theta)} unit="DEGREES · FILTERED ODOM" tone="orange" />
        <div className="quick-actions">
          <span>QUICK ACTIONS</span>
          <button type="button" onClick={() => void quickRequest({ type: "reset_odom" }, "Reset odom requested.")}>RESET ODOM</button>
          <button type="button" onClick={() => void quickRequest({ type: "reset_encoder" }, "Reset encoder reference requested.")}>RESET ENCODER</button>
          <button type="button" onClick={() => void quickRequest({ type: "start_lidar" }, "Start LiDAR requested.")}>START LIDAR</button>
        </div>
      </div>

      <div className="drive-main-grid">
        {panels.motion && <Panel title="Motion Control" eyebrow="DIABLO // MANUAL TELEOP" accent="blue" actions={<span className={`panel-chip ${hardwareReady ? "hardware-ready-chip" : "hardware-locked-chip"}`}><i className={`dot ${hardwareReady ? "green" : "amber"}`} /> {hardwareReady ? "HARDWARE READY" : "START HARDWARE REQUIRED"}</span>}>
          <div className="motion-control-grid">
            <DirectionPad keyFor={keyFor} onCommand={onPadCommand} onStop={releaseAll} activeKeys={activeKeys} enabled={hardwareReady} />
            <div className="motion-sliders">
              <label className="hmi-range"><span>MAX SPEED <b>{maxSpeed.toFixed(2)} m/s</b></span><input type="range" min="0.05" max="1.6" step=".01" value={maxSpeed} onChange={(event) => setMaxSpeed(Number(event.target.value))} /></label>
              <label className="hmi-range"><span>MAX TURN RATE <b>{turnRate.toFixed(2)} rad/s</b></span><input type="range" min="0.1" max="5" step=".05" value={turnRate} onChange={(event) => setTurnRate(Number(event.target.value))} /></label>
              <label className="hmi-range"><span>BODY HEIGHT <b>{up.toFixed(2)}</b></span><input disabled={!hardwareReady} type="range" min="-0.5" max="1" step=".01" value={up} onChange={(event) => updateBody("up", Number(event.target.value))} /></label>
              <label className="hmi-range"><span>PITCH <b>{pitch.toFixed(2)} rad</b></span><input disabled={!hardwareReady} type="range" min="-.5" max=".5" step=".01" value={pitch} onChange={(event) => updateBody("pitch", Number(event.target.value))} /></label>
              <button className="release-button" type="button" onClick={releaseAll}><Icon name="stop" size={14} /> RELEASE / ZERO COMMAND</button>
            </div>
          </div>
          <div className="motion-action-row">
            {actionBindings.slice(0, 3).map((item) => <button disabled={!hardwareReady} type="button" key={item.id} onClick={() => void executeAction(item.id)}>{item.label}<kbd>{item.key.toUpperCase()}</kbd></button>)}
          </div>
          <p className="safety-copy">W/S, A/D, Q/E mengikuti teleop resmi Diablo. Perintah mode dikirim sebagai <code>MotionCtrl.mode_mark=true</code>; tekan STOP atau lepaskan tombol untuk command nol.</p>
        </Panel>}

        {panels.trajectory && <Panel title="Trajectory Map" eyebrow="FUSED ODOMETRY // /ODOMETRY/FILTERED" accent="cyan" actions={<span className="panel-chip">LIVE TRACE</span>}>
          <TrajectoryMiniMap state={state} />
        </Panel>}
      </div>

      <div className="drive-secondary-grid">
        {panels.telemetry && <Panel title="Wheel Encoder Telemetry" eyebrow="LEG MOTORS // ENCODER FEEDBACK" accent="slate" actions={<span className="panel-chip">/DIABLO/SENSOR/MOTORS</span>}>
          <WheelTelemetry motors={state.telemetry.motors} />
        </Panel>}
        {panels.motion && <Panel title="Keybind Legend" eyebrow="DIABLO TELEOP SHORTCUTS" accent="orange" actions={assigning ? <span className="panel-chip assign-active">PRESS A KEY · ESC CANCEL</span> : <span className="panel-chip">CLICK ASSIGN TO REMAP</span>}>
          <div className="keybind-grid">
            {keybinds.map((item) => <div className={`keybind-row ${item.kind}`} key={item.id}>
              <button disabled={!hardwareReady} className={`keybind-action ${assigning === item.id ? "assigning" : ""}`} type="button" onClick={() => void executeAction(item.id)}>{item.label}</button>
              <button className={`keycap ${assigning === item.id ? "assigning" : ""}`} type="button" onClick={() => setAssigning(assigning === item.id ? null : item.id)} title="Click then press a keyboard key">{assigning === item.id ? "…" : item.key.toUpperCase()}</button>
              <small>{item.detail}</small>
            </div>)}
          </div>
          <p className="keybind-note">Official defaults include <kbd>Z</kbd> STANDING MODE and <kbd>X</kbd> CRAWLING MODE. Remapping is saved in this browser only.</p>
        </Panel>}
      </div>
    </div>
  );
}
