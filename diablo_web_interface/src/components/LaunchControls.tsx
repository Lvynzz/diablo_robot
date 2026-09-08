import { useMemo, useState } from "react";
import type { DiabloState, EventEntry, HardwareStatus, SocketCommand } from "../types";

export type LaunchComponent = "hardware" | "localization" | "navigation" | "mapping";

interface LaunchControlsProps {
  state: DiabloState;
  hardware: HardwareStatus;
  sendCommand: (command: SocketCommand) => Promise<boolean>;
  onEvent: (message: string, kind?: EventEntry["kind"]) => void;
}

interface LaunchToggleButtonProps extends LaunchControlsProps {
  component: LaunchComponent;
  compact?: boolean;
}

const LABELS: Record<LaunchComponent, { eyebrow: string; title: string; detail: string }> = {
  hardware: {
    eyebrow: "HARDWARE",
    title: "DIABLO + LIDAR + U2D2",
    detail: "Driver Diablo, LiDAR dan Dynamixel yang dijalankan oleh web interface.",
  },
  localization: {
    eyebrow: "LOCALIZATION",
    title: "AMCL / LOCALIZATION",
    detail: "Launch localization yang dikonfigurasi untuk robot ini.",
  },
  navigation: {
    eyebrow: "NAVIGATION",
    title: "NAV2",
    detail: "Launch stack navigasi yang dikonfigurasi untuk robot ini.",
  },
  mapping: {
    eyebrow: "MAPPING",
    title: "SLAM TOOLBOX",
    detail: "Buat occupancy grid baru dengan LiDAR dan odometri robot.",
  },
};

function activeFor(component: LaunchComponent, state: DiabloState, hardware: HardwareStatus) {
  if (component === "hardware") {
    return hardware.ready || hardware.starting || hardware.all_ready;
  }
  return component === "mapping"
    ? state.mapping.active || Boolean(state.processes?.mapping?.active)
    : Boolean(state.processes?.[component]?.active);
}

function statusFor(component: LaunchComponent, state: DiabloState, hardware: HardwareStatus) {
  if (component === "hardware") {
    if (hardware.all_ready) return "ALL READY";
    if (hardware.ready) return "READY";
    if (hardware.starting) return "STARTING";
    return "OFFLINE";
  }
  if (component === "mapping" && state.mapping.active) return "RUNNING";
  return String(state.processes?.[component]?.state || "idle").replace("_", " ").toUpperCase();
}

function startCommand(component: LaunchComponent): SocketCommand {
  if (component === "hardware") return { type: "start_hardware" };
  if (component === "localization") return { type: "start_localization" };
  if (component === "navigation") return { type: "start_navigation" };
  return { type: "start_mapping" };
}

function stopCommand(component: LaunchComponent): SocketCommand {
  if (component === "hardware") return { type: "stop_hardware" };
  if (component === "localization") return { type: "stop_localization" };
  if (component === "navigation") return { type: "stop_navigation" };
  return { type: "stop_mapping" };
}

function ConfirmModal({
  component,
  message,
  onCancel,
  onConfirm,
}: {
  component: LaunchComponent;
  message: string;
  onCancel: () => void;
  onConfirm: () => void;
}) {
  const label = LABELS[component].eyebrow;
  return (
    <div className="confirm-backdrop" role="presentation" onMouseDown={onCancel}>
      <section
        className="confirm-modal"
        role="dialog"
        aria-modal="true"
        aria-labelledby="confirm-modal-title"
        onMouseDown={(event) => event.stopPropagation()}
      >
        <span className="panel-eyebrow">CONFIRM COMPONENT STOP</span>
        <h2 id="confirm-modal-title">OFF {label}?</h2>
        <p>{message}</p>
        <div className="confirm-actions">
          <button type="button" onClick={onCancel}>TIDAK</button>
          <button className="danger-action" type="button" onClick={onConfirm}>IYA, STOP</button>
        </div>
      </section>
    </div>
  );
}

export function LaunchToggleButton({
  component,
  state,
  hardware,
  sendCommand,
  onEvent,
  compact = false,
}: LaunchToggleButtonProps) {
  const [confirming, setConfirming] = useState(false);
  const active = activeFor(component, state, hardware);
  const status = statusFor(component, state, hardware);
  const configured = component === "hardware"
    || state.processes?.[component]?.state !== "not_configured";
  const description = useMemo(() => {
    if (component !== "hardware" || !active) return `Status: ${status}.`;
    const dependent = (["mapping", "navigation", "localization"] as LaunchComponent[])
      .filter((name) => activeFor(name, state, hardware))
      .map((name) => LABELS[name].eyebrow);
    return dependent.length
      ? `OFF hardware akan menghentikan ${dependent.join(", ")} lebih dulu.`
      : "Command stop akan dikirim sebelum driver hardware dihentikan.";
  }, [active, component, hardware, state, status]);

  const run = async (command: SocketCommand, message: string) => {
    const accepted = await sendCommand(command);
    onEvent(message, accepted ? "success" : "error");
  };

  const request = () => {
    if (active) {
      setConfirming(true);
      return;
    }
    void run(startCommand(component), `${LABELS[component].eyebrow} startup requested.`);
  };

  const confirmStop = () => {
    setConfirming(false);
    void run(stopCommand(component), `${LABELS[component].eyebrow} stop requested.`);
  };

  return (
    <>
      <button
        className={`${compact ? "launch-toggle compact" : "primary-action launch-toggle"} ${active ? "is-active" : ""}`}
        type="button"
        onClick={request}
        disabled={!active && !configured}
      >
        <span>{active ? `OFF ${LABELS[component].eyebrow}` : `ON ${LABELS[component].eyebrow}`}</span>
        <b>{status}</b>
      </button>
      {confirming && (
        <ConfirmModal
          component={component}
          message={description}
          onCancel={() => setConfirming(false)}
          onConfirm={confirmStop}
        />
      )}
    </>
  );
}

export function LaunchControls({ state, hardware, sendCommand, onEvent }: LaunchControlsProps) {
  return (
    <div className="nav-start-grid launch-controls-grid">
      {(Object.keys(LABELS) as LaunchComponent[]).map((component) => (
        <div className="nav-start-card" key={component}>
          <div className="nav-start-card-heading">
            <span>{LABELS[component].eyebrow}</span>
            <b>{LABELS[component].title}</b>
          </div>
          <small>{LABELS[component].detail}</small>
          <LaunchToggleButton
            component={component}
            state={state}
            hardware={hardware}
            sendCommand={sendCommand}
            onEvent={onEvent}
          />
        </div>
      ))}
    </div>
  );
}
