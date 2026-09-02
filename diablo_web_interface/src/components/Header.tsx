import { useEffect, useState } from "react";
import { Icon } from "./Icon";
import type { ControlMode } from "../types";

interface HeaderProps {
  connected: boolean;
  nav2Ready: boolean;
  mode: ControlMode;
  demoMode: boolean;
  onStop: () => void;
  onMode: () => void;
  onSettings: () => void;
  onMenu: () => void;
}

export function Header({ connected, nav2Ready, mode, demoMode, onStop, onMode, onSettings, onMenu }: HeaderProps) {
  const [clock, setClock] = useState(() => new Date());
  useEffect(() => {
    const timer = window.setInterval(() => setClock(new Date()), 1000);
    return () => window.clearInterval(timer);
  }, []);

  const modeLabel = mode === "auto" ? "AUTO MODE" : mode === "stop" ? "STOPPED" : "MANUAL MODE";
  return (
    <header className="hmi-header">
      <button className="header-menu" type="button" aria-label="Open menu" onClick={onMenu}>
        <Icon name="menu" size={19} />
      </button>
      <div className="brand-lockup">
        <div className="brand-mark">D</div>
        <div>
          <strong>DIABLO ROBOT</strong>
          <small>INDUSTRIAL INTERFACE</small>
        </div>
      </div>
      <div className="header-divider" />
      <div className="header-system">
        <span>SYSTEM</span>
        <b className={connected ? "text-green" : "text-red"}>{connected ? "ONLINE" : "OFFLINE"}</b>
      </div>
      <div className="header-divider header-divider-small" />
      <div className="header-system header-clock">
        <span>LOCAL TIME</span>
        <b>{clock.toLocaleTimeString([], { hour12: false })}</b>
      </div>
      <div className="header-spacer" />
      {demoMode && <span className="demo-badge">LOCAL PREVIEW</span>}
      <div className={`header-health ${nav2Ready ? "ready" : ""}`}>
        <i /> <span>{nav2Ready ? "NAV2 READY" : "NAV2 OFFLINE"}</span>
      </div>
      <button className={`mode-button mode-${mode}`} type="button" onClick={onMode} title="Toggle manual/auto mode">
        {modeLabel}
      </button>
      <button className="icon-button" type="button" title="Open settings" onClick={onSettings}>
        <Icon name="settings" size={17} />
      </button>
      <button className="stop-button" type="button" onClick={onStop}>
        <Icon name="stop" size={15} /> STOP
      </button>
    </header>
  );
}
