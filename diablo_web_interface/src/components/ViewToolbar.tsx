import { Icon } from "./Icon";
import { PanelToggle } from "./Panel";
import type { AppView, PanelKey } from "../types";

interface ViewToolbarProps {
  view: AppView;
  panels: Record<PanelKey, boolean>;
  onToggle: (panel: PanelKey) => void;
  onRefreshTopics: () => void;
  onClearTopics: () => void;
}

export function ViewToolbar({ view, panels, onToggle, onRefreshTopics, onClearTopics }: ViewToolbarProps) {
  const title = view === "drive" ? "Drive Control" : view === "navigation" ? "Navigation" : view === "topics" ? "ROS Topics" : "Settings";
  return (
    <div className="view-toolbar">
      <div className="breadcrumb"><span>DIABLO HMI</span><b>›</b><strong>{title}</strong></div>
      <div className="toolbar-spacer" />
      {view === "drive" && <>
        <span className="toolbar-label">PANELS</span>
        <PanelToggle label="Motion" active={panels.motion} onClick={() => onToggle("motion")} />
        <PanelToggle label="Telemetry" active={panels.telemetry} onClick={() => onToggle("telemetry")} />
        <PanelToggle label="Trajectory" active={panels.trajectory} onClick={() => onToggle("trajectory")} />
      </>}
      {view === "navigation" && <>
        <span className="toolbar-label">PANELS</span>
        <PanelToggle label="Map View" active={panels.map} onClick={() => onToggle("map")} />
        <PanelToggle label="Stations" active={panels.poses} onClick={() => onToggle("poses")} />
        <PanelToggle label="Controls" active={panels.controls} onClick={() => onToggle("controls")} />
        <PanelToggle label="Restrictions" active={panels.costmaps} onClick={() => onToggle("costmaps")} />
        <PanelToggle label="Log" active={panels.log} onClick={() => onToggle("log")} />
      </>}
      {view === "topics" && <>
        <button className="toolbar-action" type="button" onClick={onRefreshTopics}><Icon name="refresh" size={14} /> REFRESH LIST</button>
        <button className="toolbar-action danger" type="button" onClick={onClearTopics}><Icon name="trash" size={14} /> STOP ECHO</button>
      </>}
    </div>
  );
}
