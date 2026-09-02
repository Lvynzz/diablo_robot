import { Icon, type IconName } from "./Icon";
import type { AppView } from "../types";

interface SidebarProps {
  view: AppView;
  collapsed: boolean;
  onSelect: (view: AppView) => void;
  onToggleCollapse: () => void;
}

const items: Array<{ view: AppView; label: string; detail: string; icon: IconName; badge?: string }> = [
  { view: "drive", label: "Drive Control", detail: "Manual motion", icon: "drive" },
  { view: "navigation", label: "Navigation", detail: "Map + goal control", icon: "navigation" },
  { view: "topics", label: "ROS Topics", detail: "Live topic echo", icon: "topics", badge: "0/4" },
];

export function Sidebar({ view, collapsed, onSelect, onToggleCollapse }: SidebarProps) {
  return (
    <aside className={`hmi-sidebar ${collapsed ? "collapsed" : ""}`}>
      <div className="sidebar-topline">
        <div className="sidebar-label">OPERATIONS</div>
        <button
          className="sidebar-collapse-toggle"
          type="button"
          aria-label={collapsed ? "Expand panel menu" : "Collapse panel menu"}
          title={collapsed ? "Expand panel menu" : "Collapse panel menu"}
          onClick={onToggleCollapse}
        >
          <Icon name="chevron" size={13} />
        </button>
      </div>
      {items.map((item) => (
        <button
          className={`sidebar-item ${view === item.view ? "active" : ""}`}
          type="button"
          key={item.view}
          onClick={() => onSelect(item.view)}
          title={item.label}
        >
          <span className="sidebar-icon"><Icon name={item.icon} size={18} /></span>
          <span className="sidebar-copy"><strong>{item.label}</strong><small>{item.detail}</small></span>
          {item.badge && <span className="sidebar-badge">{item.badge}</span>}
        </button>
      ))}
      <div className="sidebar-spacer" />
      <div className="sidebar-label">SYSTEM</div>
      <button
        className={`sidebar-item ${view === "settings" ? "active" : ""}`}
        type="button"
        onClick={() => onSelect("settings")}
        title="Settings"
      >
        <span className="sidebar-icon"><Icon name="settings" size={18} /></span>
        <span className="sidebar-copy"><strong>Settings</strong><small>Connection + frames</small></span>
      </button>
      <div className="sidebar-version"><span>DIABLO HMI</span><b>v0.1</b></div>
    </aside>
  );
}
