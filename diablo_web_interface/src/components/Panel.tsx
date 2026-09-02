import { useState, type ReactNode } from "react";
import { Icon } from "./Icon";

interface PanelProps {
  title: string;
  eyebrow?: string;
  accent?: "blue" | "cyan" | "orange" | "green" | "red" | "slate";
  actions?: ReactNode;
  className?: string;
  defaultCollapsed?: boolean;
  children: ReactNode;
}

export function Panel({ title, eyebrow, accent = "blue", actions, className = "", defaultCollapsed = false, children }: PanelProps) {
  const [collapsed, setCollapsed] = useState(defaultCollapsed);
  return (
    <section className={`panel panel-${accent} ${collapsed ? "panel-collapsed" : ""} ${className}`}>
      <header className="panel-header">
        <span className="panel-header-dot" />
        <div className="panel-heading-copy">
          {eyebrow && <span className="panel-eyebrow">{eyebrow}</span>}
          <h2>{title}</h2>
        </div>
        {actions && <div className="panel-actions">{actions}</div>}
        <button
          className="panel-collapse-action"
          type="button"
          aria-label={`${collapsed ? "Open" : "Close"} ${title}`}
          aria-expanded={!collapsed}
          onClick={() => setCollapsed((previous) => !previous)}
        >
          <Icon name="chevron" size={13} />
        </button>
      </header>
      {!collapsed && <div className="panel-body">{children}</div>}
    </section>
  );
}

export function PanelToggle({ label, active, onClick }: { label: string; active: boolean; onClick: () => void }) {
  return (
    <button className={`toolbar-toggle ${active ? "active" : ""}`} type="button" onClick={onClick}>
      <span className="toolbar-toggle-dot" />
      {label}
    </button>
  );
}

export function StatCard({ label, value, unit, tone = "default", detail }: { label: string; value: string; unit?: string; tone?: string; detail?: string }) {
  return (
    <div className={`stat-card tone-${tone}`}>
      <span className="stat-label">{label}</span>
      <strong>{value}</strong>
      {unit && <small>{unit}</small>}
      {detail && <em>{detail}</em>}
    </div>
  );
}

export function EmptyState({ title, detail }: { title: string; detail?: string }) {
  return (
    <div className="empty-state">
      <span className="empty-state-mark">//</span>
      <strong>{title}</strong>
      {detail && <small>{detail}</small>}
    </div>
  );
}
