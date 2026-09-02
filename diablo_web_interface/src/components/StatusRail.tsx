import { Icon } from "./Icon";
import { Panel, StatCard } from "./Panel";
import type { DiabloState, EventEntry } from "../types";

function number(value: number | null | undefined, digits = 2) {
  return value === null || value === undefined || !Number.isFinite(value) ? "—" : value.toFixed(digits);
}

function modeLabel(value: number | undefined) {
  if (value === undefined) return "—";
  return `MODE ${value}`;
}

interface StatusRailProps {
  state: DiabloState;
  connected: boolean;
  topicConnected: boolean;
  events: EventEntry[];
  onClearEvents: () => void;
}

export function StatusRail({ state, connected, topicConnected, events, onClearEvents }: StatusRailProps) {
  const battery = state.telemetry.battery;
  const body = state.telemetry.body_state;
  const imu = state.telemetry.imu;
  const motors = state.telemetry.motors;
  const percentage = battery?.percentage ?? 0;

  return (
    <aside className="status-rail">
      <Panel title="System Status" eyebrow="ROBOT TELEMETRY" accent="cyan">
        <div className="status-rail-links">
          <div className="link-row"><span><i className={connected ? "dot green" : "dot red"} />ROS bridge</span><b>{connected ? "CONNECTED" : "OFFLINE"}</b></div>
          <div className="link-row"><span><i className={topicConnected ? "dot green" : "dot amber"} />Topic echo</span><b>{topicConnected ? "READY" : "STANDBY"}</b></div>
        </div>
        <div className="battery-block">
          <div className="battery-heading"><span><Icon name="battery" size={16} /> BATTERY</span><strong>{battery ? `${number(battery.percentage, 0)}%` : "—"}</strong></div>
          <div className="battery-track"><span style={{ width: `${Math.max(0, Math.min(100, percentage))}%` }} /></div>
          <div className="battery-meta"><span>{number(battery?.voltage, 1)} V</span><span>{number(battery?.current, 1)} A</span><span>{number(battery?.temperature, 1)} °C</span></div>
        </div>
        <div className="rail-stat-grid">
          <StatCard label="ROBOT STATE" value={modeLabel(body?.robot_mode)} detail={body?.error ? `ERROR 0x${body.error.toString(16)}` : "NO ERROR"} tone={body?.error ? "red" : "green"} />
          <StatCard label="BODY CONTROL" value={modeLabel(body?.ctrl_mode)} detail={body?.warning ? `WARN 0x${body.warning.toString(16)}` : "NOMINAL"} tone={body?.warning ? "orange" : "default"} />
        </div>
      </Panel>

      <Panel title="Pose & IMU" eyebrow="STATE ESTIMATION" accent="blue">
        <div className="pose-rail-grid">
          <StatCard label="X POSITION" value={number(state.pose?.x)} unit="meters" />
          <StatCard label="Y POSITION" value={number(state.pose?.y)} unit="meters" />
          <StatCard label="HEADING θ" value={number(state.pose?.theta)} unit="radians" />
          <StatCard label="IMU YAW" value={number(imu?.yaw)} unit="radians" />
        </div>
        <div className="imu-strip"><span>ROLL <b>{number(imu?.roll)}</b></span><span>PITCH <b>{number(imu?.pitch)}</b></span><span>ωZ <b>{number(imu?.angular_velocity.z)} rad/s</b></span></div>
      </Panel>

      <Panel title="Wheel Feedback" eyebrow="MOTOR TELEMETRY" accent="orange">
        <div className="motor-table">
          <div className="motor-row motor-head"><span>CHANNEL</span><span>POSITION</span><span>VELOCITY</span></div>
          <div className="motor-row"><span>LEFT WHEEL</span><b>{number(motors?.left_wheel.position)}</b><b>{number(motors?.left_wheel.velocity)} rad/s</b></div>
          <div className="motor-row"><span>RIGHT WHEEL</span><b>{number(motors?.right_wheel.position)}</b><b>{number(motors?.right_wheel.velocity)} rad/s</b></div>
        </div>
        <div className="leg-height"><span>LEG HEIGHT</span><b>{number(motors?.left_leg_length, 3)} / {number(motors?.right_leg_length, 3)} m</b></div>
      </Panel>

      <Panel
        title="Event Stream"
        eyebrow="SYSTEM LOG"
        accent="slate"
        actions={<button className="panel-icon-action" type="button" onClick={onClearEvents} title="Clear log"><Icon name="trash" size={14} /></button>}
      >
        <div className="event-stream">
          {events.length === 0 ? <div className="event-empty">No events recorded.</div> : events.slice(-8).map((event) => (
            <div className={`event-row ${event.kind}`} key={event.id}><time>{event.time}</time><span>{event.message}</span></div>
          ))}
        </div>
      </Panel>
    </aside>
  );
}
