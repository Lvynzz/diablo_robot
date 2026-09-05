import { Icon } from "./Icon";
import { Panel, StatCard } from "./Panel";
import type { DiabloState, EventEntry, WebConfig } from "../types";

const fallbackConfig: WebConfig = {
  robot: "Diablo",
  manual_cmd_topic: "/diablo/MotionCmd/manual",
  control_mode_topic: "/diablo/control_mode",
  map_topic: "/map",
  odom_topic: "/odometry/filtered",
  scan_topic: "/scan",
  base_frame: "diablo_base_link",
  map_frame: "map",
  limits: { forward: 1, turn: 1, roll: 0.2 },
};

interface SettingsViewProps {
  config: WebConfig | null;
  state: DiabloState;
  connected: boolean;
  nav2Ready: boolean;
  onReconnect: () => void;
  onEvent: (message: string, kind?: EventEntry["kind"]) => void;
}

export function SettingsView({ config, state, connected, nav2Ready, onReconnect, onEvent }: SettingsViewProps) {
  const current = config || fallbackConfig;
  return (
    <div className="view-stack settings-view">
      <div className="topics-summary"><div><span className="page-eyebrow">SYSTEM CONFIGURATION</span><h1>Settings & Diagnostics</h1><p>Runtime endpoints and frame assumptions used by the Diablo HMI.</p></div><button className="toolbar-action" type="button" onClick={() => { onReconnect(); onEvent("Reconnect requested.", "info"); }}><Icon name="refresh" size={14} /> RECONNECT</button></div>
      <div className="settings-grid">
        <Panel title="Connection" eyebrow="WEB HMI ARCHITECTURE" accent="cyan">
          <div className="architecture-flow"><span>Browser<br /><b>React + Vite</b></span><i>↔</i><span>FastAPI<br /><b>web_node.py</b></span><i>↔</i><span>ROS 2<br /><b>rclpy</b></span></div>
          <div className="settings-status-grid"><StatCard label="WEBSOCKET /WS" value={connected ? "ONLINE" : "OFFLINE"} tone={connected ? "green" : "red"} /><StatCard label="NAVIGATE_TO_POSE" value={nav2Ready ? "READY" : "OFFLINE"} tone={nav2Ready ? "green" : "orange"} /></div>
          <div className="settings-list"><div><span>FastAPI host</span><code>{window.location.host}</code></div><div><span>State stream</span><code>/ws</code></div><div><span>Topic echo stream</span><code>/ws/topics</code></div><div><span>REST namespace</span><code>/api/*</code></div></div>
        </Panel>
        <Panel title="ROS Frame Map" eyebrow="NAV2 CONTRACT" accent="blue">
          <div className="settings-list"><div><span>Map frame</span><code>{current.map_frame}</code></div><div><span>Base frame</span><code>{current.base_frame}</code></div><div><span>Odometry</span><code>{current.odom_topic} · fused wheel/IMU</code></div><div><span>Laser scan</span><code>{current.scan_topic}</code></div><div><span>Static map</span><code>{current.map_topic}</code></div><div><span>Reset pose</span><code>/diablo/reset_odom</code></div><div><span>Reset encoder</span><code>{current.reset_encoder_service || "not configured"}</code></div><div><span>Start LiDAR</span><code>{current.lidar_start_service || "not configured"}</code></div></div>
          <div className="settings-callout"><Icon name="map" size={17} /><span>Full-body hardware publishes filtered odometry on <code>/odometry/filtered</code>; EKF owns <code>odom → diablo_base_link</code>.</span></div>
        </Panel>
      </div>
      <Panel title="Diablo Command Contract" eyebrow="MOTIONCTRL ROUTING" accent="orange">
        <div className="routing-table"><div className="routing-row routing-head"><span>SOURCE</span><span>TOPIC</span><span>DESTINATION</span></div><div className="routing-row"><span>Web teleop</span><code>{current.manual_cmd_topic}</code><b>motion_cmd_mux</b></div><div className="routing-row"><span>Nav2 velocity</span><code>/cmd_vel_smoothed</code><b>motion_cmd_bridge</b></div><div className="routing-row"><span>Robot driver</span><code>/diablo/MotionCmd</code><b>diablo_ctrl_node</b></div></div>
      </Panel>
      <Panel title="Deployment Checklist" eyebrow="ROBOT HANDOFF" accent="slate">
        <div className="checklist"><div><i className="checkmark">✓</i><span>React frontend source available in <code>src/</code></span></div><div><i className="checkmark">✓</i><span>FastAPI serves built assets under <code>/static/</code></span></div><div><i className="checkmark pending">!</i><span>Install Node.js/npm and run <code>npm install && npm run build</code></span></div><div><i className="checkmark pending">!</i><span>Verify <code>/scan</code>, TF, map→odom, and wheel calibration on robot</span></div></div>
      </Panel>
      <div className="settings-footer-note">State packet timestamp: {new Date(state.stamp * 1000).toLocaleTimeString([], { hour12: false })} · Current mode: {state.control_mode.toUpperCase()}</div>
    </div>
  );
}
