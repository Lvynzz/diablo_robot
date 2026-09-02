import { useCallback, useEffect, useRef, useState } from "react";
import { Header } from "./components/Header";
import { Sidebar } from "./components/Sidebar";
import { StatusRail } from "./components/StatusRail";
import { ViewToolbar } from "./components/ViewToolbar";
import { DriveView } from "./components/DriveView";
import { NavigationView } from "./components/NavigationView";
import { TopicsView } from "./components/TopicsView";
import { SettingsView } from "./components/SettingsView";
import { useDiabloConnection } from "./hooks/useDiabloConnection";
import { useTopicEcho } from "./hooks/useTopicEcho";
import type { AppView, EventEntry, PanelKey, WebConfig } from "./types";
import "./styles.css";

const initialPanels: Record<PanelKey, boolean> = {
  motion: true,
  telemetry: true,
  trajectory: true,
  map: true,
  poses: true,
  controls: true,
  costmaps: true,
  log: true,
};

function App() {
  const [view, setView] = useState<AppView>("drive");
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);
  const [panels, setPanels] = useState(initialPanels);
  const [events, setEvents] = useState<EventEntry[]>([
      { id: 1, time: new Date().toLocaleTimeString([], { hour12: false }), message: "Diablo Robot HMI initialized in local preview mode.", kind: "info" },
  ]);
  const [config, setConfig] = useState<WebConfig | null>(null);
  const eventId = useRef(2);
  const lastConnection = useRef<boolean | null>(null);
  const lastNavState = useRef<string | null>(null);
  const lastHardwareReady = useRef<boolean | null>(null);
  const connection = useDiabloConnection();
  const topics = useTopicEcho();

  const addEvent = useCallback((message: string, kind: EventEntry["kind"] = "info") => {
    const entry: EventEntry = {
      id: eventId.current++,
      time: new Date().toLocaleTimeString([], { hour12: false }),
      message,
      kind,
    };
    setEvents((previous) => [...previous.slice(-80), entry]);
  }, []);

  useEffect(() => {
    fetch("/api/config")
      .then((response) => response.ok ? response.json() : Promise.reject(new Error("config unavailable")))
      .then((value: WebConfig) => setConfig(value))
      .catch(() => setConfig(null));
  }, []);

  useEffect(() => {
    if (lastConnection.current === null) {
      lastConnection.current = connection.connected;
      return;
    }
    if (lastConnection.current === connection.connected) return;
    lastConnection.current = connection.connected;
    addEvent(connection.connected ? "Connected to FastAPI / ROS bridge." : "FastAPI / ROS bridge disconnected; showing preview data.", connection.connected ? "success" : "warn");
  }, [addEvent, connection.connected]);

  useEffect(() => {
    if (!connection.lastPacket) return;
    const packet = connection.lastPacket;
    if (packet.type === "goal_pose_ack") addEvent(packet.accepted ? "Nav2 goal submitted." : String(packet.message || "Nav2 goal rejected."), packet.accepted ? "success" : "error");
    if (packet.type === "goal_cancel_ack") addEvent(String(packet.message || "Navigation cancel requested."), "warn");
    if (packet.type === "initial_pose_ack") addEvent("Initial pose acknowledged by backend.", "success");
    if (["reset_odom_ack", "reset_encoder_ack", "start_lidar_ack", "start_hardware_ack", "start_localization_ack", "start_navigation_ack", "start_mapping_ack"].includes(String(packet.type))) {
      const accepted = packet.requested !== false;
      addEvent(String(packet.message || "Control request acknowledged."), accepted ? "success" : "warn");
    }
    if (packet.type === "error") addEvent(String(packet.detail || "WebSocket error"), "error");
  }, [addEvent, connection.lastPacket]);

  useEffect(() => {
    const current = connection.state.nav_goal.state;
    if (lastNavState.current === null) {
      lastNavState.current = current;
      return;
    }
    if (lastNavState.current === current) return;
    lastNavState.current = current;
    if (["succeeded", "aborted", "error", "rejected", "canceled"].includes(current)) {
      addEvent(connection.state.nav_goal.message, current === "succeeded" ? "success" : current === "canceled" ? "warn" : "error");
    }
  }, [addEvent, connection.state.nav_goal]);

  useEffect(() => {
    const ready = connection.state.hardware.ready;
    if (lastHardwareReady.current === null) {
      lastHardwareReady.current = ready;
      return;
    }
    if (lastHardwareReady.current === ready) return;
    lastHardwareReady.current = ready;
    addEvent(
      ready ? "Hardware ready: Drive Control unlocked." : "Hardware feedback lost: Drive Control locked.",
      ready ? "success" : "error",
    );
  }, [addEvent, connection.state.hardware.ready]);

  const togglePanel = (panel: PanelKey) => setPanels((previous) => ({ ...previous, [panel]: !previous[panel] }));
  const stop = () => {
    void connection.sendCommand({ type: "stop" });
    addEvent("STOP command sent; motion output released.", "error");
  };
  const toggleMode = () => {
    const mode = connection.state.control_mode === "auto" ? "manual" : "auto";
    void connection.sendCommand({ type: "mode", mode });
    addEvent(`Control mode changed to ${mode.toUpperCase()}.`, mode === "auto" ? "warn" : "info");
  };
  return (
    <div className={`hmi-app theme-light ${sidebarCollapsed ? "sidebar-is-collapsed" : ""}`}>
      <Header connected={connection.connected} nav2Ready={connection.nav2Ready} mode={connection.state.control_mode} demoMode={connection.demoMode} onStop={stop} onMode={toggleMode} onSettings={() => setView("settings")} onMenu={() => setSidebarCollapsed((previous) => !previous)} />
      <div className="hmi-body">
        <Sidebar view={view} collapsed={sidebarCollapsed} onSelect={setView} onToggleCollapse={() => setSidebarCollapsed((previous) => !previous)} />
        <div className="hmi-workspace">
          <ViewToolbar view={view} panels={panels} onToggle={togglePanel} onRefreshTopics={() => { void topics.refresh(); addEvent("ROS topic catalog refreshed.", "info"); }} onClearTopics={() => { topics.clear(); addEvent("Topic echo stopped.", "warn"); }} />
          <main className={`hmi-content content-${view}`}>
            <div className="primary-view">
              {view === "drive" && <DriveView state={connection.state} hardwareReady={connection.state.hardware.ready} panels={panels} sendCommand={connection.sendCommand} onEvent={addEvent} />}
              {view === "navigation" && <NavigationView state={connection.state} hardware={connection.state.hardware} panels={panels} sendCommand={connection.sendCommand} events={events} onEvent={addEvent} />}
              {view === "topics" && <TopicsView catalog={topics.catalog} selected={topics.selected} packets={topics.packets} connected={topics.connected} onSubscribe={topics.subscribe} onClear={topics.clear} onRefresh={() => { void topics.refresh(); addEvent("ROS topic catalog refreshed.", "info"); }} onEvent={addEvent} />}
              {view === "settings" && <SettingsView config={config} state={connection.state} connected={connection.connected} nav2Ready={connection.nav2Ready} onReconnect={connection.reconnect} onEvent={addEvent} />}
            </div>
            {view !== "settings" && view !== "drive" && <StatusRail state={connection.state} connected={connection.connected} topicConnected={topics.connected} events={events} onClearEvents={() => setEvents([])} />}
          </main>
        </div>
      </div>
    </div>
  );
}

export default App;
