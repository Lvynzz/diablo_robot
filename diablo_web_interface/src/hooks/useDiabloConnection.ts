import { useCallback, useEffect, useRef, useState } from "react";
import { demoState } from "../demo";
import type { DiabloState, SocketCommand } from "../types";

interface SocketPacket {
  type?: string;
  nav2_ready?: boolean;
  [key: string]: unknown;
}

function restPathFor(command: SocketCommand): string | null {
  switch (command.type) {
    case "manual":
      return "/api/teleop";
    case "stop":
      return "/api/control/stop";
    case "stand":
      return "/api/teleop/stand";
    case "reset_odom":
      return "/api/odom/reset";
    case "reset_encoder":
      return "/api/encoder/reset";
    case "start_lidar":
      return "/api/sensors/lidar/start";
    case "start_hardware":
      return "/api/hardware/start";
    case "start_localization":
      return "/api/navigation/start-localization";
    case "start_navigation":
      return "/api/navigation/start";
    case "start_mapping":
      return "/api/mapping/start";
    case "mode":
      return "/api/control/mode";
    case "goal_pose":
      return "/api/goal/nav2";
    case "initial_pose":
      return "/api/localization/initialpose";
    case "cancel_goal":
      return "/api/goal/cancel";
    default:
      return null;
  }
}

export function useDiabloConnection() {
  const socketRef = useRef<WebSocket | null>(null);
  const retryRef = useRef<number | null>(null);
  const mountedRef = useRef(true);
  const [state, setState] = useState<DiabloState>(demoState);
  const [connected, setConnected] = useState(false);
  const [nav2Ready, setNav2Ready] = useState(false);
  const [lastPacket, setLastPacket] = useState<SocketPacket | null>(null);

  const connect = useCallback(() => {
    if (!mountedRef.current) return;
    if (socketRef.current && socketRef.current.readyState <= WebSocket.OPEN) return;

    const scheme = window.location.protocol === "https:" ? "wss" : "ws";
    const socket = new WebSocket(`${scheme}://${window.location.host}/ws`);
    socketRef.current = socket;

    socket.onopen = () => {
      if (!mountedRef.current) return;
      setConnected(true);
    };
    socket.onmessage = (event) => {
      if (!mountedRef.current) return;
      try {
        const packet = JSON.parse(event.data) as SocketPacket;
      if (packet.type === "state") {
          setState((previous) => ({ ...previous, ...(packet as Partial<DiabloState>) }));
          setNav2Ready(Boolean(packet.nav2_ready));
        } else {
          setLastPacket(packet);
        }
      } catch {
        setLastPacket({ type: "error", detail: "Invalid WebSocket packet" });
      }
    };
    socket.onerror = () => {
      if (mountedRef.current) setConnected(false);
    };
    socket.onclose = () => {
      if (!mountedRef.current) return;
      setConnected(false);
      socketRef.current = null;
      if (retryRef.current === null) {
        retryRef.current = window.setTimeout(() => {
          retryRef.current = null;
          connect();
        }, 2000);
      }
    };
  }, []);

  useEffect(() => {
    mountedRef.current = true;
    connect();
    return () => {
      mountedRef.current = false;
      if (retryRef.current !== null) window.clearTimeout(retryRef.current);
      retryRef.current = null;
      socketRef.current?.close();
      socketRef.current = null;
    };
  }, [connect]);

  const sendCommand = useCallback(async (command: SocketCommand) => {
    const socket = socketRef.current;
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(command));
      return true;
    }

    const path = restPathFor(command);
    if (!path) return false;
    try {
      const response = await fetch(path, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(command.type === "mode" ? { mode: command.mode } : command),
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const result = await response.json().catch(() => null) as { requested?: boolean; started?: boolean; accepted?: boolean } | null;
      if (result && (result.requested === false || result.started === false || result.accepted === false)) return false;
      return true;
    } catch {
      return false;
    }
  }, []);

  return {
    state,
    connected,
    nav2Ready,
    demoMode: !connected,
    lastPacket,
    sendCommand,
    reconnect: connect,
  };
}
