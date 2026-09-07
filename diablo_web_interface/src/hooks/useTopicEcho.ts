import { useCallback, useEffect, useRef, useState } from "react";
import { demoTopics } from "../demo";
import type { TopicDescriptor, TopicPacket } from "../types";

function demoPacket(topic: string, slot: number): TopicPacket {
  const data = topic.includes("Battery")
    ? { voltage: 24.6, percentage: 82, current: 1.8 }
    : topic.includes("Motors")
      ? { left_wheel_pos: 4.2, right_wheel_pos: -4.0, left_wheel_vel: 0.14, right_wheel_vel: -0.13 }
      : topic === "/diablo/odometry" || topic === "/odometry/filtered" || topic === "/odom"
        ? { pose: { pose: { position: { x: 0.62, y: 0.42 } } }, twist: { twist: { linear: { x: 0.04 } } } }
        : topic === "/scan"
          ? { header: { frame_id: "laser" }, ranges: "[180 values]" }
          : { status: "demo preview", topic };
  const type = demoTopics.find((item) => item.name === topic)?.types[0] || "demo_msgs/msg/Preview";
  return {
    type: "topic",
    slot,
    topic,
    msg_type: type,
    count: slot * 17,
    stamp: "preview",
    data,
  };
}

export function useTopicEcho() {
  const socketRef = useRef<WebSocket | null>(null);
  const retryRef = useRef<number | null>(null);
  const mountedRef = useRef(true);
  const selectedRef = useRef<string[]>([]);
  const [catalog, setCatalog] = useState<TopicDescriptor[]>(demoTopics);
  const [selected, setSelected] = useState<string[]>([]);
  const [packets, setPackets] = useState<Record<number, TopicPacket>>({});
  const [connected, setConnected] = useState(false);

  const sendSubscription = useCallback((topics: string[]) => {
    const socket = socketRef.current;
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify({ type: "subscribe", topics }));
    }
  }, []);

  const connect = useCallback(() => {
    if (!mountedRef.current) return;
    if (socketRef.current && socketRef.current.readyState <= WebSocket.OPEN) return;
    const scheme = window.location.protocol === "https:" ? "wss" : "ws";
    const socket = new WebSocket(`${scheme}://${window.location.host}/ws/topics`);
    socketRef.current = socket;
    socket.onopen = () => {
      if (!mountedRef.current) return;
      setConnected(true);
      sendSubscription(selectedRef.current);
    };
    socket.onmessage = (event) => {
      if (!mountedRef.current) return;
      try {
        const packet = JSON.parse(event.data) as TopicPacket;
        if (packet.type === "topic") {
          setPackets((previous) => ({ ...previous, [packet.slot]: packet }));
        }
      } catch {
        // The topic panel keeps its last valid packet when a malformed packet arrives.
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
        }, 2500);
      }
    };
  }, [sendSubscription]);

  const refresh = useCallback(async () => {
    try {
      const response = await fetch("/api/topics");
      if (!response.ok) throw new Error("topic API unavailable");
      const result = (await response.json()) as { topics?: TopicDescriptor[] };
      setCatalog(result.topics || []);
    } catch {
      setCatalog(demoTopics);
    }
  }, []);

  useEffect(() => {
    mountedRef.current = true;
    selectedRef.current = selected;
  }, [selected]);

  useEffect(() => {
    mountedRef.current = true;
    refresh();
    connect();
    return () => {
      mountedRef.current = false;
      if (retryRef.current !== null) window.clearTimeout(retryRef.current);
      retryRef.current = null;
      socketRef.current?.close();
      socketRef.current = null;
    };
  }, [connect, refresh]);

  const subscribe = useCallback((topics: string[]) => {
    const unique = [...new Set(topics)].slice(0, 4);
    selectedRef.current = unique;
    setSelected(unique);
    const nextPackets: Record<number, TopicPacket> = {};
    unique.forEach((topic, index) => {
      nextPackets[index + 1] = demoPacket(topic, index + 1);
    });
    setPackets(nextPackets);
    sendSubscription(unique);
  }, [sendSubscription]);

  const clear = useCallback(() => {
    selectedRef.current = [];
    setSelected([]);
    setPackets({});
    const socket = socketRef.current;
    if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify({ type: "clear" }));
  }, []);

  return { catalog, selected, packets, connected, subscribe, clear, refresh };
}
