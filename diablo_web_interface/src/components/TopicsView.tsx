import { useMemo, useState } from "react";
import { Icon } from "./Icon";
import { EmptyState, Panel } from "./Panel";
import type { EventEntry, TopicPacket } from "../types";

interface TopicsViewProps {
  catalog: Array<{ name: string; types: string[] }>;
  selected: string[];
  packets: Record<number, TopicPacket>;
  connected: boolean;
  onSubscribe: (topics: string[]) => void;
  onClear: () => void;
  onRefresh: () => void;
  onEvent: (message: string, kind?: EventEntry["kind"]) => void;
}

function TopicCard({ packet, topic, slot }: { packet?: TopicPacket; topic: string; slot: number }) {
  return (
    <article className="topic-card">
      <header><div><span className="topic-slot">SLOT {String(slot).padStart(2, "0")}</span><strong>{packet?.topic || topic}</strong></div><span className="topic-count">{packet ? `${packet.count} MSG` : "WAITING"}</span></header>
      <div className="topic-meta"><span>{packet?.msg_type || "Resolving message type…"}</span><time>{packet?.stamp || "—"}</time></div>
      <pre>{JSON.stringify(packet?.data || { status: "waiting for message" }, null, 2)}</pre>
    </article>
  );
}

export function TopicsView({ catalog, selected, packets, connected, onSubscribe, onClear, onRefresh, onEvent }: TopicsViewProps) {
  const [filter, setFilter] = useState("");
  const [selectedOption, setSelectedOption] = useState("");
  const options = useMemo(() => catalog.filter((item) => !filter || item.name.toLowerCase().includes(filter.toLowerCase())), [catalog, filter]);
  const addTopic = () => {
    const topic = selectedOption || options[0]?.name;
    if (!topic) return;
    if (selected.includes(topic)) { onEvent("Topic is already in the echo list.", "warn"); return; }
    if (selected.length >= 4) { onEvent("Topic echo is limited to four topics.", "warn"); return; }
    onSubscribe([...selected, topic]);
    onEvent(`Subscribed to ${topic}.`, "success");
  };

  return (
    <div className="view-stack topic-view">
      <div className="topics-summary"><div><span className="page-eyebrow">LIVE ROS 2 DATA</span><h1>Topic Inspector</h1><p>Inspect the live ROS graph without leaving the operator HMI.</p></div><div className={`echo-status ${connected ? "online" : ""}`}><i /> {connected ? "SOCKET STREAM ACTIVE" : "LOCAL PREVIEW DATA"}</div></div>
      <div className="topic-layout">
        <Panel title="Topic Catalog" eyebrow="ROS GRAPH" accent="blue" actions={<button className="panel-icon-action" type="button" onClick={onRefresh} title="Refresh topics"><Icon name="refresh" size={15} /></button>}>
          <label className="search-field"><span>FILTER TOPICS</span><input type="search" placeholder="/diablo/…" value={filter} onChange={(event) => setFilter(event.target.value)} /></label>
          <div className="topic-select-row"><select value={selectedOption} onChange={(event) => setSelectedOption(event.target.value)}><option value="">Select a ROS topic…</option>{options.map((item) => <option value={item.name} key={item.name}>{item.name}</option>)}</select><button className="primary-action" type="button" onClick={addTopic}><Icon name="plus" size={15} /> ADD</button></div>
          <div className="catalog-list">{options.length ? options.map((item) => <button type="button" key={item.name} className={selected.includes(item.name) ? "selected" : ""} onClick={() => setSelectedOption(item.name)}><span className="catalog-topic">{item.name}</span><small>{item.types.join(", ")}</small>{selected.includes(item.name) && <b>ON</b>}</button>) : <EmptyState title="No topics match" />}</div>
          <div className="topic-limit"><span>ACTIVE ECHO SLOTS</span><strong>{selected.length} / 4</strong></div>
        </Panel>
        <Panel title="Live Echo" eyebrow="MESSAGE STREAM" accent="cyan" actions={<button className="danger-text-button" type="button" onClick={onClear}><Icon name="trash" size={14} /> STOP ECHO</button>}>
          {!selected.length ? <EmptyState title="Select topics to begin" detail="Choose up to four ROS topics from the catalog." /> : <div className="topic-card-grid">{selected.map((topic, index) => <TopicCard key={topic} topic={topic} slot={index + 1} packet={packets[index + 1]} />)}</div>}
        </Panel>
      </div>
      <Panel title="Echo Notes" eyebrow="OPERATOR REFERENCE" accent="slate">
        <div className="echo-notes"><span><b>01</b> Messages are rate-limited to protect the browser connection.</span><span><b>02</b> Large arrays are truncated by the FastAPI serializer.</span><span><b>03</b> The browser never publishes directly to arbitrary ROS topics.</span></div>
      </Panel>
    </div>
  );
}
