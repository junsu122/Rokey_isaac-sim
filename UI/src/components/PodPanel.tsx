import { useState } from "react";
import type { SectionData, PodData } from "@/types";

const POD_STATE_STYLE: Record<string, { bg: string; color: string; label: string }> = {
  full:    { bg: "#FF9900", color: "#111",    label: "가득" },
  filling: { bg: "#FFF3CD", color: "#856404", label: "채우는 중" },
  moving:  { bg: "#E8F4FD", color: "#007185", label: "이동 중" },
  empty:   { bg: "#F3F3F3", color: "#aaa",    label: "비어있음" },
};

function PodCell({ pod }: { pod: PodData }) {
  const [hovered, setHovered] = useState(false);
  const style = POD_STATE_STYLE[pod.state] ?? POD_STATE_STYLE.empty;

  return (
    <div
      onMouseEnter={() => setHovered(true)}
      onMouseLeave={() => setHovered(false)}
      style={{
        background: style.bg,
        color: style.color,
        border: `1px solid ${style.color}44`,
        borderRadius: 4,
        padding: "8px 4px",
        textAlign: "center",
        cursor: "default",
        transition: "transform 0.1s",
        transform: hovered ? "scale(1.05)" : "scale(1)",
        position: "relative",
      }}
    >
      <div style={{ fontSize: 10, fontWeight: 700 }}>{pod.pod_id}</div>
      <div style={{ fontSize: 9, marginTop: 2 }}>{style.label}</div>

      {/* 호버 시 위치 툴팁 */}
      {hovered && (
        <div style={{
          position: "absolute", bottom: "110%", left: "50%",
          transform: "translateX(-50%)",
          background: "#333", color: "#fff",
          fontSize: 10, padding: "3px 6px", borderRadius: 3,
          whiteSpace: "nowrap", zIndex: 10,
          pointerEvents: "none",
        }}>
          x:{pod.location?.x?.toFixed(2) ?? "0.00"}&nbsp;
          y:{pod.location?.y?.toFixed(2) ?? "0.00"}
        </div>
      )}
    </div>
  );
}

interface Props {
  sections: SectionData[];
}

export function PodPanel({ sections }: Props) {
  const [activeTab, setActiveTab] = useState<"A" | "B" | "C">("A");

  const section = sections.find((s) => s.section_id === activeTab);
  const pods = section?.pods ?? [];

  const counts = pods.reduce<Record<string, number>>((acc, p) => {
    acc[p.state] = (acc[p.state] ?? 0) + 1;
    return acc;
  }, {});

  const headStyle: React.CSSProperties = {
    padding: "7px 14px",
    borderBottom: "1px solid var(--amz-border)",
    fontSize: 13, fontWeight: 700,
    color: "var(--amz-dark)",
    background: "#F3F3F3",
    display: "flex",
    alignItems: "center",
    gap: 12,
  };

  return (
    <div style={{
      border: "1px solid var(--amz-border)",
      borderRadius: 4,
      background: "#fff",
      overflow: "hidden",
    }}>
      {/* 헤더 + 탭 */}
      <div style={headStyle}>
        <span>Pod 상태</span>

        {/* 섹션 탭 */}
        <div style={{ display: "flex", gap: 4 }}>
          {(["A", "B", "C"] as const).map((id) => (
            <button
              key={id}
              onClick={() => setActiveTab(id)}
              style={{
                padding: "3px 12px",
                borderRadius: 3,
                border: activeTab === id
                  ? "1px solid var(--amz-orange)"
                  : "1px solid var(--amz-border)",
                background: activeTab === id ? "var(--amz-orange)" : "#fff",
                color: activeTab === id ? "#111" : "var(--amz-muted)",
                fontWeight: 700, fontSize: 12,
                cursor: "pointer",
              }}
            >
              Section {id}
            </button>
          ))}
        </div>

        {/* 요약 카운트 */}
        <div style={{ marginLeft: "auto", display: "flex", gap: 10, fontSize: 12 }}>
          {(["full", "filling", "moving", "empty"] as const).map((state) => (
            <span key={state} style={{ color: POD_STATE_STYLE[state].color }}>
              {POD_STATE_STYLE[state].label}: <strong>{counts[state] ?? 0}</strong>
            </span>
          ))}
        </div>
      </div>

      {/* Pod 그리드 */}
      <div style={{ padding: 14 }}>
        {pods.length === 0 ? (
          <p style={{ color: "var(--amz-muted)", fontSize: 13 }}>Pod 데이터 없음</p>
        ) : (
          <div style={{
            display: "grid",
            gridTemplateColumns: "repeat(auto-fill, minmax(70px, 1fr))",
            gap: 8,
          }}>
            {pods.map((pod) => (
              <PodCell key={pod.pod_id} pod={pod} />
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
