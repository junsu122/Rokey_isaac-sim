import { StatusBadge } from "@/components/StatusBadge";
import type { SectionData } from "@/types";

const PACKAGE_COLOR: Record<string, string> = {
  Big:    "#9A4700",
  Medium: "#007185",
  Small:  "#067D62",
};

const POD_STATE_COLOR: Record<string, string> = {
  full:    "#FF9900",
  filling: "#F08804",
  moving:  "#007185",
  empty:   "#D5D9D9",
};

interface Props {
  section: SectionData;
}

export function SectionCard({ section }: Props) {
  const { section_id, package_size, robots, pods, last_updated } = section;

  const podCounts = pods.reduce<Record<string, number>>((acc, pod) => {
    acc[pod.state] = (acc[pod.state] ?? 0) + 1;
    return acc;
  }, {});

  const totalPods   = pods.length;
  const fullPct     = totalPods > 0 ? ((podCounts.full ?? 0) / totalPods) * 100 : 0;
  const fillingPct  = totalPods > 0 ? ((podCounts.filling ?? 0) / totalPods) * 100 : 0;
  const movingPct   = totalPods > 0 ? ((podCounts.moving ?? 0) / totalPods) * 100 : 0;

  const isOnline = last_updated
    ? Date.now() / 1000 - last_updated.seconds < 30
    : false;

  return (
    <div style={{
      border: "1px solid var(--amz-border)",
      borderRadius: 4,
      background: "#fff",
      overflow: "hidden",
    }}>
      {/* 헤더 */}
      <div style={{
        padding: "10px 14px",
        borderBottom: "1px solid var(--amz-border)",
        background: "#F3F3F3",
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
      }}>
        <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
          <span style={{ fontSize: 22, fontWeight: 900, color: "var(--amz-dark)" }}>
            Section {section_id}
          </span>
          <span style={{
            fontSize: 11, fontWeight: 700,
            color: PACKAGE_COLOR[package_size] ?? "#555",
            background: `${PACKAGE_COLOR[package_size]}22`,
            border: `1px solid ${PACKAGE_COLOR[package_size]}55`,
            padding: "2px 8px", borderRadius: 3,
          }}>
            {package_size}
          </span>
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 4 }}>
          <span style={{
            width: 8, height: 8, borderRadius: "50%",
            background: isOnline ? "#067D62" : "#ccc",
            boxShadow: isOnline ? "0 0 5px #067D62" : "none",
          }} />
          <span style={{ fontSize: 11, color: isOnline ? "#067D62" : "#aaa" }}>
            {isOnline ? "온라인" : "오프라인"}
          </span>
        </div>
      </div>

      <div style={{ padding: "12px 14px", display: "flex", flexDirection: "column", gap: 10 }}>
        {/* M0609 */}
        <div style={{
          display: "flex", alignItems: "center", justifyContent: "space-between",
          padding: "8px 10px",
          background: "#FAFAFA",
          border: "1px solid var(--amz-border)",
          borderRadius: 4,
        }}>
          <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
            <span style={{ fontSize: 18 }}>🦾</span>
            <div>
              <p style={{ margin: 0, fontWeight: 700, fontSize: 13, color: "var(--amz-dark)" }}>M0609</p>
              <p style={{ margin: 0, fontSize: 11, color: "var(--amz-muted)", fontFamily: "monospace" }}>
                {robots?.m0609?.robot_name ?? "—"}
              </p>
            </div>
          </div>
          <StatusBadge status={robots?.m0609?.state ?? "stop"} />
        </div>

        {/* iw_hub */}
        <div style={{
          display: "flex", alignItems: "center", justifyContent: "space-between",
          padding: "8px 10px",
          background: "#FAFAFA",
          border: "1px solid var(--amz-border)",
          borderRadius: 4,
        }}>
          <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
            <span style={{ fontSize: 18 }}>🚗</span>
            <div>
              <p style={{ margin: 0, fontWeight: 700, fontSize: 13, color: "var(--amz-dark)" }}>iw_hub</p>
              <p style={{ margin: 0, fontSize: 11, color: "var(--amz-muted)", fontFamily: "monospace" }}>
                {robots?.iw_hub?.robot_name ?? "—"}
              </p>
              <p style={{ margin: 0, fontSize: 10, color: "#aaa", fontFamily: "monospace" }}>
                x:{robots?.iw_hub?.location?.x?.toFixed(2) ?? "0.00"}&nbsp;
                y:{robots?.iw_hub?.location?.y?.toFixed(2) ?? "0.00"}
              </p>
            </div>
          </div>
          <StatusBadge status={robots?.iw_hub?.state ?? "stop"} />
        </div>

        {/* Pod 요약 */}
        <div>
          <p style={{ margin: "0 0 6px", fontSize: 11, color: "var(--amz-muted)", fontWeight: 700 }}>
            Pod 현황 ({totalPods}개)
          </p>

          {/* 스택 바 */}
          <div style={{
            height: 10, borderRadius: 5, overflow: "hidden",
            background: "#E8E8E8", display: "flex",
            border: "1px solid var(--amz-border)",
          }}>
            {fullPct > 0 && (
              <div style={{ width: `${fullPct}%`, background: POD_STATE_COLOR.full, transition: "width 0.5s" }} />
            )}
            {fillingPct > 0 && (
              <div style={{ width: `${fillingPct}%`, background: POD_STATE_COLOR.filling, transition: "width 0.5s" }} />
            )}
            {movingPct > 0 && (
              <div style={{ width: `${movingPct}%`, background: POD_STATE_COLOR.moving, transition: "width 0.5s" }} />
            )}
          </div>

          {/* 범례 */}
          <div style={{ display: "flex", gap: 10, marginTop: 6, flexWrap: "wrap" }}>
            {(["full", "filling", "moving", "empty"] as const).map((state) => (
              <div key={state} style={{ display: "flex", alignItems: "center", gap: 4 }}>
                <div style={{
                  width: 8, height: 8, borderRadius: 2,
                  background: POD_STATE_COLOR[state],
                  border: "1px solid #ccc",
                }} />
                <span style={{ fontSize: 11, color: "var(--amz-muted)" }}>
                  {state === "full" ? "가득" : state === "filling" ? "채우는 중" : state === "moving" ? "이동 중" : "비어있음"}
                  &nbsp;{podCounts[state] ?? 0}
                </span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
