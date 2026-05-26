import { StatusBadge } from "@/components/StatusBadge";
import type { SectionData } from "@/types";

const SECTION_COLOR: Record<string, string> = {
  A: "#FF9900",
  B: "#067D62",
  C: "#8B5CF6",
};

const PACKAGE_LABEL: Record<string, string> = {
  Big: "대형", Medium: "중형", Small: "소형",
};

interface Props {
  sections: SectionData[];
}

export function RobotPanel({ sections }: Props) {
  return (
    <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
      {sections.map((sec) => {
        const col = SECTION_COLOR[sec.section_id] ?? "#888";
        const hub = sec.robots?.iw_hub;
        const arm = sec.robots?.m0609;

        return (
          <div key={sec.section_id} style={{
            border: `1px solid ${col}55`,
            borderLeft: `4px solid ${col}`,
            borderRadius: 4,
            background: "#fff",
            overflow: "hidden",
          }}>
            {/* 섹션 헤더 */}
            <div style={{
              padding: "8px 14px",
              borderBottom: "1px solid var(--amz-border)",
              background: "#F8F8F8",
              display: "flex", alignItems: "center", gap: 10,
            }}>
              <span style={{ fontSize: 16, fontWeight: 900, color: col }}>
                Section {sec.section_id}
              </span>
              <span style={{
                fontSize: 11, fontWeight: 700,
                color: col,
                background: `${col}18`,
                border: `1px solid ${col}44`,
                padding: "1px 7px", borderRadius: 3,
              }}>
                {PACKAGE_LABEL[sec.package_size] ?? sec.package_size}
              </span>
              <span style={{
                marginLeft: "auto",
                fontSize: 11, color: "var(--amz-muted)"
              }}>
                Pod {sec.pods.length}개 |&nbsp;
                가득 {sec.pods.filter(p => p.state === "full").length} /
                채우는 중 {sec.pods.filter(p => p.state === "filling").length} /
                이동 중 {sec.pods.filter(p => p.state === "moving").length} /
                빈 것 {sec.pods.filter(p => p.state === "empty").length}
              </span>
            </div>

            {/* 로봇 카드 2개 */}
            <div style={{
              display: "grid", gridTemplateColumns: "1fr 1fr",
              gap: 0,
            }}>
              {/* M0609 */}
              <div style={{
                padding: "12px 16px",
                borderRight: "1px solid var(--amz-border)",
                display: "flex", alignItems: "flex-start", gap: 12,
              }}>
                <div style={{
                  width: 40, height: 40, borderRadius: 6,
                  background: "#FFF0E6",
                  border: "1px solid #C8643C44",
                  display: "flex", alignItems: "center", justifyContent: "center",
                  fontSize: 20, flexShrink: 0,
                }}>
                  🦾
                </div>
                <div style={{ flex: 1 }}>
                  <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 4 }}>
                    <span style={{ fontWeight: 700, fontSize: 13, color: "var(--amz-dark)" }}>M0609</span>
                    <StatusBadge status={arm?.state ?? "stop"} />
                  </div>
                  <div style={{ fontSize: 11, color: "var(--amz-muted)", fontFamily: "monospace" }}>
                    {arm?.robot_name ?? "—"}
                  </div>
                  <div style={{ fontSize: 11, color: "var(--amz-muted)", marginTop: 4 }}>
                    협동 로봇 팔 · 흡착 그리퍼
                  </div>
                </div>
              </div>

              {/* iw_hub */}
              <div style={{
                padding: "12px 16px",
                display: "flex", alignItems: "flex-start", gap: 12,
              }}>
                <div style={{
                  width: 40, height: 40, borderRadius: 6,
                  background: "#E6F9F9",
                  border: "1px solid #00DCDC44",
                  display: "flex", alignItems: "center", justifyContent: "center",
                  fontSize: 20, flexShrink: 0,
                }}>
                  🚗
                </div>
                <div style={{ flex: 1 }}>
                  <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 4 }}>
                    <span style={{ fontWeight: 700, fontSize: 13, color: "var(--amz-dark)" }}>iw_hub</span>
                    <StatusBadge status={hub?.state ?? "stop"} />
                  </div>
                  <div style={{ fontSize: 11, color: "var(--amz-muted)", fontFamily: "monospace" }}>
                    {hub?.robot_name ?? "—"}
                  </div>
                  {hub?.location && (
                    <div style={{
                      marginTop: 4,
                      display: "inline-flex", gap: 6,
                      fontSize: 11, fontFamily: "monospace",
                      background: "#F0FAFA", border: "1px solid #00DCDC44",
                      borderRadius: 3, padding: "2px 6px",
                      color: "#007185",
                    }}>
                      <span>x: {hub.location.x.toFixed(2)}</span>
                      <span style={{ color: "#ccc" }}>|</span>
                      <span>y: {hub.location.y.toFixed(2)}</span>
                    </div>
                  )}
                </div>
              </div>
            </div>
          </div>
        );
      })}

      {sections.length === 0 && (
        <div style={{
          padding: 24, textAlign: "center",
          color: "var(--amz-muted)", fontSize: 14,
          background: "#fff", border: "1px solid var(--amz-border)", borderRadius: 4,
        }}>
          Firebase 연결 중...
        </div>
      )}
    </div>
  );
}
