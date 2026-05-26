import type { SectionData } from "@/types";

// World range: x[-18, 24], y[-18, 18] → SVG viewBox 840×720
const W = 840, H = 720;
function wx(x: number) { return (x + 18) * 20; }
function wy(y: number) { return (18 - y) * 20; }

// 섹션 경계
const SECTIONS = [
  { id: "A", x0: -4.9, x1: 4.9, y0: 6.1,  y1: 13.9, fill: "#FF990022", stroke: "#FF9900" },
  { id: "B", x0: -4.9, x1: 4.9, y0: -3.9, y1:  3.9, fill: "#067D6222", stroke: "#067D62" },
  { id: "C", x0: -4.9, x1: 4.9, y0:-13.9, y1: -6.1, fill: "#8B5CF622", stroke: "#8B5CF6" },
];

// 조작실 (서측)
const MANIP_ROOMS = [
  { x0: -16.5, x1: -8.0, y0:  3.5, y1: 10.0 },
  { x0: -16.5, x1: -8.0, y0: -3.5, y1:  3.5 },
  { x0: -16.5, x1: -8.0, y0:-10.0, y1: -3.5 },
];

// 선별실 (동측)
const SORT_ROOMS = [
  { x0: 16.5, x1: 23.0, y0:  5.1, y1: 15.9 },
  { x0: 16.5, x1: 23.0, y0: -5.3, y1:  5.3 },
  { x0: 16.5, x1: 23.0, y0:-15.9, y1: -5.1 },
];

// M0609 고정 위치
const M0609_POS = [
  { name: "M0609_A", x: -12.07, y: 7.92 },
  { name: "M0609_B", x:  -9.45, y: 0.79 },
  { name: "M0609_C", x: -10.45, y: -7.80 },
];

// Pod 상태 색상
const POD_COLORS: Record<string, string> = {
  full:    "#FF9900",
  filling: "#F0C040",
  moving:  "#007185",
  empty:   "#D5D9D9",
};

interface Props {
  sections: SectionData[];
}

export function WarehouseMap({ sections }: Props) {
  return (
    <div style={{
      border: "1px solid var(--amz-border)",
      borderRadius: 4,
      background: "#fff",
      overflow: "hidden",
    }}>
      <div style={{
        padding: "8px 14px",
        borderBottom: "1px solid var(--amz-border)",
        background: "#F3F3F3",
        display: "flex", alignItems: "center", gap: 12,
      }}>
        <span style={{ fontWeight: 700, fontSize: 13, color: "var(--amz-dark)" }}>창고 맵</span>
        <div style={{ display: "flex", gap: 12, fontSize: 11, color: "var(--amz-muted)" }}>
          {[
            { color: "#FF9900", label: "가득" },
            { color: "#F0C040", label: "채우는 중" },
            { color: "#007185", label: "이동 중" },
            { color: "#D5D9D9", label: "비어있음" },
          ].map(({ color, label }) => (
            <span key={label} style={{ display: "flex", alignItems: "center", gap: 4 }}>
              <span style={{ width: 8, height: 8, background: color, border: "1px solid #aaa", borderRadius: 2, display: "inline-block" }} />
              {label}
            </span>
          ))}
          <span style={{ display: "flex", alignItems: "center", gap: 4 }}>
            <span style={{ width: 10, height: 10, background: "#00DCDC", borderRadius: "50%", display: "inline-block" }} />
            iw_hub
          </span>
          <span style={{ display: "flex", alignItems: "center", gap: 4 }}>
            <span style={{ width: 8, height: 8, background: "#C8643C", display: "inline-block" }} />
            M0609
          </span>
        </div>
      </div>

      <div style={{ padding: 12 }}>
        <svg
          viewBox={`0 0 ${W} ${H}`}
          style={{ width: "100%", height: "auto", display: "block" }}
        >
          {/* 배경 */}
          <rect x={0} y={0} width={W} height={H} fill="#1C1C1C" />

          {/* 그리드 */}
          {Array.from({ length: 22 }, (_, i) => -18 + i * 2).map((v) => (
            <g key={`g${v}`}>
              <line x1={wx(v)} y1={0} x2={wx(v)} y2={H} stroke="#333" strokeWidth={0.5} />
              <line x1={0} y1={wy(v)} x2={W} y2={wy(v)} stroke="#333" strokeWidth={0.5} />
            </g>
          ))}

          {/* 외벽 */}
          <rect
            x={wx(-16.65)} y={wy(16.65)}
            width={wx(16.65) - wx(-16.65)} height={wy(-16.65) - wy(16.65)}
            fill="none" stroke="#666" strokeWidth={2}
          />

          {/* 조작실 */}
          {MANIP_ROOMS.map((r, i) => (
            <rect key={i}
              x={wx(r.x0)} y={wy(r.y1)}
              width={wx(r.x1) - wx(r.x0)} height={wy(r.y0) - wy(r.y1)}
              fill="#2A2A3A" stroke="#444" strokeWidth={1}
            />
          ))}

          {/* 선별실 */}
          {SORT_ROOMS.map((r, i) => (
            <rect key={i}
              x={wx(r.x0)} y={wy(r.y1)}
              width={wx(r.x1) - wx(r.x0)} height={wy(r.y0) - wy(r.y1)}
              fill="#2A2A3A" stroke="#444" strokeWidth={1}
            />
          ))}

          {/* 섹션 구역 */}
          {SECTIONS.map((s) => (
            <g key={s.id}>
              <rect
                x={wx(s.x0)} y={wy(s.y1)}
                width={wx(s.x1) - wx(s.x0)} height={wy(s.y0) - wy(s.y1)}
                fill={s.fill} stroke={s.stroke} strokeWidth={2}
              />
              <text
                x={wx((s.x0 + s.x1) / 2)} y={wy((s.y0 + s.y1) / 2) + 5}
                textAnchor="middle" fill={s.stroke} fontSize={18} fontWeight="bold" opacity={0.6}
              >
                Sec {s.id}
              </text>
            </g>
          ))}

          {/* Pod 위치 (Firebase) */}
          {sections.flatMap((sec) =>
            sec.pods.map((pod) => {
              const px = wx(pod.location?.x ?? 0);
              const py = wy(pod.location?.y ?? 0);
              const col = POD_COLORS[pod.state] ?? "#888";
              return (
                <g key={`${sec.section_id}-${pod.pod_id}`}>
                  <rect x={px - 7} y={py - 7} width={14} height={14}
                    fill={col} stroke="#fff" strokeWidth={0.5} opacity={0.9} />
                  <title>{pod.pod_id} [{pod.state}] ({pod.location?.x?.toFixed(1)}, {pod.location?.y?.toFixed(1)})</title>
                </g>
              );
            })
          )}

          {/* M0609 고정 위치 */}
          {M0609_POS.map((r) => (
            <g key={r.name}>
              <rect x={wx(r.x) - 8} y={wy(r.y) - 8} width={16} height={16}
                fill="#C8643C" stroke="#FF9900" strokeWidth={1.5} />
              <text x={wx(r.x)} y={wy(r.y) + 20}
                textAnchor="middle" fill="#C8643C" fontSize={9} fontWeight="bold">
                {r.name.split("_")[1]}
              </text>
            </g>
          ))}

          {/* iw_hub 실시간 위치 (Firebase) */}
          {sections.map((sec) => {
            const hub = sec.robots?.iw_hub;
            if (!hub?.location) return null;
            const px = wx(hub.location.x);
            const py = wy(hub.location.y);
            const isWorking = hub.state === "working";
            return (
              <g key={sec.section_id}>
                <circle cx={px} cy={py} r={10} fill="#00DCDC" stroke="#fff" strokeWidth={1.5} opacity={0.9} />
                {isWorking && (
                  <circle cx={px} cy={py} r={13} fill="none" stroke="#00DCDC" strokeWidth={1} opacity={0.5}>
                    <animate attributeName="r" from="10" to="18" dur="1.2s" repeatCount="indefinite" />
                    <animate attributeName="opacity" from="0.5" to="0" dur="1.2s" repeatCount="indefinite" />
                  </circle>
                )}
                <text x={px} y={py + 22}
                  textAnchor="middle" fill="#00DCDC" fontSize={9} fontWeight="bold">
                  {hub.robot_name}
                </text>
                <title>{hub.robot_name} [{hub.state}] ({hub.location.x.toFixed(2)}, {hub.location.y.toFixed(2)})</title>
              </g>
            );
          })}

          {/* 원점 축 */}
          <line x1={wx(0)} y1={wy(0)} x2={wx(2)} y2={wy(0)} stroke="#4444CC" strokeWidth={2} markerEnd="url(#arrowX)" />
          <line x1={wx(0)} y1={wy(0)} x2={wx(0)} y2={wy(2)} stroke="#44AA44" strokeWidth={2} markerEnd="url(#arrowY)" />
          <text x={wx(2) + 4} y={wy(0) + 4} fill="#6666EE" fontSize={10}>X</text>
          <text x={wx(0) + 4} y={wy(2) - 4} fill="#44AA44" fontSize={10}>Y</text>

          <defs>
            <marker id="arrowX" markerWidth="6" markerHeight="6" refX="3" refY="3" orient="auto">
              <path d="M0,0 L6,3 L0,6 Z" fill="#4444CC" />
            </marker>
            <marker id="arrowY" markerWidth="6" markerHeight="6" refX="3" refY="3" orient="auto">
              <path d="M0,0 L6,3 L0,6 Z" fill="#44AA44" />
            </marker>
          </defs>
        </svg>
      </div>
    </div>
  );
}
