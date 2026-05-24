import type { ProductData, ItemData } from "@/types";

interface Props {
  products: ProductData[];
  items: ItemData[];
}

const DEST_COLOR: Record<string, string> = {
  Gangnam:        "#9A4700",
  Seocho:         "#067D62",
  "Guro Digital": "#4527A0",
};

const STATUS_STYLE: Record<string, { label: string; bg: string; color: string }> = {
  detected:   { label: "인식됨",   bg: "#FFF3CD", color: "#856404" },
  in_transit: { label: "운반 중",  bg: "#E8F4FD", color: "#007185" },
  delivered:  { label: "배송 완료",bg: "#E6F4EA", color: "#067D62" },
  returned:   { label: "반품",     bg: "#FDECEA", color: "#B00020" },
  waiting:    { label: "대기",     bg: "#F3F3F3", color: "#555" },
  registered: { label: "등록",     bg: "#F3F3F3", color: "#555" },
};

function StatusBadge({ status }: { status: string }) {
  const s = STATUS_STYLE[status] ?? { label: status, bg: "#F3F3F3", color: "#555" };
  return (
    <span style={{
      padding: "2px 7px", borderRadius: 3, fontSize: 11, fontWeight: 700,
      background: s.bg, color: s.color, border: `1px solid ${s.color}33`,
    }}>
      {s.label}
    </span>
  );
}

function formatTime(ts: { seconds: number } | null): string {
  if (!ts) return "—";
  return new Date(ts.seconds * 1000).toLocaleTimeString("ko-KR", {
    hour: "2-digit", minute: "2-digit", second: "2-digit",
  });
}

const ALL_SECTIONS = ["A-1", "A-2", "A-3", "B-1", "B-2"];

export function InventoryPanel({ products, items }: Props) {
  // 물품별 구획별 수량 (delivered/returned 제외)
  const sectionsByProduct: Record<string, Record<string, number>> = {};
  for (const p of products) {
    sectionsByProduct[p.name] = {};
    for (const s of ALL_SECTIONS) sectionsByProduct[p.name][s] = 0;
  }
  for (const item of items) {
    if (item.status === "delivered" || item.status === "returned") continue;
    if (!sectionsByProduct[item.name]) continue;
    sectionsByProduct[item.name][item.section] =
      (sectionsByProduct[item.name][item.section] ?? 0) + 1;
  }

  const activeItems     = items.filter((i) => i.status === "detected" || i.status === "in_transit");
  const recentDelivered = items.filter((i) => i.status === "delivered").slice(0, 5);

  const boxStyle: React.CSSProperties = {
    border: "1px solid var(--amz-border)",
    borderRadius: 4,
    overflow: "hidden",
    marginBottom: 12,
    background: "#fff",
  };
  const headStyle: React.CSSProperties = {
    padding: "7px 14px",
    borderBottom: "1px solid var(--amz-border)",
    fontSize: 13,
    fontWeight: 700,
    color: "var(--amz-dark)",
    background: "#F3F3F3",
  };

  return (
    <div>
      {/* 물품 재고 현황 */}
      <div style={boxStyle}>
        <div style={headStyle}>물품 재고 현황 (선반 기준)</div>
        <div style={{
          padding: 12,
          display: "grid",
          gridTemplateColumns: "repeat(auto-fill, minmax(180px, 1fr))",
          gap: 10,
        }}>
          {products.map((p) => {
            const secCounts = sectionsByProduct[p.name] ?? {};
            const total = ALL_SECTIONS.reduce((s, k) => s + (secCounts[k] ?? 0), 0);
            return (
              <div key={p.product_id} style={{
                border: "1px solid var(--amz-border)",
                borderRadius: 4,
                padding: "10px 12px",
                background: "#FAFAFA",
              }}>
                {/* 물품명 + 마커 */}
                <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 8 }}>
                  <span style={{ fontWeight: 700, fontSize: 13, color: "var(--amz-dark)" }}>{p.name}</span>
                  <span style={{ fontSize: 11, color: "#aaa" }}>#{p.marker_id}</span>
                </div>

                {/* 구획별 수량 */}
                <div style={{ display: "flex", flexDirection: "column", gap: 5 }}>
                  {ALL_SECTIONS.map((s) => {
                    const cnt = secCounts[s] ?? 0;
                    return (
                      <div key={s} style={{ display: "flex", alignItems: "center", gap: 6 }}>
                        <span style={{ fontSize: 11, color: "var(--amz-link)", width: 30 }}>{s}</span>
                        <div style={{
                          flex: 1, height: 6, background: "#E8E8E8",
                          borderRadius: 3, overflow: "hidden",
                        }}>
                          <div style={{
                            height: "100%", borderRadius: 3,
                            background: cnt > 0 ? "var(--amz-orange)" : "#E8E8E8",
                            width: cnt > 0 ? `${Math.min(100, cnt * 30)}%` : "0%",
                            transition: "width 0.4s ease",
                          }} />
                        </div>
                        <span style={{
                          fontSize: 12, fontWeight: 700, width: 18, textAlign: "right",
                          color: cnt > 0 ? "var(--amz-dark)" : "#ccc",
                        }}>
                          {cnt}
                        </span>
                      </div>
                    );
                  })}
                </div>

                {/* 합계 */}
                <div style={{
                  marginTop: 8, paddingTop: 6,
                  borderTop: "1px solid var(--amz-border)",
                  fontSize: 11, color: "var(--amz-muted)",
                  display: "flex", justifyContent: "space-between",
                }}>
                  <span>총 재고</span>
                  <span style={{ fontWeight: 700, color: total > 0 ? "var(--amz-dark)" : "#ccc" }}>
                    {total}개
                  </span>
                </div>
              </div>
            );
          })}
        </div>
      </div>

      {/* 진행 중인 배송 */}
      <div style={boxStyle}>
        <div style={{ ...headStyle, display: "flex", alignItems: "center", gap: 8 }}>
          진행 중인 배송
          {activeItems.length > 0 && (
            <span style={{
              background: "var(--amz-orange)", color: "#111",
              borderRadius: 10, padding: "1px 8px", fontSize: 11, fontWeight: 700,
            }}>
              {activeItems.length}
            </span>
          )}
        </div>
        <div style={{ padding: 12 }}>
          {activeItems.length === 0 ? (
            <p style={{ color: "var(--amz-muted)", fontSize: 13 }}>진행 중인 배송 없음</p>
          ) : (
            <div style={{ overflowX: "auto" }}>
              <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 13 }}>
                <thead>
                  <tr style={{ borderBottom: "2px solid var(--amz-border)" }}>
                    {["물품", "구획", "배송지", "담당 로봇", "상태", "인식 시각"].map((h) => (
                      <th key={h} style={{
                        textAlign: "left", padding: "6px 12px 6px 0",
                        fontSize: 12, color: "var(--amz-muted)", fontWeight: 700,
                      }}>{h}</th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  {activeItems.map((item, i) => (
                    <tr key={item.item_id} style={{
                      borderBottom: "1px solid var(--amz-border)",
                      background: i % 2 === 0 ? "#fff" : "#FAFAFA",
                    }}>
                      <td style={{ padding: "8px 12px 8px 0", fontWeight: 600 }}>{item.name}</td>
                      <td style={{ padding: "8px 12px 8px 0", color: "var(--amz-link)" }}>{item.section}</td>
                      <td style={{ padding: "8px 12px 8px 0", fontWeight: 600, color: DEST_COLOR[item.destination] ?? "var(--amz-dark)" }}>
                        {item.destination}
                      </td>
                      <td style={{ padding: "8px 12px 8px 0", fontFamily: "monospace", fontSize: 11, color: "var(--amz-muted)" }}>
                        {item.assigned_robot ?? "—"}
                      </td>
                      <td style={{ padding: "8px 12px 8px 0" }}>
                        <StatusBadge status={item.status} />
                      </td>
                      <td style={{ padding: "8px 0", color: "var(--amz-muted)", fontSize: 12 }}>
                        {formatTime(item.detected_at)}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </div>
      </div>

      {/* 최근 배송 완료 */}
      <div style={boxStyle}>
        <div style={headStyle}>최근 배송 완료</div>
        <div style={{ padding: 12 }}>
          {recentDelivered.length === 0 ? (
            <p style={{ color: "var(--amz-muted)", fontSize: 13 }}>배송 완료 내역 없음</p>
          ) : (
            <div style={{ display: "flex", flexDirection: "column", gap: 6 }}>
              {recentDelivered.map((item) => (
                <div key={item.item_id} style={{
                  display: "flex", alignItems: "center", justifyContent: "space-between",
                  padding: "8px 12px",
                  background: "#F6FFF8",
                  border: "1px solid #C6EFD1",
                  borderRadius: 4,
                }}>
                  <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
                    <span style={{ color: "#067D62", fontSize: 16 }}>✓</span>
                    <span style={{ fontWeight: 600, fontSize: 13 }}>{item.name}</span>
                    <span style={{
                      fontSize: 12, fontWeight: 600,
                      color: DEST_COLOR[item.destination] ?? "var(--amz-dark)"
                    }}>
                      → {item.destination}
                    </span>
                  </div>
                  <span style={{ fontSize: 12, color: "var(--amz-muted)" }}>
                    {formatTime(item.delivered_at)}
                  </span>
                </div>
              ))}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
