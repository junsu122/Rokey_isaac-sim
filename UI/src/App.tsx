import { useState, useEffect } from "react";
import { useSections } from "@/hooks/useSections";
import { SectionCard } from "@/components/SectionCard";
import { PodPanel } from "@/components/PodPanel";
import { AmazonLogo } from "@/components/AmazonLogo";

export default function App() {
  const sections = useSections();
  const [time, setTime] = useState(new Date());

  useEffect(() => {
    const t = setInterval(() => setTime(new Date()), 1000);
    return () => clearInterval(t);
  }, []);

  return (
    <div style={{ minHeight: "100vh", backgroundColor: "var(--amz-bg)" }}>

      {/* 상단 네비게이션 바 */}
      <header style={{ backgroundColor: "var(--amz-dark)" }}>
        <div style={{
          display: "flex", alignItems: "center", gap: "24px",
          padding: "10px 20px", borderBottom: "1px solid var(--amz-mid)"
        }}>
          <AmazonLogo />

          <div style={{ color: "#ccc", fontSize: 12, lineHeight: 1.3 }}>
            <div style={{ fontSize: 10, color: "#aaa" }}>물류센터</div>
            <div style={{ fontWeight: 700, color: "#fff", fontSize: 13 }}>인천 물류허브</div>
          </div>

          <div style={{
            flex: 1, display: "flex", alignItems: "center",
            background: "#fff", borderRadius: 4, overflow: "hidden",
            border: "2px solid var(--amz-orange)"
          }}>
            <div style={{
              background: "#F3F3F3", padding: "6px 10px",
              fontSize: 12, color: "#555", borderRight: "1px solid #ccc",
              whiteSpace: "nowrap"
            }}>
              전체
            </div>
            <div style={{ flex: 1, padding: "6px 12px", color: "#999", fontSize: 13 }}>
              섹션 / 로봇 / Pod 검색...
            </div>
            <div style={{
              background: "var(--amz-orange)", padding: "6px 14px",
              display: "flex", alignItems: "center"
            }}>
              <svg width="18" height="18" viewBox="0 0 24 24" fill="#111">
                <path d="M21 21l-4.35-4.35M17 11A6 6 0 1 1 5 11a6 6 0 0 1 12 0z"
                  stroke="#111" strokeWidth="2.5" strokeLinecap="round" fill="none"/>
              </svg>
            </div>
          </div>

          <div style={{ display: "flex", gap: "16px", color: "#fff" }}>
            <div style={{ fontSize: 12, lineHeight: 1.4, cursor: "pointer" }}>
              <div style={{ fontSize: 10, color: "#ccc" }}>안녕하세요</div>
              <div style={{ fontWeight: 700 }}>관리자 ▾</div>
            </div>
            <div style={{ fontSize: 12, lineHeight: 1.4, cursor: "pointer" }}>
              <div style={{ fontSize: 10, color: "#ccc" }}>로봇 상태</div>
              <div style={{ fontWeight: 700 }}>모니터링 ▾</div>
            </div>
          </div>
        </div>

        {/* 서브 네비 */}
        <div style={{
          display: "flex", alignItems: "center", gap: "4px",
          padding: "6px 20px", backgroundColor: "var(--amz-mid)"
        }}>
          {["전체 현황", "섹션 관리", "Pod 현황", "로봇 제어"].map((menu) => (
            <div key={menu} style={{
              color: "#fff", fontSize: 13, padding: "4px 10px",
              borderRadius: 2, cursor: "pointer", whiteSpace: "nowrap",
              border: "1px solid transparent",
            }}
              onMouseEnter={e => (e.currentTarget.style.border = "1px solid #fff")}
              onMouseLeave={e => (e.currentTarget.style.border = "1px solid transparent")}
            >
              {menu}
            </div>
          ))}
          <div style={{ marginLeft: "auto", color: "var(--amz-orange)", fontSize: 12, fontWeight: 700 }}>
            ● LIVE&nbsp;&nbsp;{time.toLocaleTimeString("ko-KR")}
          </div>
        </div>
      </header>

      {/* 메인 콘텐츠 */}
      <main style={{ padding: "16px 20px", maxWidth: 1400, margin: "0 auto" }}>

        {/* 섹션 카드 3개 */}
        <section style={{ marginBottom: 16 }}>
          <div style={{
            display: "grid",
            gridTemplateColumns: "repeat(3, 1fr)",
            gap: 12,
          }}>
            {sections.length === 0 ? (
              <div style={{
                gridColumn: "1 / -1",
                padding: 24, textAlign: "center",
                color: "var(--amz-muted)", fontSize: 14,
                background: "#fff", border: "1px solid var(--amz-border)", borderRadius: 4,
              }}>
                Firebase 연결 중...
              </div>
            ) : (
              sections.map((s) => <SectionCard key={s.section_id} section={s} />)
            )}
          </div>
        </section>

        {/* Pod 현황 */}
        <section>
          <PodPanel sections={sections} />
        </section>

      </main>

      {/* 푸터 */}
      <footer style={{
        marginTop: 40, backgroundColor: "var(--amz-dark)",
        padding: "20px", textAlign: "center",
        color: "#aaa", fontSize: 12
      }}>
        © 2026 Amaezon Robotics Logistics — Powered by Firebase &amp; ROS2
      </footer>
    </div>
  );
}
