interface Props {
  status: string;
}

const STATUS_CONFIG: Record<string, { label: string; bg: string; color: string }> = {
  // 로봇 상태
  working:       { label: "작동 중",  bg: "#E6F4EA", color: "#067D62" },
  stop:          { label: "정지",     bg: "#F3F3F3", color: "#555" },
  // Pod 상태
  full:          { label: "가득",     bg: "#FFE8CC", color: "#9A4700" },
  filling:       { label: "채우는 중",bg: "#FFF3CD", color: "#856404" },
  moving:        { label: "이동 중",  bg: "#E8F4FD", color: "#007185" },
  empty:         { label: "비어있음", bg: "#F3F3F3", color: "#aaa" },
  // 기타 (하위 호환)
  idle:          { label: "대기",     bg: "#F3F3F3", color: "#555" },
  error:         { label: "오류",     bg: "#FDECEA", color: "#B00020" },
};

export function StatusBadge({ status }: Props) {
  const cfg = STATUS_CONFIG[status] ?? { label: status, bg: "#F3F3F3", color: "#555" };
  return (
    <span style={{
      display: "inline-block",
      padding: "2px 8px",
      borderRadius: 3,
      fontSize: 11,
      fontWeight: 700,
      background: cfg.bg,
      color: cfg.color,
      border: `1px solid ${cfg.color}33`,
    }}>
      {cfg.label}
    </span>
  );
}
