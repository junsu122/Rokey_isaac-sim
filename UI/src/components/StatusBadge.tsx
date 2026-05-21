interface Props {
  status: string;
}

const STATUS_CONFIG: Record<string, { label: string; bg: string; color: string }> = {
  idle:          { label: "대기",     bg: "#F3F3F3", color: "#555" },
  moving:        { label: "이동 중",  bg: "#E8F4FD", color: "#007185" },
  picking:       { label: "픽업",     bg: "#FFF3CD", color: "#856404" },
  placing:       { label: "배치",     bg: "#FFE8CC", color: "#9A4700" },
  error:         { label: "오류",     bg: "#FDECEA", color: "#B00020" },
  charging:      { label: "충전 중",  bg: "#E6F4EA", color: "#067D62" },
  operating:     { label: "운행",     bg: "#E8F4FD", color: "#007185" },
  empty:         { label: "빈 카트",  bg: "#F3F3F3", color: "#555" },
  loading:       { label: "수납 중",  bg: "#FFF3CD", color: "#856404" },
  transporting:  { label: "운반 중",  bg: "#E8F4FD", color: "#007185" },
  unloading:     { label: "하역 중",  bg: "#FFE8CC", color: "#9A4700" },
  taking_off:    { label: "이륙",     bg: "#F3E8FD", color: "#6200EA" },
  flying:        { label: "비행",     bg: "#EDE7F6", color: "#4527A0" },
  hovering:      { label: "호버링",   bg: "#E8EAF6", color: "#283593" },
  landing:       { label: "착륙",     bg: "#F3E8FD", color: "#6200EA" },
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
