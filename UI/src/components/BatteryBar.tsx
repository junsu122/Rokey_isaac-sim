interface Props {
  level: number;
}

export function BatteryBar({ level }: Props) {
  const color =
    level > 50 ? "#067D62" :
    level > 20 ? "#F08804" :
    "#B00020";

  return (
    <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
      <div style={{
        flex: 1, height: 6, background: "#E8E8E8",
        borderRadius: 3, overflow: "hidden",
        border: "1px solid #D5D9D9"
      }}>
        <div style={{
          height: "100%", borderRadius: 3,
          background: color,
          width: `${Math.max(0, Math.min(100, level))}%`,
          transition: "width 0.5s ease",
        }} />
      </div>
      <span style={{ fontSize: 12, color: "#565959", width: 36, textAlign: "right" }}>
        {level.toFixed(0)}%
      </span>
    </div>
  );
}
