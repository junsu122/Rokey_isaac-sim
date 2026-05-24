import React from "react";
import { BatteryBar } from "@/components/BatteryBar";
import { StatusBadge } from "@/components/StatusBadge";
import type { AMRData, DroneData, ArmData } from "@/types";

type RobotData = AMRData | DroneData | ArmData;

interface Props {
  data: RobotData | null;
  label: string;
  icon: string;
}

function isOnline(data: RobotData): boolean {
  if (!data.last_updated) return false;
  const lastSec = (data.last_updated as { seconds: number }).seconds;
  return Date.now() / 1000 - lastSec < 60;
}

function getStatusKey(data: RobotData): string {
  if (data.type === "arm") return (data as ArmData).status;
  return (data as AMRData | DroneData).charge_status;
}

function getCargoKey(data: RobotData): string | null {
  if (data.type === "arm") return null;
  return (data as AMRData | DroneData).cargo_status;
}

function getHeldItem(data: RobotData): string | null {
  if (data.type === "arm") {
    const item = (data as ArmData).detected_item;
    return item ? item.label : null;
  }
  const cargo = (data as AMRData | DroneData).cargo_status;
  if (cargo !== "empty") return data.current_task ?? "물품 적재됨";
  return null;
}

function getPositionLabel(data: RobotData): string {
  const p = data.position as { x: number; y: number; z?: number };
  if (data.type === "drone") {
    const d = data as DroneData;
    return `x:${p.x.toFixed(2)} y:${p.y.toFixed(2)} z:${d.altitude.toFixed(2)}m`;
  }
  return `x:${p.x.toFixed(2)} y:${p.y.toFixed(2)}`;
}

export function RobotCard({ data, label, icon }: Props) {
  const cardStyle: React.CSSProperties = {
    border: "1px solid var(--amz-border)",
    borderRadius: 4,
    padding: 14,
    display: "flex",
    flexDirection: "column",
    gap: 10,
    background: "#fff",
  };

  if (!data) {
    return (
      <div style={cardStyle}>
        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
          <span style={{ fontSize: 22 }}>{icon}</span>
          <span style={{ fontWeight: 600, color: "var(--amz-muted)" }}>{label}</span>
          <span style={{ marginLeft: "auto", width: 8, height: 8, borderRadius: "50%", background: "#ccc" }} />
        </div>
        <p style={{ color: "var(--amz-muted)", fontSize: 13 }}>데이터 없음</p>
      </div>
    );
  }

  const online = isOnline(data);
  const heldItem = getHeldItem(data);
  const statusKey = getStatusKey(data);
  const cargoKey = getCargoKey(data);

  return (
    <div style={cardStyle}>
      {/* Header */}
      <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
        <span style={{ fontSize: 22 }}>{icon}</span>
        <div>
          <p style={{ fontWeight: 700, color: "var(--amz-dark)", fontSize: 14, margin: 0 }}>{label}</p>
          <p style={{ fontSize: 11, color: "var(--amz-muted)", margin: 0 }}>{data.robot_id}</p>
        </div>
        <div style={{ marginLeft: "auto", display: "flex", alignItems: "center", gap: 4 }}>
          <span style={{
            width: 8, height: 8, borderRadius: "50%",
            background: online ? "#067D62" : "#ccc",
            boxShadow: online ? "0 0 5px #067D62" : "none",
          }} />
          <span style={{ fontSize: 11, color: online ? "#067D62" : "#aaa" }}>
            {online ? "온라인" : "오프라인"}
          </span>
        </div>
      </div>

      {/* Status badges */}
      <div style={{ display: "flex", flexWrap: "wrap", gap: 6 }}>
        <StatusBadge status={statusKey} />
        {cargoKey && <StatusBadge status={cargoKey} />}
        {data.type === "arm" && (
          <span style={{
            padding: "2px 8px", borderRadius: 3, fontSize: 11, fontWeight: 600,
            background: "#F3F3F3", color: "var(--amz-dark)",
            border: "1px solid var(--amz-border)"
          }}>
            그리퍼: {(data as ArmData).gripper === "open" ? "열림" : "닫힘"}
          </span>
        )}
      </div>

      {/* Battery */}
      <div>
        <p style={{ fontSize: 11, color: "var(--amz-muted)", marginBottom: 4 }}>배터리</p>
        <BatteryBar level={data.battery ?? 0} />
      </div>

      {/* Held item */}
      <div style={{ borderTop: "1px solid var(--amz-border)", paddingTop: 8 }}>
        <p style={{ fontSize: 11, color: "var(--amz-muted)", marginBottom: 2 }}>보유 물품</p>
        {heldItem ? (
          <span style={{ fontSize: 13, fontWeight: 600, color: "var(--amz-orange2)" }}>{heldItem}</span>
        ) : (
          <span style={{ fontSize: 13, color: "#aaa" }}>없음</span>
        )}
      </div>

      {/* Current task */}
      <div>
        <p style={{ fontSize: 11, color: "var(--amz-muted)", marginBottom: 2 }}>현재 작업</p>
        <span style={{ fontSize: 12, color: "var(--amz-link)", fontFamily: "monospace" }}>
          {data.current_task ?? "—"}
        </span>
      </div>

      {/* Position */}
      <div style={{ borderTop: "1px solid var(--amz-border)", paddingTop: 8 }}>
        <p style={{ fontSize: 11, color: "var(--amz-muted)", marginBottom: 2 }}>위치</p>
        <span style={{ fontSize: 11, fontFamily: "monospace", color: "var(--amz-dark)" }}>
          {getPositionLabel(data)}
        </span>
      </div>
    </div>
  );
}
