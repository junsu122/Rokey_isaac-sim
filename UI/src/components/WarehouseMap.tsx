import type { AMRData, DroneData, ArmData } from "@/types";

interface Props {
  amr: AMRData | null;
  drone: DroneData | null;
  arm: ArmData | null;
}

const WORLD = { minX: -0.8, maxX: 2.0, minY: -0.8, maxY: 0.8 };
const SVG_W = 700;
const SVG_H = 360;
const PAD = 44;

const AMZ = {
  dark:    "#232F3E",
  mid:     "#37475A",
  orange:  "#FF9900",
  orange2: "#F08804",
  bg:      "#F3F3F3",
  card:    "#FFFFFF",
  border:  "#D5D9D9",
  link:    "#007185",
  muted:   "#888",
  green:   "#067D62",
};

function toSvg(x: number, y: number): [number, number] {
  const sx = PAD + ((x - WORLD.minX) / (WORLD.maxX - WORLD.minX)) * (SVG_W - PAD * 2);
  const sy = PAD + ((WORLD.maxY - y) / (WORLD.maxY - WORLD.minY)) * (SVG_H - PAD * 2);
  return [sx, sy];
}

const SECTIONS = [
  { id: "A-1", x: -0.4, y:  0.3 },
  { id: "A-2", x:  0.0, y:  0.3 },
  { id: "A-3", x:  0.4, y:  0.3 },
  { id: "B-1", x: -0.4, y: -0.3 },
  { id: "B-2", x:  0.0, y: -0.3 },
];

const DESTINATIONS = [
  { id: "강남", x: 1.5, y:  0.5, color: "#9A4700" },
  { id: "서초", x: 1.5, y:  0.0, color: AMZ.green },
  { id: "구로", x: 1.5, y: -0.5, color: "#4527A0" },
];

const ARM_WORLD = { x: 1.0, y: 0.0 };

// 암 → 각 배송지 컨베이어 벨트
const BELT_DATA = [
  { id: "gangnam", color: "#9A4700", destX: 1.5, destY:  0.5 },
  { id: "seocho",  color: AMZ.green, destX: 1.5, destY:  0.0 },
  { id: "guro",    color: "#4527A0", destX: 1.5, destY: -0.5 },
];

interface RobotIconProps {
  x: number;
  y: number;
  color: string;
  label: string;
  emoji: string;
  yaw?: number;
}

function RobotIcon({ x, y, color, label, emoji, yaw = 0 }: RobotIconProps) {
  const [sx, sy] = toSvg(x, y);
  return (
    <g transform={`translate(${sx}, ${sy})`}>
      <circle r={15} fill="#00000015" transform="translate(2,2)" />
      <circle r={15} fill="#fff" stroke={color} strokeWidth={2} />
      <text
        textAnchor="middle"
        dominantBaseline="central"
        fontSize={14}
        transform={`rotate(${-yaw})`}
      >
        {emoji}
      </text>
      <circle r={15} fill="none" stroke={color} strokeWidth={1.5} opacity={0.6}>
        <animate attributeName="r" values="15;24;15" dur="2s" repeatCount="indefinite" />
        <animate attributeName="opacity" values="0.6;0;0.6" dur="2s" repeatCount="indefinite" />
      </circle>
      <rect x={-22} y={18} width={44} height={14} rx={3} fill={color} />
      <text y={28} textAnchor="middle" fontSize={8} fill="#fff" fontWeight="bold">
        {label}
      </text>
    </g>
  );
}

interface BeltProps {
  id: string;
  color: string;
  x1: number;
  y1: number;
  x2: number;
  y2: number;
}

function ConveyorBelt({ color, x1, y1, x2, y2 }: BeltProps) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const len = Math.sqrt(dx * dx + dy * dy);
  const angle = Math.atan2(dy, dx) * 180 / Math.PI;

  return (
    <g transform={`translate(${x1}, ${y1}) rotate(${angle})`}>
      {/* 트랙 베이스 */}
      <rect x={0} y={-8} width={len} height={16} rx={4} fill="#CACACA" />
      {/* 벨트 표면 */}
      <rect x={0} y={-6} width={len} height={12} rx={3}
        fill={color + "1A"} stroke={color} strokeWidth={1.2} />
      {/* 이동 스트라이프 (암→배송지 방향) */}
      <line x1={0} y1={0} x2={len} y2={0}
        stroke={color} strokeWidth={5} strokeDasharray="10 14" opacity={0.5}>
        <animate attributeName="stroke-dashoffset"
          values="0;-24" dur="0.65s" repeatCount="indefinite" />
      </line>
      {/* 벨트 테두리 강조선 */}
      <line x1={0} y1={-6} x2={len} y2={-6} stroke={color} strokeWidth={0.6} opacity={0.4} />
      <line x1={0} y1={6}  x2={len} y2={6}  stroke={color} strokeWidth={0.6} opacity={0.4} />
    </g>
  );
}

export function WarehouseMap({ amr, drone, arm }: Props) {
  const [armSx, armSy] = toSvg(ARM_WORLD.x, ARM_WORLD.y);

  return (
    <svg
      viewBox={`0 0 ${SVG_W} ${SVG_H}`}
      style={{ width: "100%", maxHeight: 380, background: AMZ.bg, borderRadius: 4, display: "block" }}
    >
      <defs>
        <pattern id="amz-grid" width="36" height="36" patternUnits="userSpaceOnUse">
          <path d="M 36 0 L 0 0 0 36" fill="none" stroke="#D5D9D9" strokeWidth="0.8" />
        </pattern>
        <filter id="shadow" x="-10%" y="-10%" width="120%" height="130%">
          <feDropShadow dx="1" dy="2" stdDeviation="2" floodOpacity="0.12" />
        </filter>
      </defs>

      {/* 배경 */}
      <rect width={SVG_W} height={SVG_H} fill={AMZ.bg} />
      <rect width={SVG_W} height={SVG_H} fill="url(#amz-grid)" />

      {/* Row A / B 레이블 */}
      {[{ label: "Row A", y: 0.3 }, { label: "Row B", y: -0.3 }].map(({ label, y }) => {
        const [, sy] = toSvg(-0.8, y);
        return (
          <text key={label} x={10} y={sy + 4} fontSize={10} fill={AMZ.muted} fontWeight="700">
            {label}
          </text>
        );
      })}

      {/* 배송지 구역 레이블 */}
      {(() => {
        const [cx] = toSvg(1.5, 0);
        return (
          <text x={cx} y={PAD - 14} textAnchor="middle" fontSize={10} fill={AMZ.muted} fontWeight="700">
            배송지
          </text>
        );
      })()}

      {/* 섹션 박스 */}
      {SECTIONS.map((s) => {
        const [cx, cy] = toSvg(s.x, s.y);
        const bw = 76, bh = 46;
        return (
          <g key={s.id} filter="url(#shadow)">
            <rect x={cx - bw / 2} y={cy - bh / 2} width={bw} height={bh} rx={4}
              fill={AMZ.card} stroke={AMZ.border} strokeWidth={1.5} />
            <rect x={cx - bw / 2} y={cy - bh / 2} width={bw} height={5} rx={4} fill={AMZ.orange} />
            <rect x={cx - bw / 2} y={cy - bh / 2 + 2} width={bw} height={3} fill={AMZ.orange} />
            <text x={cx} y={cy - 4} textAnchor="middle" fontSize={12} fill={AMZ.dark} fontWeight="800">
              {s.id}
            </text>
            <text x={cx} y={cy + 10} textAnchor="middle" fontSize={9} fill={AMZ.muted}>
              선반
            </text>
          </g>
        );
      })}

      {/* 컨베이어 벨트 (암 오른쪽 → 배송지 왼쪽) */}
      {BELT_DATA.map((belt) => {
        const [destSx, destSy] = toSvg(belt.destX, belt.destY);
        return (
          <ConveyorBelt
            key={belt.id}
            id={belt.id}
            color={belt.color}
            x1={armSx + 24}
            y1={armSy}
            x2={destSx - 38}
            y2={destSy}
          />
        );
      })}

      {/* 배송지 존 */}
      {DESTINATIONS.map((d) => {
        const [cx, cy] = toSvg(d.x, d.y);
        return (
          <g key={d.id}>
            <rect x={cx - 38} y={cy - 24} width={76} height={48} rx={4}
              fill={d.color + "12"} stroke={d.color} strokeWidth={1.5} strokeDasharray="6 3" />
            <text x={cx - 14} y={cy + 5} fontSize={14} textAnchor="middle">📦</text>
            <text x={cx + 14} y={cy + 5} fontSize={10} fill={d.color} fontWeight="800" textAnchor="middle">
              {d.id}
            </text>
          </g>
        );
      })}

      {/* 로봇 암 */}
      {(() => {
        const [cx, cy] = toSvg(ARM_WORLD.x, ARM_WORLD.y);
        return (
          <g filter="url(#shadow)">
            <rect x={cx - 24} y={cy - 26} width={48} height={48} rx={4}
              fill={AMZ.card} stroke={AMZ.orange} strokeWidth={2} />
            <rect x={cx - 24} y={cy - 26} width={48} height={5} rx={4} fill={AMZ.orange} />
            <rect x={cx - 24} y={cy - 24} width={48} height={3} fill={AMZ.orange} />
            <text x={cx} y={cy - 4} textAnchor="middle" fontSize={16}>🦾</text>
            <text x={cx} y={cy + 10} textAnchor="middle" fontSize={8} fill={AMZ.dark} fontWeight="700">
              M0609
            </text>
            {arm && (
              <text x={cx} y={cy + 20} textAnchor="middle" fontSize={7} fill={AMZ.muted}>
                {arm.status}
              </text>
            )}
          </g>
        );
      })()}

      {/* AMR */}
      {amr && (
        <RobotIcon
          x={amr.position.x} y={amr.position.y}
          yaw={amr.position.yaw ?? 0}
          color={AMZ.link} label="AMR" emoji="🚗"
        />
      )}

      {/* Drone */}
      {drone && (
        <RobotIcon
          x={drone.position.x} y={drone.position.y}
          color={AMZ.orange2}
          label={`드론 ${drone.altitude.toFixed(1)}m`}
          emoji="🚁"
        />
      )}

      {/* 범례 */}
      <g transform={`translate(12, ${SVG_H - 22})`}>
        <circle cx={6} cy={6} r={5} fill="#fff" stroke={AMZ.link} strokeWidth={1.5} />
        <text x={14} y={10} fontSize={9} fill={AMZ.muted}>AMR</text>

        <circle cx={52} cy={6} r={5} fill="#fff" stroke={AMZ.orange2} strokeWidth={1.5} />
        <text x={60} y={10} fontSize={9} fill={AMZ.muted}>드론</text>

        <rect x={96} y={1} width={10} height={10} rx={2} fill={AMZ.card} stroke={AMZ.orange} strokeWidth={1.5} />
        <text x={109} y={10} fontSize={9} fill={AMZ.muted}>로봇암</text>

        <rect x={150} y={1} width={10} height={10} rx={2} fill={AMZ.card} stroke={AMZ.border} strokeWidth={1.5} />
        <text x={163} y={10} fontSize={9} fill={AMZ.muted}>섹션</text>

        <rect x={200} y={1} width={10} height={10} rx={2} fill="transparent" stroke={AMZ.muted} strokeWidth={1} strokeDasharray="3 2" />
        <text x={213} y={10} fontSize={9} fill={AMZ.muted}>배송지</text>

        <rect x={254} y={3} width={16} height={6} rx={2} fill="#CACACA" />
        <rect x={254} y={4} width={16} height={4} rx={1} fill={AMZ.mid + "40"} stroke={AMZ.mid} strokeWidth={0.8} />
        <text x={273} y={10} fontSize={9} fill={AMZ.muted}>컨베이어</text>
      </g>
    </svg>
  );
}
