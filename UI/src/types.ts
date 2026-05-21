export interface Position2D {
  x: number;
  y: number;
  yaw?: number;
}

export interface Position3D {
  x: number;
  y: number;
  z: number;
}

export interface DetectedItem {
  marker_id: number;
  label: string;
  category: string;
  detected_at: unknown;
}

export interface AMRData {
  robot_id: string;
  type: "amr";
  battery: number;
  charge_status: "charging" | "operating";
  cargo_status: "empty" | "loading" | "transporting" | "unloading";
  position: Position2D;
  speed: number;
  current_task: string | null;
  last_updated: { seconds: number };
}

export interface DroneData {
  robot_id: string;
  type: "drone";
  battery: number;
  charge_status: "charging" | "operating";
  cargo_status: "empty" | "loading" | "transporting" | "unloading";
  position: Position3D;
  altitude: number;
  heading: number;
  speed: number;
  current_task: string | null;
  last_updated: { seconds: number };
}

export interface ArmData {
  robot_id: string;
  type: "arm";
  status: "idle" | "picking" | "placing" | "moving" | "error";
  battery: number;
  gripper: "open" | "closed";
  position: Position3D;
  joints: number[];
  current_task: string | null;
  detected_item: DetectedItem | null;
  last_updated: { seconds: number };
}

export interface RobotFleetData {
  amr: AMRData | null;
  drone: DroneData | null;
  arm: ArmData | null;
}

export interface ProductData {
  product_id: string;
  name: string;
  marker_id: number;
  section: string;
  destination: string;
}

export interface ItemData {
  item_id: string;
  product_id: string;
  name: string;
  marker_id: number;
  section: string;
  destination: string;
  status: "registered" | "waiting" | "detected" | "in_transit" | "delivered" | "returned";
  assigned_robot: string | null;
  detected_at: { seconds: number } | null;
  delivered_at: { seconds: number } | null;
}
