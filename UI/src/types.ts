export interface RobotState {
  robot_name: string;
  state: "working" | "stop";
}

export interface IwHubState extends RobotState {
  location: { x: number; y: number };
}

export interface PodData {
  pod_id: string;
  state: "full" | "empty" | "filling" | "moving";
  location: { x: number; y: number };
}

export interface SectionData {
  section_id: "A" | "B" | "C";
  package_size: "Big" | "Medium" | "Small";
  pod_amount: number;
  robots: {
    m0609: RobotState;
    iw_hub: IwHubState;
  };
  last_updated: { seconds: number } | null;
  pods: PodData[];
}
