export type AppView = "drive" | "navigation" | "topics" | "settings";
export type ControlMode = "manual" | "auto" | "stop";

export interface Pose {
  x: number;
  y: number;
  theta: number;
  source?: string;
}

export interface MapOrigin {
  x: number;
  y: number;
  yaw: number;
}

export interface OccupancyGrid {
  frame_id: string;
  resolution: number;
  width: number;
  height: number;
  origin: MapOrigin;
  data: number[];
}

export interface ScanData {
  frame_id: string;
  angle_min: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  ranges: Array<number | null>;
}

export interface PathData {
  frame_id: string;
  poses: Array<{ x: number; y: number }>;
}

export interface BatteryTelemetry {
  voltage: number | null;
  current: number | null;
  percentage: number | null;
  temperature: number | null;
}

export interface BodyTelemetry {
  ctrl_mode: number;
  robot_mode: number;
  error: number;
  warning: number;
}

export interface ImuTelemetry {
  roll: number;
  pitch: number;
  yaw: number;
  angular_velocity: { x: number; y: number; z: number };
}

export interface MotorTelemetry {
  left_wheel: { position: number; velocity: number; revolutions: number };
  right_wheel: { position: number; velocity: number; revolutions: number };
  left_leg_length: number;
  right_leg_length: number;
}

export interface Telemetry {
  battery: BatteryTelemetry | null;
  body_state: BodyTelemetry | null;
  imu: ImuTelemetry | null;
  motors: MotorTelemetry | null;
}

export interface NavGoalStatus {
  state: string;
  message: string;
  seq: number;
  distance_remaining: number | null;
}

export type HardwareComponentState =
  | "ready"
  | "starting"
  | "waiting"
  | "offline"
  | "not_configured"
  | "error";

export interface HardwareComponent {
  id: "diablo" | "lidar" | "dynamixel";
  label: string;
  state: HardwareComponentState;
  detail: string;
}

export interface HardwareStatus {
  ready: boolean;
  starting: boolean;
  message: string;
  components: HardwareComponent[];
  updated: number;
}

export interface DiabloState {
  type: "state";
  stamp: number;
  pose: Pose | null;
  wheel_pose: Pose | null;
  wheel_trajectory: Array<{ x: number; y: number }>;
  telemetry: Telemetry;
  control_mode: ControlMode;
  nav_goal: NavGoalStatus;
  hardware: HardwareStatus;
  versions: Record<string, number>;
  map: OccupancyGrid | null;
  local_costmap: OccupancyGrid | null;
  global_costmap: OccupancyGrid | null;
  path: PathData | null;
  scan: ScanData | null;
}

export interface TopicDescriptor {
  name: string;
  types: string[];
}

export interface TopicPacket {
  type: "topic";
  slot: number;
  topic: string;
  msg_type: string;
  count: number;
  stamp: string;
  data: unknown;
}

export interface WebConfig {
  robot: string;
  manual_cmd_topic: string;
  control_mode_topic: string;
  map_topic: string;
  odom_topic: string;
  scan_topic: string;
  base_frame: string;
  map_frame: string;
  reset_encoder_service?: string;
  lidar_start_service?: string;
  diablo_start_command?: string;
  lidar_start_command?: string;
  dynamixel_start_command?: string;
  localization_start_command?: string;
  navigation_start_command?: string;
  mapping_start_command?: string;
  limits: { forward: number; turn: number; roll: number };
}

export interface MotionCommand {
  type: "manual";
  forward: number;
  left: number;
  roll: number;
  up: number;
  pitch: number;
  mode_mark?: boolean;
  height_ctrl_mode?: boolean;
  pitch_ctrl_mode?: boolean;
  roll_ctrl_mode?: boolean;
  stand_mode?: boolean;
  jump_mode?: boolean;
  split_mode?: boolean;
}

export interface GoalCommand {
  type: "goal_pose" | "initial_pose";
  x: number;
  y: number;
  theta: number;
}

export type SocketCommand =
  | MotionCommand
  | GoalCommand
  | { type: "stop" }
  | { type: "stand"; stand: boolean }
  | { type: "reset_odom" }
  | { type: "reset_encoder" }
  | { type: "start_lidar" }
  | { type: "start_hardware" }
  | { type: "start_localization" }
  | { type: "start_navigation" }
  | { type: "start_mapping" }
  | { type: "cancel_goal" }
  | { type: "mode"; mode: ControlMode }
  | { type: "ping" };

export interface TopicEchoPacket {
  type: "subscribe" | "clear";
  topics?: string[];
}

export interface EventEntry {
  id: number;
  time: string;
  message: string;
  kind: "info" | "success" | "warn" | "error";
}

export type PanelKey =
  | "motion"
  | "telemetry"
  | "trajectory"
  | "map"
  | "poses"
  | "controls"
  | "costmaps"
  | "log";
