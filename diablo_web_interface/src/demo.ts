import type {
  DiabloState,
  OccupancyGrid,
  PathData,
  ScanData,
  TopicDescriptor,
} from "./types";

function makeDemoGrid(): OccupancyGrid {
  const width = 84;
  const height = 54;
  const data = Array.from({ length: width * height }, (_, index) => {
    const x = index % width;
    const y = Math.floor(index / width);
    const border = x === 0 || y === 0 || x === width - 1 || y === height - 1;
    const blockA = x > 22 && x < 29 && y > 12 && y < 43;
    const blockB = x > 48 && x < 67 && y > 28 && y < 34;
    const blockC = x > 68 && x < 75 && y > 8 && y < 23;
    return border || blockA || blockB || blockC ? 100 : 0;
  });
  return {
    frame_id: "map",
    resolution: 0.05,
    width,
    height,
    origin: { x: -1, y: -1, yaw: 0 },
    data,
  };
}

function makeDemoPath(): PathData {
  const poses = Array.from({ length: 24 }, (_, index) => ({
    x: 0.5 + index * 0.12,
    y: 0.35 + Math.sin(index / 4) * 0.18,
  }));
  return { frame_id: "map", poses };
}

function makeDemoCostmap(grid: OccupancyGrid, variant: "local" | "global"): OccupancyGrid {
  const radius = variant === "local" ? 2 : 3;
  const data = grid.data.map((value, index) => {
    if (value >= 65) return 100;
    const x = index % grid.width;
    const y = Math.floor(index / grid.width);
    for (let dy = -radius; dy <= radius; dy += 1) {
      for (let dx = -radius; dx <= radius; dx += 1) {
        const distance = Math.hypot(dx, dy);
        const neighbor = grid.data[(y + dy) * grid.width + (x + dx)];
        if (distance <= radius && neighbor !== undefined && neighbor >= 65) {
          return Math.max(25, Math.round(95 - distance * 18));
        }
      }
    }
    return 0;
  });
  return { ...grid, frame_id: variant === "local" ? "odom" : "map", data };
}

function makeDemoWheelTrajectory(): Array<{ x: number; y: number }> {
  return Array.from({ length: 32 }, (_, index) => ({
    x: 0.25 + index * 0.035,
    y: 0.22 + Math.sin(index / 5) * 0.06,
  }));
}

function makeDemoScan(): ScanData {
  const ranges = Array.from({ length: 180 }, (_, index) => {
    const obstacle = index > 76 && index < 103;
    return obstacle ? 1.15 + Math.sin(index) * 0.04 : 2.4 + Math.sin(index / 9) * 0.25;
  });
  return {
    frame_id: "laser",
    angle_min: -Math.PI,
    angle_increment: (Math.PI * 2) / ranges.length,
    range_min: 0.12,
    range_max: 8,
    ranges,
  };
}

const demoMap = makeDemoGrid();

export const demoState: DiabloState = {
  type: "state",
  stamp: Date.now() / 1000,
  pose: { x: 0.62, y: 0.42, theta: 0.18, source: "demo /map" },
  wheel_pose: { x: 0.62, y: 0.42, theta: 0.18, source: "demo /odometry/filtered" },
  wheel_trajectory: makeDemoWheelTrajectory(),
  telemetry: {
    battery: { voltage: 24.6, current: 1.8, percentage: 82, temperature: 31.2 },
    body_state: { ctrl_mode: 1, robot_mode: 2, error: 0, warning: 0 },
    imu: {
      roll: -0.02,
      pitch: 0.03,
      yaw: 0.18,
      angular_velocity: { x: 0.01, y: -0.01, z: 0.04 },
    },
    motors: {
      left_wheel: { position: 4.2, velocity: 0.14, revolutions: 0 },
      right_wheel: { position: -4.0, velocity: -0.13, revolutions: 0 },
      left_leg_length: 0.31,
      right_leg_length: 0.31,
    },
  },
  control_mode: "manual",
  nav_goal: {
    state: "idle",
    message: "No active navigation goal",
    seq: 0,
    distance_remaining: null,
  },
  hardware: {
    ready: false,
    starting: false,
    message: "Preview mode — press START HARDWARE when connected to the robot",
    components: [
      { id: "diablo", label: "DIABLO ROS2", state: "offline", detail: "Waiting for robot" },
      { id: "lidar", label: "LIDAR", state: "not_configured", detail: "Configure lidar_start_command" },
      { id: "dynamixel", label: "DYNAMIXEL U2D2", state: "offline", detail: "Waiting for robot" },
    ],
    updated: Date.now() / 1000,
  },
  versions: { map: 1, local_costmap: 1, global_costmap: 1, path: 1, scan: 1 },
  map: demoMap,
  local_costmap: makeDemoCostmap(demoMap, "local"),
  global_costmap: makeDemoCostmap(demoMap, "global"),
  path: makeDemoPath(),
  scan: makeDemoScan(),
};

export const demoTopics: TopicDescriptor[] = [
  { name: "/diablo/sensor/Battery", types: ["sensor_msgs/msg/BatteryState"] },
  { name: "/diablo/sensor/Body_state", types: ["motion_msgs/msg/RobotStatus"] },
  { name: "/diablo/sensor/Imu", types: ["sensor_msgs/msg/Imu"] },
  { name: "/diablo/sensor/Motors", types: ["motion_msgs/msg/LegMotors"] },
  { name: "/odometry/filtered", types: ["nav_msgs/msg/Odometry"] },
  { name: "/scan", types: ["sensor_msgs/msg/LaserScan"] },
  { name: "/diablo/MotionCmd", types: ["motion_msgs/msg/MotionCtrl"] },
];
