#!/usr/bin/env python3
"""Convert end-effector poses into MoveIt IK + plan/execute requests.

The node deliberately defaults to plan-only mode.  It never talks to a
serial port itself; hardware motion can happen only when MoveIt's controller
manager is connected to a real FollowJointTrajectory controller and the user
explicitly sets ``execute:=true``.
"""

import copy
import math
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PositionIKRequest,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState


LEFT_JOINTS = [
    "upper_left_shoulder_pitch_joint",
    "upper_left_shoulder_roll_joint",
    "upper_left_elbow_joint",
]
RIGHT_JOINTS = [
    "upper_right_shoulder_pitch_joint",
    "upper_right_shoulder_roll_joint",
    "upper_right_elbow_joint",
]

# Geometry of the three-DOF planning chain in
# diablo_simulation.xacro.  These are used only to create a DH-style
# straight-pointing seed for a far target; MoveIt still validates the pose,
# limits and collisions before planning.
DH_ARM_GEOMETRY = {
    "left": {
        "shoulder": (-0.0107, 0.0950, 0.2622),
        "roll_origin": (0.0187, 0.0405, 0.0),
        "roll_base": math.pi / 2.0,
        "roll_sign": -1.0,
        "elbow_origin": (-0.0190, 0.0194, -0.1505),
        "tip_origin": (0.0, -0.0190, -0.0910),
    },
    "right": {
        "shoulder": (-0.0107, -0.0950, 0.2622),
        "roll_origin": (0.0187, -0.0405, 0.0),
        "roll_base": -math.pi / 2.0,
        "roll_sign": 1.0,
        "elbow_origin": (-0.0190, -0.0194, -0.1505),
        "tip_origin": (0.0, 0.0190, -0.0910),
    },
}


class IkMoveItBridge(Node):
    """Receive PoseStamped goals and ask MoveIt to solve, plan and optionally execute."""

    def __init__(self) -> None:
        super().__init__("ik_moveit_bridge")

        self.declare_parameter("target_topic", "/diablo/ik_target")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("move_action", "/move_action")
        self.declare_parameter("group_name", "left_manipulator")
        self.declare_parameter("ik_link_name", "left_arm_tip_link")
        self.declare_parameter("planning_frame", "torso_link")
        # Humble cannot infer the type of an empty YAML sequence.  Use a
        # one-element string array as the typed empty sentinel and filter it
        # below, so the parameter can safely be overridden with joint names.
        self.declare_parameter("controlled_joints", [""])
        self.declare_parameter("execute", False)
        self.declare_parameter("ik_timeout_sec", 0.25)
        self.declare_parameter("planning_time_sec", 5.0)
        self.declare_parameter("joint_goal_tolerance", 0.02)
        self.declare_parameter("max_velocity_scaling", 0.10)
        self.declare_parameter("max_acceleration_scaling", 0.10)
        self.declare_parameter("fallback_enabled", True)
        self.declare_parameter("fallback_search_radius_m", 0.15)
        self.declare_parameter("fallback_search_step_m", 0.02)
        self.declare_parameter("fallback_max_candidates", 220)
        self.declare_parameter("fallback_refinement_steps", 6)
        self.declare_parameter("fallback_ik_timeout_sec", 0.10)
        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("pointing_enabled", True)
        self.declare_parameter("pointing_trigger_distance_m", 0.45)
        self.declare_parameter("pointing_elbow_target_rad", 0.0)
        self.declare_parameter("pointing_elbow_tolerance_rad", 0.08)
        self.declare_parameter("pointing_relaxed_step_m", 0.01)
        self.declare_parameter("pointing_relaxed_candidates", 4)
        self.declare_parameter("dh_seed_enabled", True)
        self.declare_parameter("pointing_lock_elbow", False)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.ik_service_name = str(self.get_parameter("ik_service").value)
        self.move_action_name = str(self.get_parameter("move_action").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link_name = str(self.get_parameter("ik_link_name").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.execute = bool(self.get_parameter("execute").value)
        self.ik_timeout_sec = float(self.get_parameter("ik_timeout_sec").value)
        self.planning_time_sec = float(self.get_parameter("planning_time_sec").value)
        self.joint_goal_tolerance = float(
            self.get_parameter("joint_goal_tolerance").value
        )
        self.max_velocity_scaling = float(
            self.get_parameter("max_velocity_scaling").value
        )
        self.max_acceleration_scaling = float(
            self.get_parameter("max_acceleration_scaling").value
        )
        self.fallback_enabled = bool(self.get_parameter("fallback_enabled").value)
        self.fallback_search_radius_m = float(
            self.get_parameter("fallback_search_radius_m").value
        )
        self.fallback_search_step_m = float(
            self.get_parameter("fallback_search_step_m").value
        )
        self.fallback_max_candidates = int(
            self.get_parameter("fallback_max_candidates").value
        )
        self.fallback_refinement_steps = int(
            self.get_parameter("fallback_refinement_steps").value
        )
        self.fallback_ik_timeout_sec = float(
            self.get_parameter("fallback_ik_timeout_sec").value
        )
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)
        self.pointing_enabled = bool(self.get_parameter("pointing_enabled").value)
        self.pointing_trigger_distance_m = float(
            self.get_parameter("pointing_trigger_distance_m").value
        )
        self.pointing_elbow_target_rad = float(
            self.get_parameter("pointing_elbow_target_rad").value
        )
        self.pointing_elbow_tolerance_rad = float(
            self.get_parameter("pointing_elbow_tolerance_rad").value
        )
        self.pointing_relaxed_step_m = float(
            self.get_parameter("pointing_relaxed_step_m").value
        )
        self.pointing_relaxed_candidates = int(
            self.get_parameter("pointing_relaxed_candidates").value
        )
        self.dh_seed_enabled = bool(self.get_parameter("dh_seed_enabled").value)
        self.pointing_lock_elbow = bool(
            self.get_parameter("pointing_lock_elbow").value
        )

        configured_joints = self.get_parameter("controlled_joints").value
        self.controlled_joints = [
            str(name) for name in (configured_joints or []) if str(name)
        ]
        if not self.controlled_joints:
            if self.group_name in ("left_manipulator", "left_arm"):
                self.controlled_joints = list(LEFT_JOINTS)
            elif self.group_name in ("right_manipulator", "right_arm"):
                self.controlled_joints = list(RIGHT_JOINTS)

        self._current_joint_state: Optional[JointState] = None
        self._busy = False
        self._requested_target: Optional[PoseStamped] = None
        self._fallback_targets = []
        self._fallback_index = 0
        self._refine_active = False
        self._refine_low = 0.0
        self._refine_high = 0.0
        self._refine_mid = 0.0
        self._refine_direction = None
        self._refine_best_target = None
        self._refine_best_response = None
        self._refine_iteration = 0
        self._refine_attempt_index = 0
        self._refine_seed = None
        self._refine_constraints = None
        self._refine_mode = "fallback"

        # Candidate metadata is kept parallel to _fallback_targets.  This
        # lets a far-target candidate carry a DH seed and an elbow constraint
        # while ordinary fallback candidates remain unconstrained.
        self._fallback_seeds = []
        self._fallback_constraints = []
        self._fallback_modes = []

        self._ik_client = self.create_client(GetPositionIK, self.ik_service_name)
        self._move_client = ActionClient(self, MoveGroup, self.move_action_name)

        self._ik_solution_pub = self.create_publisher(
            JointState, "~/ik_solution", 10
        )
        self._accepted_target_pub = self.create_publisher(
            PoseStamped, "~/accepted_target", 10
        )
        self.create_subscription(
            JointState, self.joint_states_topic, self._joint_state_callback, 10
        )
        self.create_subscription(
            PoseStamped, self.target_topic, self._target_callback, 10
        )

        mode = "EXECUTE" if self.execute else "PLAN ONLY"
        self.get_logger().info(
            f"{mode}: group={self.group_name}, tip={self.ik_link_name}, "
            f"target_topic={self.target_topic}"
        )
        if self.fallback_enabled:
            self.get_logger().info(
                "IK fallback enabled: "
                f"radius={self.fallback_search_radius_m:.3f} m, "
                f"step={self.fallback_search_step_m:.3f} m, "
                f"max_candidates={self.fallback_max_candidates}, "
                f"refinement_steps={self.fallback_refinement_steps}, "
                f"avoid_collisions={self.avoid_collisions}"
            )
        if self.pointing_enabled:
            self.get_logger().info(
                "Far-target pointing enabled: "
                f"trigger={self.pointing_trigger_distance_m:.3f} m, "
                f"elbow_target={self.pointing_elbow_target_rad:.3f} rad, "
                f"dh_seed={self.dh_seed_enabled}, "
                f"lock_elbow={self.pointing_lock_elbow}"
            )
        if self.execute:
            self.get_logger().warn(
                "execute=true: MoveIt may move hardware if its trajectory "
                "controller is active. Verify the robot, limits and emergency "
                "stop before publishing a target."
            )
        if not self.controlled_joints:
            self.get_logger().warn(
                "controlled_joints is empty. The bridge will use every joint "
                "returned by IK; set it explicitly for a real robot."
            )

    def _joint_state_callback(self, msg: JointState) -> None:
        self._current_joint_state = copy.deepcopy(msg)

    @staticmethod
    def _duration(seconds: float) -> Duration:
        seconds = max(0.0, float(seconds))
        whole = int(seconds)
        return Duration(
            sec=whole,
            nanosec=int((seconds - whole) * 1_000_000_000),
        )

    def _normalise_pose(self, incoming: PoseStamped) -> PoseStamped:
        pose = copy.deepcopy(incoming)
        if not pose.header.frame_id:
            pose.header.frame_id = self.planning_frame

        q = pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-9:
            q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
        else:
            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm
        return pose

    def _current_robot_state(self) -> RobotState:
        state = RobotState()
        state.is_diff = True
        if self._current_joint_state is not None:
            state.joint_state = copy.deepcopy(self._current_joint_state)
        return state

    @staticmethod
    def _position_distance(first: PoseStamped, second: PoseStamped) -> float:
        dx = first.pose.position.x - second.pose.position.x
        dy = first.pose.position.y - second.pose.position.y
        dz = first.pose.position.z - second.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _rotate_x(vector, angle):
        """Rotate a 3-vector around local X without requiring NumPy."""
        x, y, z = vector
        c = math.cos(angle)
        s = math.sin(angle)
        return (x, c * y - s * z, s * y + c * z)

    @staticmethod
    def _rotate_y(vector, angle):
        """Rotate a 3-vector around local Y without requiring NumPy."""
        x, y, z = vector
        c = math.cos(angle)
        s = math.sin(angle)
        return (c * x + s * z, y, -s * x + c * z)

    @staticmethod
    def _normalise_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _arm_side(self):
        if self.group_name in ("left_manipulator", "left_arm"):
            return "left"
        if self.group_name in ("right_manipulator", "right_arm"):
            return "right"
        return None

    def _pointing_constraints(self, tolerance):
        """Constrain the elbow near the straight-arm configuration."""
        side = self._arm_side()
        if side is None:
            return None

        constraint = JointConstraint()
        constraint.joint_name = (
            LEFT_JOINTS[2] if side == "left" else RIGHT_JOINTS[2]
        )
        constraint.position = self.pointing_elbow_target_rad
        constraint.tolerance_above = max(0.0, float(tolerance))
        constraint.tolerance_below = max(0.0, float(tolerance))
        constraint.weight = 1.0

        constraints = Constraints()
        constraints.joint_constraints.append(constraint)
        return constraints

    def _dh_straight_solution(self, target):
        """Calculate a DH-style straight-arm solution for a far target.

        The URDF chain can be reduced to:

            p_tip = p_shoulder + R_y(q_pitch) * w(q_roll, q_elbow)

        where ``w`` is built from the fixed link offsets and the elbow
        rotation.  For pointing, q_elbow is held near zero and q_roll is
        sampled to find the vector whose direction best matches the target.
        The resulting joints are a seed for MoveIt's KDL solver; they are not
        sent to hardware directly.
        """
        if not self.pointing_enabled or target.header.frame_id != self.planning_frame:
            return None

        side = self._arm_side()
        if side is None:
            return None
        geometry = DH_ARM_GEOMETRY[side]

        shoulder = geometry["shoulder"]
        raw = (
            target.pose.position.x - shoulder[0],
            target.pose.position.y - shoulder[1],
            target.pose.position.z - shoulder[2],
        )
        distance = math.sqrt(sum(value * value for value in raw))
        if distance < self.pointing_trigger_distance_m:
            return None

        direction = tuple(value / distance for value in raw)
        direction_xz = math.hypot(direction[0], direction[2])
        if direction_xz < 1e-9:
            direction_angle = 0.0
        else:
            direction_angle = math.atan2(direction[2], direction[0])

        elbow = self.pointing_elbow_target_rad
        elbow_vector = self._rotate_y(geometry["tip_origin"], -elbow)
        combined_vector = tuple(
            geometry["elbow_origin"][index] + elbow_vector[index]
            for index in range(3)
        )

        best = None
        # 0..2.2 rad matches upper_only_joint_limits.yaml.  The dense scan is
        # cheap and makes the geometric seed deterministic across runs.
        roll_samples = 2201
        for index in range(roll_samples):
            q_roll = 2.20 * index / float(roll_samples - 1)
            beta = geometry["roll_base"] + geometry["roll_sign"] * q_roll
            rotated = self._rotate_x(combined_vector, beta)
            local_vector = tuple(
                geometry["roll_origin"][component] + rotated[component]
                for component in range(3)
            )
            local_radius = math.hypot(local_vector[0], local_vector[2])
            local_length = math.sqrt(sum(value * value for value in local_vector))
            if local_length < 1e-9:
                continue

            if direction_xz < 1e-9:
                q_pitch = 0.0
            else:
                q_pitch = self._normalise_angle(
                    math.atan2(local_vector[2], local_vector[0])
                    - direction_angle
                )
            rotated_vector = self._rotate_y(local_vector, q_pitch)
            rotated_length = math.sqrt(
                sum(value * value for value in rotated_vector)
            )
            if rotated_length < 1e-9:
                continue
            unit_vector = tuple(value / rotated_length for value in rotated_vector)
            direction_error = math.sqrt(
                sum(
                    (unit_vector[component] - direction[component]) ** 2
                    for component in range(3)
                )
            )

            # Prefer the direction that is closest to the requested ray. If
            # two samples are equivalent, prefer the larger reach.
            score = (direction_error, -local_length)
            if best is None or score < best[0]:
                best = (score, q_pitch, q_roll, rotated_vector, direction_error)

        if best is None:
            return None

        _, q_pitch, q_roll, tip_vector, direction_error = best
        tip_position = tuple(
            shoulder[component] + tip_vector[component] for component in range(3)
        )
        seed = {
            LEFT_JOINTS[0] if side == "left" else RIGHT_JOINTS[0]: q_pitch,
            LEFT_JOINTS[1] if side == "left" else RIGHT_JOINTS[1]: q_roll,
            LEFT_JOINTS[2] if side == "left" else RIGHT_JOINTS[2]: elbow,
        }
        return {
            "position": tip_position,
            "direction": direction,
            "reach": math.sqrt(sum(value * value for value in tip_vector)),
            "direction_error": direction_error,
            "seed": seed if self.dh_seed_enabled else None,
        }

    def _build_candidate_set(self, target):
        """Build requested/fallback candidates or a far-target pointing set."""
        dh_solution = self._dh_straight_solution(target)
        if dh_solution is not None:
            direction = dh_solution["direction"]
            base = copy.deepcopy(target)
            base.pose.position.x = dh_solution["position"][0]
            base.pose.position.y = dh_solution["position"][1]
            base.pose.position.z = dh_solution["position"][2]

            targets = [base]
            seeds = [dh_solution["seed"]]
            constraints = [
                self._pointing_constraints(self.pointing_elbow_tolerance_rad)
                if self.pointing_lock_elbow
                else None
            ]
            modes = ["dh_pointing"]

            # If the exact straight pose is in collision, try slightly
            # shorter points on the same ray with a small elbow tolerance.
            # This preserves the pointing behavior while allowing MoveIt to
            # bend just enough to clear the torso or another object.
            relaxed_tolerance = max(self.pointing_elbow_tolerance_rad, 0.35)
            for index in range(1, max(0, self.pointing_relaxed_candidates) + 1):
                shorter = copy.deepcopy(base)
                offset = index * max(0.0, self.pointing_relaxed_step_m)
                shorter.pose.position.x -= offset * direction[0]
                shorter.pose.position.y -= offset * direction[1]
                shorter.pose.position.z -= offset * direction[2]
                targets.append(shorter)
                seeds.append(dh_solution["seed"])
                constraints.append(
                    self._pointing_constraints(relaxed_tolerance)
                    if self.pointing_lock_elbow
                    else None
                )
                modes.append("pointing_relaxed")

            side = self._arm_side()
            seed = dh_solution["seed"] or {}
            seed_joints = LEFT_JOINTS if side == "left" else RIGHT_JOINTS
            self.get_logger().info(
                "DH pointing candidate: "
                f"reach={dh_solution['reach']:.3f} m, "
                f"direction_error={dh_solution['direction_error']:.4f}, "
                f"q=({seed.get(seed_joints[0], 0.0):.3f}, "
                f"{seed.get(seed_joints[1], 0.0):.3f}, "
                f"{seed.get(seed_joints[2], 0.0):.3f})"
            )
            return targets, seeds, constraints, modes

        targets = self._build_fallback_targets(target)
        return (
            targets,
            [None] * len(targets),
            [None] * len(targets),
            ["requested"] + ["fallback"] * (len(targets) - 1),
        )

    def _state_with_seed(self, seed):
        state = self._current_robot_state()
        if not seed:
            return state

        names = list(state.joint_state.name)
        positions = list(state.joint_state.position)
        values = dict(zip(names, positions))
        for name, position in seed.items():
            if name not in values:
                names.append(name)
            values[name] = float(position)
        state.joint_state.name = names
        state.joint_state.position = [values[name] for name in names]
        return state

    def _build_fallback_targets(self, target: PoseStamped):
        """Return the requested pose followed by radial fallback poses.

        A dense Cartesian grid wastes most IK calls at nearly identical
        distances. This search uses 26 directions (axes, face diagonals and
        body diagonals) on increasing radial shells. It therefore reaches the
        configured radius with a bounded number of calls. Once a candidate
        succeeds, the caller refines the boundary along that direction.
        """
        targets = [target]
        if not self.fallback_enabled:
            return targets

        radius = max(0.0, self.fallback_search_radius_m)
        step = self.fallback_search_step_m
        max_candidates = max(0, self.fallback_max_candidates)
        if radius <= 0.0 or step <= 0.0 or max_candidates == 0:
            return targets

        directions = []
        for ix in (-1, 0, 1):
            for iy in (-1, 0, 1):
                for iz in (-1, 0, 1):
                    if ix == 0 and iy == 0 and iz == 0:
                        continue
                    norm = math.sqrt(ix * ix + iy * iy + iz * iz)
                    # Try cardinal directions before diagonals at the same
                    # radius. This is useful when the target is behind the
                    # torso or on the wrong side of the arm.
                    nonzero = abs(ix) + abs(iy) + abs(iz)
                    directions.append((nonzero, (ix / norm, iy / norm, iz / norm)))
        directions.sort(key=lambda item: item[0])

        shell_count = max(1, int(math.ceil(radius / step)))
        candidates = []
        for shell in range(1, shell_count + 1):
            distance = min(shell * step, radius)
            for _, direction in directions:
                dx = distance * direction[0]
                dy = distance * direction[1]
                dz = distance * direction[2]
                candidate = copy.deepcopy(target)
                candidate.pose.position.x += dx
                candidate.pose.position.y += dy
                candidate.pose.position.z += dz
                candidates.append((distance, candidate))

        # The default budget covers all 26 directions for the default 15 cm
        # radius and 2 cm shell spacing (8 * 26 = 208 candidates).
        targets.extend(candidate for _, candidate in candidates[:max_candidates])
        return targets

    def _send_ik_request(
        self, target: PoseStamped, attempt_index: int, refinement: bool = False
    ) -> None:
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ik_link_name
        request.ik_request.pose_stamped = target
        if refinement:
            seed = self._refine_seed
            constraints = self._refine_constraints
        else:
            seed = self._fallback_seeds[attempt_index]
            constraints = self._fallback_constraints[attempt_index]
        request.ik_request.robot_state = self._state_with_seed(seed)
        if constraints is not None:
            request.ik_request.constraints = copy.deepcopy(constraints)
        request.ik_request.avoid_collisions = self.avoid_collisions
        timeout = self.ik_timeout_sec
        if attempt_index > 0 and self.fallback_ik_timeout_sec > 0.0:
            timeout = self.fallback_ik_timeout_sec
        request.ik_request.timeout = self._duration(timeout)

        future = self._ik_client.call_async(request)
        future.add_done_callback(
            lambda result: self._ik_done(
                result, target, attempt_index, refinement=refinement
            )
        )

    def _start_refinement(self, target: PoseStamped, response, attempt_index: int):
        """Bisect between the failed original target and a valid candidate."""
        if self._requested_target is None or self.fallback_refinement_steps <= 0:
            self._handle_ik_success(response, target, attempt_index)
            return

        dx = target.pose.position.x - self._requested_target.pose.position.x
        dy = target.pose.position.y - self._requested_target.pose.position.y
        dz = target.pose.position.z - self._requested_target.pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance < 1e-9:
            self._handle_ik_success(response, target, attempt_index)
            return

        self._refine_active = True
        self._refine_low = 0.0
        self._refine_high = distance
        self._refine_direction = (dx / distance, dy / distance, dz / distance)
        self._refine_best_target = copy.deepcopy(target)
        self._refine_best_response = response
        self._refine_iteration = 0
        self._refine_attempt_index = attempt_index
        self._refine_seed = self._fallback_seeds[attempt_index]
        self._refine_constraints = self._fallback_constraints[attempt_index]
        self._refine_mode = self._fallback_modes[attempt_index]
        self._send_next_refinement()

    def _send_next_refinement(self) -> None:
        if not self._refine_active:
            return
        if self._refine_iteration >= self.fallback_refinement_steps:
            target = self._refine_best_target
            response = self._refine_best_response
            attempt_index = self._refine_attempt_index
            self._refine_active = False
            self._handle_ik_success(
                response, target, attempt_index, self._refine_mode
            )
            return

        midpoint = 0.5 * (self._refine_low + self._refine_high)
        self._refine_mid = midpoint
        direction = self._refine_direction
        target = copy.deepcopy(self._requested_target)
        target.pose.position.x += midpoint * direction[0]
        target.pose.position.y += midpoint * direction[1]
        target.pose.position.z += midpoint * direction[2]
        self._send_ik_request(
            target,
            self._refine_attempt_index,
            refinement=True,
        )

    def _target_callback(self, incoming: PoseStamped) -> None:
        if self._busy:
            self.get_logger().warn("Ignoring target: a previous request is still active")
            return
        if not self._ik_client.service_is_ready():
            self.get_logger().error(
                f"IK service {self.ik_service_name} is not ready; start MoveIt first"
            )
            return

        target = self._normalise_pose(incoming)
        self.get_logger().info(
            f"Received target: frame={target.header.frame_id}, "
            f"position=({target.pose.position.x:.3f}, "
            f"{target.pose.position.y:.3f}, {target.pose.position.z:.3f})"
        )
        self._requested_target = target
        (
            self._fallback_targets,
            self._fallback_seeds,
            self._fallback_constraints,
            self._fallback_modes,
        ) = self._build_candidate_set(target)
        self._fallback_index = 0
        self._busy = True
        self._send_ik_request(self._fallback_targets[0], self._fallback_index)

    def _finish_request(self) -> None:
        self._busy = False
        self._refine_active = False
        self._refine_seed = None
        self._refine_constraints = None

    def _ik_done(
        self,
        future,
        target: PoseStamped,
        attempt_index: int,
        refinement: bool = False,
    ) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - middleware exception path
            if refinement and self._refine_best_response is not None:
                self.get_logger().warn(
                    f"IK refinement request failed ({exc}); using the last "
                    "valid fallback candidate"
                )
                best_target = self._refine_best_target
                best_response = self._refine_best_response
                best_attempt = self._refine_attempt_index
                self._refine_active = False
                self._handle_ik_success(
                    best_response,
                    best_target,
                    best_attempt,
                    self._refine_mode,
                )
                return
            self.get_logger().error(f"IK service call failed: {exc}")
            self._finish_request()
            return

        error_code = response.error_code.val
        if refinement:
            if error_code == MoveItErrorCodes.SUCCESS:
                self._refine_high = self._refine_mid
                self._refine_best_target = copy.deepcopy(target)
                self._refine_best_response = response
            else:
                self._refine_low = self._refine_mid
            self._refine_iteration += 1
            self._send_next_refinement()
            return

        if error_code != MoveItErrorCodes.SUCCESS:
            next_index = attempt_index + 1
            if next_index < len(self._fallback_targets):
                self._fallback_index = next_index
                if attempt_index == 0:
                    self.get_logger().warn(
                        "Original target has no valid IK solution; searching "
                        "for the nearest reachable target"
                    )
                self._send_ik_request(
                    self._fallback_targets[next_index], next_index
                )
                return

            self.get_logger().error(
                f"MoveIt IK failed with error code {error_code} after testing "
                f"{len(self._fallback_targets)} target position(s)"
            )
            self._finish_request()
            return

        mode = self._fallback_modes[attempt_index]
        if mode == "fallback" and self.fallback_refinement_steps > 0:
            self._start_refinement(target, response, attempt_index)
            return

        self._handle_ik_success(response, target, attempt_index, mode)

    def _handle_ik_success(
        self,
        response,
        target: PoseStamped,
        attempt_index: int,
        mode: str = "requested",
    ) -> None:
        if mode.startswith("pointing") or mode == "dh_pointing":
            distance = (
                self._position_distance(target, self._requested_target)
                if self._requested_target is not None
                else 0.0
            )
            self.get_logger().warn(
                "Using DH pointing target: "
                f"({target.pose.position.x:.3f}, "
                f"{target.pose.position.y:.3f}, "
                f"{target.pose.position.z:.3f}) m; "
                f"distance from requested target={distance:.3f} m"
            )
        elif self._requested_target is not None and attempt_index > 0:
            distance = self._position_distance(target, self._requested_target)
            self.get_logger().warn(
                "Using nearest tested reachable target: "
                f"({target.pose.position.x:.3f}, "
                f"{target.pose.position.y:.3f}, "
                f"{target.pose.position.z:.3f}) m; "
                f"distance from requested target={distance:.3f} m"
            )
        self._accepted_target_pub.publish(target)

        solution = response.solution
        self._ik_solution_pub.publish(solution.joint_state)
        solution_map: Dict[str, float] = dict(
            zip(solution.joint_state.name, solution.joint_state.position)
        )
        joint_names = list(self.controlled_joints)
        if not joint_names:
            joint_names = list(solution_map.keys())
        missing = [name for name in joint_names if name not in solution_map]
        if missing:
            self.get_logger().error(
                "IK returned no values for joint(s): " + ", ".join(missing)
            )
            self._finish_request()
            return

        self.get_logger().info(
            "IK succeeded for joints: " + ", ".join(joint_names)
        )

        if not self._move_client.server_is_ready():
            self.get_logger().error(
                f"MoveGroup action {self.move_action_name} is not ready; "
                "start move_group first"
            )
            self._finish_request()
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.start_state = self._current_robot_state()
        goal.request.allowed_planning_time = self.planning_time_sec
        goal.request.num_planning_attempts = 5
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling

        constraints = Constraints()
        for name in joint_names:
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(solution_map[name])
            constraint.tolerance_above = self.joint_goal_tolerance
            constraint.tolerance_below = self.joint_goal_tolerance
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = not self.execute
        goal.planning_options.replan = False

        self.get_logger().info(
            "Sending %s request to MoveIt"
            % ("execution" if self.execute else "plan-only")
        )

        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - middleware exception path
            self.get_logger().error(f"MoveGroup goal failed to send: {exc}")
            self._finish_request()
            return

        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup rejected the request")
            self._finish_request()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._move_result)

    def _move_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # pragma: no cover - middleware exception path
            self.get_logger().error(f"MoveGroup result failed: {exc}")
            self._finish_request()
            return

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            mode = "executed" if self.execute else "planned"
            self.get_logger().info(
                f"MoveIt request {mode} successfully in {result.planning_time:.3f}s"
            )
        else:
            self.get_logger().error(
                f"MoveIt planning/execution failed with error code "
                f"{result.error_code.val}"
            )
        self._finish_request()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IkMoveItBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
