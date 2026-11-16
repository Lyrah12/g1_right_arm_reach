#!/usr/bin/env python3
"""MuJoCo bridge for the decoupled G1-23DOF + Revo2 controller.

Runtime control is deliberately mixed by joint group:
  * five G1 right-arm joints: torque or joint-position control (startup mode);
  * six Revo2 active joints: position targets from /hand_commands.position;
  * all other G1 joints: fixed-target position servos generated in the MJCF.

This Python file is simulation-only. Production controllers remain C++.
"""

from __future__ import annotations

import inspect
import os
import threading
import time

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from g1_right_arm_reach.msg import Contact, ContactArray


DEFAULT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
]

DEFAULT_HAND_JOINTS = [
    "right_thumb_proximal_joint",
    "right_thumb_metacarpal_joint",
    "right_index_proximal_joint",
    "right_middle_proximal_joint",
    "right_ring_proximal_joint",
    "right_pinky_proximal_joint",
]


class MuJoCoBridgeNode(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__("mujoco_bridge_node", parameter_overrides=parameter_overrides)
        self.declare_parameter("scene_path", "")
        self.declare_parameter("physics_rate", 500.0)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("viewer", False)
        self.declare_parameter("viewer_rate", 60.0)
        self.declare_parameter("arm_command_timeout", 0.10)
        self.declare_parameter("arm_control_mode", "torque")
        self.declare_parameter("arm_position_kp", [80.0, 80.0, 60.0, 60.0, 12.0])
        self.declare_parameter("arm_position_kd", [8.0, 8.0, 5.0, 5.0, 1.5])
        self.declare_parameter("arm_position_max_torque", [20.0, 20.0, 15.0, 15.0, 3.0])
        self.declare_parameter("arm_position_max_velocity", [1.0, 1.0, 1.0, 1.0, 1.5])
        self.declare_parameter("diagnostic_period", 1.0)
        self.declare_parameter("instability_velocity", 30.0)
        self.declare_parameter(
            "initial_arm_positions", [0.0, -0.35, 0.0, 0.0, 0.35])
        self.declare_parameter("arm_joint_names", DEFAULT_ARM_JOINTS)
        self.declare_parameter("hand_joint_names", DEFAULT_HAND_JOINTS)
        self.declare_parameter("ee_site", "right_revo2_tcp")

        scene_path = self.get_parameter("scene_path").value
        if not scene_path:
            scene_path = os.path.join(
                get_package_share_directory("g1_right_arm_reach"),
                "config", "models", "g1_23dof_revo2_arm_only.xml")
        if not os.path.isfile(scene_path):
            raise FileNotFoundError(scene_path)

        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.gravity_data = mujoco.MjData(self.model)
        self.arm_names = list(self.get_parameter("arm_joint_names").value)
        self.hand_names = list(self.get_parameter("hand_joint_names").value)
        self.arm_mode = str(self.get_parameter("arm_control_mode").value)
        if self.arm_mode not in ("torque", "position"):
            raise RuntimeError("arm_control_mode must be torque or position")
        self.position_kp = self._arm_parameter("arm_position_kp")
        self.position_kd = self._arm_parameter("arm_position_kd", allow_zero=True)
        self.position_max_torque = self._arm_parameter("arm_position_max_torque")
        self.position_max_velocity = self._arm_parameter("arm_position_max_velocity")
        self.arm_timeout = float(self.get_parameter("arm_command_timeout").value)
        self.diagnostic_period = float(self.get_parameter("diagnostic_period").value)
        self.instability_velocity = float(
            self.get_parameter("instability_velocity").value)
        self.lock = threading.Lock()
        self.arm_torque: dict[str, float] = {}
        self.hand_position: dict[str, float] = {}
        self.last_arm_command = -1.0

        self.joint_ids: dict[str, int] = {}
        self.qpos_addresses: dict[str, int] = {}
        self.dof_addresses: dict[str, int] = {}
        for joint_id in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if not name or self.model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
                continue
            self.joint_ids[name] = joint_id
            self.qpos_addresses[name] = int(self.model.jnt_qposadr[joint_id])
            self.dof_addresses[name] = int(self.model.jnt_dofadr[joint_id])

        self.motor_actuators: dict[str, list[int]] = {}
        self.position_actuators: dict[str, list[int]] = {}
        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            if joint_id < 0:
                continue
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if not name:
                continue
            if self.model.actuator_biastype[actuator_id] == mujoco.mjtBias.mjBIAS_NONE:
                self.motor_actuators.setdefault(name, []).append(actuator_id)
            else:
                self.position_actuators.setdefault(name, []).append(actuator_id)

        self._validate_model()
        self._set_initial_state()
        self._initialize_controls()
        mujoco.mj_forward(self.model, self.data)
        self.initial_qpos = self.data.qpos.copy()
        self.gravity_torque = self._compute_gravity()
        self.arm_position_target = np.array([
            self.data.qpos[self.qpos_addresses[name]] for name in self.arm_names])
        self.arm_position_reference = self.arm_position_target.copy()

        site_name = str(self.get_parameter("ee_site").value)
        self.ee_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.ee_site_id < 0:
            raise RuntimeError(f"MJCF site not found: {site_name}")

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, "/actual_ee_pose", 10)
        self.gravity_pub = self.create_publisher(Float64MultiArray, "/mujoco_gravity", 10)
        self.contacts_pub = self.create_publisher(ContactArray, "/mujoco/contacts", 10)
        if self.arm_mode == "position":
            self.arm_command_sub = self.create_subscription(
                JointState, "/arm_position_commands", self._arm_position_callback, 10)
        else:
            self.arm_command_sub = self.create_subscription(
                JointState, "/joint_commands", self._arm_callback, 10)
        self.hand_command_sub = self.create_subscription(
            JointState, "/hand_commands", self._hand_callback, 10)

        physics_rate = float(self.get_parameter("physics_rate").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        viewer_rate = float(self.get_parameter("viewer_rate").value)
        if not all(np.isfinite(rate) and rate > 0.0
                   for rate in (physics_rate, publish_rate, viewer_rate)):
            raise RuntimeError("physics_rate, publish_rate and viewer_rate must be finite and positive")
        if publish_rate > physics_rate:
            raise RuntimeError("publish_rate must not exceed physics_rate")
        self.model.opt.timestep = 1.0 / physics_rate
        self.publish_period = 1.0 / publish_rate
        self.publish_elapsed = 0.0
        self.step_count = 0
        self.timer = self.create_timer(1.0 / physics_rate, self._physics_step)

        self.viewer = None
        self.viewer_timer = None
        self.viewer_sync_seconds = 0.0
        self.viewer_sync_count = 0
        self.viewer_state_only = False
        if bool(self.get_parameter("viewer").value):
            from mujoco import viewer as mujoco_viewer
            self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
            self.viewer_state_only = "state_only" in inspect.signature(self.viewer.sync).parameters
            self.viewer_timer = self.create_timer(1.0 / viewer_rate, self._sync_viewer)

        self.last_diagnostic = time.monotonic()
        self.last_diagnostic_sim_time = float(self.data.time)
        self.last_diagnostic_step = self.step_count

        self.get_logger().info(
            f"Loaded {scene_path}: nq={self.model.nq}, nv={self.model.nv}, "
            f"nu={self.model.nu}; dt={self.model.opt.timestep:.4f}s, "
            f"integrator={int(self.model.opt.integrator)}, "
            f"arm={self.arm_mode}(5), hand=position(6), "
            f"viewer_rate={viewer_rate:.1f}Hz state_only={self.viewer_state_only}")

    def _arm_parameter(self, name: str, allow_zero: bool = False) -> np.ndarray:
        values = np.asarray(self.get_parameter(name).value, dtype=np.float64)
        valid_sign = values >= 0.0 if allow_zero else values > 0.0
        if (values.shape != (len(self.arm_names),)
                or not np.all(np.isfinite(values)) or not np.all(valid_sign)):
            raise RuntimeError(f"{name} requires one finite positive value per arm joint"
                               + (" (zero allowed)" if allow_zero else ""))
        return values

    def _validate_model(self) -> None:
        if (len(self.arm_names) != 5 or len(set(self.arm_names)) != 5
                or len(self.hand_names) != 6 or len(set(self.hand_names)) != 6):
            raise RuntimeError("expected five unique arm joints and six unique hand joints")
        missing_arm = [name for name in self.arm_names if name not in self.motor_actuators]
        missing_hand = [name for name in self.hand_names if name not in self.position_actuators]
        if missing_arm or missing_hand:
            raise RuntimeError(
                f"model actuator mismatch: missing arm motors={missing_arm}, "
                f"missing hand position servos={missing_hand}")
        overlap = set(self.arm_names) & set(self.hand_names)
        if overlap:
            raise RuntimeError(f"arm/hand control groups overlap: {sorted(overlap)}")

    def _set_initial_state(self) -> None:
        initial = list(self.get_parameter("initial_arm_positions").value)
        if len(initial) != len(self.arm_names) or not np.all(np.isfinite(initial)):
            raise RuntimeError("initial_arm_positions must contain five finite values")
        for name, value in zip(self.arm_names, initial):
            joint_id = self.joint_ids[name]
            if self.model.jnt_limited[joint_id]:
                lower, upper = self.model.jnt_range[joint_id]
                value = float(np.clip(value, lower, upper))
            self.data.qpos[self.qpos_addresses[name]] = value
        self.data.qvel[:] = 0.0

    def _initialize_controls(self) -> None:
        # Do not teleport qpos every step. Non-target G1 joints are held by their
        # generated position actuators; all direct motors start at zero torque.
        for actuator_ids in self.motor_actuators.values():
            for actuator_id in actuator_ids:
                self.data.ctrl[actuator_id] = 0.0
        for name, actuator_ids in self.position_actuators.items():
            target = float(self.data.qpos[self.qpos_addresses[name]])
            for actuator_id in actuator_ids:
                lower, upper = self.model.actuator_ctrlrange[actuator_id]
                self.data.ctrl[actuator_id] = np.clip(target, lower, upper)

    def _arm_callback(self, command: JointState) -> None:
        if self.arm_mode != "torque":
            return
        updated: dict[str, float] = {}
        for index, name in enumerate(command.name):
            if name in self.arm_names and index < len(command.effort):
                value = float(command.effort[index])
                if np.isfinite(value):
                    updated[name] = value
        if len(updated) == len(self.arm_names):
            with self.lock:
                self.arm_torque = updated
                self.last_arm_command = time.monotonic()

    def _arm_position_callback(self, command: JointState) -> None:
        if self.arm_mode != "position":
            return
        # Accept complete, named commands atomically; never retain a half update.
        if (len(command.name) != len(self.arm_names)
                or len(command.position) != len(self.arm_names)
                or len(set(command.name)) != len(self.arm_names)
                or set(command.name) != set(self.arm_names)):
            self.get_logger().warning("Rejected arm position command: require all five unique joint names and positions")
            return
        values = dict(zip(command.name, command.position))
        target = np.array([values[name] for name in self.arm_names], dtype=np.float64)
        if not np.all(np.isfinite(target)):
            self.get_logger().warning("Rejected non-finite arm position command")
            return
        for name, value in zip(self.arm_names, target):
            joint_id = self.joint_ids[name]
            if self.model.jnt_limited[joint_id]:
                lower, upper = self.model.jnt_range[joint_id]
                if not lower <= value <= upper:
                    self.get_logger().warning(f"Rejected arm position command: {name} outside [{lower}, {upper}] rad")
                    return
        with self.lock:
            self.arm_position_target = target

    def _hand_callback(self, command: JointState) -> None:
        updated: dict[str, float] = {}
        for index, name in enumerate(command.name):
            if name in self.hand_names and index < len(command.position):
                value = float(command.position[index])
                if np.isfinite(value):
                    updated[name] = value
        if updated:
            with self.lock:
                self.hand_position.update(updated)

    def _apply_commands(self) -> None:
        with self.lock:
            arm_fresh = time.monotonic() - self.last_arm_command <= self.arm_timeout
            arm_torque = dict(self.arm_torque)
            hand_position = dict(self.hand_position)
            position_target = self.arm_position_target.copy()

        if self.arm_mode == "position":
            step = self.position_max_velocity * self.model.opt.timestep
            self.arm_position_reference += np.clip(
                position_target - self.arm_position_reference, -step, step)

        for index, name in enumerate(self.arm_names):
            dof = self.dof_addresses[name]
            # Stale command fallback is pure gravity only. Do not use the live
            # qfrc_bias here: at nonzero velocity it also contains Coriolis and
            # centrifugal terms from an older state.
            torque = (arm_torque.get(name, 0.0) if arm_fresh
                      else float(self.gravity_torque[dof]))
            if self.arm_mode == "position":
                error = self.arm_position_reference[index] - self.data.qpos[self.qpos_addresses[name]]
                torque = (self.gravity_torque[dof] + self.position_kp[index] * error
                          - self.position_kd[index] * self.data.qvel[dof])
                torque = float(np.clip(torque, -self.position_max_torque[index],
                                       self.position_max_torque[index]))
            for actuator_id in self.motor_actuators[name]:
                lower, upper = self.model.actuator_ctrlrange[actuator_id]
                if self.model.actuator_ctrllimited[actuator_id]:
                    torque = float(np.clip(torque, lower, upper))
                self.data.ctrl[actuator_id] = torque

        for name, target in hand_position.items():
            if name not in self.position_actuators:
                continue
            for actuator_id in self.position_actuators[name]:
                lower, upper = self.model.actuator_ctrlrange[actuator_id]
                self.data.ctrl[actuator_id] = float(np.clip(target, lower, upper))

    def _physics_step(self) -> None:
        self._apply_commands()
        mujoco.mj_step(self.model, self.data)
        finite = (
            np.all(np.isfinite(self.data.qpos))
            and np.all(np.isfinite(self.data.qvel))
            and np.all(np.isfinite(self.data.qacc))
        )
        if not finite:
            self.get_logger().error(
                "NON-FINITE MuJoCo state detected; restoring initial simulation state")
            self._restore_initial_state()
            return
        max_velocity = float(np.max(np.abs(self.data.qvel)))
        if max_velocity > self.instability_velocity:
            self.get_logger().warning(
                f"SIMULATION UNSTABLE: max|qvel|={max_velocity:.2f} rad/s, "
                f"ncon={self.data.ncon}")
        self.publish_elapsed += self.model.opt.timestep
        if self.publish_elapsed + 1.0e-12 >= self.publish_period:
            self.publish_elapsed -= self.publish_period
            # mj_step integrates qpos after evaluating contacts/sites. Refresh
            # derived fields so all observations describe the current qpos.
            mujoco.mj_forward(self.model, self.data)
            stamp = self.get_clock().now().to_msg()
            self._publish_state(stamp)
            self._publish_ee_pose(stamp)
            self.contacts_pub.publish(self._contact_state(stamp))
            self._publish_gravity()
        if time.monotonic() - self.last_diagnostic >= self.diagnostic_period:
            self._log_diagnostics()
            self.last_diagnostic = time.monotonic()
        self.step_count += 1

    def _sync_viewer(self) -> None:
        # Rendering runs at display rate, independently of the 500 Hz physics
        # timer. Full model synchronization on every step can stall physics.
        if self.viewer is not None and self.viewer.is_running():
            start = time.monotonic()
            if self.viewer_state_only:
                self.viewer.sync(state_only=True)
            else:
                self.viewer.sync()
            self.viewer_sync_seconds += time.monotonic() - start
            self.viewer_sync_count += 1

    def _restore_initial_state(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.initial_qpos
        self.data.qvel[:] = 0.0
        self._initialize_controls()
        mujoco.mj_forward(self.model, self.data)
        self.gravity_torque = self._compute_gravity()
        self.last_diagnostic_sim_time = float(self.data.time)
        self.last_diagnostic = time.monotonic()
        self.last_diagnostic_step = self.step_count
        with self.lock:
            self.arm_torque.clear()
            self.hand_position.clear()
            self.last_arm_command = -1.0
            self.arm_position_target = np.array([
                self.data.qpos[self.qpos_addresses[name]] for name in self.arm_names])
            self.arm_position_reference = self.arm_position_target.copy()
        self.publish_elapsed = 0.0

    def _log_diagnostics(self) -> None:
        wall_time = time.monotonic()
        elapsed = max(wall_time - self.last_diagnostic, 1.0e-9)
        real_time_factor = (self.data.time - self.last_diagnostic_sim_time) / elapsed
        physics_hz = (self.step_count - self.last_diagnostic_step) / elapsed
        viewer_sync_ms = (1000.0 * self.viewer_sync_seconds / self.viewer_sync_count
                          if self.viewer_sync_count else 0.0)
        self.last_diagnostic_sim_time = float(self.data.time)
        self.last_diagnostic_step = self.step_count
        self.viewer_sync_seconds = 0.0
        self.viewer_sync_count = 0
        arm_q = [self.data.qpos[self.qpos_addresses[name]] for name in self.arm_names]
        arm_dq = [self.data.qvel[self.dof_addresses[name]] for name in self.arm_names]
        arm_ctrl = [
            self.data.ctrl[self.motor_actuators[name][0]] for name in self.arm_names
        ]
        hand_q = [self.data.qpos[self.qpos_addresses[name]] for name in self.hand_names]
        hand_ctrl = [
            self.data.ctrl[self.position_actuators[name][0]] for name in self.hand_names
        ]
        maximum_acceleration = float(np.max(np.abs(self.data.qacc)))
        maximum_constraint = float(np.max(np.abs(self.data.qfrc_constraint)))
        fmt = lambda values: "[" + ", ".join(f"{value:.3f}" for value in values) + "]"
        self.get_logger().info(
            "SIM_DIAG "
            f"rtf={real_time_factor:.3f} physics_hz={physics_hz:.1f} "
            f"viewer_sync_ms={viewer_sync_ms:.2f} "
            f"q_arm={fmt(arm_q)} dq_arm={fmt(arm_dq)} tau={fmt(arm_ctrl)} "
            f"q_hand={fmt(hand_q)} hand_ref={fmt(hand_ctrl)} "
            f"max|qacc|={maximum_acceleration:.2f} "
            f"max|constraint|={maximum_constraint:.2f} ncon={self.data.ncon}")

    def _publish_state(self, stamp=None) -> None:
        message = JointState()
        message.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        for name, joint_id in self.joint_ids.items():
            qpos = self.qpos_addresses[name]
            dof = self.dof_addresses[name]
            message.name.append(name)
            message.position.append(float(self.data.qpos[qpos]))
            message.velocity.append(float(self.data.qvel[dof]))
            message.effort.append(float(self.data.qfrc_actuator[dof]))
        self.joint_state_pub.publish(message)

    def _publish_ee_pose(self, stamp=None) -> None:
        message = PoseStamped()
        message.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        message.header.frame_id = "world"
        position = self.data.site_xpos[self.ee_site_id]
        rotation = self.data.site_xmat[self.ee_site_id]
        quaternion = np.empty(4, dtype=np.float64)  # MuJoCo order: w,x,y,z
        mujoco.mju_mat2Quat(quaternion, rotation)
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.w = float(quaternion[0])
        message.pose.orientation.x = float(quaternion[1])
        message.pose.orientation.y = float(quaternion[2])
        message.pose.orientation.z = float(quaternion[3])
        self.ee_pose_pub.publish(message)

    def _contact_state(self, stamp) -> ContactArray:
        message = ContactArray()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            value = Contact()
            value.geom1_id, value.geom2_id = (int(i) for i in contact.geom)
            for number, geom_id in enumerate((value.geom1_id, value.geom2_id), start=1):
                geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                body_id = int(self.model.geom_bodyid[geom_id])
                body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                setattr(value, f"geom{number}_name", geom_name or f"geom_{geom_id}")
                setattr(value, f"body{number}_name", body_name or f"body_{body_id}")
            value.position.x, value.position.y, value.position.z = map(float, contact.pos)
            frame = contact.frame.reshape(3, 3)
            value.normal.x, value.normal.y, value.normal.z = map(float, frame[0])
            value.distance = float(contact.dist)
            value.active = bool(contact.efc_address >= 0)
            local_wrench = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(self.model, self.data, index, local_wrench)
            value.normal_force = float(local_wrench[0])
            world_force = frame.T @ local_wrench[:3]
            world_torque = frame.T @ local_wrench[3:]
            value.wrench.force.x, value.wrench.force.y, value.wrench.force.z = map(float, world_force)
            value.wrench.torque.x, value.wrench.torque.y, value.wrench.torque.z = map(float, world_torque)
            message.contacts.append(value)
        return message

    def _publish_gravity(self) -> None:
        self.gravity_torque = self._compute_gravity()
        message = Float64MultiArray()
        message.data = [
            float(self.gravity_torque[self.dof_addresses[name]])
            for name in self.arm_names
        ]
        self.gravity_pub.publish(message)

    def _compute_gravity(self) -> np.ndarray:
        """Compute gravity at q with qvel=0, excluding velocity bias forces."""
        self.gravity_data.qpos[:] = self.data.qpos
        self.gravity_data.qvel[:] = 0.0
        self.gravity_data.qacc[:] = 0.0
        mujoco.mj_forward(self.model, self.gravity_data)
        return self.gravity_data.qfrc_bias.copy()

    def destroy_node(self) -> bool:
        if self.viewer is not None:
            self.viewer.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MuJoCoBridgeNode()
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
