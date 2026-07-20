#!/usr/bin/env python3
"""
MuJoCo physics bridge — ROS 2 ↔ MuJoCo simulation with position-servo actuation.

Publishes: /joint_states, /actual_ee_pose, /actual_hand_orientation,
           /mujoco_jacobian, /mujoco_wrist_jacobian (plus /camera/rgb, /camera/depth, /wrist_ft).
Subscribes: /joint_commands (arm 4-DOF), /hand_commands (hand 23-DOF).

Dependencies: pip install mujoco opencv-python "numpy<2"
"""

import os
import sys
import threading
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# MuJoCo imports（延迟导入，方便 import 错误时给出友好提示）
# ---------------------------------------------------------------------------
try:
    import mujoco
    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False

try:
    import cv2 as cv
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# ---------------------------------------------------------------------------
# ROS2 imports
# ---------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import WrenchStamped, PointStamped, QuaternionStamped
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge


# ===========================================================================
# MODULE: MuJoCoBridgeNode — ROS2 node that steps MuJoCo and bridges topics
# ===========================================================================
class MuJoCoBridgeNode(Node):
    """MuJoCo physics bridge node.

    Responsibilities:
      1. Load MJCF scene, initialize MuJoCo model and data
      2. Subscribe /joint_commands and /hand_commands, cache latest commands
      3. Step MuJoCo at physics_rate (200 Hz)
      4. Publish /joint_states (positions + velocities)
      5. Render and publish virtual camera images at camera_rate
      6. Publish /wrist_ft (force/torque)

    Attributes:
        model: MuJoCo MjModel.
        data: MuJoCo MjData.
        renderer: MuJoCo Renderer (off-screen rendering).
        bridge: CvBridge (ROS2 ↔ OpenCV image conversion).
        joint_cmd: Latest arm command dict {name: rad}.
        hand_cmd: Latest hand command dict {name: rad}.
        lock: Threading lock protecting command dicts.
    """

    def __init__(self):
        super().__init__('mujoco_bridge_node')

        # -----------------------------------------------------------------
        # SECTION: 检查依赖
        # -----------------------------------------------------------------
        if not HAS_MUJOCO:
            self.get_logger().fatal(
                "MuJoCo 未安装！请执行: pip install mujoco"
            )
            raise ImportError("mujoco not installed")

        # -----------------------------------------------------------------
        # SECTION: 声明参数
        # -----------------------------------------------------------------
        self.declare_parameter('scene_path', '')
        self.declare_parameter('physics_rate', 200.0)       # Hz
        self.declare_parameter('camera_rate', 15.0)         # Hz（相机渲染较耗性能）
        self.declare_parameter('camera_name', 'head_cam')   # MJCF 中相机名称
        self.declare_parameter('publish_camera', True)
        self.declare_parameter('publish_ft', True)
        self.declare_parameter('base_frame', 'pelvis')
        self.declare_parameter('viewer', False)             # 是否打开交互式窗口
        self.declare_parameter('domain_id', 0)              # DDS domain（0=默认）
        self.declare_parameter('active_joints', [
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint',
        ])  # G1 右臂 7-DOF (arm 4 + wrist 3)，手指关节由 hand_controller 管理
        self.declare_parameter('clamp_arm_joints', False)  # 手臂关节硬夹持（默认关闭，靠位置伺服）
        self.declare_parameter('hard_clamp_pelvis', True)   # 硬夹持骨盆（固定机器人位置）
        self.declare_parameter('hard_clamp_joints', True)   # 硬夹持非右臂关节（除右臂外全部锁死）

        # 手指执行器增益缩放（MJCF kp ~0.2-0.7 太低，仿真中需大幅提升）
        self.declare_parameter('finger_kp_scale', 60.0)   # kp 倍率：默认 60× → kp = 5×60 = 300
        self.declare_parameter('finger_kv_scale', 1.0)    # kv 倍率：默认 1× (no boost needed)
        self.declare_parameter('finger_ema_alpha', 1.0)   # EMA 平滑系数（手指，1.0=无平滑）
        self.declare_parameter('wrist_ema_alpha', 1.0)  # EMA 平滑系数（手腕, 1.0=无平滑）

        # 实际读取硬夹持参数值（在 MuJoCo 加载后赋值）
        self.hard_clamp_pelvis = False
        self.hard_clamp_joints = False

        # -----------------------------------------------------------------
        # SECTION: 加载 MuJoCo 场景
        # -----------------------------------------------------------------
        scene_path = self.get_parameter('scene_path').get_parameter_value().string_value
        if not scene_path:
            # 默认路径：<package_share>/config/g1_grasp_scene.xml
            from ament_index_python.packages import get_package_share_directory
            try:
                share_dir = get_package_share_directory('g1_right_arm_reach')
                scene_path = os.path.join(share_dir, 'config', 'g1_grasp_scene.xml')
            except Exception:
                self.get_logger().fatal(
                    "无法解析场景文件路径！请通过 --ros-args -p scene_path:=/path/to/scene.xml 指定"
                )
                raise

        if not os.path.exists(scene_path):
            self.get_logger().fatal(f"场景文件不存在: {scene_path}")
            raise FileNotFoundError(scene_path)

        self.get_logger().info(f"加载 MuJoCo 场景: {scene_path}")
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.get_logger().info(
            f"模型加载完成: nq={self.model.nq}, nv={self.model.nv}, "
            f"nu={self.model.nu}, nsensordata={self.model.nsensordata}"
        )

        # ─────────────────────────────────────────────────────────────
        # 诊断：统计 body 数量（详细列表用 debug 级别）
        # ─────────────────────────────────────────────────────────────
        wuji_bodies = []
        for bid in range(self.model.nbody):
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
            if bname and ("palm" in bname or "finger" in bname):
                wuji_bodies.append(bname)
        self.get_logger().info(f"MJCF: {self.model.nbody} bodies ({len(wuji_bodies)} Wuji hand bodies)")
        self.get_logger().debug(f"Wuji hand bodies: {', '.join(wuji_bodies)}")

        # -----------------------------------------------------------------
        # SECTION: 建立关节名 → 所有执行器 ID 映射
        # -----------------------------------------------------------------
        # 每个关节可能被多个执行器驱动（MJCF include 的 <motor> + 我们的 <position>）。
        # 存储全部执行器 ID，统一设 ctrl 使所有执行器朝向同一目标。
        self.actuator_map = {}       # {joint_name: [act_id, ...]} — 所有执行器
        self.actuator_map_primary = {}  # {joint_name: act_id} — 主执行器（诊断用）
        for act_id in range(self.model.nu):
            act_joint_id = self.model.actuator_trnid[act_id][0]  # 传动目标关节
            if act_joint_id >= 0 and act_joint_id < self.model.njnt:
                joint_name = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, act_joint_id
                )
                if joint_name:
                    if joint_name not in self.actuator_map:
                        self.actuator_map[joint_name] = []
                    self.actuator_map[joint_name].append(act_id)
                    self.actuator_map_primary[joint_name] = act_id  # 最后一个 = position 执行器
                    self.get_logger().debug(
                        f"  执行器映射: '{joint_name}' ← act[{act_id}]"
                    )

        # ─────────────────────────────────────────────────────────────
        # 关键区分：motor（力矩控制）vs position servo（位置伺服）
        #
        # MuJoCo 中 <motor> 和 <position> 的 gaintype 都是 0 (mjGAIN_FIXED)！
        # 区别在于 biastype：
        #   biastype=0 (mjBIAS_NONE)  → <motor>:    force = gainprm[0] × ctrl
        #                                            ctrl 是力矩指令，应设为 0（禁用）
        #   biastype=1 (mjBIAS_AFFINE) → <position>: force = kp×(ctrl−q) − kv×qvel
        #                                            ctrl 是目标位置
        #
        # <position> 的实现原理：gainprm[0]=kp, biasprm[1]=-kp, biasprm[2]=-kv
        #   → force = kp*ctrl + 0 + (-kp)*q + (-kv)*qvel = kp*(ctrl-q) - kv*qvel
        # ─────────────────────────────────────────────────────────────
        self.actuator_motor_ids = set()      # 力矩电机（ctrl→力矩，恒为 0）
        self.actuator_position_ids = set()   # 位置伺服（ctrl→目标位置）
        for act_id in range(self.model.nu):
            bt = self.model.actuator_biastype[act_id]
            if bt == 0:  # mjBIAS_NONE → pure motor
                self.actuator_motor_ids.add(act_id)
            else:        # mjBIAS_AFFINE → position servo
                self.actuator_position_ids.add(act_id)
        self.get_logger().debug(
            f"执行器分类: {len(self.actuator_motor_ids)} motor + "
            f"{len(self.actuator_position_ids)} position servo"
        )
        self.get_logger().info(
            f"执行器类型: {len(self.actuator_motor_ids)} 个 motor (力矩控制, ctrl→0), "
            f"{len(self.actuator_position_ids)} 个 position servo (位置伺服)"
        )

        # ─────────────────────────────────────────────────────────────
        # 运行时提升手指 position servo 增益
        # MJCF kp ~0.2-0.7 / kv ~0.008-0.03 — 为真实电机柔顺设计，仿真太慢
        # 通过 gainprm / biasprm 等比例缩放，保持正确位置伺服公式
        #
        # MuJoCo position servo 力公式:
        #   force = gainprm[0] × ctrl + biasprm[1] × q + biasprm[2] × q̇
        #
        # MJCF <position kp="K"> → gainprm[0]=K, biasprm[1]=−K, biasprm[2]=−kv
        #   原始: force = K×(ctrl−q) − kv×q̇
        #   scale: force = (K×s)×(ctrl−q) − (kv×s_v)×q̇  ← 正确缩放
        #   必须同时缩放 gainprm[0] 和 biasprm[1] 否则平衡点偏移。
        # ─────────────────────────────────────────────────────────────
        finger_kp_scale = self.get_parameter('finger_kp_scale').get_parameter_value().double_value
        finger_kv_scale = self.get_parameter('finger_kv_scale').get_parameter_value().double_value
        finger_count = 0
        for joint_name, act_ids in self.actuator_map.items():
            if 'finger' not in joint_name:
                continue
            for act_id in act_ids:
                if act_id not in self.actuator_position_ids:
                    continue
                old_kp = float(self.model.actuator_gainprm[act_id][0])
                old_kv_fb = float(self.model.actuator_biasprm[act_id][2])
                old_kp_fb = float(self.model.actuator_biasprm[act_id][1])

                new_kp = old_kp * finger_kp_scale
                new_kv = old_kv_fb * finger_kv_scale
                new_kp_fb = old_kp_fb * finger_kp_scale  # CRITICAL: scale position feedback

                self.model.actuator_gainprm[act_id][0] = new_kp
                self.model.actuator_biasprm[act_id][1] = new_kp_fb
                self.model.actuator_biasprm[act_id][2] = new_kv
                finger_count += 1
        self.get_logger().info(
            f"手指执行器增益缩放: {finger_count} joints, "
            f"kp×{finger_kp_scale:.0f}, kv×{finger_kv_scale:.0f}"
        )

        # 区分主动关节（右臂 5 DOF）与锁定关节（其余 18 DOF）
        active_joints = self.get_parameter('active_joints').get_parameter_value().string_array_value
        self.active_joint_names = set(active_joints)
        self.wrist_joint_names = {  # 手腕关节 — 独立 EMA 平滑抑制 yaw 颤抖
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint',
        }
        self.locked_joint_names = set()
        for joint_name in self.actuator_map:
            if joint_name not in self.active_joint_names:
                self.locked_joint_names.add(joint_name)

        if not self.actuator_map:
            self.get_logger().warn(
                "未找到任何执行器→关节映射！"
                "请确保 scene XML 中定义了 <position> 执行器"
            )
        else:
            total_acts = sum(len(v) for v in self.actuator_map.values())
            self.get_logger().info(
                f"执行器映射: {len(self.actuator_map)} 个关节, "
                f"{total_acts} 个执行器 "
                f"(主动={len(self.active_joint_names)}, "
                f"锁定={len(self.locked_joint_names)})"
            )

        # ─────────────────────────────────────────────────────────────
        # 记录初始位姿（用于夹持 floating_base + 锁定关节）
        #
        # 重要：为什么锁定关节必须硬夹持？
        #   人形机器人自重 ~35 kg，重力在各关节上产生 10–50 N·m 力矩。
        #   位置伺服 kp=500 下稳态误差 = 50/500 = 0.1 rad ≈ 5.7°/关节。
        #   22 个锁定关节各自累积 5° 误差 → 整体姿态完全塌缩。
        #   要达到 < 0.5° 误差需 kp > 5000，这种刚度在 200 Hz 下会
        #   导致数值振荡。所以用"硬夹持 + 位置伺服备份"的双重锁死策略：
        #     - 硬夹持：每步覆写 qpos，消除累积误差
        #     - 位置伺服：步内持续提供力矩，减小夹持的物理突变
        #   右臂的振荡并非由夹持引起（夹持只影响锁定关节），而是由
        #   kp=1000 + 20Hz 阶跃指令 + 积分突变三者的互动造成。
        # ─────────────────────────────────────────────────────────────
        self.init_pelvis_pos = np.array(self.data.qpos[0:3].copy(), dtype=np.float64)
        self.init_pelvis_quat = np.array(self.data.qpos[3:7].copy(), dtype=np.float64)
        # 骨盆抬高 3cm（z += 0.03），减少脚底穿透地面的深度，避免接触力反弹振荡
        self.init_pelvis_pos[2] += 0.03
        self.data.qpos[0:3] = self.init_pelvis_pos
        self.get_logger().info(
            f"骨盆初始位置 (MJCF qpos0 + 抬高): [{self.init_pelvis_pos[0]:.3f}, "
            f"{self.init_pelvis_pos[1]:.3f}, {self.init_pelvis_pos[2]:.3f}]"
        )

        # ─────────────────────────────────────────────────────────────
        # 左臂归零：MJCF 默认 qpos0 可能非零，在锁死前强制归零
        # 确保左右臂对称自然下垂
        # ─────────────────────────────────────────────────────────────
        left_arm_joints = [
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint',
            'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
        ]
        left_arm_zeroed = []
        for joint_name in left_arm_joints:
            if joint_name in self.actuator_map_primary:
                act_id = self.actuator_map_primary[joint_name]
                act_joint_id = self.model.actuator_trnid[act_id][0]
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                old_q = float(self.data.qpos[qpos_addr])
                self.data.qpos[qpos_addr] = 0.0
                short = joint_name.replace('left_', '').replace('_joint', '')
                left_arm_zeroed.append(f"{short}: {old_q:.3f}→0.000")
        if left_arm_zeroed:
            self.get_logger().info(
                f"左臂归零（锁死前）: {', '.join(left_arm_zeroed)}"
            )

        # 锁定关节双重锁死：硬夹持（qpos 覆写）+ 位置伺服（步内维持）
        # 使用 list（非 tuple）以便运行时更新手臂关节的目标值
        self.locked_joint_clamps = []  # [[qpos_addr, target_val, dof_addr], ...]
        self.locked_joint_info = []    # [(joint_name, qpos_addr, init_q), ...] 诊断用
        self.arm_clamp_index = {}      # {joint_name: index_in_clamps} 快速查找

        # 只采集非手指、非右臂的锁定关节
        # 手指关节由 hand_controller 通过 position servo 控制，硬夹持会导致
        # servo 与夹持互相对抗 → 手指无法移动 + 反作用力引起手腕颤抖
        for joint_name in self.locked_joint_names:
            if 'finger' in joint_name:
                continue  # skip — hand_controller manages these via position servo
            act_id = self.actuator_map_primary[joint_name]
            act_joint_id = self.model.actuator_trnid[act_id][0]
            if act_joint_id >= 0 and act_joint_id < self.model.njnt:
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                dof_addr = self.model.jnt_dofadr[act_joint_id]
                init_q = float(self.data.qpos[qpos_addr])
                self.locked_joint_clamps.append([qpos_addr, init_q, dof_addr])
                self.locked_joint_info.append((joint_name, qpos_addr, init_q))

        # ─────────────────────────────────────────────────────────────
        # 初始化所有执行器 ctrl = MJCF 默认关节角度（qpos0）
        # 关键：对每个关节的 ALL 执行器（电机 + 位置伺服）统一设 ctrl，
        # 使所有执行器朝同一目标施力，而非互相对抗。
        # ─────────────────────────────────────────────────────────────
        for joint_name, act_ids in self.actuator_map.items():
            # 用主执行器获取关节信息
            act_id_primary = self.actuator_map_primary[joint_name]
            act_joint_id = self.model.actuator_trnid[act_id_primary][0]
            if act_joint_id >= 0 and act_joint_id < self.model.njnt:
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                init_q = float(self.data.qpos[qpos_addr])
                # motor → ctrl=0（零力矩）；position servo → ctrl=init_q（位置目标）
                for act_id in act_ids:
                    if act_id in self.actuator_motor_ids:
                        self.data.ctrl[act_id] = 0.0
                    else:
                        self.data.ctrl[act_id] = init_q
                # 仅归零速度（安全措施）
                dof_addr = self.model.jnt_dofadr[act_joint_id]
                if dof_addr >= 0 and dof_addr < self.model.nv:
                    self.data.qvel[dof_addr] = 0.0
                self.get_logger().debug(
                    f"  初始化 '{joint_name}': q={init_q:.3f} rad, "
                    f"{len(act_ids)} 个执行器 "
                    f"(motor={sum(1 for a in act_ids if a in self.actuator_motor_ids)}→0, "
                    f"pos={sum(1 for a in act_ids if a in self.actuator_position_ids)}→{init_q:.3f})"
                )

        total_motor_ctrl = sum(1 for act_id in self.actuator_motor_ids)
        total_pos_ctrl = sum(1 for act_id in self.actuator_position_ids)
        self.get_logger().info(
            f"执行器 ctrl 初始化完成: {total_motor_ctrl} 个 motor → ctrl=0 (禁用), "
            f"{total_pos_ctrl} 个 position servo → ctrl=qpos0"
        )

        # 读取硬夹持参数（必须在 MuJoCo 加载后）
        self.hard_clamp_pelvis = self.get_parameter('hard_clamp_pelvis').get_parameter_value().bool_value
        self.hard_clamp_joints = self.get_parameter('hard_clamp_joints').get_parameter_value().bool_value
        self.get_logger().info(
            f"硬夹持配置: pelvis={'ON' if self.hard_clamp_pelvis else 'OFF'}, "
            f"joints={'ON' if self.hard_clamp_joints else 'OFF'}"
        )

        # ─────────────────────────────────────────────────────────────
        # 设置右臂 "预备跑" 姿态（微屈肘，手在前方 ≈ 跑步预备姿势）
        # 在 MJCF 默认站立姿态基础上覆盖右臂 5 关节
        # ─────────────────────────────────────────────────────────────

        # EMA 平滑滤波器状态（必须在 ready pose 之前初始化）
        self.ctrl_smoothed = {}  # {actuator_id: smoothed_ctrl_value}

        # 诊断：打印右臂关节在 MJCF 默认姿态下的 qpos（就绪姿态覆盖前）
        default_arm_q = []
        for joint_name in ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
                           'right_shoulder_yaw_joint', 'right_elbow_joint',
                           'right_wrist_roll_joint', 'right_wrist_pitch_joint',
                           'right_wrist_yaw_joint']:
            if joint_name in self.actuator_map:
                act_id = self.actuator_map_primary[joint_name]
                act_joint_id = self.model.actuator_trnid[act_id][0]
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                default_q = float(self.data.qpos[qpos_addr])
                short = joint_name.replace('right_', '').replace('_joint', '')
                default_arm_q.append(f"{short}={default_q:.3f}")
        self.get_logger().info(f"右臂 MJCF 默认 qpos: {', '.join(default_arm_q)}")

        arm_ready_pose = {
            'right_shoulder_pitch_joint': -0.3,   # slight forward lean (avoids Jacobian singularity)
            'right_shoulder_roll_joint':   0.0,
            'right_shoulder_yaw_joint':    0.0,
            'right_elbow_joint':           0.3,   # slight bend
            'right_wrist_roll_joint':      0.0,   # neutral — palm quat handles ALL orientation
            'right_wrist_pitch_joint':     0.0,
            'right_wrist_yaw_joint':       0.0,
        }
        for joint_name, angle in arm_ready_pose.items():
            if joint_name in self.actuator_map:
                act_ids = self.actuator_map[joint_name]
                act_id_primary = self.actuator_map_primary[joint_name]
                act_joint_id = self.model.actuator_trnid[act_id_primary][0]
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                self.data.qpos[qpos_addr] = angle
                # motor → ctrl=0（零力矩）；position servo → ctrl=angle（位置目标）
                for act_id in act_ids:
                    if act_id in self.actuator_motor_ids:
                        self.data.ctrl[act_id] = 0.0
                    else:
                        self.data.ctrl[act_id] = angle
                        self.ctrl_smoothed[act_id] = float(angle)  # EMA 滤波器初值（仅位置伺服）
                n_m = sum(1 for a in act_ids if a in self.actuator_motor_ids)
                n_p = sum(1 for a in act_ids if a in self.actuator_position_ids)
                self.get_logger().info(
                    f"  右臂就绪姿态: {joint_name} = {angle:.3f} rad "
                    f"(motor×{n_m}→0, pos×{n_p}→{angle:.3f})"
                )
            else:
                self.get_logger().warn(
                    f"  关节 '{joint_name}' 未在执行器映射中找到，跳过"
                )

        # ─────────────────────────────────────────────────────────────
        # 手指默认半握姿态（Wuji Hand 20-DOF）
        #   joint1 = MCP 弯曲, joint2 = 外展/内收, joint3 = PIP, joint4 = DIP
        #   finger1 = 拇指（对掌，joint2 抬高 > 食指）, finger2-5 = 四指并拢
        # ─────────────────────────────────────────────────────────────
        finger_grasp_pose = {
            # 拇指 (finger1) — 外展抬高 (joint2=0.3)，处于对掌预备位
            'right_finger1_joint1': 0.6,
            'right_finger1_joint2': 0.3,
            'right_finger1_joint3': 0.5,
            'right_finger1_joint4': 0.4,
            # 食指 (finger2) — 四指并拢 (joint2=0)
            'right_finger2_joint1': 0.8,
            'right_finger2_joint2': 0.0,
            'right_finger2_joint3': 0.7,
            'right_finger2_joint4': 0.6,
            # 中指 (finger3)
            'right_finger3_joint1': 0.8,
            'right_finger3_joint2': 0.0,
            'right_finger3_joint3': 0.7,
            'right_finger3_joint4': 0.6,
            # 无名指 (finger4)
            'right_finger4_joint1': 0.8,
            'right_finger4_joint2': 0.0,
            'right_finger4_joint3': 0.7,
            'right_finger4_joint4': 0.6,
            # 小指 (finger5)
            'right_finger5_joint1': 0.8,
            'right_finger5_joint2': 0.0,
            'right_finger5_joint3': 0.7,
            'right_finger5_joint4': 0.6,
        }
        finger_count = 0
        for joint_name, angle in finger_grasp_pose.items():
            if joint_name in self.actuator_map:
                act_ids = self.actuator_map[joint_name]
                act_id_primary = self.actuator_map_primary[joint_name]
                act_joint_id = self.model.actuator_trnid[act_id_primary][0]
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                self.data.qpos[qpos_addr] = angle
                for act_id in act_ids:
                    if act_id in self.actuator_position_ids:
                        self.data.ctrl[act_id] = angle
                        self.ctrl_smoothed[act_id] = float(angle)
                finger_count += 1
        if finger_count > 0:
            self.get_logger().info(
                f"手指半握姿态: {finger_count} 关节已设置 (semi-grasp)"
            )

        # 手臂关节硬夹持（由参数 clamp_arm_joints 控制）
        clamp_arm = self.get_parameter('clamp_arm_joints').get_parameter_value().bool_value
        if clamp_arm:
            self.get_logger().info(
                "手臂关节硬夹持已启用 (--ros-args -p clamp_arm_joints:=true)"
            )
            for joint_name in self.active_joint_names:
                if joint_name in self.actuator_map:
                    act_id = self.actuator_map_primary[joint_name]
                    act_joint_id = self.model.actuator_trnid[act_id][0]
                    if act_joint_id >= 0 and act_joint_id < self.model.njnt:
                        qpos_addr = self.model.jnt_qposadr[act_joint_id]
                        dof_addr = self.model.jnt_dofadr[act_joint_id]
                        init_q = float(self.data.qpos[qpos_addr])  # 此时已是就绪姿态
                        idx = len(self.locked_joint_clamps)
                        self.locked_joint_clamps.append([qpos_addr, init_q, dof_addr])
                        self.locked_joint_info.append((joint_name, qpos_addr, init_q))
                        self.arm_clamp_index[joint_name] = idx  # 快速查找
        self.get_logger().info(
            f"硬夹持关节总数: {len(self.locked_joint_clamps)} 个 "
            f"(其中手臂={len(self.active_joint_names) if clamp_arm else 0})"
        )

        # -----------------------------------------------------------------
        # SECTION: 关节传感器映射
        # -----------------------------------------------------------------
        # 建立 sensor 名 → 传感器 ID 映射（供快速查询）
        self.sensor_pos_ids = {}   # {joint_name: sensor_id} for jointpos
        self.sensor_vel_ids = {}   # {joint_name: sensor_id} for jointvel
        for sens_id in range(self.model.nsensordata):
            sens_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, sens_id
            )
            if sens_name is None:
                continue
            sens_type = self.model.sensor_type[sens_id]
            # 获取传感器关联的关节
            obj_id = self.model.sensor_objid[sens_id]
            if obj_id >= 0:
                obj_name = mujoco.mj_id2name(
                    self.model, self.model.sensor_objtype[sens_id], obj_id
                )
                if sens_type == mujoco.mjtSensor.mjSENS_JOINTPOS:
                    self.sensor_pos_ids[obj_name] = sens_id
                elif sens_type == mujoco.mjtSensor.mjSENS_JOINTVEL:
                    self.sensor_vel_ids[obj_name] = sens_id

        # -----------------------------------------------------------------
        # SECTION: 初始化渲染器（离屏）
        # -----------------------------------------------------------------
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.camera_id = -1
        for cam_id in range(self.model.ncam):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_id)
            if name == camera_name:
                self.camera_id = cam_id
                break

        if self.camera_id >= 0:
            cam_res = self.model.cam_resolution[self.camera_id]
            self.get_logger().info(
                f"虚拟相机 '{camera_name}' (ID={self.camera_id}): "
                f"{cam_res[0]}×{cam_res[1]}"
            )
            self.renderer = mujoco.Renderer(self.model, cam_res[0], cam_res[1])
        else:
            self.get_logger().warn(f"未找到相机 '{camera_name}'，相机发布禁用")
            self.renderer = None

        # -----------------------------------------------------------------
        # SECTION: cv_bridge（ROS2 ↔ OpenCV 图像转换）
        # -----------------------------------------------------------------
        if HAS_CV2:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            if self.get_parameter('publish_camera').get_parameter_value().bool_value:
                self.get_logger().warn(
                    "cv_bridge 不可用，相机发布将使用 numpy→Image 手动转换"
                )

        # -----------------------------------------------------------------
        # SECTION: ROS2 订阅 & 发布
        # -----------------------------------------------------------------
        self.joint_cmd = {}        # {joint_name: position_rad} — arm+wrist (7-DOF)
        self.joint_cmd_effort = {}  # {joint_name: effort_Nm} (torque diagnostics)
        self.hand_cmd = {}          # {joint_name: position_rad} — fingers (20-DOF)
        self.lock = threading.Lock()

        self.cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self._cmd_callback, 10
        )

        self.hand_cmd_sub = self.create_subscription(
            JointState, '/hand_commands', self._hand_cmd_callback, 10
        )

        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        publish_camera = self.get_parameter('publish_camera').get_parameter_value().bool_value
        if publish_camera and self.camera_id >= 0:
            self.rgb_pub = self.create_publisher(Image, '/camera/rgb', 10)
            self.depth_pub = self.create_publisher(Image, '/camera/depth', 10)
        else:
            self.rgb_pub = None
            self.depth_pub = None

        publish_ft = self.get_parameter('publish_ft').get_parameter_value().bool_value
        if publish_ft:
            self.ft_pub = self.create_publisher(WrenchStamped, '/wrist_ft', 10)
        else:
            self.ft_pub = None

        # --- MuJoCo 真实末端位置发布（供 PID 使用，消除 URDF/MJCF FK 差异）---
        # 29-DOF: right_wrist_yaw_link (最后腕关节的 link)
        # 23-DOF: right_wrist_roll_rubber_hand (腕末端 rubber hand)
        self.ee_body_id = -1
        for bid in range(self.model.nbody):
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
            if bname == 'right_wrist_yaw_link':
                self.ee_body_id = bid
                break
        if self.ee_body_id < 0:
            for bid in range(self.model.nbody):
                bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
                if bname == 'right_wrist_roll_rubber_hand':
                    self.ee_body_id = bid
                    break
        if self.ee_body_id >= 0:
            ee_body_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, self.ee_body_id)
            self.ee_pub = self.create_publisher(PointStamped, '/actual_ee_pose', 10)
            self.get_logger().info(
                f"真实末端位置发布: body='{ee_body_name}' (ID={self.ee_body_id})"
            )
        else:
            self.ee_pub = None
            self.get_logger().warn(
                "未找到 right_wrist_yaw_link 或 right_wrist_roll_rubber_hand body，"
                "末端位置不发布"
            )

        # ─────────────────────────────────────────────────────────────
        # Hand body (palm) for orientation publishing (Plan C: wrist in Hand module)
        # ─────────────────────────────────────────────────────────────
        self.hand_body_id = -1
        for bid in range(self.model.nbody):
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
            if bname == 'right_palm_link':
                self.hand_body_id = bid
                break
        if self.hand_body_id < 0:
            # fallback: use wrist_yaw_link
            self.hand_body_id = self.ee_body_id

        if self.hand_body_id >= 0:
            hb_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, self.hand_body_id)
            self.hand_ori_pub = self.create_publisher(
                QuaternionStamped, '/actual_hand_orientation', 10)
            self.get_logger().info(
                f"手部朝向发布: body='{hb_name}' (ID={self.hand_body_id})"
            )
        else:
            self.hand_ori_pub = None
            self.get_logger().warn("未找到手部body，朝向不发布")

        # --- MuJoCo 数值 Jacobian 发布（供 PID 使用，消除 URDF/MJCF Jacobian 差异）---
        arm_joint_order = [
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint',
        ]
        self.arm_joint_qpos_addrs = []  # [qpos_addr, ...] 按 arm_joint_order 顺序
        for joint_name in arm_joint_order:
            if joint_name in self.actuator_map_primary:
                act_id = self.actuator_map_primary[joint_name]
                act_joint_id = self.model.actuator_trnid[act_id][0]
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                self.arm_joint_qpos_addrs.append(qpos_addr)
            else:
                self.get_logger().warn(f"Jacobian: 关节 '{joint_name}' 不在 actuator_map 中")
        if self.ee_body_id >= 0 and len(self.arm_joint_qpos_addrs) >= 7:
            self.jac_pub = self.create_publisher(Float64MultiArray, '/mujoco_jacobian', 10)
            self.get_logger().info(
                f"MuJoCo 数值 Jacobian 发布已启用 "
                f"({len(self.arm_joint_qpos_addrs)} 关节: arm 4 + wrist 3)"
            )
        else:
            self.jac_pub = None
            self.get_logger().warn(
                f"Jacobian 发布禁用（EE body={self.ee_body_id}, "
                f"arm joint qpos_addrs={len(self.arm_joint_qpos_addrs)} 个）"
            )

        # ─────────────────────────────────────────────────────────────
        # Wrist orientation Jacobian (3×3) for hand_controller (Plan C)
        # ─────────────────────────────────────────────────────────────
        wrist_joint_order = [
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint',
        ]
        self.wrist_joint_qpos_addrs = []
        for joint_name in wrist_joint_order:
            if joint_name in self.actuator_map_primary:
                act_id = self.actuator_map_primary[joint_name]
                act_joint_id = self.model.actuator_trnid[act_id][0]
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                self.wrist_joint_qpos_addrs.append(qpos_addr)
        if self.hand_body_id >= 0 and len(self.wrist_joint_qpos_addrs) == 3:
            self.wrist_jac_pub = self.create_publisher(
                Float64MultiArray, '/mujoco_wrist_jacobian', 10)
            self.get_logger().info(
                f"Wrist orientation Jacobian (3×3) → /mujoco_wrist_jacobian"
            )
        else:
            self.wrist_jac_pub = None
            self.get_logger().warn(
                f"Wrist Jacobian 发布禁用 "
                f"(hand_body={self.hand_body_id}, wrist_addrs={len(self.wrist_joint_qpos_addrs)})"
            )

        # -----------------------------------------------------------------
        # SECTION: 仿真定时器
        # -----------------------------------------------------------------
        physics_rate = self.get_parameter('physics_rate').get_parameter_value().double_value
        self.physics_dt = 1.0 / physics_rate
        self.model.opt.timestep = self.physics_dt

        physics_period_ms = int(1000.0 / physics_rate)
        self.physics_timer = self.create_timer(
            physics_period_ms / 1000.0, self._physics_step
        )

        # 相机发布计数器（降频渲染）
        camera_rate = self.get_parameter('camera_rate').get_parameter_value().double_value
        self.camera_step_interval = max(1, int(physics_rate / camera_rate))
        self.step_counter = 0

        # -----------------------------------------------------------------
        # SECTION: 交互式查看器（可选）
        # -----------------------------------------------------------------
        self.viewer_enabled = self.get_parameter('viewer').get_parameter_value().bool_value
        if self.viewer_enabled:
            try:
                import glfw  # noqa: F401
                from mujoco import viewer
                self.viewer = viewer.launch_passive(self.model, self.data)
                self.get_logger().info("MuJoCo 交互式查看器已启动")
            except Exception as e:
                self.get_logger().warn(f"无法启动查看器: {e}")
                self.viewer_enabled = False
                self.viewer = None
        else:
            self.viewer = None

        # -----------------------------------------------------------------
        # SECTION: 就绪
        # -----------------------------------------------------------------
        self.get_logger().info(
            f"MuJoCoBridgeNode 就绪 | 物理频率: {physics_rate:.0f} Hz | "
            f"相机频率: {camera_rate:.0f} Hz | "
            f"执行器: {len(self.actuator_map)} 个"
        )

    # =========================================================================
    # MODULE: Subscription Callbacks
    # =========================================================================

    def _cmd_callback(self, msg: JointState):
        """缓存 /joint_commands 中的关节位置和力矩。

        torque_mode: 位置指令已通过 PID 的 τ→q 转换编码力矩前馈
        （q_cmd = q_ik + (τ_cmd + Kv·dq) / Kp_inner）。
        桥接节点直接使用位置指令，无需额外处理。

        线程安全：使用 lock 保护 joint_cmd 字典的写入。

        Args:
            msg: sensor_msgs/JointState，来自 reach_node 的关节指令。
        """
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.joint_cmd[name] = pos
            # 缓存力矩信息（用于诊断日志）
            for name, eff in zip(msg.name, msg.effort):
                self.joint_cmd_effort[name] = eff

    def _hand_cmd_callback(self, msg: JointState):
        """缓存 /hand_commands 中的手指关节位置。

        Args:
            msg: sensor_msgs/JointState，来自 hand_controller 的手指关节指令。
        """
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.hand_cmd[name] = pos

    # =========================================================================
    # MODULE: Physics Loop
    # =========================================================================

    def _physics_step(self):
        """执行一个 MuJoCo 物理步进。

        流程：
          1. 将最新的 /joint_commands 写入 MuJoCo ctrl
          2. 调用 mj_step() 推进物理仿真
          3. 发布 /joint_states
          4. 每 N 步渲染并发布相机图像
          5. 发布腕部力/力矩
        """
        # 1. 写入最新关节指令到执行器（带平滑，消除 50Hz→200Hz 阶跃）
        #    reach_node 每 20ms 更新一次 /joint_commands（50Hz），
        #    平滑策略：EMA，alpha=0.3 → τ ≈ 3步=15ms，滤除高频噪声。
        with self.lock:
            alpha_wrist = self.get_parameter('wrist_ema_alpha').get_parameter_value().double_value
            for joint_name, act_ids in self.actuator_map.items():
                if joint_name in self.joint_cmd:
                    target = self.joint_cmd[joint_name]
                    # 使用主执行器做 EMA 平滑
                    act_primary = self.actuator_map_primary[joint_name]
                    prev = self.ctrl_smoothed.get(act_primary, target)
                    # 手腕关节用独立更低的 alpha（抑制 yaw 颤抖）
                    if joint_name in self.wrist_joint_names:
                        alpha = alpha_wrist
                    else:
                        alpha = 1.0  # 肩/肘关节 EMA: 1.0=无平滑，直通
                    smoothed = prev + alpha * (target - prev)
                    # motor → ctrl=0（零力矩）；position servo → ctrl=smoothed（位置目标）
                    for act_id in act_ids:
                        if act_id in self.actuator_motor_ids:
                            self.data.ctrl[act_id] = 0.0
                        else:
                            self.ctrl_smoothed[act_id] = smoothed
                            self.data.ctrl[act_id] = float(smoothed)
                    # 如果手臂关节被硬夹持，同步更新夹持目标（EMA 平滑）
                    if joint_name in self.arm_clamp_index:
                        idx = self.arm_clamp_index[joint_name]
                        self.locked_joint_clamps[idx][1] = float(smoothed)
                elif joint_name in self.hand_cmd:
                    # 手指/手腕关节：来自 /hand_commands，独立 EMA 平滑
                    target = self.hand_cmd[joint_name]
                    act_primary = self.actuator_map_primary[joint_name]
                    prev = self.ctrl_smoothed.get(act_primary, target)
                    # 手腕关节用独立更低 alpha（抑制 yaw 颤抖）
                    if joint_name in self.wrist_joint_names:
                        alpha_hand = self.get_parameter('wrist_ema_alpha').get_parameter_value().double_value
                    else:
                        alpha_hand = self.get_parameter('finger_ema_alpha').get_parameter_value().double_value
                    smoothed = prev + alpha_hand * (target - prev)
                    for act_id in act_ids:
                        if act_id in self.actuator_motor_ids:
                            self.data.ctrl[act_id] = 0.0
                        else:
                            self.ctrl_smoothed[act_id] = smoothed
                            self.data.ctrl[act_id] = float(smoothed)
                elif joint_name in self.locked_joint_names:
                    # 锁定关节：motor ctrl 保持 0，position servo ctrl 保持初始值
                    # （已在 __init__ 中设好），不需要平滑（目标值不变）
                    pass

        # 1.5 根据参数决定是否硬夹持（默认关闭，完全依赖位置伺服 + 足地接触）
        if self.hard_clamp_pelvis:
            self.data.qpos[0:3] = self.init_pelvis_pos
            self.data.qpos[3:7] = self.init_pelvis_quat
            self.data.qvel[0:6] = 0.0
            self.data.qfrc_applied[0:6] = 0.0

        if self.hard_clamp_joints:
            for clamp_entry in self.locked_joint_clamps:
                qpos_addr = clamp_entry[0]
                target_val = clamp_entry[1]
                dof_addr = clamp_entry[2]
                self.data.qpos[qpos_addr] = target_val
                if dof_addr >= 0:
                    self.data.qvel[dof_addr] = 0.0

        # 2. 步进物理
        mujoco.mj_step(self.model, self.data)

        # 每 10 秒检查基座漂移（仅在硬夹持骨盆时检查，否则漂移是正常的）
        if self.step_counter % 2000 == 0:
            if self.hard_clamp_pelvis:
                drift = float(np.linalg.norm(self.data.qpos[0:3] - self.init_pelvis_pos))
                if drift > 5e-3:
                    self.get_logger().warn(
                        f"基座漂移: |drift|={drift:.4f} m "
                        f"(pelvis={self.data.qpos[0]:.4f},{self.data.qpos[1]:.4f},{self.data.qpos[2]:.4f})"
                    )
            # 锁定关节漂移诊断（仅在硬夹持时检查，无硬夹持时漂移是正常现象）
            if self.hard_clamp_joints:
                max_lock_drift = 0.0
                max_lock_name = ""
                for joint_name, qpos_addr, init_q in self.locked_joint_info:
                    if joint_name in self.active_joint_names:
                        continue
                    d = abs(float(self.data.qpos[qpos_addr]) - init_q)
                    if d > max_lock_drift:
                        max_lock_drift = d
                        max_lock_name = joint_name
                if max_lock_drift > 0.05:
                    self.get_logger().warn(
                        f"锁定关节最大漂移: {max_lock_name} = {max_lock_drift:.4f} rad "
                        f"({max_lock_drift*57.3:.1f}°) — 检查执行器 kp/kv 是否足够"
                    )

        # 3. 发布关节状态
        self._publish_joint_states()

        # 3.5 发布 MuJoCo 真实末端位置 + 手部朝向 + Jacobian (20Hz，与 PID 同频)
        if self.ee_pub is not None and self.step_counter % 10 == 0:
            self._publish_ee_pose()
            self._publish_hand_orientation()
            if self.jac_pub is not None:
                self._publish_jacobian()
            if self.wrist_jac_pub is not None:
                self._publish_wrist_jacobian()

        # 4. 按降频渲染相机
        self.step_counter += 1
        if (self.renderer is not None
                and self.rgb_pub is not None
                and self.step_counter % self.camera_step_interval == 0):
            self._publish_camera()

        # 5. 发布腕部力/力矩
        if self.ft_pub is not None:
            self._publish_wrist_ft()

        # 6. 更新交互式查看器
        if self.viewer_enabled and self.viewer is not None:
            self.viewer.sync()

    # =========================================================================
    # MODULE: Publishers
    # =========================================================================

    def _publish_ee_pose(self):
        """发布 /actual_ee_pose — MuJoCo 中右手心的真实世界坐标。"""
        pos = self.data.xpos[self.ee_body_id]  # (3,) array: x, y, z
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x = float(pos[0])
        msg.point.y = float(pos[1])
        msg.point.z = float(pos[2])
        self.ee_pub.publish(msg)

    def _publish_hand_orientation(self):
        """发布 /actual_hand_orientation — MuJoCo 中手掌的真实世界朝向 (quaternion)。"""
        if self.hand_ori_pub is None or self.hand_body_id < 0:
            return
        xmat = self.data.xmat[self.hand_body_id]  # 9-element rotation matrix (row-major)
        # Convert 3×3 rotation matrix to quaternion
        R = xmat.reshape(3, 3)
        # Use standard rotation-matrix-to-quaternion conversion
        tr = float(R[0, 0] + R[1, 1] + R[2, 2])
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * s
            qx = float(R[2, 1] - R[1, 2]) / s
            qy = float(R[0, 2] - R[2, 0]) / s
            qz = float(R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = float(R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = float(R[0, 1] + R[1, 0]) / s
            qz = float(R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = float(R[0, 2] - R[2, 0]) / s
            qx = float(R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = float(R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = float(R[1, 0] - R[0, 1]) / s
            qx = float(R[0, 2] + R[2, 0]) / s
            qy = float(R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        msg = QuaternionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.quaternion.x = float(qx)
        msg.quaternion.y = float(qy)
        msg.quaternion.z = float(qz)
        msg.quaternion.w = float(qw)
        self.hand_ori_pub.publish(msg)

    def _publish_wrist_jacobian(self):
        """发布 /mujoco_wrist_jacobian — 3×3 手腕朝向 Jacobian (orientation, not position).

        对 3 个腕关节分别 ±ε 扰动，观测 hand body 的朝向变化，
        使用中心差分计算 ∂ω/∂q_wrist (3×3 矩阵)。
        """
        if self.wrist_jac_pub is None or self.hand_body_id < 0:
            return
        eps = 0.001
        J = np.zeros((3, 3), dtype=np.float64)

        for i, qpos_addr in enumerate(self.wrist_joint_qpos_addrs):
            orig = float(self.data.qpos[qpos_addr])

            # +ε
            self.data.qpos[qpos_addr] = orig + eps
            mujoco.mj_forward(self.model, self.data)
            R_plus = self.data.xmat[self.hand_body_id].reshape(3, 3).copy()

            # −ε
            self.data.qpos[qpos_addr] = orig - eps
            mujoco.mj_forward(self.model, self.data)
            R_minus = self.data.xmat[self.hand_body_id].reshape(3, 3).copy()

            # Orientation difference: R_diff = R_plus * R_minus^T
            # For small rotations: R_diff ≈ I + [ω]×
            R_diff = R_plus @ R_minus.T
            omega = 0.5 * np.array([
                R_diff[2, 1] - R_diff[1, 2],
                R_diff[0, 2] - R_diff[2, 0],
                R_diff[1, 0] - R_diff[0, 1],
            ], dtype=np.float64)

            J[:, i] = omega / (2.0 * eps)

            # Restore
            self.data.qpos[qpos_addr] = orig

        # Restore full kinematic state
        mujoco.mj_forward(self.model, self.data)

        msg = Float64MultiArray()
        msg.data = J.flatten().tolist()  # row-major: [J00, J01, J02, J10, J11, J12, J20, J21, J22]
        self.wrist_jac_pub.publish(msg)

    def _publish_jacobian(self):
        """发布 /mujoco_jacobian — MuJoCo 3×7 中心差分数值 Jacobian。

        对 7 个主动关节分别 ±ε 扰动，用 mj_forward() 计算 EE 位移。
        扰动后恢复原始 qpos，不改变仿真状态。
        """
        eps = 0.001  # rad
        J = np.zeros((3, len(self.arm_joint_qpos_addrs)), dtype=np.float64)

        # 保存并扰动各关节
        for i, qpos_addr in enumerate(self.arm_joint_qpos_addrs):
            orig = float(self.data.qpos[qpos_addr])

            # +ε
            self.data.qpos[qpos_addr] = orig + eps
            mujoco.mj_forward(self.model, self.data)
            pos_plus = self.data.xpos[self.ee_body_id].copy()

            # −ε
            self.data.qpos[qpos_addr] = orig - eps
            mujoco.mj_forward(self.model, self.data)
            pos_minus = self.data.xpos[self.ee_body_id].copy()

            # 中心差分
            J[:, i] = (pos_plus - pos_minus) / (2.0 * eps)

            # 恢复
            self.data.qpos[qpos_addr] = orig

        # 恢复完整运动学状态
        mujoco.mj_forward(self.model, self.data)

        msg = Float64MultiArray()
        msg.data = J.flatten().tolist()  # row-major: [J00..J06, J10..J16, J20..J26]
        self.jac_pub.publish(msg)

    def _publish_joint_states(self):
        """发布 /joint_states（所有主动关节的位置 + 速度）。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

        # 从 MuJoCo 传感器读取（如果有的话），否则用 qpos/qvel 直接查
        for joint_name in self.actuator_map:
            msg.name.append(joint_name)
            act_id = self.actuator_map_primary[joint_name]

            # 优先从传感器读取
            if joint_name in self.sensor_pos_ids:
                sens_id = self.sensor_pos_ids[joint_name]
                msg.position.append(float(self.data.sensordata[sens_id]))
            else:
                # 从 qpos 获取（需要知道 joint 在 qpos 中的地址）
                act_joint_id = self.model.actuator_trnid[act_id][0]
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                msg.position.append(float(self.data.qpos[qpos_addr]))

            if joint_name in self.sensor_vel_ids:
                sens_id = self.sensor_vel_ids[joint_name]
                msg.velocity.append(float(self.data.sensordata[sens_id]))
            else:
                act_joint_id = self.model.actuator_trnid[act_id][0]
                dof_addr = self.model.jnt_dofadr[act_joint_id]
                msg.velocity.append(float(self.data.qvel[dof_addr]))

            msg.effort.append(float(self.data.actuator_force[act_id]))

        self.js_pub.publish(msg)

    def _publish_camera(self):
        """渲染虚拟相机并发布 RGB + Depth 图像。

        使用 MuJoCo 的离屏渲染器。Depth 图像需要 MuJoCo >= 3.1。
        """
        if self.renderer is None or self.camera_id < 0:
            return

        try:
            self.renderer.update_scene(self.data, camera=self.camera_id)

            # --- RGB 图像 ---
            rgb_pixels = self.renderer.render()
            # 发布 RGB
            if self.bridge is not None:
                rgb_msg = self.bridge.cv2_to_imgmsg(
                    cv.cvtColor(rgb_pixels, cv.COLOR_RGB2BGR), encoding='bgr8'
                )
            else:
                # 手动构建 Image 消息（无 cv_bridge 回退方案）
                rgb_msg = Image()
                rgb_msg.height = rgb_pixels.shape[0]
                rgb_msg.width = rgb_pixels.shape[1]
                rgb_msg.encoding = 'rgb8'
                rgb_msg.step = rgb_pixels.shape[1] * 3
                rgb_msg.data = rgb_pixels.tobytes()

            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            rgb_msg.header.frame_id = self.get_parameter('camera_name').get_parameter_value().string_value
            self.rgb_pub.publish(rgb_msg)

            # --- Depth 图像（16-bit 单通道，单位 mm）---
            try:
                self.renderer.enable_depth_rendering()
                depth_pixels = self.renderer.render()
                self.renderer.disable_depth_rendering()
                # depth rendering returns shape (H, W)
                if len(depth_pixels.shape) == 2:
                    depth_mm = (depth_pixels * 1000).astype(np.uint16)
                    if self.bridge is not None:
                        depth_msg = self.bridge.cv2_to_imgmsg(
                            depth_mm, encoding='mono16'
                        )
                    else:
                        depth_msg = Image()
                        depth_msg.height = depth_mm.shape[0]
                        depth_msg.width = depth_mm.shape[1]
                        depth_msg.encoding = 'mono16'
                        depth_msg.step = depth_mm.shape[1] * 2
                        depth_msg.data = depth_mm.tobytes()
                    depth_msg.header = rgb_msg.header
                    self.depth_pub.publish(depth_msg)
                else:
                    self.get_logger().debug(
                        "render() 返回非 depth 格式，"
                        "请升级到 MuJoCo >= 3.1 以获取真实深度图",
                        throttle_duration_sec=30
                    )
            except Exception:
                self.get_logger().debug(
                    "深度渲染失败（可能需要 MuJoCo >= 3.1）",
                    throttle_duration_sec=30
                )

        except Exception as e:
            self.get_logger().warn(
                f"相机渲染失败: {e}", throttle_duration_sec=5
            )

    def _publish_wrist_ft(self):
        """发布腕部力/力矩传感器数据。

        使用 WrenchStamped（ROS2 中 Wrench 不含 header）。
        通过名称搜索传感器：首先查找 'wrist_force' 和 'wrist_torque'。
        如果找不到，发布零值。
        """
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'right_wrist'

        # 在 sensordata 中搜索力/力矩传感器
        for sens_id in range(self.model.nsensordata):
            sens_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, sens_id
            )
            if sens_name is None:
                continue
            sens_addr = self.model.sensor_adr[sens_id]
            sens_dim = self.model.sensor_dim[sens_id]

            if sens_name == 'wrist_force' and sens_dim >= 3:
                msg.wrench.force.x = float(self.data.sensordata[sens_addr])
                msg.wrench.force.y = float(self.data.sensordata[sens_addr + 1])
                msg.wrench.force.z = float(self.data.sensordata[sens_addr + 2])
            elif sens_name == 'wrist_torque' and sens_dim >= 3:
                msg.wrench.torque.x = float(self.data.sensordata[sens_addr])
                msg.wrench.torque.y = float(self.data.sensordata[sens_addr + 1])
                msg.wrench.torque.z = float(self.data.sensordata[sens_addr + 2])

        self.ft_pub.publish(msg)

    # =========================================================================
    # MODULE: Cleanup
    # =========================================================================

    def destroy_node(self):
        """关闭查看器并清理资源。"""
        if self.viewer_enabled and self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


# ===========================================================================
# MODULE: Main — ROS2 entry point
# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    try:
        node = MuJoCoBridgeNode()
        rclpy.spin(node)
    except (ImportError, FileNotFoundError) as e:
        print(f"[FATAL] 初始化失败: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
