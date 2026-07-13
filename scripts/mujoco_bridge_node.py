#!/usr/bin/env python3
"""
mujoco_bridge_node.py — ROS2 ↔ MuJoCo 物理仿真桥接节点
========================================================

替代 joint_state_publisher 的纯运动学闭环，提供完整的物理仿真：
  - 重力、碰撞、接触力
  - 虚拟相机（RGB + 深度图）
  - 腕部力/力矩传感器

架构：
  reach_node                     mujoco_bridge_node
  ──────────                     ──────────────────
  /joint_commands (JointState) ───→ 写入 MuJoCo ctrl
                                    ↓ 步进物理仿真
  /joint_states   (JointState) ←─── 读取传感器数据
  /camera/rgb     (Image)      ←─── 渲染虚拟相机
  /camera/depth   (Image)      ←─── 渲染深度图
  /wrist_ft       (Wrench)     ←─── 腕部力/力矩（Phase 3）

依赖：
  pip install mujoco opencv-python
  # cv_bridge 用于 ROS2 Image 转换（可选，无 cv_bridge 时用 numpy 裸发）

用法：
  ros2 run g1_right_arm_reach mujoco_bridge_node \
      --ros-args -p scene_path:=/path/to/g1_grasp_scene.xml
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
from geometry_msgs.msg import WrenchStamped, PointStamped
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge


# ===========================================================================
# MODULE: MuJoCoBridgeNode — ROS2 node that steps MuJoCo and bridges topics
# ===========================================================================
class MuJoCoBridgeNode(Node):
    """MuJoCo 物理仿真桥接节点。

    职责：
      1. 加载 MJCF 场景文件，初始化 MuJoCo 模型和数据
      2. 订阅 /joint_commands，缓存最新关节指令
      3. 以 physics_rate 的频率步进 MuJoCo 仿真
      4. 发布 /joint_states（关节位置+速度）
      5. 以 camera_rate 的频率渲染并发布虚拟相机图像
      6. 发布 /wrist_ft（腕部力/力矩）

    Attributes:
        model: MuJoCo MjModel。
        data: MuJoCo MjData。
        renderer: MuJoCo Renderer（用于离屏渲染相机图像）。
        bridge: CvBridge（ROS2 ↔ OpenCV 图像转换）。
        joint_cmd: 最新的关节指令 dict {joint_name: position_rad}。
        lock: 线程锁，保护 joint_cmd 的读写。
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
        ])  # 主动控制关节（由 /joint_commands 更新）
        self.declare_parameter('clamp_arm_joints', False)  # 手臂关节硬夹持（默认关闭，靠位置伺服）
        self.declare_parameter('hard_clamp_pelvis', True)   # 硬夹持骨盆（固定机器人位置）
        self.declare_parameter('hard_clamp_joints', True)   # 硬夹持非右臂关节（除右臂外全部锁死）

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
        # 诊断：打印所有顶层 body 名称（排查 weld 约束 body 名）
        # ─────────────────────────────────────────────────────────────
        body_names = []
        for bid in range(min(self.model.nbody, 30)):
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
            if bname:
                parent_id = self.model.body_parentid[bid]
                parent_name = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, parent_id
                ) if parent_id >= 0 else "world"
                body_names.append(f"  [{bid}] '{bname}' (parent={parent_name})")
        self.get_logger().info(
            f"MJCF Body 树（前{min(self.model.nbody, 30)}个）:\n" +
            "\n".join(body_names)
        )

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
        # 诊断：打印前几个执行器的类型，验证分类正确
        sample_log = []
        for act_id in range(min(self.model.nu, 6)):
            act_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_id)
            gt = self.model.actuator_gaintype[act_id]
            bt = self.model.actuator_biastype[act_id]
            kp = self.model.actuator_gainprm[act_id][0]
            tag = "motor" if act_id in self.actuator_motor_ids else "POS"
            sample_log.append(
                f"  act[{act_id}]='{act_name}' gaintype={gt} biastype={bt} "
                f"gainprm0={kp:.0f} → {tag}"
            )
        self.get_logger().info("执行器分类诊断（前6个）:\n" + "\n".join(sample_log))
        # 额外诊断：打印右臂关节的执行器参数（gainprm / biasprm）
        arm_act_diag = []
        for joint_name in ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
                           'right_shoulder_yaw_joint', 'right_elbow_joint',
                           'right_wrist_roll_joint']:
            if joint_name in self.actuator_map:
                for act_id in self.actuator_map[joint_name]:
                    act_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_id)
                    gt = self.model.actuator_gaintype[act_id]
                    bt = self.model.actuator_biastype[act_id]
                    gp0 = self.model.actuator_gainprm[act_id][0]
                    bp1 = self.model.actuator_biasprm[act_id][1]  # position feedback coeff
                    bp2 = self.model.actuator_biasprm[act_id][2]  # velocity feedback coeff
                    tag = "motor" if act_id in self.actuator_motor_ids else "POS"
                    arm_act_diag.append(
                        f"  {joint_name.split('_joint')[0].replace('right_','')}"
                        f" act[{act_id}]='{act_name}' gt={gt} bt={bt} "
                        f"gp0={gp0:.0f} bp1={bp1:.0f} bp2={bp2:.0f} → {tag}"
                    )
        self.get_logger().info("右臂执行器参数详情:\n" + "\n".join(arm_act_diag))
        self.get_logger().info(
            f"执行器类型: {len(self.actuator_motor_ids)} 个 motor (力矩控制, ctrl→0), "
            f"{len(self.actuator_position_ids)} 个 position servo (位置伺服)"
        )

        # 区分主动关节（右臂 5 DOF）与锁定关节（其余 18 DOF）
        active_joints = self.get_parameter('active_joints').get_parameter_value().string_array_value
        self.active_joint_names = set(active_joints)
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

        # 锁定关节双重锁死：硬夹持（qpos 覆写）+ 位置伺服（步内维持）
        # 使用 list（非 tuple）以便运行时更新手臂关节的目标值
        self.locked_joint_clamps = []  # [[qpos_addr, target_val, dof_addr], ...]
        self.locked_joint_info = []    # [(joint_name, qpos_addr, init_q), ...] 诊断用
        self.arm_clamp_index = {}      # {joint_name: index_in_clamps} 快速查找

        # 只采集非手臂锁定关节（手臂关节就在绪姿态之后采集）
        for joint_name in self.locked_joint_names:
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
                           'right_wrist_roll_joint']:
            if joint_name in self.actuator_map:
                act_id = self.actuator_map_primary[joint_name]
                act_joint_id = self.model.actuator_trnid[act_id][0]
                qpos_addr = self.model.jnt_qposadr[act_joint_id]
                default_q = float(self.data.qpos[qpos_addr])
                short = joint_name.replace('right_', '').replace('_joint', '')
                default_arm_q.append(f"{short}={default_q:.3f}")
        self.get_logger().info(f"右臂 MJCF 默认 qpos: {', '.join(default_arm_q)}")

        arm_ready_pose = {
            'right_shoulder_pitch_joint': -0.2,   # 上臂微前倾（离开Z-min奇异点）
            'right_shoulder_roll_joint':   0.0,   # 不外展
            'right_shoulder_yaw_joint':    0.0,   # 旋转中立
            'right_elbow_joint':           0.3,   # 肘微屈（离开伸直极限，留双向运动空间）
            'right_wrist_roll_joint':      0.0,   # 腕中立
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
        self.joint_cmd = {}   # {joint_name: position_rad}
        self.lock = threading.Lock()

        self.cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self._cmd_callback, 10
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
        self.ee_body_id = -1
        for bid in range(self.model.nbody):
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
            if bname == 'right_wrist_roll_rubber_hand':
                self.ee_body_id = bid
                break
        if self.ee_body_id >= 0:
            self.ee_pub = self.create_publisher(PointStamped, '/actual_ee_pose', 10)
            self.get_logger().info(
                f"真实末端位置发布: body='right_wrist_roll_rubber_hand' (ID={self.ee_body_id})"
            )
        else:
            self.ee_pub = None
            self.get_logger().warn("未找到 right_wrist_roll_rubber_hand body，末端位置不发布")

        # --- MuJoCo 数值 Jacobian 发布（供 PID 使用，消除 URDF/MJCF Jacobian 差异）---
        arm_joint_order = [
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
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
        if self.ee_body_id >= 0 and len(self.arm_joint_qpos_addrs) == 5:
            self.jac_pub = self.create_publisher(Float64MultiArray, '/mujoco_jacobian', 10)
            self.get_logger().info(
                f"MuJoCo 数值 Jacobian 发布已启用 (5 关节, qpos_addrs={self.arm_joint_qpos_addrs})"
            )
        else:
            self.jac_pub = None
            self.get_logger().warn("Jacobian 发布禁用（EE body 或 arm joint qpos 地址缺失）")

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
        """缓存 /joint_commands 中的关节位置。

        线程安全：使用 lock 保护 joint_cmd 字典的写入。

        Args:
            msg: sensor_msgs/JointState，来自 reach_node 的关节指令。
        """
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.joint_cmd[name] = pos

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
        # 1. 写入最新关节指令到执行器（带平滑，消除 20Hz→200Hz 阶跃）
        #    reach_node 每 50ms 更新一次 /joint_commands，如果不加平滑，
        #    MuJoCo 位置伺服会在每个新指令到达时面临目标阶跃 →
        #    瞬态大扭矩 → 激起振荡。
        #    平滑策略：指数移动平均，时间常数 ≈ 25ms（半个 reach_node 周期），
        #    足够滤除阶跃同时保持响应速度。
        with self.lock:
            for joint_name, act_ids in self.actuator_map.items():
                if joint_name in self.joint_cmd:
                    target = self.joint_cmd[joint_name]
                    # 使用主执行器做 EMA 平滑
                    act_primary = self.actuator_map_primary[joint_name]
                    prev = self.ctrl_smoothed.get(act_primary, target)
                    alpha = 0.2  # 每步移动 20% 向目标
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

        # 每 2 秒输出精简状态摘要（400 步 / 200 Hz）
        if self.step_counter % 400 == 0 and self.ee_body_id >= 0:
            ee_pos = self.data.xpos[self.ee_body_id]
            # 快速关节状态摘要
            arm_summary = []
            for joint_name in self.active_joint_names:
                if joint_name in self.actuator_map_primary:
                    act_id = self.actuator_map_primary[joint_name]
                    act_joint_id = self.model.actuator_trnid[act_id][0]
                    qpos_addr = self.model.jnt_qposadr[act_joint_id]
                    short = joint_name.replace('right_', '').replace('_joint', '')
                    arm_summary.append(
                        f"{short}={float(self.data.qpos[qpos_addr]):+.2f}"
                    )
            self.get_logger().info(
                f"EE=[{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] | "
                f"arm: {' '.join(arm_summary)}"
            )

        # 每 30 秒输出右臂状态摘要（6000 步 / 200 Hz）
        if self.step_counter % 6000 == 0:
            arm_diag_parts = []
            for joint_name, act_ids in self.actuator_map.items():
                if joint_name in self.active_joint_names:
                    act_id_primary = self.actuator_map_primary[joint_name]
                    act_joint_id = self.model.actuator_trnid[act_id_primary][0]
                    qpos_addr = self.model.jnt_qposadr[act_joint_id]
                    dof_addr = self.model.jnt_dofadr[act_joint_id]
                    actual_q = float(self.data.qpos[qpos_addr])
                    cmd_q = float(self.data.ctrl[act_id_primary])
                    err = abs(actual_q - cmd_q)
                    qvel = float(self.data.qvel[dof_addr]) if dof_addr >= 0 else 0.0
                    motor_f = 0.0
                    pos_f = 0.0
                    for aid in act_ids:
                        af = float(self.data.actuator_force[aid])
                        if aid in self.actuator_motor_ids:
                            motor_f += af
                        else:
                            pos_f += af
                    total_f = motor_f + pos_f
                    short_name = joint_name.replace('right_', '').replace('_joint', '')
                    arm_diag_parts.append(
                        f"{short_name}: cmd={cmd_q:.3f} act={actual_q:.3f} "
                        f"err={err:.3f} vel={qvel:.2f} "
                        f"τ_m={motor_f:.0f} τ_p={pos_f:.0f} τ_tot={total_f:.0f}"
                    )
            self.get_logger().info(
                f"右臂诊断 [cmd/act/err(rad)/vel(rad/s)/τ_m/τ_p/τ_tot(N·m)]: "
                f"EE=({float(self.data.xpos[self.ee_body_id][0]):.3f}, "
                f"{float(self.data.xpos[self.ee_body_id][1]):.3f}, "
                f"{float(self.data.xpos[self.ee_body_id][2]):.3f})\n  " +
                "\n  ".join(arm_diag_parts)
            )

        # 3. 发布关节状态
        self._publish_joint_states()

        # 3.5 发布 MuJoCo 真实末端位置（20Hz，与 PID 同频）
        if self.ee_pub is not None and self.step_counter % 10 == 0:
            self._publish_ee_pose()
            if self.jac_pub is not None:
                self._publish_jacobian()

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

    def _publish_jacobian(self):
        """发布 /mujoco_jacobian — MuJoCo 3×5 中心差分数值 Jacobian。

        对 5 个主动关节分别 ±ε 扰动，用 mj_forward() 计算 EE 位移。
        扰动后恢复原始 qpos，不改变仿真状态。
        """
        eps = 0.001  # rad
        J = np.zeros((3, 5), dtype=np.float64)

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
        msg.data = J.flatten().tolist()  # row-major: [J00..J04, J10..J14, J20..J24]
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
