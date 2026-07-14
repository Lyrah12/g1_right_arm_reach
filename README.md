<p align="center">
  <h1 align="center">G1 右臂末端控制</h1>
  <p align="center">ROS 2 + Pinocchio + MuJoCo · 23-DOF Arm-Hand 解耦架构 · 50Hz 任务空间 IK</p>
</p>

---

## 架构：Arm-Hand 解耦

```
/target_pose (PoseStamped, 6-DOF, 肩部坐标系)
       │
       ▼
┌──────────────────────────────────────────┐
│            reach_node  (50 Hz)            │
│                                          │
│  computeArm()      computeWrist()        │
│  4-DOF 臂 IK       可变 DOF 腕朝向求解    │
│  (肩×3 + 肘)       (23-DOF: 1 关节)      │
│  3×4 阻尼伪逆       3×N 阻尼伪逆          │
│  + 零空间姿态吸引   (可扩展至 3-DOF)      │
│       │                   │              │
│       └───────┬───────────┘              │
│               ▼                          │
│      /joint_commands (5-DOF)             │
└──────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────┐
│      mujoco_bridge_node  (200 Hz)        │
│      物理仿真 · Position Servo · Jacobian │
└──────────────────────────────────────────┘
```

**关节分组（23-DOF 硬约束）：**

| 关节名 | 归属 | 用途 |
|--------|------|------|
| right_shoulder_pitch_joint | 臂 IK (4-DOF) | 位置控制 |
| right_shoulder_roll_joint | 臂 IK (4-DOF) | 位置控制 |
| right_shoulder_yaw_joint | 臂 IK (4-DOF) | 位置控制 |
| right_elbow_joint | 臂 IK (4-DOF) | 位置控制 |
| right_wrist_roll_joint | 腕求解 (1-DOF) | 掌心朝向 |
| right_wrist_pitch_joint | 灵巧手范畴 | 下一阶段 |
| right_wrist_yaw_joint | 灵巧手范畴 | 下一阶段 |

## 坐标系

**肩部坐标系**（目标输入 `frame_id: right_shoulder`）：

| 轴 | 方向 |
|----|------|
| +X | 机器人前方 |
| +Y | 机器人左侧 |
| +Z | 垂直向上 |

- 原点 = 右肩关节（`right_shoulder_pitch_joint`）
- 骨盆偏移 `pelvis_offset: [0, 0, 0.823]` 用于 URDF→MuJoCo 坐标对齐
- 臂最大伸展距离 ≈ 23 cm，目标必须在范围内

## 快速开始

### 依赖

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-desktop ros-jazzy-pinocchio ros-jazzy-robot-state-publisher

# Python
pip install --break-system-packages mujoco opencv-python "numpy<2"

# G1 模型
mkdir -p ~/deps && cd ~/deps
git clone https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```

### 编译

```bash
source /opt/ros/jazzy/setup.bash
cd ~/g1_ws
colcon build --packages-select g1_right_arm_reach --symlink-install
source install/setup.bash
```

### 启动

```bash
# MuJoCo 仿真（带查看器）
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
    urdf_path:=${HOME}/deps/unitree_ros/robots/g1_description/g1_23dof_rev_1_0.urdf \
    scene_path:=${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_grasp_scene.xml \
    viewer:=true

# 带自动测试目标
# 添加 use_test_target:=true

# RViz 运动学模式（无物理）
ros2 launch g1_right_arm_reach rviz_demo.launch.py \
    urdf_path:=${HOME}/deps/unitree_ros/robots/g1_description/g1_23dof_rev_1_0.urdf
```

### 发送目标（肩部坐标系，距离 ≤ 23cm）

```bash
source /opt/ros/jazzy/setup.bash

# 3-DOF 位置（默认朝向）
ros2 topic pub -1 /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'right_shoulder'}, \
    pose: {position: {x: 0.18, y: -0.05, z: -0.05}, \
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# 带掌心朝向（前臂水平指向前方时 wrist_roll 可翻掌心）
ros2 topic pub -1 /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'right_shoulder'}, \
    pose: {position: {x: 0.20, y: -0.05, z: 0.0}, \
            orientation: {x: -0.7071, y: 0.0, z: 0.0, w: 0.7071}}}"
```

**可达目标参考：**

| 目标 (x, y, z) | 距离 | 说明 |
|----------------|------|------|
| `[0.18, -0.05, -0.05]` | 19.4 cm | 前方偏下 |
| `[0.20, -0.05, 0.00]` | 20.6 cm | 前方与肩同高（前臂水平） |
| `[0.15, -0.05, -0.15]` | 21.8 cm | 前下方 |
| `[0.10, 0.00, -0.20]` | 22.4 cm | 近身体正下方 |

### 监控 & 调参

```bash
ros2 topic echo /debug/position_error              # 位置误差
ros2 param set /reach_node pid.kp 2.5               # 臂 P 增益
ros2 param set /reach_node pid.ki 0.8               # 臂 I 增益
ros2 param set /reach_node pid.kp_wrist 0.5         # 腕 P 增益
ros2 param set /reach_node pid.posture_gain 0.3     # 姿态吸引强度
```

## 参数

完整参数见 [`config/params.yaml`](config/params.yaml)。

**臂 IK（4-DOF → 3-DOF 位置）：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pid.kp` | 2.0 | P 增益 |
| `pid.ki` | 0.4 | I 增益（anti-windup 上限 0.10） |
| `pid.kd` | 0.05 | D 增益 |
| `pid.damping` | 2e-4 | 阻尼伪逆 λ（自适应下限） |
| `pid.tolerance` | 0.01 | 收敛容差 (m) |
| `pid.posture_gain` | 0.2 | 零空间姿态吸引 |
| `pid.posture_target` | `[-0.3, 0, 0, 0.3, 0]` | 首选关节姿态 |

**腕朝向求解（可变 DOF）：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pid.kp_wrist` | 1.0 | 腕朝向 P 增益 |
| `pid.kd_wrist` | 0.02 | 腕朝向 D 增益 |

**模型 & 节点：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `reach_node.pelvis_offset` | `[0, 0, 0.823]` | 骨盆世界位置 |
| `reach_node.control_rate` | 50.0 | 控制频率 (Hz) |
| `robot_model.arm_joints` | 4 个 | 臂关节名列表 |
| `robot_model.wrist_joints` | `[wrist_roll, wrist_pitch, wrist_yaw]` | 腕关节（仅 wrist_roll 在 23-DOF 中活跃） |

## 项目阶段

| 阶段 | 状态 | 内容 |
|------|:----:|------|
| 运动学仿真 | ✅ | RViz + Pinocchio FK/IK |
| 物理仿真 | ✅ | MuJoCo 重力/接触/Jacobian |
| 末端位置控制 | ✅ | Arm-Hand 解耦 4+1 DOF IK |
| **灵巧手集成** | 🔜 | 手指运动学 + 抓取动作 + 力控 |
| 视觉对接 | 📋 | 检测→目标→抓取闭环 |
| 真机部署 | 🔮 | G1 实机 |

---

## 下一阶段：灵巧手集成

### 当前接口预留

代码已为灵巧手阶段预留以下扩展点：

**1. 腕关节扩展（wrist_joints）**

`params.yaml` 中 `wrist_joints` 已列出 3 个关节（roll/pitch/yaw），`computeWrist()` 的 `3×N` 伪逆求解器自动适配 1~3 DOF。切换到 29-DOF 模型后无需修改 C++ 代码。

**2. EE 帧与手帧分离**

| 帧 | 用途 | 当前值 |
|----|------|--------|
| `ee_frame_id_` | 臂 IK 控制点（腕原点） | `right_wrist_roll_link` |
| `hand_frame_id_` | 朝向观测（手掌） | `right_wrist_roll_rubber_hand` |

灵巧手 mesh 挂载到 `hand_frame_id_` 后，臂 IK 不受影响。

**3. 自定义消息（已定义，待启用）**

| 消息 | 用途 |
|------|------|
| `DetectedObject.msg` | 视觉检测结果 → 抓取目标 |
| `DetectedObjectArray.msg` | 多目标列表 |
| `ArmStatus.msg` | 手臂状态反馈（位置/朝向/收敛） |
| `GraspResult.msg` | 抓取结果（成功/失败/力数据） |
| `GraspObject.action` | 抓取动作接口 |

**4. 控制管线扩展点**

```
当前:  /target_pose → Arm IK + Wrist → /joint_commands (5-DOF)
未来:  /target_pose → Arm IK + Wrist → /joint_commands (5-DOF)
       /grasp_command → Hand Controller → /hand_commands (N-DOF)
```

灵巧手控制器作为独立模块，与手臂控制并行运行。手臂负责把腕部送到抓取位置，灵巧手负责手指运动规划。

### 接入新灵巧手模型的步骤

1. 将灵巧手 MJCF include 到 `g1_grasp_scene.xml`
2. 若为 29-DOF 模型，`wrist_joints` 中 pitch/yaw 自动激活
3. 添加手指关节到 bridge 的主动关节列表
4. 实现手指控制器节点（独立于 reach_node）
5. 通过 `GraspObject.action` 协调手臂→手掌→抓取动作序列

## 开发

```bash
# 编译
colcon build --packages-select g1_right_arm_reach --symlink-install

# 测试
export G1_TEST_URDF_PATH=${HOME}/deps/unitree_ros/robots/g1_description/g1_23dof_rev_1_0.urdf
colcon test --packages-select g1_right_arm_reach
```

## 许可

BSD-3-Clause
