# Unitree G1 右臂 + Inspire RH56 灵巧手 控制

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros" alt="ROS 2">
  <img src="https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B" alt="C++17">
  <img src="https://img.shields.io/badge/Python-3.10+-3776AB?logo=python" alt="Python">
  <img src="https://img.shields.io/badge/MuJoCo-3.1+-00A98F" alt="MuJoCo">
  <img src="https://img.shields.io/badge/License-BSD--3--Clause-blue.svg" alt="License">
  <img src="https://img.shields.io/badge/DOF-13-important" alt="13-DOF">
</p>

---

**`g1_right_arm_reach`** 是一个 ROS2 软件包，用于控制 Unitree G1 人形机器人的右臂和因时 Inspire RH56 灵巧手（MuJoCo 仿真运行 + 可移植到实物）。实现了**力矩级**、完全解耦的**臂手架构**：手臂通过阻尼伪逆 IK 负责 4 自由度腕部定位，手部模块独立管理 3 自由度腕部朝向（5010 手腕）和 6 自由度手指抓取（RH56 欠驱动：6 主动 + 6 equality 耦合从动）。**PID 输出力矩 τ**（MIT 模式），与实物 G1 底层 `LowCmd` 接口一致。控制节点全部基于 **Pinocchio**（FK/Jacobian/重力本地计算），不依赖 MuJoCo 专属 topic——**实物只需替换仿真桥为 Unitree SDK 硬件桥**。

## 架构

### 臂手完全解耦

```
                         /action_command (ActionCommand)
                                 │
                                ▼
┌───────────────────────────────────────────────────────────────────┐
│                   action_sequencer_node  (50 Hz)                  │
│           状态机: IDLE → REACHING → SETTLING → GRASPING           │
│                      │                         │                  │
│                 /target_point              /hand_command          │
│                      │                         │                  │
│   ┌──────────────────┼──────┐   ┌──────────────┼──────────────┐   │
│   │   reach_node (50 Hz)    │   │   hand_controller (50 Hz)   │   │
│   │                         │   │                             │   │
│   │  4-DOF 手臂 IK          │   │  腕部朝向 PID                │   │
│   │  (肩部×3 + 肘部)         │   │  (3×3 Jacobian: Pinocchio)   │   │
│   │  → 腕部位置              │   │  手指阻抗控制 (6-DOF)        │   │
│   │                         │   │  GraspLibrary (15 种模式)    │   │
│   │     /joint_commands     │   │     /hand_commands          │   │
│   │     (4-DOF 手臂)        │   │     (9-DOF hand: 腕部+手指)      │   │
│   └────────────┬────────────┘   └──────────────┬──────────────┘   │
└───────────────┼────────────────────────────────┼──────────────────┘
                └────────────────┬───────────────┘
                                 ▼
                   ┌───────────────────────────────┐
                   │  mujoco_bridge_node.py        │
                   │     (200 Hz, Python)          │
                   │                               │
                   │  MuJoCo 物理步进               │
                   │  Torque mode (motor ctrl=τ)    │
                   │  /joint_states + EE/朝向/Jacobian (sim)│
                   └───────────────────────────────┘
                        (实物: Unitree SDK 硬件桥替代)
```

### 解耦边界

```
     手臂模块 (reach_node)            │    手部模块 (hand_controller)
                                     │
  shoulder_pitch ─┐                  │
  shoulder_roll  ─┤ 4-DOF IK         │
  shoulder_yaw   ─┤ → 腕部位置        │
  elbow          ─┘                  │
                                     │
  ─ ─ ─ ─ ─ ─  解耦边界  ─ ─ ─ ─ ─ ─ ─│
                                     │
                        wrist_roll  ─┴─ PID + Jacobian⁻¹
                        wrist_pitch ─┴─ → 朝向控制
                        wrist_yaw   ─┘
                                     │
                         thumb_yaw  ─┴─ 阻抗控制 (6 主动)
                         thumb_pitch─┴─ 直接扭矩输出
                         index/middle/ring/pinky proximal ─┘
                                     │
                         (6 从动 equality 耦合: 1.6×/2.4×/1.0×)

  /joint_commands (4-DOF)            │  /hand_commands (9-DOF hand)
```

**核心思想：** 手臂模块只管腕部在三维空间中的*位置*，手部模块接管腕部以下的所有内容——朝向、手指姿态和抓取执行。这使得系统**不依赖特定手部模型**。

### 灵巧手规格

| 属性 | 值 |
|-----------|-------|
| 型号 | 因时 Inspire RH56（5 指欠驱动） |
| 主动手指自由度 | **6-DOF**（拇指 yaw/pitch + 四指 proximal） |
| 从动耦合自由度 | **6-DOF**（equality 刚性耦合：拇指 1.6×/2.4×，四指 1:1） |
| 腕部自由度 | roll + pitch + yaw = **3-DOF**（5010，pitch/yaw ±13.4 N·m） |
| 手部控制 | **9-DOF**（腕 3 + 主动手指 6），力矩输出 |
| 手臂自由度 | 肩部×3 + 肘部 = **4-DOF**（力矩） |

## 特性

- **臂手解耦架构** —— 4-DOF 手臂 IK + 9-DOF hand 手部控制，模块完全独立
- **阻尼伪逆 IK** —— 4-DOF → 3-DOF，自适应阻尼 + 零空间姿态吸引
- **腕部朝向 PID** —— 3×3 数值 Jacobian（Pinocchio 优先，仿真 MuJoCo 覆盖）
- **手指阻抗控制** —— 直接扭矩输出（τ = Kp·e + Kd·ė），抓取指令直接跳变
- **15 种预定义抓取模式** —— 精细抓取、力量抓取、手势姿态
- **动作编排状态机** —— IDLE → REACHING → SETTLING → GRASPING，带收敛检测
- **统一命令行工具** —— 交互模式支持 RPY 朝向、单次命令和 Topic 级访问
- **200 Hz MuJoCo 物理仿真** —— Torque mode（motor ctrl=τ，MIT 模式，与实物 LowCmd 一致）
- **实物可移植** —— 控制节点 Pinocchio-only，不依赖 MuJoCo topic；只需写 Unitree SDK 硬件桥
- **运行时参数调节** —— 所有增益可通过 `ros2 param set` 动态调整，无需重启

## 快速开始

### 环境要求

| 依赖 | 版本 | 用途 |
|-----------|---------|---------|
| ROS 2 | Jazzy | 通信中间件 |
| Pinocchio | 最新 | 刚体动力学（FK、Jacobian） |
| MuJoCo | ≥ 3.1 | 物理仿真（Python 绑定） |
| Eigen3 | ≥ 3.4 | 线性代数 |
| Python | ≥ 3.10 | MuJoCo 桥接 |

### 1. 安装依赖

```bash
# ROS 2 + Pinocchio
sudo apt install ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-pinocchio \
  ros-${ROS_DISTRO}-robot-state-publisher

# MuJoCo + OpenCV
pip install --break-system-packages mujoco opencv-python "numpy<2"

# Eigen3（通常已预装；如未装则执行）
sudo apt install libeigen3-dev
```

### 2. 克隆外部模型

```bash
mkdir -p ~/deps && cd ~/deps

# G1 机器人模型 (URDF + MJCF)。setup_inspire_hand.sh 会按此顺序查找：
#   1) unitree_mujoco/unitree_robots/g1/   （官方 MuJoCo 模型）
#   2) unitree_ros_models/g1_description/  （ROS 模型包）
# 二选一即可；推荐 unitree_mujoco（官方，路径最简单）。
git clone https://github.com/unitreerobotics/unitree_mujoco.git

# 或（备选）
# git clone https://github.com/unitreerobotics/unitree_ros_models.git

# Inspire RH56 手模型已随包提交（config/inspire_hand/，无需克隆）
```

### 3. 配置 Inspire RH56 场景

```bash
# 生成补丁 URDF + 注入 Inspire 手到 MJCF，并解析 scene 的 @G1_INSPIRE_MJCF@ 占位符。
# ⚠️ 每次部署到新机器都必须先跑它（重新生成 config/g1_29dof_inspire.urdf + g1_inspire_scene.xml）。
bash ~/g1_ws/src/g1_right_arm_reach/scripts/setup_inspire_hand.sh
```

### 4. 编译

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/g1_ws
colcon build --packages-select g1_right_arm_reach --symlink-install
source install/setup.bash
```

### 5. 启动

```bash
# 终端 1：启动 MuJoCo 仿真及可视化窗口
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf \
  scene_path:=${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=true
```

> ⚠️ **`urdf_path` 必须是补丁 URDF `config/g1_29dof_inspire.urdf`**（由 `setup_inspire_hand.sh` 生成），
> 它把 5010 腕部 + Inspire 手质量并入 URDF，使 Pinocchio 重力补偿与 MuJoCo 一致
> （原始 `g1_29dof_rev_1_0.urdf` 无手，重力会差 ~0.5 N·m）。`scene_path` 中的
> `@G1_INSPIRE_MJCF@` 占位符也由 setup 脚本解析为当前机器的实际路径。
>
> 💡 `viewer:=true` 打开 MuJoCo 交互窗口（需 WSLg / X Server 图形环境）；无法显示窗口时
> 用 `viewer:=false` 纯 headless 运行，另开 `use_rviz:=true` 用 RViz 看，或从 Windows
> 侧启动 `rviz2`。

### 6. 发送第一条指令

```bash
# 终端 2：张手 → 伸手并捏取
source ~/g1_ws/install/setup.bash

python3 ~/g1_ws/install/g1_right_arm_reach/lib/g1_right_arm_reach/action_cli.py open
python3 ~/g1_ws/install/g1_right_arm_reach/lib/g1_right_arm_reach/action_cli.py \
  reach_and_grasp 0.20 -0.15 -0.15 close
```

> 目标坐标是**肩部相对坐标系**（+X 前、+Y 左、+Z 上）。`[0.20, -0.15, -0.15]`
> 是已验证可达的目标（收敛 5mm）。避免使用超过单臂臂展（~45cm）的坐标。

## 使用方式

### Python 交互模式

```bash
python3 ~/g1_ws/install/g1_right_arm_reach/lib/g1_right_arm_reach/action_cli.py interactive
```

交互模式命令示例：

```
>>> list                                # 列出所有命令和抓取模式
>>> 0.20 -0.15 -0.15 open               # 伸手 + 张手
>>> 0.20 -0.15 -0.15 close              # 伸手 + 握拳
>>> 0.20 -0.15 -0.15 pinch              # 伸手 + 精准捏取
>>> 0.20 -0.15 -0.15 rpy 0 0 0.5 close  # 伸手 + yaw 旋转 90° + 握拳
>>> grasp power_grasp                   # 仅动手（不移动手臂）
>>> home                                # 回到就绪姿态
```

### Python 单次命令

```bash
# 列出所有可用命令和抓取模式
python3 action_cli.py list

# 仅动手臂（手部不动）
python3 action_cli.py reach 0.20 -0.15 -0.15

# 仅动手（手臂不动）
python3 action_cli.py grasp power_grasp

# 手臂 + 抓取（完整动作流水线）
python3 action_cli.py reach_and_grasp 0.20 -0.15 -0.15 pinch

# 快捷命令
python3 action_cli.py home          # 回到就绪姿态
python3 action_cli.py open          # 张手
python3 action_cli.py close         # 握拳
```

### 直接 Topic 发布

```bash
ros2 topic pub -1 /action_command g1_right_arm_reach/msg/ActionCommand \
  "{action_type: reach_and_grasp, target_x: 0.2, target_y: -0.15, \
    target_z: -0.15, ori_w: 1.0, grasp_type: pinch, grasp_duration: 0.08}"
```

### 监控与调参

```bash
# 观察位置/朝向收敛情况
ros2 topic echo /debug/position_error          # 手臂位置误差 (m)
ros2 topic echo /debug/orientation_error       # 腕部朝向误差 (rad)

# 运行时参数调节
ros2 param set /reach_node pid.kp 200.0
ros2 param set /hand_controller kp_wrist 3.5
ros2 param set /hand_controller kp_finger 5.0
```

## 坐标系

**肩部相对坐标系** (`frame_id: right_shoulder`)：

| 轴 | 方向 |
|------|-----------|
| +X | 前方 |
| +Y | 左方 |
| +Z | 上方 |

- **原点**：`right_shoulder_pitch_joint`
- **单臂臂展**：约 45 cm
- **骨盆偏移**：`[0, 0, 0.793]`（URDF 骨盆原点 → MuJoCo 世界坐标对齐；基座固定后骨盆位于 body_pos z=0.793）


| 目标 `(x, y, z)` | 距离 | 说明 |
|---------------------|----------|-------|
| `[0.20, −0.15, −0.15]` | ~28 cm | 默认推荐目标（已验证收敛 5mm） |
| `[0.26, −0.22, −0.12]` | ~37 cm | 桌面物体（前偏左，接近臂展边缘） |
| `[0.15, −0.10, −0.05]` | ~18 cm | 前偏左，略向下 |

> 就绪姿态（上臂下垂 + 前臂水平 + 手半握）是全部关节零位，EE 约在 `[0.20, −0.05, −0.20]`。
> 表格中的目标是**伸手到达点**（肩相对），不是就绪姿态本身。

## 抓取模式库

全部 15 种模式定义在 `src/grasp_library.cpp` 中。添加新模式只需编辑此文件，无需修改其他代码。

### 基础 (3)

| # | 模式 | 说明 |
|---|---------|-------------|
| 1 | `open` | 五指完全伸展 |
| 2 | `close` | 全握拳——拇指包裹食指 |
| 3 | `home` | 自然半握（默认休息姿态） |

### 精细抓取 (4)

| # | 模式 | 说明 |
|---|---------|-------------|
| 4 | `pinch` | 拇指 + 食指尖对尖 |
| 5 | `tripod` | 拇指 + 食指 + 中指 |
| 6 | `three_finger` | 拇指 + 食指 + 中指（精细变体） |
| 7 | `lateral_pinch` | 拇指抵住食指侧面（钥匙捏法） |

### 力量抓取 (4)

| # | 模式 | 说明 |
|---|---------|-------------|
| 8 | `power_grasp` | 全包裹，拇指扣在手指上方 |
| 9 | `cylindrical_grasp` | 均匀弯曲，适用于水瓶/杯子 |
| 10 | `spherical_grasp` | 张开 + 弯曲，适用于球形物体 |
| 11 | `hook_grasp` | PIP/DIP 弯曲，MCP 伸展（提手柄） |

### 手势姿态 (4)

| # | 模式 | 说明 |
|---|---------|-------------|
| 12 | `point` | 食指伸出，其余握拳 |
| 13 | `thumbs_up` | 拇指向上伸出 |
| 14 | `peace_sign` | 食指 + 中指伸出（V 字手势） |
| 15 | `trigger` | 食指伸出，DIP 微屈（扣扳机） |

## 控制律

### 手臂：阻尼伪逆 IK (4-DOF → 3-DOF)

$$\Delta q = \alpha \cdot J^{+} \cdot e_{pos} + (I - J^{+}J) \cdot k_{posture} \cdot (q_{pref} - q)$$

其中 $J^{+} = J^{T}(JJ^{T} + \lambda^{2}I)^{-1}$，$\lambda$ 随任务速度自适应调节。

### 腕部：朝向 PID → 扭矩输出

$$e_{ori} = 2 \cdot \text{axis}(q_{des} \cdot q_{curr}^{-1}) \quad \text{(轴角误差)}$$

$$\tau_{wrist} = J^T(k_p \cdot e_{ori} + k_i \cdot \int e_{ori}\\dt + k_d \cdot \dot{e}_{ori}) + g_{wrist} - k_d^{joint} \cdot \dot{q}_{wrist}$$

其中 $J$ 为 3×3 腕部朝向 Jacobian —— **Pinocchio 数值雅可比（实物优先）；仿真中 MuJoCo
topic 存在时覆盖**。$g_{wrist}$ 为腕部重力前馈（含手质量，Pinocchio RNEA），$k_d^{joint}\dot{q}$
为关节阻尼，防止 ~1 Hz 欠阻尼振荡。

### 手指：阻抗控制（直接扭矩输出）

$$q_{des} = q_{target}\quad\text{(收到抓取指令时直接跳变)}$$

$$\tau_{finger} = k_p^{finger} \cdot (q_{des} - q_{act}) + k_d^{finger} \cdot (0 - \dot{q}_{act})$$

输出 $\tau_{finger}$（限幅 ±`tau_max_finger`）直接写入 `effort` 字段，桥接写入电机 `ctrl=τ`
（MIT 模式，与实物 G1 LowCmd 一致）。

## 配置参数

全部参数见 [`config/params.yaml`](config/params.yaml)。关键参数如下：

### 手臂 IK (`reach_node`)

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `pid.kp` | 200.0 | 任务空间刚度 (N/m) — 积分带限修复后 P 主导 |
| `pid.ki` | 1.0 | IK 积分增益（仅微调，带限 0.15，误差增长即清零） |
| `pid.kd` | 0.3 | IK 微分增益 |
| `pid.damping` | 2e−4 | 阻尼伪逆 λ（自适应） |
| `pid.tolerance` | 0.005 | 位置收敛容差 (m) |
| `pid.posture_gain` | 0.2 | 零空间姿态吸引强度 |
| `pid.torque_mode` | true | 力矩输出（MIT 模式，实物 LowCmd 接口一致） |
| `reach_node.pelvis_offset` | [0,0,0.793] | URDF 骨盆原点 → MuJoCo 世界对齐 |

### 手部控制器

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `kp_wrist` | 3.5 | 腕部朝向 P 增益 |
| `ki_wrist` | 0.05 | 腕部朝向 I 增益 |
| `kd_wrist` | 0.8 | 腕部朝向 D 增益 |
| `wrist_joint_damping` | 0.6 | 关节空间速度阻尼 (N·m·s/rad) |
| `wrist_offset_limit` | 2.0 | 腕部偏移上限（相对就绪姿态，rad） |
| `kp_finger` | 5.0 | 手指阻抗刚度 (N·m/rad) |
| `kd_finger` | 0.3 | 手指阻抗阻尼 (N·m·s/rad) |
| `tau_max_finger` | 1.5 | 手指电机力矩上限 (N·m) |
| `tau_max_wrist` | 13.4 | 5010 腕部力矩上限 (N·m) |

### MuJoCo 桥接

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `physics_rate` | 200.0 | 物理步进频率 (Hz) |
| `wrist_ema_alpha` | 1.0 | 腕部 EMA 平滑 |
| `finger_ema_alpha` | 1.0 | 手指 EMA 平滑 |

## Topic 总览

| Topic | 类型 | 方向 | 说明 |
|-------|------|-----------|-------|
| `/action_command` | `ActionCommand` | CLI → sequencer | 高层动作指令（位置 + 朝向 + 抓取） |
| `/target_point` | `Point` | sequencer → reach_node | 3-DOF 肩部相对位置 |
| `/hand_command` | `HandCommand` | sequencer → hand_controller | 朝向四元数 + 抓取类型 |
| `/joint_commands` | `JointState` | reach_node → bridge | 4-DOF 手臂 |
| `/hand_commands` | `JointState` | hand_controller → bridge | 9-DOF hand（腕部×3 + 手指×6） |
| `/joint_states` | `JointState` | bridge → 各节点 | 全 13-DOF 反馈（臂4 + 腕3 + 手指6） |
| `/debug/position_error` | `Float64` | reach_node → sequencer | 位置收敛误差 (m) |
| `/debug/orientation_error` | `Float64` | hand_controller → sequencer | 朝向收敛误差 (rad) |

**仿真专用 topic**（MuJoCo bridge 发布；实物上不存在，控制节点用 Pinocchio FK 计算替代）：

| Topic | 类型 | 方向 | 说明 | 实物替代 |
|-------|------|-----------|-------|----------|
| `/actual_ee_pose` | `PointStamped` | bridge → reach_node | 腕部世界位置 | Pinocchio FK + pelvis_offset |
| `/actual_hand_orientation` | `QuaternionStamped` | bridge → hand_controller | 手掌朝向 | Pinocchio FK |
| `/mujoco_wrist_jacobian` | `Float64MultiArray` | bridge → hand_controller | 3×3 腕部 Jacobian | Pinocchio 数值雅可比 |
| `/mujoco_gravity` | `Float64MultiArray` | bridge → 各节点 | 重力补偿（含手） | Pinocchio RNEA |
| `/camera/rgb`、`/camera/depth` | `Image` | bridge → 无订阅者 | 虚拟相机 | 实物真相机 |
| `/wrist_ft` | `WrenchStamped` | bridge → 无订阅者 | 腕部力/力矩 | 实物 F/T 传感器 |

## 项目结构

```
g1_right_arm_reach/
├── include/g1_right_arm_reach/
│   ├── robot_model.hpp           # Pinocchio 封装（URDF → FK/Jacobian）
│   ├── kinematics.hpp            # 子集数值 Jacobian
│   ├── pid_controller.hpp        # 阻尼伪逆 IK 求解器
│   ├── grasp_library.hpp         # 15 种抓取模式注册表
│   ├── hand_controller.hpp       # 腕部 PID + 手指阻抗
│   └── action_sequencer.hpp      # 动作状态机
├── src/
│   ├── robot_model.cpp
│   ├── kinematics.cpp
│   ├── pid_controller.cpp
│   ├── reach_node.cpp            # 手臂 IK 节点 (50 Hz, 4-DOF → 3-DOF)
│   ├── grasp_library.cpp         # 抓取模式定义
│   ├── hand_controller_node.cpp  # 手部控制节点 (50 Hz, 9-DOF hand)
│   ├── action_sequencer_node.cpp # 动作编排节点 (50 Hz)
│   └── target_publisher_node.cpp # 测试工具
├── scripts/
│   ├── mujoco_bridge_node.py     # MuJoCo 物理桥接 (200 Hz)
│   ├── inject_inspire_hand.py    # Inspire RH56 MJCF 注入器
│   ├── inspire_urdf_to_mjcf.py   # DFQ URDF → MJCF 片段转换
│   ├── patch_inspire_urdf.py     # 补丁 URDF（Pinocchio 重力对齐）
│   ├── setup_inspire_hand.sh     # Inspire RH56 一键配置
│   ├── action_cli.py             # 统一命令行工具
│   └── plot_error.py             # 误差绘图工具
├── msg/
│   ├── ActionCommand.msg         # 统一动作指令
│   └── HandCommand.msg           # 手部朝向 + 抓取
├── config/
│   ├── params.yaml               # 全部运行时参数
│   ├── g1_29dof_inspire.urdf     # 补丁 URDF（5010 腕 + Inspire 手并入，Pinocchio 重力对齐）
│   ├── g1_inspire_scene.xml      # MJCF: G1 + Inspire RH56（scene_path）
│   ├── inspire_hand_right_body.xml   # 手 body 树片段（setup 生成）
│   ├── inspire_hand_right_assets.xml # 手 mesh 资产（setup 生成）
│   └── inspire_hand/             # DFQ_right_hand.urdf + 13 个 STL mesh
├── launch/
│   ├── mujoco_demo.launch.py     # 启动全部节点
│   ├── mujoco_armonly.launch.py  # 仅 bridge + reach_node（隔离调试）
│   └── rviz_demo.launch.py       # 无物理 RViz 演示
├── test/
│   ├── test_kinematics.cpp       # FK/Jacobian 单元测试
│   └── test_controllers.cpp      # PID 收敛测试
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 路线图

| 阶段 | 状态 | 内容 |
|-------|:------:|-------------|
| 运动学 | ✅ | Pinocchio FK/IK、阻尼伪逆 |
| 物理仿真 | ✅ | MuJoCo 重力/接触/数值 Jacobian |
| 位置控制 | ✅ | 4+9 DOF 臂手解耦 (Plan C) |
| 灵巧手 | ✅ | Inspire RH56、15 种抓取模式、CLI |
| 实物化准备 | ✅ | 控制节点 Pinocchio-only，不依赖 MuJoCo topic（EE/朝向/雅可比/重力均本地计算） |
| 视觉集成 | 📋 | 检测 → 目标 → 自主抓取闭环 |
| 实物部署 | 🔮 | G1 真实机器人 + Unitree SDK 硬件桥（发布 /joint_states、写 LowCmd.tau；接口 /locked_joint_hold 已预留） |
