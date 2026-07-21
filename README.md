# Unitree G1 右臂 + Wuji 灵巧手 控制

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros" alt="ROS 2">
  <img src="https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B" alt="C++17">
  <img src="https://img.shields.io/badge/Python-3.10+-3776AB?logo=python" alt="Python">
  <img src="https://img.shields.io/badge/MuJoCo-3.1+-00A98F" alt="MuJoCo">
  <img src="https://img.shields.io/badge/License-BSD--3--Clause-blue.svg" alt="License">
  <img src="https://img.shields.io/badge/DOF-27-important" alt="27-DOF">
</p>

---

**`g1_right_arm_reach`** 是一个 ROS2 软件包，用于在 MuJoCo 仿真中控制 Unitree G1 人形机器人的右臂和五指灵巧手（Wuji Hand 2）。实现了完全解耦的**臂手架构**：手臂通过阻尼伪逆 IK 负责 4 自由度腕部定位，手部模块独立管理 3 自由度腕部朝向和 20 自由度手指抓取——共计 **27 个自由度**。

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
│   │  (肩部×3 + 肘部)         │   │  (3×3 MuJoCo Jacobian 求逆) │   │
│   │  → 腕部位置              │   │  手指阻抗控制 (20-DOF)       │   │
│   │                         │   │  GraspLibrary (15 种模式)    │   │
│   │     /joint_commands     │   │     /hand_commands          │   │
│   │     (4-DOF 手臂)        │   │     (23-DOF: 腕部+手指)      │   │
│   └────────────┬────────────┘   └──────────────┬──────────────┘   │
└───────────────┼────────────────────────────────┼──────────────────┘
                └────────────────┬───────────────┘
                                 ▼
                   ┌───────────────────────────────┐
                   │  mujoco_bridge_node.py        │
                   │     (200 Hz, Python)          │
                   │                               │
                   │  MuJoCo 物理步进               │
                   │  位置伺服执行器                │
                   │  数值 Jacobian (3×7+3×3)      │
                   │  EE 位姿 + 手掌朝向            │
                   └───────────────────────────────┘
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
                         finger1×4 ─── 拇指 (4 关节)
                         finger2×4 ─── 食指 (4 关节)
                         finger3×4 ─── 中指 (4 关节)
                         finger4×4 ─── 无名指 (4 关节)
                         finger5×4 ─── 小指 (4 关节)

  /joint_commands (4-DOF)            │  /hand_commands (23-DOF)
```

**核心思想：** 手臂模块只管腕部在三维空间中的*位置*，手部模块接管腕部以下的所有内容——朝向、手指姿态和抓取执行。这使得系统**不依赖特定手部模型**。

### 灵巧手规格

| 属性 | 值 |
|-----------|-------|
| 型号 | Wuji Hand 2（5 根独立驱动手指） |
| 手指自由度 | 5 × 4 = **20-DOF** |
| 腕部自由度 | roll + pitch + yaw = **3-DOF** |
| 手部合计 | **23-DOF** |
| 手臂自由度 | 肩部×3 + 肘部 = **4-DOF** |
| **系统总计** | **27-DOF** |

## 特性

- **臂手解耦架构** —— 4-DOF 手臂 IK + 23-DOF 手部控制，模块完全独立
- **阻尼伪逆 IK** —— 4-DOF → 3-DOF，自适应阻尼 + 零空间姿态吸引
- **腕部朝向 PID** —— MuJoCo 3×3 数值 Jacobian（避免 URDF FK 模型不匹配）
- **手指阻抗控制** —— 位置编码力矩，直接跳变 + 阻抗控制器过渡
- **15 种预定义抓取模式** —— 精细抓取、力量抓取、手势姿态
- **动作编排状态机** —— IDLE → REACHING → SETTLING → GRASPING，带收敛检测
- **统一命令行工具** —— 交互模式支持 RPY 朝向、单次命令和 Topic 级访问
- **200 Hz MuJoCo 物理仿真** —— 位置伺服执行器，支持分组 EMA 平滑
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

# G1 机器人模型 (URDF + MJCF)
git clone https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/unitreerobotics/unitree_mujoco.git

# Wuji Hand 2 模型
git clone https://github.com/wuji-technology/wuji-hand-description.git
```

### 3. 配置 Wuji Hand 场景

```bash
# 复制并修补 MJCF 文件，将 Wuji Hand 注入 G1 场景
bash ~/g1_ws/src/g1_right_arm_reach/scripts/setup_wuji_hand.sh
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
  urdf_path:=${HOME}/deps/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf \
  scene_path:=${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_wuji_scene.xml \
  viewer:=true
```

### 6. 发送第一条指令

```bash
# 终端 2：张手 → 伸手并捏取
source ~/g1_ws/install/setup.bash

python3 ~/g1_ws/install/g1_right_arm_reach/lib/g1_right_arm_reach/action_cli.py open
python3 ~/g1_ws/install/g1_right_arm_reach/lib/g1_right_arm_reach/action_cli.py \
  reach_and_grasp 0.20 -0.05 -0.10 close
```

## 使用方式

### Python 交互模式

```bash
python3 ~/g1_ws/install/g1_right_arm_reach/lib/g1_right_arm_reach/action_cli.py interactive
```

交互模式命令示例：

```
>>> list                                # 列出所有命令和抓取模式
>>> 0.20 -0.05 -0.10 open               # 伸手 + 张手
>>> 0.20 -0.05 -0.10 close              # 伸手 + 握拳
>>> 0.20 -0.05 -0.10 pinch              # 伸手 + 精准捏取
>>> 0.20 -0.05 -0.10 rpy 0 0 0.5 close  # 伸手 + yaw 旋转 90° + 握拳
>>> grasp power_grasp                   # 仅动手（不移动手臂）
>>> home                                # 回到就绪姿态
```

### Python 单次命令

```bash
# 列出所有可用命令和抓取模式
python3 action_cli.py list

# 仅动手臂（手部不动）
python3 action_cli.py reach 0.20 -0.05 -0.10

# 仅动手（手臂不动）
python3 action_cli.py grasp power_grasp

# 手臂 + 抓取（完整动作流水线）
python3 action_cli.py reach_and_grasp 0.20 -0.05 -0.10 pinch

# 快捷命令
python3 action_cli.py home          # 回到就绪姿态
python3 action_cli.py open          # 张手
python3 action_cli.py close         # 握拳
```

### 直接 Topic 发布

```bash
ros2 topic pub -1 /action_command g1_right_arm_reach/msg/ActionCommand \
  "{action_type: reach_and_grasp, target_x: 0.2, target_y: -0.05, \
    target_z: -0.1, ori_w: 1.0, grasp_type: pinch, grasp_duration: 0.08}"
```

### 监控与调参

```bash
# 观察位置/朝向收敛情况
ros2 topic echo /debug/position_error          # 手臂位置误差 (m)
ros2 topic echo /debug/orientation_error       # 腕部朝向误差 (rad)

# 运行时参数调节
ros2 param set /reach_node pid.kp 2.5
ros2 param set /hand_controller kp_wrist 2.5
ros2 param set /hand_controller kp_finger 250.0
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
- **骨盆偏移**：`[0, 0, 0.823]`（URDF 骨盆原点 → MuJoCo 世界坐标对齐）


| 目标 `(x, y, z)` | 距离 | 说明 |
|---------------------|----------|-------|
| `[0.18, −0.05, −0.10]` | ~21 cm | 默认就绪姿态 |
| `[0.25, 0.00, 0.00]` | ~25 cm | 正前方中等伸展 |
| `[0.15, −0.10, −0.05]` | ~18 cm | 前偏左，略向下 |

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

### 腕部：朝向 PID + Jacobian 求逆

$$e_{ori} = 2 \cdot \text{axis}(q_{des} \cdot q_{curr}^{-1}) \quad \text{(轴角误差)}$$

$$\tau_{task} = k_p \cdot e_{ori} + k_i \cdot \int e_{ori}\\dt + k_d \cdot \dot{e}_{ori}$$

$$\Delta q = J_{wrist}^{-1} \cdot \tau_{task} \quad \text{(MuJoCo 3×3 数值 Jacobian)}$$

$$q_{cmd} = q_{ref} + \int \Delta q\\dt \quad \text{(基于偏移量，带抗饱和)}$$

### 手指：前馈阻抗控制

$$q_{des} = q_{target}\quad\text{(收到抓取指令时直接跳变)}$$

$$\tau_{des} = k_p^{finger} \cdot (q_{des} - q_{act}) + k_d^{finger} \cdot (0 - \dot{q}_{act})$$

$$q_{cmd} = q_{des} + \frac{\tau_{des}}{k_p^{inner}} \quad \text{(位置编码力矩)}$$

## 配置参数

全部参数见 [`config/params.yaml`](config/params.yaml)。关键参数如下：

### 手臂 IK (`reach_node`)

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `pid.kp` | 2.0 | IK 比例增益 |
| `pid.ki` | 0.4 | IK 积分增益（抗饱和上限 0.10） |
| `pid.kd` | 0.05 | IK 微分增益 |
| `pid.damping` | 2e−4 | 阻尼伪逆 λ（自适应） |
| `pid.tolerance` | 0.005 | 位置收敛容差 (m) |
| `pid.posture_gain` | 0.2 | 零空间姿态吸引强度 |

### 手部控制器

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `kp_wrist` | 2.0 | 腕部朝向 P 增益 |
| `ki_wrist` | 0.05 | 腕部朝向 I 增益 |
| `kd_wrist` | 0.12 | 腕部朝向 D 增益 |
| `wrist_offset_limit` | 2.0 | 腕部偏移上限（相对就绪姿态，rad） |
| `kp_finger` | 50.0 | 手指阻抗刚度 (N·m/rad) |
| `kd_finger` | 0.5 | 手指阻抗阻尼 (N·m·s/rad) |
| `inner_servo_kp` | 5.0 | MuJoCo 内环 kp（必须匹配 MJCF 手指 kp） |

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
| `/hand_commands` | `JointState` | hand_controller → bridge | 23-DOF（腕部×3 + 手指×20） |
| `/joint_states` | `JointState` | bridge → 各节点 | 全 27-DOF 反馈 |
| `/actual_ee_pose` | `PointStamped` | bridge → reach_node | 腕部位置（世界坐标系） |
| `/actual_hand_orientation` | `QuaternionStamped` | bridge → hand_controller | 手掌朝向（世界坐标系） |
| `/mujoco_wrist_jacobian` | `Float64MultiArray` | bridge → hand_controller | 3×3 腕部 Jacobian（行主序） |
| `/debug/position_error` | `Float64` | reach_node → sequencer | 位置收敛误差 (m) |
| `/debug/orientation_error` | `Float64` | hand_controller → sequencer | 朝向收敛误差 (rad) |

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
│   ├── hand_controller_node.cpp  # 手部控制节点 (50 Hz, 23-DOF)
│   ├── action_sequencer_node.cpp # 动作编排节点 (50 Hz)
│   └── target_publisher_node.cpp # 测试工具
├── scripts/
│   ├── mujoco_bridge_node.py     # MuJoCo 物理桥接 (200 Hz)
│   ├── inject_wuji_hand.py       # Wuji Hand MJCF 注入器
│   ├── setup_wuji_hand.sh        # Wuji Hand 一键配置
│   ├── action_cli.py             # 统一命令行工具
│   └── plot_error.py             # 误差绘图工具
├── msg/
│   ├── ActionCommand.msg         # 统一动作指令
│   ├── HandCommand.msg           # 手部朝向 + 抓取
│   ├── GraspCommand.msg          # 遗留抓取指令
│   ├── DetectedObject.msg        # 视觉检测 → 目标（预留）
│   ├── DetectedObjectArray.msg   # 批量检测（预留）
│   ├── ArmStatus.msg             # 手臂状态反馈（预留）
│   └── GraspResult.msg           # 抓取结果（预留）
├── action/
│   └── GraspObject.action        # 异步抓取 Action 接口（预留）
├── config/
│   ├── params.yaml               # 全部运行时参数
│   ├── g1_grasp_scene.xml        # MJCF: G1 + 简易夹爪
│   └── g1_wuji_scene.xml         # MJCF: G1 + Wuji Hand 2
├── launch/
│   └── mujoco_demo.launch.py     # 启动全部节点
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
| 位置控制 | ✅ | 4+23 DOF 臂手解耦 (Plan C) |
| 灵巧手 | ✅ | Wuji Hand 2、15 种抓取模式、CLI |
| 视觉集成 | 📋 | 检测 → 目标 → 自主抓取闭环 |
| 实物部署 | 🔮 | G1 真实机器人 + Unitree SDK |
