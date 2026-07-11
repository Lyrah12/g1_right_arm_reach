# G1 右臂抓取控制器

基于 ROS2 Jazzy + Pinocchio + MuJoCo，控制宇树 G1 右臂到达目标点并实现灵巧手抓取。收敛误差 ≤ 1cm，调节时间 ~1.8s。

## 阶段

| 阶段 | 状态 | 内容 |
|------|:----:|------|
| Phase 1 | ✅ | RViz 运动学演示 — PID IK |
| Phase 2 | ✅ | MuJoCo 物理仿真 — 重力/接触/相机 |
| Phase 3 | 🔧 | 灵巧手操作 — 指尖→掌心映射 + 抓取动作库 |
| Phase 4 | 📋 | 视觉模块对接 |
| Phase 5 | 🔮 | 真机部署 |

## 快速开始

### 环境

```bash
# ROS2 Jazzy + 依赖
sudo apt install -y ros-jazzy-desktop ros-jazzy-pinocchio ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher ros-jazzy-cv-bridge libeigen3-dev

# MuJoCo
pip install --break-system-packages mujoco opencv-python "numpy<2"

# G1 模型
mkdir -p ~/deps && cd ~/deps
git clone https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```

### 构建

```bash
source /opt/ros/jazzy/setup.bash
cd ~/g1_ws
colcon build --packages-select g1_right_arm_reach --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 运行

```bash
# Phase 1: 运动学演示
ros2 launch g1_right_arm_reach rviz_demo.launch.py \
    urdf_path:=${HOME}/deps/unitree_ros/robots/g1_description/g1_23dof_rev_1_0.urdf

# Phase 2: MuJoCo 物理仿真
sed -i 's|g1_23dof.xml|'${HOME}'/deps/unitree_mujoco/unitree_robots/g1/g1_23dof.xml|' \
    ~/g1_ws/src/g1_right_arm_reach/config/g1_grasp_scene.xml
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
    urdf_path:=${HOME}/deps/unitree_ros/robots/g1_description/g1_23dof_rev_1_0.urdf \
    scene_path:=${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_grasp_scene.xml

# 发送目标（世界坐标，pelvis 高 0.793m）
ros2 topic pub /target_point geometry_msgs/msg/Point "{x: 0.15, y: -0.15, z: 0.90}" -1
```

### 调参

```bash
ros2 param set /reach_node pid.kp 2.0   # 运行时动态调整
ros2 topic echo /debug/position_error    # 监控误差
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `pid.kp` | 1.5 | 比例增益 |
| `pid.ki` | 0.5 | 积分增益 |
| `pid.tolerance` | 0.01 | 收敛容差 (m) |

## 架构

```
Phase 2:  /target_point → reach_node (PID IK) ←→ mujoco_bridge (200Hz 物理)
                             ↓                        ↓
                       /joint_commands           /joint_states
                                                  /actual_ee_pose (世界系)
                                                  /camera/rgb /camera/depth

Phase 3:  抓取动作库 (YAML) → 掌心 IK + 指尖 FK → /joint_commands (臂+手指)
```

核心原则：`reach_node` 只与 Topic 交互，不感知底层是仿真还是真机。

## Phase 3规划: 灵巧手操作

核心思路：**不同物体的抓取 = 手指末端相对掌心的位置配置不同** → 建立可复用的抓取动作库。

### 子任务

| # | 任务 | 说明 |
|---|------|------|
| 3.1 | 手指运动学模型 | 扩展 `active_joints` 加入手指关节，建立指尖 FK（相对掌心） |
| 3.2 | 掌心 IK | 掌心世界位姿 + 指尖配置 → 全臂+手关节角度 |
| 3.3 | 抓取动作库 | YAML 定义标准抓取姿态（捏取/包络/钩取/球形） |
| 3.4 | MuJoCo 仿真验证 | 接触力分布、抓取稳定性测试 |
| 3.5 | 力控/柔顺 | 力矩控制防抓碎（鸡蛋、纸杯） |

### 抓取类型

| 类型 | 适用物体 | 指尖特征 |
|------|----------|----------|
| 捏取 (precision) | 鸡蛋、瓶盖 | 拇指对向食/中指 |
| 包络 (power) | 电缆、工具柄 | 五指屈曲合拢 |
| 侧捏 (lateral) | 卡片、钥匙 | 拇指侧压食指 |
| 球形 (spherical) | 苹果、网球 | 五指张开环抱 |

## Phase 4规划: 视觉模块接口 

| Topic | 类型 | 方向 |
|-------|------|:----:|
| `/grasp_target` | `PoseStamped` | 视觉 → 抓取 |
| `/arm_status` | `ArmStatus` | 抓取 → 视觉 |
| `/grasp_result` | `GraspResult` | 抓取 → 视觉 |

## 测试

```bash
export G1_TEST_URDF_PATH=${HOME}/deps/unitree_ros/robots/g1_description/g1_23dof_rev_1_0.urdf
colcon test --packages-select g1_right_arm_reach
```

## 许可证

BSD-3-Clause
