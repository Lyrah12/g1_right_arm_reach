<p align="center">
  <h1 align="center">G1 右臂抓取控制器</h1>
  <p align="center">ROS 2 + Pinocchio + MuJoCo · 任务空间 PID IK · 收敛 ≤1cm</p>
</p>

---

## 运行效果

20Hz PID 控制器驱动 G1 右臂 5 自由度到达三维目标点，MuJoCo 200Hz 物理仿真闭环。

```
收敛时间 ~1.7s  |  稳态误差 ≤ 1cm  |  支持运行时动态调参
```

## 坐标系

**世界坐标系**（MuJoCo global），原点 = 骨盆正下方地面：

| 轴 | 方向 |
|----|------|
| +X | 机器人前方 |
| +Y | 机器人**左侧**（右臂在 -Y） |
| +Z | 垂直向上（重力 -Z） |

| 关键位置 | 世界坐标 |
|----------|----------|
| 骨盆 | (0, 0, 0.823) |
| 桌面 | (0.50, -0.15, 0.77) |
| 杯子 | (0.20, 0.00, 0.85) |
| 方块 | (0.30, -0.10, 0.795) |

> `/target_point` 锁定 **腕关节**（`right_wrist_roll_rubber_hand`），手掌 mesh 向前延伸 ~10cm。抓取时请偏移。

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
colcon build --packages-select g1_right_arm_reach --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 启动

```bash
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
    urdf_path:=${HOME}/deps/unitree_ros/robots/g1_description/g1_23dof_rev_1_0.urdf \
    scene_path:=${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_grasp_scene.xml \
    viewer:=true
```

### 发送目标

```bash
# 杯子附近
ros2 topic pub /target_point geometry_msgs/msg/Point "{x: 0.20, y: 0.00, z: 0.90}" -1

# 方块附近
ros2 topic pub /target_point geometry_msgs/msg/Point "{x: 0.20, y: -0.10, z: 0.85}" -1
```

### 监控 & 调参

```bash
ros2 topic echo /debug/position_error              # 位置误差
ros2 param set /reach_node pid.kp 2.5               # 增大增益
ros2 param set /reach_node pid.ki 0.8               # 增大积分
ros2 param set /reach_node pid.posture_gain 0.3     # 调整姿态吸引
```

## 架构

```
  /target_point                /actual_ee_pose
  (世界坐标)                   /mujoco_jacobian
       │                       /joint_states
       ▼                            ▲
┌─────────────────┐    /joint_commands    ┌──────────────────────────┐
│   reach_node    │ ────────────────────► │   mujoco_bridge_node     │
│   PID IK 20Hz   │                       │   物理仿真 200Hz          │
│                 │                       │                          │
│ · 阻尼伪逆       │                       │ · EMA 平滑 (α=0.2)       │
│ · 零空间姿态吸引  │                       │ · Position servo         │
│ · 抗积分饱和      │                       │   (kp=300, kv=5)         │
│ · 发散检测       │                       │ · 骨盆+关节 hard clamp    │
└─────────────────┘                       │ · 数值 Jacobian           │
                                          └────────────┬─────────────┘
                                                       │
                                                       ▼
                                                MuJoCo 物理引擎
                                               (2ms 步长, 重力 -9.81)
```

`reach_node` 只通过 Topic 通信，不区分仿真还是真机。

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pid.kp` | 2.0 | 比例增益 |
| `pid.ki` | 0.4 | 积分增益 |
| `pid.kd` | 0.05 | 微分增益 |
| `pid.damping` | 2e-4 | 阻尼伪逆 λ |
| `pid.tolerance` | 0.01 | 收敛容差 (m) |
| `pid.posture_gain` | 0.2 | 零空间姿态吸引 |
| `reach_node.base_offset` | `[0, 0, 0.793]` | 骨盆世界位置 |

完整参数列表见 [`config/params.yaml`](config/params.yaml)。

## 项目阶段

| 阶段 | 状态 | 内容 |
|------|:----:|------|
| 运动学仿真 | ✅ | RViz + Pinocchio FK/IK |
| 物理仿真 | ✅ | MuJoCo 重力/接触/相机/Jacobian |
| 灵巧手 | 🔧 | 手指运动学 + 抓取动作库 |
| 视觉对接 | 📋 | 检测→目标→抓取闭环 |
| 真机部署 | 🔮 | G1 实机 |

## 开发

```bash
# 编译（Release）
colcon build --packages-select g1_right_arm_reach --cmake-args -DCMAKE_BUILD_TYPE=Release

# 测试
export G1_TEST_URDF_PATH=${HOME}/deps/unitree_ros/robots/g1_description/g1_23dof_rev_1_0.urdf
colcon test --packages-select g1_right_arm_reach
```

## 许可

BSD-3-Clause
