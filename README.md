# G1 右臂抓取控制器

基于 ROS2 Jazzy 的 C++ 软件包，用于控制宇树 G1 人形机器人的右臂到达指定的 (x, y, z) 目标点。

## 项目需求

| 条目 | 内容 |
|------|------|
| 机器人 | 宇树 G1 人形机器人（23 DOF 教育版） |
| 任务 | 右臂末端移动到指定 (x, y, z) 目标点，位置误差 ≤ 3cm |
| 可动关节 | 仅右臂 5 关节（shoulder_pitch / roll / yaw, elbow, wrist_roll），其余全部锁定 |
| 外部依赖 | 目标点由外部指定（Topic 发送），不需要视觉识别 |
| 控制策略 | 纯 PID（阻尼伪逆 IK + 变增益 + 零空间关节限位避让） |
| 语言 | C++（C++17） |
| 系统 | Ubuntu 24.04 LTS |
| 框架 | ROS2 Jazzy Jalisco |
| 运动学 | Pinocchio |
| 数值计算 | Eigen3 |
| 构建 | colcon + CMake |
| 测试 | GTest（ament_cmake_gtest） |

## 阶段规划

| 阶段 | 状态 | 内容 |
|------|:----:|------|
| Phase 1 | ✅ 已完成 | RViz 运动学演示——给定目标点，纯 PID IK 求解关节轨迹 |
| Phase 2 | 📋 规划中 | 轨迹跟踪——接收视觉感知输出的指尖参考轨迹，在线跟踪 |
| Phase 3 | 🔮 远期 | 抓取集成——末端抓手控制、抓取检测 |
| Phase 4 | 🔮 远期 | 真机部署——G1 实际右臂控制 |

## 架构

```
Target (x,y,z) → [reach_node] → [PID 控制器] → 关节指令
```

- **PID**：解析运动率控制（RMRC），采用阻尼最小二乘伪逆、变增益和零空间关节限位避让。唯一控制器——无抽象层、无 MPC。

## 前置条件

- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- C++17 编译器（GCC 13+）

### 安装依赖

```bash
# ROS2 Jazzy（二进制安装）
# 参见：https://docs.ros.org/en/jazzy/Installation.html

# Pinocchio
sudo apt install ros-jazzy-pinocchio

# Eigen3
sudo apt install libeigen3-dev

# GTest（用于单元测试）
sudo apt install ros-jazzy-ament-cmake-gtest

# RViz + 可视化工具
sudo apt install ros-jazzy-rviz2 ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui
```

### 获取 G1 URDF 模型

```bash
cd ~/deps
git clone https://github.com/unitreerobotics/unitree_ros.git
# URDF 位于：unitree_ros/robots/g1_description/
# 23 自由度教育版：urdf/g1_23dof_rev_1_0.urdf
```

## 构建

```bash
mkdir -p ~/g1_ws/src
cd ~/g1_ws/src
git clone <仓库地址> g1_right_arm_reach

cd ~/g1_ws
colcon build --packages-select g1_right_arm_reach --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行（Phase 1：RViz 运动学演示）

```bash
# 终端 1：启动 RViz + reach_node
ros2 launch g1_right_arm_reach rviz_demo.launch.py \
    urdf_path:=/path/to/g1_23dof_rev_1_0.urdf

# 终端 2：发送测试目标点
ros2 run g1_right_arm_reach target_publisher_node

# 或发送单个目标点：
ros2 topic pub /target_point geometry_msgs/msg/Point "{x: 0.30, y: -0.40, z: 0.20}"
```

## 监控

```bash
# 查看位置误差
ros2 topic echo /debug/position_error

# 查看控制器状态
ros2 topic echo /debug/controller_status
```

## 调参

参数文件位于 `config/params.yaml`。关键参数：

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `pid.kp` | 2.5 | PID 比例增益 |
| `pid.ki` | 3.0 | PID 积分增益 |
| `pid.kd` | 0.05 | PID 微分增益 |
| `pid.damping` | 1e-4 | 雅可比阻尼（越大越稳定，精度越低） |
| `pid.tolerance` | 0.003 | 收敛容差（米） |

## 运行测试

```bash
export G1_TEST_URDF_PATH=/path/to/g1_23dof_rev_1_0.urdf
colcon test --packages-select g1_right_arm_reach
colcon test-result --verbose
```

## 项目结构

```
.
├── include/g1_right_arm_reach/
│   ├── robot_model.hpp           # URDF 加载 + buildReducedModel
│   ├── kinematics.hpp            # 正运动学 + 雅可比 + 伪逆
│   └── pid_controller.hpp        # PID IK 控制器 + ControlResult 结构体
├── src/
│   ├── robot_model.cpp           # URDF、简化模型、正运动学
│   ├── kinematics.cpp            # 雅可比、伪逆、积分
│   ├── pid_controller.cpp        # PID IK 实现
│   ├── reach_node.cpp            # ROS2 主节点
│   └── target_publisher_node.cpp # 测试目标发布节点
├── scripts/                      # 工具脚本（末端计算、误差绘图）
├── launch/                       # ROS2 启动文件
├── config/                       # YAML 参数 + RViz 配置
├── test/                         # GTest 单元测试
├── CMakeLists.txt                # 构建配置
├── package.xml                   # ROS2 软件包清单
└── README.md
```

---

## 下一步：轨迹跟踪（Phase 2）

### 背景

当前系统实现的是**单点静态抓取**——给定一个固定的 (x, y, z) 目标，机械臂移动到该位置。

下一阶段的目标是**轨迹跟踪**：视觉感知博士会输出夹爪两个末端点（指尖）的参考轨迹，控制系统需要驱动右臂末端跟上这条时变轨迹。

### 架构演变

```
Phase 1（当前）:
  Target (x,y,z) → reach_node → PID → 关节指令

Phase 2（目标）:
  视觉感知 → 轨迹 → trajectory_tracker → 前馈+PID → 关节指令
```

### 第一步：接口约定（与视觉团队对齐）

在与视觉团队对接之前，需要约定好消息格式。建议：

**方案 A（简单）**：用 `nav_msgs/msg/Path`，每个 `PoseStamped` 的 `header.stamp` 表示期望到达时刻。

**方案 B（精确）**：自定义消息，含位置 + 速度前馈。

```
# TrajectoryPoint.msg
builtin_interfaces/Time  stamp
geometry_msgs/Point      position
geometry_msgs/Vector3    velocity   # 可选：前馈速度
```

**建议先选方案 A**，用标准消息快速验证链路，后续再考虑自定义消息。

需要视觉团队回答的问题：
1. 轨迹的发布频率？（10Hz? 30Hz?）
2. 时间戳的语义？（"此刻应该在的位置" vs "未来目标"）
3. 两个指尖分别发两条轨迹，还是合并为夹爪中心 + 开合角？
4. 轨迹是否包含速度信息，还是仅位置？

### 第二步：代码改造清单

| 模块 | 改动 | 说明 |
|------|------|------|
| `reach_node.cpp` | 新增轨迹订阅 | 订阅 `Path` 或自定义消息替代单点 `Point` |
| 新建 `trajectory_tracker.cpp` | 轨迹插值 + 前馈 | 根据时间戳在线插值出当前期望位置/速度 |
| `pid_controller.cpp` | 增加速度前馈 | `v_task = Kp·e + Kd·ė + Ki·∫e + v_feedforward` |
| `reach_node.cpp` | 控制率提升 | 从 20Hz 提升到 100Hz+（跟踪需要更高带宽） |
| `CMakeLists.txt` | 编译新文件 | 添加 `trajectory_tracker` 库和可执行文件 |

### 第三步：轨迹插值策略

视觉轨迹通常是离散时间采样点（如 30Hz），而控制循环可能更快（100Hz），中间帧需要插值：

```
视觉轨迹:  ●────●────●────●   (低频采样, 如30Hz)
控制周期:  |||||||||||||||||   (高频控制, 如100Hz)
           ↑ 插值点
           
插值方法（按复杂度递增）:
  1. 零阶保持    — 直接用最近采样点（简单，但不够平滑）
  2. 线性插值    — 两点间线性过渡（推荐起步方案）
  3. 三次样条    — 更平滑的速度/加速度（最终方案）
```

### 第四步：性能要求升级

| 指标 | Phase 1（当前） | Phase 2（轨迹跟踪） |
|------|:-----------:|:----------------:|
| 跟踪误差 | < 1cm（稳态） | < 1cm（动态） |
| 控制频率 | 20Hz | ≥ 100Hz |
| 输入类型 | 固定点 | 时变轨迹 |
| 前馈 | 无 | 速度前馈 |
| 控制器 | PID | PID + Feedforward |

### 第五步：调试与验证

1. **离线回放**：先录制一段视觉轨迹到 rosbag，反复回放调试控制参数
2. **先跟踪虚拟轨迹**：用 `target_publisher_node` 改造成轨迹发布器（发正弦波等），不依赖视觉团队即可开发
3. **逐级测试**：先跟踪慢速轨迹（周期 > 5s），逐步加快

### 参考消息类型

```bash
# 查看可用消息
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 interface show nav_msgs/msg/Path

# 测试发送轨迹（单点反复发送模拟慢速轨迹）
ros2 topic pub /target_trajectory nav_msgs/msg/Path "{header: {frame_id: 'pelvis'}, poses: [{header: {stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: 0.10, y: -0.15, z: 0.10}, orientation: {w: 1.0}}}]}"
```

---

## 许可证

BSD-3-Clause
