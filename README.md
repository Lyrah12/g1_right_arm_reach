# G1 23DOF 右臂 + BrainCo Revo2 灵巧手

面向宇树 **G1 23DOF 教育版**与 BrainCo 强脑 **Revo2 触觉灵巧手**的 ROS 2 控制与 MuJoCo 仿真项目。机械臂使用右肩三轴、右肘和右腕 roll 共五个关节，灵巧手使用六个主动手指关节；臂与手由独立接口控制。

## 项目概况

- **关节控制**：五轴机械臂位置目标、可选原始力矩输入，以及六轴手指位置目标。
- **TCP 控制与动作编排**：保留 TCP 三维位置 + 工具轴 roll 的任务空间力矩控制、内置手型及 `reach_and_grasp` 编排。roll 相对标定零姿态定义，默认工具轴为 Revo2 局部 `+Z`。
- **状态反馈**：关节位置、速度、执行器力矩、TCP 世界位姿和 MuJoCo 接触信息。
- **控制保护**：关节位置模式提供参考速度和输出力矩限制；原 TCP/硬件控制路径保留速度、加速度、任务力、力矩及变化率限幅、deadman 和状态超时保护。
- **仿真与实机代码**：原 TCP 控制律由 C++ 实现，可供仿真和实机共用；保留 Unitree 与 BrainCo SDK 适配器，以及可选的实体手触觉抓稳闭环。

默认场景采用固定基座，右臂和手之外的 G1 关节固定，不包含桌面或抓取物体。

## 模块与控制方式

| 模块 | 功能 | 默认频率 |
|---|---|---|
| `mujoco_bridge_node.py` | 物理仿真、五轴关节 PD/力矩执行、手指位置执行、状态与接触反馈 | 物理/关节 PD 500 Hz，反馈 200 Hz |
| `reach_node` | TCP XYZ + roll 控制，输出右臂五轴力矩 | 50 Hz |
| `hand_controller_node` | 六轴手型与位置插值，可选触觉闭环 | 50 Hz |
| `action_sequencer_node` | 臂与手的动作编排 | 50 Hz |
| `g1_arm_sdk_adapter` | Unitree `rt/arm_sdk` 实机通信 | 200 Hz |
| `revo2_sdk_adapter` | BrainCo SDK 位置、速度及触觉 I/O | 50 Hz |

原 TCP 控制采用 `tau = J_alloc^T * wrench + gravity + N * (tau_posture - D * dq)`，将位置任务分配给肩、肘，将 roll 任务分配给腕 roll。关节 demo 则直接由 MuJoCo bridge 执行重力补偿与逐轴 PD。

**关节接口使用 `mujoco_joint_demo.launch.py`**，只加载 MJCF 并启动 MuJoCo bridge。模型位于 `config/models/`，参数位于 `config/params.yaml`。

## 1. 环境与编译

环境：Ubuntu 24.04 / ROS 2 Jazzy、C++17、Eigen3、Pinocchio。ROS 所用 Python 环境需安装 `mujoco`、`numpy`、`rclpy`；本机验证的 MuJoCo 版本为 3.11.0。

在仓库根目录编译，以下以 `~/Desktop/g1_right_arm_reach` 为例：

```bash
cd ~/Desktop/g1_right_arm_reach
source /opt/ros/jazzy/setup.bash
python3 -c "import mujoco, numpy, rclpy; print(mujoco.__version__)"
colcon build --symlink-install --packages-select g1_right_arm_reach \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
```

## 2. 对外 Topic

关节 demo 默认使用位置模式（`arm_control_mode:=position`）。

| Topic | 消息类型 | 方向与语义 |
|---|---|---|
| `/arm_position_commands` | `sensor_msgs/msg/JointState` | 输入：五轴绝对关节位置目标，使用 name + position [rad]；仅位置模式订阅 |
| `/hand_commands` | `sensor_msgs/msg/JointState` | 输入：六轴主动手指绝对位置目标，使用 name + position [rad] |
| `/joint_states` | `sensor_msgs/msg/JointState` | 输出：实际 position [rad]、velocity [rad/s]、执行器 effort [N·m] |
| `/actual_ee_pose` | `geometry_msgs/msg/PoseStamped` | 输出：TCP 世界位姿，位置 [m]，ROS 四元数 x/y/z/w，frame_id=world |
| `/mujoco/contacts` | `g1_right_arm_reach/msg/ContactArray` | 输出：所有接触的当前快照；无接触也持续发布 contacts: [] |
| `/mujoco_gravity` | `std_msgs/msg/Float64MultiArray` | 输出：按下述臂顺序排列的重力补偿力矩 [N·m]，不包含速度偏置项 |
| `/joint_commands` | `sensor_msgs/msg/JointState` | 可选输入：五轴原始力矩，使用 name + effort [N·m]；仅力矩模式订阅 |

默认物理 **500 Hz**，关节状态、TCP 和接触反馈约 **200 Hz**；同一快照使用相同 ROS 时间戳。

默认 `/joint_states` 有 **16 个关节**：5 个臂关节、6 个主动手指、5 个从动手指。请按 `name` 匹配状态；`effort` 为仿真执行器力矩。

## 3. 五轴机械臂位置目标

| 顺序 | 关节名 | 默认 MJCF 范围 [rad] |
|---|---|---|
| 0 | `right_shoulder_pitch_joint` | [-3.0892, 2.6704] |
| 1 | `right_shoulder_roll_joint` | [-2.2515, 1.5882] |
| 2 | `right_shoulder_yaw_joint` | [-2.618, 2.618] |
| 3 | `right_elbow_joint` | [-1.0472, 2.0944] |
| 4 | `right_wrist_roll_joint` | [-1.97222, 1.97222] |

初始位置为 `[0.0, -0.35, 0.0, 0.0, 0.35]` rad。

每条命令须包含五个互不重复的关节名和五个有限、未越界的位置，`position[i]` 对应 `name[i]`，顺序可调整。缺轴、重复名、未知名、NaN/Inf 或越界会**整条拒绝并保持上一目标**，原因写入节点日志。仅使用 name 和 position 字段。

```bash
ros2 topic pub --once /arm_position_commands sensor_msgs/msg/JointState \
  "{name: [right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint, right_elbow_joint, right_wrist_roll_joint], position: [0.15, -0.30, 0.10, 0.20, 0.45]}"
```

**单次发送后持续保持目标**，直到收到新目标或仿真重启。通过 `/joint_states` 判断实际到位情况。

参数位于 `config/params.yaml` 的 `mujoco_bridge`，按上表顺序排列，启动时读取、重启生效：

| 参数 | 默认值 | 单位 |
|---|---|---|
| arm_position_kp | [80, 80, 60, 60, 12] | N·m/rad |
| arm_position_kd | [8, 8, 5, 5, 1.5] | N·m·s/rad |
| arm_position_max_torque | [20, 20, 15, 15, 3] | N·m，包含重力补偿后的总输出限幅 |
| arm_position_max_velocity | [1, 1, 1, 1, 1.5] | rad/s，内部参考位置最大变化率 |

## 4. 六轴灵巧手位置目标

| 顺序 | 关节名 | 默认范围 [rad] |
|---|---|---|
| 0 | `right_thumb_proximal_joint` | [0, 1.0297443]，拇指屈伸 |
| 1 | `right_thumb_metacarpal_joint` | [0, 1.5707963]，拇指内收外展 |
| 2 | `right_index_proximal_joint` | [0, 1.4137167]，食指 |
| 3 | `right_middle_proximal_joint` | [0, 1.4137167]，中指 |
| 4 | `right_ring_proximal_joint` | [0, 1.4137167]，无名指 |
| 5 | `right_pinky_proximal_joint` | [0, 1.4137167]，小拇指 |

```bash
ros2 topic pub --once /hand_commands sensor_msgs/msg/JointState \
  "{name: [right_thumb_proximal_joint, right_thumb_metacarpal_joint, right_index_proximal_joint, right_middle_proximal_joint, right_ring_proximal_joint, right_pinky_proximal_joint], position: [0.30, 0.20, 0.40, 0.35, 0.30, 0.25]}"
```

手部按名称匹配，支持部分更新，其余关节保持上一目标。未知名称、缺少位置和非有限值的项忽略；有限目标按执行器范围裁剪。仅使用 name 和 position 字段，单次发送后持续保持。接触可能阻挡手指到位，应读取实际反馈。

默认手指位置伺服 kp=30、kv=0.8。

## 5. 接触反馈

```bash
ros2 topic echo /mujoco/contacts --once
ros2 topic hz /mujoco/contacts
```

`ContactArray` 包含 `header` 和当前接触列表 `contacts[]`，`header.frame_id=world`；无接触时列表为空。

| Contact 字段 | 含义 |
|---|---|
| geom1_id、geom2_id | 模型内的几何体 ID |
| geom1_name、geom2_name | 碰撞几何体名称；未命名时为 `geom_<id>` |
| body1_name、body2_name | 几何体所属刚体名称 |
| position | 世界坐标接触点 [m] |
| normal | 世界坐标单位法向，从 geom1 指向 geom2 |
| distance | 有符号分离距离 [m]；负值表示穿透 |
| active | 是否参与 MuJoCo 约束求解 |
| normal_force | 压缩法向力 [N]；未参与求解时为零 |
| wrench.force | 作用在 geom2 上、世界坐标表达的接触力 [N] |
| wrench.torque | 作用在 geom2 上、关于接触点的力矩 [N·m]，不是关于刚体原点 |

力来自 `mj_contactForce`，转换到世界坐标系后发布；geom1 承受反向力/力矩。可按几何体或刚体名称筛选接触，结合 active 与 normal_force 判断受力。

## 6. 启动方式

默认启动五轴关节位置控制与六轴手指位置控制，显示 MuJoCo 窗口：

```bash
cd ~/Desktop/g1_right_arm_reach
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch g1_right_arm_reach mujoco_joint_demo.launch.py viewer:=true
```

| 用途 | 启动方式 |
|---|---|
| 无界面关节仿真 | `ros2 launch g1_right_arm_reach mujoco_joint_demo.launch.py viewer:=false` |
| 原始五轴力矩 | `ros2 launch g1_right_arm_reach mujoco_joint_demo.launch.py arm_control_mode:=torque viewer:=true` |
| TCP 控制与手型编排 | `ros2 launch g1_right_arm_reach mujoco_demo.launch.py viewer:=true` |
| 实机控制 | `hardware_controllers.launch.py`，另需 Unitree `unitree_sdk2` 和 BrainCo `bc_stark_sdk` |

默认场景为 `config/models/g1_23dof_revo2_arm_only.xml`，可用 `scene_path:=/absolute/path/scene.xml` 指定其他场景。

力矩模式仅接收 `/joint_commands`，五轴 effort 须完整且持续刷新（建议 50 Hz），0.10 s 超时后退回重力补偿；发送端负责限幅。同一 ROS 域只启动一套控制入口。

TCP 入口使用 URDF/Pinocchio，接收 `/end_effector_command`（XYZ 相对肩部）、`/hand_command` 和 `/action_command`。例如 `ros2 run g1_right_arm_reach action_cli reach 0.30 -0.10 0.00 0.30`；`reach_and_grasp` 的成功结果表示编排完成。

## 7. 验证

默认关节 demo 运行时，另开已 source 的终端执行：

```bash
ros2 run g1_right_arm_reach verify_mujoco_interfaces.py
```

脚本会移动臂和手，检查位置到位、目标保持、越界拒绝、反馈频率和自接触，结束后恢复初始姿态并张手。
