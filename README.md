# G1 + Revo2 MuJoCo 控制接口 Demo

本项目提供固定基座 Unitree G1 23DOF + BrainCo Revo2 的 ROS 2 仿真接口：**五轴机械臂位置目标、六轴手指位置目标、关节状态、TCP 世界位姿和接触反馈**。场景不含桌面或抓取物体，不提供 RL 环境、奖励或训练配置。

**对接使用 `mujoco_joint_demo.launch.py`。** 此入口只加载 MuJoCo MJCF XML，不读取 URDF，不启动 RViz、TF 发布器或原有 TCP/手型控制器。原有 TCP 和硬件控制代码保留，见文末。

## 1. 环境、编译与启动

环境：Ubuntu 24.04 / ROS 2 Jazzy、C++17、Eigen3、Pinocchio，以及 ROS 所用 Python 中的 `mujoco`、`numpy`、`rclpy`。整个 ROS 包的构建仍保留旧 C++ 控制器的 Eigen/Pinocchio 依赖；新关节 demo 运行不使用 URDF。MuJoCo Python 包需单独准备，`package.xml` 不会自动安装它；本机验证版本为 MuJoCo 3.11.0。

可直接在本仓库根目录编译，无需移动到另一个工作空间。当前机器的仓库位于 `~/Desktop/g1_right_arm_reach`；其他机器请把下面的 `cd` 改为实际仓库路径：

```bash
cd ~/Desktop/g1_right_arm_reach
source /opt/ros/jazzy/setup.bash
python3 -c "import mujoco, numpy, rclpy; print(mujoco.__version__)"
colcon build --symlink-install --packages-select g1_right_arm_reach \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash

# 带 MuJoCo 窗口
ros2 launch g1_right_arm_reach mujoco_joint_demo.launch.py viewer:=true

# 无界面运行时使用：
# ros2 launch g1_right_arm_reach mujoco_joint_demo.launch.py viewer:=false
```

每次新开终端，都需要加载 ROS 和本项目安装环境；仅 `cd` 到项目目录不会让 ROS 自动找到包：

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/g1_right_arm_reach/install/setup.bash
ros2 pkg prefix g1_right_arm_reach
ros2 launch g1_right_arm_reach mujoco_joint_demo.launch.py viewer:=true
```

若报 `Package 'g1_right_arm_reach' not found`，先检查上述 source 是否执行成功；若包已找到但新 launch 或 Contact 消息不存在，重新执行前面的编译步骤，再 source。若把仓库放在 `~/g1_ws/src/` 中并从 `~/g1_ws` 编译，则应 source `~/g1_ws/install/setup.bash`，以实际构建位置为准。

默认场景为 `config/models/g1_23dof_revo2_arm_only.xml`；可用 `scene_path:=/absolute/path/scene.xml` 覆盖。自定义模型须保留五轴臂 motor、六轴手指 position 执行器及 `right_revo2_tcp` site，并在参数中指定匹配的关节名称。

## 2. 对外 Topic

以下均为 ROS 2 Topic；“输入”表示外部程序向仿真发布，“输出”表示外部程序订阅。新入口默认 `arm_control_mode:=position`，仅启动一个 `/mujoco_bridge` 节点。

| Topic | 消息类型 | 方向与语义 |
|---|---|---|
| `/arm_position_commands` | `sensor_msgs/msg/JointState` | 输入：五轴绝对关节位置目标，使用 name + position [rad]；仅位置模式订阅 |
| `/hand_commands` | `sensor_msgs/msg/JointState` | 输入：六轴主动手指绝对位置目标，使用 name + position [rad] |
| `/joint_states` | `sensor_msgs/msg/JointState` | 输出：实际 position [rad]、velocity [rad/s]、执行器 effort [N·m] |
| `/actual_ee_pose` | `geometry_msgs/msg/PoseStamped` | 输出：TCP 世界位姿，位置 [m]，ROS 四元数 x/y/z/w，frame_id=world |
| `/mujoco/contacts` | `g1_right_arm_reach/msg/ContactArray` | 输出：所有接触的当前快照；无接触也持续发布 contacts: [] |
| `/mujoco_gravity` | `std_msgs/msg/Float64MultiArray` | 输出：按下述臂顺序排列的重力补偿力矩 [N·m]，不包含速度偏置项 |
| `/joint_commands` | `sensor_msgs/msg/JointState` | 可选输入：五轴原始力矩，使用 name + effort [N·m]；仅力矩模式订阅 |

默认物理 **500 Hz**，关节状态、TCP 和接触反馈约 **200 Hz**；三种观测同一快照使用相同 ROS 时间戳。时间戳为 ROS 时钟时间，频率受机器负载影响；本 demo 不提供同步 step/reset 服务或 /clock。

默认 `/joint_states` 有 **16 个关节**：5 个臂关节、6 个主动手指、5 个从动手指。读取时按 `name` 查找，不能假设反馈数组与命令列表顺序一致。`effort` 为执行器广义力，不是接触力或实体力矩传感器测量。

## 3. 五轴机械臂位置目标

| 顺序 | 关节名 | 默认 MJCF 范围 [rad] |
|---|---|---|
| 0 | `right_shoulder_pitch_joint` | [-3.0892, 2.6704] |
| 1 | `right_shoulder_roll_joint` | [-2.2515, 1.5882] |
| 2 | `right_shoulder_yaw_joint` | [-2.618, 2.618] |
| 3 | `right_elbow_joint` | [-1.0472, 2.0944] |
| 4 | `right_wrist_roll_joint` | [-1.97222, 1.97222] |

这是绝对关节角度 [rad]，不是 TCP 坐标，也不是角度制。初始位置为 `[0.0, -0.35, 0.0, 0.0, 0.35]` rad。

每条命令必须包含五个互不重复的正确名称和五个有限、未越界的位置。名称可重新排序，`position[i]` 对应 `name[i]`。缺轴、重复名、未知名、NaN/Inf 或越界会**整条拒绝并保持上一目标**，拒绝原因写入节点日志。输入 header、velocity、effort 不参与位置控制。

```bash
ros2 topic pub --once /arm_position_commands sensor_msgs/msg/JointState \
  "{name: [right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint, right_elbow_joint, right_wrist_roll_joint], position: [0.15, -0.30, 0.10, 0.20, 0.45]}"
```

**单次发送后持续保持目标**，发布端退出后也保持，直到收到新目标或仿真重启。底层通过 MuJoCo 重力补偿 + 逐轴 PD 力矩驱动，不瞬移 qpos。内部参考位置有变化速度限制，输出总力矩有限幅；这些不是实际关节速度的硬上限，也不保证所有目标姿态都无自碰撞。需要判定到位时读取实际关节反馈。

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

手部按名称匹配，支持只更新部分主动关节，其余保持上一目标。未知名称、没有对应位置的项和非有限值忽略；有限目标按执行器范围裁剪。单次发送后持续保持；header、velocity、effort 不参与此接口，数值不是 SDK 的 0..1000 归一化值。接触阻挡时实际位置可以不到目标，应读取反馈。

默认手指位置伺服 kp=30、kv=0.8。臂和手的控制关节集合互斥，但仍存在机械动力学耦合。

## 5. 接触反馈

```bash
ros2 interface show g1_right_arm_reach/msg/ContactArray
ros2 interface show g1_right_arm_reach/msg/Contact
ros2 topic echo /mujoco/contacts --once
ros2 topic hz /mujoco/contacts
```

`ContactArray` 包含 `header` 和 `contacts[]`，`header.frame_id=world`。它是当前快照，不是开始/结束接触事件。

| Contact 字段 | 含义 |
|---|---|
| geom1_id、geom2_id | 当前模型的几何体 ID；更换模型后不能假定不变 |
| geom1_name、geom2_name | 碰撞几何体名称；未命名时回退为 geom_<id> |
| body1_name、body2_name | 几何体所属刚体名称 |
| position | 世界坐标接触点 [m] |
| normal | 世界坐标单位法向，从 geom1 指向 geom2 |
| distance | 有符号分离距离 [m]；负值表示穿透 |
| active | 是否参与 MuJoCo 约束求解 |
| normal_force | 压缩法向力 [N]；未参与求解时为零 |
| wrench.force | 作用在 geom2 上、世界坐标表达的接触力 [N] |
| wrench.torque | 作用在 geom2 上、关于接触点的力矩 [N·m]，不是关于刚体原点 |

geom1 承受反向力/力矩。力来自 `mj_contactForce` 并从接触坐标系转换到世界坐标系。消息包含自接触、地面接触和自定义场景的其他接触；可按几何体/刚体名称过滤，判断受力时结合 active 与 normal_force。几何接近不一定产生承载力。

**收到空数组表示当前无接触；没有发布者或长时间没有消息是另一种状态。** 当前不模拟实体手的触觉阵列，也不将接触数据发布为 `/revo2/tactile_force_summary`。

## 6. 外部 Python 程序示例

先启动默认关节 demo，在已 source 工作空间的 ROS Python 环境运行：

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from g1_right_arm_reach.msg import ContactArray

rclpy.init()
node = Node('my_demo_client')
arm_pub = node.create_publisher(JointState, '/arm_position_commands', 10)
command = JointState()
command.name = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
                'right_shoulder_yaw_joint', 'right_elbow_joint',
                'right_wrist_roll_joint']
command.position = [0.15, -0.30, 0.10, 0.20, 0.45]

# 持续发送也可；相同目标不会重启运动。也可等待 DDS 发现后只发送一次。
timer = node.create_timer(0.05, lambda: arm_pub.publish(command))
latest = {}
state_sub = node.create_subscription(
    JointState, '/joint_states', lambda msg: latest.update(joints=msg), 10)
contact_sub = node.create_subscription(
    ContactArray, '/mujoco/contacts', lambda msg: latest.update(contacts=msg), 10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

手部发布方式相同，改用 `/hand_commands` 和手部的 name/position。TCP 订阅类型为 `geometry_msgs/msg/PoseStamped`。

## 7. 可选原始力矩模式

关闭位置模式实例后启动：

```bash
ros2 launch g1_right_arm_reach mujoco_joint_demo.launch.py \
  arm_control_mode:=torque viewer:=true
```

此时只订阅 `/joint_commands` 的五轴 name + effort [N·m]，不订阅 `/arm_position_commands`。力矩必须完整提供五轴，并以明显高于 10 Hz 的频率刷新，例如 50 Hz；超过 `arm_command_timeout=0.10 s` 后回退纯重力补偿。此原始通道不采用位置模式的 PD、参考速度和总力矩限制，发送端负责力矩限幅与运动控制。手指和观测接口不变。

模式和控制参数在启动时读取，修改后重启生效。同一 ROS 域只启动一个 bridge，不要同时运行旧 TCP/手型控制器向相同输出话题发送命令。

## 8. 验证与实测范围

在默认关节 demo 运行时，另开已 source 的终端：

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /joint_states --once
ros2 topic echo /actual_ee_pose --once
ros2 run g1_right_arm_reach verify_mujoco_interfaces.py
```

验证脚本会移动臂和手：检查五轴到位、六轴到位、单次命令保持、越界命令拒绝、反馈频率和真实手指自接触力，结束后恢复初始臂姿态并张手，不添加场景物体。适用于默认模型/增益，自定义模型需调整容差。

本机无界面实测通过：臂最大误差约 0.00067 rad，手指约 0.00051 rad，三类观测均约 200 Hz；闭手最多产生 6 个自接触，并收到非零力反馈。另有 7 个 Python 接口测试和原有 14 个 C++ 运动学/控制器测试通过。这些验证不代表实体手标定精度，也不代表已验证手与外部物体接触或窗口显示效果。

在仓库根目录运行单元测试（已 source ROS 和本目录的 install）：

```bash
export G1_TEST_URDF_PATH="$PWD/config/models/g1_23dof_revo2.urdf"
colcon test --packages-select g1_right_arm_reach --return-code-on-test-failure
colcon test-result --verbose
```

此处 URDF 仅供保留的 C++ 模型测试使用，不是新关节 demo 的运行输入。新 Python 测试直接加载 MJCF，覆盖位置跟随、非法输入、模式互斥、限幅、接触坐标/力方向和异常重置。

## 9. 保留的旧入口与模型说明

- `mujoco_demo.launch.py`：旧 TCP XYZ + tool-axis roll 控制，内部需要 URDF/Pinocchio，默认启动 reach、hand_controller 和 action_sequencer；不要与新的关节 demo 同时运行。
- 旧 TCP 命令为 `/end_effector_command`，使用本包 EndEffectorCommand；XYZ 相对肩部，roll 为相对标定零姿态的工具轴旋转，和五轴位置命令不同。
- 旧 `/hand_command`、`/action_command` 和 action_cli 由相应旧节点处理；新入口不启动这些节点。`ActionResult.success` 表示动作编排完成，不是物理抓取成功检测。
- `hardware_controllers.launch.py` 和 SDK 适配器代码保留；不是本 demo 的启动步骤，未在本次交付中验证实体设备。
- 本次接口工作未改变手指机械耦合参数，也未修复旧 URDF 的可视化网格引用；新 demo 以所加载 MJCF 为准。
- 默认使用 arm-only 固定基座模型。全身版本还保留额外被动指尖关节，切换模型前应核对自由度和动力学，不能直接假设两版手部行为相同。
