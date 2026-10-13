# G1 23DOF 右臂 + BrainCo Revo2 触觉手

面向宇树 **G1 23DOF 教育版**与 BrainCo 强脑 **Revo2 触觉灵巧手**的 ROS 2 控制包。项目使用右肩三轴、右肘和右腕 roll 共五个关节，以力矩控制 TCP 空间位置 `[x, y, z]` 与工具轴 roll；Revo2 六个主动手指关节使用独立的位置控制。

## 功能

- TCP 三维位置 + 工具轴 roll 的任务空间力矩控制；
- Revo2 六轴位置＋速度控制及 `reach_and_grasp` 动作编排；
- 速度、加速度、任务力、关节力矩和变化率多层限幅；
- MuJoCo/实机共用 C++ 控制律，臂与手拥有互斥的关节集合；
- 独立 deadman、状态超时保护及可选触觉抓稳闭环。

roll 定义为“TCP 绕局部工具轴、相对标定零姿态的空间旋转”。默认工具轴为 Revo2 局部 `+Z`。

## 架构

| 模块 | 频率 | 输出 |
|---|---:|---|
| `reach_node` | 50 Hz | 右臂五轴力矩 |
| `hand_controller_node` | 50 Hz | Revo2 六轴位置目标 |
| `action_sequencer_node` | 50 Hz | 臂/手动作编排 |
| `g1_arm_sdk_adapter` | 200 Hz | Unitree `rt/arm_sdk` |
| `revo2_sdk_adapter` | 50 Hz | BrainCo SDK 状态、位置＋速度与触觉 I/O |
| MuJoCo bridge | 500 Hz 物理 / 200 Hz 状态 | 仿真状态与执行器 |

臂控制器只拥有 `right_shoulder_pitch/roll/yaw`、`right_elbow` 和 `right_wrist_roll`；手控制器只拥有 Revo2 六个手指关节。BrainCo SDK 与 BrainCo `ros2_control` 后端互斥，同一时刻只能启用一个。

右臂控制律为：

```text
tau = J_alloc^T * wrench + gravity + N * (tau_posture - D * dq)
```

`J_alloc` 将位置力分配给肩、肘，将 roll 力矩分配给腕 roll；任务速度仍由完整 Jacobian `J·dq` 计算。TASK 阻尼只作用于任务零空间，HOLD 保留全关节阻尼。

## 环境

Ubuntu 24.04 / WSL2、ROS 2 Jazzy、C++17、Eigen3、Pinocchio 和 MuJoCo。实机适配器另外需要 `unitree_sdk2` 与 BrainCo `bc_stark_sdk`。

## 快速开始

将仓库放入 ROS 2 工作空间：

```bash
cd ~/g1_ws/src
git clone YOUR_REPOSITORY_URL g1_right_arm_reach

cd ~/g1_ws
set +u
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select g1_right_arm_reach \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source ~/g1_ws/install/setup.bash
```

`set +u` 可避免某些 shell 配置触发 `AMENT_TRACE_SETUP_FILES: unbound variable`。Windows 源码建议用 `rsync -a --exclude='.git/'` 同步到 WSL。

启动 MuJoCo：

```bash
ros2 launch g1_right_arm_reach mujoco_demo.launch.py viewer:=true
```

另开一个已 source 工作空间的终端发送动作：

```bash
ros2 run g1_right_arm_reach action_cli reach \
  0.30 -0.10 0.00 0.30

ros2 run g1_right_arm_reach action_cli reach_and_grasp \
  0.30 -0.10 0.00 0.30 cable_pinch 0.8
```

内置手型见 [`src/grasp_library.cpp`](src/grasp_library.cpp)，电缆抓取建议从 `cable_pinch` 开始。`reach_and_grasp` 仅在位置和 roll 同时稳定进入容差后执行手指动作；8 秒未收敛会安全失败。

## 验证

```bash
cd ~/g1_ws
set +u
source /opt/ros/jazzy/setup.bash
bash src/g1_right_arm_reach/scripts/validate_wsl.sh

# 仿真运行时，在另一个终端执行
python3 src/g1_right_arm_reach/scripts/reach_measure.py
python3 src/g1_right_arm_reach/scripts/roll_step_test.py
python3 src/g1_right_arm_reach/scripts/validate_reach.py
```

全部命令返回零才算通过。动作门槛为 `15 mm / 0.06 rad`，单次动作总时长不得超过 5 秒。

## 配置

主要参数位于 [`config/params.yaml`](config/params.yaml)。MuJoCo launch 会使用仅限仿真的快速覆盖值；硬件 launch 保留更保守的默认值：

| 参数 | MuJoCo | 实机 |
|---|---:|---:|
| 最大任务力 / 单关节力矩 | `25 N` / `20 N·m` | `15 N` / `3 N·m` |
| 力矩变化率 | `200 N·m/s` | `80 N·m/s` |
| effort derate | `0.60` | `0.15` |
| 最大线速度 / roll 速度 | `0.30 m/s` / `1.00 rad/s` | `0.15 m/s` / `0.50 rad/s` |

不要将仿真快速参数直接复制到实机。调参时应同时检查 `CTRL_DIAG` 中的 `fsat`、`rsa`、`jlim`、`slew` 和 `alloc_sing`。

### 灵巧手响应速度

手指插值倍率 `hand_controller.motion.speed_scale` 默认为 `2.0`，目标插值时间为 `grasp_duration / speed_scale`；例如 `0.8 s` 的动作在 `0.4 s` 内发送完整目标。动作编排仍保持原始 `grasp_duration` 的命令刷新窗口，给设备跟随目标留出时间。这个窗口结束仅表示编排完成，实际到位时间还受设备速度、负载和接触影响。`speed_scale` 必须为有限正数，`1.0` 恢复原插值速度。

SDK 后端使用 `stark_set_finger_positions_and_speeds` 同时发送六轴位置与速度。`revo2_sdk_adapter.finger_speeds` 是六个整数，范围为 `1..1000`，默认全为 `1000`。位置按关节范围换算为 `0..1000`；速度采用 SDK 归一化单位，`1000` 为最大速度指令，开合方向由目标位置决定。这里的速度值不是 `rad/s`，也不是持续运动的独立速度模式。`/hand_commands.velocity` 不作为该归一化速度参数。

启动时可分别调节插值倍率和 SDK 速度（以下硬件命令保留输出锁定）：

```bash
ros2 launch g1_right_arm_reach mujoco_demo.launch.py hand_speed_scale:=2.0

ros2 launch g1_right_arm_reach hardware_controllers.launch.py \
  network_interface:=eth0 use_revo2_sdk:=true \
  hand_speed_scale:=2.0 \
  hand_finger_speeds:='[1000, 1000, 1000, 1000, 1000, 1000]'
```

六轴顺序为拇指屈伸、拇指内收外展、食指、中指、无名指、小拇指。`hand_finger_speeds` 只作用于 SDK 后端；外部 `ros2_control` 后端仍接收位置轨迹。速度参数在节点启动时读取，修改后需重启。触觉闭环在接近过程中保留最终抓取目标，过力时只停止收紧并回退对应手指；完成插值后才根据触觉继续微调抓稳。

可视化仿真由 MuJoCo 位置伺服驱动，SDK 的 `hand_finger_speeds` 不控制仿真手指。仿真六轴使用 `kp=30`、`kv=0.8`，手指关节默认 `damping=0.05`、`armature=0.001`、`frictionloss=0.01`；这些值也写入模型生成脚本，重新生成模型后仍然生效。

窗口默认按 `viewer_rate:=60.0` 同步，物理仍按 500 Hz 推进；新版 MuJoCo 使用 [仅同步状态的接口](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer)，旧版自动使用完整同步。`SIM_DIAG` 新增 `rtf`（仿真时间/墙钟时间）、`physics_hz` 和平均 `viewer_sync_ms`。正常 `rtf` 应接近 `1.0`；如果明显偏低，实际看到的臂和手都会变慢，可以降低 `viewer_rate` 并检查运行负载。

本机诊断中，旧版每个物理步完整同步窗口时 `rtf=0.188`，修改后约 `1.000`。以 0.4 秒线性插值驱动无接触单指到 0.7 rad，并要求误差小于 0.02 rad、速度小于 0.05 rad/s，旧参数稳定到位约需 1.6 秒仿真时间，新参数约需 0.5 秒。这些数值用于对比本机仿真性能，接触抓取仍可能因碰撞无法到达空载目标位置。

## 主要接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/action_command` | `ActionCommand` | 组合动作请求 |
| `/end_effector_command` | `EndEffectorCommand` | TCP XYZ + roll |
| `/hand_command` | `HandCommand` | 手型或六轴自定义位置 |
| `/arm_deadman` | `std_msgs/Bool` | 右臂输出许可 |
| `/hand_deadman` | `std_msgs/Bool` | 手部输出许可 |
| `/debug/ctrl_state` | `Float64MultiArray` | 控制速度和限幅诊断 |


## 参考资料

- [宇树 G1 开发文档](https://support.unitree.com/home/zh/G1_developer/about_G1)
- [BrainCo Revo2 参数](https://www.brainco-hz.com/docs/revolimb-hand/revo2/parameters.html)
- [BrainCo Revo2 C/C++ SDK 位置＋速度接口](https://www.brainco-hz.com/docs/revolimb-hand/revo2/c_sdk.html)
