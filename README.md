# G1 Right Arm Reach Controller

ROS2 Jazzy C++ package for controlling the Unitree G1 humanoid robot's right arm to reach a specified (x, y, z) target point.

## Architecture

```
Target (x,y,z) → [reach_node] → [PID Controller] → Joint Commands
```

- **PID**: Resolved Motion Rate Control with damped least-squares pseudoinverse, variable gain, and null-space joint-limit avoidance. The sole controller — no abstraction layers, no MPC.

## Prerequisites

- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- C++17 compiler (GCC 13+)

### Install Dependencies

```bash
# ROS2 Jazzy (binary install)
# See: https://docs.ros.org/en/jazzy/Installation.html

# Pinocchio
sudo apt install ros-jazzy-pinocchio

# Eigen3
sudo apt install libeigen3-dev

# RViz + visualization tools
sudo apt install ros-jazzy-rviz2 ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui
```

### Get G1 URDF Model

```bash
cd ~/deps
git clone https://github.com/unitreerobotics/unitree_ros.git
# The URDF is at: unitree_ros/robots/g1_description/
# For 23-DOF EDU version: urdf/g1_23dof_rev_1_0.urdf
```

## Build

```bash
mkdir -p ~/g1_ws/src
cd ~/g1_ws/src
git clone <repo-url> g1_right_arm_reach

cd ~/g1_ws
colcon build --packages-select g1_right_arm_reach --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run (Phase 1: RViz Kinematic Demo)

```bash
# Terminal 1: Launch RViz + reach_node
ros2 launch g1_right_arm_reach rviz_demo.launch.py \
    urdf_path:=/path/to/g1_23dof_rev_1_0.urdf

# Terminal 2: Send test targets
ros2 run g1_right_arm_reach target_publisher_node

# Or send a single target:
ros2 topic pub /target_point geometry_msgs/msg/Point "{x: 0.30, y: -0.40, z: 0.20}"
```

## Monitor

```bash
# Watch position error
ros2 topic echo /debug/position_error

# Watch controller status
ros2 topic echo /debug/controller_status
```

## Tuning

Parameters are in `config/params.yaml`. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pid.kp` | 2.5 | PID proportional gain |
| `pid.ki` | 3.0 | PID integral gain |
| `pid.kd` | 0.05 | PID derivative gain |
| `pid.damping` | 1e-4 | Jacobian damping (higher = more stable, less accurate) |
| `pid.tolerance` | 0.003 | Convergence tolerance (meters) |

## Run Tests

```bash
export G1_TEST_URDF_PATH=/path/to/g1_23dof_rev_1_0.urdf
colcon test --packages-select g1_right_arm_reach
colcon test-result --verbose
```

## Project Structure

```
.
├── include/g1_right_arm_reach/
│   ├── robot_model.hpp       # URDF loading + buildReducedModel
│   ├── kinematics.hpp        # FK + Jacobian + pseudoinverse
│   └── pid_controller.hpp    # PID IK controller + ControlResult struct
├── src/
│   ├── robot_model.cpp       # URDF, reduced model, FK
│   ├── kinematics.cpp        # Jacobian, pseudoinverse, integration
│   ├── pid_controller.cpp    # PID IK implementation
│   ├── reach_node.cpp        # ROS2 main node
│   └── target_publisher_node.cpp  # Test target publisher
├── scripts/                  # Utility scripts (compute EE, plot error)
├── launch/                   # ROS2 launch files
├── config/                   # YAML params + RViz config
├── test/                     # GTest unit tests
├── CMakeLists.txt            # Build configuration
├── package.xml               # ROS2 package manifest
└── README.md
```

## License

BSD-3-Clause
