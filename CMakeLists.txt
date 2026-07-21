#
# CMakeLists.txt — G1 Right Arm + Dexterous Hand Control (Plan C)
#
# Libraries:
#   g1_right_arm_core   — RobotModel + Kinematics (Pinocchio wrapper)
#   g1_controllers      — PIDController (damped pseudoinverse IK)
#   grasp_library       — 15 pre-defined grasp patterns
#
# Nodes:
#   reach_node          — Arm IK (50 Hz, 4-DOF → 3-DOF position)
#   hand_controller_node — Hand control (50 Hz, 23-DOF wrist+fingers)
#   action_sequencer_node — State machine coordinator (50 Hz)
#   target_publisher    — Test utility
#

cmake_minimum_required(VERSION 3.22)
project(g1_right_arm_reach)

# MODULE: Build Configuration
# ============================================================================

# C++17 required for Pinocchio
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release for performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# MODULE: Dependencies
# ============================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# MODULE: Custom Messages and Actions
# ============================================================================

# Generate ROS2 interfaces for vision-grasp communication.
# These are used by reach_node (C++) and mujoco_bridge_node (Python).
rosidl_generate_interfaces(${PROJECT_NAME}
    # --- Messages ---
    "msg/DetectedObject.msg"
    "msg/DetectedObjectArray.msg"
    "msg/ArmStatus.msg"
    "msg/GraspResult.msg"
    "msg/GraspCommand.msg"
    "msg/ActionCommand.msg"
    "msg/HandCommand.msg"
    # --- Actions ---
    "action/GraspObject.action"
    DEPENDENCIES
        std_msgs
        geometry_msgs
        action_msgs
    ADD_LINTER_TESTS
)

# MODULE: Core Library — robot model + kinematics (Pinocchio wrapper)
# ============================================================================

# g1_right_arm_core: provides RobotModel and Kinematics classes used by all
# other targets in this package. Links Pinocchio and Eigen3 for rigid-body
# dynamics and forward kinematics / Jacobian computation.
add_library(g1_right_arm_core SHARED
    src/robot_model.cpp
    src/kinematics.cpp
)
target_include_directories(g1_right_arm_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(g1_right_arm_core
    pinocchio::pinocchio
    Eigen3::Eigen
)
ament_target_dependencies(g1_right_arm_core
    rclcpp
)

# MODULE: Controller Library — PID
# ============================================================================

# g1_controllers: provides PIDController (damped least-squares IK solver).
# Depends on g1_right_arm_core for the robot model and kinematics interface.
add_library(g1_controllers SHARED
    src/pid_controller.cpp
)
target_include_directories(g1_controllers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(g1_controllers
    g1_right_arm_core
    pinocchio::pinocchio
    Eigen3::Eigen
)

# MODULE: Grasp Library — standalone grasp pattern registry
# ============================================================================

# grasp_library: provides 15+ pre-defined grasp patterns for Wuji Hand 2.
# ROS-independent — can be used by any node (hand_controller, action_sequencer,
# or future vision-grasp integration nodes).
add_library(grasp_library SHARED
    src/grasp_library.cpp
)
target_include_directories(grasp_library PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# MODULE: Executables — ROS 2 nodes
# ============================================================================

# reach_node: Arm IK (50 Hz). Subscribes /target_point (3-DOF, shoulder-relative),
# runs damped pseudoinverse IK, publishes /joint_commands (4-DOF arm).
add_executable(reach_node src/reach_node.cpp)
target_link_libraries(reach_node g1_right_arm_core g1_controllers)
ament_target_dependencies(reach_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs)
rosidl_target_interfaces(reach_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# target_publisher_node: Test utility for publishing marker poses.
add_executable(target_publisher_node src/target_publisher_node.cpp)
ament_target_dependencies(target_publisher_node rclcpp geometry_msgs)

# hand_controller_node: Hand module (50 Hz, Plan C). 23-DOF (wrist×3 + fingers×20).
# Wrist: orientation PID + MuJoCo 3×3 Jacobian. Fingers: impedance control + smoothstep.
add_executable(hand_controller_node src/hand_controller_node.cpp)
target_include_directories(hand_controller_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(hand_controller_node grasp_library Eigen3::Eigen)
ament_target_dependencies(hand_controller_node rclcpp std_msgs sensor_msgs geometry_msgs)
rosidl_target_interfaces(hand_controller_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# action_sequencer_node: Action coordinator (50 Hz, Plan C).
# State machine: IDLE → REACHING → SETTLING → GRASPING → DONE.
# Position → /target_point, Orientation+Grasp → /hand_command.
add_executable(action_sequencer_node src/action_sequencer_node.cpp)
target_include_directories(action_sequencer_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(action_sequencer_node Eigen3::Eigen)
ament_target_dependencies(action_sequencer_node rclcpp std_msgs sensor_msgs geometry_msgs)
rosidl_target_interfaces(action_sequencer_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# MODULE: Install
# ============================================================================

install(TARGETS g1_right_arm_core g1_controllers grasp_library
    reach_node target_publisher_node hand_controller_node action_sequencer_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

# Install Python scripts (MuJoCo bridge, CLI, utilities)
install(PROGRAMS
    scripts/mujoco_bridge_node.py
    scripts/action_cli.py
    scripts/setup_wuji_hand.sh
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

# MODULE: Tests
# ============================================================================

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # test_kinematics: validates FK, Jacobian, joint-limit, and pseudoinverse
    # correctness against the G1 URDF model via the core library.
    ament_add_gtest(test_kinematics test/test_kinematics.cpp)
    target_link_libraries(test_kinematics g1_right_arm_core)
    ament_target_dependencies(test_kinematics rclcpp)

    # test_controllers: validates PID convergence, joint-limit enforcement,
    # PID reset, and ControlResult field correctness.
    ament_add_gtest(test_controllers test/test_controllers.cpp)
    target_link_libraries(test_controllers g1_controllers g1_right_arm_core)
    ament_target_dependencies(test_controllers rclcpp)
endif()

ament_package()
