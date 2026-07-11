#
# CMakeLists.txt — G1 Right Arm Reach
#
# Builds the g1_right_arm_reach ROS 2 package:
#   - Core library (robot model + kinematics wrapping Pinocchio)
#   - Controller library (PID)
#   - ROS 2 nodes (reach_node, target_publisher)
#   - GTest-based unit tests
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

# MODULE: Executables — ROS 2 nodes
# ============================================================================

# reach_node: main ROS 2 node that subscribes to a 3D target point,
# runs the IK controller loop, and publishes joint commands / markers.
add_executable(reach_node src/reach_node.cpp)
target_link_libraries(reach_node g1_right_arm_core g1_controllers)
ament_target_dependencies(reach_node rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs)
rosidl_target_interfaces(reach_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# target_publisher_node: testing utility that publishes a marker pose so the
# reach_node has a target to track without needing external input.
add_executable(target_publisher_node src/target_publisher_node.cpp)
ament_target_dependencies(target_publisher_node rclcpp geometry_msgs)

# MODULE: Install
# ============================================================================

install(TARGETS g1_right_arm_core g1_controllers reach_node target_publisher_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

# Install Python scripts (MuJoCo bridge, utilities)
install(PROGRAMS
    scripts/mujoco_bridge_node.py
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
