cmake_minimum_required(VERSION 3.22)
project(g1_right_arm_reach)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unitree_sdk2 QUIET CONFIG)
set(BRAINCO_STARK_SDK_ROOT "" CACHE PATH "BrainCo brainco-hand-sdk root or dist directory")
find_path(BRAINCO_STARK_INCLUDE_DIR stark-sdk.h
  HINTS ${BRAINCO_STARK_SDK_ROOT}
  PATH_SUFFIXES dist/include include)
find_library(BRAINCO_STARK_LIBRARY bc_stark_sdk
  HINTS ${BRAINCO_STARK_SDK_ROOT}
  PATH_SUFFIXES dist/shared/linux shared/linux lib)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EndEffectorCommand.msg"
  "msg/ActionCommand.msg"
  "msg/HandCommand.msg"
  "msg/ActionResult.msg"
  "msg/Contact.msg"
  "msg/ContactArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

add_library(g1_right_arm_core SHARED
  src/robot_model.cpp
  src/kinematics.cpp
)
target_include_directories(g1_right_arm_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(g1_right_arm_core pinocchio::pinocchio Eigen3::Eigen)

add_library(g1_controllers SHARED src/pid_controller.cpp)
target_include_directories(g1_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(g1_controllers g1_right_arm_core Eigen3::Eigen)

add_library(grasp_library SHARED src/grasp_library.cpp)
target_include_directories(grasp_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(reach_node src/reach_node.cpp)
target_link_libraries(reach_node g1_controllers g1_right_arm_core Eigen3::Eigen)
ament_target_dependencies(reach_node rclcpp sensor_msgs geometry_msgs std_msgs std_srvs)
rosidl_target_interfaces(reach_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(hand_controller_node src/hand_controller_node.cpp)
target_link_libraries(hand_controller_node grasp_library Eigen3::Eigen)
target_include_directories(hand_controller_node PRIVATE include)
ament_target_dependencies(hand_controller_node rclcpp sensor_msgs std_msgs trajectory_msgs)
rosidl_target_interfaces(hand_controller_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(action_sequencer_node src/action_sequencer_node.cpp)
target_include_directories(action_sequencer_node PRIVATE include)
ament_target_dependencies(action_sequencer_node rclcpp std_msgs)
rosidl_target_interfaces(action_sequencer_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(target_publisher_node src/target_publisher_node.cpp)
ament_target_dependencies(target_publisher_node rclcpp)
rosidl_target_interfaces(target_publisher_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(action_cli src/action_cli.cpp)
ament_target_dependencies(action_cli rclcpp)
rosidl_target_interfaces(action_cli ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(PROJECT_RUNTIME_TARGETS
  reach_node hand_controller_node action_sequencer_node target_publisher_node action_cli)
if(unitree_sdk2_FOUND AND TARGET unitree_sdk2)
  add_executable(g1_arm_sdk_adapter src/g1_arm_sdk_adapter.cpp)
  ament_target_dependencies(g1_arm_sdk_adapter rclcpp sensor_msgs std_msgs)
  target_link_libraries(g1_arm_sdk_adapter unitree_sdk2)
  list(APPEND PROJECT_RUNTIME_TARGETS g1_arm_sdk_adapter)
  message(STATUS "Building optional Unitree rt/arm_sdk adapter")
else()
  message(WARNING "unitree_sdk2 not found; g1_arm_sdk_adapter will not be built")
endif()
if(BRAINCO_STARK_INCLUDE_DIR AND BRAINCO_STARK_LIBRARY)
  find_package(Threads REQUIRED)
  get_filename_component(BRAINCO_STARK_LIBRARY_DIR ${BRAINCO_STARK_LIBRARY} DIRECTORY)
  add_executable(revo2_sdk_adapter src/revo2_sdk_adapter.cpp)
  target_include_directories(revo2_sdk_adapter PRIVATE ${BRAINCO_STARK_INCLUDE_DIR})
  target_link_libraries(revo2_sdk_adapter
    ${BRAINCO_STARK_LIBRARY} Threads::Threads ${CMAKE_DL_LIBS})
  ament_target_dependencies(revo2_sdk_adapter rclcpp sensor_msgs std_msgs std_srvs)
  set_target_properties(revo2_sdk_adapter PROPERTIES
    INSTALL_RPATH "$ORIGIN;${BRAINCO_STARK_LIBRARY_DIR}")
  list(APPEND PROJECT_RUNTIME_TARGETS revo2_sdk_adapter)
  message(STATUS "Building optional exclusive BrainCo Revo2 SDK backend")
else()
  message(WARNING "BrainCo Stark SDK not found; revo2_sdk_adapter will not be built")
endif()

install(TARGETS
  g1_right_arm_core g1_controllers grasp_library
  ${PROJECT_RUNTIME_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
  scripts/mujoco_bridge_node.py
  scripts/verify_mujoco_interfaces.py
  scripts/compose_g1_23dof_revo2.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/models DESTINATION share/${PROJECT_NAME}/config)
install(FILES config/params.yaml config/rviz_config.rviz
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics g1_right_arm_core)
  ament_add_gtest(test_controllers test/test_controllers.cpp)
  target_link_libraries(test_controllers g1_controllers g1_right_arm_core)
  ament_add_pytest_test(test_mujoco_interfaces test/test_mujoco_interfaces.py
    TIMEOUT 60
    APPEND_ENV
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py")
endif()

ament_export_include_directories(include)
ament_export_libraries(g1_right_arm_core g1_controllers grasp_library)
ament_export_dependencies(pinocchio Eigen3)
ament_package()
