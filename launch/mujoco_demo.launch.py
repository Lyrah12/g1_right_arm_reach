#!/usr/bin/env python3
"""
MuJoCo physics simulation launch — Arm-Hand decoupled control.

  reach_node (50Hz) ←→ /joint_commands /joint_states ←→ mujoco_bridge_node (200Hz)
       ↑                                                        │
  /target_pose (PoseStamped)                         /actual_ee_pose, /mujoco_jacobian
  (shoulder-relative)                                              │
                                                          RViz (optional)

Usage:
  ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
    urdf_path:=/path/to/g1_23dof_rev_1_0.urdf \
    scene_path:=/path/to/g1_grasp_scene.xml \
    viewer:=true use_test_target:=false
"""
import os, re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    scene_path = LaunchConfiguration('scene_path').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    use_test_target = LaunchConfiguration('use_test_target').perform(context)
    viewer = LaunchConfiguration('viewer').perform(context)
    pkg_share = FindPackageShare('g1_right_arm_reach').perform(context)

    if not urdf_path:
        raise RuntimeError("urdf_path:=/path/to/g1_23dof_rev_1_0.urdf required")
    if not scene_path:
        scene_path = os.path.join(pkg_share, 'config', 'g1_grasp_scene.xml')

    # Load URDF, fix relative mesh paths
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    robot_desc = robot_desc.replace('filename="meshes/', f'filename="file://{urdf_dir}/meshes/')

    LogInfo(msg=f"URDF: {urdf_path}  |  Scene: {scene_path}")

    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}], output='screen'),

        Node(package='g1_right_arm_reach', executable='mujoco_bridge_node.py',
             parameters=[{'scene_path': scene_path, 'physics_rate': 200.0,
                 'camera_rate': 15.0, 'publish_camera': False, 'publish_ft': True,
                 'viewer': viewer.lower() == 'true',
                 'active_joints': ['right_shoulder_pitch_joint',
                     'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
                     'right_elbow_joint', 'right_wrist_roll_joint']}],
             output='screen'),

        Node(package='g1_right_arm_reach', executable='reach_node',
             parameters=[os.path.join(pkg_share, 'config', 'params.yaml'),
                         {'robot_model.urdf_path': urdf_path},
                         {'pid.torque_mode': False},
                         {'pid.gravity_compensation': False},
                         {'manager.control_rate': 50.0}],
             output='screen'),

        Node(package='g1_right_arm_reach', executable='target_publisher_node',
             condition=IfCondition(use_test_target), output='screen'),

        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(use_rviz)),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value='',
            description='Path to G1 URDF'),
        DeclareLaunchArgument('scene_path', default_value='',
            description='Path to MJCF scene (auto-resolved if empty)'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution(
            [FindPackageShare('g1_right_arm_reach'), 'config', 'rviz_config.rviz'])),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_test_target', default_value='false'),
        DeclareLaunchArgument('viewer', default_value='false',
            description='Open MuJoCo interactive viewer'),
        DeclareLaunchArgument('hard_clamp_pelvis', default_value='false',
            description='Hard-clamp pelvis free joint'),
        DeclareLaunchArgument('hard_clamp_joints', default_value='false',
            description='Hard-clamp locked joint qpos'),
        DeclareLaunchArgument('clamp_arm_joints', default_value='false',
            description='Hard-clamp arm joints'),
        OpaqueFunction(function=launch_setup),
    ])
