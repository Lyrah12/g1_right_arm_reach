#!/usr/bin/env python3
"""
MuJoCo physics simulation launch — Plan C: Arm-Hand fully decoupled.

  reach_node (50Hz, 4-DOF)  +  hand_controller (50Hz, 23-DOF)
       │                              │
  /joint_commands (4-DOF)    /hand_commands (23-DOF)
       │                              │
       └──────────┬───────────────────┘
                  ▼
  mujoco_bridge_node (200Hz) — MuJoCo physics + position servo
       │
  /actual_ee_pose, /actual_hand_orientation,
  /mujoco_jacobian, /mujoco_wrist_jacobian, /joint_states

Usage:
  ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
    urdf_path:=${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf \
    scene_path:=${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
    viewer:=true

  urdf_path MUST be the patched URDF (config/g1_29dof_inspire.urdf) — the stock
  G1 URDF has no hand, so Pinocchio gravity FF would be ~0.5 N·m too weak (the
  sim used to hide this by preferring MuJoCo gravity; reach_node now uses
  Pinocchio only, so the URDF must carry the 5010 wrist + Inspire hand).
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
        raise RuntimeError("urdf_path:=/path/to/g1_29dof_rev_1_0.urdf required")
    if not scene_path:
        scene_path = os.path.join(pkg_share, 'config', 'g1_inspire_scene.xml')

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
             name='mujoco_bridge',
             parameters=[os.path.join(pkg_share, 'config', 'params.yaml'),
                         {'scene_path': scene_path,
                          'viewer': viewer.lower() == 'true'}],
             output='screen'),

        Node(package='g1_right_arm_reach', executable='reach_node',
             parameters=[os.path.join(pkg_share, 'config', 'params.yaml'),
                         {'robot_model.urdf_path': urdf_path},
                         {'pid.torque_mode': True},
                         {'pid.gravity_compensation': True},
                         {'manager.control_rate': 50.0}],
             output='screen'),

        Node(package='g1_right_arm_reach', executable='hand_controller_node',
             parameters=[os.path.join(pkg_share, 'config', 'params.yaml'),
                         {'robot_model.urdf_path': urdf_path}],
             output='screen'),

        Node(package='g1_right_arm_reach', executable='action_sequencer_node',
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
