#!/usr/bin/env python3
"""
RViz kinematic demo (no physics) — Arm-Hand decoupled IK visualization.

Usage:
  ros2 launch g1_right_arm_reach rviz_demo.launch.py \
    urdf_path:=/path/to/g1_23dof_rev_1_0.urdf
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    use_gui = LaunchConfiguration('use_gui').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    pkg_share = FindPackageShare('g1_right_arm_reach').perform(context)

    if not urdf_path:
        raise RuntimeError("urdf_path:=/path/to/g1_23dof_rev_1_0.urdf required")

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    robot_desc = robot_desc.replace('filename="meshes/', f'filename="file://{urdf_dir}/meshes/')

    print(f"[INFO] URDF: {urdf_path} ({len(robot_desc)} bytes)")

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}], output='screen'),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(use_gui)),

        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'source_list': ['/joint_commands'], 'use_sim_time': False}],
             output='screen'),

        Node(package='g1_right_arm_reach', executable='reach_node',
             parameters=[os.path.join(pkg_share, 'config', 'params.yaml'),
                         {'robot_model.urdf_path': urdf_path},
                         {'pid.torque_mode': False},
                         {'pid.gravity_compensation': False},
                         {'manager.control_rate': 50.0}],
             output='screen'),

        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value='',
            description='Path to G1 URDF'),
        DeclareLaunchArgument('use_gui', default_value='false',
            description='Enable joint_state_publisher_gui'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution(
            [FindPackageShare('g1_right_arm_reach'), 'config', 'rviz_config.rviz'])),
        OpaqueFunction(function=launch_setup),
    ])
