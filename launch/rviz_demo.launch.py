#!/usr/bin/env python3
"""Visualization-only launch for the generated G1-23DOF + Revo2 URDF.

This launch never starts the torque controller. Use mujoco_demo.launch.py for
closed-loop simulation and hardware_controllers.launch.py for physical tests.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    share = FindPackageShare("g1_right_arm_reach").perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    if not urdf_path:
        urdf_path = os.path.join(
            share, "config", "models", "g1_23dof_revo2.urdf")
    with open(urdf_path, encoding="utf-8") as stream:
        robot_description = stream.read()

    use_gui = LaunchConfiguration("use_gui")
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("g1_right_arm_reach"),
                "config",
                "rviz_config.rviz",
            ]),
        ),
        OpaqueFunction(function=launch_setup),
    ])
