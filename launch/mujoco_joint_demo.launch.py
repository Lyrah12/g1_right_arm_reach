#!/usr/bin/env python3
"""MJCF-only joint interfaces: no URDF, RViz or external controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("g1_right_arm_reach")
    return LaunchDescription([
        DeclareLaunchArgument("scene_path", default_value=PathJoinSubstitution([
            share, "config", "models", "g1_23dof_revo2_arm_only.xml"])),
        DeclareLaunchArgument("arm_control_mode", default_value="position",
                              choices=["position", "torque"]),
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("viewer_rate", default_value="60.0"),
        Node(
            package="g1_right_arm_reach", executable="mujoco_bridge_node.py",
            name="mujoco_bridge", output="screen",
            parameters=[PathJoinSubstitution([share, "config", "params.yaml"]), {
                "scene_path": LaunchConfiguration("scene_path"),
                "arm_control_mode": LaunchConfiguration("arm_control_mode"),
                "viewer": ParameterValue(LaunchConfiguration("viewer"), value_type=bool),
                "viewer_rate": ParameterValue(
                    LaunchConfiguration("viewer_rate"), value_type=float),
            }],
        ),
    ])
