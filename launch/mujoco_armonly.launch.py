#!/usr/bin/env python3
"""Minimal launch: bridge + reach_node ONLY (no hand_controller, no sequencer).
Isolates arm gravity-hold from wrist/finger controllers."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    scene_path = LaunchConfiguration('scene_path').perform(context)
    pkg_share = FindPackageShare('g1_right_arm_reach').perform(context)

    if not scene_path:
        scene_path = os.path.join(pkg_share, 'config', 'g1_inspire_scene.xml')

    return [
        Node(package='g1_right_arm_reach', executable='mujoco_bridge_node.py',
             name='mujoco_bridge',
             parameters=[os.path.join(pkg_share, 'config', 'params.yaml'),
                         {'scene_path': scene_path, 'viewer': False}],
             output='screen'),
        Node(package='g1_right_arm_reach', executable='reach_node',
             parameters=[os.path.join(pkg_share, 'config', 'params.yaml'),
                         {'robot_model.urdf_path': urdf_path},
                         {'pid.torque_mode': True},
                         {'pid.gravity_compensation': True},
                         {'manager.control_rate': 50.0}],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=''),
        DeclareLaunchArgument('scene_path', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
