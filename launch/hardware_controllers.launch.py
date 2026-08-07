#!/usr/bin/env python3
"""Physical G1/Revo2 controller launch; all hardware outputs default locked."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    share = FindPackageShare("g1_right_arm_reach").perform(context)
    urdf = LaunchConfiguration("urdf_path").perform(context)
    if not urdf:
        urdf = os.path.join(share, "config", "models", "g1_23dof_revo2.urdf")
    params = os.path.join(share, "config", "params.yaml")
    return [
        # Built only when unitree_sdk2 is discoverable by CMake.
        Node(package="g1_right_arm_reach", executable="g1_arm_sdk_adapter",
             parameters=[{
                 "network_interface": LaunchConfiguration("network_interface"),
                 "enable_hardware_output": ParameterValue(
                     LaunchConfiguration("enable_arm_torque"), value_type=bool),
                 "deadman_required": True,
                 "deadman_timeout": 0.10,
                 # Transport repeats the latest 50 Hz bounded torque at 200 Hz.
                 "control_rate": 200.0,
                 "command_timeout": 0.10,
                 "max_abs_torque": ParameterValue(
                     LaunchConfiguration("max_joint_torque"), value_type=float),
             }], output="screen"),
        Node(package="g1_right_arm_reach", executable="reach_node",
             parameters=[params, {
                 "robot_model.urdf_path": urdf,
                 "safety.enable_torque_output": ParameterValue(
                     LaunchConfiguration("enable_arm_torque"), value_type=bool),
                 "safety.deadman_required": True,
                 "safety.effort_derate": ParameterValue(
                     LaunchConfiguration("effort_derate"), value_type=float),
                 "safety.max_joint_torque": ParameterValue(
                     LaunchConfiguration("max_joint_torque"), value_type=float),
                 "safety.max_task_force": ParameterValue(
                     LaunchConfiguration("max_task_force"), value_type=float),
                 "safety.max_roll_moment": ParameterValue(
                     LaunchConfiguration("max_roll_moment"), value_type=float),
             }], output="screen"),
        # The hand controller publishes both the official JointTrajectory topic
        # and /hand_commands. Select exactly one physical backend below/external.
        Node(package="g1_right_arm_reach", executable="hand_controller_node",
             parameters=[params, {
                 "safety.enable_output": ParameterValue(
                     LaunchConfiguration("enable_hand_output"), value_type=bool),
                 "safety.deadman_required": True,
                 "safety.deadman_timeout": 0.10,
                 "tactile.enable": ParameterValue(
                     LaunchConfiguration("enable_tactile"), value_type=bool),
             }], output="screen"),
        # Exclusive BrainCo SDK backend: position command, state and tactile on
        # one transport handle. Never run it with BrainCo ros2_control hardware.
        Node(package="g1_right_arm_reach", executable="revo2_sdk_adapter",
             condition=IfCondition(LaunchConfiguration("use_revo2_sdk")),
             parameters=[{
                 "port": LaunchConfiguration("revo2_port"),
                 "protocol": LaunchConfiguration("revo2_protocol"),
                 "slave_id": 127,
                 "enable_position_output": ParameterValue(
                     LaunchConfiguration("enable_hand_output"), value_type=bool),
                 "enable_tactile": ParameterValue(
                     LaunchConfiguration("enable_tactile"), value_type=bool),
                 "io_rate": 50.0,
                 "command_timeout": 0.20,
                 "normal_force_full_scale": ParameterValue(
                     LaunchConfiguration("tactile_full_scale"), value_type=float),
             }], output="screen"),
        Node(package="g1_right_arm_reach", executable="action_sequencer_node",
             parameters=[params], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("network_interface", description="Robot Ethernet NIC, e.g. eth0"),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("enable_arm_torque", default_value="false"),
        DeclareLaunchArgument("enable_hand_output", default_value="false"),
        DeclareLaunchArgument("enable_tactile", default_value="false"),
        DeclareLaunchArgument("use_revo2_sdk", default_value="false"),
        # Conservative supported-rig defaults; increase only after staged tests.
        DeclareLaunchArgument("effort_derate", default_value="0.15"),
        DeclareLaunchArgument("max_joint_torque", default_value="3.0"),
        DeclareLaunchArgument("max_task_force", default_value="15.0"),
        DeclareLaunchArgument("max_roll_moment", default_value="1.5"),
        DeclareLaunchArgument("revo2_port", default_value=""),
        DeclareLaunchArgument("revo2_protocol", default_value="auto"),
        DeclareLaunchArgument("tactile_full_scale", default_value="1000.0"),
        OpaqueFunction(function=launch_setup),
    ])
