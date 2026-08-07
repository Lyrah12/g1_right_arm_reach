#!/usr/bin/env python3
"""G1 23DOF + Revo2 decoupled MuJoCo demo."""

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
    scene = LaunchConfiguration("scene_path").perform(context)
    if not urdf:
        urdf = os.path.join(share, "config", "models", "g1_23dof_revo2.urdf")
    if not scene:
        scene = os.path.join(share, "config", "models", "g1_23dof_revo2_arm_only.xml")
    params = os.path.join(share, "config", "params.yaml")
    with open(urdf, encoding="utf-8") as stream:
        robot_description = stream.read()

    # Simulation-only fast-tuning knobs. Defaults match params.yaml (the
    # conservative hardware default); pass e.g. max_force:=20 to speed up the
    # sim WITHOUT touching the hardware defaults.
    fast = {
        "safety.max_task_force": ParameterValue(LaunchConfiguration("max_force"), value_type=float),
        "safety.max_torque_slew_rate": ParameterValue(LaunchConfiguration("max_slew"), value_type=float),
        "safety.effort_derate": ParameterValue(LaunchConfiguration("effort_derate"), value_type=float),
        "trajectory.max_linear_velocity": ParameterValue(LaunchConfiguration("max_vel"), value_type=float),
        "trajectory.max_linear_acceleration": ParameterValue(LaunchConfiguration("max_acc"), value_type=float),
        "trajectory.max_roll_velocity": ParameterValue(LaunchConfiguration("max_roll_vel"), value_type=float),
        "trajectory.max_roll_acceleration": ParameterValue(LaunchConfiguration("max_roll_acc"), value_type=float),
    }

    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="g1_right_arm_reach", executable="mujoco_bridge_node.py",
             name="mujoco_bridge", parameters=[params, {
                 "scene_path": scene,
                 "viewer": ParameterValue(LaunchConfiguration("viewer"), value_type=bool),
             }], output="screen"),
        Node(package="g1_right_arm_reach", executable="reach_node",
             parameters=[params, {"robot_model.urdf_path": urdf,
                                  "task_pid.use_external_gravity": True,
                                  "safety.enable_torque_output": True},
                         fast], output="screen"),
        Node(package="g1_right_arm_reach", executable="hand_controller_node",
             parameters=[params, {"safety.enable_output": True}], output="screen"),
        Node(package="g1_right_arm_reach", executable="action_sequencer_node",
             parameters=[params], output="screen"),
        Node(package="g1_right_arm_reach", executable="target_publisher_node",
             condition=IfCondition(LaunchConfiguration("use_test_target")), output="screen"),
        Node(package="rviz2", executable="rviz2",
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("scene_path", default_value=""),
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_test_target", default_value="false"),
        # Simulation-only fast defaults (validated 2026-08-07: reach 2.4s ->
        # 1.9s end-to-end, zero force/joint saturation, no roll limit cycle).
        # params.yaml + reach_node defaults stay conservative for hardware.
        DeclareLaunchArgument("max_force", default_value="25.0"),
        DeclareLaunchArgument("max_slew", default_value="200.0"),
        DeclareLaunchArgument("effort_derate", default_value="0.60"),
        DeclareLaunchArgument("max_vel", default_value="0.30"),
        DeclareLaunchArgument("max_acc", default_value="1.00"),
        DeclareLaunchArgument("max_roll_vel", default_value="1.00"),
        DeclareLaunchArgument("max_roll_acc", default_value="4.00"),
        OpaqueFunction(function=launch_setup),
    ])
