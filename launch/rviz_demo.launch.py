#!/usr/bin/env python3
"""
Launch file for G1 right arm reaching demo in RViz (Phase 1: Kinematic).

Architecture:
  reach_node  ──► /joint_commands ──► joint_state_publisher ──► /joint_states /tf
                                                                       │
                                                                    RViz

By default, joint_state_publisher_gui is disabled (use use_gui:=true to enable).
When disabled, a regular joint_state_publisher reads from /joint_commands
to publish the TF tree, forming a closed kinematic loop.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ============================================================================
# MODULE: launch_setup — resolve config, load URDF, build the 5-node graph
# ============================================================================
def launch_setup(context, *args, **kwargs):
    """OpaqueFunction that constructs the node list for the launch description.

    Called by ROS2 launch system after all LaunchConfigurations are resolved
    (except substitutions, which are only evaluated at runtime).

    Returns:
        list[Node]: the 5 nodes forming the kinematic closed-loop demo.
    """
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    use_gui = LaunchConfiguration('use_gui').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    pkg_share = FindPackageShare('g1_right_arm_reach').perform(context)

    if not urdf_path:
        raise RuntimeError("URDF path not set! Use urdf_path:=/path/to/...urdf")

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    print(f"[INFO] Loaded URDF from: {urdf_path} ({len(robot_description)} bytes)")

    nodes = []

    # 1. robot_state_publisher — publishes /tf from full URDF + /joint_states
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    ))

    # 2. joint_state_publisher_gui — optional manual joint-slider GUI
    nodes.append(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
    ))

    # 3. joint_state_publisher — bridges /joint_commands → /joint_states → /tf
    #    This closes the loop: controller output drives the visualization.
    nodes.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/joint_commands'],
            'use_sim_time': False,
        }],
        output='screen',
    ))

    # 4. reach_node — IK + PID joint-space controller
    params_path = os.path.join(pkg_share, 'config', 'params.yaml')
    nodes.append(Node(
        package='g1_right_arm_reach',
        executable='reach_node',
        name='reach_node',
        parameters=[params_path, {'robot_model.urdf_path': urdf_path}],
        output='screen',
    ))

    # 5. rviz2 — 3D visualisation of the robot and TF tree
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    ))

    return nodes


# ============================================================================
# MODULE: generate_launch_description — declare arguments and return LaunchDescription
# ============================================================================
def generate_launch_description():
    """Top-level entry point: declare launch arguments and return the final LaunchDescription."""
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path', default_value='',
        description='Path to G1 23DOF URDF file'
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='false',
        description='Enable joint_state_publisher_gui for manual joint testing'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('g1_right_arm_reach'), 'config', 'rviz_config.rviz']
        ),
        description='Path to RViz config'
    )

    return LaunchDescription([
        urdf_path_arg,
        use_gui_arg,
        rviz_config_arg,
        OpaqueFunction(function=launch_setup),
    ])
