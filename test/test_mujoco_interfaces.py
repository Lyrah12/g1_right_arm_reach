"""Exercise position control and contact semantics using real MuJoCo physics."""

from pathlib import Path

import mujoco
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from mujoco_bridge_node import MuJoCoBridgeNode


SCENE = Path(__file__).resolve().parents[1] / 'config/models/g1_23dof_revo2_arm_only.xml'


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init(args=[], domain_id=186)
    yield
    rclpy.shutdown()


@pytest.fixture
def bridge():
    node = MuJoCoBridgeNode(parameter_overrides=[
        Parameter('scene_path', value=str(SCENE)),
        Parameter('arm_control_mode', value='position'),
    ])
    yield node
    node.destroy_node()


def position_command(names, values):
    message = JointState()
    message.name = list(names)
    message.position = list(values)
    return message


def test_named_targets_follow_physics_with_independent_hand(bridge):
    target = [0.15, -0.30, 0.10, 0.20, 0.45]
    bridge._arm_position_callback(position_command(bridge.arm_names[::-1], target[::-1]))
    hand = [0.30, 0.20, 0.40, 0.35, 0.30, 0.25]
    bridge._hand_callback(position_command(bridge.hand_names, hand))
    for _ in range(2000):
        bridge._physics_step()
    actual = [float(bridge.data.joint(n).qpos[0]) for n in bridge.arm_names]
    actual_hand = [float(bridge.data.joint(n).qpos[0]) for n in bridge.hand_names]
    np.testing.assert_allclose(actual, target, atol=0.005)
    np.testing.assert_allclose(actual_hand, hand, atol=0.002)
    assert np.isfinite(bridge.data.qvel).all()


def test_invalid_commands_leave_entire_target_unchanged(bridge):
    initial = bridge.arm_position_target.copy()
    cases = [
        position_command(bridge.arm_names[:4], initial[:4]),
        position_command([bridge.arm_names[0]] * 5, initial),
        position_command(bridge.arm_names, [0.0] * 4),
        position_command([], initial),
        position_command(bridge.arm_names, [np.nan, *initial[1:]]),
        position_command(bridge.arm_names, [np.inf, *initial[1:]]),
        position_command(bridge.arm_names, [99.0, *initial[1:]]),
        position_command([*bridge.arm_names[:4], bridge.hand_names[0]], initial),
    ]
    for command in cases:
        bridge._arm_position_callback(command)
        np.testing.assert_array_equal(bridge.arm_position_target, initial)


def test_torque_input_cannot_override_position_mode(bridge):
    command = JointState()
    command.name = bridge.arm_names
    command.effort = [100.0] * 5
    bridge._arm_callback(command)
    assert not bridge.arm_torque
    bridge._apply_commands()
    np.testing.assert_allclose(
        [bridge.data.ctrl[bridge.motor_actuators[n][0]] for n in bridge.arm_names],
        [bridge.gravity_torque[bridge.dof_addresses[n]] for n in bridge.arm_names])


def test_position_controller_bounds_reference_and_torque(bridge):
    initial = bridge.arm_position_reference.copy()
    bridge._arm_position_callback(position_command(
        bridge.arm_names, initial + np.array([0.5, 0.5, 0.5, 0.5, 0.5])))
    bridge._apply_commands()
    assert np.all(np.abs(bridge.arm_position_reference - initial)
                  <= bridge.position_max_velocity * bridge.model.opt.timestep + 1e-12)
    bridge.arm_position_reference = initial + 1.0
    bridge._apply_commands()
    actual = np.array([
        bridge.data.ctrl[bridge.motor_actuators[n][0]] for n in bridge.arm_names])
    assert np.all(np.abs(actual) <= bridge.position_max_torque + 1e-12)
    assert np.any(np.isclose(np.abs(actual), bridge.position_max_torque))


def test_legacy_torque_mode_still_accepts_efforts():
    node = MuJoCoBridgeNode(parameter_overrides=[
        Parameter('scene_path', value=str(SCENE)),
        Parameter('arm_control_mode', value='torque'),
    ])
    try:
        initial = node.arm_position_target.copy()
        node._arm_position_callback(position_command(node.arm_names, initial + 0.1))
        np.testing.assert_array_equal(node.arm_position_target, initial)
        command = JointState()
        command.name = node.arm_names
        command.effort = [0.1, 0.2, 0.3, 0.4, 0.5]
        node._arm_callback(command)
        node._apply_commands()
        np.testing.assert_allclose(
            [node.data.ctrl[node.motor_actuators[n][0]] for n in node.arm_names],
            command.effort)
    finally:
        node.destroy_node()


def test_contact_world_frame_force_and_empty_snapshot(bridge):
    # Analytic floor/ball support verifies both the frame transform and sign.
    # This model exists only in the test; the delivered scene has no objects.
    bridge.model = mujoco.MjModel.from_xml_string('''
        <mujoco><worldbody>
          <geom name="floor" type="plane" size="1 1 .1"/>
          <body name="ball" pos="0 0 .049"><freejoint/>
            <geom name="ball_geom" type="sphere" size=".05" mass="1"/>
          </body>
        </worldbody></mujoco>''')
    bridge.data = mujoco.MjData(bridge.model)
    mujoco.mj_forward(bridge.model, bridge.data)
    stamp = bridge.get_clock().now().to_msg()
    message = bridge._contact_state(stamp)
    assert message.header.frame_id == 'world'
    assert message.header.stamp == stamp
    assert len(message.contacts) == 1
    contact = message.contacts[0]
    assert (contact.geom1_name, contact.geom2_name) == ('floor', 'ball_geom')
    assert (contact.body1_name, contact.body2_name) == ('world', 'ball')
    assert contact.active and contact.distance < 0.0 and contact.normal_force > 0.0
    assert contact.normal.z == pytest.approx(1.0)
    assert contact.wrench.force.z == pytest.approx(contact.normal_force)
    assert contact.wrench.force.x == pytest.approx(0.0)
    assert contact.wrench.force.y == pytest.approx(0.0)
    bridge.data.qpos[2] = 1.0
    mujoco.mj_forward(bridge.model, bridge.data)
    assert bridge._contact_state(stamp).contacts == []


def test_instability_reset_does_not_replay_position_target(bridge):
    initial = bridge.arm_position_target.copy()
    bridge._arm_position_callback(position_command(bridge.arm_names, initial + 0.1))
    bridge._restore_initial_state()
    np.testing.assert_array_equal(bridge.arm_position_target, initial)
    np.testing.assert_array_equal(bridge.arm_position_reference, initial)
    assert not bridge.arm_torque and not bridge.hand_position
