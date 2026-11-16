#!/usr/bin/env python3
"""Verify public ROS interfaces against the default joint demo.

Moves the arm and fingers, checks self-contact, then restores the initial pose.
No scene objects are added. Run in the same ROS domain as the joint demo.
"""

import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from g1_right_arm_reach.msg import ContactArray


ARM = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
       'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint']
HAND = ['right_thumb_proximal_joint', 'right_thumb_metacarpal_joint',
        'right_index_proximal_joint', 'right_middle_proximal_joint',
        'right_ring_proximal_joint', 'right_pinky_proximal_joint']
INITIAL = [0.0, -0.35, 0.0, 0.0, 0.35]


class Verify(Node):
    def __init__(self):
        super().__init__('verify_mujoco_interfaces')
        self.latest = {}
        self.counts = {'state': 0, 'pose': 0, 'contacts': 0}
        self.max_contacts = 0
        self.max_force = 0.0
        self.arm_pub = self.create_publisher(JointState, '/arm_position_commands', 10)
        self.hand_pub = self.create_publisher(JointState, '/hand_commands', 10)
        self.refs = []
        for key, topic, kind in [
            ('state', '/joint_states', JointState),
            ('pose', '/actual_ee_pose', PoseStamped),
            ('contacts', '/mujoco/contacts', ContactArray),
        ]:
            self.refs.append(self.create_subscription(
                kind, topic, lambda msg, key=key: self.receive(key, msg), 10))

    def receive(self, key, message):
        self.latest[key] = message
        self.counts[key] += 1
        if key != 'contacts':
            return
        assert message.header.frame_id == 'world'
        self.max_contacts = max(self.max_contacts, len(message.contacts))
        for value in message.contacts:
            assert value.geom1_name and value.geom2_name
            assert value.body1_name and value.body2_name
            assert value.geom1_id >= 0 and value.geom2_id >= 0
            normal = np.array([value.normal.x, value.normal.y, value.normal.z])
            force = np.array([
                value.wrench.force.x, value.wrench.force.y, value.wrench.force.z])
            assert np.isfinite([value.distance, value.normal_force, *normal, *force]).all()
            assert abs(np.linalg.norm(normal) - 1.0) < 1e-6
            assert abs(float(normal @ force) - value.normal_force) < 1e-6
            self.max_force = max(self.max_force, value.normal_force)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def command(self, arm, hand):
        msg = JointState()
        msg.name, msg.position = ARM[::-1], list(arm)[::-1]
        self.arm_pub.publish(msg)
        msg = JointState()
        msg.name, msg.position = HAND, list(hand)
        self.hand_pub.publish(msg)

    def q(self, names):
        state = self.latest['state']
        return np.array([state.position[state.name.index(name)] for name in names])

    def run(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            self.spin_for(0.05)
            if (len(self.latest) == 3 and self.arm_pub.get_subscription_count()
                    and self.hand_pub.get_subscription_count()):
                break
        else:
            raise RuntimeError('Start mujoco_joint_demo.launch.py in position mode first')
        self.command(INITIAL, [0.0] * 6)
        self.spin_for(2.0)
        assert not self.latest['contacts'].contacts, 'Expected an empty contact snapshot'
        target = [0.15, -0.30, 0.10, 0.20, 0.45]
        hand = [0.30, 0.20, 0.40, 0.35, 0.30, 0.25]
        before = dict(self.counts)
        start = time.monotonic()
        self.command(target, hand)  # Send once to check target latching.
        self.spin_for(3.0)
        elapsed = time.monotonic() - start
        arm_error = float(np.max(np.abs(self.q(ARM) - target)))
        hand_error = float(np.max(np.abs(self.q(HAND) - hand)))
        assert arm_error < 0.02, f'Arm error {arm_error} rad'
        assert hand_error < 0.01, f'Hand error {hand_error} rad'
        state = self.latest['state']
        assert len(state.name) == len(state.position) == len(state.velocity) == len(state.effort)
        assert np.isfinite([*state.position, *state.velocity, *state.effort]).all()
        rates = {key: (self.counts[key] - before[key]) / elapsed for key in before}
        assert all(rate > 100.0 for rate in rates.values()), rates
        self.command([99.0, *target[1:]], hand)
        self.spin_for(0.5)
        assert np.max(np.abs(self.q(ARM) - target)) < 0.02, 'Invalid target accepted'
        self.max_contacts, self.max_force = 0, 0.0
        self.command(target, [0.95, 1.4, 1.3, 1.3, 1.3, 1.3])
        self.spin_for(2.0)
        assert self.max_contacts > 0 and self.max_force > 0.0, 'No self-contact feedback'
        assert np.max(np.abs(self.q(ARM) - target)) < 0.02, 'Arm did not hold during hand closure'
        assert self.latest['pose'].header.frame_id == 'world'
        print(json.dumps({
            'result': 'PASS', 'arm_max_error_rad': arm_error,
            'hand_max_error_rad': hand_error, 'observation_rates_hz': rates,
            'max_self_contacts': self.max_contacts,
            'max_normal_force_N': self.max_force,
            'out_of_range_rejected': True, 'one_shot_targets_held': True,
        }, indent=2), flush=True)


def main():
    rclpy.init()
    node = Verify()
    try:
        node.run()
    finally:
        if node.arm_pub.get_subscription_count() and node.hand_pub.get_subscription_count():
            node.command(INITIAL, [0.0] * 6)
            node.spin_for(2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
