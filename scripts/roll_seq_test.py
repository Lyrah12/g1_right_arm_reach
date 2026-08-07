#!/usr/bin/env python3
"""Deterministic reach-then-roll sequence. Repeats each command to defeat DDS race."""
import rclpy, time
from rclpy.node import Node
from g1_right_arm_reach.msg import EndEffectorCommand

STEPS = [
    # (label, x, y, z, roll, duration_s)
    ("reach-roll0",   0.30, -0.10, 0.00,  0.00, 18.0),
    ("roll+0.5",      0.30, -0.10, 0.00,  0.50, 14.0),
    ("roll-0.5",      0.30, -0.10, 0.00, -0.50, 14.0),
]

class Seq(Node):
    def __init__(self):
        super().__init__("roll_seq_test")
        self.pub = self.create_publisher(EndEffectorCommand, "/end_effector_command", 10)

    def send(self, label, x, y, z, roll, cid, dur):
        msg = EndEffectorCommand()
        msg.target_x, msg.target_y, msg.target_z, msg.target_roll = x, y, z, roll
        msg.command_id = cid
        deadline = time.time() + dur
        n = 0
        while time.time() < deadline:
            self.pub.publish(msg)
            n += 1
            time.sleep(0.1)
        self.get_logger().info(f"DONE {label} roll={roll} ({n} pubs)")

def main():
    rclpy.init()
    node = Seq()
    for cid, (label, x, y, z, roll, dur) in enumerate(STEPS):
        node.send(label, x, y, z, roll, cid, dur)
    node.get_logger().info("SEQ DONE")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
