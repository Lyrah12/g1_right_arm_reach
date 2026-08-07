#!/usr/bin/env python3
"""Roll-tracking validation: publish a sequence of EndEffectorCommand roll steps
and assert each converges (pos <= tol, |roll| <= tol) within a wall-clock budget.

Publishes to /end_effector_command directly (bypasses the action sequencer).
Exits non-zero if any step fails to converge. Requires the sim running.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from g1_right_arm_reach.msg import EndEffectorCommand

STEPS = [
    # (label, x, y, z, roll)
    ("reach-roll0", 0.30, -0.10, 0.00, 0.00),
    ("roll+0.5",    0.30, -0.10, 0.00, 0.50),
    ("roll-0.5",    0.30, -0.10, 0.00, -0.50),
]
POS_TOL = 0.015          # m; match the action sequencer acceptance
ROLL_TOL = 0.06          # rad; match the action sequencer acceptance
STEP_TIMEOUT = 8.0       # s per step
INTERSTEP_HOLD = 0.5     # s continuously inside tolerance


class RollTest(Node):
    def __init__(self):
        super().__init__("roll_step_test")
        self.pos = 999.0
        self.roll = 999.0
        self.cmd_id = 200
        self.pub = self.create_publisher(EndEffectorCommand, "/end_effector_command", 10)
        self.create_subscription(Float64, "/debug/position_error", self.on_pos, 10)
        self.create_subscription(Float64, "/debug/roll_error", self.on_roll, 10)

    def on_pos(self, msg):
        self.pos = msg.data

    def on_roll(self, msg):
        self.roll = msg.data

    def run_step(self, label, x, y, z, roll):
        self.cmd_id += 1
        cid = self.cmd_id
        t0 = time.monotonic()
        settled_since = None
        converged_at = None
        completed = False
        deadline = t0 + STEP_TIMEOUT
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            msg = EndEffectorCommand()
            msg.target_x, msg.target_y, msg.target_z, msg.target_roll = x, y, z, roll
            msg.command_id = cid
            self.pub.publish(msg)
            if self.pos <= POS_TOL and abs(self.roll) <= ROLL_TOL:
                if settled_since is None:
                    settled_since = time.monotonic()
                if time.monotonic() - settled_since >= INTERSTEP_HOLD:
                    converged_at = settled_since - t0
                    completed = True
                    break
            else:
                # A brief tolerance crossing is not convergence. Require one
                # uninterrupted hold interval after the latest excursion.
                settled_since = None
            time.sleep(0.01)
        ok = completed
        self.get_logger().info(
            f"STEP {label}: {'PASS' if ok else 'FAIL'} "
            f"converged@ {converged_at if ok else 'N/A'}s "
            f"pos={self.pos:.4f} roll={self.roll:.4f}")
        return ok


def main():
    rclpy.init()
    node = RollTest()
    # wait for feedback
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pos < 999.0:
            break
    results = [node.run_step(*step) for step in STEPS]
    rclpy.shutdown()
    if not all(results):
        print("FAIL: one or more roll steps did not converge")
        sys.exit(1)
    print("PASS: all roll steps converged")
    sys.exit(0)


if __name__ == "__main__":
    main()
