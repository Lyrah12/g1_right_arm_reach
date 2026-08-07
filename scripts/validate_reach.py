#!/usr/bin/env python3
"""End-to-end reach validation through the action sequencer.

Publishes an /action_command (reach) with repeat-delivery to defeat the DDS
discovery race, then waits for /action_result and asserts SUCCESS. Prints the
measured duration and exits non-zero on timeout or failure.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from g1_right_arm_reach.msg import ActionCommand, ActionResult

# (label, x, y, z, roll, convergence_timeout, settle_time, max_duration)
TASKS = [
    ("reach-0",  0.20, -0.15, -0.15, 0.0, 8.0, 0.2, 5.0),
    ("roll+0.5", 0.20, -0.15, -0.15, 0.5, 8.0, 0.2, 5.0),
]


class Validate(Node):
    def __init__(self):
        super().__init__("validate_reach")
        self.result = None
        self.action_id = 0
        self.pub = self.create_publisher(ActionCommand, "/action_command", 10)
        self.sub = self.create_subscription(ActionResult, "/action_result", self.on_result, 10)

    def on_result(self, msg):
        self.result = msg

    def run(self, label, x, y, z, roll, timeout, settle, max_duration):
        self.result = None
        self.action_id += 1
        cid = self.action_id
        msg = ActionCommand()
        msg.action_type = "reach"
        msg.action_id = cid
        msg.target_x, msg.target_y, msg.target_z, msg.target_roll = x, y, z, roll
        msg.grasp_type = "none"
        msg.grasp_duration = 0.0
        msg.convergence_timeout = timeout
        msg.settle_time = settle

        t0 = time.monotonic()
        deadline = t0 + timeout + settle + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            # repeat-publish to defeat DDS discovery race
            self.pub.publish(msg)
            if self.result is not None and self.result.action_id == cid:
                break
            time.sleep(0.02)
        r = self.result
        if r is None:
            self.get_logger().info(f"TASK {label}: FAIL (no result)")
            return False, None
        ok = r.success and r.duration_s <= max_duration
        reason = r.reason
        if r.success and r.duration_s > max_duration:
            reason = f"duration {r.duration_s:.2f}s exceeds {max_duration:.2f}s budget"
        self.get_logger().info(
            f"TASK {label}: {'PASS' if ok else 'FAIL'} reason={reason} "
            f"pos={r.position_error_m*1000:.1f}mm roll={r.orientation_error_rad:.3f} "
            f"dur={r.duration_s:.2f}s")
        return ok, r.duration_s


def main():
    rclpy.init()
    node = Validate()
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
    results = []
    for task in TASKS:
        ok, dur = node.run(*task)
        results.append(ok)
    rclpy.shutdown()
    if not all(results):
        print("FAIL: one or more actions did not succeed")
        sys.exit(1)
    print("PASS: all actions succeeded")
    sys.exit(0)


if __name__ == "__main__":
    main()
