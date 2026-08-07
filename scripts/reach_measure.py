#!/usr/bin/env python3
"""Auto-validated reach measurement against the running sim.

Phase 1 (reset): command the arm to a far rest pose and wait until settled.
Phase 2 (measure): command a reach and record position/roll errors plus the
split saturation diagnostics. Spins while publishing so feedback is processed
during the motion. Exits non-zero if the goal is not converged within timeout.

Requires: sim running with reach_node publishing /debug/position_error,
/debug/roll_error and /debug/ctrl_state.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from g1_right_arm_reach.msg import EndEffectorCommand

REST = (0.30, -0.10, 0.00)
GOAL = (0.20, -0.15, -0.15)
POS_TOL = 0.015          # m
ROLL_TOL = 0.06          # rad
TIMEOUT = 12.0           # s wall-clock budget for the measured reach
DWELL = 0.5              # s both errors must continuously remain in tolerance
MEASURE_DURATION = TIMEOUT + DWELL + 1.0


class Measure(Node):
    def __init__(self):
        super().__init__("reach_measure")
        self.t0 = None
        self.errs = []          # (t, pos_err)
        self.rolls = []         # (t, roll_err)
        self.ctrl = []          # (t, linear_vel, roll_vel, fsat, rsa, jlim, slew, alloc_sing)
        self.poses = []         # (t, x, y, z)
        self.cmd_id = 100
        self.pending = None     # (target, roll, until)
        self.pub = self.create_publisher(EndEffectorCommand, "/end_effector_command", 10)
        self.sub_err = self.create_subscription(Float64, "/debug/position_error", self.on_err, 10)
        self.sub_roll = self.create_subscription(Float64, "/debug/roll_error", self.on_roll, 10)
        self.sub_ctrl = self.create_subscription(Float64MultiArray, "/debug/ctrl_state", self.on_ctrl, 10)
        from geometry_msgs.msg import PoseStamped
        self.sub_pose = self.create_subscription(PoseStamped, "/actual_ee_pose", self.on_pose, 10)

    # -- feedback
    def on_err(self, msg):
        if self.t0 is None:
            return
        self.errs.append((time.monotonic() - self.t0, msg.data))

    def on_roll(self, msg):
        if self.t0 is None:
            return
        self.rolls.append((time.monotonic() - self.t0, msg.data))

    def on_ctrl(self, msg):
        if self.t0 is None or len(msg.data) < 9:
            return
        d = msg.data
        self.ctrl.append((time.monotonic() - self.t0, d[0], d[1], d[4], d[5], d[6], d[7], d[8]))

    def on_pose(self, msg):
        if self.t0 is None:
            return
        p = msg.pose.position
        self.poses.append((time.monotonic() - self.t0, p.x, p.y, p.z))

    # -- driving
    def send(self, target, roll, duration):
        self.pending = (target, roll, time.monotonic() + duration)

    def clear_history(self):
        self.t0 = None
        self.errs.clear(); self.rolls.clear(); self.ctrl.clear(); self.poses.clear()

    def spin_pump(self, seconds):
        """Spin while delivering any pending target for its window."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.pending is not None:
                target, roll, until = self.pending
                if time.monotonic() >= until:
                    self.pending = None
                else:
                    msg = EndEffectorCommand()
                    msg.target_x, msg.target_y, msg.target_z = target
                    msg.target_roll = roll
                    self.cmd_id += 1
                    msg.command_id = self.cmd_id
                    self.pub.publish(msg)
            time.sleep(0.005)


def main():
    rclpy.init()
    node = Measure()
    # wait for feedback topics
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pending is None:
            pass
    # Phase 1: reset far away and settle
    node.send(REST, 0.0, 4.0)
    node.spin_pump(10.0)
    # Phase 2: fresh measurement
    node.clear_history()
    node.t0 = time.monotonic()
    node.send(GOAL, 0.0, MEASURE_DURATION)
    node.spin_pump(MEASURE_DURATION)
    rclpy.shutdown()

    if not node.errs or not node.rolls:
        print("FAIL: no feedback captured")
        sys.exit(2)

    # Convergence: the position and roll error streams are independent 50 Hz
    # messages with different timestamps. A dwell is required — the error must
    # stay inside tolerance for DWELL seconds — so the handful of samples
    # recorded while the previous target is still live (arm already near the
    # old goal) cannot be mistaken for convergence of the new one.
    def first_simultaneously_settled(position_rows, roll_rows):
        """Return dwell-completion time for simultaneous position/roll tolerance."""
        events = [(t, "position", e) for t, e in position_rows]
        events += [(t, "roll", e) for t, e in roll_rows]
        position = None
        roll = None
        inside_since = None
        for t, kind, value in sorted(events):
            if kind == "position":
                position = value
            else:
                roll = value
            inside = (position is not None and roll is not None
                      and position <= POS_TOL and abs(roll) <= ROLL_TOL)
            if not inside:
                inside_since = None
                continue
            if inside_since is None:
                inside_since = t
            if t - inside_since >= DWELL:
                return t
        return None

    tc = first_simultaneously_settled(node.errs, node.rolls)
    max_pos = max(e for _, e in node.errs)
    max_roll = max(abs(e) for _, e in node.rolls)
    fsat = sum(1 for s in node.ctrl if s[3] > 0.5)
    jlim = sum(1 for s in node.ctrl if s[5] > 0.5)
    slew = sum(1 for s in node.ctrl if s[6] > 0.5)
    nctrl = max(1, len(node.ctrl))
    alloc_min = min((s[7] for s in node.ctrl), default=1.0)

    print(f"GOAL={GOAL}  t<{POS_TOL*1000:.0f}mm+roll<{ROLL_TOL:.2f}: "
          f"{tc if tc is None else f'{tc:.2f}s'}")
    print(f"max_pos_err={max_pos*1000:.1f}mm  max_roll_err={max_roll:.3f}")
    print(f"ctrl samples={len(node.ctrl)}  force_sat={fsat/nctrl*100:.0f}%  "
          f"joint_lim={jlim/nctrl*100:.0f}%  slew_lim={slew/nctrl*100:.0f}%  "
          f"alloc_min_sing={alloc_min:.4f}")

    # Acceptance is the SETTLED end state, not the initial offset away from the
    # reset pose (which always exceeds tolerance at t=0).
    final_pos = node.errs[-1][1] if node.errs else 1.0
    final_roll = abs(node.rolls[-1][1]) if node.rolls else 1.0
    if (tc is None or tc > TIMEOUT
            or final_pos > POS_TOL * 2 or final_roll > ROLL_TOL * 2):
        print(f"final_pos={final_pos*1000:.1f}mm final_roll={final_roll:.3f} "
              f"settled={tc}")
        print("FAIL: goal not converged")
        sys.exit(1)
    print("PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()
