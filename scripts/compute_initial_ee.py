#!/usr/bin/env python3
"""
Compute the end-effector position at the initial neutral configuration (q = [0,0,0,0,0]).

This script helps verify: "where does the right hand start when the robot boots up?"

Usage (run in WSL2):
    python3 scripts/compute_initial_ee.py <path_to_urdf>

Scenarios computed:
  1. Neutral config     — all 5 right-arm joints at zero.
  2. Storage pose       — URDF-specified default joint values.
  3. Mid-range config   — each joint at 50 % of its limit range.
  4. Workspace sampling — 1000 random configs within joint limits.

Dependencies: pinocchio, numpy.
"""

import sys
import pinocchio as pin
import numpy as np


# ============================================================================
# MODULE: find_ee_frame — locate the end-effector frame for a given side
# ============================================================================
def find_ee_frame(model, side="right"):
    """Find the end-effector frame ID for the given side in a Pinocchio model.

    Searches by exact frame name using common G1 naming conventions, then falls
    back to case-insensitive substring matching, and finally to the last frame.

    Args:
        model: A Pinocchio Model object (full or reduced).
        side: Which arm to search for ("right" or "left").

    Returns:
        int: Frame index into model.frames for the end-effector.
    """
    hints = [
        f"{side}_rubber_hand",
        f"{side}_wrist_roll_rubber_hand",
        f"{side}_palm",
        f"{side}_hand",
        f"{side}_gripper",
    ]

    # Exact match first
    for hint in hints:
        for fid in range(model.nframes):
            if model.frames[fid].name == hint:
                return fid

    # Case-insensitive fallback
    for fid in range(model.nframes):
        name = model.frames[fid].name.lower()
        if side in name and "hand" in name:
            return fid

    # Last frame fallback
    print(f"[WARN] Could not find {side} EE frame, using last frame")
    return model.nframes - 1


# ============================================================================
# MODULE: main — load URDF, build reduced model, compute FK for 4 scenarios
# ============================================================================
def main():
    """Load a G1 URDF, reduce it to the 5 right-arm joints, then compute FK
    under four different joint configurations to characterise the workspace.

    The scenarios are:
      1. Neutral    – where is the hand when all 5 arm joints are at zero?
      2. Storage    – where is the hand in the URDF default (storage) pose?
      3. Mid-range  – what position corresponds to the middle of each joint range?
      4. Sampling   – what is the full reachable volume (1000 random configs)?
    """
    if len(sys.argv) < 2:
        print("Usage: python3 compute_initial_ee.py <urdf_path>")
        sys.exit(1)

    urdf_path = sys.argv[1]
    print(f"Loading URDF: {urdf_path}")

    # Load full model
    full_model = pin.buildModelFromUrdf(urdf_path)
    full_data = full_model.createData()
    print(f"Full model: {full_model.nq} pos DOF, {full_model.nv} vel DOF")

    # Identify right-arm joints to keep active
    right_arm_joints = [
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]

    keep_ids = set()
    for name in right_arm_joints:
        if full_model.existJointName(name):
            jid = full_model.getJointId(name)
            keep_ids.add(jid)
            print(f"  Keeping joint: {name} (ID={jid})")
        else:
            print(f"  [WARN] Joint '{name}' not found!")

    # All other joints (except universe=0) to lock
    joints_to_lock = []
    for jid in range(1, full_model.njoints):
        if jid not in keep_ids:
            joints_to_lock.append(jid)

    print(f"  Joints to lock: {len(joints_to_lock)}")
    print(f"  Joints active:  {len(keep_ids)}")

    # Reference config: neutral (all zeros)
    q_ref = pin.neutral(full_model)
    print(f"\nReference config (neutral): q_ref = {q_ref}")

    # Build reduced model
    reduced_model = pin.buildReducedModel(full_model, joints_to_lock, q_ref)
    reduced_data = reduced_model.createData()
    print(f"Reduced model: {reduced_model.nq} pos DOF, {reduced_model.nv} vel DOF")

    # Find EE frame
    ee_fid = find_ee_frame(reduced_model, "right")
    print(f"EE frame: '{reduced_model.frames[ee_fid].name}' (ID={ee_fid})")

    # Joint limits
    print(f"\nJoint limits (reduced model):")
    for i in range(reduced_model.nv):
        name = reduced_model.names[i + 1]  # skip universe
        q_min = reduced_model.lowerPositionLimit[i]
        q_max = reduced_model.upperPositionLimit[i]
        print(f"  [{i}] {name:35s}  [{q_min:+.3f}, {q_max:+.3f}] rad")

    # ========================================================================
    # SCENARIO 1: FK at neutral config (all joints at zero)
    # ========================================================================
    q_neutral = pin.neutral(reduced_model)
    print(f"\n{'='*60}")
    print(f"SCENARIO 1: Neutral config q = {q_neutral} (all zeros)")
    print(f"{'='*60}")
    pin.forwardKinematics(reduced_model, reduced_data, q_neutral)
    pin.updateFramePlacements(reduced_model, reduced_data)
    ee_pose = reduced_data.oMf[ee_fid]
    ee_pos = ee_pose.translation
    print(f"  EE position: x={ee_pos[0]:.4f}, y={ee_pos[1]:.4f}, z={ee_pos[2]:.4f} (meters)")
    print(f"  EE position: x={ee_pos[0]*100:.1f}, y={ee_pos[1]*100:.1f}, z={ee_pos[2]*100:.1f} (cm)")

    # ========================================================================
    # SCENARIO 2: FK at URDF-specified neutral (G1 storage pose)
    # ========================================================================
    # The URDF neutral for right arm joints (from the log):
    # [2.67, -2.25, -2.62, -1.05, 1.97]
    q_storage = np.array([2.67, -2.25, -2.62, -1.05, 1.97])
    print(f"\n{'='*60}")
    print(f"SCENARIO 2: G1 storage pose q = {q_storage}")
    print(f"{'='*60}")
    pin.forwardKinematics(reduced_model, reduced_data, q_storage)
    pin.updateFramePlacements(reduced_model, reduced_data)
    ee_pose = reduced_data.oMf[ee_fid]
    ee_pos = ee_pose.translation
    print(f"  EE position: x={ee_pos[0]:.4f}, y={ee_pos[1]:.4f}, z={ee_pos[2]:.4f} (meters)")
    print(f"  EE position: x={ee_pos[0]*100:.1f}, y={ee_pos[1]*100:.1f}, z={ee_pos[2]*100:.1f} (cm)")

    # ========================================================================
    # SCENARIO 3: FK at mid-range config (each joint at 50 % of its limit range)
    # ========================================================================
    print(f"\n{'='*60}")
    print(f"SCENARIO 3: Mid-range config (all joints at 50% of range)")
    print(f"{'='*60}")
    q_mid = np.zeros(reduced_model.nv)
    for i in range(reduced_model.nv):
        q_min = reduced_model.lowerPositionLimit[i]
        q_max = reduced_model.upperPositionLimit[i]
        q_mid[i] = (q_min + q_max) / 2.0
    print(f"  q_mid = {q_mid}")
    pin.forwardKinematics(reduced_model, reduced_data, q_mid)
    pin.updateFramePlacements(reduced_model, reduced_data)
    ee_pos = reduced_data.oMf[ee_fid].translation
    print(f"  EE position: x={ee_pos[0]:.4f}, y={ee_pos[1]:.4f}, z={ee_pos[2]:.4f} (meters)")
    print(f"  EE position: x={ee_pos[0]*100:.1f}, y={ee_pos[1]*100:.1f}, z={ee_pos[2]*100:.1f} (cm)")

    # ========================================================================
    # SCENARIO 4: Workspace sampling (1000 random configs within joint limits)
    # ========================================================================
    print(f"\n{'='*60}")
    print(f"SCENARIO 4: Workspace sampling (1000 random configs)")
    print(f"{'='*60}")
    np.random.seed(42)
    x_vals, y_vals, z_vals = [], [], []
    for _ in range(1000):
        q_rand = np.zeros(reduced_model.nv)
        for i in range(reduced_model.nv):
            q_min = reduced_model.lowerPositionLimit[i]
            q_max = reduced_model.upperPositionLimit[i]
            q_rand[i] = q_min + np.random.random() * (q_max - q_min)
        pin.forwardKinematics(reduced_model, reduced_data, q_rand)
        pin.updateFramePlacements(reduced_model, reduced_data)
        pos = reduced_data.oMf[ee_fid].translation
        x_vals.append(pos[0])
        y_vals.append(pos[1])
        z_vals.append(pos[2])

    print(f"  X: min={min(x_vals):.3f}, max={max(x_vals):.3f} m  ({min(x_vals)*100:.0f} ~ {max(x_vals)*100:.0f} cm)")
    print(f"  Y: min={min(y_vals):.3f}, max={max(y_vals):.3f} m  ({min(y_vals)*100:.0f} ~ {max(y_vals)*100:.0f} cm)")
    print(f"  Z: min={min(z_vals):.3f}, max={max(z_vals):.3f} m  ({min(z_vals)*100:.0f} ~ {max(z_vals)*100:.0f} cm)")
    print(f"\n  Known test target (0.15, -0.25, 0.15): reached 0.5cm error ✓")
    print(f"  Known test target (0.30, -0.40, 0.20): stuck at 11.5cm   ✗")

if __name__ == "__main__":
    main()
