#!/usr/bin/env python3
"""Compare MuJoCo shoulder world position vs the reach_node's Pinocchio+offset
assumption. If they differ, the reach target is misplaced → arm chases wrong point."""
import mujoco
import numpy as np
import sys, os

# --- MuJoCo: shoulder world position at ready pose ---
m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)
def qpa(j): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
d.qpos[qpa("right_elbow_joint")] = 0.3
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)
sh = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_shoulder_pitch_link")
sh_mj = d.xpos[sh].copy()
ee_mj = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")].copy()
print("MuJoCo pelvis xpos     =", np.round(d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")], 4))
print("MuJoCo shoulder xpos   =", np.round(sh_mj, 4))
print("MuJoCo EE (yaw) xpos   =", np.round(ee_mj, 4))

# --- Pinocchio: shoulder position in URDF frame ---
urdf = sys.argv[2]
import pinocchio
model = pinocchio.buildModelFromUrdf(urdf)
data = pinocchio.Data(model)
active = ["right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
          "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]
lock = [jid for jid in range(1, model.njoints) if model.names[jid] not in active]
red = pinocchio.buildReducedModel(model, lock, pinocchio.neutral(model))
rd = pinocchio.Data(red)
q = pinocchio.neutral(red)
q[red.joints[1].idx_q if hasattr(red.joints[1],'idx_q') else 0] = -0.3
# set elbow (joint 4)
# find indices
pidx = {}
for jid in range(1, red.njoints):
    name = red.names[jid]
    if name in active:
        pidx[name] = jid
pinocchio.forwardKinematics(red, rd, q)
pinocchio.updateFramePlacements(red, rd)
def frame_pos(frame_name):
    for fid in range(red.nframes):
        if red.frames[fid].name == frame_name:
            return rd.oMf[fid].translation.copy()
    return None
sh_pin = frame_pos("right_shoulder_pitch_link")
print("\nPinocchio shoulder (URDF frame) =", np.round(sh_pin, 4) if sh_pin is not None else "N/A")

pelvis_offset = np.array([0.0, 0.0, 0.823])
sh_pred = sh_pin + pelvis_offset
print("reach_node's shoulder_world (pin + offset) =", np.round(sh_pred, 4))
print("\nDifference MuJoCo_shoulder vs predicted:", np.round(sh_mj - sh_pred, 4))
print("  magnitude:", round(np.linalg.norm(sh_mj - sh_pred), 4), "m")
