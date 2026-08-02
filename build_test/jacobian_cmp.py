#!/usr/bin/env python3
"""Compare the Pinocchio (URDF) arm Jacobian vs MuJoCo (MJCF) Jacobian at the
same pose. If signs differ, the controller drives the arm the wrong way."""
import mujoco, numpy as np, sys, pinocchio

# ---- MuJoCo ----
m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)
def qpa(j): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
arm_j = ["right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
         "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]
for j,q in {"right_shoulder_pitch_joint":-0.3,"right_elbow_joint":0.3}.items():
    d.qpos[qpa(j)] = q
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)
ee_b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")
J_mj = np.zeros((3, 4))
eps = 1e-4
for k, j in enumerate(arm_j[:4]):
    orig = d.qpos[qpa(j)]
    d.qpos[qpa(j)] = orig + eps; mujoco.mj_forward(m, d); pp = d.xpos[ee_b].copy()
    d.qpos[qpa(j)] = orig - eps; mujoco.mj_forward(m, d); pm = d.xpos[ee_b].copy()
    d.qpos[qpa(j)] = orig
    J_mj[:, k] = (pp - pm) / (2*eps)
mujoco.mj_forward(m, d)
print("MuJoCo J_arm (3x4):")
print(np.round(J_mj, 4))

# ---- Pinocchio (same as reach_node's reduced model) ----
urdf = sys.argv[2]
full = pinocchio.buildModelFromUrdf(urdf)
lock = [jid for jid in range(1, full.njoints) if full.names[jid] not in arm_j]
red = pinocchio.buildReducedModel(full, lock, pinocchio.neutral(full))
rd = pinocchio.Data(red)
q = pinocchio.neutral(red)
# set sp and elbow to match
def jq(name, val):
    jid = red.getJointId(name)
    q[red.joints[jid].idx_q] = val
jq("right_shoulder_pitch_joint", -0.3)
jq("right_elbow_joint", 0.3)
pinocchio.forwardKinematics(red, rd, q)
pinocchio.updateFramePlacements(red, rd)
# EE frame
ee_fid = None
for fid in range(red.nframes):
    if red.frames[fid].name == "right_wrist_yaw_link":
        ee_fid = fid; break
print("pin EE frame:", red.frames[ee_fid].name if ee_fid is not None else "N/A")
def frame_pos(frame_name):
    for fid in range(red.nframes):
        if red.frames[fid].name == frame_name:
            return rd.oMf[fid].translation.copy()
    return None
J_pin = np.zeros((3, 4))
for k, j in enumerate(arm_j[:4]):
    jid = red.getJointId(j)
    qi = red.joints[jid].idx_q
    qplus = q.copy(); qplus[qi] += eps
    qminus = q.copy(); qminus[qi] -= eps
    pinocchio.forwardKinematics(red, rd, qplus); pinocchio.updateFramePlacements(red, rd)
    pp = frame_pos("right_wrist_yaw_link")
    pinocchio.forwardKinematics(red, rd, qminus); pinocchio.updateFramePlacements(red, rd)
    pm = frame_pos("right_wrist_yaw_link")
    J_pin[:, k] = (pp - pm) / (2*eps)
print("\nPinocchio J_arm (3x4):")
print(np.round(J_pin, 4))
print("\nDifference (Pin - Mj):")
print(np.round(J_pin - J_mj, 4))
