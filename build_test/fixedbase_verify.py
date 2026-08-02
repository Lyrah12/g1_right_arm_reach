#!/usr/bin/env python3
"""Verify on the FIXED-BASE generated MJCF: does +qfrc_bias gravity FF hold the
arm static? Load the actual generated file directly (nv=53, includes resolved)."""
import mujoco
import numpy as np
import sys

path = sys.argv[1]  # scene xml (includes generated MJCF + hand assets)
m = mujoco.MjModel.from_xml_path(path)
d = mujoco.MjData(m)
print("nv =", m.nv, " nq =", m.nq)

def qpa(j): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
def da(j): return m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
arm = ["right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
       "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]
left_arm = ["left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
            "left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint"]
hand_all = ["R_thumb_proximal_yaw_joint","R_thumb_proximal_pitch_joint","R_thumb_intermediate_joint",
        "R_thumb_distal_joint","R_index_proximal_joint","R_index_intermediate_joint",
        "R_middle_proximal_joint","R_middle_intermediate_joint","R_ring_proximal_joint",
        "R_ring_intermediate_joint","R_pinky_proximal_joint","R_pinky_intermediate_joint"]
act = {}
for a in range(m.nu):
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0])
    if jn in arm or jn in hand_all: act[jn] = a

# ready pose
d.qpos[:] = m.qpos0[:]
d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
d.qpos[qpa("right_elbow_joint")] = 0.3
d.qvel[:] = 0.0
for a in range(m.nu): d.ctrl[a] = 0.0
mujoco.mj_forward(m, d)
qb = {j: d.qfrc_bias[da(j)] for j in arm}
print("qfrc_bias at ready:", {j: round(qb[j],3) for j in arm})

def qacc(scale):
    d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
    d.qpos[qpa("right_elbow_joint")] = 0.3
    d.qvel[:] = 0.0
    for a in range(m.nu): d.ctrl[a] = 0.0
    for j in arm: d.ctrl[act[j]] = scale * qb[j]
    mujoco.mj_forward(m, d)
    return {j: d.qacc[da(j)] for j in arm}

for scale in [-1.0, +1.0]:
    a = qacc(scale)
    mx = max(abs(a[j]) for j in arm)
    print(f"scale={scale:+.0f} → max|qacc| = {mx:8.3f}")

# step 2s with each scale
for scale in [-1.0, +1.0]:
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
    d.qpos[qpa("right_elbow_joint")] = 0.3
    for a in range(m.nu): d.ctrl[a] = 0.0
    for j in arm: d.ctrl[act[j]] = scale * qb[j]
    q0 = np.array([d.qpos[qpa(j)] for j in arm])
    n = int(2.0 / m.opt.timestep)
    for i in range(n):
        mujoco.mj_step(m, d)
    qf = np.array([d.qpos[qpa(j)] for j in arm])
    print(f"scale={scale:+.0f} → 2s drift = {np.max(np.abs(qf-q0)):.4f} rad")
