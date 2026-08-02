#!/usr/bin/env python3
"""Probe the residual: which joint has qacc≠0 with +qfrc_bias on the fixed base?
Also test compensating hand masters + finger hold."""
import mujoco
import numpy as np
import sys

m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)
print("nv =", m.nv)

def qpa(j): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
def da(j): return m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
arm = ["right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
       "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]
hand_m = ["R_thumb_proximal_yaw_joint","R_thumb_proximal_pitch_joint","R_index_proximal_joint",
          "R_middle_proximal_joint","R_ring_proximal_joint","R_pinky_proximal_joint"]
act = {}
for a in range(m.nu):
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0])
    if jn in arm or jn in hand_m: act[jn] = a

d.qpos[:] = m.qpos0[:]
d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
d.qpos[qpa("right_elbow_joint")] = 0.3
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)
qb = {j: d.qfrc_bias[da(j)] for j in arm + hand_m}

def setup():
    d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
    d.qpos[qpa("right_elbow_joint")] = 0.3
    d.qvel[:] = 0.0
    for a in range(m.nu): d.ctrl[a] = 0.0
    mujoco.mj_forward(m, d)

# case A: arm only, +qfrc_bias
setup()
for j in arm: d.ctrl[act[j]] = qb[j]
mujoco.mj_forward(m, d)
print("\nA) arm +qfrc_bias only:")
for j in arm: print(f"   {j:35s} qacc={d.qacc[da(j)]:+9.3f}")

# case B: arm + hand masters, +qfrc_bias
setup()
for j in arm + hand_m: d.ctrl[act[j]] = qb[j]
mujoco.mj_forward(m, d)
print("\nB) arm + hand masters +qfrc_bias:")
for j in arm: print(f"   {j:35s} qacc={d.qacc[da(j)]:+9.3f}")
