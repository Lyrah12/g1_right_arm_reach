#!/usr/bin/env python3
"""Make the base TRULY FIXED (free joint -> fixed) and verify the arm gravity
FF (+qfrc_bias) now cancels gravity exactly (qacc -> 0) and holds static."""
import mujoco
import numpy as np
import sys

scene = sys.argv[1]
with open(scene, encoding='utf-8') as f:
    text = f.read()
# Replace the free base joint with a fixed one
import re
# robust: anywhere a free joint for the base appears
new_text = text.replace('type="free"', 'type="fixed"')
print("free->fixed applied:", new_text != text)
text = new_text
m = mujoco.MjModel.from_xml_string(text)
d = mujoco.MjData(m)
print("nv =", m.nv, "(was 59 with free base)")

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
locked = []
for jid in range(1, m.njnt):
    n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
    if n is None or n in arm or n in left_arm or n in hand_all:
        continue
    locked.append((m.jnt_qposadr[jid], m.jnt_dofadr[jid], float(d.qpos[m.jnt_qposadr[jid]])))
act = {}
for a in range(m.nu):
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0])
    if jn in arm: act[jn] = a

def setup():
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
    d.qpos[qpa("right_elbow_joint")] = 0.3
    for a in range(m.nu): d.ctrl[a] = 0.0
    mujoco.mj_forward(m, d)

setup()
qb = {j: d.qfrc_bias[da(j)] for j in arm}
print("\nqfrc_bias (fixed base) at ready:", {j: round(qb[j],3) for j in arm})

def qacc(scale):
    setup()
    for j in arm: d.ctrl[act[j]] = scale * qb[j]
    mujoco.mj_forward(m, d)
    return {j: d.qacc[da(j)] for j in arm}

for scale in [-1.0, +1.0]:
    a = qacc(scale)
    mx = max(abs(a[j]) for j in arm)
    print(f"scale={scale:+.0f} → max|qacc| = {mx:8.3f}")

# step test with correct sign
for scale in [-1.0, +1.0]:
    setup()
    for j in arm: d.ctrl[act[j]] = scale * qb[j]
    q0 = np.array([d.qpos[qpa(j)] for j in arm])
    n = int(2.0 / m.opt.timestep)
    for i in range(n):
        for addr, dof, val in locked:
            d.qpos[addr] = val; d.qvel[dof] = 0.0
        mujoco.mj_step(m, d)
    qf = np.array([d.qpos[qpa(j)] for j in arm])
    print(f"scale={scale:+.0f} → 2s drift = {np.max(np.abs(qf-q0)):.4f} rad")
