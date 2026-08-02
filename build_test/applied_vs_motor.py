#!/usr/bin/env python3
"""Decisive: apply gravity hold via qfrc_applied (exact generalized force) vs
motor ctrl, with both signs, and measure qacc. Settles the sign + path."""
import mujoco
import numpy as np
import sys

scene = sys.argv[1]
with open(scene, encoding='utf-8') as f:
    text = f.read()
weld = '  <equality>\n    <weld name="weld_pelvis" body1="pelvis" body2="world" solref="0.002 2"/>\n  </equality>\n</mujoco>\n'
if "<equality>" not in text:
    text = text.rstrip()[:-len("</mujoco>")].rstrip() + "\n" + weld
else:
    idx = text.rindex("</equality>")
    text = text[:idx] + '    <weld name="weld_pelvis" body1="pelvis" body2="world" solref="0.002 2"/>\n' + text[idx:]
m = mujoco.MjModel.from_xml_string(text)
d = mujoco.MjData(m)

def qpa(j): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
def da(j): return m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
arm = ["right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
       "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]
act = {}
for a in range(m.nu):
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0])
    if jn in arm: act[jn] = a

def setup():
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
    d.qpos[qpa("right_elbow_joint")] = 0.3
    d.qfrc_applied[:] = 0.0
    for a in range(m.nu): d.ctrl[a] = 0.0
    mujoco.mj_forward(m, d)

setup()
qb = np.array([d.qfrc_bias[da(j)] for j in arm])
print("qfrc_bias:", [round(v,3) for v in qb])

def qacc_max():
    mujoco.mj_forward(m, d)
    return max(abs(d.qacc[da(j)]) for j in arm)

# 1) qfrc_applied = +bias
setup();
for k, j in enumerate(arm): d.qfrc_applied[da(j)] = qb[k]
print(f"\nqfrc_applied=+bias     → max|qacc| = {qacc_max():8.3f}")

# 2) qfrc_applied = -bias
setup()
for k, j in enumerate(arm): d.qfrc_applied[da(j)] = -qb[k]
print(f"qfrc_applied=-bias     → max|qacc| = {qacc_max():8.3f}")

# 3) motor ctrl = +bias
setup()
for k, j in enumerate(arm): d.ctrl[act[j]] = qb[k]
print(f"motor ctrl   =+bias    → max|qacc| = {qacc_max():8.3f}")

# 4) motor ctrl = -bias
setup()
for k, j in enumerate(arm): d.ctrl[act[j]] = -qb[k]
print(f"motor ctrl   =-bias    → max|qacc| = {qacc_max():8.3f}")
