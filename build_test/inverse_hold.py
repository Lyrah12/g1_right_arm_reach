#!/usr/bin/env python3
"""Compute the exact hold torque via mj_inverse(qacc=0) and verify it holds
the arm static in the welded scene. Also print qfrc_inverse vs qfrc_bias."""
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
left_arm = ["left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
            "left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint"]
hand = ["R_thumb_proximal_yaw_joint","R_thumb_proximal_pitch_joint","R_thumb_intermediate_joint",
        "R_thumb_distal_joint","R_index_proximal_joint","R_index_intermediate_joint",
        "R_middle_proximal_joint","R_middle_intermediate_joint","R_ring_proximal_joint",
        "R_ring_intermediate_joint","R_pinky_proximal_joint","R_pinky_intermediate_joint"]
locked = []
for jid in range(1, m.njnt):
    n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
    if n is None or n in arm or n in left_arm or n in hand:
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

def hold_torque():
    """mj_inverse with qacc=0 → qfrc_inverse = generalized force producing qacc=0."""
    d.qacc[:] = 0.0
    mujoco.mj_inverse(m, d)
    return d.qfrc_inverse.copy()

setup()
hold = hold_torque()
print("qfrc_inverse (qacc=0) vs qfrc_bias for arm joints:")
for j in arm:
    print(f"  {j:35s} inverse={hold[da(j)]:+8.3f}   bias={d.qfrc_bias[da(j)]:+8.3f}")

# apply ctrl = qfrc_inverse (the hold torque) to arm motors
for j in arm:
    d.ctrl[act[j]] = hold[da(j)]
mujoco.mj_forward(m, d)
print("\nqacc after applying qfrc_inverse as ctrl:")
maxa = 0
for j in arm:
    maxa = max(maxa, abs(d.qacc[da(j)]))
    print(f"  {j:35s} qacc={d.qacc[da(j)]:+9.3f}")
print(f"max|qacc| = {maxa:.4f}")

# now step and measure drift
q0 = np.array([d.qpos[qpa(j)] for j in arm])
n = int(2.0 / m.opt.timestep)
for i in range(n):
    for addr, dof, val in locked:
        d.qpos[addr] = val; d.qvel[dof] = 0.0
    mujoco.mj_step(m, d)
qf = np.array([d.qpos[qpa(j)] for j in arm])
print(f"\n2s drift with inverse-hold = {np.max(np.abs(qf-q0)):.4f} rad")
