#!/usr/bin/env python3
"""In the WELDED scene, check instantaneous qacc with ctrl=+/-qfrc_bias.
Correct FF = the sign that gives qacc ~ 0."""
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

d.qpos[:] = m.qpos0[:]
d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
d.qpos[qpa("right_elbow_joint")] = 0.3
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)
qb = {j: d.qfrc_bias[da(j)] for j in arm}

def qacc_with(scale):
    d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
    d.qpos[qpa("right_elbow_joint")] = 0.3
    d.qvel[:] = 0.0
    for a in range(m.nu): d.ctrl[a] = 0.0
    for j in arm: d.ctrl[act[j]] = scale * qb[j]
    mujoco.mj_forward(m, d)
    return {j: d.qacc[da(j)] for j in arm}

print("qfrc_bias at ready:", {j: round(qb[j],3) for j in arm})
for scale in [-1.0, +1.0]:
    a = qacc_with(scale)
    norm = max(abs(a[j]) for j in arm)
    print(f"\nscale={scale:+.0f} (ctrl={'+/-'}qfrc_bias) → max|qacc| = {norm:.2f} rad/s²:")
    for j in arm:
        print(f"   {j:35s} qacc={a[j]:+9.2f}")
