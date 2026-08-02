#!/usr/bin/env python3
"""Clean gravity-sign test on the CURRENT fixed-base model (no hard clamps).

Loads the merged scene exactly as the bridge does (fixed base, no free joint),
sets the right arm to the bridge's ready pose, then applies ctrl = scale*qfrc_bias
to the 7 arm motors and measures 2s drift. Correct sign == smallest drift.

Run: python3 gravity_sign_clean.py <scene.xml>
"""
import sys
import numpy as np
import mujoco

scene = sys.argv[1]
model = mujoco.MjModel.from_xml_path(scene)
data = mujoco.MjData(model)

# Bridge ready pose (from bridge startup log)
ready = {
    'right_shoulder_pitch_joint': -0.3,
    'right_shoulder_roll_joint': 0.0,
    'right_shoulder_yaw_joint': 0.0,
    'right_elbow_joint': 0.0,
    'right_wrist_roll_joint': 0.0,
    'right_wrist_pitch_joint': 0.0,
    'right_wrist_yaw_joint': 0.0,
}
arm_joints = list(ready.keys())

def jid(j): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
def qaddr(j): return model.jnt_qposadr[jid(j)]
def dof(j): return model.jnt_dofadr[jid(j)]

act = {}
for a in range(model.nu):
    trn = model.actuator_trnid[a][0]
    jn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, trn)
    if jn in arm_joints:
        act[jn] = a
print('motor->joint map:', {j: act[j] for j in arm_joints})

def set_ready():
    data.qpos[:] = model.qpos0[:]
    data.qvel[:] = 0.0
    for j, q in ready.items():
        data.qpos[qaddr(j)] = q
    mujoco.mj_forward(model, data)

def qfrc_at_rest():
    qv = data.qvel.copy(); data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    q = data.qfrc_bias.copy(); data.qvel[:] = qv
    return q

set_ready()
qr = qfrc_at_rest()
print('qfrc_bias (zero-vel, ready pose):')
for j in arm_joints:
    print(f'  {j:35s} dof={dof(j):3d} {qr[dof(j)]:+8.3f}')

def run(label, scale, T=2.0):
    set_ready()
    qr2 = qfrc_at_rest()
    for a in range(model.nu):
        data.ctrl[a] = 0.0
    for j in arm_joints:
        data.ctrl[act[j]] = scale * qr2[dof(j)]
    q0 = {j: data.qpos[qaddr(j)] for j in arm_joints}
    n = int(T / model.opt.timestep)
    for i in range(n):
        mujoco.mj_step(model, data)
    qf = {j: data.qpos[qaddr(j)] for j in arm_joints}
    drift = max(abs(qf[j] - q0[j]) for j in arm_joints)
    print(f'[{label:6s}] drift {T}s = {drift:8.4f}  sp={qf["right_shoulder_pitch_joint"]:+7.3f} '
          f'ro={qf["right_shoulder_roll_joint"]:+6.3f} elb={qf["right_elbow_joint"]:+7.3f} '
          f'wp={qf["right_wrist_pitch_joint"]:+6.3f}')
    return drift

print('\n=== experiment (correct sign = smallest drift) ===')
res = {}
for scale in [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]:
    res[scale] = run(f's={scale:+.1f}', scale)
best = min(res, key=lambda s: res[s])
print(f'\nBEST scale = {best:+.1f} (drift {res[best]:.4f})')
if abs(best - 1.0) < 0.01:
    print('=> HOLD = +qfrc_bias')
elif abs(best + 1.0) < 0.01:
    print('=> HOLD = -qfrc_bias')
else:
    print(f'=> neither ±1 exactly; best is scale {best:+.1f}')
