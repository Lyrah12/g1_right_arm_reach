#!/usr/bin/env python3
"""Long (10s) clean hold: arm motors get +qfrc_bias RECOMPUTED each step (tracking),
locked joints held by position servos (as the bridge does), wrist+hand FREE.
Does the arm hold, or drift (as in the sim) because the free wrist droops?
"""
import mujoco, numpy as np

xml = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml'
m = mujoco.MjModel.from_xml_path(xml); d = mujoco.MjData(m)

def jid_f(j): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
def qa(j): return m.jnt_qposadr[jid_f(j)]
def dd(j): return m.jnt_dofadr[jid_f(j)]

arm = ['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint']
arm_set = set(arm)
act = {}
for a in range(m.nu):
    trn = m.actuator_trnid[a][0]
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, trn)
    if jn in arm: act[jn] = a

# ready pose (bridge's)
ready = {'right_shoulder_pitch_joint':-0.3,'right_shoulder_roll_joint':0.0,'right_shoulder_yaw_joint':0.0,
         'right_elbow_joint':0.0,'right_wrist_roll_joint':0.0,'right_wrist_pitch_joint':0.0,'right_wrist_yaw_joint':0.0}

d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
for j,q in ready.items(): d.qpos[qa(j)] = q
mujoco.mj_forward(m,d)

# position-servo ctrl = qpos0 (as bridge); motor ctrl=0
for a in range(m.nu):
    if m.actuator_biastype[a] == 1:
        trn = m.actuator_trnid[a][0]
        jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, trn)
        d.ctrl[a] = m.qpos0[m.jnt_qposadr[jid_f(jn)]]
    else:
        d.ctrl[a] = 0.0

q0 = {j: d.qpos[qa(j)] for j in arm}
w0 = d.qpos[qa('right_wrist_pitch_joint')]
T = 10.0; n = int(T/m.opt.timestep)
report_every = n//10
for i in range(n):
    if i % report_every == 0:
        pass
    # recompute qfrc_bias at current pose, apply to arm motors
    qv = d.qvel.copy(); d.qvel[:] = 0.0; mujoco.mj_forward(m,d)
    qb = d.qfrc_bias.copy(); d.qvel[:] = qv
    for j in arm: d.ctrl[act[j]] = qb[dd(j)]
    # keep position-servo ctrl unchanged (they self-hold)
    mujoco.mj_step(m,d)

qf = {j: d.qpos[qa(j)] for j in arm}
drift = max(abs(qf[j]-q0[j]) for j in arm)
wf = d.qpos[qa('right_wrist_pitch_joint')]
print('after %.1fs:' % T)
for j in arm:
    print('  %-32s %+.3f -> %+.3f' % (j, q0[j], qf[j]))
print('arm drift = %.4f rad' % drift)
print('wrist_pitch: %+.3f -> %+.3f  (wrist drift %.3f)' % (w0, wf, wf-w0))
