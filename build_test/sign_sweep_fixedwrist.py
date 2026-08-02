#!/usr/bin/env python3
"""Definitive arm-hold sign test at a NON-equilibrium pose, with wrist+hand frozen.

Freezes all non-arm joints (re-clamped qpos+qvel each step) so only the arm moves.
Sweeps arm motor ctrl = s*qfrc_bias. Correct sign == smallest arm drift.
Pose: sp=-0.97, elb=-0.30 (the sim's drift pose, far from shoulder equilibrium).
"""
import mujoco, numpy as np

xml = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml'
m = mujoco.MjModel.from_xml_path(xml); d = mujoco.MjData(m)

def jid_f(j): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
def qa(j): return m.jnt_qposadr[jid_f(j)]
def dd(j): return m.jnt_dofadr[jid_f(j)]

arm = ['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint']
arm_set = set(arm)
arm_qaddr = [qa(j) for j in arm]
arm_daddr = [dd(j) for j in arm]

# all other joints to freeze (by qpos addr + dof addr)
freeze_pairs = []
for i in range(m.njnt):
    nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
    if nm and nm not in arm_set:
        freeze_pairs.append((m.jnt_qposadr[i], m.jnt_dofadr[i]))

act = {}
for a in range(m.nu):
    trn = m.actuator_trnid[a][0]
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, trn)
    if jn in arm: act[jn] = a

pose = {'right_shoulder_pitch_joint':-0.97,'right_shoulder_roll_joint':0.155,'right_shoulder_yaw_joint':0.233,
        'right_elbow_joint':-0.30,'right_wrist_roll_joint':0.0,'right_wrist_pitch_joint':-0.375,'right_wrist_yaw_joint':0.0}

def setup():
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    for j,q in pose.items():
        d.qpos[qa(j)] = q
    mujoco.mj_forward(m,d)

def run(scale, T=3.0):
    setup()
    qv=d.qvel.copy(); d.qvel[:]=0.0; mujoco.mj_forward(m,d); qb=d.qfrc_bias.copy(); d.qvel[:]=qv
    # freeze targets
    frozen = {qa_addr: d.qpos[qa_addr] for qa_addr, _ in freeze_pairs}
    for a in range(m.nu): d.ctrl[a]=0.0
    for j in arm: d.ctrl[act[j]] = scale*qb[dd(j)]
    q0={j:d.qpos[qa(j)] for j in arm}
    n=int(T/m.opt.timestep)
    for i in range(n):
        for qa_addr, dof_addr in freeze_pairs:
            d.qpos[qa_addr] = frozen[qa_addr]
            if dof_addr >= 0: d.qvel[dof_addr] = 0.0
        mujoco.mj_step(m,d)
    qf={j:d.qpos[qa(j)] for j in arm}
    drift=max(abs(qf[j]-q0[j]) for j in arm)
    print(f's={scale:+4.1f}: drift {drift:7.4f}  sp={qf[arm[0]]:+7.3f} ro={qf[arm[1]]:+6.3f} ya={qf[arm[2]]:+6.3f} elb={qf[arm[3]]:+7.3f}')
    return drift

print('qfrc_bias at this pose:')
setup()
qv=d.qvel.copy(); d.qvel[:]=0.0; mujoco.mj_forward(m,d); qb=d.qfrc_bias.copy(); d.qvel[:]=qv
for j in arm: print('  %-32s %+.3f' % (j, qb[dd(j)]))

print('\nsweep (non-arm joints frozen):')
res={}
for s in [-2.0,-1.5,-1.0,-0.5,0.0,0.5,1.0,1.5,2.0]:
    res[s]=run(s)
best=min(res,key=lambda s:res[s])
print(f'\nBEST scale = {best:+.1f} (drift {res[best]:.4f})')
