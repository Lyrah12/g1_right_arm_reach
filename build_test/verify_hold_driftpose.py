#!/usr/bin/env python3
"""At the sim's actual drift pose (wrist drooped to limit), does ctrl=+qfrc_bias hold?
Clean MuJoCo, fixed base, no clamps. Compares hold at (a) ready pose, (b) drift pose.
"""
import mujoco, numpy as np

xml = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml'
m = mujoco.MjModel.from_xml_path(xml); d = mujoco.MjData(m)

def jid(j): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
def qa(j): return m.jnt_qposadr[jid(j)]
def dd(j): return m.jnt_dofadr[jid(j)]

arm = ['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint']
act = {}
for a in range(m.nu):
    trn = m.actuator_trnid[a][0]
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, trn)
    if jn in arm: act[jn] = a

ready = {'right_shoulder_pitch_joint':-0.3,'right_shoulder_roll_joint':0,'right_shoulder_yaw_joint':0,
         'right_elbow_joint':0,'right_wrist_roll_joint':0,'right_wrist_pitch_joint':0,'right_wrist_yaw_joint':0}
drift = {'right_shoulder_pitch_joint':-0.7,'right_shoulder_roll_joint':0.12,'right_shoulder_yaw_joint':0.1,
         'right_elbow_joint':-0.1,'right_wrist_roll_joint':0,'right_wrist_pitch_joint':1.565,'right_wrist_yaw_joint':0}

def qfrc_at(pose):
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    for j,q in pose.items(): d.qpos[qa(j)] = q
    mujoco.mj_forward(m,d)
    qv=d.qvel.copy(); d.qvel[:]=0.0; mujoco.mj_forward(m,d)
    qb=d.qfrc_bias.copy(); d.qvel[:]=qv
    return qb

def run(label, pose, T=4.0):
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    for j,q in pose.items(): d.qpos[qa(j)] = q
    mujoco.mj_forward(m,d)
    qb = qfrc_at(pose)
    for a in range(m.nu): d.ctrl[a] = 0.0
    for j in arm: d.ctrl[act[j]] = qb[dd(j)]
    # wrist left unpowered (ctrl=0)
    q0 = {j:d.qpos[qa(j)] for j in arm}
    n=int(T/m.opt.timestep)
    for i in range(n): mujoco.mj_step(m,d)
    qf = {j:d.qpos[qa(j)] for j in arm}
    drift_max = max(abs(qf[j]-q0[j]) for j in arm)
    wp = d.qpos[qa('right_wrist_pitch_joint')]
    print(f'[{label}] hold T={T}s: arm drift={drift_max:.4f}  final sp={qf[arm[0]]:+.3f} '
          f'elb={qf[arm[3]]:+.3f}  wrist_pitch={wp:+.3f}')

print('qfrc_bias READY: sp=%+.3f elb=%+.3f' % (qfrc_at(ready)[dd("right_shoulder_pitch_joint")], qfrc_at(ready)[dd("right_elbow_joint")]))
print('qfrc_bias DRIFT: sp=%+.3f elb=%+.3f' % (qfrc_at(drift)[dd("right_shoulder_pitch_joint")], qfrc_at(drift)[dd("right_elbow_joint")]))
run('ready', ready)
run('drift', drift)
