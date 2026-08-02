#!/usr/bin/env python3
"""Replicate the sim hold EXACTLY in clean MuJoCo: use the sim's actual full qpos,
lock legs/waist/left-arm with position-servo ctrl=qpos0 (as the bridge does),
apply ctrl=+qfrc_bias to the arm motors only. Does the arm hold or drift?

Reads the full 41-joint position array from a captured /joint_states sample.
"""
import mujoco, numpy as np, sys, re

xml = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml'
m = mujoco.MjModel.from_xml_path(xml); d = mujoco.MjData(m)

# ---- load full joint names in MuJoCo order ----
names = []
for jid in range(m.njnt):
    nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
    names.append(nm)

# ---- read captured full position array ----
raw = open('/tmp/js_traj2.log').read()
blocks = raw.split('---')
def get_pos(b):
    names_b = re.findall(r'name:\s*\n((?:\s*-\s*\S+\n?)+)', b)
    pos = re.findall(r'position:\s*\n((?:\s*-\s*[0-9.eE+-]+\n?)+)', b)
    if not names_b or not pos: return None
    ns = re.findall(r'-\s*(\S+)', names_b[0])
    ps = [float(x) for x in re.findall(r'-\s*([0-9.eE+-]+)', pos[0])]
    return dict(zip(ns, ps))
samples = [s for s in (get_pos(b) for b in blocks) if s]
print(f'{len(samples)} captured samples')
state = samples[-1]  # use last (most settled)
print('using sample with sp=%+.3f elb=%+.3f wp=%+.3f' % (
    state.get('right_shoulder_pitch_joint'), state.get('right_elbow_joint'),
    state.get('right_wrist_pitch_joint')))

# ---- set qpos from captured state ----
d.qpos[:] = m.qpos0[:]
d.qvel[:] = 0.0
for j, q in state.items():
    if j in names:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        d.qpos[m.jnt_qposadr[jid]] = q

arm = ['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint']
# actuator -> joint map
act = {}
act_type = {}
for a in range(m.nu):
    trn = m.actuator_trnid[a][0]
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, trn)
    act[jn] = act.get(jn, []) + [a]
    act_type[a] = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)

# ---- set ctrl like the bridge: locked joints (position servo) -> qpos0 ----
# position-servo actuators: set ctrl to qpos0 of their joint
def jid_f(j): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
for a in range(m.nu):
    trn = m.actuator_trnid[a][0]
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, trn)
    # motor (torque) vs position: check biastype
    if m.actuator_biastype[a] == 1:  # mjBIAS_AFFINE = position servo
        d.ctrl[a] = m.qpos0[m.jnt_qposadr[jid_f(jn)]]
    else:
        d.ctrl[a] = 0.0

# arm motors: apply +qfrc_bias
mujoco.mj_forward(m, d)
qv = d.qvel.copy(); d.qvel[:] = 0.0; mujoco.mj_forward(m, d)
qb = d.qfrc_bias.copy(); d.qvel[:] = qv
print('qfrc_bias at this pose:')
for j in arm:
    print('  %-32s %+.3f' % (j, qb[m.jnt_dofadr[jid_f(j)]]))
for j in arm:
    for a in act[j]:
        if act_type[a].startswith('act_r_'):
            d.ctrl[a] = qb[m.jnt_dofadr[jid_f(j)]]

q0 = {j: d.qpos[m.jnt_qposadr[jid_f(j)]] for j in arm}
T = 3.0; n = int(T / m.opt.timestep)
for i in range(n):
    mujoco.mj_step(m, d)
qf = {j: d.qpos[m.jnt_qposadr[jid_f(j)]] for j in arm}
drift = max(abs(qf[j]-q0[j]) for j in arm)
print('RESULT after %.1fs: arm drift = %.4f rad' % (T, drift))
for j in arm:
    print('  %-32s %+.3f -> %+.3f' % (j, q0[j], qf[j]))
wp = d.qpos[m.jnt_qposadr[jid('right_wrist_pitch_joint')]]
print('wrist_pitch -> %+.3f' % wp)
