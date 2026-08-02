#!/usr/bin/env python3
"""Compare right-arm subtree (shoulder_pitch down, incl. hand) total mass & COM
between MuJoCo and Pinocchio(patched URDF) at the converged config."""
import mujoco
import numpy as np
import pinocchio as pin
import re

SCENE = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml'
URDF = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf'

Q7 = [-0.0626, -0.2869, -0.3036, -0.1485, 0.0794, 0.3352, 1.1167]
ARM7 = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
        'right_shoulder_yaw_joint', 'right_elbow_joint',
        'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint']

# ---------- MuJoCo ----------
m = mujoco.MjModel.from_xml_path(SCENE)
d = mujoco.MjData(m)
def qpa(j): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
for j, q in zip(ARM7, Q7): d.qpos[qpa(j)] = q
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)
# subtree = right_shoulder_pitch_link + all descendants
sp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'right_shoulder_pitch_link')
sub = set()
stack = [sp]
while stack:
    b = stack.pop()
    sub.add(b)
    for c in range(m.nbody):
        if m.body_parentid[c] == b:
            stack.append(c)
tot_mj = sum(float(m.body_mass[b]) for b in sub)
com_mj = sum(float(m.body_mass[b]) * d.xipos[b] for b in sub) / tot_mj
print(f"MuJoCo subtree: {len(sub)} bodies, mass={tot_mj:.4f}, COM world={np.round(com_mj,4)}")

# ---------- Pinocchio (patched URDF, fixed base + reduced) ----------
full = pin.buildModelFromUrdf(URDF)
names = list(full.names)
keep = ARM7
lock_ids = [i for i, nm in enumerate(names) if i != 0 and nm not in keep]
model = pin.buildReducedModel(full, lock_ids, pin.neutral(full))
data = model.createData()
q = np.zeros(model.nq)
idx = {nm: i for i, nm in enumerate(model.names)}
for j, v in zip(ARM7, Q7): q[model.joints[idx[j]].idx_q] = v
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
# subtree COM in world (URDF base frame): all bodies from shoulder_pitch down
# reduced model: shoulder_pitch is the first child of universe (fixed base)
sp_id = model.getBodyId('right_shoulder_pitch_link')
tot_pin = 0.0; com_num = np.zeros(3)
# iterate bodies via joints (joint i -> child body i in Pinocchio reduced model)
print("Joints in Pinocchio reduced model:", model.njoints)
for jid in range(1, model.njoints):
    im = model.inertias[jid]
    tot_pin += im.mass
    com_num += im.mass * data.oMi[jid].translation
com_pin = com_num / tot_pin
print(f"Pinocchio ALL-bodies: mass={tot_pin:.4f}, COM world={np.round(com_pin,4)}")
# Subtree: bodies whose parent chain goes through shoulder_pitch joint
sub_com_num = np.zeros(3); sub_mass = 0.0
for jid in range(1, model.njoints):
    c = jid
    in_arm = False
    while c != 0:
        c = model.parents[c]
        if c == sp_id: in_arm = True; break
    if in_arm:
        im = model.inertias[jid]
        sub_mass += im.mass
        sub_com_num += im.mass * data.oMi[jid].translation
if sub_mass > 0:
    print(f"Pinocchio arm-subtree: mass={sub_mass:.4f}, COM world={np.round(sub_com_num/sub_mass,4)}")
