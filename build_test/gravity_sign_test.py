#!/usr/bin/env python3
"""Definitive gravity-sign test — replicates the bridge's hard-clamp EXACTLY.

Clamp pelvis (pos+quat+qvel) and locked joints (qpos+qvel) every step, set all
non-arm motors to 0, then apply ctrl = scale * qfrc_bias to the 7 right-arm
motors and measure drift. Correct sign == smallest drift.

Run: python3 gravity_sign_test.py <scene.xml>
"""
import sys
import numpy as np
import mujoco

scene = sys.argv[1]
model = mujoco.MjModel.from_xml_path(scene)
data = mujoco.MjData(model)

ready = {
    'right_shoulder_pitch_joint': -0.3,
    'right_shoulder_roll_joint': 0.0,
    'right_shoulder_yaw_joint': 0.0,
    'right_elbow_joint': 0.3,
    'right_wrist_roll_joint': 0.0,
    'right_wrist_pitch_joint': 0.0,
    'right_wrist_yaw_joint': 0.0,
}
arm_joints = list(ready.keys())

def qid(j): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
def qpos_addr(j): return model.jnt_qposadr[qid(j)]
def dof_addr(j): return model.jnt_dofadr[qid(j)]

# motor actuator per arm joint
act = {}
for a in range(model.nu):
    trn = model.actuator_trnid[a][0]
    jn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, trn)
    if jn in arm_joints:
        act[jn] = a

# --- like bridge: pelvis at qpos0 (NOT identity), locked joints clamped ---
init_pelvis_pos = np.array(data.qpos[0:3])
init_pelvis_quat = np.array(data.qpos[3:7])
init_pelvis_pos[2] += 0.03  # bridge lifts pelvis 3cm

# left arm joints zeroed (like bridge)
left_arm = ['left_shoulder_pitch_joint','left_shoulder_roll_joint','left_shoulder_yaw_joint',
            'left_elbow_joint','left_wrist_roll_joint','left_wrist_pitch_joint','left_wrist_yaw_joint']
for j in left_arm:
    data.qpos[qpos_addr(j)] = 0.0

# right arm ready pose
for j, q in ready.items():
    data.qpos[qpos_addr(j)] = q

# locked joints = all except arm + hand (bridge: hand excluded from clamp)
hand_joints = ['R_thumb_proximal_yaw_joint','R_thumb_proximal_pitch_joint','R_thumb_intermediate_joint',
               'R_thumb_distal_joint','R_index_proximal_joint','R_index_intermediate_joint',
               'R_middle_proximal_joint','R_middle_intermediate_joint','R_ring_proximal_joint',
               'R_ring_intermediate_joint','R_pinky_proximal_joint','R_pinky_intermediate_joint']
locked = []   # (qpos_addr, dof_addr, init_q)
for jid in range(1, model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
    if name is None or name in arm_joints or name in left_arm or name in hand_joints:
        continue
    if name in ('floating_base_joint',):
        continue
    locked.append((model.jnt_qposadr[jid], model.jnt_dofadr[jid], float(data.qpos[model.jnt_qposadr[jid]])))

mujoco.mj_forward(model, data)

def qfrc_at_rest():
    qv = data.qvel.copy(); data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    q = data.qfrc_bias.copy(); data.qvel[:] = qv
    return q

qr = qfrc_at_rest()
print("qfrc_bias (raw) at ready pose:")
for j in arm_joints:
    print(f"  {j:35s} dof={dof_addr(j):3d} {qr[dof_addr(j)]:+8.3f}")

def run(label, scale):
    data.qpos[:] = model.qpos0[:]
    data.qvel[:] = 0.0
    data.qpos[0:3] = init_pelvis_pos
    data.qpos[3:7] = init_pelvis_quat
    for j in left_arm: data.qpos[qpos_addr(j)] = 0.0
    for j, q in ready.items(): data.qpos[qpos_addr(j)] = q
    for addr, dof, val in locked: data.qpos[addr] = val
    mujoco.mj_forward(model, data)
    data.qvel[:] = 0.0
    qr2 = qfrc_at_rest()
    for a in range(model.nu):
        data.ctrl[a] = 0.0
    for j in arm_joints:
        data.ctrl[act[j]] = scale * qr2[dof_addr(j)]
    q0 = {j: data.qpos[qpos_addr(j)] for j in arm_joints}
    n = int(3.0 / model.opt.timestep)
    for i in range(n):
        # hard clamp pelvis
        data.qpos[0:3] = init_pelvis_pos; data.qpos[3:7] = init_pelvis_quat; data.qvel[0:6] = 0.0
        # hard clamp locked joints
        for addr, dof, val in locked:
            data.qpos[addr] = val
            data.qvel[dof] = 0.0
        mujoco.mj_step(model, data)
    qf = {j: data.qpos[qpos_addr(j)] for j in arm_joints}
    drift = max(abs(qf[j] - q0[j]) for j in arm_joints)
    print(f"[{label:5s}] drift 3s = {drift:8.4f}  sp={qf['right_shoulder_pitch_joint']:7.3f} "
          f"elb={qf['right_elbow_joint']:7.3f} wyaw={qf['right_wrist_yaw_joint']:7.3f}")
    return drift

print("\n=== experiment (correct sign = smallest drift) ===")
d0 = run("0",   0.0)
dp = run("+X", +1.0)
dm = run("-X", -1.0)
best = min([(d0, "0 (no FF)"), (dp, "+qfrc_bias"), (dm, "-qfrc_bias")])
print(f"\nBEST: {best[1]} drift={best[0]:.4f} rad")
if best[1] == "+qfrc_bias":
    print("=> HOLD = +qfrc_bias  (MuJoCo: M qdd + qfrc_bias = tau)")
elif best[1] == "-qfrc_bias":
    print("=> HOLD = -qfrc_bias  (MuJoCo: M qdd = qfrc_bias + tau)")
else:
    print("=> no FF holds best — model/actuator issue")
