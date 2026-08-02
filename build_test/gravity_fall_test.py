#!/usr/bin/env python3
"""Investigate: can the arm move at all under gravity in the clamped scene?
1. Arm at raised pose (sp=1.0), 0 torque -> should fall if dynamic.
2. Arm at raised pose, ctrl=+qfrc_bias -> compare.
3. Also print arm joint inertia/armature/damping.
Run: python3 gravity_fall_test.py <scene.xml>
"""
import sys
import numpy as np
import mujoco

scene = sys.argv[1]
model = mujoco.MjModel.from_xml_path(scene)
data = mujoco.MjData(model)

arm_joints = ['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint',
              'right_elbow_joint','right_wrist_roll_joint','right_wrist_pitch_joint','right_wrist_yaw_joint']
def qid(j): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
def qpos_addr(j): return model.jnt_qposadr[qid(j)]
def dof_addr(j): return model.jnt_dofadr[qid(j)]

print("arm joint properties:")
mujoco.mj_forward(model, data)
for j in arm_joints:
    jid = qid(j)
    dof = dof_addr(j)
    print(f"  {j:35s} qpos0={model.qpos0[qpos_addr(j)]:+7.3f} range={model.jnt_range[jid]} "
          f"damping={model.dof_damping[dof]:.4f} armature={model.dof_armature[dof]:.5f} "
          f"M_diag={data.M[dof]:.5f}")

act = {}
for a in range(model.nu):
    trn = model.actuator_trnid[a][0]
    jn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, trn)
    if jn in arm_joints:
        act[jn] = a

left_arm = ['left_shoulder_pitch_joint','left_shoulder_roll_joint','left_shoulder_yaw_joint',
            'left_elbow_joint','left_wrist_roll_joint','left_wrist_pitch_joint','left_wrist_yaw_joint']
hand_joints = ['R_thumb_proximal_yaw_joint','R_thumb_proximal_pitch_joint','R_thumb_intermediate_joint',
               'R_thumb_distal_joint','R_index_proximal_joint','R_index_intermediate_joint',
               'R_middle_proximal_joint','R_middle_intermediate_joint','R_ring_proximal_joint',
               'R_ring_intermediate_joint','R_pinky_proximal_joint','R_pinky_intermediate_joint']
locked = []
for jid in range(1, model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
    if name is None or name in arm_joints or name in left_arm or name in hand_joints:
        continue
    locked.append((model.jnt_qposadr[jid], model.jnt_dofadr[jid], float(data.qpos[model.jnt_qposadr[jid]])))

init_pelvis_pos = np.array(data.qpos[0:3]); init_pelvis_quat = np.array(data.qpos[3:7])
init_pelvis_pos[2] += 0.03

def qfrc_at_rest():
    qv = data.qvel.copy(); data.qvel[:] = 0.0
    mujoco.mj_forward(model, data); q = data.qfrc_bias.copy(); data.qvel[:] = qv
    return q

def run(label, sp0, scale):
    data.qpos[:] = model.qpos0[:]; data.qvel[:] = 0.0
    data.qpos[0:3] = init_pelvis_pos; data.qpos[3:7] = init_pelvis_quat
    for j in left_arm: data.qpos[qpos_addr(j)] = 0.0
    for j, q in {'right_shoulder_pitch_joint':sp0,'right_shoulder_roll_joint':0.0,
                 'right_shoulder_yaw_joint':0.0,'right_elbow_joint':0.3,
                 'right_wrist_roll_joint':0.0,'right_wrist_pitch_joint':0.0,
                 'right_wrist_yaw_joint':0.0}.items():
        data.qpos[qpos_addr(j)] = q
    for addr, dof, val in locked: data.qpos[addr] = val
    mujoco.mj_forward(model, data); data.qvel[:] = 0.0
    qr = qfrc_at_rest()
    for a in range(model.nu): data.ctrl[a] = 0.0
    if scale != 0.0:
        for j in arm_joints: data.ctrl[act[j]] = scale * qr[dof_addr(j)]
    q0 = data.qpos[qpos_addr('right_shoulder_pitch_joint')]
    n = int(2.0 / model.opt.timestep)
    hist = []
    for i in range(n):
        data.qpos[0:3] = init_pelvis_pos; data.qpos[3:7] = init_pelvis_quat; data.qvel[0:6] = 0.0
        for addr, dof, val in locked: data.qpos[addr] = val; data.qvel[dof] = 0.0
        mujoco.mj_step(model, data)
        if i % 250 == 0:
            hist.append(f"{i*model.opt.timestep:.2f}s:sp={data.qpos[qpos_addr('right_shoulder_pitch_joint')]:6.3f} "
                        f"elb={data.qpos[qpos_addr('right_elbow_joint')]:6.3f}")
    qf = data.qpos[qpos_addr('right_shoulder_pitch_joint')]
    drift = abs(qf - q0)
    print(f"[{label}] sp0={sp0:.1f} scale={scale:+.1f} drift2s={drift:8.4f}  final sp={qf:7.3f}  "
          f"qp0={qr[dof_addr('right_shoulder_pitch_joint')]:+7.3f}")
    print("        ", " | ".join(hist))
    return drift

print("\n=== free fall from raised pose (0 torque) ===")
run("FALL+1.0", 1.0, 0.0)
run("FALL+1.5", 1.5, 0.0)
print("\n=== raised pose + qfrc_bias FF ===")
run("FF+1.0", 1.0, +1.0)
run("FF-1.0", 1.0, -1.0)
