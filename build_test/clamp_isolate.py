#!/usr/bin/env python3
"""Isolate which clamping freezes the arm. 0 torque, raised arm, 1s fall.
Compare: (a) no clamp, (b) clamp pelvis only, (c) clamp pelvis+locked qpos only,
(d) clamp pelvis+locked qpos+qvel (full bridge)."""
import mujoco
import numpy as np
import sys

m = mujoco.MjModel.from_xml_path(sys.argv[1])
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

pelv_pos = np.array(d.qpos[0:3]); pelv_quat = np.array(d.qpos[3:7])

def setup():
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    d.qpos[qpa("right_shoulder_pitch_joint")] = 1.0
    d.qpos[qpa("right_elbow_joint")] = 0.0
    for a in range(m.nu): d.ctrl[a] = 0.0
    mujoco.mj_forward(m, d)

def run(label, clamp_mode):
    setup()
    q0 = d.qpos[qpa("right_shoulder_pitch_joint")]
    n = int(1.0 / m.opt.timestep)
    for i in range(n):
        if clamp_mode in ("pelvis","pelvis+locked_q","pelvis+locked_qv"):
            d.qpos[0:3] = pelv_pos; d.qpos[3:7] = pelv_quat; d.qvel[0:6] = 0.0
        if clamp_mode in ("pelvis+locked_q","pelvis+locked_qv"):
            for addr, dof, val in locked:
                d.qpos[addr] = val
                if clamp_mode == "pelvis+locked_qv":
                    d.qvel[dof] = 0.0
        mujoco.mj_step(m, d)
    qf = d.qpos[qpa("right_shoulder_pitch_joint")]
    elb = d.qpos[qpa("right_elbow_joint")]
    print("  [%-16s] sp drift 1s = %+.4f  (sp %.3f -> %.3f, elb %.3f)" % (
        clamp_mode, qf-q0, q0, qf, elb))

print("arm raised (sp=1.0), 0 torque, 1 s fall:")
run("A","none")
run("B","pelvis")
run("C","pelvis+locked_q")
run("D","pelvis+locked_qv")
