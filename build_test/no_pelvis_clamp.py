#!/usr/bin/env python3
"""Fix candidate: clamp locked joints but NOT pelvis. Does the robot stand
(feet hold it) AND does the arm move normally?
Also: gravity FF (correct sign) holds the arm static?"""
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

act = {}
for a in range(m.nu):
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a][0])
    if jn in arm:
        act[jn] = a

def setup(sp, elb):
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    d.qpos[qpa("right_shoulder_pitch_joint")] = sp
    d.qpos[qpa("right_elbow_joint")] = elb
    for a in range(m.nu): d.ctrl[a] = 0.0
    mujoco.mj_forward(m, d)

def qfrc():
    qv = d.qvel.copy(); d.qvel[:] = 0.0
    mujoco.mj_forward(m, d); q = d.qfrc_bias.copy(); d.qvel[:] = qv
    return q

def run(label, sp0, elb0, ff):
    setup(sp0, elb0)
    if ff != 0.0:
        qr = qfrc()
        for j in arm: d.ctrl[act[j]] = ff * qr[da(j)]
    q0p = np.array(d.qpos[0:3])
    q0 = d.qpos[qpa("right_shoulder_pitch_joint")]
    n = int(2.0 / m.opt.timestep)
    for i in range(n):
        for addr, dof, val in locked:
            d.qpos[addr] = val; d.qvel[dof] = 0.0
        mujoco.mj_step(m, d)
    pelv_drift = np.linalg.norm(d.qpos[0:3] - q0p)
    sp = d.qpos[qpa("right_shoulder_pitch_joint")]
    elb = d.qpos[qpa("right_elbow_joint")]
    print("  [%-20s] sp drift 2s=%+.4f (%.3f->%.3f) elb=%.3f  pelvis_drift=%.4f m" % (
        label, sp-q0, q0, sp, elb, pelv_drift))

print("locked clamped, pelvis FREE:")
run("fall sp=1.0", 1.0, 0.0, 0.0)     # arm should fall
run("ready 0torque", -0.3, 0.3, 0.0)  # ready pose, no FF
print("gravity FF applied (hold ready):")
run("ready ff=-bias", -0.3, 0.3, -1.0)
run("ready ff=+bias", -0.3, 0.3, +1.0)
