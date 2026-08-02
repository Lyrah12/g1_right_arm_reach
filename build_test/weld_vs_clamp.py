#!/usr/bin/env python3
"""Does the hard-clamp pelvis STILL freeze the arm when a weld is present?"""
import mujoco
import numpy as np
import sys

scene = sys.argv[1]
with open(scene, encoding='utf-8') as f:
    text = f.read()
weld = '  <equality>\n    <weld name="weld_pelvis" body1="pelvis" body2="world" solref="0.002 2"/>\n  </equality>\n</mujoco>\n'
if "<equality>" not in text:
    text = text.rstrip()[:-len("</mujoco>")].rstrip() + "\n" + weld
else:
    idx = text.rindex("</equality>")
    text = text[:idx] + '    <weld name="weld_pelvis" body1="pelvis" body2="world" solref="0.002 2"/>\n' + text[idx:]
m = mujoco.MjModel.from_xml_string(text)
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

def run(label, clamp_pelvis):
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    d.qpos[qpa("right_shoulder_pitch_joint")] = 1.0
    d.qpos[qpa("right_elbow_joint")] = 0.0
    for a in range(m.nu): d.ctrl[a] = 0.0
    mujoco.mj_forward(m, d)
    q0 = d.qpos[qpa("right_shoulder_pitch_joint")]
    n = int(1.0 / m.opt.timestep)
    for i in range(n):
        if clamp_pelvis:
            d.qpos[0:3] = pelv_pos; d.qpos[3:7] = pelv_quat; d.qvel[0:6] = 0.0
        for addr, dof, val in locked:
            d.qpos[addr] = val; d.qvel[dof] = 0.0
        mujoco.mj_step(m, d)
    qf = d.qpos[qpa("right_shoulder_pitch_joint")]
    pd = np.linalg.norm(d.qpos[0:3]-pelv_pos)
    print(f"  [weld+{label}] sp drift 1s = {qf-q0:+.4f} (1.0 -> {qf:.3f})  pelvis_drift={pd:.4f}")

print("welded scene, arm raised sp=1.0, 0 torque, 1s:")
run("no_pelvis_clamp", False)
run("pelvis_clamp", True)
