#!/usr/bin/env python3
"""Empirically find the gravity-FF scale that holds the arm static at ready pose
(with pelvis welded). Sweep ctrl = scale * qfrc_bias, measure arm drift 3s."""
import mujoco
import numpy as np
import sys

scene = sys.argv[1]
with open(scene, encoding='utf-8') as f:
    text = f.read()
weld = '    <weld name="weld_pelvis" body1="pelvis" body2="world" solref="0.002 2"/>\n'
if "<equality>" in text:
    idx = text.rindex("</equality>")
    text = text[:idx] + weld + text[idx:]
else:
    text = text.rstrip()[:-len("</mujoco>")].rstrip() + "\n  <equality>\n" + weld + "  </equality>\n</mujoco>\n"
m = mujoco.MjModel.from_xml_string(text)
d = mujoco.MjData(m)
# also hold the hand: give hand masters their gravity FF so the hand doesn't flop
hand_active = ["R_thumb_proximal_yaw_joint","R_thumb_proximal_pitch_joint","R_index_proximal_joint",
        "R_middle_proximal_joint","R_ring_proximal_joint","R_pinky_proximal_joint"]
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
    if jn in arm or jn in hand_active: act[jn] = a

def setup():
    d.qpos[:] = m.qpos0[:]; d.qvel[:] = 0.0
    d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
    d.qpos[qpa("right_elbow_joint")] = 0.3
    for a in range(m.nu): d.ctrl[a] = 0.0
    mujoco.mj_forward(m, d)

def qfrc():
    qv = d.qvel.copy(); d.qvel[:] = 0.0
    mujoco.mj_forward(m, d); q = d.qfrc_bias.copy(); d.qvel[:] = qv
    return q

def run(scale):
    setup()
    qr = qfrc()
    for j in arm + hand_active: d.ctrl[act[j]] = scale * qr[da(j)]
    q0 = np.array([d.qpos[qpa(j)] for j in arm])
    n = int(3.0 / m.opt.timestep)
    for i in range(n):
        for addr, dof, val in locked:
            d.qpos[addr] = val; d.qvel[dof] = 0.0
        mujoco.mj_step(m, d)
    qf = np.array([d.qpos[qpa(j)] for j in arm])
    return np.max(np.abs(qf - q0))

setup(); qr = qfrc()
print("qfrc_bias at ready (raw):", [round(qr[da(j)],3) for j in arm])
print("\nscale sweep (ctrl = scale * qfrc_bias), arm drift 3s:")
best = (1e9, None)
for scale in np.arange(-1.2, 1.3, 0.2):
    dr = run(scale)
    if dr < best[0]: best = (dr, scale)
    print(f"  scale={scale:+.1f}  drift={dr:8.4f} rad")
print(f"\nBEST scale = {best[1]:+.1f} (drift {best[0]:.4f})")
print("=> gravity FF should be: ctrl = %.1f * qfrc_bias" % best[1])
