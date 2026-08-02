#!/usr/bin/env python3
"""Print contact pairs at the ready pose to find what pins the arm."""
import mujoco
import numpy as np
import sys

m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)

def qpa(j): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
d.qpos[:] = m.qpos0[:]
d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
d.qpos[qpa("right_elbow_joint")] = 0.3
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)

print(f"ncon = {d.ncon}")
for i in range(d.ncon):
    c = d.contact[i]
    g1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
    g2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
    print(f"  contact {i}: {g1}  <->  {g2}   dist={c.dist:.4f}")

# EE / hand body world pos
for bname in ["right_wrist_yaw_link", "R_hand_base_link"]:
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bname)
    if b >= 0:
        print(f"\n{bname} pos = {np.round(d.xpos[b], 3)}")
# key body positions
for bname in ["pelvis", "torso_link", "right_thigh_link", "right_hip_pitch_link"]:
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bname)
    if b >= 0:
        print(f"{bname:22s} pos = {np.round(d.xpos[b], 3)}")
