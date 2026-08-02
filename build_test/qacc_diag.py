#!/usr/bin/env python3
"""Forward-dynamics diagnostic: qacc of the right arm at ready pose with
ctrl=0 and with gravity FF applied. Reveals if the arm is dynamically free."""
import mujoco
import numpy as np
import sys

m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)

def qid(j): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
def qpa(j): return m.jnt_qposadr[qid(j)]
def da(j): return m.jnt_dofadr[qid(j)]
arm = ["right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
       "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]

d.qpos[:] = m.qpos0[:]
d.qpos[qpa("right_shoulder_pitch_joint")] = -0.3
d.qpos[qpa("right_elbow_joint")] = 0.3
d.qvel[:] = 0.0
for a in range(m.nu): d.ctrl[a] = 0.0
mujoco.mj_forward(m, d)

print("ready pose: sp=%.3f elb=%.3f" % (d.qpos[qpa("right_shoulder_pitch_joint")], d.qpos[qpa("right_elbow_joint")]))
print("ncon (contacts):", d.ncon)
print("\nctrl=0 → qacc at ready:")
for j in arm:
    print("  %-35s qfrc_bias=%+8.3f  qacc=%+10.2f" % (j, d.qfrc_bias[da(j)], d.qacc[da(j)]))

# Apply ctrl = -qfrc_bias (bridge's current published sign) to all arm motors
act = {}
for a in range(m.nu):
    trn = m.actuator_trnid[a][0]
    jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, trn)
    if jn in arm:
        act[jn] = a
for j in arm:
    d.ctrl[act[j]] = -d.qfrc_bias[da(j)]
mujoco.mj_forward(m, d)
print("\nctrl = -qfrc_bias → qacc at ready:")
for j in arm:
    print("  %-35s ctrl=%+7.3f qacc=%+10.2f" % (j, d.ctrl[act[j]], d.qacc[da(j)]))

# Apply ctrl = +qfrc_bias
for j in arm:
    d.ctrl[act[j]] = +d.qfrc_bias[da(j)]
mujoco.mj_forward(m, d)
print("\nctrl = +qfrc_bias → qacc at ready:")
for j in arm:
    print("  %-35s ctrl=%+7.3f qacc=%+10.2f" % (j, d.ctrl[act[j]], d.qacc[da(j)]))
