#!/usr/bin/env python3
"""Pure fall test: NO clamping at all. Set arm raised, 0 torque, step 1s.
If the arm falls -> dynamics are fine (clamping was the culprit).
If it stays -> model/gravity issue."""
import mujoco
import numpy as np
import sys

m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)
print("gravity:", m.opt.gravity)

def qpa(j): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
arm = ["right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
       "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]

# pure raised pose, no clamp
d.qpos[:] = m.qpos0[:]
d.qpos[qpa("right_shoulder_pitch_joint")] = 1.0
d.qpos[qpa("right_elbow_joint")] = 0.0
d.qvel[:] = 0.0
for a in range(m.nu): d.ctrl[a] = 0.0
mujoco.mj_forward(m, d)
print("start sp=%.3f elb=%.3f  (no clamping, 0 torque)" % (
    d.qpos[qpa("right_shoulder_pitch_joint")], d.qpos[qpa("right_elbow_joint")]))

n = int(1.0 / m.opt.timestep)
for i in range(n):
    mujoco.mj_step(m, d)
    if i % 200 == 0:
        print("  t=%.2f sp=%+.3f elb=%+.3f spd=%.3f elbvel=%.3f ncon=%d" % (
            i*m.opt.timestep,
            d.qpos[qpa("right_shoulder_pitch_joint")], d.qpos[qpa("right_elbow_joint")],
            d.qvel[m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "right_shoulder_pitch_joint")]],
            d.qvel[m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "right_elbow_joint")]],
            d.ncon))

print("final sp=%.3f elb=%.3f" % (d.qpos[qpa("right_shoulder_pitch_joint")], d.qpos[qpa("right_elbow_joint")]))
