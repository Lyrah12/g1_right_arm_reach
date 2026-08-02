#!/usr/bin/env python3
"""What cancels the arm's gravity? Check qfrc_constraint + full M solve."""
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

print("qfrc breakdown for arm joints (ctrl=0):")
for j in arm:
    dof = da(j)
    print("  %-35s bias=%+8.3f act=%+8.3f con=%+8.3f applied=%+8.3f qacc=%+9.2f"
          % (j, d.qfrc_bias[dof], d.qfrc_actuator[dof], d.qfrc_constraint[dof],
             d.qfrc_applied[dof], d.qacc[dof]))

# full dense M solve: a = M^-1 (bias + actuator)
dense_M = np.zeros((m.nv, m.nv))
mujoco.mj_fullM(m, dense_M, d.M)
f = d.qfrc_bias + d.qfrc_actuator
a_unconstrained = np.linalg.solve(dense_M, f)
print("\nunconstrained M^-1*(bias+act) (no constraint solver):")
for j in arm:
    print("  %-35s a=%+9.2f" % (j, a_unconstrained[da(j)]))

# what do the contacts/equality hold? print equality constraint objects
print("\nequality constraints:")
for e in range(m.neq):
    print("  eq[%d] type=%s name=%s obj1=%s obj2=%s" % (
        e, m.eq_type[e], mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, e),
        mujoco.mj_id2name(m, m.eq_objtype[e], m.eq_obj1id[e]),
        mujoco.mj_id2name(m, m.eq_objtype[e], m.eq_obj2id[e])))
