#!/usr/bin/env python3
"""Compare MuJoCo gravity (incl. Inspire hand) vs Pinocchio gravity (G1-only URDF)
at the converged reach config. The difference = the gravity-FF error a real robot
would see with the current (G1-only) URDF — i.e. the amount the integral/PD would
have to absorb on hardware, which sim currently hides by using MuJoCo gravity."""
import sys
import numpy as np

SCENE = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml'
URDF = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf'  # patched (5010 + hand)

ARM = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint",
       "right_shoulder_yaw_joint", "right_elbow_joint"]
WRIST = ["right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"]

# ACTUAL converged config captured from the sim (capture_converged.sh), incl. wrist
# (wrist_yaw=1.12 rad — the wrist is yawed significantly, which matters for gravity).
Q_ARM = np.array([-0.0626, -0.2869, -0.3036, -0.1485])
Q_WRIST = np.array([0.0794, 0.3352, 1.1167])

# ---- MuJoCo ----
import mujoco
m = mujoco.MjModel.from_xml_path(SCENE)
d = mujoco.MjData(m)
def qpa(j): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
def da(j): return m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
for j, q in zip(ARM, Q_ARM): d.qpos[qpa(j)] = q
for j, q in zip(WRIST, Q_WRIST): d.qpos[qpa(j)] = q
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)
g_mj = {j: float(d.qfrc_bias[da(j)]) for j in ARM + WRIST}

# ---- Pinocchio (G1-only URDF), EXACTLY as C++ RobotModel does it ----
# fixed base (no FreeFlyer) + buildReducedModel keeping only arm+wrist joints.
import pinocchio as pin
full = pin.buildModelFromUrdf(URDF)          # no root type → fixed base (like C++)
names_full = [n for n in full.names]
keep = ARM + WRIST
lock_ids = []
for i, nm in enumerate(names_full):
    if i == 0:
        continue  # universe
    if nm not in keep:
        lock_ids.append(i)
model = pin.buildReducedModel(full, lock_ids, pin.neutral(full))
names = [n for n in model.names]
print("Reduced Pinocchio: nq=%d nv=%d, joints=%s" % (model.nq, model.nv, names))
idx = {}
for i, nm in enumerate(names):
    idx[nm] = i
data = model.createData()
q = np.zeros(model.nq)
for j, v in zip(ARM + WRIST, list(Q_ARM) + list(Q_WRIST)):
    q[model.joints[idx[j]].idx_q] = v
g_pin_vec = pin.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
g_pin = {j: float(g_pin_vec[model.joints[idx[j]].idx_v]) for j in ARM + WRIST}

print("\n=== Arm gravity (N·m) at converged config ===")
print(f"{'joint':<28}{'MuJoCo(hand)':>12}{'Pinocchio(G1)':>14}{'diff':>10}")
for j in ARM + WRIST:
    print(f"{j:<28}{g_mj[j]:>12.3f}{g_pin[j]:>14.3f}{g_mj[j]-g_pin[j]:>10.3f}")

diff_arm = max(abs(g_mj[j] - g_pin[j]) for j in ARM)
print(f"\nMax |MuJoCo−Pinocchio| over arm joints = {diff_arm:.3f} N·m")
