"""Cross-check URDF (Pinocchio) vs MJCF (MuJoCo) for the Revo2 arm, and report
achieved TCP vs target for a known joint configuration."""
import numpy as np
import mujoco

# --- MuJoCo side ---
M = "/mnt/d/Users/Lenovo/Desktop/g1_right_arm_reach/config/models/g1_23dof_revo2_arm_only.xml"
m = mujoco.MjModel.from_xml_path(M)
d = mujoco.MjData(m)
NAMES = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint",
         "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint"]

def set_q(m, d, qvals):
    for nm, v in zip(NAMES, qvals):
        d.qpos[m.jnt_qposadr[m.joint(nm).id]] = v
    mujoco.mj_forward(m, d)
    return d.site("right_revo2_tcp").xpos.copy(), d.site_xmat[m.site("right_revo2_tcp").id].reshape(3, 3).copy()

# --- Pinocchio side ---
import pinocchio as pin
import os
URDF = "/mnt/d/Users/Lenovo/Desktop/g1_right_arm_reach/config/models/g1_23dof_revo2.urdf"
model = pin.buildModelFromUrdf(URDF, pin.JointModelFreeFlyer())  # root freeflyer to handle fixed root
# buildModelFromUrdf defaults: root is the base link. Use buildModelFromUrdf with root_joint=None keeps root fixed.
model2 = pin.buildModelFromUrdf(URDF)
data2 = pin.Data(model2)
print("URDF joint count:", model2.njoints, "nq:", model2.nq)
# arm joints in URDF model
for nm in NAMES:
    if model2.existJointName(nm):
        jid = model2.getJointId(nm)
        print(f"  {nm}: nq={model2.joints[jid].nq} idx_q={model2.joints[jid].idx_q}")
    else:
        print(f"  {nm}: MISSING from URDF model")
# tcp frame
tcp = model2.getFrameId("right_revo2_tcp")

cases = {
    "hold":       [0.0, -0.35, 0.0, 0.0, 0.35],
    "run1_final": [-0.398, -0.087, -0.339, -0.300, -0.027],
    "target_est": [-0.4, -0.1, -0.34, -0.3, 0.0],
}
print("\n=== TCP position cross-check (URDF world == MuJoCo world with pelvis at z=0) ===")
for label, q in cases.items():
    qp, Rm = set_q(m, d, q)
    qpin = np.zeros(model2.nq)
    for nm, v in zip(NAMES, q):
        jid = model2.getJointId(nm)
        qpin[model2.joints[jid].idx_q] = v
    pin.forwardKinematics(model2, data2, qpin)
    pin.updateFramePlacements(model2, data2)
    pm = data2.oMf[tcp]
    print(f"{label:12s} MJ: {np.round(qp,3)}   URDF: {np.round(pm.translation,3)}   diff={np.linalg.norm(qp-pm.translation):.4f} m")

# --- achieved TCP-shoulder for run1 final, vs the commanded target ---
q = cases["run1_final"]
qp, Rm = set_q(m, d, q)
shoulder = d.xpos[m.body("right_shoulder_pitch_link").id]
print("\n=== run1_final achieved vs target ===")
print("TCP world:   ", np.round(qp, 3))
print("shoulder:    ", np.round(shoulder, 3))
print("TCP-shoulder:", np.round(qp - shoulder, 3))
print("target (shoulder-rel): [0.30, -0.10, 0.00]  -> err =", np.round(np.linalg.norm((qp-shoulder)-np.array([0.30,-0.10,0.0])), 4))
# tool axis direction at final config
tool = Rm @ np.array([0., 0., 1.])
print("tool axis (world):", np.round(tool, 3))
print("tool axis (shoulder rel yaw/pitch): az=%.1f el=%.1f deg" % (
    np.degrees(np.arctan2(tool[1], tool[0])), np.degrees(np.arcsin(tool[2]))))
