import mujoco, numpy as np
M = "/mnt/d/Users/Lenovo/Desktop/g1_right_arm_reach/config/models/g1_23dof_revo2_arm_only.xml"
m = mujoco.MjModel.from_xml_path(M)
d = mujoco.MjData(m)
print("nq:", m.nq, "nv:", m.nv)
names = ["right_shoulder_pitch_joint","right_shoulder_roll_joint",
         "right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint"]
vals  = [0.0,-0.35,0.0,0.0,0.35]
for nm, val in zip(names, vals):
    jid = m.joint(nm).id
    d.qpos[m.jnt_qposadr[jid]] = val
mujoco.mj_forward(m,d)

tcp_site = m.site("right_revo2_tcp").id
R_tcp = d.site_xmat[tcp_site].reshape(3,3)
tool_world = R_tcp @ np.array([0.,0.,1.])

wrist_bid = m.body("right_wrist_roll_link").id
R_wrist = d.xmat[wrist_bid].reshape(3,3)
tool_in_wrist = R_wrist.T @ tool_world
print("tool axis in wrist frame:", np.round(tool_in_wrist,4))

jax = m.joint("right_wrist_roll_joint").axis
print("wrist_roll joint axis (body-local):", jax)
print("dot(tool_in_wrist, wrist_roll_axis) =", round(float(np.dot(tool_in_wrist,jax)),4))
print("=> 1.0 means aligned (roll drives wrist), 0.0 means perpendicular (roll CANNOT drive wrist)")

J = np.zeros((6, m.nv))
Jp = np.zeros((3, m.nv))
mujoco.mj_jacSite(m, d, Jp, J[3:6], tcp_site)
J[0:3] = Jp
print("\narm joint roll-row (J[3:6,:] projected on tool axis):")
for nm in names:
    jid = m.joint(nm).id
    dv = m.jnt_dofadr[jid]
    w = J[3:6, dv]
    print(f"  {nm:38s} roll_row={np.dot(w,tool_world):+.4f}  |w|={np.linalg.norm(w):.3f}")

print("\nTCP pos:", np.round(d.site(tcp_site).xpos,4))
shoulder = d.xpos[m.body("right_shoulder_pitch_link").id]
print("shoulder pos:", np.round(shoulder,4))
print("TCP-shoulder:", np.round(d.site(tcp_site).xpos - shoulder,4))

# perturbation: rotate wrist_roll +0.1, measure toolRoll-like change
def tool_roll(R):
    # local Z of tcp
    return R[:,2]
R0 = R_tcp.copy()
q0 = d.qpos[m.jnt_qposadr[m.joint("right_wrist_roll_joint").id]]
d.qpos[m.jnt_qposadr[m.joint("right_wrist_roll_joint").id]] += 0.1
mujoco.mj_forward(m,d)
R1 = d.site_xmat[tcp_site].reshape(3,3)
rel = R0.T @ R1   # rotation from start to end, expressed in start frame
# axis-angle of rel
import math
angle = math.acos(np.clip((np.trace(rel)-1)/2,-1,1))
axis = np.array([rel[2,1]-rel[1,2], rel[0,2]-rel[2,0], rel[1,0]-rel[0,1]])/(2*math.sin(angle)) if abs(math.sin(angle))>1e-9 else np.zeros(3)
print("\nwrist_roll +0.1 rad perturbation:")
print("  rotation axis (start frame):", np.round(axis,4))
print("  tool axis in start frame:", np.round(tool_world,4))
print("  dot(axis, tool_axis) =", round(float(np.dot(axis, tool_world)),4))
d.qpos[m.jnt_qposadr[m.joint("right_wrist_roll_joint").id]] = q0
