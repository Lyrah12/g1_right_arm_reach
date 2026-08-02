#!/usr/bin/env python3
"""Definitive MuJoCo sign convention with a single pendulum.

Joint axis = +Y, gravity = -Z. Pendulum hangs down at q=0 (stable).
Apply ctrl = scale*qfrc_bias via a motor, measure if it holds static.
- Pendulum at q=0 (hanging) is a stable equilibrium: 0 torque holds it.
- Tilt it to q=+0.3: gravity pulls it back. The hold torque should cancel gravity.
Find the scale that holds q=0.3 static. This reveals the sign convention.
"""
import mujoco
import numpy as np

xml = """
<mujoco model="pendulum">
  <option integrator="implicitfast" timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <light name="l" pos="0 0 3"/>
    <body name="pivot" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" range="-1.5 1.5" damping="0.05" armature="0.001"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.03" mass="1.0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="m" joint="hinge" forcerange="-10 10"/>
  </actuator>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
hinge_dof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge")]
act_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "m")

def run(scale, tilt=0.3):
    d.qpos[:] = 0.0
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge")]] = tilt
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)
    # qfrc_bias at this pose (zero vel)
    qv = d.qvel.copy(); d.qvel[:] = 0.0
    mujoco.mj_forward(m, d); qb = d.qfrc_bias[hinge_dof].copy(); d.qvel[:] = qv
    d.ctrl[act_id] = scale * qb
    q0 = d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge")]]
    n = int(1.0 / m.opt.timestep)
    for i in range(n):
        mujoco.mj_step(m, d)
    qf = d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge")]]
    return qf - q0, qb

print("tilt pendulum to +0.3 rad, 1s. qfrc_bias(q=0.3):")
# print bias at tilt
d.qpos[:] = 0.0
d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge")]] = 0.3
d.qvel[:] = 0.0
mujoco.mj_forward(m, d)
qb = d.qfrc_bias[hinge_dof]
print(f"  qfrc_bias at q=0.3 = {qb:+.3f}")

for scale in [-1.0, -0.5, 0.0, 0.5, 1.0]:
    drift, qb = run(scale)
    print(f"  scale={scale:+.1f} (ctrl={scale*qb:+.3f})  drift 1s = {drift:+.5f} rad")
