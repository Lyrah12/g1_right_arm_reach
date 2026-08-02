#!/usr/bin/env python3
"""临时验证：完整场景（力矩执行器 + implicitfast + 重力）加载与稳定性。"""
import os
import numpy as np
import mujoco

PKG = "/mnt/d/Users/Lenovo/Desktop/g1_right_arm_reach"
CFG = os.path.join(PKG, "config")

# Assemble a local test scene mirroring g1_inspire_scene.xml's option+actuators
scene = f"""
<mujoco model="inspire_test_scene">
  <visual><global offwidth="1280" offheight="720"/></visual>
  <include file="{PKG}/build_test/g1_29dof_inspire.xml"/>
  <include file="{CFG}/inspire_hand_right_assets.xml"/>
  <option integrator="implicitfast" timestep="0.002" gravity="0 0 -9.81"/>
  <default><joint damping="0.05" armature="0.001"/></default>
  <worldbody>
    <geom name="test_floor" type="plane" size="0 0 0.05" friction="1.0 0.5 0.5" rgba="0.8 0.8 0.8 1"/>
  </worldbody>
  <actuator>
    <motor name="act_r_shoulder_pitch" joint="right_shoulder_pitch_joint" forcerange="-25 25"/>
    <motor name="act_r_shoulder_roll"  joint="right_shoulder_roll_joint"  forcerange="-25 25"/>
    <motor name="act_r_shoulder_yaw"   joint="right_shoulder_yaw_joint"   forcerange="-25 25"/>
    <motor name="act_r_elbow"          joint="right_elbow_joint"          forcerange="-25 25"/>
    <motor name="act_r_wrist_roll"     joint="right_wrist_roll_joint"     forcerange="-25 25"/>
    <motor name="act_r_wrist_pitch"    joint="right_wrist_pitch_joint"    forcerange="-13.4 13.4"/>
    <motor name="act_r_wrist_yaw"      joint="right_wrist_yaw_joint"      forcerange="-13.4 13.4"/>
    <motor name="act_thumb_yaw"   joint="R_thumb_proximal_yaw_joint"   forcerange="-1.5 1.5"/>
    <motor name="act_thumb_pitch" joint="R_thumb_proximal_pitch_joint" forcerange="-1.5 1.5"/>
    <motor name="act_index_prox"  joint="R_index_proximal_joint"       forcerange="-1.5 1.5"/>
    <motor name="act_middle_prox" joint="R_middle_proximal_joint"      forcerange="-1.5 1.5"/>
    <motor name="act_ring_prox"   joint="R_ring_proximal_joint"        forcerange="-1.5 1.5"/>
    <motor name="act_pinky_prox"  joint="R_pinky_proximal_joint"       forcerange="-1.5 1.5"/>
    <position name="act_l_shoulder_pitch" joint="left_shoulder_pitch_joint" kp="100" kv="5"/>
    <position name="act_l_shoulder_roll"  joint="left_shoulder_roll_joint"  kp="100" kv="5"/>
    <position name="act_l_shoulder_yaw"   joint="left_shoulder_yaw_joint"   kp="100" kv="5"/>
    <position name="act_l_elbow"          joint="left_elbow_joint"          kp="100" kv="5"/>
    <position name="act_l_wrist_roll"     joint="left_wrist_roll_joint"     kp="100" kv="5"/>
    <position name="act_l_wrist_pitch"    joint="left_wrist_pitch_joint"    kp="30"  kv="2"/>
    <position name="act_l_wrist_yaw"      joint="left_wrist_yaw_joint"      kp="30"  kv="2"/>
    <position name="act_waist_yaw"   joint="waist_yaw_joint"   kp="300" kv="20"/>
    <position name="act_waist_roll"  joint="waist_roll_joint"  kp="300" kv="20"/>
    <position name="act_waist_pitch" joint="waist_pitch_joint" kp="300" kv="20"/>
    <position name="act_left_hip_pitch"   joint="left_hip_pitch_joint"   kp="600" kv="50"/>
    <position name="act_left_hip_roll"    joint="left_hip_roll_joint"    kp="600" kv="50"/>
    <position name="act_left_hip_yaw"     joint="left_hip_yaw_joint"     kp="600" kv="50"/>
    <position name="act_left_knee"        joint="left_knee_joint"        kp="600" kv="50"/>
    <position name="act_left_ankle_pitch" joint="left_ankle_pitch_joint" kp="600" kv="50"/>
    <position name="act_left_ankle_roll"  joint="left_ankle_roll_joint"  kp="600" kv="50"/>
    <position name="act_right_hip_pitch"   joint="right_hip_pitch_joint"   kp="600" kv="50"/>
    <position name="act_right_hip_roll"    joint="right_hip_roll_joint"    kp="600" kv="50"/>
    <position name="act_right_hip_yaw"     joint="right_hip_yaw_joint"     kp="600" kv="50"/>
    <position name="act_right_knee"        joint="right_knee_joint"        kp="600" kv="50"/>
    <position name="act_right_ankle_pitch" joint="right_ankle_pitch_joint" kp="600" kv="50"/>
    <position name="act_right_ankle_roll"  joint="right_ankle_roll_joint"  kp="600" kv="50"/>
  </actuator>
</mujoco>
"""
wp = os.path.join(CFG, "_test_scene.xml")
open(wp, "w").write(scene)
m = mujoco.MjModel.from_xml_path(wp)
d = mujoco.MjData(m)
print(f"scene loaded: nq={m.nq} nv={m.nv} nbody={m.nbody} njnt={m.njnt} nact={m.nu} neq={m.neq}")
print(f"integrator={mujoco.mjtIntegrator(m.opt.integrator).name} dt={m.opt.timestep}")

# position servos auto-set to qpos0? No — need to init ctrl for position servos at qpos0.
# Simulate: step, check stability with gravity + locked joints.
bad = None
for k in range(3000):
    mujoco.mj_step(m, d)
    if not np.all(np.isfinite(d.qvel)):
        bad = k
        break
print(f"gravity-on stability (3000 steps): {'NaN at step ' + str(bad) if bad else 'STABLE'}")

if not bad:
    # drive hand active joints with torque: curl fingers
    def ai(name):
        return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    # apply small closing torque to index/middle/ring/pinky + thumb
    d.ctrl[ai("act_index_prox")] = -0.5
    d.ctrl[ai("act_middle_prox")] = -0.5
    d.ctrl[ai("act_ring_prox")] = -0.4
    d.ctrl[ai("act_pinky_prox")] = -0.4
    d.ctrl[ai("act_thumb_pitch")] = 0.4
    bad2 = None
    for k in range(3000):
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qvel)):
            bad2 = k; break
    print(f"finger torque test (3000 steps): {'NaN at step ' + str(bad2) if bad2 else 'STABLE'}")
    if not bad2:
        def q(n):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            return d.qpos[m.jnt_qposadr[j]]
        print("finger joint positions after torque:")
        for n in ["R_index_proximal_joint", "R_index_intermediate_joint", "R_middle_proximal_joint", "R_thumb_proximal_pitch_joint"]:
            print(f"  {n:34s} q={q(n):+.4f}")
os.remove(wp)
