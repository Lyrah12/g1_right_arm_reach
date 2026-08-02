#!/usr/bin/env python3
"""Compare right-arm link mass/COM between the MJCF scene and the (patched) URDF."""
import re
import mujoco

URDF = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf'
SCENE = '/home/lyrah/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml'

LINKS = ['right_shoulder_pitch_link', 'right_shoulder_roll_link', 'right_shoulder_yaw_link',
         'right_elbow_link', 'right_wrist_roll_link', 'right_wrist_pitch_link',
         'right_wrist_yaw_link']

m = mujoco.MjModel.from_xml_path(SCENE)
print(f"{'link':<30}{'MJCF mass':>10}{'URDF mass':>10}{'MJCF com':>30}{'URDF com':>30}")
for ln in LINKS:
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, ln)
    mj_mass = m.body_mass[bid] if bid >= 0 else float('nan')
    # MJCF inertial origin (COM) in body frame: m.body_inertia gives ixx,..; COM is in
    # the inertial origin — MuJoCo stores it implicitly. We can read d.xipos after forward
    # but that's world; instead read the body's inertial pose from m.body_ipos (in body frame).
    mj_ipos = m.body_ipos[bid] if bid >= 0 else (float('nan'),) * 3
    urdf = open(URDF).read()
    mm = re.search(r'<link name="' + ln + r'">(.*?)</link>', urdf, re.S)
    inert = re.search(
        r'<inertial>.*?<origin xyz="([0-9.eE+-]+) ([0-9.eE+-]+) ([0-9.eE+-]+)".*?'
        r'<mass value="([0-9.eE+-]+)"', mm.group(1), re.S) if mm else None
    if inert:
        u_mass = inert.group(4)
        u_com = f"({inert.group(1)},{inert.group(2)},{inert.group(3)})"
    else:
        u_mass, u_com = '?', '?'
    print(f"{ln:<30}{mj_mass:>10.5f}{u_mass:>10}{str(mj_ipos):>30}{u_com:>30}")
