#!/usr/bin/env python3
"""
patch_inspire_urdf.py — Patch the stock G1 URDF so Pinocchio gravity matches the
MuJoCo scene (which carries the 5010 wrist + Inspire RH56 hand).

Why (hardware fidelity, 2026-08-02):
  - reach_node computes gravity FF from Pinocchio. The stock G1 URDF has NO hand,
    so its arm gravity is ~0.5 N·m too weak at the shoulder vs the real robot.
  - In sim this was masked because reach_node preferred MuJoCo gravity (exact).
    On real hardware there is no MuJoCo -> the arm would settle ~1 cm off.
  - Fix: lump the hand (mass 0.1918 kg) AND the MJCF's own yaw-link COM into the
    URDF's right_wrist_yaw_link, so the yaw link's total (mass, COM) equals the
    MJCF's (right_wrist_yaw_link + R_* hand subtree). Gravity depends only on
    mass + COM, so per-joint arm gravity then matches MuJoCo exactly.

IMPORTANT: the yaw-link COM is derived from the MJCF (m.body_ipos of the yaw body
plus the hand subtree's weighted COM at qpos0). Do NOT derive the hand COM from
the hand URDF: inspire_urdf_to_mjcf.py re-poses the hand links, so the URDF frame
!= MJCF frame (URDF gives [0.033,-0.184,-0.019], which overshoots by ~0.9 N·m).

Also: g1_29dof.xml and g1_29dof_rev_1_0.urdf differ in the wrist-link inertial
(MJCF yaw COM [0.0708,-0.0002,0.0016] vs URDF [0.0220,-0.0005,0.0005]), so we
must match the MJCF values, not the URDF's.

Usage: python3 scripts/patch_inspire_urdf.py <stock_urdf> <out_urdf> <scene_xml>
"""
import os
import re
import sys

import mujoco
import numpy as np

PITCH_MASS = "0.684"  # 5010 spec, matches MJCF


def measure_yaw_com(scene_xml: str):
    """Return (mass, com[3]) of MJCF right_wrist_yaw_link + R_* hand subtree,
    expressed in the yaw-link body frame (matches m.body_ipos convention)."""
    m = mujoco.MjModel.from_xml_path(scene_xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    wy = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")
    if wy < 0:
        raise RuntimeError("right_wrist_yaw_link not found in scene")
    # yaw link itself
    m_yaw = float(m.body_mass[wy])
    c_yaw = np.array(m.body_ipos[wy], dtype=float)  # COM in body frame
    # hand subtree
    hand = [i for i in range(m.nbody)
            if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or "").startswith("R_")]
    m_hand = sum(float(m.body_mass[b]) for b in hand)
    com_world = sum(float(m.body_mass[b]) * d.xipos[b] for b in hand) / m_hand
    p_wy = d.xpos[wy]
    R_wy = d.xmat[wy].reshape(3, 3)
    c_hand = R_wy.T @ (com_world - p_wy)
    # combine
    m_tot = m_yaw + m_hand
    c_tot = (m_yaw * c_yaw + m_hand * c_hand) / m_tot
    return m_tot, c_tot


def patch_link(urdf: str, link_name: str, mass: str, com: str | None) -> str:
    pat = re.compile(r'(<link name="' + link_name + r'">)(.*?)(</link>)', re.S)

    def repl(m):
        head, inner, tail = m.group(1), m.group(2), m.group(3)

        def patch_inertial(im):
            body = im.group(0)
            body = re.sub(r'<mass value="[^"]*"', f'<mass value="{mass}"', body, count=1)
            if com is not None:
                body = re.sub(
                    r'(<inertial>\s*<origin xyz=")[^"]*(")',
                    lambda mm: mm.group(1) + com + mm.group(2),
                    body, count=1)
            return body

        inner_new, n = re.subn(r'<inertial>.*?</inertial>', patch_inertial,
                               inner, count=1, flags=re.S)
        if n == 0:
            print(f"ERROR: no <inertial> in link '{link_name}'")
            sys.exit(1)
        return head + inner_new + tail

    new, n = pat.subn(repl, urdf, count=1)
    if n == 0:
        print(f"ERROR: link '{link_name}' not found")
        sys.exit(1)
    return new


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)
    src, dst, scene = sys.argv[1], sys.argv[2], sys.argv[3]

    m_tot, c_tot = measure_yaw_com(scene)
    yaw_mass = f"{m_tot:.6f}"
    yaw_com = " ".join(f"{v:.6f}" for v in c_tot)
    print(f"MJCF yaw_link + hand subtree: mass={m_tot:.4f} kg, com=[{yaw_com}]")

    urdf = open(src).read()
    urdf = patch_link(urdf, "right_wrist_pitch_link", PITCH_MASS, None)
    urdf = patch_link(urdf, "right_wrist_yaw_link", yaw_mass, yaw_com)
    # The stock G1 URDF keeps a right_rubber_hand (0.17 kg) below right_wrist_yaw_link.
    # buildReducedModel folds it into the yaw link, DOUBLE-COUNTING the Inspire hand
    # (which we just lumped in). The MJCF's inject script replaces that body, so zero it.
    urdf = patch_link(urdf, "right_rubber_hand", "0.0", None)

    # The patched URDF lives in config/, so its relative mesh paths ("meshes/...")
    # would resolve to config/meshes/ (doesn't exist) and break RViz. Rewrite them
    # to absolute paths pointing at the stock URDF's mesh directory.
    mesh_dir = os.path.join(os.path.dirname(os.path.abspath(src)), "meshes")
    urdf = urdf.replace('filename="meshes/',
                        f'filename="file://{mesh_dir}/')
    with open(dst, "w") as f:
        f.write(urdf)
    print(f"Patched -> {dst}  (meshes -> {mesh_dir})")


if __name__ == "__main__":
    main()
