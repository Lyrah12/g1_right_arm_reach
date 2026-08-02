#!/usr/bin/env python3
"""
inspire_urdf_to_mjcf.py — Convert Inspire RH56 (DFQ right hand) URDF -> MJCF fragments.

Generates:
  <out_dir>/inspire_hand_right_body.xml     body tree rooted at R_hand_base_link
                                            (mount pos/quat baked in; inject under right_wrist_yaw_link)
  <out_dir>/inspire_hand_right_assets.xml   <asset> mesh definitions (absolute file paths)

Conventions (verified against MuJoCo 3.11 URDF import ground truth):
  - URDF joint `axis` is expressed in the child-body (joint) frame -> MJCF `axis` = URDF axis VERBATIM
    (no R(origin_rpy)^T transform; MuJoCo keeps it verbatim).
  - URDF joint `origin` xyz/rpy -> MJCF child `<body pos quat>` (quat = MuJoCo scalar-first w,x,y,z).
  - `<inertial>` kept as `<inertial>` (full tensor).
  - mesh geoms `density="0"` (mass comes from `<inertial>` only) — two geoms: visual + collision.
  - fixed mount joint (R_base_link_joint) folded into R_hand_base_link body pos/quat.
  - `<mimic>` dropped; coupling ratios printed for the inject script's `<equality>` constraints.

Usage:
  python3 inspire_urdf_to_mjcf.py <DFQ_right_hand.urdf> <out_dir>
"""

import sys
import os
import xml.etree.ElementTree as ET
import numpy as np

# ============================================================================
# Mount transform (from URDF R_base_link_joint fixed joint)
# ============================================================================
MOUNT_XYZ = np.array([0.0415, 0.0, 0.0])
MOUNT_RPY = np.array([np.pi, 0.0, -np.pi / 2])

# 6 active joints (get <motor> in the scene) vs 6 slave joints (equality-driven)
ACTIVE_JOINTS = [
    "R_thumb_proximal_yaw_joint",
    "R_thumb_proximal_pitch_joint",
    "R_index_proximal_joint",
    "R_middle_proximal_joint",
    "R_ring_proximal_joint",
    "R_pinky_proximal_joint",
]

# Mimic coupling (slave -> master, ratio) — to be expressed as <equality> by inject script
MIMIC = [
    ("R_thumb_intermediate_joint", "R_thumb_proximal_pitch_joint", 1.6),
    ("R_thumb_distal_joint",       "R_thumb_proximal_pitch_joint", 2.4),
    ("R_index_intermediate_joint", "R_index_proximal_joint",       1.0),
    ("R_middle_intermediate_joint","R_middle_proximal_joint",      1.0),
    ("R_ring_intermediate_joint",  "R_ring_proximal_joint",        1.0),
    ("R_pinky_intermediate_joint", "R_pinky_proximal_joint",       1.0),
]
SLAVE_JOINTS = [m[0] for m in MIMIC]

# Widened slave ranges (equality drives slave = m*master; URDF slave limits too tight)
SLAVE_RANGE = (-0.5, 2.0)

# Collision model: full STL meshes have huge convex hulls -> the whole hand overlaps
# at rest (108 contacts). Standard fix: mesh geoms are VISUAL ONLY (contype=0),
# grasping contact comes from small primitives: palm box + fingertip spheres.
# Tip sphere = (body_name, local_pos_xyz, radius); palm box = (center, halfsize).
TIP_SPHERES = [
    ("R_thumb_distal",       (0.020, 0.020, 0.000), 0.013),
    ("R_index_intermediate", (0.000, 0.042, 0.000), 0.013),
    ("R_middle_intermediate",(0.000, 0.042, 0.000), 0.013),
    ("R_ring_intermediate",  (0.000, 0.042, 0.000), 0.013),
    ("R_pinky_intermediate", (0.000, 0.040, 0.000), 0.012),
]
PALM_BOX = ((0.000, -0.040, 0.000), (0.035, 0.050, 0.015))  # center, halfsize

FINGER_RGBA = {
    "R_thumb_proximal_base":   "0.85 0.55 0.20 1",
    "R_thumb_proximal":        "0.85 0.55 0.20 1",
    "R_thumb_intermediate":    "0.85 0.55 0.20 1",
    "R_thumb_distal":          "0.85 0.55 0.20 1",
    "R_index_proximal":        "0.60 0.75 0.90 1",
    "R_index_intermediate":    "0.60 0.75 0.90 1",
    "R_middle_proximal":       "0.70 0.80 0.70 1",
    "R_middle_intermediate":   "0.70 0.80 0.70 1",
    "R_ring_proximal":         "0.85 0.75 0.70 1",
    "R_ring_intermediate":     "0.85 0.75 0.70 1",
    "R_pinky_proximal":        "0.80 0.70 0.85 1",
    "R_pinky_intermediate":    "0.80 0.70 0.85 1",
}
BASE_RGBA = "0.45 0.45 0.48 1"

# ============================================================================
# Rotation / quaternion helpers (MuJoCo scalar-first w,x,y,z)
# ============================================================================

def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def roty(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rpy_to_rot(rpy):
    """URDF fixed-axis ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r, p, y = rpy
    return rotz(y) @ roty(p) @ rotx(r)


def rot_to_quat(R):
    """Rotation matrix -> quaternion, MuJoCo scalar-first (w,x,y,z)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    q = np.array([w, x, y, z])
    q /= np.linalg.norm(q)
    return q


def fmt(v, nd=6):
    return ("%." + str(nd) + "f") % float(v)


def fmt_vec(v, nd=6):
    return " ".join(fmt(x, nd) for x in v)


def parse_vec(s):
    return np.array([float(x) for x in s.split()])


# ============================================================================

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 inspire_urdf_to_mjcf.py <DFQ_right_hand.urdf> <out_dir>")
        sys.exit(1)

    urdf_path, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)

    root = ET.parse(urdf_path).getroot()
    links = {ln.get("name"): ln for ln in root.findall("link")}
    joints = {jn.get("name"): jn for jn in root.findall("joint")}

    # child -> (joint_name, parent_name, joint_el) for revolute joints only
    children = {}
    for jname, jn in joints.items():
        if jn.get("type") == "revolute":
            child = jn.find("child").get("link")
            parent = jn.find("parent").get("link")
            children[child] = (jname, parent, jn)

    def children_of(name):
        return [c for c, (_, p, _) in children.items() if p == name]

    def link_meshes(ln):
        names = []
        for geom in ln.findall("visual"):
            m = geom.find("geometry/mesh")
            if m is not None:
                names.append(os.path.basename(m.get("filename")).rsplit(".", 1)[0])
        return names

    def build_body(name, prefix):
        ln = links[name]
        # child body frame = incoming revolute joint origin (xyz/rpy). CRITICAL:
        # without this all finger bodies collapse onto the parent frame.
        pos_attr = ""
        if name in children:
            _jname, _parent, _jn = children[name]
            _origin = _jn.find("origin")
            _xyz = parse_vec(_origin.get("xyz"))
            _rpy = parse_vec(_origin.get("rpy"))
            _quat = rot_to_quat(rpy_to_rot(_rpy))
            pos_attr = f" pos=\"{fmt_vec(_xyz)}\" quat=\"{fmt_vec(_quat, 6)}\""
        out = [f"{prefix}<body name=\"{name}\"{pos_attr}>"]
        inert = ln.find("inertial")
        if inert is not None:
            i_o = inert.find("origin")
            i_pos = parse_vec(i_o.get("xyz")) if i_o is not None else np.zeros(3)
            i_rpy = parse_vec(i_o.get("rpy")) if i_o is not None else np.zeros(3)
            i_mass = float(inert.find("mass").get("value"))
            it = inert.find("inertia")
            iq = rot_to_quat(rpy_to_rot(i_rpy))
            # diagonal inertia only — DFQ off-diagonal terms are ~2% of diagonal, negligible
            ixx = float(it.get("ixx")); iyy = float(it.get("iyy")); izz = float(it.get("izz"))
            out.append(f"{prefix}  <inertial pos=\"{fmt_vec(i_pos)}\" quat=\"{fmt_vec(iq, 5)}\" "
                       f"mass=\"{fmt(i_mass, 7)}\" diaginertia=\"{fmt(ixx, 10)} {fmt(iyy, 10)} {fmt(izz, 10)}\"/>")
        if name in children:
            jname, parent, jn = children[name]
            j_axis = parse_vec(jn.find("axis").get("xyz"))
            j_lim = jn.find("limit")
            lo, hi = float(j_lim.get("lower")), float(j_lim.get("upper"))
            if jname in SLAVE_JOINTS:  # equality-driven slave: widen to cover master_range x ratio
                lo, hi = SLAVE_RANGE
            out.append(f"{prefix}  <joint name=\"{jname}\" pos=\"0 0 0\" axis=\"{fmt_vec(j_axis)}\" "
                       f"range=\"{fmt(lo)} {fmt(hi)}\" armature=\"0.01\"/>")
        rgba = FINGER_RGBA.get(name, BASE_RGBA)
        for mesh_name in link_meshes(ln):
            # mesh = VISUAL ONLY (full-STL convex hulls massively overlap at rest)
            out.append(f"{prefix}  <geom type=\"mesh\" mesh=\"{mesh_name}\" contype=\"0\" conaffinity=\"0\" "
                       f"group=\"1\" density=\"0\" rgba=\"{rgba}\"/>")
        # collision primitives (grasping contact) — rgba alpha=0: INVISIBLE but still collide.
        # These are contact proxies (palm box + fingertip spheres); rendering them translucent
        # (old: box 0.5 gray / spheres 0.6 orange) overlaid the colored finger meshes with
        # semi-transparent blobs. Keep contype=1 for grasping, hide the visuals.
        if name == "R_hand_base_link":
            cx, cy, cz = PALM_BOX[0]
            hx, hy, hz = PALM_BOX[1]
            out.append(f"{prefix}  <geom type=\"box\" pos=\"{cx} {cy} {cz}\" size=\"{hx} {hy} {hz}\" "
                       f"density=\"0\" rgba=\"0 0 0 0\" contype=\"1\" conaffinity=\"0\"/>")
        for body_name, tip_pos, radius in TIP_SPHERES:
            if name == body_name:
                out.append(f"{prefix}  <geom type=\"sphere\" pos=\"{tip_pos[0]} {tip_pos[1]} {tip_pos[2]}\" "
                           f"size=\"{radius}\" density=\"0\" rgba=\"0 0 0 0\" "
                           f"contype=\"1\" conaffinity=\"0\"/>")
        for cname in children_of(name):
            out.append(build_body(cname, prefix + "  "))
        out.append(f"{prefix}</body>")
        return "\n".join(out)

    # ---- Body tree: R_hand_base_link carries the mount transform ----
    mq = rot_to_quat(rpy_to_rot(MOUNT_RPY))
    base_inner = build_body("R_hand_base_link", "    ").split("\n")
    parts = [
        "<!-- Inspire RH56 right hand — body tree (auto-generated). Inject under right_wrist_yaw_link. -->",
        "<mujoco>",
        "  <worldbody>",
        f"    <body name=\"R_hand_base_link\" pos=\"{fmt_vec(MOUNT_XYZ)}\" quat=\"{fmt_vec(mq, 6)}\">",
        *base_inner[1:-1],
        "    </body>",
        "  </worldbody>",
        "</mujoco>",
    ]
    out_body = os.path.join(out_dir, "inspire_hand_right_body.xml")
    with open(out_body, "w") as f:
        f.write("\n".join(parts) + "\n")
    print(f"[OK] body tree   -> {out_body}")

    # ---- Assets: mesh definitions with paths relative to <out_dir> (portable across machines) ----
    meshes = set()
    for ln in links.values():
        meshes.update(link_meshes(ln))
    mesh_dir = os.path.abspath(os.path.join(urdf_path, os.pardir, "meshes"))
    out_abs = os.path.abspath(out_dir)
    asset_parts = [
        "<!-- Inspire RH56 right hand — mesh assets (auto-generated). Paths relative to this file's dir. -->",
        "<mujoco>",
        "  <asset>",
    ]
    for m in sorted(meshes):
        rel = os.path.relpath(os.path.join(mesh_dir, m + ".STL"), out_abs).replace(os.sep, "/")
        asset_parts.append(f"    <mesh name=\"{m}\" content_type=\"model/stl\" file=\"{rel}\"/>")
    asset_parts += ["  </asset>", "</mujoco>"]
    out_assets = os.path.join(out_dir, "inspire_hand_right_assets.xml")
    with open(out_assets, "w") as f:
        f.write("\n".join(asset_parts) + "\n")
    print(f"[OK] assets      -> {out_assets}")

    # ---- Report coupling for inject script ----
    print("\n=== Underactuation coupling (for <equality>) ===")
    for slave, master, ratio in MIMIC:
        print(f"  {slave:32s} = {ratio} x {master}")
    print(f"  Active (motor): {len(ACTIVE_JOINTS)} | Slave (equality): {len(SLAVE_JOINTS)}")

    # ---- Print resolved child frames for validation (vs MuJoCo URDF dump) ----
    print("\n=== Direct children of R_hand_base_link (relative frames) ===")
    for cname in children_of("R_hand_base_link"):
        cjn = children[cname][2]
        c_xyz = parse_vec(cjn.find("origin").get("xyz"))
        c_rpy = parse_vec(cjn.find("origin").get("rpy"))
        c_quat = rot_to_quat(rpy_to_rot(c_rpy))
        print(f"  {cname:22s} pos=[{fmt_vec(c_xyz)}] quat=[{fmt_vec(c_quat, 6)}]")


if __name__ == "__main__":
    main()
