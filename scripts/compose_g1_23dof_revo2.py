#!/usr/bin/env python3
"""Build deterministic G1 23DOF + BrainCo Revo2 URDF/MJCF models.

This is an offline model-generation utility. Runtime control remains C++.
The default wrist-to-hand transform is a temporary calibration value and must
be replaced before real TCP accuracy is accepted.
"""

from __future__ import annotations

import argparse
import copy
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


RIGHT_ARM_JOINTS = {
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
}

REVO_ACTIVE_JOINTS = (
    "right_thumb_proximal_joint",
    "right_thumb_metacarpal_joint",
    "right_index_proximal_joint",
    "right_middle_proximal_joint",
    "right_ring_proximal_joint",
    "right_pinky_proximal_joint",
)

# MuJoCo finger dynamics, independent of the physical SDK speed settings.
REVO_SERVO_KP = 30.0
REVO_SERVO_KV = 0.8
REVO_JOINT_DAMPING = 0.05
REVO_JOINT_ARMATURE = 0.001
REVO_JOINT_FRICTION = 0.01

REVO_LIMITS = {
    "right_thumb_metacarpal_joint": (0.0, math.radians(90.0)),
    "right_thumb_proximal_joint": (0.0, math.radians(59.0)),
    "right_thumb_distal_joint": (0.0, math.radians(59.0)),
    "right_index_proximal_joint": (0.0, math.radians(81.0)),
    "right_index_distal_joint": (0.0, 1.63),
    "right_middle_proximal_joint": (0.0, math.radians(81.0)),
    "right_middle_distal_joint": (0.0, 1.63),
    "right_ring_proximal_joint": (0.0, math.radians(81.0)),
    "right_ring_distal_joint": (0.0, 1.63),
    "right_pinky_proximal_joint": (0.0, math.radians(81.0)),
    "right_pinky_distal_joint": (0.0, 1.63),
}

REVO_MIMIC = {
    "right_thumb_distal_joint": ("right_thumb_proximal_joint", 1.0),
    "right_index_distal_joint": ("right_index_proximal_joint", 1.155),
    "right_middle_distal_joint": ("right_middle_proximal_joint", 1.155),
    "right_ring_distal_joint": ("right_ring_proximal_joint", 1.155),
    "right_pinky_distal_joint": ("right_pinky_proximal_joint", 1.155),
}

ADAPTER_LINK = "right_revo2_adapter_link"
ADAPTER_RADIUS = 0.025
ADAPTER_MASS = 0.08


def parse_triplet(text: str) -> tuple[float, float, float]:
    values = tuple(float(v) for v in text.replace(",", " ").split())
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected exactly three numbers")
    return values


def vector_text(values: tuple[float, ...]) -> str:
    return " ".join(f"{v:.12g}" for v in values)


def urdf_rpy_to_mjcf_quat(
    rpy: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """Return MuJoCo wxyz quaternion for a URDF fixed-joint RPY rotation."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def adapter_spec(
    mount_xyz: tuple[float, float, float],
) -> tuple[
    float,
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    """Cylinder geometry joining the wrist origin to the Revo2 mount."""
    x, y, z = mount_xyz
    length = math.sqrt(x * x + y * y + z * z)
    if length < 1.0e-6:
        raise ValueError("Revo2 mount translation is too short for an adapter")
    midpoint = (0.5 * x, 0.5 * y, 0.5 * z)
    yaw = math.atan2(y, x)
    pitch = math.atan2(math.hypot(x, y), z)
    axis_rpy = (0.0, pitch, yaw)
    axial = 0.5 * ADAPTER_MASS * ADAPTER_RADIUS**2
    transverse = ADAPTER_MASS * (3.0 * ADAPTER_RADIUS**2 + length**2) / 12.0
    return length, midpoint, axis_rpy, (transverse, transverse, axial)


def load_xml(path: Path) -> ET.ElementTree:
    if not path.is_file():
        raise FileNotFoundError(path)
    return ET.parse(path)


def write_xml(tree: ET.ElementTree, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    with tempfile.NamedTemporaryFile(
        mode="wb", prefix=path.name, suffix=".tmp", dir=path.parent, delete=False
    ) as stream:
        tmp_path = Path(stream.name)
        tree.write(stream, encoding="utf-8", xml_declaration=True)
        stream.write(b"\n")
    tmp_path.replace(path)


def require(root: ET.Element, query: str, description: str) -> ET.Element:
    element = root.find(query)
    if element is None:
        raise RuntimeError(f"missing {description}: {query}")
    return element


def rewrite_urdf_meshes(root: ET.Element, prefix: str) -> None:
    for mesh in root.findall(".//mesh[@filename]"):
        filename = Path(mesh.attrib["filename"].replace("\\", "/")).name
        mesh.set("filename", f"package://g1_right_arm_reach/config/models/{prefix}/{filename}")


def build_urdf(
    g1_23_path: Path,
    g1_29_path: Path,
    revo_path: Path,
    output_path: Path,
    mount_xyz: tuple[float, float, float],
    mount_rpy: tuple[float, float, float],
    tcp_xyz: tuple[float, float, float],
) -> None:
    g1_tree = load_xml(g1_23_path)
    g1_root = g1_tree.getroot()
    g1_root.set("name", "g1_23dof_revo2_right")
    rewrite_urdf_meshes(g1_root, "g1_23dof/meshes")

    wrist_joint = require(
        g1_root, ".//joint[@name='right_wrist_roll_joint']", "23DOF wrist joint"
    )
    require(wrist_joint, "child", "23DOF wrist child").set("link", "right_wrist_roll_link")

    old_wrist = require(
        g1_root, "./link[@name='right_wrist_roll_rubber_hand']", "rubber-hand wrist link"
    )
    g1_root.remove(old_wrist)

    g1_29_root = load_xml(g1_29_path).getroot()
    wrist_link = copy.deepcopy(
        require(g1_29_root, "./link[@name='right_wrist_roll_link']", "standalone wrist link")
    )
    rewrite_urdf_meshes(wrist_link, "g1_23dof/meshes")
    g1_root.append(wrist_link)

    adapter_length, adapter_midpoint, adapter_rpy, adapter_inertia = adapter_spec(
        mount_xyz)
    adapter_link = ET.Element("link", {"name": ADAPTER_LINK})
    inertial = ET.SubElement(adapter_link, "inertial")
    ET.SubElement(inertial, "origin", {
        "xyz": vector_text(adapter_midpoint), "rpy": vector_text(adapter_rpy)})
    ET.SubElement(inertial, "mass", {"value": f"{ADAPTER_MASS:.8g}"})
    ET.SubElement(inertial, "inertia", {
        "ixx": f"{adapter_inertia[0]:.12g}", "ixy": "0", "ixz": "0",
        "iyy": f"{adapter_inertia[1]:.12g}", "iyz": "0",
        "izz": f"{adapter_inertia[2]:.12g}",
    })
    for tag, radius in (("visual", ADAPTER_RADIUS),
                        ("collision", ADAPTER_RADIUS * 0.92)):
        item = ET.SubElement(adapter_link, tag)
        ET.SubElement(item, "origin", {
            "xyz": vector_text(adapter_midpoint), "rpy": vector_text(adapter_rpy)})
        geometry = ET.SubElement(item, "geometry")
        ET.SubElement(geometry, "cylinder", {
            "radius": f"{radius:.8g}", "length": f"{adapter_length:.8g}"})
        if tag == "visual":
            material = ET.SubElement(item, "material", {"name": "revo2_adapter_dark"})
            ET.SubElement(material, "color", {"rgba": "0.16 0.18 0.20 1"})
    g1_root.append(adapter_link)

    adapter_joint = ET.Element(
        "joint", {"name": "right_revo2_adapter_joint", "type": "fixed"})
    ET.SubElement(adapter_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(adapter_joint, "parent", {"link": "right_wrist_roll_link"})
    ET.SubElement(adapter_joint, "child", {"link": ADAPTER_LINK})
    g1_root.append(adapter_joint)

    revo_root = load_xml(revo_path).getroot()
    rewrite_urdf_meshes(revo_root, "revo2_right/meshes")
    for element in list(revo_root):
        if element.tag in {"link", "joint"}:
            g1_root.append(copy.deepcopy(element))

    mount_joint = ET.Element("joint", {"name": "right_revo2_mount_joint", "type": "fixed"})
    ET.SubElement(mount_joint, "origin", {
        "xyz": vector_text(mount_xyz),
        "rpy": vector_text(mount_rpy),
    })
    ET.SubElement(mount_joint, "parent", {"link": ADAPTER_LINK})
    ET.SubElement(mount_joint, "child", {"link": "right_base_link"})
    g1_root.append(mount_joint)

    g1_root.append(ET.Element("link", {"name": "right_revo2_tcp"}))
    tcp_joint = ET.Element("joint", {"name": "right_revo2_tcp_joint", "type": "fixed"})
    ET.SubElement(tcp_joint, "origin", {"xyz": vector_text(tcp_xyz), "rpy": "0 0 0"})
    ET.SubElement(tcp_joint, "parent", {"link": "right_base_link"})
    ET.SubElement(tcp_joint, "child", {"link": "right_revo2_tcp"})
    g1_root.append(tcp_joint)

    joint_names = [joint.attrib["name"] for joint in g1_root.findall("joint")]
    if len(joint_names) != len(set(joint_names)):
        raise RuntimeError("combined URDF contains duplicate joint names")
    if not RIGHT_ARM_JOINTS.issubset(joint_names):
        raise RuntimeError("combined URDF is missing one or more G1 right-arm joints")
    if not set(REVO_ACTIVE_JOINTS).issubset(joint_names):
        raise RuntimeError("combined URDF is missing one or more Revo2 active joints")

    write_xml(g1_tree, output_path)


def rewrite_mjcf_meshes(root: ET.Element, prefix: str) -> None:
    for mesh in root.findall(".//mesh[@file]"):
        filename = Path(mesh.attrib["file"].replace("\\", "/")).name
        mesh.set("file", f"{prefix}/{filename}")


def find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.findall(".//body"):
        if body.get("name") == name:
            return body
    raise RuntimeError(f"missing MJCF body '{name}'")


def merge_named_children(destination: ET.Element, source: ET.Element) -> None:
    existing = {(child.tag, child.get("name")) for child in destination}
    for child in source:
        key = (child.tag, child.get("name"))
        if key not in existing:
            destination.append(copy.deepcopy(child))
            existing.add(key)


def add_hold_actuators(root: ET.Element) -> None:
    actuator = require(root, "./actuator", "G1 actuator section")
    existing_names = {child.get("name") for child in actuator}
    # Only the original G1 motor joints receive whole-body hold actuators.
    # Revo2's mechanically coupled distal joints also have ranges, but adding
    # position actuators to them would fight the equality/mimic constraints.
    g1_motor_joints = {
        child.get("joint", "") for child in actuator if child.tag == "motor"
    }
    for joint in root.findall(".//joint[@name]"):
        name = joint.get("name", "")
        if name not in g1_motor_joints or name in RIGHT_ARM_JOINTS:
            continue
        if joint.get("type") == "free" or not joint.get("range"):
            continue
        hold_name = f"hold_{name}"
        if hold_name in existing_names:
            continue
        actuator.append(ET.Element("position", {
            "name": hold_name,
            "joint": name,
            "kp": "200",
            "kv": "20",
            "ctrlrange": joint.get("range", "-3.14159 3.14159"),
        }))
        existing_names.add(hold_name)


def configure_dynamics(root: ET.Element) -> None:
    """Use a stable fixed-base integration setup and modest joint damping."""
    option = root.find("./option")
    if option is None:
        option = ET.Element("option")
        compiler = require(root, "./compiler", "MJCF compiler")
        root.insert(list(root).index(compiler) + 1, option)
    option.set("timestep", "0.002")
    option.set("integrator", "implicitfast")

    hand_joint = require(root, "./default/default[@class='righthand']/joint",
                         "Revo2 joint defaults")
    hand_joint.set("damping", f"{REVO_JOINT_DAMPING:g}")
    hand_joint.set("armature", f"{REVO_JOINT_ARMATURE:g}")
    hand_joint.set("frictionloss", f"{REVO_JOINT_FRICTION:g}")

    for joint in root.findall(".//joint[@name]"):
        if joint.get("name") in RIGHT_ARM_JOINTS:
            joint.set("damping", "0.5")
            joint.set("armature", "0.01")
            joint.set("frictionloss", "0.05")


def make_arm_only(root: ET.Element) -> ET.ElementTree:
    clone = copy.deepcopy(root)
    clone.set("model", "g1_23dof_revo2_arm_only")

    for body in clone.findall(".//body"):
        for joint in list(body.findall("joint")):
            name = joint.get("name", "")
            if name not in RIGHT_ARM_JOINTS and name not in REVO_LIMITS:
                body.remove(joint)

    actuator = require(clone, "./actuator", "arm-only actuator section")
    for item in list(actuator):
        joint = item.get("joint", "")
        if joint not in RIGHT_ARM_JOINTS and joint not in REVO_ACTIVE_JOINTS:
            actuator.remove(item)

    return ET.ElementTree(clone)


def build_mjcf(
    g1_23_path: Path,
    g1_29_path: Path,
    revo_path: Path,
    full_output: Path,
    arm_only_output: Path,
    mount_xyz: tuple[float, float, float],
    mount_rpy: tuple[float, float, float],
    tcp_xyz: tuple[float, float, float],
) -> None:
    g1_tree = load_xml(g1_23_path)
    g1_root = g1_tree.getroot()
    g1_root.set("model", "g1_23dof_revo2_right")
    compiler = require(g1_root, "./compiler", "G1 MJCF compiler")
    compiler.attrib.pop("meshdir", None)
    rewrite_mjcf_meshes(g1_root, "g1_23dof/meshes")

    # This project does not control locomotion. Fix the simulation base instead
    # of overwriting qpos every step (which injects impulses into the arm). Real
    # hardware uses Unitree's standing service to own the remaining joints.
    pelvis_body = find_body(g1_root, "pelvis")
    for joint in list(pelvis_body.findall("joint")):
        if joint.get("type") == "free" or joint.get("name") == "floating_base_joint":
            pelvis_body.remove(joint)

    g1_asset = require(g1_root, "./asset", "G1 asset section")
    for mesh in list(g1_asset.findall("mesh")):
        if mesh.get("name") == "right_wrist_roll_rubber_hand":
            g1_asset.remove(mesh)

    g1_29_root = load_xml(g1_29_path).getroot()
    wrist_mesh = copy.deepcopy(
        require(g1_29_root, "./asset/mesh[@name='right_wrist_roll_link']", "wrist mesh")
    )
    wrist_mesh.set("file", "g1_23dof/meshes/right_wrist_roll_link.STL")
    g1_asset.append(wrist_mesh)

    wrist_body = find_body(g1_root, "right_wrist_roll_rubber_hand")
    wrist_body.set("name", "right_wrist_roll_link")
    for child in list(wrist_body):
        if child.tag in {"inertial", "geom"}:
            wrist_body.remove(child)

    source_wrist_body = find_body(g1_29_root, "right_wrist_roll_link")
    insert_at = 0
    for child in source_wrist_body:
        if child.tag in {"inertial", "geom"}:
            wrist_body.insert(insert_at, copy.deepcopy(child))
            insert_at += 1

    revo_root = load_xml(revo_path).getroot()
    rewrite_mjcf_meshes(revo_root, "revo2_right/meshes")

    revo_default = revo_root.find("./default")
    if revo_default is not None:
        g1_default = g1_root.find("./default")
        if g1_default is None:
            g1_default = ET.Element("default")
            g1_root.insert(list(g1_root).index(g1_asset), g1_default)
        merge_named_children(g1_default, revo_default)

    merge_named_children(g1_asset, require(revo_root, "./asset", "Revo asset section"))

    revo_contact = revo_root.find("./contact")
    if revo_contact is not None:
        g1_contact = g1_root.find("./contact")
        if g1_contact is None:
            g1_contact = ET.Element("contact")
            worldbody = require(g1_root, "./worldbody", "G1 worldbody")
            g1_root.insert(list(g1_root).index(worldbody), g1_contact)
        for item in revo_contact:
            g1_contact.append(copy.deepcopy(item))

    adapter_length, adapter_midpoint, adapter_rpy, adapter_inertia = adapter_spec(
        mount_xyz)
    revo_body = copy.deepcopy(find_body(revo_root, "right_base_link"))
    revo_body.set("pos", vector_text(mount_xyz))
    # URDF uses Rz(yaw)*Ry(pitch)*Rx(roll). Writing the same three numbers as
    # an MJCF euler attribute changes their composition order. A quaternion is
    # unambiguous and keeps the URDF and MuJoCo hand mount identical.
    revo_body.attrib.pop("euler", None)
    revo_body.set("quat", vector_text(urdf_rpy_to_mjcf_quat(mount_rpy)))
    revo_body.set("childclass", "righthand")
    revo_body.append(ET.Element("site", {
        "name": "right_revo2_tcp",
        "pos": vector_text(tcp_xyz),
        "group": "4",
        "size": "0.006",
        "rgba": "1 0.2 0.2 1",
    }))
    adapter_body = ET.Element("body", {"name": ADAPTER_LINK})
    adapter_body.append(ET.Element("inertial", {
        "pos": vector_text(adapter_midpoint),
        "quat": vector_text(urdf_rpy_to_mjcf_quat(adapter_rpy)),
        "mass": f"{ADAPTER_MASS:.8g}",
        "diaginertia": vector_text(adapter_inertia),
    }))
    adapter_body.append(ET.Element("geom", {
        "name": "right_revo2_adapter",
        "type": "cylinder",
        "fromto": f"0 0 0 {vector_text(mount_xyz)}",
        "size": f"{ADAPTER_RADIUS:.8g}",
        "rgba": "0.16 0.18 0.20 1",
        "density": "0",
    }))
    adapter_body.append(revo_body)
    wrist_body.append(adapter_body)

    g1_actuator = require(g1_root, "./actuator", "G1 actuator section")
    for item in require(revo_root, "./actuator", "Revo actuator section"):
        copy_item = copy.deepcopy(item)
        name = copy_item.get("joint", "")
        if name in REVO_LIMITS:
            lo, hi = REVO_LIMITS[name]
            copy_item.set("ctrlrange", f"{lo:.8g} {hi:.8g}")
        if name in REVO_ACTIVE_JOINTS and copy_item.tag == "position":
            copy_item.set("kp", f"{REVO_SERVO_KP:g}")
            copy_item.set("kv", f"{REVO_SERVO_KV:g}")
        g1_actuator.append(copy_item)

    equality = g1_root.find("./equality")
    if equality is None:
        equality = ET.Element("equality")
        g1_root.append(equality)
    for slave, (master, multiplier) in REVO_MIMIC.items():
        equality.append(ET.Element("joint", {
            "name": f"mimic_{slave}",
            "joint1": master,
            "joint2": slave,
            "polycoef": f"0 {multiplier:.8g} 0 0 0",
        }))

    for joint in g1_root.findall(".//joint[@name]"):
        name = joint.get("name", "")
        if name in REVO_LIMITS:
            lo, hi = REVO_LIMITS[name]
            joint.set("range", f"{lo:.8g} {hi:.8g}")

    configure_dynamics(g1_root)
    add_hold_actuators(g1_root)
    write_xml(g1_tree, full_output)
    write_xml(make_arm_only(g1_root), arm_only_output)


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    project_dir = script_dir.parent
    models_dir = project_dir / "config" / "models"
    source_dir = models_dir / "sources"

    parser = argparse.ArgumentParser()
    parser.add_argument("--mount-xyz", type=parse_triplet, default=parse_triplet("0.0591 0 0"))
    parser.add_argument(
        "--mount-rpy",
        type=parse_triplet,
        default=parse_triplet(f"{math.pi / 2} 0 {math.pi / 2}"),
    )
    parser.add_argument("--tcp-xyz", type=parse_triplet, default=parse_triplet("0.03 0 0.11"))
    args = parser.parse_args()

    build_urdf(
        source_dir / "g1" / "g1_23dof_rev_1_0.urdf",
        source_dir / "g1" / "g1_29dof_rev_1_0.urdf",
        source_dir / "revo2" / "revo2_right_hand.urdf",
        models_dir / "g1_23dof_revo2.urdf",
        args.mount_xyz,
        args.mount_rpy,
        args.tcp_xyz,
    )
    build_mjcf(
        source_dir / "g1" / "g1_23dof_rev_1_0.xml",
        source_dir / "g1" / "g1_29dof_rev_1_0.xml",
        source_dir / "revo2" / "brainco-righthand-v2.xml",
        models_dir / "g1_23dof_revo2.xml",
        models_dir / "g1_23dof_revo2_arm_only.xml",
        args.mount_xyz,
        args.mount_rpy,
        args.tcp_xyz,
    )
    print(f"Generated {models_dir / 'g1_23dof_revo2.urdf'}")
    print(f"Generated {models_dir / 'g1_23dof_revo2.xml'}")
    print(f"Generated {models_dir / 'g1_23dof_revo2_arm_only.xml'}")
    print("WARNING: wrist-to-Revo2 transform is provisional; calibrate before real TCP tests.")


if __name__ == "__main__":
    main()
