#!/usr/bin/env python3
"""
inject_inspire_hand.py — Inject Inspire RH56 hand + 5010 wrist patch into G1 MJCF.

Reads the G1 29-DOF MJCF and the Inspire hand body file (inspire_hand_right_body.xml),
then:
  1. Fixes relative meshdir / <include> paths to absolute.
  2. Hides native RIGHT rubber_hand geoms (visual + collision).
  3. Patches right wrist to 5010 specs (torque limit ±13.4 N·m, link masses 0.684 / 0.0846).
  4. Injects the Inspire hand (R_hand_base_link, mount baked in) as a child of right_wrist_yaw_link.
  5. Adds <equality> joint constraints for the 6 underactuated slave joints
     (slave = ratio x master — MuJoCo polycoef: joint1 = polycoef[1]·joint2).

Usage: python3 inject_inspire_hand.py <g1_29dof.xml> <inspire_body.xml> <output.xml>
"""

import sys
import re
import os

# Slave joint coupling: (slave, master, ratio). equality: joint1=slave, joint2=master.
COUPLING = [
    ("R_thumb_intermediate_joint", "R_thumb_proximal_pitch_joint", 1.6),
    ("R_thumb_distal_joint",       "R_thumb_proximal_pitch_joint", 2.4),
    ("R_index_intermediate_joint", "R_index_proximal_joint",       1.0),
    ("R_middle_intermediate_joint","R_middle_proximal_joint",      1.0),
    ("R_ring_intermediate_joint",  "R_ring_proximal_joint",        1.0),
    ("R_pinky_intermediate_joint", "R_pinky_proximal_joint",       1.0),
]

# 5010 wrist: torque limit + link masses (mode_15 g1_29dof_mode_15.urdf)
WRIST_FRCRANGE = "13.4"
WRIST_MASS = {
    "right_wrist_pitch_link": 0.684,
    "right_wrist_yaw_link":   0.08457647,
}


def find_matching_close(text, tag_name, open_pattern, start_pos=0):
    """Find the matching closing tag for a nestable XML element."""
    m = re.search(open_pattern, text[start_pos:])
    if not m:
        return None, None, None
    open_start = start_pos + m.start()
    open_end = start_pos + m.end()
    depth = 1
    pos = open_end
    while pos < len(text):
        next_open = text.find(f'<{tag_name}', pos)
        next_close = text.find(f'</{tag_name}>', pos)
        if next_open == -1:
            next_open = len(text) + 1
        if next_close == -1:
            next_close = len(text) + 1
        if next_open < next_close:
            depth += 1
            pos = next_open + len(tag_name) + 1
        else:
            depth -= 1
            if depth == 0:
                return open_start, open_end, next_close
            pos = next_close + len(tag_name) + 3
    return open_start, open_end, None


def hide_rubber_geoms(text):
    """Disable RIGHT rubber_hand geoms (visual + collision). Returns (text, count)."""
    count = 0

    def disable(m):
        nonlocal count
        geom = m.group(0)
        for attr in ['rgba', 'contype', 'conaffinity']:
            geom = re.sub(rf'\b{attr}="[^"]*"', '', geom)
        disable_attrs = 'rgba="0 0 0 0" contype="0" conaffinity="0"'
        if geom.rstrip().endswith('/>'):
            geom = geom.rstrip()[:-2].rstrip() + f' {disable_attrs}/>'
        else:
            geom = geom.rstrip()[:-1].rstrip() + f' {disable_attrs}>'
        count += 1
        return geom

    text = re.sub(r'<geom\b[^>]*\bmesh="[^"]*right[^"]*rubber_hand[^"]*"[^>]*/?>', disable, text)
    text = re.sub(r'<geom\b[^>]*\bname="[^"]*right[^"]*rubber_hand[^"]*"[^>]*/?>', disable, text)
    return text, count


def patch_wrist_5010(text):
    """Patch right wrist pitch/yaw torque limit to ±13.4 and link masses to 5010 values."""
    # Torque limit on the two 5010 joints (right only)
    for joint in ["right_wrist_pitch_joint", "right_wrist_yaw_joint"]:
        pattern = (rf'(<joint name="{joint}"[^>]*actuatorfrcrange=")-?[0-9.]+ -?[0-9.]+(")')
        text, n = re.subn(pattern, rf'\g<1>-{WRIST_FRCRANGE} {WRIST_FRCRANGE}\g<2>', text)
        print(f"  patched {joint} frcrange -> ±{WRIST_FRCRANGE} ({n} match)")

    # Link masses within the named body's <inertial>
    for link, mass in WRIST_MASS.items():
        pat = re.compile(rf'(<body\b[^>]*\bname="{link}"[^>]*>.*?<inertial\b[^>]*\bmass=")[^"]*(")', re.DOTALL)
        text, n = pat.subn(rf'\g<1>{mass:g}\g<2>', text, count=1)
        if n == 0:
            print(f"  [!] WARNING: could not patch {link} mass -> {mass} (regex no match)")
        else:
            print(f"  patched {link} mass -> {mass}")
    return text


def add_equality_solref(text):
    """Insert <equality> joint constraints for the 6 slave joints.
    REQUIRES solref="0.001 1" (stiff) — the default solref is too soft and the
    joint equality never converges on these tiny-inertia joints (MuJoCo 3.11)."""
    eq = ['  <equality>']
    for slave, master, ratio in COUPLING:
        eq.append(f'    <joint joint1="{slave}" joint2="{master}" polycoef="0 {ratio} 0 0" '
                  f'solref="0.001 1"/>')
    eq.append('  </equality>')
    eq_block = "\n".join(eq)

    m = re.search(r'<equality>', text)
    if m:
        _, _, close = find_matching_close(text, 'equality', r'<equality>')
        text = text[:close] + "\n" + "\n".join(eq[1:-1]) + "\n" + text[close:]
        print("  appended to existing <equality> (solref=0.001 1)")
    else:
        text = text.rstrip()[:-len("</mujoco>")].rstrip() + "\n" + eq_block + "\n</mujoco>\n"
        print("  inserted new <equality> (solref=0.001 1)")
    return text


def fix_base_to_world(text):
    """Remove the floating base free joint → pelvis becomes FIXED to the world.

    The old bridge approach hard-clamped the free-joint qpos each step, which
    FREEZES the right arm in the implicitfast solver (the arm cannot move under
    its own dynamics) — the torque controller fights a frozen arm → oscillation.
    Removing the free joint entirely makes the base truly fixed: the arm is a
    fixed-base 7-DOF chain and the controller behaves like a normal fixed-base arm.
    (MuJoCo has no 'fixed' joint type — a body with no joint is fixed to parent.)"""
    import re
    text, n = re.subn(r'<joint name="floating_base_joint"[^>]*/>\s*', '', text)
    if n != 1:
        print(f"  [!] WARNING: expected exactly 1 'floating_base_joint' to remove, "
              f"found {n}. The base may NOT be fixed — check the base MJCF's free-joint "
              f"name (must be 'floating_base_joint', self-closing).")
    return text


# Right arm motor reflected inertia (real G1/5010 motors). The scene default
# armature=0.001 is far too light: the arm inertia is ~0.1 kg·m², so a tiny
# gravity-FF residual (a couple N·m) becomes ~16 rad/s² of acceleration, the
# joint damping saturates (±25 N·m), and the arm spins away. Real joints carry
# the motor+gearbox reflected inertia, which both slows the arm to a controllable
# bandwidth and gives the damping room to act.
ARM_ARMATURE = {
    "right_shoulder_pitch_joint": 0.30,
    "right_shoulder_roll_joint": 0.30,
    "right_shoulder_yaw_joint": 0.25,
    "right_elbow_joint": 0.20,
    "right_wrist_roll_joint": 0.10,
    "right_wrist_pitch_joint": 0.10,
    "right_wrist_yaw_joint": 0.05,
}


def add_arm_armature(text):
    """Add realistic motor reflected inertia (armature) to the right-arm joints."""
    import re
    for joint, arm in ARM_ARMATURE.items():
        pat = re.compile(r'(<joint name="%s"[^>]*?)(\s*/>)' % joint)
        def repl(m):
            tag = m.group(1)
            # strip any existing armature= to avoid duplicates
            tag = re.sub(r'\barmature="[^"]*"', '', tag)
            return f'{tag} armature="{arm}"{m.group(2)}'
        text, n = pat.subn(repl, text, count=1)
        if n == 0:
            print(f"  [!] WARNING: could not add armature to {joint}")
        else:
            print(f"  armature {joint} -> {arm}")
    return text


def strip_base_actuators(text):
    """Remove the base G1 <actuator> block (native <motor>s). The scene defines all
    actuators (torque motors for right arm + hand, position servos for locked joints);
    keeping the base motors would double-drive the joints."""
    m = re.search(r'<actuator>', text)
    if not m:
        return text, 0
    _, _, close = find_matching_close(text, 'actuator', r'<actuator>')
    block = text[m.start():close + len('</actuator>')]
    n = block.count('<motor')
    text = text[:m.start()] + text[close + len('</actuator>'):]
    return text, n


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <g1_29dof.xml> <inspire_body.xml> <output.xml>")
        sys.exit(1)
    g1_path, inspire_body_path, output_path = sys.argv[1], sys.argv[2], sys.argv[3]

    with open(g1_path, encoding='utf-8') as f:
        text = f.read()
    with open(inspire_body_path, encoding='utf-8') as f:
        inspire_text = f.read()

    # --- Remove base native actuators (scene owns all actuators) ---
    text, n_motors = strip_base_actuators(text)
    print(f"Removed {n_motors} base native <motor> actuator(s)")

    # --- Fix base to world (free joint → fixed): no hard-clamp freezing, no
    #     weld base-coupling → clean fixed-base arm dynamics ---
    text = fix_base_to_world(text)
    print("Fixed floating base to world (free joint → fixed)")

    # --- Realistic motor reflected inertia for the right arm (stability) ---
    text = add_arm_armature(text)

    # --- Hand joints: bump armature 0.0005 → 0.01 (motor+gearbox reflected
    #     inertia; 0.0005 makes the 50 Hz impedance loop a bang-bang limit cycle:
    #     ω_n = sqrt(kp/0.0005) ≈ 100 rad/s >> 50 Hz control Nyquist → fingers
    #     tremble and the rectified limit cycle drifts toward the joint limits).
    #     MUST operate on inspire_text (the hand body fragment) — `text` is the
    #     base G1 MJCF which has no R_* joints yet (hand is injected later), so
    #     targeting text silently matches 0 joints. ---
    import re as _re
    inspire_text, _n = _re.subn(r'(<joint name="R_[^"]*"[^>]*?)armature="[^"]*"',
                                r'\1armature="0.01"', inspire_text)
    if _n != 12:
        print(f"  [!] WARNING: expected 12 hand joints, bumped only {_n}")
    print(f"Bumped hand joint armature -> 0.01 ({_n} joints)")

    # --- Fix relative paths in the base to absolute ---
    g1_dir = os.path.dirname(os.path.abspath(g1_path))
    text = re.sub(r'meshdir="([^"]*)"',
                  lambda m: f'meshdir="{os.path.normpath(os.path.join(g1_dir, m.group(1)))}"' if not os.path.isabs(m.group(1)) else m.group(0),
                  text)
    text = re.sub(r'<include\s+file="([^"]*)"',
                  lambda m: f'<include file="{os.path.normpath(os.path.join(g1_dir, m.group(1)))}"' if not os.path.isabs(m.group(1)) else m.group(0),
                  text)

    # --- Hide right rubber_hand ---
    text, n_hidden = hide_rubber_geoms(text)
    print(f"Hidden {n_hidden} right rubber_hand geom(s)")

    # --- Patch wrist to 5010 ---
    text = patch_wrist_5010(text)

    # --- Extract Inspire body tree (R_hand_base_link) ---
    m = re.search(r'<body\b[^>]*\bname="R_hand_base_link"[^>]*>', inspire_text)
    if not m:
        print("ERROR: R_hand_base_link body not found in inspire body file")
        sys.exit(1)
    open_start, open_end, close_pos = find_matching_close(inspire_text, 'body', r'<body\b[^>]*\bname="R_hand_base_link"[^>]*>')
    inspire_body = inspire_text[open_start:close_pos + len('</body>')]
    print(f"Extracted Inspire body tree ({len(inspire_body)} chars)")

    # --- Inject into right_wrist_yaw_link ---
    pat = r'<body\b[^>]*\bname="right_wrist_yaw_link"[^>]*>'
    w_open, w_end, w_close = find_matching_close(text, 'body', pat)
    if w_open is None:
        print("ERROR: right_wrist_yaw_link body not found in base MJCF")
        sys.exit(1)
    text = text[:w_close] + "\n" + inspire_body + "\n" + text[w_close:]
    print("Injected Inspire hand under right_wrist_yaw_link")

    # --- Underactuation coupling: 6 joint-equality constraints (stiff solref).
    #     Slave = ratio x master, enforced as a rigid kinematic constraint. ---
    text = add_equality_solref(text)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(text)
    print(f"[OK] Merged MJCF written to: {output_path}")


if __name__ == "__main__":
    main()
