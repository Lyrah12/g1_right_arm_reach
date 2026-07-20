#!/usr/bin/env python3
"""
inject_wuji_hand.py — Inject Wuji Hand body tree into G1 29-DOF MJCF.

Reads the G1 29-DOF MJCF and the Wuji hand body file, then inserts the Wuji
palm + fingers as children of right_wrist_yaw_link (kinematic attachment).
Also hides the native RIGHT rubber_hand visual + collision geoms.

Usage: python3 inject_wuji_hand.py <g1_29dof.xml> <wuji_body.xml> <output.xml>
"""

import sys
import re
import os


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


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <g1_29dof.xml> <wuji_body.xml> <output.xml>")
        sys.exit(1)

    g1_path = sys.argv[1]
    wuji_body_path = sys.argv[2]
    output_path = sys.argv[3]

    # Read files
    with open(g1_path, 'r') as f:
        g1_text = f.read()

    with open(wuji_body_path, 'r') as f:
        wuji_text = f.read()

    # Fix relative meshdir in compiler tag to absolute
    g1_dir = os.path.dirname(os.path.abspath(g1_path))
    def fix_meshdir(m):
        md = m.group(1)
        if not os.path.isabs(md):
            abs_md = os.path.normpath(os.path.join(g1_dir, md))
            return f'meshdir="{abs_md}"'
        return m.group(0)
    g1_text = re.sub(r'meshdir="([^"]*)"', fix_meshdir, g1_text)

    # Fix relative <include file="..."> paths to absolute
    def fix_include(m):
        inc = m.group(1)
        if not os.path.isabs(inc):
            abs_inc = os.path.normpath(os.path.join(g1_dir, inc))
            return f'<include file="{abs_inc}"'
        return m.group(0)
    g1_text = re.sub(r'<include\s+file="([^"]*)"', fix_include, g1_text)

    # =========================================================================
    # STEP 1: Hide RIGHT rubber_hand geoms ONLY (visual + collision)
    # =========================================================================
    # Matches <geom ... mesh="*right*rubber_hand*" ... />
    # Adds rgba="0 0 0 0" contype="0" conaffinity="0" to fully disable
    count_hidden = 0

    def hide_rubber_geom(m):
        nonlocal count_hidden
        geom = m.group(0)
        # Strip existing visual/collision attrs
        for attr in ['rgba', 'contype', 'conaffinity']:
            geom = re.sub(rf'\b{attr}="[^"]*"', '', geom)
        # Insert disable attrs before closing /> or >
        disable = 'rgba="0 0 0 0" contype="0" conaffinity="0"'
        if geom.rstrip().endswith('/>'):
            geom = geom.rstrip()[:-2].rstrip() + f' {disable}/>'
        else:
            geom = geom.rstrip()[:-1].rstrip() + f' {disable}>'
        count_hidden += 1
        return geom

    # Only match RIGHT rubber_hand geoms (mesh="*right*rubber_hand*")
    g1_text = re.sub(
        r'<geom\b[^>]*\bmesh="[^"]*right[^"]*rubber_hand[^"]*"[^>]*/?>',
        hide_rubber_geom,
        g1_text
    )
    # Also match geoms named *right*rubber_hand*
    g1_text = re.sub(
        r'<geom\b[^>]*\bname="[^"]*right[^"]*rubber_hand[^"]*"[^>]*/?>',
        hide_rubber_geom,
        g1_text
    )
    print(f"Hidden {count_hidden} right rubber_hand geom(s) (visual + collision → OFF)")

    # =========================================================================
    # STEP 2: Extract Wuji body tree + rotate palm so fingers point +X (forward)
    # =========================================================================
    # Wuji fingers extend along +Z in palm-local frame. G1 world +X = forward.
    # quat="0.7071 0 -0.7071 0" = -90° around Y → maps palm +Z → world +X.
    # This keeps wrist_pitch=0 (straight wrist, no ugly bend).

    m = re.search(r'<worldbody>\s*(.*?)\s*</worldbody>', wuji_text, re.DOTALL)
    if not m:
        print("ERROR: Could not find <worldbody> in Wuji body file")
        sys.exit(1)
    wuji_body_content = m.group(1).strip()

    # Add quat to right_palm_link to orient the hand correctly:
    #   Fingers (palm +Z) → world +X (forward)
    #   Palm normal        → world -Y (left)
    # Single 120° rotation with 180° twist about finger axis.
    PALM_QUAT = 'quat="0.5 0.5 0.5 0.5"'
    if '<body name="right_palm_link" quat=' in wuji_body_content:
        print("WARNING: right_palm_link already has quat, replacing")
        wuji_body_content = re.sub(
            r'<body\s+name="right_palm_link"\s+quat="[^"]*"',
            f'<body name="right_palm_link" {PALM_QUAT}',
            wuji_body_content, count=1
        )
    else:
        wuji_body_content = wuji_body_content.replace(
            '<body name="right_palm_link">',
            f'<body name="right_palm_link" {PALM_QUAT}>',
            1
        )
    print(f"Palm quat set: {PALM_QUAT} (fingers +Z → world +X)")

    # =========================================================================
    # STEP 3: Inject Wuji body tree into right_wrist_yaw_link
    # =========================================================================
    pattern = r'<body\b[^>]*\bname="right_wrist_yaw_link"[^>]*>'
    open_start, open_end, close_pos = find_matching_close(g1_text, 'body', pattern)

    if open_start is None:
        print(f"ERROR: body tag for right_wrist_yaw_link not found in G1 MJCF")
        sys.exit(1)

    print(f"Found right_wrist_yaw_link body: open at char {open_start}, close at char {close_pos}")
    print(f"Inserting Wuji body tree ({len(wuji_body_content)} chars)")

    # Insert Wuji body content before the closing </body>
    g1_text = g1_text[:close_pos] + "\n" + wuji_body_content + "\n" + g1_text[close_pos:]

    # =========================================================================
    # STEP 4: Disable RIGHT rubber_hand body physics (if it exists as a body)
    # =========================================================================
    for hand_name in ['right_rubber_hand']:  # RIGHT only, keep left
        pattern = rf'<body\b[^>]*\bname="{hand_name}"[^>]*>'
        body_open, body_end, body_close = find_matching_close(g1_text, 'body', pattern)
        if body_open is not None:
            old_open = g1_text[body_open:body_end]
            if 'mocap=' not in old_open:
                new_open = old_open.rstrip()[:-1] + ' mocap="true">'
                g1_text = g1_text[:body_open] + new_open + g1_text[body_end:]
            print(f"Set {hand_name} body to mocap (disabled physics)")

    # =========================================================================
    # Write output
    # =========================================================================
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(g1_text)

    print(f"✅ Patched G1 MJCF written to: {output_path}")
    print(f"   - Wuji body tree injected as kinematic child of right_wrist_yaw_link")
    print(f"   - {count_hidden} right rubber_hand geom(s) hidden (visual + collision off)")
    print(f"   - All relative paths fixed to absolute")


if __name__ == "__main__":
    main()
