#!/bin/bash
# =============================================================================
# setup_wuji_hand.sh — Clone wuji-hand-description + generate patched MJCF
# =============================================================================
# Usage: bash scripts/setup_wuji_hand.sh
#
# Generates: config/wuji_hand_right_patched.xml  (assets + finger actuators, kp/kv boosted)
#            config/wuji_hand_right_body.xml      (body tree fragment for injection)
#            <g1_dir>/g1_29dof_wuji.xml           (G1 + Wuji body injected)
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_DIR="${PKG_DIR}/config"
WUJI_DIR="${HOME}/deps/wuji-hand-description"

echo "============================================"
echo " Wuji Hand — MJCF Setup"
echo "============================================"

# --- Step 1: Clone repo ---
if [ ! -d "${WUJI_DIR}" ]; then
    echo "[1/4] Cloning wuji-hand-description..."
    git clone https://github.com/wuji-technology/wuji-hand-description.git "${WUJI_DIR}"
else
    echo "[1/4] Already cloned at ${WUJI_DIR}"
fi

# --- Step 2: Verify MJCF exists ---
WUJI_MJCF="${WUJI_DIR}/mjcf/right.xml"
if [ ! -f "${WUJI_MJCF}" ]; then
    echo "ERROR: MJCF not found at ${WUJI_MJCF}"
    exit 1
fi

# --- Step 3: Generate patched versions ---
WUJI_MESHDIR="${WUJI_DIR}/meshes/right"
TEMP="${CONFIG_DIR}/wuji_temp.xml"

echo "[2/4] Applying patches (meshdir + kp/kv + strip conflicting tags)..."
# Boost kp/kv, fix meshdir, strip <option> (RK4/Newton) and <default> (damping=0)
# Our scene uses Euler integrator + damping=2 — these must not conflict
sed -e "s|meshdir=\"../meshes/right/\"|meshdir=\"${WUJI_MESHDIR}/\"|" \
    -e 's|kp="[0-9.]*"|kp="5"|g' \
    -e 's|kv="[0-9.]*"|kv="0.5"|g' \
    -e 's|<option[^>]*>||g' \
    -e 's|<default><joint[^>]*></default>||g' \
    "${WUJI_MJCF}" > "${TEMP}"

echo "[3/4] Splitting into body + assets..."

# --- Generate assets+actuators file (included at <mujoco> level) ---
# Contains: compiler (meshdir), asset (mesh defs), actuator (finger servos)
OUTPUT_ASSETS="${CONFIG_DIR}/wuji_hand_right_patched.xml"
{
    echo '<!-- Wuji Hand Right — assets + actuators (auto-generated) -->'
    echo '<mujoco>'
    # compiler with absolute meshdir
    grep '<compiler' "${TEMP}" | head -1
    # asset block (mesh definitions)
    sed -n '/<asset>/,/<\/asset>/p' "${TEMP}"
    # actuator block (finger position servos with boosted kp/kv)
    sed -n '/<actuator>/,/<\/actuator>/p' "${TEMP}"
} > "${OUTPUT_ASSETS}"

# --- Generate body-only file (for injection into G1 MJCF) ---
OUTPUT_BODY="${CONFIG_DIR}/wuji_hand_right_body.xml"
{
    echo '<!-- Wuji Hand Right — body tree (auto-generated) -->'
    echo '<mujoco>'
    echo '  <worldbody>'
    # Extract body tree from <worldbody>...</worldbody>
    sed -n '/<worldbody>/,/<\/worldbody>/p' "${TEMP}" \
      | sed '1d' \
      | sed 's|</worldbody>||'
    echo '  </worldbody>'
} > "${OUTPUT_BODY}"

rm -f "${TEMP}"

# --- Step 4: Inject Wuji hand body into G1 29-DOF MJCF ---
G1_MJCF="${HOME}/deps/unitree_mujoco/unitree_robots/g1/g1_29dof.xml"
# Output to SAME directory as G1 MJCF to preserve relative <include> paths
G1_DIR="$(dirname "${G1_MJCF}")"
G1_PATCHED="${G1_DIR}/g1_29dof_wuji.xml"

if [ -f "${G1_MJCF}" ]; then
    echo "[4/4] Injecting Wuji hand into G1 29-DOF MJCF..."
    python3 "${PKG_DIR}/scripts/inject_wuji_hand.py" \
        "${G1_MJCF}" \
        "${OUTPUT_BODY}" \
        "${G1_PATCHED}"
else
    echo "[4/4] ⚠ G1 29-DOF MJCF not found at ${G1_MJCF}"
    echo "    Skipping injection. Make sure unitree_mujoco is installed."
    echo "    The scene will NOT work without the injected file!"
    exit 1
fi

echo ""
echo "✅ Done."
echo "   Assets + actuators: ${OUTPUT_ASSETS}"
echo "   Body tree:          ${OUTPUT_BODY}"
echo "   Injected MJCF:      ${G1_PATCHED}"
echo "   Joints: right_finger{1..5}_joint{1..4} (20 DOF, kp=5 kv=0.5)"
echo ""
echo "Launch:"
echo "  ros2 launch g1_right_arm_reach mujoco_demo.launch.py \\"
echo "    urdf_path:=\${HOME}/deps/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf \\"
echo "    scene_path:=\${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_wuji_scene.xml \\"
echo "    viewer:=true"
