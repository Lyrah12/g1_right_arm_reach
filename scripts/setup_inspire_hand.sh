#!/bin/bash
# =============================================================================
# setup_inspire_hand.sh — Generate Inspire RH56 merged MJCF for the G1
# =============================================================================
# Usage: bash scripts/setup_inspire_hand.sh
#
# Generates:
#   config/inspire_hand_right_body.xml     body tree fragment (mount baked in)
#   config/inspire_hand_right_assets.xml   mesh asset definitions
#   <g1_dir>/g1_29dof_inspire.xml          G1 (5010 wrist) + Inspire hand injected
#
# Assumes config/inspire_hand/ already contains DFQ_right_hand.urdf + meshes/
# (committed to the repo). Clones unitree_mujoco into ~/deps if missing.
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_DIR="${PKG_DIR}/config"
DEPS_DIR="${HOME}/deps"
INSPIRE_URDF="${CONFIG_DIR}/inspire_hand/DFQ_right_hand.urdf"

echo "============================================"
echo " Inspire RH56 Hand — MJCF Setup"
echo "============================================"

# --- Step 1: locate the base G1 MJCF (deprecated 29-DOF, 7-DOF arms + 3-DOF waist) ---
# Try unitree_mujoco (original) then unitree_ros_models (local copy).
for cand in \
    "${DEPS_DIR}/unitree_mujoco/unitree_robots/g1/g1_29dof.xml" \
    "${DEPS_DIR}/unitree_ros_models/g1_description/g1_29dof.xml"
do
    if [ -f "${cand}" ]; then G1_MJCF="${cand}"; break; fi
done
if [ -z "${G1_MJCF:-}" ]; then
    echo "[1/4] ERROR: base G1 MJCF not found."
    echo "    Put g1_29dof.xml at one of:"
    echo "      ${DEPS_DIR}/unitree_mujoco/unitree_robots/g1/g1_29dof.xml"
    echo "      ${DEPS_DIR}/unitree_ros_models/g1_description/g1_29dof.xml"
    exit 1
fi
echo "[1/4] Base G1 MJCF: ${G1_MJCF}"

# --- Step 2: regenerate MJCF fragments from the DFQ URDF ---
echo "[2/4] Converting DFQ URDF -> MJCF fragments..."
python3 "${PKG_DIR}/scripts/inspire_urdf_to_mjcf.py" "${INSPIRE_URDF}" "${CONFIG_DIR}"

# --- Step 3: inject into the G1 base (5010 wrist patch + equality + hand) ---
G1_DIR="$(dirname "${G1_MJCF}")"
G1_PATCHED="${G1_DIR}/g1_29dof_inspire.xml"
echo "[3/4] Injecting Inspire hand + 5010 wrist into G1 MJCF..."
python3 "${PKG_DIR}/scripts/inject_inspire_hand.py" \
    "${G1_MJCF}" \
    "${CONFIG_DIR}/inspire_hand_right_body.xml" \
    "${G1_PATCHED}"

# --- Step 4: patch the G1 URDF so Pinocchio gravity == MuJoCo (5010 + Inspire hand) ---
URDF_CAND=""
for cand in \
    "${DEPS_DIR}/unitree_ros_models/g1_description/g1_29dof_rev_1_0.urdf" \
    "${DEPS_DIR}/unitree_mujoco/unitree_robots/g1/g1_29dof_rev_1_0.urdf"
do
    if [ -f "${cand}" ]; then URDF_CAND="${cand}"; break; fi
done
if [ -n "${URDF_CAND}" ]; then
    echo "[4/4] Patching G1 URDF for Pinocchio gravity (5010 wrist + Inspire hand)..."
    python3 "${PKG_DIR}/scripts/patch_inspire_urdf.py" \
        "${URDF_CAND}" \
        "${CONFIG_DIR}/g1_29dof_inspire.urdf" \
        "${CONFIG_DIR}/g1_inspire_scene.xml"
else
    echo "[4/4] WARNING: stock G1 URDF not found; g1_29dof_inspire.urdf NOT generated."
fi

echo ""
echo "============================================"
echo " Done."
echo "   Body tree:      ${CONFIG_DIR}/inspire_hand_right_body.xml"
echo "   Assets:         ${CONFIG_DIR}/inspire_hand_right_assets.xml"
echo "   Injected MJCF:  ${G1_PATCHED}"
echo "   Pinocchio URDF: ${CONFIG_DIR}/g1_29dof_inspire.urdf (5010 + Inspire hand)"
echo "   Active joints:  6 (thumb_yaw, thumb_pitch, index/middle/ring/pinky prox)"
echo "   Slave joints:   6 (equality-coupled: int=ratio x master, solref=0.001 1)"
echo "   Wrist:          5010 (pitch/yaw ±13.4 N.m)"
echo ""
echo " Launch:"
echo "   ros2 launch g1_right_arm_reach mujoco_demo.launch.py \\"
echo "     urdf_path:=\${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf \\"
echo "     scene_path:=\${HOME}/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \\"
echo "     viewer:=true"
