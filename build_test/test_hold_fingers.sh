#!/usr/bin/env bash
# test_hold_fingers.sh — long home-hold: verify no thumb_yaw drift / finger trembling.
# Before the armature fix the [Wrist] line would show Ty drifting 0.25 → 1.3 and
# fingers 0.8 → limits. After: Ty stays 0.25, fingers stay 0.8, all stable.
PKILL() { pkill -9 -f mujoco_bridge_node.py 2>/dev/null; pkill -9 -f reach_node 2>/dev/null; pkill -9 -f hand_controller_node 2>/dev/null; pkill -9 -f action_sequencer_node 2>/dev/null; pkill -9 -f action_cli 2>/dev/null; }
PKILL; sleep 1
cd ~/g1_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
LOG=/tmp/hold_fingers.log
HOLD_S=${1:-60}
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep "$HOLD_S"
kill "$LAUNCH_PID" 2>/dev/null; PKILL; sleep 1
echo "=== [Wrist] finger lines (1/s) over ${HOLD_S}s ==="
grep -a '\[Wrist\]' "$LOG" | sed -E 's/.*\| (Ty:[0-9.\-]+\(d[0-9.\-]+\) Tp:[0-9.\-]+\(d[0-9.\-]+\) Ix:[0-9.\-]+\(d[0-9.\-]+\) Md:[0-9.\-]+ Rg:[0-9.\-]+ Pk:[0-9.\-]+).*/\1/' | head -${HOLD_S}
echo "=== first vs last Ty/Tp/Ix ==="
grep -a '\[Wrist\]' "$LOG" | head -1 | sed -E 's/.*\| (Ty:[0-9.\-]+\(d[0-9.\-]+\) Tp:[0-9.\-]+\(d[0-9.\-]+\) Ix:[0-9.\-]+\(d[0-9.\-]+\)).*/\1/'
grep -a '\[Wrist\]' "$LOG" | tail -1 | sed -E 's/.*\| (Ty:[0-9.\-]+\(d[0-9.\-]+\) Tp:[0-9.\-]+\(d[0-9.\-]+\) Ix:[0-9.\-]+\(d[0-9.\-]+\)).*/\1/'
echo "=== thumb_yaw drift check (should stay ~0.25) ==="
grep -a '\[Wrist\]' "$LOG" | sed -E 's/.*Ty:([0-9.\-]+)\(d([0-9.\-]+)\).*/\1 \2/' | awk '{printf "%.3f ", $1} END {print ""}' | head -c 400
echo ""
