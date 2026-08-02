#!/usr/bin/env bash
# Hold-only test: gravity FF only, NO reach command. Sample EE over time.
# If gravity FF + hold are correct, EE should be ~static.

PKILL() {
  pkill -9 -f mujoco_bridge_node.py 2>/dev/null
  pkill -9 -f reach_node 2>/dev/null
  pkill -9 -f hand_controller_node 2>/dev/null
  pkill -9 -f action_sequencer_node 2>/dev/null
  pkill -9 -f action_cli 2>/dev/null
  pkill -9 -f target_publisher_node 2>/dev/null
}

LOG=/tmp/sim_hold.log
PKILL; sleep 1

cd ~/g1_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/deps/unitree_ros_models/g1_description/g1_29dof_rev_1_0.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!

sleep 8
echo "=== gravity FF ==="
grep -a "gravity FF" "$LOG" | head -2
echo "=== hold-only: sample /actual_ee_pose 8x over ~8s (should be STATIC if gravity FF correct) ==="
for i in 1 2 3 4 5 6 7 8; do
  pos=$(timeout 2 ros2 topic echo /actual_ee_pose --once 2>/dev/null | grep -E "x:|y:|z:" | grep -oE '[-0-9.]+' | tr '\n' ' ')
  echo "t=${i}: ee=($pos)"
  sleep 1
done
echo "=== sample /joint_states arm velocities ==="
timeout 2 ros2 topic echo /joint_states --once 2>/dev/null | grep -A4 "right_shoulder_pitch_joint" | head -8

kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1
echo "=== wrist status (any oscillation while holding?) ==="
grep -a "\[Wrist\]" "$LOG" | head -6
echo "=== errors ==="
grep -a "DIVERGE\|FATAL\|Traceback" "$LOG" | head -3
