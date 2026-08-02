#!/usr/bin/env bash
# Headless stability test: reach target, observe arm+wrist convergence.

PKILL() {
  pkill -9 -f mujoco_bridge_node.py 2>/dev/null
  pkill -9 -f reach_node 2>/dev/null
  pkill -9 -f hand_controller_node 2>/dev/null
  pkill -9 -f action_sequencer_node 2>/dev/null
  pkill -9 -f action_cli 2>/dev/null
  pkill -9 -f target_publisher_node 2>/dev/null
}

LOG=/tmp/sim_stability.log
PKILL; sleep 1

cd ~/g1_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=== launch ==="
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/deps/unitree_ros_models/g1_description/g1_29dof_rev_1_0.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "launch pid=$LAUNCH_PID"

sleep 8
echo "=== nodes ==="
ros2 node list 2>/dev/null | head

echo "=== gravity FF (should be +4 sp, +2.4 elbow after sign fix) ==="
grep -a "gravity FF" "$LOG" | head -3

echo "=== send reach 0.18 -0.05 -0.10 ==="
ros2 run g1_right_arm_reach action_cli.py reach 0.18 -0.05 -0.10 2>&1 | grep -a "OK\|REACH"

echo "=== wait 12s ==="
sleep 12

kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1

echo ""
echo "########## [ARM] status (reach_node, 1Hz) ##########"
grep -a "\[ARM\]" "$LOG" | tail -12

echo ""
echo "########## [Wrist] status (hand_controller, 1Hz) ##########"
grep -a "\[Wrist\]" "$LOG" | tail -8

echo ""
echo "########## sequencer result ##########"
grep -a "\[SEQ\]\|\[CMD\]\|TIMEOUT\|converged" "$LOG" | head -8

echo ""
echo "########## gravity / errors ##########"
grep -a "gravity FF\|DIVERGE\|Traceback\|FATAL" "$LOG" | head -6
