#!/usr/bin/env bash
PKILL() { pkill -9 -f mujoco_bridge_node.py 2>/dev/null; pkill -9 -f reach_node 2>/dev/null; pkill -9 -f hand_controller_node 2>/dev/null; pkill -9 -f action_sequencer_node 2>/dev/null; pkill -9 -f action_cli 2>/dev/null; }
PKILL; sleep 1
cd ~/g1_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
LOG=/tmp/sim_long.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 10
ros2 run g1_right_arm_reach action_cli.py reach 0.20 -0.15 -0.15 2>&1 | grep -a OK
sleep 25
kill "$LAUNCH_PID" 2>/dev/null; PKILL; sleep 1
echo '=== last 14 [ARM] ==='
grep -a '\[ARM\]' "$LOG" | tail -14
echo '=== SEQ ==='
grep -a '\[SEQ\]' "$LOG" | tail -3
