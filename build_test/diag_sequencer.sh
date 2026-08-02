#!/usr/bin/env bash
PKILL() { pkill -9 -f mujoco_bridge_node.py 2>/dev/null; pkill -9 -f reach_node 2>/dev/null; pkill -9 -f hand_controller_node 2>/dev/null; pkill -9 -f action_sequencer_node 2>/dev/null; pkill -9 -f action_cli 2>/dev/null; }
PKILL; sleep 1
cd ~/g1_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
LOG=/tmp/diag_seq.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  use_rviz:=false viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 10
echo "=== nodes ==="
ros2 node list 2>/dev/null | head -8
echo "=== echo /action_command (background) ==="
timeout 20 ros2 topic echo /action_command > /tmp/act_cmd.txt 2>/dev/null &
ECHO_PID=$!
sleep 2
echo "=== publish reach_and_grasp ==="
ros2 run g1_right_arm_reach action_cli.py reach_and_grasp 0.20 -0.15 -0.15 pinch 2>&1 | grep -a OK
sleep 6
echo "=== action_command echo received? ==="
head -20 /tmp/act_cmd.txt
kill $ECHO_PID 2>/dev/null
echo "=== sequencer output ==="
grep -a 'action_sequencer' "$LOG" | grep -v INFO | head -10
echo "=== reach_node output ==="
grep -a 'reach_node' "$LOG" | grep '\[ARM\] tgt\|\[TARGET\]' | head -4
kill "$LAUNCH_PID" 2>/dev/null; PKILL; sleep 1
