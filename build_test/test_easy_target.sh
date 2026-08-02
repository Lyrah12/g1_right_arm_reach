#!/usr/bin/env bash
PKILL() {
  pkill -9 -f mujoco_bridge_node.py 2>/dev/null
  pkill -9 -f reach_node 2>/dev/null
  pkill -9 -f hand_controller_node 2>/dev/null
  pkill -9 -f action_sequencer_node 2>/dev/null
  pkill -9 -f action_cli 2>/dev/null
}
TARGET="${*:-0.10 0.0 0.05}"
PKILL; sleep 1
cd ~/g1_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
LOG=/tmp/sim_easy.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 10
echo "=== target: $TARGET ==="
for i in 1 2 3; do
  ros2 run g1_right_arm_reach action_cli.py reach $TARGET 2>&1 | grep -a "OK"
  sleep 1
done
sleep 12
kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1
echo "=== [ARM] ==="
grep -a "tgt=" "$LOG" | tail -6
echo "=== [SEQ] ==="
grep -a "\[SEQ\]\|\[CMD\]" "$LOG" | tail -4
