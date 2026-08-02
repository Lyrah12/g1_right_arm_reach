#!/usr/bin/env bash
PKILL() {
  pkill -9 -f mujoco_bridge_node.py 2>/dev/null
  pkill -9 -f reach_node 2>/dev/null
  pkill -9 -f hand_controller_node 2>/dev/null
  pkill -9 -f action_sequencer_node 2>/dev/null
  pkill -9 -f action_cli 2>/dev/null
}
PKILL; sleep 1
cd ~/g1_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
LOG=/tmp/sim_stab2.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 10
echo "=== nodes ==="
ros2 node list | head -6
echo "=== send reach twice ==="
for i in 1 2; do
  ros2 run g1_right_arm_reach action_cli.py reach 0.18 -0.05 -0.10 2>&1 | grep -a "OK\|REACH"
  sleep 1
done
sleep 18
kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1
echo "=== [ARM] status (tgt=) ==="
grep -a "tgt=" "$LOG" | tail -8
echo "=== [SEQ]/[CMD] ==="
grep -a "\[SEQ\]\|\[CMD\]" "$LOG" | tail -6
echo "=== [Wrist] last 4 ==="
grep -a "\[Wrist\]" "$LOG" | tail -4
