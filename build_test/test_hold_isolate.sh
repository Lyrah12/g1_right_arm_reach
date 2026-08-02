#!/usr/bin/env bash
# Isolated test: pure gravity hold (NO target). Detect arm drift during hold phase.
PKILL() {
  pkill -9 -f mujoco_bridge_node.py 2>/dev/null
  pkill -9 -f reach_node 2>/dev/null
  pkill -9 -f hand_controller_node 2>/dev/null
  pkill -9 -f action_sequencer_node 2>/dev/null
}
PKILL; sleep 1
cd ~/g1_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
LOG=/tmp/sim_hold3.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/deps/unitree_ros_models/g1_description/g1_29dof_rev_1_0.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 10
echo "=== capturing joint_states (pure hold) 8s ==="
timeout 8 ros2 topic echo /joint_states > /tmp/js_hold3.log 2>&1
echo "captured: $(wc -l < /tmp/js_hold3.log) lines"
kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1
python3 - <<'PYEOF'
import re
txt=open('/tmp/js_hold3.log').read()
blocks=txt.split('---')
def get_pos(b):
    names=re.findall(r'name:\s*\n((?:\s*-\s*\S+\n?)+)',b)
    pos=re.findall(r'position:\s*\n((?:\s*-\s*[0-9.eE+-]+\n?)+)',b)
    if not names or not pos: return None
    ns=re.findall(r'-\s*(\S+)',names[0])
    ps=[float(x) for x in re.findall(r'-\s*([0-9.eE+-]+)',pos[0])]
    d=dict(zip(ns,ps))
    arm=['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint',
         'right_wrist_roll_joint','right_wrist_pitch_joint','right_wrist_yaw_joint']
    return {a:round(d[a],3) if a in d else None for a in arm}
first=get_pos(blocks[1]) if len(blocks)>1 else None
last=get_pos(blocks[-2]) if len(blocks)>1 else None
print('first:',first)
print('last :',last)
PYEOF
