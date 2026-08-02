#!/usr/bin/env bash
PKILL() { pkill -9 -f mujoco_bridge_node.py 2>/dev/null; pkill -9 -f reach_node 2>/dev/null; pkill -9 -f hand_controller_node 2>/dev/null; pkill -9 -f action_sequencer_node 2>/dev/null; pkill -9 -f action_cli 2>/dev/null; }
PKILL; sleep 1
cd ~/g1_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
LOG=/tmp/cap.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_29dof_inspire.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 10
ros2 run g1_right_arm_reach action_cli.py reach 0.20 -0.15 -0.15 2>&1 | grep -a OK
sleep 22
ros2 topic echo /joint_states --once > /tmp/js_captured.txt 2>/dev/null
kill "$LAUNCH_PID" 2>/dev/null; PKILL; sleep 1
python3 - <<'PYEOF'
import re
txt = open('/tmp/js_captured.txt').read()
# ros2 topic echo output: YAML with dash lists. Parse each block: <key>:\n  - val...
def get_list(block):
    items = re.findall(r'^\s*-\s*(.+?)\s*$', block, re.M)
    return [x for x in items]
m = re.split(r'\n(?=name:|position:|velocity:|effort:)', txt)
d = {}
for b in m:
    k = re.match(r'(\w+):', b)
    if k and k.group(1) not in d:
        d[k.group(1)] = get_list(b)
want = ['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint',
        'right_elbow_joint','right_wrist_roll_joint','right_wrist_pitch_joint','right_wrist_yaw_joint']
names = d.get('name', []); pos = d.get('position', []); vel = d.get('velocity', [])
posmap = dict(zip(names, pos)); velmap = dict(zip(names, vel))
q = [round(float(posmap[j]),4) for j in want if j in posmap]
print('CONVERGED_Q =', q)
print('CONVERGED_VEL =', [round(float(velmap[j]),4) if j in velmap else 0 for j in want])
PYEOF
