#!/usr/bin/env bash
# Capture /joint_commands effort (what reach_node sends) + joint_states during hold
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
LOG=/tmp/sim_diag_hold.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/deps/unitree_ros_models/g1_description/g1_29dof_rev_1_0.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 8
echo "=== /joint_commands effort (4s) ==="
timeout 4 ros2 topic echo /joint_commands --field effort > /tmp/cmd_eff.log 2>&1 &
C1=$!
echo "=== joint_states arm (4s) ==="
timeout 4 ros2 topic echo /joint_states --field position > /tmp/js_pos.log 2>&1 &
C2=$!
wait $C1 $C2 2>/dev/null
echo "cmd lines: $(wc -l < /tmp/cmd_eff.log) | js lines: $(wc -l < /tmp/js_pos.log)"
kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1
python3 - <<'PYEOF'
import re
# parse commanded efforts: messages have name[] and effort[]
cmd = open('/tmp/cmd_eff.log').read()
js = open('/tmp/js_pos.log').read()

def parse_effort(txt):
    blocks = txt.split('---')
    out = []
    for b in blocks:
        names = re.findall(r'name:\s*\n((?:\s*-\s*\S+\n?)+)', b)
        eff   = re.findall(r'effort:\s*\n((?:\s*-\s*[0-9.eE+-]+\n?)+)', b)
        if not names or not eff: continue
        ns = re.findall(r'-\s*(\S+)', names[0])
        es = [float(x) for x in re.findall(r'-\s*([0-9.eE+-]+)', eff[0])]
        out.append(dict(zip(ns, es)))
    return out

def parse_pos(txt):
    blocks = txt.split('---')
    out = []
    for b in blocks:
        names = re.findall(r'name:\s*\n((?:\s*-\s*\S+\n?)+)', b)
        pos   = re.findall(r'position:\s*\n((?:\s*-\s*[0-9.eE+-]+\n?)+)', b)
        if not names or not pos: continue
        ns = re.findall(r'-\s*(\S+)', names[0])
        ps = [float(x) for x in re.findall(r'-\s*([0-9.eE+-]+)', pos[0])]
        out.append(dict(zip(ns, ps)))
    return out

cmd_msgs = parse_effort(cmd)
js_msgs = parse_pos(js)
arm = ['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint']
print(f'{len(cmd_msgs)} cmd msgs, {len(js_msgs)} js msgs')
if cmd_msgs:
    print('\nfirst cmd effort:', {j: round(cmd_msgs[0][j],3) for j in arm})
    print('mid   cmd effort:', {j: round(cmd_msgs[len(cmd_msgs)//2][j],3) for j in arm})
    print('last  cmd effort:', {j: round(cmd_msgs[-1][j],3) for j in arm})
if js_msgs:
    print('\nfirst js pose:', {j: round(js_msgs[0][j],3) for j in arm})
    print('mid   js pose:', {j: round(js_msgs[len(js_msgs)//2][j],3) for j in arm})
    print('last  js pose:', {j: round(js_msgs[-1][j],3) for j in arm})
PYEOF
