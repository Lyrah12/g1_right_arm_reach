#!/usr/bin/env bash
# Capture /mujoco_gravity + joint_states simultaneously; check FF matches pose
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
LOG=/tmp/sim_grav_match.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/deps/unitree_ros_models/g1_description/g1_29dof_rev_1_0.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 8
echo "=== /mujoco_gravity (4s) ==="
timeout 4 ros2 topic echo /mujoco_gravity --field data > /tmp/grav.log 2>&1 &
G1=$!
echo "=== /joint_commands effort (4s) ==="
timeout 4 ros2 topic echo /joint_commands --field effort > /tmp/cmd2.log 2>&1 &
C1=$!
wait $G1 $C1 2>/dev/null
kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1
echo "grav: $(wc -l < /tmp/grav.log) | cmd: $(wc -l < /tmp/cmd2.log)"
python3 - <<'PYEOF'
import re
def parse_arr(fn):
    txt=open(fn).read()
    vals=[]
    for m in re.findall(r'\[([0-9.,\s+eE-]+)\]',txt):
        try:
            vals.append([float(x) for x in m.split(',') if x.strip()])
        except: pass
    return vals
g=parse_arr('/tmp/grav.log')
c=parse_arr('/tmp/cmd2.log')
print(f'{len(g)} gravity msgs, {len(c)} cmd msgs')
if g:
    print('gravity first:', [round(x,3) for x in g[0]])
    print('gravity mid  :', [round(x,3) for x in g[len(g)//2]])
    print('gravity last :', [round(x,3) for x in g[-1]])
if c:
    print('cmd first:', [round(x,3) for x in c[0]])
    print('cmd mid  :', [round(x,3) for x in c[len(c)//2]])
    print('cmd last :', [round(x,3) for x in c[-1]])
# check gravity internal consistency: qfrc_bias sp should track pose
PYEOF
