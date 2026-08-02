#!/usr/bin/env bash
# Launch bridge+reach_node only. Capture arm trajectory 8s. Does arm still drift?
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
LOG=/tmp/sim_armonly.log
# ensure the new launch file is installed
cp -r /mnt/d/Users/Lenovo/Desktop/g1_right_arm_reach/launch/mujoco_armonly.launch.py ~/g1_ws/src/g1_right_arm_reach/launch/
colcon build --packages-select g1_right_arm_reach > /tmp/build_armonly.log 2>&1
source install/setup.bash
ros2 launch g1_right_arm_reach mujoco_armonly.launch.py \
  urdf_path:=$HOME/deps/unitree_ros_models/g1_description/g1_29dof_rev_1_0.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  > "$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 3
echo "=== nodes ==="
ros2 node list 2>/dev/null
echo "=== capturing 8s ==="
timeout 8 ros2 topic echo /joint_states > /tmp/js_armonly.log 2>&1
kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1
python3 - <<'PYEOF'
import re
txt=open('/tmp/js_armonly.log').read()
blocks=txt.split('---')
def parse(b):
    names=re.findall(r'name:\s*\n((?:\s*-\s*\S+\n?)+)',b)
    pos=re.findall(r'position:\s*\n((?:\s*-\s*[0-9.eE+-]+\n?)+)',b)
    if not names or not pos: return None
    ns=re.findall(r'-\s*(\S+)',names[0]); ps=[float(x) for x in re.findall(r'-\s*([0-9.eE+-]+)',pos[0])]
    return dict(zip(ns,ps))
samples=[s for s in (parse(b) for b in blocks) if s]
print(f'{len(samples)} samples over 8s')
arm=['right_shoulder_pitch_joint','right_elbow_joint','right_wrist_pitch_joint']
print('t(s)  sp     elb    wp')
n=len(samples); step=max(1,n//12)
for i in range(0,n,step):
    s=samples[i]
    print(f'{i*0.05:5.2f}  {s[arm[0]]:+6.3f} {s[arm[1]]:+6.3f} {s[arm[2]]:+6.3f}')
s=samples[-1]
print(f'end   {s[arm[0]]:+6.3f} {s[arm[1]]:+6.3f} {s[arm[2]]:+6.3f}')
PYEOF
