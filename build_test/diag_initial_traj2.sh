#!/usr/bin/env bash
# Capture FULL joint_states (names+positions) from t=2s for 8s, parse by name
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
LOG=/tmp/sim_init2.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/deps/unitree_ros_models/g1_description/g1_29dof_rev_1_0.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "launched. capturing 9s..."
sleep 2
timeout 9 ros2 topic echo /joint_states > /tmp/js_traj2.log 2>&1
kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1
echo "captured: $(wc -l < /tmp/js_traj2.log) lines"
python3 - <<'PYEOF'
import re
txt=open('/tmp/js_traj2.log').read()
blocks=txt.split('---')
def parse(b):
    names=re.findall(r'name:\s*\n((?:\s*-\s*\S+\n?)+)',b)
    pos=re.findall(r'position:\s*\n((?:\s*-\s*[0-9.eE+-]+\n?)+)',b)
    if not names or not pos: return None
    ns=re.findall(r'-\s*(\S+)',names[0])
    ps=[float(x) for x in re.findall(r'-\s*([0-9.eE+-]+)',pos[0])]
    return dict(zip(ns,ps))
samples=[parse(b) for b in blocks]
samples=[s for s in samples if s]
print(f'{len(samples)} samples')
arm=['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint','right_wrist_pitch_joint','right_wrist_yaw_joint']
# print name order from first msg
print('name order (first):', list(samples[0].keys())[:15])
print('\nt(s)  sp     ro     ya     elb    wp     wy')
n=len(samples)
step=max(1,n//15)
for i in range(0,n,step):
    s=samples[i]
    print(f'{i*0.05:5.2f}  '+ ' '.join(f'{s[a]:+6.3f}' for a in arm))
s=samples[-1]
print(f'end  '+ ' '.join(f'{s[a]:+6.3f}' for a in arm))
PYEOF
