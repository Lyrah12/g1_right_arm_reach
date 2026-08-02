#!/usr/bin/env bash
# Capture arm joint trajectory from t=1.5s to t=10s after launch
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
LOG=/tmp/sim_init_traj.log
ros2 launch g1_right_arm_reach mujoco_demo.launch.py \
  urdf_path:=$HOME/deps/unitree_ros_models/g1_description/g1_29dof_rev_1_0.urdf \
  scene_path:=$HOME/g1_ws/src/g1_right_arm_reach/config/g1_inspire_scene.xml \
  viewer:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "launched. waiting 2s then capturing 9s..."
sleep 2
timeout 9 ros2 topic echo /joint_states --field position > /tmp/js_traj.log 2>&1
kill "$LAUNCH_PID" 2>/dev/null
PKILL; sleep 1
echo "captured: $(wc -l < /tmp/js_traj.log) lines"
python3 - <<'PYEOF'
import re
txt=open('/tmp/js_traj.log').read()
blocks=txt.split('---')
arm_idx={'right_shoulder_pitch_joint':0,'right_shoulder_roll_joint':1,'right_shoulder_yaw_joint':2,
         'right_elbow_joint':3,'right_wrist_roll_joint':4,'right_wrist_pitch_joint':5,'right_wrist_yaw_joint':6}
def vecs(b):
    m=re.findall(r'-\s*([0-9.eE+-]+)',b)
    if len(m)<7: return None
    return [float(x) for x in m[:7]]
samples=[]
for b in blocks:
    v=vecs(b)
    if v: samples.append(v)
print(f'{len(samples)} samples, {len(samples)/9:.0f} Hz')
print('time(s)   sp      ro      ya      elb     wroll   wpitch  wyaw')
n=len(samples)
step=max(1,n//18)
for i in range(0,n,step):
    v=samples[i]
    print(f'{i*0.05:6.2f}   {v[0]:+6.3f} {v[1]:+6.3f} {v[2]:+6.3f} {v[3]:+6.3f} {v[4]:+6.3f} {v[5]:+6.3f} {v[6]:+6.3f}')
print(f'last:   {samples[-1][0]:+6.3f} {samples[-1][1]:+6.3f} {samples[-1][2]:+6.3f} {samples[-1][3]:+6.3f} {samples[-1][4]:+6.3f} {samples[-1][5]:+6.3f} {samples[-1][6]:+6.3f}')
PYEOF
