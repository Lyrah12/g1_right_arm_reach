#!/usr/bin/env bash
set -eo pipefail

package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-jazzy}"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source "/opt/ros/${ros_distro}/setup.bash"
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi

echo "[1/7] Regenerate deterministic G1 23DOF + Revo2 models"
python3 "${package_dir}/scripts/compose_g1_23dof_revo2.py"

echo "[2/7] Validate controller architecture"
python3 - <<PY
from pathlib import Path
import yaml

package = Path(r"${package_dir}")
params = yaml.safe_load((package / "config" / "params.yaml").read_text())
reach = params["reach_node"]["ros__parameters"]
hand = params["hand_controller"]["ros__parameters"]
sequence = params["action_sequencer"]["ros__parameters"]
arm_joints = set(reach["robot_model"]["active_joints"])
hand_joints = set(hand["joint_names"])

assert reach["control_rate"] == 50.0, "reach_node must run at 50 Hz"
assert hand["control_rate"] == 50.0, "hand_controller must run at 50 Hz"
assert sequence["control_rate"] == 50.0, "action_sequencer must run at 50 Hz"
assert reach["trajectory"]["max_linear_acceleration"] > 0.0
assert reach["trajectory"]["max_roll_acceleration"] > 0.0
assert not (arm_joints & hand_joints), "arm and hand joint sets must be disjoint"
assert reach["task_pid"]["use_external_gravity"] is False, (
    "hardware default must use model gravity, not a simulation topic")
assert reach["topics"]["deadman"] != hand["topics"]["deadman"], (
    "arm and hand must have independent deadman topics")
launch_text = (package / "launch" / "hardware_controllers.launch.py").read_text()
assert launch_text.count('"safety.deadman_required": True') >= 2
assert '"control_rate": 200.0' in launch_text
pid_text = (package / "src" / "pid_controller.cpp").read_text()
assert "target_velocity - task_velocity" in pid_text, (
    "moving-reference velocity feedforward must remain enabled")
print("  50 Hz high-level loops; disjoint arm/hand joints; hardware deadmen enabled")
PY

echo "[3/7] Validate URDF tree"
check_urdf "${package_dir}/config/models/g1_23dof_revo2.urdf" >/dev/null

echo "[4/7] Load both MJCF models with MuJoCo"
python3 - <<PY
import mujoco
from pathlib import Path
base = Path(r"${package_dir}") / "config" / "models"
for name in ("g1_23dof_revo2.xml", "g1_23dof_revo2_arm_only.xml"):
    model = mujoco.MjModel.from_xml_path(str(base / name))
    print(f"  {name}: nq={model.nq}, nv={model.nv}, nu={model.nu}, neq={model.neq}")
PY

echo "[5/7] Build package"
cd "${workspace_dir}"
colcon build --symlink-install --packages-select g1_right_arm_reach \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "[6/7] Run C++ tests against combined URDF"
source "${workspace_dir}/install/setup.bash"
export G1_TEST_URDF_PATH="${package_dir}/config/models/g1_23dof_revo2.urdf"
colcon test --packages-select g1_right_arm_reach --event-handlers console_direct+

echo "[7/7] Report test results"
colcon test-result --verbose
echo "Validation complete"
