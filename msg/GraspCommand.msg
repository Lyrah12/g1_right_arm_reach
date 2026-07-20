# GraspCommand.msg — Legacy grasp-only command (Plan C backward compat).
# Use HandCommand.msg for new code (includes wrist orientation).
# Published to /grasp_command, consumed by hand_controller.

string   grasp_type        # Pattern name from GraspLibrary (open/pinch/close/...)
float64  duration          # Interpolation duration (s), 0 = use default
float64  force_limit       # Max joint torque (N·m), 0 = position only (reserved)
float64[] custom_joints    # Custom joint angles (grasp_type="custom" only)
string[] custom_names      # Joint names for custom mode (grasp_type="custom" only)
