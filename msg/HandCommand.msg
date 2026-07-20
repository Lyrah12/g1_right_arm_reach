# HandCommand.msg — Unified hand command (wrist orientation + finger grasp).
# Published to /hand_command. Consumed by hand_controller_node (Plan C).

# ── Wrist orientation target (quaternion) ──
# Identity (0,0,0,1) = no orientation change (hold current).
# Non-identity = rotation RELATIVE to current palm orientation.
# [0.5,0.5,0.5,0.5] = reset to default injection quat (absolute world-frame target).
float64  ori_x  0.0
float64  ori_y  0.0
float64  ori_z  0.0
float64  ori_w  1.0

# ── Grasp parameters ──
string   grasp_type          # Pattern name from GraspLibrary
float64  grasp_duration 0.5  # Interpolation duration (s)

# ── Compliance (reserved) ──
float64  force_limit  0.0    # Max joint torque (N·m), 0 = position only
string   control_mode  "impedance"  # "impedance" | "position" (future)
