# ActionCommand.msg — Unified action command (arm + hand, Plan C).
# Published to /action_command. Consumed by ActionSequencerNode.
# Direct control: publish to /target_point or /hand_command (bypass sequencer).

# ── Action type ──
string   action_type         # reach | grasp | reach_and_grasp | home

# ── Arm target (reach / reach_and_grasp) ──
float64  target_x  0.0       # Shoulder-relative X (m, forward)
float64  target_y  0.0       # Shoulder-relative Y (m, left)
float64  target_z  0.0       # Shoulder-relative Z (m, up)
float64  ori_x     0.0       # Wrist orientation quat X (relative to palm)
float64  ori_y     0.0       # Wrist orientation quat Y
float64  ori_z     0.0       # Wrist orientation quat Z
float64  ori_w     1.0       # Wrist orientation quat W

# ── Grasp (grasp / reach_and_grasp) ──
string   grasp_type          # Pattern name from GraspLibrary
float64  grasp_duration 0.5  # Interpolation duration (s)

# ── Timing ──
float64  convergence_timeout 5.0   # Max wait for arm convergence (s)
float64  settle_time         0.15  # Settle time before grasp (s)
