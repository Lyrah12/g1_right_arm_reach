# One-shot outcome report from ActionSequencerNode.
int32 action_id
bool success
# CONVERGED | DONE | REACH_TIMEOUT | UNKNOWN_ACTION | ...
string reason
float64 position_error_m
# Final calibrated tool-axis roll error.
float64 orientation_error_rad
float64 duration_s
