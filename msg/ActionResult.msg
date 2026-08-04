# ActionResult.msg — one-shot outcome report from ActionSequencerNode (P0-2).
# Published to /action_result when an action finishes (success or abort), so
# callers / tests can verify that a timeout or validation failure was acted on
# (not silently ignored), and logs can correlate a failure to its action_id.
int32   action_id             # Echo of ActionCommand.action_id
bool    success
string  reason                # CONVERGED | DONE | REACH_TIMEOUT | ACTION_BUDGET | ...
float64 position_error_m      # Final position error at finish (m)
float64 orientation_error_rad # Final orientation error at finish (rad)
float64 duration_s            # Total action duration (REACHING+SETTLING, s)
