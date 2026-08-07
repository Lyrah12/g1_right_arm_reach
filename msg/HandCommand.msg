# Revo2 finger-only command; wrist roll belongs to EndEffectorCommand.
string grasp_type
float64 grasp_duration
bool use_joint_positions
# Official ROS2 order: thumb flexion, thumb opposition, index, middle, ring, pinky.
float64[6] joint_positions
int32 command_id
