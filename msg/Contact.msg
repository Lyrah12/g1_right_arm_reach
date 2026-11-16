# One MuJoCo contact. Position, normal and wrench are expressed in world.
int32 geom1_id
int32 geom2_id
string geom1_name
string geom2_name
string body1_name
string body2_name
geometry_msgs/Point position
# Unit normal pointing from geom1 to geom2.
geometry_msgs/Vector3 normal
# Signed separation [m]; negative means penetration.
float64 distance
# True if this contact participates in the constraint solver.
bool active
# Compressive normal force [N], zero for inactive contacts.
float64 normal_force
# Force [N] and torque [N*m] applied to geom2, about the contact point.
# The equal and opposite wrench acts on geom1.
geometry_msgs/Wrench wrench
