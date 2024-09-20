string id #Unique name for the target
string description # description of the target

int8 CARTESIAN = 0 # if the target is relative to the previous
int8 JOINT = 1 # if the target contains absolute joint position
int8 space_type # CARTESIAN OR JOINT

int8 RELATIVE = 0 # if the target is relative to the previous
int8 ABSOLUTE = 1 # if the target contains absolute joint position
int8 scale_type # RELATIVE OR ABSOLUTE

# Only one array per target type can be filled and ONLY the first element is used!
# If both only joint target is considered
JointTarget joint_target
CartesianTarget cartesian_target


float32 max_velocity_scaling_factor # scaling factor to apply at the max velocity in joint space
float32 max_acceleration_scaling_factor  # scaling factor to apply at the max acceleration in joint space (indirectly limits the jerk (a_max-a_min)/0.001 (0.001 = dt))
float32 blending_radius