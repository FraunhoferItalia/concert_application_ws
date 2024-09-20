string id #Unique name for the motion
string description # description of the motion
string pipeline_id # id of the pipeline used for planning (ompl, pilz_industrial_motion_planner)

int8 CARTESIAN = 0 # cartesian motion
int8 JOINT = 1 # joint space motion

# type (CARTESIAN_MOTION or JOINT_MOTION)
int8 motion_type

# Constraints of the motion
moveit_msgs/Constraints moveit_constraints

# sequence of targets
Target[] targets