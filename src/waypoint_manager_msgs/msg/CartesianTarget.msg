string frame_id  # The frame of the reference system
geometry_msgs/Pose pose # Desired motion

# seed in joint space for the target (ignored if empty)
float64[] seed

# name of the tip frame if empty the default one will be used
string tip_frame