# Message emitted by Nao landmark detection module.
# Contains information about the landmarks detected

Header header

# Vectors holding the landmark information
std_msgs/Int32[] mark_ids
std_msgs/Float32[] shape_alpha
std_msgs/Float32[] shape_beta
std_msgs/Float32[] shape_sizex
std_msgs/Float32[] shape_sizey

geometry_msgs/Pose camera_local_pose

geometry_msgs/Pose camera_world_pose

std_msgs/String camera_name
