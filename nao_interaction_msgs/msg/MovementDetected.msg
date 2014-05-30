# Message emitted by Nao movement detection module.
# Contains information about the motion detection module

Header header

# [x,y] contains the angular coordinates (in radians) of the center of gravity of the cluster
geometry_msgs/Point gravity_center

# [vx,vy] corresponds to the mean velocity of the cluster, computed from the velocity of all the moving points. It is an angular velocity, expressed in radians/s
geometry_msgs/Point mean_velocity

# Coordinates of the moving points
geometry_msgs/Point[] points_poses

# Velocities of the moving points
geometry_msgs/Point[] points_speeds
