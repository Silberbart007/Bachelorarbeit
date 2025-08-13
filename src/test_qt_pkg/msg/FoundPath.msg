#message format for path found by planning algorithm
geometry_msgs/Pose[] path   #all points along the path, including start and finish
int32 path_points   #num of path points, 0 represents no possible path
float32 cost
