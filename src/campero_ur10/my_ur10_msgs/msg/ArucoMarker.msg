#id of the mark
uint32 id
# pose of the mark
geometry_msgs/Pose pose
# positions of the corners in the image
geometry_msgs/Point32[] img_points
# mark to represent the ArUco mark in RViz
visualization_msgs/Marker mark_rviz
