# id of the marker
uint32 id
# pose of the marker
geometry_msgs/Pose pose
# positions of the corners in the image
geometry_msgs/Point32[] img_points
# marker to represent the ArUco marker in RViz
visualization_msgs/Marker mark_rviz
