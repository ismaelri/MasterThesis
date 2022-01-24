#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import Image
from my_ur10_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import cv2



def send_desired_p():
    
    pub_im = rospy.Publisher('/desired/image', Image, queue_size=10)
    pub_mp = rospy.Publisher('/desired/markers_pose', ArucoMarkerArray, queue_size=10)
    rospy.init_node('send_desired_p', anonymous=True)
    rate = rospy.Rate(200) # 10hz
    bridge = CvBridge()

    while not rospy.is_shutdown():

        cv2_img = cv2.imread('/home/ismael/catkin_ur10_ismael_ws/src/campero_ur10/desired_positions/desired_image.jpeg')
        image = bridge.cv2_to_imgmsg(cv2_img, "bgr8")
        
        desired_markers_pose = ArucoMarkerArray()
        desired_marker = ArucoMarker()
        corner_point = Point32()

        desired_marker.id = 582
        desired_marker.pose.position.x = 0.0104188928381
        desired_marker.pose.position.y = 0.0913594588637
        desired_marker.pose.position.z = 0.671579241753
        desired_marker.pose.orientation.x = -0.000112583625832
        desired_marker.pose.orientation.y = 0.998989002931
        desired_marker.pose.orientation.z = -0.00159421706268
        desired_marker.pose.orientation.w = -0.0449268051391
        corner_point.x = 1105.99987793
        corner_point.y = 605.342590332
        corner_point.z = 0.0
        desired_marker.img_points.append(corner_point)
        corner_point = Point32()
        corner_point.x = 1106.0
        corner_point.y = 855.082336426
        corner_point.z = 0.0
        desired_marker.img_points.append(corner_point)
        corner_point = Point32()
        corner_point.x = 858.999938965
        corner_point.y = 850.160827637
        corner_point.z = 0.0
        desired_marker.img_points.append(corner_point)
        corner_point = Point32()
        corner_point.x = 859.000061035
        corner_point.y = 604.015441895
        corner_point.z = 0.0
        desired_marker.img_points.append(corner_point)
        desired_markers_pose.markers.append(desired_marker)

        pub_im.publish(image)
        pub_mp.publish(desired_markers_pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        send_desired_p()
    except rospy.ROSInterruptException:
        pass


