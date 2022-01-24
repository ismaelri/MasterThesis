#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from my_ur10_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import cv2


bridge = CvBridge()

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard '+ error)
    global cnt
    if cnt == 0:
        print("desired_marker.id = " + str(data.markers[0].id))
        print("desired_marker.pose.position.x = " + str(data.markers[0].pose.position.x))
        print("desired_marker.pose.position.y = " + str(data.markers[0].pose.position.y))
        print("desired_marker.pose.position.z = " + str(data.markers[0].pose.position.z))
        print("desired_marker.pose.orientation.x = " + str(data.markers[0].pose.orientation.x))
        print("desired_marker.pose.orientation.y = " + str(data.markers[0].pose.orientation.y))
        print("desired_marker.pose.orientation.z = " + str(data.markers[0].pose.orientation.z))
        print("desired_marker.pose.orientation.w = " + str(data.markers[0].pose.orientation.w))
        for j in range(0, 4):
            print("corner_point.x = " + str(data.markers[0].img_points[j].x))
            print("corner_point.y = " + str(data.markers[0].img_points[j].y))
            print("corner_point.z = " + str(data.markers[0].img_points[j].z))
            print("desired_marker.img_points.append(corner_point)")
            print("corner_point = Point32()")
        print("desired_markers_pose.markers.append(desired_marker)")
        print("desired_marker = ArucoMarker()")
        cnt += 1


def callback2(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard '+ error)
    global cnt2
    if cnt2 == 0:
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite('/home/ismael/catkin_ur10_ismael_ws/src/campero_ur10/desired_positions/desired_image.jpeg', cv2_img)
            rospy.sleep(1)
        cnt2 += 1


    



def desired_p():

    rospy.init_node('desired_p', anonymous=True)

    rospy.Subscriber('/aruco_multiple/markers_pose', ArucoMarkerArray, callback)
    rospy.Subscriber('/camera/color/image_raw', Image, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    cnt = 0
    cnt2 = 0
    desired_p()


