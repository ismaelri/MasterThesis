#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray, UInt32
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import Image
from my_ur10_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
#geometry_msgs/Point32[] img_points

def callback(info_data):
    global num_ite 
    num_ite = info_data.data

def send_desired_i():
    
    pub_im = rospy.Publisher('/desired/image', Image, queue_size=10)
    pub_mp = rospy.Publisher('/desired/markers_pose', ArucoMarkerArray, queue_size=10)
    rospy.Subscriber('/interaction_num', UInt32, callback)
    rospy.init_node('send_desired_i', anonymous=True)
    rate = rospy.Rate(200) # 10hz
    bridge = CvBridge()
    global interaction
    global num_ite

    while not rospy.is_shutdown():

        if interaction == "False":
            cv2_img = cv2.imread('/home/ismael/catkin_ur10_ismael_ws/src/campero_ur10/desired_positions/IBVC/desired_image_p2.jpeg')
            image = bridge.cv2_to_imgmsg(cv2_img, "bgr8")

            desired_markers_pose = ArucoMarkerArray()
            desired_marker = ArucoMarker()
            corner_point = Point32()

            desired_marker.id = 10
            desired_marker.pose.position.x = 0.046693585813
            desired_marker.pose.position.y = 0.0568988919258
            desired_marker.pose.position.z = 0.403495699167
            desired_marker.pose.orientation.x = 0.00534729107636
            desired_marker.pose.orientation.y = 0.999728226191
            desired_marker.pose.orientation.z = -0.0220885499086
            desired_marker.pose.orientation.w = 0.0051938615423
            corner_point.x = 1221.33569336
            corner_point.y = 631.295349121
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 1225.36621094
            corner_point.y = 838.012207031
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 1018.74523926
            corner_point.y = 840.823303223
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 1016.00683594
            corner_point.y = 633.252868652
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            desired_markers_pose.markers.append(desired_marker)
            desired_marker = ArucoMarker()
            desired_marker.id = 17
            desired_marker.pose.position.x = -0.0235830619931
            desired_marker.pose.position.y = 0.124452121556
            desired_marker.pose.position.z = 0.403962761164
            desired_marker.pose.orientation.x = 0.00561065303403
            desired_marker.pose.orientation.y = 0.998407435329
            desired_marker.pose.orientation.z = -0.0556382986516
            desired_marker.pose.orientation.w = 0.00744938756578
            corner_point.x = 980.728942871
            corner_point.y = 860.710083008
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 983.053222656
            corner_point.y = 1072.17871094
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 775.99822998
            corner_point.y = 1075.71179199
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 776.01171875
            corner_point.y = 863.389587402
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            desired_markers_pose.markers.append(desired_marker)
            desired_marker = ArucoMarker()
            desired_marker.id = 26
            desired_marker.pose.position.x = -0.0239146091044
            desired_marker.pose.position.y = -0.00966113992035
            desired_marker.pose.position.z = 0.404481321573
            desired_marker.pose.orientation.x = 0.00418398668982
            desired_marker.pose.orientation.y = 0.999482945335
            desired_marker.pose.orientation.z = -0.029958029215
            desired_marker.pose.orientation.w = 0.0109019597413
            corner_point.x = 979.702331543
            corner_point.y = 404.245819092
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 981.752746582
            corner_point.y = 608.856262207
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 775.000061035
            corner_point.y = 610.963195801
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 774.999938965
            corner_point.y = 405.29675293
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            desired_markers_pose.markers.append(desired_marker)
            desired_marker = ArucoMarker()
            desired_marker.id = 91
            desired_marker.pose.position.x = 0.0468087084591
            desired_marker.pose.position.y = -0.00983435474336
            desired_marker.pose.position.z = 0.40441980958
            desired_marker.pose.orientation.x = 0.00408920974068
            desired_marker.pose.orientation.y = 0.999481177564
            desired_marker.pose.orientation.z = -0.029169450551
            desired_marker.pose.orientation.w = 0.0130306259626
            corner_point.x = 1220.71887207
            corner_point.y = 403.712463379
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 1224.82189941
            corner_point.y = 608.346130371
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 1018.94226074
            corner_point.y = 610.16015625
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 1016.61547852
            corner_point.y = 404.702453613
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            desired_markers_pose.markers.append(desired_marker)
            desired_marker = ArucoMarker()
            desired_marker.id = 200
            desired_marker.pose.position.x = -0.0237675141543
            desired_marker.pose.position.y = 0.0571288838983
            desired_marker.pose.position.z = 0.40377920866
            desired_marker.pose.orientation.x = 0.00526252310797
            desired_marker.pose.orientation.y = 0.999808639578
            desired_marker.pose.orientation.z = -0.0184994017305
            desired_marker.pose.orientation.w = 0.00357242384862
            corner_point.x = 980.278015137
            corner_point.y = 631.804260254
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 982.31652832
            corner_point.y = 838.706665039
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 776.088256836
            corner_point.y = 841.29107666
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 774.66998291
            corner_point.y = 633.965576172
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            desired_markers_pose.markers.append(desired_marker)
            desired_marker = ArucoMarker()
            desired_marker.id = 582
            desired_marker.pose.position.x = 0.0467164032161
            desired_marker.pose.position.y = 0.123768940568
            desired_marker.pose.position.z = 0.401942640543
            desired_marker.pose.orientation.x = 0.00183582917654
            desired_marker.pose.orientation.y = 0.99748586405
            desired_marker.pose.orientation.z = -0.0692585380106
            desired_marker.pose.orientation.w = 0.0148941486462
            corner_point.x = 1220.75146484
            corner_point.y = 860.069213867
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 1227.18505859
            corner_point.y = 1073.6887207
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 1019.14935303
            corner_point.y = 1076.23498535
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            corner_point = Point32()
            corner_point.x = 1016.63708496
            corner_point.y = 862.079467773
            corner_point.z = 0.0
            desired_marker.img_points.append(corner_point)
            desired_markers_pose.markers.append(desired_marker)



            pub_im.publish(image)
            pub_mp.publish(desired_markers_pose)
            rate.sleep()

        else:
            if num_ite == 0:
                cv2_img = cv2.imread('/home/ismael/catkin_ur10_ismael_ws/src/campero_ur10/desired_positions/IBVC/desired_image_p2.jpeg')
                image = bridge.cv2_to_imgmsg(cv2_img, "bgr8")

                desired_markers_pose = ArucoMarkerArray()
                desired_marker = ArucoMarker()
                corner_point = Point32()

                desired_marker.id = 10
                desired_marker.pose.position.x = 0.046693585813
                desired_marker.pose.position.y = 0.0568988919258
                desired_marker.pose.position.z = 0.403495699167
                desired_marker.pose.orientation.x = 0.00534729107636
                desired_marker.pose.orientation.y = 0.999728226191
                desired_marker.pose.orientation.z = -0.0220885499086
                desired_marker.pose.orientation.w = 0.0051938615423
                corner_point.x = 1221.33569336
                corner_point.y = 631.295349121
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 1225.36621094
                corner_point.y = 838.012207031
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 1018.74523926
                corner_point.y = 840.823303223
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 1016.00683594
                corner_point.y = 633.252868652
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                desired_markers_pose.markers.append(desired_marker)
                desired_marker = ArucoMarker()
                desired_marker.id = 17
                desired_marker.pose.position.x = -0.0235830619931
                desired_marker.pose.position.y = 0.124452121556
                desired_marker.pose.position.z = 0.403962761164
                desired_marker.pose.orientation.x = 0.00561065303403
                desired_marker.pose.orientation.y = 0.998407435329
                desired_marker.pose.orientation.z = -0.0556382986516
                desired_marker.pose.orientation.w = 0.00744938756578
                corner_point.x = 980.728942871
                corner_point.y = 860.710083008
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 983.053222656
                corner_point.y = 1072.17871094
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 775.99822998
                corner_point.y = 1075.71179199
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 776.01171875
                corner_point.y = 863.389587402
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                desired_markers_pose.markers.append(desired_marker)
                desired_marker = ArucoMarker()
                desired_marker.id = 26
                desired_marker.pose.position.x = -0.0239146091044
                desired_marker.pose.position.y = -0.00966113992035
                desired_marker.pose.position.z = 0.404481321573
                desired_marker.pose.orientation.x = 0.00418398668982
                desired_marker.pose.orientation.y = 0.999482945335
                desired_marker.pose.orientation.z = -0.029958029215
                desired_marker.pose.orientation.w = 0.0109019597413
                corner_point.x = 979.702331543
                corner_point.y = 404.245819092
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 981.752746582
                corner_point.y = 608.856262207
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 775.000061035
                corner_point.y = 610.963195801
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 774.999938965
                corner_point.y = 405.29675293
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                desired_markers_pose.markers.append(desired_marker)
                desired_marker = ArucoMarker()
                desired_marker.id = 91
                desired_marker.pose.position.x = 0.0468087084591
                desired_marker.pose.position.y = -0.00983435474336
                desired_marker.pose.position.z = 0.40441980958
                desired_marker.pose.orientation.x = 0.00408920974068
                desired_marker.pose.orientation.y = 0.999481177564
                desired_marker.pose.orientation.z = -0.029169450551
                desired_marker.pose.orientation.w = 0.0130306259626
                corner_point.x = 1220.71887207
                corner_point.y = 403.712463379
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 1224.82189941
                corner_point.y = 608.346130371
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 1018.94226074
                corner_point.y = 610.16015625
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 1016.61547852
                corner_point.y = 404.702453613
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                desired_markers_pose.markers.append(desired_marker)
                desired_marker = ArucoMarker()
                desired_marker.id = 200
                desired_marker.pose.position.x = -0.0237675141543
                desired_marker.pose.position.y = 0.0571288838983
                desired_marker.pose.position.z = 0.40377920866
                desired_marker.pose.orientation.x = 0.00526252310797
                desired_marker.pose.orientation.y = 0.999808639578
                desired_marker.pose.orientation.z = -0.0184994017305
                desired_marker.pose.orientation.w = 0.00357242384862
                corner_point.x = 980.278015137
                corner_point.y = 631.804260254
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 982.31652832
                corner_point.y = 838.706665039
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 776.088256836
                corner_point.y = 841.29107666
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 774.66998291
                corner_point.y = 633.965576172
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                desired_markers_pose.markers.append(desired_marker)
                desired_marker = ArucoMarker()
                desired_marker.id = 582
                desired_marker.pose.position.x = 0.0467164032161
                desired_marker.pose.position.y = 0.123768940568
                desired_marker.pose.position.z = 0.401942640543
                desired_marker.pose.orientation.x = 0.00183582917654
                desired_marker.pose.orientation.y = 0.99748586405
                desired_marker.pose.orientation.z = -0.0692585380106
                desired_marker.pose.orientation.w = 0.0148941486462
                corner_point.x = 1220.75146484
                corner_point.y = 860.069213867
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 1227.18505859
                corner_point.y = 1073.6887207
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 1019.14935303
                corner_point.y = 1076.23498535
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                corner_point = Point32()
                corner_point.x = 1016.63708496
                corner_point.y = 862.079467773
                corner_point.z = 0.0
                desired_marker.img_points.append(corner_point)
                desired_markers_pose.markers.append(desired_marker)
            else:
                if num_ite == 1:
                    cv2_img = cv2.imread('/home/ismael/catkin_ur10_ismael_ws/src/campero_ur10/desired_positions/IBVC/desired_image_p1.jpeg')
                    image = bridge.cv2_to_imgmsg(cv2_img, "bgr8")

                    desired_markers_pose = ArucoMarkerArray()
                    desired_marker = ArucoMarker()
                    corner_point = Point32()

                    desired_marker.id = 10
                    desired_marker.pose.position.x = 0.0464661344886
                    desired_marker.pose.position.y = 0.05880086869
                    desired_marker.pose.position.z = 0.685755729675
                    desired_marker.pose.orientation.x = 0.00337663567104
                    desired_marker.pose.orientation.y = 0.999809824824
                    desired_marker.pose.orientation.z = -0.0147463814366
                    desired_marker.pose.orientation.w = 0.0123067766144
                    corner_point.x = 1113.70202637
                    corner_point.y = 597.704101562
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 1115.18347168
                    corner_point.y = 719.06060791
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 993.556640625
                    corner_point.y = 720.341003418
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 993.067626953
                    corner_point.y = 598.727416992
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    desired_markers_pose.markers.append(desired_marker)
                    desired_marker = ArucoMarker()
                    desired_marker.id = 17
                    desired_marker.pose.position.x = -0.0244027804583
                    desired_marker.pose.position.y = 0.126588702202
                    desired_marker.pose.position.z = 0.686363279819
                    desired_marker.pose.orientation.x = 0.00418152914319
                    desired_marker.pose.orientation.y = 0.998317486043
                    desired_marker.pose.orientation.z = -0.0565197421893
                    desired_marker.pose.orientation.w = 0.0122568599844
                    corner_point.x = 970.676757812
                    corner_point.y = 733.804748535
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 971.897583008
                    corner_point.y = 856.721069336
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 850.00012207
                    corner_point.y = 858.210571289
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 850.0
                    corner_point.y = 735.285705566
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    desired_markers_pose.markers.append(desired_marker)
                    desired_marker = ArucoMarker()
                    desired_marker.id = 26
                    desired_marker.pose.position.x = -0.0242561418563
                    desired_marker.pose.position.y = -0.00801906455308
                    desired_marker.pose.position.z = 0.684716701508
                    desired_marker.pose.orientation.x = 0.00199905449751
                    desired_marker.pose.orientation.y = 0.998276730136
                    desired_marker.pose.orientation.z = 0.0496386626552
                    desired_marker.pose.orientation.w = -0.031234228337
                    corner_point.x = 970.818481445
                    corner_point.y = 462.999938965
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 972.301330566
                    corner_point.y = 584.0
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 851.28314209
                    corner_point.y = 583.999938965
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 849.795410156
                    corner_point.y = 462.99987793
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    desired_markers_pose.markers.append(desired_marker)
                    desired_marker = ArucoMarker()
                    desired_marker.id = 91
                    desired_marker.pose.position.x = 0.0466713011265
                    desired_marker.pose.position.y = -0.00823654234409
                    desired_marker.pose.position.z = 0.68653357029
                    desired_marker.pose.orientation.x = 0.00190922961573
                    desired_marker.pose.orientation.y = 0.99933812136
                    desired_marker.pose.orientation.z = -0.0315223322178
                    desired_marker.pose.orientation.w = 0.0180559300777
                    corner_point.x = 1113.75048828
                    corner_point.y = 462.877075195
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 1115.2520752
                    corner_point.y = 583.999938965
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 994.262023926
                    corner_point.y = 583.99987793
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 993.358276367
                    corner_point.y = 463.255462646
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    desired_markers_pose.markers.append(desired_marker)
                    desired_marker = ArucoMarker()
                    desired_marker.id = 200
                    desired_marker.pose.position.x = -0.0242616627365
                    desired_marker.pose.position.y = 0.0585340261459
                    desired_marker.pose.position.z = 0.685277879238
                    desired_marker.pose.orientation.x = 0.00304452508594
                    desired_marker.pose.orientation.y = 0.996441673089
                    desired_marker.pose.orientation.z = -0.0745027901079
                    desired_marker.pose.orientation.w = 0.0392944940884
                    corner_point.x = 970.670166016
                    corner_point.y = 597.890930176
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 972.165527344
                    corner_point.y = 718.84387207
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 850.000244141
                    corner_point.y = 720.313781738
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 849.999694824
                    corner_point.y = 598.226074219
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    desired_markers_pose.markers.append(desired_marker)
                    desired_marker = ArucoMarker()
                    desired_marker.id = 582
                    desired_marker.pose.position.x = 0.0463749468327
                    desired_marker.pose.position.y = 0.126793086529
                    desired_marker.pose.position.z = 0.686377346516
                    desired_marker.pose.orientation.x = 2.9161045957e-05
                    desired_marker.pose.orientation.y = 0.997871715506
                    desired_marker.pose.orientation.z = -0.065124510883
                    desired_marker.pose.orientation.w = 0.00329190285709
                    corner_point.x = 1113.27307129
                    corner_point.y = 734.999938965
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 1115.34606934
                    corner_point.y = 857.723693848
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 993.000183105
                    corner_point.y = 858.114318848
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    corner_point = Point32()
                    corner_point.x = 993.016113281
                    corner_point.y = 734.999938965
                    corner_point.z = 0.0
                    desired_marker.img_points.append(corner_point)
                    desired_markers_pose.markers.append(desired_marker)    



            pub_im.publish(image)
            pub_mp.publish(desired_markers_pose)
            rate.sleep()



if __name__ == '__main__':
    try:
        interaction = sys.argv[1]
        num_ite = 0
        print(num_ite)
        send_desired_i()
    except rospy.ROSInterruptException:
        pass

